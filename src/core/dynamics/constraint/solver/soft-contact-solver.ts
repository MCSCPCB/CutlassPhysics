// 软接触求解：带偏置求解、位置积分和无偏置松弛。
// 接触准备每步执行一次，子步复用力臂、有效质量和法线，并按当前位姿更新分离距离。
// 法线从刚体 2 指向刚体 1，分离距离 s = (p1 - p2)·n；s > 0 表示预测接触。
import { CONTACT_ENABLE_BOUNCE_THRESHOLD, LINEAR_SLOP, MAX_MANIFOLD_POINTS } from "../../../settings.js";
import type { ContactConstraint } from "../contact/contact-constraint.js";

/** 软约束参数：偏置比例、有效质量缩放和累积冲量衰减。 */
export class Softness {
  biasRate = 0;
  massScale = 1;
  impulseScale = 0;

  /** 频率 hertz（Hz）、阻尼比 zeta、步长 h（秒） */
  set(hertz: number, zeta: number, h: number): void {
    if (hertz === 0) {
      this.biasRate = 0;
      this.massScale = 1;
      this.impulseScale = 0;
      return;
    }
    const omega = 2 * Math.PI * hertz;
    const a1 = 2 * zeta + h * omega;
    const a2 = h * omega * a1;
    const a3 = 1 / (1 + a2);
    this.biasRate = omega / a1;
    this.massScale = a2 * a3;
    this.impulseScale = a3;
  }
}

/** 一个接触点的求解数据。力臂和有效质量在每步开始时算一次，这一步里不变 */
class SoftPoint {
  /** 力臂：接触点相对两个刚体原点的位置（世界方向） */
  r1X = 0; r1Y = 0; r1Z = 0;
  r2X = 0; r2Y = 0; r2Z = 0;
  /** 逆惯性张量 × (r × 方向)：施加单位冲量时的角速度变化。N 法向、T 切向、B 副切向 */
  iaN1X = 0; iaN1Y = 0; iaN1Z = 0; iaN2X = 0; iaN2Y = 0; iaN2Z = 0;
  iaT1X = 0; iaT1Y = 0; iaT1Z = 0; iaT2X = 0; iaT2Y = 0; iaT2Z = 0;
  iaB1X = 0; iaB1Y = 0; iaB1Z = 0; iaB2X = 0; iaB2Y = 0; iaB2Z = 0;
  /** 单位法向冲量引起的切向相对速度变化。 */
  couplingT = 0;
  couplingB = 0;
  massN = 0;
  massT = 0;
  massB = 0;
  /** 这一步开始时的分离距离 */
  separation = 0;
  /** 最近一次求解使用的分离距离；位姿未变化时复用 */
  currentSeparation = 0;
  /** 分离距离的修正量：用当前位姿重算的 (p1 - p2)·n 加上它，等于检测器给出的分离距离 */
  offset = 0;
  /** 这一步开始时的法向相对速度（反弹用） */
  relativeVelocity = 0;
  /** 累积冲量：法向、切向、副切向 */
  impulseN = 0;
  impulseT = 0;
  impulseB = 0;
  /** 本步的最大法向冲量；未施加法向冲量的点不反弹。 */
  maxImpulseN = 0;
  /** 是否满足撞击速度及非持续接触条件。 */
  bounce = false;
}

export class SoftContactSolver {
  readonly constraint: ContactConstraint;
  private readonly points: SoftPoint[] = [];
  private numPoints = 0;
  private friction = 0;
  private restitution = 0;
  /** 任一侧为静态、运动学或休眠刚体时，使用更硬的软约束参数。 */
  private fixed = false;
  /** 本步接触双方的逆质量。休眠刚体按静止对象求解，逆质量、逆惯量和速度视为 0。 */
  private invM1 = 0;
  private invM2 = 0;
  private live1 = true;
  private live2 = true;
  /** 子步模式是否按当前位姿重新计算分离距离。 */
  private substepped = false;
  /** 本 tick 接触到被撞刚体，参与额外迭代。 */
  impacted = false;

  constructor(constraint: ContactConstraint) {
    this.constraint = constraint;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.points.push(new SoftPoint());
  }

  /**
   * 准备力臂、有效质量、分离距离和热启动冲量，返回最大法向靠近速度。
   * impulseRatio 缩放热启动冲量；substepDt > 0 时改用本接触的历史步长比。
   * substepped 允许未施加过冲量的预测接触反弹，skin 为静态接触面的偏移。
   */
  prepare(impulseRatio = 1, substepped = false, skin = 0, substepDt = 0, bounceThreshold = CONTACT_ENABLE_BOUNCE_THRESHOLD): number {
    this.substepped = substepped;
    const c = this.constraint, m = c.manifold, b1 = c.b1!, b2 = c.b2!;
    if (substepDt > 0 && c.substepDt > 0) impulseRatio = substepDt / c.substepDt;
    // 撞击冲量不作为下一 tick 的持续支撑冲量。
    if (substepped && this.impacted) impulseRatio = 0;
    c.substepDt = substepDt;
    this.friction = Math.sqrt(c.s1!.friction * c.s2!.friction);
    if (b1.drag !== null || b2.drag !== null) this.friction *= 0.3;
    this.restitution = c.maxRestitution
      ? Math.max(c.s1!.restitution, c.s2!.restitution)
      : Math.sqrt(c.s1!.restitution * c.s2!.restitution);
    // skin 仅应用于静态或运动学接触，不应用于休眠动态刚体。
    const surface = b1.invMass === 0 || b2.invMass === 0 ? skin : 0;
    const live1 = !b1.sleeping, live2 = !b2.sleeping;
    const invM1 = live1 ? b1.invMass : 0, invM2 = live2 ? b2.invMass : 0;
    this.invM1 = invM1; this.invM2 = invM2;
    this.live1 = live1; this.live2 = live2;
    this.fixed = invM1 === 0 || invM2 === 0;
    const nX = m.nX, nY = m.nY, nZ = m.nZ, tX = m.tX, tY = m.tY, tZ = m.tZ, bX = m.bX, bY = m.bY, bZ = m.bZ;
    let i100 = 0, i101 = 0, i102 = 0, i110 = 0, i111 = 0, i112 = 0, i120 = 0, i121 = 0, i122 = 0;
    let i200 = 0, i201 = 0, i202 = 0, i210 = 0, i211 = 0, i212 = 0, i220 = 0, i221 = 0, i222 = 0;
    let v1X = 0, v1Y = 0, v1Z = 0, w1X = 0, w1Y = 0, w1Z = 0;
    let v2X = 0, v2Y = 0, v2Z = 0, w2X = 0, w2Y = 0, w2Z = 0;
    if (live1) {
      i100 = b1.ii00; i101 = b1.ii01; i102 = b1.ii02; i110 = b1.ii10; i111 = b1.ii11; i112 = b1.ii12; i120 = b1.ii20; i121 = b1.ii21; i122 = b1.ii22;
      v1X = b1.vx; v1Y = b1.vy; v1Z = b1.vz; w1X = b1.wx; w1Y = b1.wy; w1Z = b1.wz;
    }
    if (live2) {
      i200 = b2.ii00; i201 = b2.ii01; i202 = b2.ii02; i210 = b2.ii10; i211 = b2.ii11; i212 = b2.ii12; i220 = b2.ii20; i221 = b2.ii21; i222 = b2.ii22;
      v2X = b2.vx; v2Y = b2.vy; v2Z = b2.vz; w2X = b2.wx; w2Y = b2.wy; w2Z = b2.wz;
    }
    // 准备期间不改变刚体的位置和速度，所有接触点共用这些值。
    const p1X = b1.transform.px, p1Y = b1.transform.py, p1Z = b1.transform.pz;
    const p2X = b2.transform.px, p2Y = b2.transform.py, p2Z = b2.transform.pz;
    let approach = 0;
    this.numPoints = m.numPoints;
    for (let i = 0; i < m.numPoints; i++) {
      const p = m.points[i], sp = this.points[i];
      p.disabled = false;
      // 重置未参与热启动的接触冲量。
      if (!p.warmStarted) p.impulse.clear();
      // 力臂分别取两侧表面的接触点，与局部锚点保持一致。
      const r1X = p.p1X - p1X, r1Y = p.p1Y - p1Y, r1Z = p.p1Z - p1Z;
      const r2X = p.p2X - p2X, r2Y = p.p2Y - p2Y, r2Z = p.p2Z - p2Z;
      sp.r1X = r1X; sp.r1Y = r1Y; sp.r1Z = r1Z;
      sp.r2X = r2X; sp.r2Y = r2Y; sp.r2Z = r2Z;
      // 法向：a = r × n，ia = I⁻¹ a，有效质量 1 / (m1⁻¹ + m2⁻¹ + a1·ia1 + a2·ia2)
      let a1X = r1Y * nZ - r1Z * nY, a1Y = r1Z * nX - r1X * nZ, a1Z = r1X * nY - r1Y * nX;
      let a2X = r2Y * nZ - r2Z * nY, a2Y = r2Z * nX - r2X * nZ, a2Z = r2X * nY - r2Y * nX;
      sp.iaN1X = i100 * a1X + i101 * a1Y + i102 * a1Z; sp.iaN1Y = i110 * a1X + i111 * a1Y + i112 * a1Z; sp.iaN1Z = i120 * a1X + i121 * a1Y + i122 * a1Z;
      sp.iaN2X = i200 * a2X + i201 * a2Y + i202 * a2Z; sp.iaN2Y = i210 * a2X + i211 * a2Y + i212 * a2Z; sp.iaN2Z = i220 * a2X + i221 * a2Y + i222 * a2Z;
      let k = invM1 + invM2 + (a1X * sp.iaN1X + a1Y * sp.iaN1Y + a1Z * sp.iaN1Z) + (a2X * sp.iaN2X + a2Y * sp.iaN2Y + a2Z * sp.iaN2Z);
      sp.massN = k > 0 ? 1 / k : 0;
      // 切向
      a1X = r1Y * tZ - r1Z * tY; a1Y = r1Z * tX - r1X * tZ; a1Z = r1X * tY - r1Y * tX;
      a2X = r2Y * tZ - r2Z * tY; a2Y = r2Z * tX - r2X * tZ; a2Z = r2X * tY - r2Y * tX;
      sp.iaT1X = i100 * a1X + i101 * a1Y + i102 * a1Z; sp.iaT1Y = i110 * a1X + i111 * a1Y + i112 * a1Z; sp.iaT1Z = i120 * a1X + i121 * a1Y + i122 * a1Z;
      sp.iaT2X = i200 * a2X + i201 * a2Y + i202 * a2Z; sp.iaT2Y = i210 * a2X + i211 * a2Y + i212 * a2Z; sp.iaT2Z = i220 * a2X + i221 * a2Y + i222 * a2Z;
      k = invM1 + invM2 + (a1X * sp.iaT1X + a1Y * sp.iaT1Y + a1Z * sp.iaT1Z) + (a2X * sp.iaT2X + a2Y * sp.iaT2Y + a2Z * sp.iaT2Z);
      sp.massT = k > 0 ? 1 / k : 0;
      sp.couplingT = (invM1 + invM2) * (tX * nX + tY * nY + tZ * nZ) + a1X * sp.iaN1X + a1Y * sp.iaN1Y + a1Z * sp.iaN1Z + a2X * sp.iaN2X + a2Y * sp.iaN2Y + a2Z * sp.iaN2Z;
      // 副切向
      a1X = r1Y * bZ - r1Z * bY; a1Y = r1Z * bX - r1X * bZ; a1Z = r1X * bY - r1Y * bX;
      a2X = r2Y * bZ - r2Z * bY; a2Y = r2Z * bX - r2X * bZ; a2Z = r2X * bY - r2Y * bX;
      sp.iaB1X = i100 * a1X + i101 * a1Y + i102 * a1Z; sp.iaB1Y = i110 * a1X + i111 * a1Y + i112 * a1Z; sp.iaB1Z = i120 * a1X + i121 * a1Y + i122 * a1Z;
      sp.iaB2X = i200 * a2X + i201 * a2Y + i202 * a2Z; sp.iaB2Y = i210 * a2X + i211 * a2Y + i212 * a2Z; sp.iaB2Z = i220 * a2X + i221 * a2Y + i222 * a2Z;
      k = invM1 + invM2 + (a1X * sp.iaB1X + a1Y * sp.iaB1Y + a1Z * sp.iaB1Z) + (a2X * sp.iaB2X + a2Y * sp.iaB2Y + a2Z * sp.iaB2Z);
      sp.massB = k > 0 ? 1 / k : 0;
      sp.couplingB = (invM1 + invM2) * (bX * nX + bY * nY + bZ * nZ) + a1X * sp.iaN1X + a1Y * sp.iaN1Y + a1Z * sp.iaN1Z + a2X * sp.iaN2X + a2Y * sp.iaN2Y + a2Z * sp.iaN2Z;
      // 分离距离为 -depth - skin；offset 保持后续按位姿重算的结果一致。
      sp.separation = -p.depth - surface;
      sp.offset = sp.separation - ((p.p1X - p.p2X) * nX + (p.p1Y - p.p2Y) * nY + (p.p1Z - p.p2Z) * nZ);
      // 法向相对速度：(v1 + w1 × r1 - v2 - w2 × r2)·n
      const dvX = v1X + (w1Y * r1Z - w1Z * r1Y) - v2X - (w2Y * r2Z - w2Z * r2Y);
      const dvY = v1Y + (w1Z * r1X - w1X * r1Z) - v2Y - (w2Z * r2X - w2X * r2Z);
      const dvZ = v1Z + (w1X * r1Y - w1Y * r1X) - v2Z - (w2X * r2Y - w2Y * r2X);
      const vn = dvX * nX + dvY * nY + dvZ * nZ;
      sp.relativeVelocity = vn;
      if (-vn > approach) approach = -vn;
      const imp = p.impulse;
      // 多个子步时：靠近速度达到阈值就反弹，持续接触靠落定后速度小排除。单步时持续接触的点不反弹
      sp.bounce = substepped ? vn <= -bounceThreshold : vn < -bounceThreshold && !p.warmStarted;
      // 上一步的累积冲量：摩擦存的是世界坐标的向量，投影到这一步的切向上
      sp.impulseN = imp.impulseN * impulseRatio;
      sp.impulseT = (imp.impulseLX * tX + imp.impulseLY * tY + imp.impulseLZ * tZ) * impulseRatio;
      sp.impulseB = (imp.impulseLX * bX + imp.impulseLY * bY + imp.impulseLZ * bZ) * impulseRatio;
      sp.maxImpulseN = 0;
    }
    c.approachSpeed = approach;
    return approach;
  }

  /** 热启动：把累积冲量施加上去。每一步都要做 */
  warmStart(): void {
    const c = this.constraint, m = c.manifold, b1 = c.b1!, b2 = c.b2!;
    const invM1 = this.invM1, invM2 = this.invM2, live1 = this.live1, live2 = this.live2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ, tX = m.tX, tY = m.tY, tZ = m.tZ, bX = m.bX, bY = m.bY, bZ = m.bZ;
    let v1X = 0, v1Y = 0, v1Z = 0, w1X = 0, w1Y = 0, w1Z = 0;
    let v2X = 0, v2Y = 0, v2Z = 0, w2X = 0, w2Y = 0, w2Z = 0;
    if (live1) { v1X = b1.vx; v1Y = b1.vy; v1Z = b1.vz; w1X = b1.wx; w1Y = b1.wy; w1Z = b1.wz; }
    if (live2) { v2X = b2.vx; v2Y = b2.vy; v2Z = b2.vz; w2X = b2.wx; w2Y = b2.wy; w2Z = b2.wz; }
    for (let i = 0; i < this.numPoints; i++) {
      const sp = this.points[i];
      const iN = sp.impulseN, iT = sp.impulseT, iB = sp.impulseB;
      if (iN === 0 && iT === 0 && iB === 0) continue;
      const pX = nX * iN + tX * iT + bX * iB, pY = nY * iN + tY * iT + bY * iB, pZ = nZ * iN + tZ * iT + bZ * iB;
      v1X += invM1 * pX; v1Y += invM1 * pY; v1Z += invM1 * pZ;
      v2X -= invM2 * pX; v2Y -= invM2 * pY; v2Z -= invM2 * pZ;
      w1X += sp.iaN1X * iN + sp.iaT1X * iT + sp.iaB1X * iB;
      w1Y += sp.iaN1Y * iN + sp.iaT1Y * iT + sp.iaB1Y * iB;
      w1Z += sp.iaN1Z * iN + sp.iaT1Z * iT + sp.iaB1Z * iB;
      w2X -= sp.iaN2X * iN + sp.iaT2X * iT + sp.iaB2X * iB;
      w2Y -= sp.iaN2Y * iN + sp.iaT2Y * iT + sp.iaB2Y * iB;
      w2Z -= sp.iaN2Z * iN + sp.iaT2Z * iT + sp.iaB2Z * iB;
    }
    if (live1) { b1.vx = v1X; b1.vy = v1Y; b1.vz = v1Z; b1.wx = w1X; b1.wy = w1Y; b1.wz = w1Z; }
    if (live2) { b2.vx = v2X; b2.vy = v2Y; b2.vz = v2Z; b2.wx = w2X; b2.wy = w2Y; b2.wz = w2Z; }
  }

  /**
   * 先求法向支撑，再求摩擦；返回冲量变化的绝对值之和。
   * useBias 启用穿透修正；关闭时仅阻止继续靠近。
   * reuseSeparation 仅在位姿未变时有效，reverse 控制接触点迭代方向。
   */
  solve(useBias: boolean, reverse: boolean, invH: number, dynamic: Softness, fixed: Softness, maxPushVelocity: number, reuseSeparation = false): number {
    const c = this.constraint, m = c.manifold, b1 = c.b1!, b2 = c.b2!;
    const invM1 = this.invM1, invM2 = this.invM2, live1 = this.live1, live2 = this.live2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ, tX = m.tX, tY = m.tY, tZ = m.tZ, bX = m.bX, bY = m.bY, bZ = m.bZ;
    const soft = this.fixed ? fixed : dynamic;
    const biasRate = soft.biasRate, softMass = soft.massScale, softImpulse = soft.impulseScale;
    const n = this.numPoints;
    // 分离距离只依赖位姿，可以在速度求解前一起准备；复用时不读取变换矩阵。
    if (!reuseSeparation) {
      if (useBias && !this.substepped) {
        for (let i = 0; i < n; i++) this.points[i].currentSeparation = this.points[i].separation;
      } else {
        const tf1 = b1.transform, tf2 = b2.transform;
        const t1X = tf1.px, t1Y = tf1.py, t1Z = tf1.pz, t2X = tf2.px, t2Y = tf2.py, t2Z = tf2.pz;
        const a00 = tf1.r00, a01 = tf1.r01, a02 = tf1.r02, a10 = tf1.r10, a11 = tf1.r11, a12 = tf1.r12, a20 = tf1.r20, a21 = tf1.r21, a22 = tf1.r22;
        const c00 = tf2.r00, c01 = tf2.r01, c02 = tf2.r02, c10 = tf2.r10, c11 = tf2.r11, c12 = tf2.r12, c20 = tf2.r20, c21 = tf2.r21, c22 = tf2.r22;
        // 用当前位姿重算：p = 刚体位置 + 旋转 × 局部坐标
        for (let i = 0; i < n; i++) {
          const p = m.points[i];
          const l1X = p.lp1X, l1Y = p.lp1Y, l1Z = p.lp1Z, l2X = p.lp2X, l2Y = p.lp2Y, l2Z = p.lp2Z;
          const p1X = t1X + a00 * l1X + a01 * l1Y + a02 * l1Z;
          const p1Y = t1Y + a10 * l1X + a11 * l1Y + a12 * l1Z;
          const p1Z = t1Z + a20 * l1X + a21 * l1Y + a22 * l1Z;
          const p2X = t2X + c00 * l2X + c01 * l2Y + c02 * l2Z;
          const p2Y = t2Y + c10 * l2X + c11 * l2Y + c12 * l2Z;
          const p2Z = t2Z + c20 * l2X + c21 * l2Y + c22 * l2Z;
          this.points[i].currentSeparation = (p1X - p2X) * nX + (p1Y - p2Y) * nY + (p1Z - p2Z) * nZ + this.points[i].offset;
        }
      }
    }
    let v1X = 0, v1Y = 0, v1Z = 0, w1X = 0, w1Y = 0, w1Z = 0;
    let v2X = 0, v2Y = 0, v2Z = 0, w2X = 0, w2Y = 0, w2Z = 0;
    if (live1) { v1X = b1.vx; v1Y = b1.vy; v1Z = b1.vz; w1X = b1.wx; w1Y = b1.wy; w1Z = b1.wz; }
    if (live2) { v2X = b2.vx; v2Y = b2.vy; v2Z = b2.vz; w2X = b2.wx; w2Y = b2.wy; w2Z = b2.wz; }
    const friction = this.friction;
    let error = 0;
    for (let k = 0; k < n; k++) {
      const sp = this.points[reverse ? n - 1 - k : k];
      const r1X = sp.r1X, r1Y = sp.r1Y, r1Z = sp.r1Z, r2X = sp.r2X, r2Y = sp.r2Y, r2Z = sp.r2Z;
      const s = sp.currentSeparation;
      let bias = 0, massScale = 1, impulseScale = 0;
      if (s > 0) {
        // 预测接触将本步靠近量限制在剩余间距内。
        bias = s * invH;
      } else if (useBias && s < -(this.substepped ? 0 : LINEAR_SLOP)) {
        bias = biasRate * (s + (this.substepped ? 0 : LINEAR_SLOP));
        if (bias < -maxPushVelocity) bias = -maxPushVelocity;
        massScale = softMass;
        impulseScale = softImpulse;
      }
      const dvX = v1X + (w1Y * r1Z - w1Z * r1Y) - v2X - (w2Y * r2Z - w2Z * r2Y);
      const dvY = v1Y + (w1Z * r1X - w1X * r1Z) - v2Y - (w2Z * r2X - w2X * r2Z);
      const dvZ = v1Z + (w1X * r1Y - w1Y * r1X) - v2Z - (w2X * r2Y - w2Y * r2X);
      const vn = dvX * nX + dvY * nY + dvZ * nZ;
      const old = sp.impulseN;
      let impulse = -sp.massN * massScale * (vn + bias) - impulseScale * old;
      const sum = old + impulse;
      const total = sum > 0 ? sum : 0;
      impulse = total - old;
      sp.impulseN = total;
      // 预测接触可能在后续子步才受力，撞击速度包含该子步的加速。
      if (this.substepped && impulse > 0 && -vn > c.approachSpeed) c.approachSpeed = -vn;
      if (total > sp.maxImpulseN) sp.maxImpulseN = total;
      let iT = 0, iB = 0;
      // 法向与切向耦合项避免重复计算接触点速度。
      if (total !== 0 || sp.impulseT !== 0 || sp.impulseB !== 0) {
        let newT = sp.impulseT - sp.massT * (dvX * tX + dvY * tY + dvZ * tZ + sp.couplingT * impulse);
        let newB = sp.impulseB - sp.massB * (dvX * bX + dvY * bY + dvZ * bZ + sp.couplingB * impulse);
        const limit = friction * total, lenSq = newT * newT + newB * newB;
        if (lenSq > limit * limit) {
          const scale = limit / Math.sqrt(lenSq);
          newT *= scale;
          newB *= scale;
        }
        iT = newT - sp.impulseT;
        iB = newB - sp.impulseB;
        sp.impulseT = newT;
        sp.impulseB = newB;
      }
      error += Math.abs(impulse) + Math.abs(iT) + Math.abs(iB);
      if (impulse === 0 && iT === 0 && iB === 0) continue;
      const pX = nX * impulse + tX * iT + bX * iB;
      const pY = nY * impulse + tY * iT + bY * iB;
      const pZ = nZ * impulse + tZ * iT + bZ * iB;
      v1X += invM1 * pX; v1Y += invM1 * pY; v1Z += invM1 * pZ;
      v2X -= invM2 * pX; v2Y -= invM2 * pY; v2Z -= invM2 * pZ;
      w1X += sp.iaN1X * impulse + sp.iaT1X * iT + sp.iaB1X * iB;
      w1Y += sp.iaN1Y * impulse + sp.iaT1Y * iT + sp.iaB1Y * iB;
      w1Z += sp.iaN1Z * impulse + sp.iaT1Z * iT + sp.iaB1Z * iB;
      w2X -= sp.iaN2X * impulse + sp.iaT2X * iT + sp.iaB2X * iB;
      w2Y -= sp.iaN2Y * impulse + sp.iaT2Y * iT + sp.iaB2Y * iB;
      w2Z -= sp.iaN2Z * impulse + sp.iaT2Z * iT + sp.iaB2Z * iB;
    }
    if (live1) { b1.vx = v1X; b1.vy = v1Y; b1.vz = v1Z; b1.wx = w1X; b1.wy = w1Y; b1.wz = w1Z; }
    if (live2) { b2.vx = v2X; b2.vy = v2Y; b2.vz = v2Z; b2.wx = w2X; b2.wy = w2Y; b2.wz = w2Z; }
    return error;
  }

  /** 有没有要反弹的点（applyRestitution 会处理的点） */
  wantsRestitution(): boolean {
    if (this.restitution === 0) return false;
    for (let i = 0; i < this.numPoints; i++) {
      const sp = this.points[i];
      if (sp.bounce && sp.maxImpulseN !== 0) return true;
    }
    return false;
  }

  /** 反弹：求解完之后，让法向速度变成撞击时的 -restitution 倍。只处理真的推过的点 */
  applyRestitution(): void {
    if (this.restitution === 0) return;
    const c = this.constraint, m = c.manifold, b1 = c.b1!, b2 = c.b2!;
    const invM1 = this.invM1, invM2 = this.invM2, live1 = this.live1, live2 = this.live2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ;
    const e = this.restitution;
    let v1X = 0, v1Y = 0, v1Z = 0, w1X = 0, w1Y = 0, w1Z = 0;
    let v2X = 0, v2Y = 0, v2Z = 0, w2X = 0, w2Y = 0, w2Z = 0;
    if (live1) { v1X = b1.vx; v1Y = b1.vy; v1Z = b1.vz; w1X = b1.wx; w1Y = b1.wy; w1Z = b1.wz; }
    if (live2) { v2X = b2.vx; v2Y = b2.vy; v2Z = b2.vz; w2X = b2.wx; w2Y = b2.wy; w2Z = b2.wz; }
    for (let i = 0; i < this.numPoints; i++) {
      const sp = this.points[i];
      // 本步未施加法向冲量的预测接触点不反弹。
      if (!sp.bounce || sp.maxImpulseN === 0) continue;
      const r1X = sp.r1X, r1Y = sp.r1Y, r1Z = sp.r1Z, r2X = sp.r2X, r2Y = sp.r2Y, r2Z = sp.r2Z;
      const dvX = v1X + (w1Y * r1Z - w1Z * r1Y) - v2X - (w2Y * r2Z - w2Z * r2Y);
      const dvY = v1Y + (w1Z * r1X - w1X * r1Z) - v2Y - (w2Z * r2X - w2X * r2Z);
      const dvZ = v1Z + (w1X * r1Y - w1Y * r1X) - v2Z - (w2X * r2Y - w2Y * r2X);
      const vn = dvX * nX + dvY * nY + dvZ * nZ;
      let impulse = -sp.massN * (vn + e * sp.relativeVelocity);
      const total = sp.impulseN + impulse > 0 ? sp.impulseN + impulse : 0;
      impulse = total - sp.impulseN;
      sp.impulseN = total;
      if (impulse === 0) continue;
      v1X += invM1 * nX * impulse; v1Y += invM1 * nY * impulse; v1Z += invM1 * nZ * impulse;
      v2X -= invM2 * nX * impulse; v2Y -= invM2 * nY * impulse; v2Z -= invM2 * nZ * impulse;
      w1X += sp.iaN1X * impulse; w1Y += sp.iaN1Y * impulse; w1Z += sp.iaN1Z * impulse;
      w2X -= sp.iaN2X * impulse; w2Y -= sp.iaN2Y * impulse; w2Z -= sp.iaN2Z * impulse;
    }
    if (live1) { b1.vx = v1X; b1.vy = v1Y; b1.vz = v1Z; b1.wx = w1X; b1.wy = w1Y; b1.wz = w1Z; }
    if (live2) { b2.vx = v2X; b2.vy = v2Y; b2.vz = v2Z; b2.wx = w2X; b2.wy = w2Y; b2.wz = w2Z; }
  }

  /**
   * 保存累积冲量并同步接触点。
   * stamp 记录步进编号和法向冲量状态，供子步模式在求解后判断接触。
   */
  store(stamp = -1): void {
    const c = this.constraint, m = c.manifold;
    const tX = m.tX, tY = m.tY, tZ = m.tZ, bX = m.bX, bY = m.bY, bZ = m.bZ;
    let pushed = false;
    for (let i = 0; i < this.numPoints; i++) {
      const sp = this.points[i], imp = m.points[i].impulse;
      imp.impulseN = sp.impulseN;
      imp.impulseT = sp.impulseT;
      imp.impulseB = sp.impulseB;
      imp.impulseLX = tX * sp.impulseT + bX * sp.impulseB;
      imp.impulseLY = tY * sp.impulseT + bY * sp.impulseB;
      imp.impulseLZ = tZ * sp.impulseT + bZ * sp.impulseB;
      if (sp.maxImpulseN > 0) pushed = true;
    }
    if (stamp >= 0) {
      c.solvedStamp = stamp;
      c.pushed = pushed;
    }
    c.syncManifold();
  }
}
