// 移植自 OimoPhysics src/oimo/dynamics/constraint/solver/pgs/PgsContactConstraintSolver.hx
// 和 solver/common/ContactSolverMassDataRow.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
// 投影高斯-赛德尔接触求解，使用展开的标量运算。
// 累加从 0 开始，保留与上游一致的浮点零符号。
import { MAX_MANIFOLD_POINTS, POSITION_SPLIT_IMPULSE_BAUMGARTE } from "../../../settings.js";
import type { TimeStep } from "../../time-step.js";
import { ConstraintSolver } from "../constraint-solver.js";
import type { ContactConstraint } from "../contact/contact-constraint.js";
import { ContactSolverInfo } from "../contact/contact-solver-info.js";

/** 每个接触点的质量数据：雅可比乘逆质量、逆惯性张量，以及有效质量 */
class MassDataRow {
  // 法向（N）、切向（T）、副切向（B），1、2 是两个刚体
  invMLinN1X = 0; invMLinN1Y = 0; invMLinN1Z = 0;
  invMLinN2X = 0; invMLinN2Y = 0; invMLinN2Z = 0;
  invMAngN1X = 0; invMAngN1Y = 0; invMAngN1Z = 0;
  invMAngN2X = 0; invMAngN2Y = 0; invMAngN2Z = 0;
  invMLinT1X = 0; invMLinT1Y = 0; invMLinT1Z = 0;
  invMLinT2X = 0; invMLinT2Y = 0; invMLinT2Z = 0;
  invMAngT1X = 0; invMAngT1Y = 0; invMAngT1Z = 0;
  invMAngT2X = 0; invMAngT2Y = 0; invMAngT2Z = 0;
  invMLinB1X = 0; invMLinB1Y = 0; invMLinB1Z = 0;
  invMLinB2X = 0; invMLinB2Y = 0; invMLinB2Z = 0;
  invMAngB1X = 0; invMAngB1Y = 0; invMAngB1Z = 0;
  invMAngB2X = 0; invMAngB2Y = 0; invMAngB2Z = 0;
  /** 法向有效质量 */
  massN = 0;
  /** 切向、副切向的 2×2 有效质量矩阵 */
  massTB00 = 0; massTB01 = 0; massTB10 = 0; massTB11 = 0;
}

export class PgsContactConstraintSolver extends ConstraintSolver {
  private readonly constraint: ContactConstraint;
  private readonly info = new ContactSolverInfo();
  private readonly massData: MassDataRow[] = [];

  constructor(constraint: ContactConstraint) {
    super();
    this.constraint = constraint;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.massData.push(new MassDataRow());
  }

  /** 速度求解前：整理接触数据，算每个接触点的有效质量 */
  preSolveVelocity(timeStep: TimeStep): void {
    const info = this.info;
    this.constraint.getVelocitySolverInfo(timeStep, info);
    const b1 = (this.b1 = info.b1!), b2 = (this.b2 = info.b2!);
    const invM1 = b1.invMass, invM2 = b2.invMass;
    const i100 = b1.ii00, i101 = b1.ii01, i102 = b1.ii02, i110 = b1.ii10, i111 = b1.ii11, i112 = b1.ii12, i120 = b1.ii20, i121 = b1.ii21, i122 = b1.ii22;
    const i200 = b2.ii00, i201 = b2.ii01, i202 = b2.ii02, i210 = b2.ii10, i211 = b2.ii11, i212 = b2.ii12, i220 = b2.ii20, i221 = b2.ii21, i222 = b2.ii22;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i];
      // 法向有效质量
      const j = row.jacobianN;
      md.invMLinN1X = j.lin1X * invM1; md.invMLinN1Y = j.lin1Y * invM1; md.invMLinN1Z = j.lin1Z * invM1;
      md.invMLinN2X = j.lin2X * invM2; md.invMLinN2Y = j.lin2Y * invM2; md.invMLinN2Z = j.lin2Z * invM2;
      md.invMAngN1X = i100 * j.ang1X + i101 * j.ang1Y + i102 * j.ang1Z;
      md.invMAngN1Y = i110 * j.ang1X + i111 * j.ang1Y + i112 * j.ang1Z;
      md.invMAngN1Z = i120 * j.ang1X + i121 * j.ang1Y + i122 * j.ang1Z;
      md.invMAngN2X = i200 * j.ang2X + i201 * j.ang2Y + i202 * j.ang2Z;
      md.invMAngN2Y = i210 * j.ang2X + i211 * j.ang2Y + i212 * j.ang2Z;
      md.invMAngN2Z = i220 * j.ang2X + i221 * j.ang2Y + i222 * j.ang2Z;
      md.massN = invM1 + invM2 + (md.invMAngN1X * j.ang1X + md.invMAngN1Y * j.ang1Y + md.invMAngN1Z * j.ang1Z) +
        (md.invMAngN2X * j.ang2X + md.invMAngN2Y * j.ang2Y + md.invMAngN2Z * j.ang2Z);
      if (md.massN !== 0) md.massN = 1 / md.massN;
      // 切向、副切向（摩擦）的有效质量矩阵
      const jt = row.jacobianT, jb = row.jacobianB;
      md.invMLinT1X = jt.lin1X * invM1; md.invMLinT1Y = jt.lin1Y * invM1; md.invMLinT1Z = jt.lin1Z * invM1;
      md.invMLinT2X = jt.lin2X * invM2; md.invMLinT2Y = jt.lin2Y * invM2; md.invMLinT2Z = jt.lin2Z * invM2;
      md.invMLinB1X = jb.lin1X * invM1; md.invMLinB1Y = jb.lin1Y * invM1; md.invMLinB1Z = jb.lin1Z * invM1;
      md.invMLinB2X = jb.lin2X * invM2; md.invMLinB2Y = jb.lin2Y * invM2; md.invMLinB2Z = jb.lin2Z * invM2;
      md.invMAngT1X = i100 * jt.ang1X + i101 * jt.ang1Y + i102 * jt.ang1Z;
      md.invMAngT1Y = i110 * jt.ang1X + i111 * jt.ang1Y + i112 * jt.ang1Z;
      md.invMAngT1Z = i120 * jt.ang1X + i121 * jt.ang1Y + i122 * jt.ang1Z;
      md.invMAngT2X = i200 * jt.ang2X + i201 * jt.ang2Y + i202 * jt.ang2Z;
      md.invMAngT2Y = i210 * jt.ang2X + i211 * jt.ang2Y + i212 * jt.ang2Z;
      md.invMAngT2Z = i220 * jt.ang2X + i221 * jt.ang2Y + i222 * jt.ang2Z;
      md.invMAngB1X = i100 * jb.ang1X + i101 * jb.ang1Y + i102 * jb.ang1Z;
      md.invMAngB1Y = i110 * jb.ang1X + i111 * jb.ang1Y + i112 * jb.ang1Z;
      md.invMAngB1Z = i120 * jb.ang1X + i121 * jb.ang1Y + i122 * jb.ang1Z;
      md.invMAngB2X = i200 * jb.ang2X + i201 * jb.ang2Y + i202 * jb.ang2Z;
      md.invMAngB2Y = i210 * jb.ang2X + i211 * jb.ang2Y + i212 * jb.ang2Z;
      md.invMAngB2Z = i220 * jb.ang2X + i221 * jb.ang2Y + i222 * jb.ang2Z;
      const invMassTB00 = invM1 + invM2 + (md.invMAngT1X * jt.ang1X + md.invMAngT1Y * jt.ang1Y + md.invMAngT1Z * jt.ang1Z) +
        (md.invMAngT2X * jt.ang2X + md.invMAngT2Y * jt.ang2Y + md.invMAngT2Z * jt.ang2Z);
      const invMassTB01 = md.invMAngT1X * jb.ang1X + md.invMAngT1Y * jb.ang1Y + md.invMAngT1Z * jb.ang1Z +
        (md.invMAngT2X * jb.ang2X + md.invMAngT2Y * jb.ang2Y + md.invMAngT2Z * jb.ang2Z);
      const invMassTB11 = invM1 + invM2 + (md.invMAngB1X * jb.ang1X + md.invMAngB1Y * jb.ang1Y + md.invMAngB1Z * jb.ang1Z) +
        (md.invMAngB2X * jb.ang2X + md.invMAngB2Y * jb.ang2Y + md.invMAngB2Z * jb.ang2Z);
      // 2×2 求逆（矩阵对称，invMassTB10 = invMassTB01）
      let invDet = invMassTB00 * invMassTB11 - invMassTB01 * invMassTB01;
      if (invDet !== 0) invDet = 1 / invDet;
      md.massTB00 = invMassTB11 * invDet;
      md.massTB01 = -invMassTB01 * invDet;
      md.massTB10 = -invMassTB01 * invDet;
      md.massTB11 = invMassTB00 * invDet;
    }
  }

  /** 热启动：把上一步的累积冲量先施加上去 */
  warmStart(timeStep: TimeStep): void {
    const info = this.info, b1 = this.b1!, b2 = this.b2!;
    let lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz, lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    let av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz, av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse!, md = this.massData[i];
      const jt = row.jacobianT, jb = row.jacobianB;
      const impulseN = imp.impulseN;
      // 切向每步都重建，把上一步的摩擦冲量向量投影到新的切向上
      const impulseT = imp.impulseLX * jt.lin1X + imp.impulseLY * jt.lin1Y + imp.impulseLZ * jt.lin1Z;
      const impulseB = imp.impulseLX * jb.lin1X + imp.impulseLY * jb.lin1Y + imp.impulseLZ * jb.lin1Z;
      imp.impulseT = impulseT;
      imp.impulseB = impulseB;
      // 时间步长变化时按比例缩放累积冲量（原版如此：施加的是缩放前的值）
      imp.impulseN *= timeStep.dtRatio;
      imp.impulseT *= timeStep.dtRatio;
      imp.impulseB *= timeStep.dtRatio;
      lv1X += md.invMLinN1X * impulseN; lv1Y += md.invMLinN1Y * impulseN; lv1Z += md.invMLinN1Z * impulseN;
      lv1X += md.invMLinT1X * impulseT; lv1Y += md.invMLinT1Y * impulseT; lv1Z += md.invMLinT1Z * impulseT;
      lv1X += md.invMLinB1X * impulseB; lv1Y += md.invMLinB1Y * impulseB; lv1Z += md.invMLinB1Z * impulseB;
      lv2X += md.invMLinN2X * -impulseN; lv2Y += md.invMLinN2Y * -impulseN; lv2Z += md.invMLinN2Z * -impulseN;
      lv2X += md.invMLinT2X * -impulseT; lv2Y += md.invMLinT2Y * -impulseT; lv2Z += md.invMLinT2Z * -impulseT;
      lv2X += md.invMLinB2X * -impulseB; lv2Y += md.invMLinB2Y * -impulseB; lv2Z += md.invMLinB2Z * -impulseB;
      av1X += md.invMAngN1X * impulseN; av1Y += md.invMAngN1Y * impulseN; av1Z += md.invMAngN1Z * impulseN;
      av1X += md.invMAngT1X * impulseT; av1Y += md.invMAngT1Y * impulseT; av1Z += md.invMAngT1Z * impulseT;
      av1X += md.invMAngB1X * impulseB; av1Y += md.invMAngB1Y * impulseB; av1Z += md.invMAngB1Z * impulseB;
      av2X += md.invMAngN2X * -impulseN; av2Y += md.invMAngN2Y * -impulseN; av2Z += md.invMAngN2Z * -impulseN;
      av2X += md.invMAngT2X * -impulseT; av2Y += md.invMAngT2Y * -impulseT; av2Z += md.invMAngT2Z * -impulseT;
      av2X += md.invMAngB2X * -impulseB; av2Y += md.invMAngB2Y * -impulseB; av2Z += md.invMAngB2Z * -impulseB;
    }
    b1.vx = lv1X; b1.vy = lv1Y; b1.vz = lv1Z;
    b2.vx = lv2X; b2.vy = lv2Y; b2.vz = lv2Z;
    b1.wx = av1X; b1.wy = av1Y; b1.wz = av1Z;
    b2.wx = av2X; b2.wy = av2Y; b2.wz = av2Z;
  }

  /** 一次速度迭代：先解摩擦，再解法向 */
  solveVelocity(): void {
    const info = this.info, b1 = this.b1!, b2 = this.b2!;
    let lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz, lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    let av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz, av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    // 摩擦：切向、副切向一起解，再把合力限制在摩擦锥里
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse!;
      let j = row.jacobianT;
      let rvt = 0;
      rvt += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rvt -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rvt += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rvt -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      j = row.jacobianB;
      let rvb = 0;
      rvb += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rvb -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rvb += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rvb -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseT = -(rvt * md.massTB00 + rvb * md.massTB01);
      let impulseB = -(rvt * md.massTB10 + rvb * md.massTB11);
      const oldImpulseT = imp.impulseT, oldImpulseB = imp.impulseB;
      imp.impulseT += impulseT;
      imp.impulseB += impulseB;
      // 圆锥摩擦：合摩擦冲量不超过 摩擦系数 × 法向冲量
      const maxImpulse = row.friction * imp.impulseN;
      if (maxImpulse === 0) {
        imp.impulseT = 0;
        imp.impulseB = 0;
      } else {
        const lenSq = imp.impulseT * imp.impulseT + imp.impulseB * imp.impulseB;
        if (lenSq > maxImpulse * maxImpulse) {
          const invL = maxImpulse / Math.sqrt(lenSq);
          imp.impulseT *= invL;
          imp.impulseB *= invL;
        }
      }
      impulseT = imp.impulseT - oldImpulseT;
      impulseB = imp.impulseB - oldImpulseB;
      lv1X += md.invMLinT1X * impulseT; lv1Y += md.invMLinT1Y * impulseT; lv1Z += md.invMLinT1Z * impulseT;
      lv1X += md.invMLinB1X * impulseB; lv1Y += md.invMLinB1Y * impulseB; lv1Z += md.invMLinB1Z * impulseB;
      lv2X += md.invMLinT2X * -impulseT; lv2Y += md.invMLinT2Y * -impulseT; lv2Z += md.invMLinT2Z * -impulseT;
      lv2X += md.invMLinB2X * -impulseB; lv2Y += md.invMLinB2Y * -impulseB; lv2Z += md.invMLinB2Z * -impulseB;
      av1X += md.invMAngT1X * impulseT; av1Y += md.invMAngT1Y * impulseT; av1Z += md.invMAngT1Z * impulseT;
      av1X += md.invMAngB1X * impulseB; av1Y += md.invMAngB1Y * impulseB; av1Z += md.invMAngB1Z * impulseB;
      av2X += md.invMAngT2X * -impulseT; av2Y += md.invMAngT2Y * -impulseT; av2Z += md.invMAngT2Z * -impulseT;
      av2X += md.invMAngB2X * -impulseB; av2Y += md.invMAngB2Y * -impulseB; av2Z += md.invMAngB2Z * -impulseB;
    }
    // 法向：累积冲量不能为负（只能推不能拉）
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse!, j = row.jacobianN;
      let rvn = 0;
      rvn += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rvn -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rvn += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rvn -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseN = (row.rhs - rvn) * md.massN;
      const oldImpulseN = imp.impulseN;
      imp.impulseN += impulseN;
      if (imp.impulseN < 0) imp.impulseN = 0;
      impulseN = imp.impulseN - oldImpulseN;
      lv1X += md.invMLinN1X * impulseN; lv1Y += md.invMLinN1Y * impulseN; lv1Z += md.invMLinN1Z * impulseN;
      lv2X += md.invMLinN2X * -impulseN; lv2Y += md.invMLinN2Y * -impulseN; lv2Z += md.invMLinN2Z * -impulseN;
      av1X += md.invMAngN1X * impulseN; av1Y += md.invMAngN1Y * impulseN; av1Z += md.invMAngN1Z * impulseN;
      av2X += md.invMAngN2X * -impulseN; av2Y += md.invMAngN2Y * -impulseN; av2Z += md.invMAngN2Z * -impulseN;
    }
    b1.vx = lv1X; b1.vy = lv1Y; b1.vz = lv1Z;
    b2.vx = lv2X; b2.vy = lv2Y; b2.vz = lv2Z;
    b1.wx = av1X; b1.wy = av1Y; b1.wz = av1Z;
    b2.wx = av2X; b2.wy = av2Y; b2.wz = av2Z;
  }

  /** 位置修正前（原版 preSolvePosition + updatePositionData）：刚体已经积分过，重新同步接触点、算法向有效质量 */
  preSolvePosition(): void {
    const c = this.constraint, info = this.info;
    c.syncManifold();
    c.getPositionSolverInfo(info);
    const b1 = this.b1!, b2 = this.b2!;
    const invM1 = b1.invMass, invM2 = b2.invMass;
    const i100 = b1.ii00, i101 = b1.ii01, i102 = b1.ii02, i110 = b1.ii10, i111 = b1.ii11, i112 = b1.ii12, i120 = b1.ii20, i121 = b1.ii21, i122 = b1.ii22;
    const i200 = b2.ii00, i201 = b2.ii01, i202 = b2.ii02, i210 = b2.ii10, i211 = b2.ii11, i212 = b2.ii12, i220 = b2.ii20, i221 = b2.ii21, i222 = b2.ii22;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], j = row.jacobianN;
      md.invMLinN1X = j.lin1X * invM1; md.invMLinN1Y = j.lin1Y * invM1; md.invMLinN1Z = j.lin1Z * invM1;
      md.invMLinN2X = j.lin2X * invM2; md.invMLinN2Y = j.lin2Y * invM2; md.invMLinN2Z = j.lin2Z * invM2;
      md.invMAngN1X = i100 * j.ang1X + i101 * j.ang1Y + i102 * j.ang1Z;
      md.invMAngN1Y = i110 * j.ang1X + i111 * j.ang1Y + i112 * j.ang1Z;
      md.invMAngN1Z = i120 * j.ang1X + i121 * j.ang1Y + i122 * j.ang1Z;
      md.invMAngN2X = i200 * j.ang2X + i201 * j.ang2Y + i202 * j.ang2Z;
      md.invMAngN2Y = i210 * j.ang2X + i211 * j.ang2Y + i212 * j.ang2Z;
      md.invMAngN2Z = i220 * j.ang2X + i221 * j.ang2Y + i222 * j.ang2Z;
      md.massN = invM1 + invM2 + (md.invMAngN1X * j.ang1X + md.invMAngN1Y * j.ang1Y + md.invMAngN1Z * j.ang1Z) +
        (md.invMAngN2X * j.ang2X + md.invMAngN2Y * j.ang2Y + md.invMAngN2Z * j.ang2Z);
      if (md.massN !== 0) md.massN = 1 / md.massN;
      // 清掉位置冲量
      row.impulse!.impulseP = 0;
    }
  }

  /** 一次 split impulse 位置迭代：修改伪速度，把穿透推开，不影响真实速度 */
  solvePositionSplitImpulse(): void {
    const info = this.info, b1 = this.b1!, b2 = this.b2!;
    let lv1X = b1.pvx, lv1Y = b1.pvy, lv1Z = b1.pvz, lv2X = b2.pvx, lv2Y = b2.pvy, lv2Z = b2.pvz;
    let av1X = b1.pwx, av1Y = b1.pwy, av1Z = b1.pwz, av2X = b2.pwx, av2Y = b2.pwy, av2Z = b2.pwz;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse!, j = row.jacobianN;
      let rvn = 0;
      rvn += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rvn -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rvn += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rvn -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseP = (row.rhs - rvn) * md.massN * POSITION_SPLIT_IMPULSE_BAUMGARTE;
      const oldImpulseP = imp.impulseP;
      imp.impulseP += impulseP;
      if (imp.impulseP < 0) imp.impulseP = 0;
      impulseP = imp.impulseP - oldImpulseP;
      lv1X += md.invMLinN1X * impulseP; lv1Y += md.invMLinN1Y * impulseP; lv1Z += md.invMLinN1Z * impulseP;
      lv2X += md.invMLinN2X * -impulseP; lv2Y += md.invMLinN2Y * -impulseP; lv2Z += md.invMLinN2Z * -impulseP;
      av1X += md.invMAngN1X * impulseP; av1Y += md.invMAngN1Y * impulseP; av1Z += md.invMAngN1Z * impulseP;
      av2X += md.invMAngN2X * -impulseP; av2Y += md.invMAngN2Y * -impulseP; av2Z += md.invMAngN2Z * -impulseP;
    }
    b1.pvx = lv1X; b1.pvy = lv1Y; b1.pvz = lv1Z;
    b2.pvx = lv2X; b2.pvy = lv2Y; b2.pvz = lv2Z;
    b1.pwx = av1X; b1.pwy = av1Y; b1.pwz = av1Z;
    b2.pwx = av2X; b2.pwy = av2Y; b2.pwz = av2Z;
  }

  /** 求解结束：记下摩擦冲量向量（下一步热启动用），把接触冲量累加到刚体上 */
  postSolve(): void {
    const info = this.info, b1 = this.b1!, b2 = this.b2!;
    let lin1X = 0, lin1Y = 0, lin1Z = 0;
    let ang1X = 0, ang1Y = 0, ang1Z = 0;
    let ang2X = 0, ang2Y = 0, ang2Z = 0;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse!;
      const jn = row.jacobianN, jt = row.jacobianT, jb = row.jacobianB;
      const impN = imp.impulseN, impT = imp.impulseT, impB = imp.impulseB;
      let lX = 0, lY = 0, lZ = 0;
      lX += jt.lin1X * impT; lY += jt.lin1Y * impT; lZ += jt.lin1Z * impT;
      lX += jb.lin1X * impB; lY += jb.lin1Y * impB; lZ += jb.lin1Z * impB;
      imp.impulseLX = lX; imp.impulseLY = lY; imp.impulseLZ = lZ;
      lin1X += jn.lin1X * impN; lin1Y += jn.lin1Y * impN; lin1Z += jn.lin1Z * impN;
      ang1X += jn.ang1X * impN; ang1Y += jn.ang1Y * impN; ang1Z += jn.ang1Z * impN;
      ang2X += jn.ang2X * impN; ang2Y += jn.ang2Y * impN; ang2Z += jn.ang2Z * impN;
      lin1X += jt.lin1X * impT; lin1Y += jt.lin1Y * impT; lin1Z += jt.lin1Z * impT;
      ang1X += jt.ang1X * impT; ang1Y += jt.ang1Y * impT; ang1Z += jt.ang1Z * impT;
      ang2X += jt.ang2X * impT; ang2Y += jt.ang2Y * impT; ang2Z += jt.ang2Z * impT;
      lin1X += jb.lin1X * impB; lin1Y += jb.lin1Y * impB; lin1Z += jb.lin1Z * impB;
      ang1X += jb.ang1X * impB; ang1Y += jb.ang1Y * impB; ang1Z += jb.ang1Z * impB;
      ang2X += jb.ang2X * impB; ang2Y += jb.ang2Y * impB; ang2Z += jb.ang2Z * impB;
    }
    b1.lciX += lin1X; b1.lciY += lin1Y; b1.lciZ += lin1Z;
    b1.aciX += ang1X; b1.aciY += ang1Y; b1.aciZ += ang1Z;
    b2.lciX -= lin1X; b2.lciY -= lin1Y; b2.lciZ -= lin1Z;
    b2.aciX -= ang2X; b2.aciY -= ang2Y; b2.aciZ -= ang2Z;
    this.constraint.syncManifold();
  }

  /** 此阶段无额外接触处理。 */
  postSolveVelocity(): void {}

  /** 接触支持 Baumgarte 和 split impulse，此 NGS 接口为空实现。 */
  solvePositionNgs(): void {}
}
