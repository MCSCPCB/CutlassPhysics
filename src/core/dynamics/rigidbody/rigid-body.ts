// 移植自 OimoPhysics src/oimo/dynamics/rigidbody/RigidBody.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { Transform } from "../../common/transform.js";
import { compat } from "../../compat.js";
import { MAX_ROTATION_PER_STEP, MAX_TRANSLATION_PER_STEP } from "../../settings.js";
import { RigidBodyType, type RigidBodyConfig } from "./rigid-body-config.js";
import type { JointLink } from "../constraint/joint/joint-link.js";
import type { ContactLink } from "../contact.js";
import type { World } from "../world.js";
import type { Shape } from "./shape.js";

/** 刚体。矩阵和向量都存成标量字段：mIJ 是第 I 行第 J 列 */
export class RigidBody {
  prev: RigidBody | null = null;
  next: RigidBody | null = null;
  world: World | null = null;

  shapeList: Shape | null = null;
  shapeListLast: Shape | null = null;
  numShapes = 0;

  /** 接触链表：通过它找到和这个刚体有关的接触 */
  contactLinkList: ContactLink | null = null;
  contactLinkListLast: ContactLink | null = null;
  numContactLinks = 0;
  /** 关节链表：通过它找到和这个刚体有关的关节 */
  jointLinkList: JointLink | null = null;
  jointLinkListLast: JointLink | null = null;
  numJointLinks = 0;

  /** 线速度、角速度 */
  vx: number; vy: number; vz: number;
  wx: number; wy: number; wz: number;
  /** 位置修正用的伪速度，一步之后清零 */
  pvx = 0; pvy = 0; pvz = 0;
  pwx = 0; pwy = 0; pwz = 0;

  /** 上一步、这一步重心的世界变换 */
  readonly ptransform = new Transform();
  /** 最近一次达到撞击速度阈值的步进编号。 */
  impactStamp = -1;
  /** 速度变化后预测接触距离失效。下一步碰撞检测前按新速度重新同步。 */
  speculativeStale = false;
  /** 本 tick 是否跳过休眠计时，默认为 false。流体不平衡时由适配层设置。 */
  restless = false;
  readonly transform = new Transform();

  type: number;
  sleepTime = 0;
  sleeping = false;
  /** 是否锁定自动唤醒，用于等待地形时的冻结。 */
  wakeLocked = false;
  autoSleep: boolean;
  sleepingVelocityThreshold: number;
  sleepingAngularVelocityThreshold: number;
  sleepingTimeThreshold: number;

  mass = 0;
  invMass = 0;
  /** 局部惯性张量 */
  li00 = 0; li01 = 0; li02 = 0; li10 = 0; li11 = 0; li12 = 0; li20 = 0; li21 = 0; li22 = 0;
  /** 每根轴是否允许转动（1 允许，0 锁住） */
  rotFactorX = 1; rotFactorY = 1; rotFactorZ = 1;
  /** 局部逆惯性张量（已乘 rotFactor） */
  ili00 = 0; ili01 = 0; ili02 = 0; ili10 = 0; ili11 = 0; ili12 = 0; ili20 = 0; ili21 = 0; ili22 = 0;
  /** 局部逆惯性张量（未乘 rotFactor） */
  iliw00 = 0; iliw01 = 0; iliw02 = 0; iliw10 = 0; iliw11 = 0; iliw12 = 0; iliw20 = 0; iliw21 = 0; iliw22 = 0;
  /** 世界坐标下的逆惯性张量 */
  ii00 = 0; ii01 = 0; ii02 = 0; ii10 = 0; ii11 = 0; ii12 = 0; ii20 = 0; ii21 = 0; ii22 = 0;

  linearDamping: number;
  angularDamping: number;
  /** 这一步累积的力和力矩，一步之后清零 */
  fx = 0; fy = 0; fz = 0;
  tx = 0; ty = 0; tz = 0;
  /** 这一步接触产生的冲量 */
  lciX = 0; lciY = 0; lciZ = 0;
  aciX = 0; aciY = 0; aciZ = 0;

  addedToIsland = false;
  gravityScale = 1;
  userData: unknown = null;

  constructor(config: RigidBodyConfig) {
    this.vx = config.vx; this.vy = config.vy; this.vz = config.vz;
    this.wx = config.wx; this.wy = config.wy; this.wz = config.wz;
    this.ptransform.copyFrom(config.transform);
    this.transform.copyFrom(this.ptransform);
    this.type = config.type;
    this.autoSleep = config.autoSleep;
    this.sleepingVelocityThreshold = config.sleepingVelocityThreshold;
    this.sleepingAngularVelocityThreshold = config.sleepingAngularVelocityThreshold;
    this.sleepingTimeThreshold = config.sleepingTimeThreshold;
    this.linearDamping = config.linearDamping;
    this.angularDamping = config.angularDamping;
  }

  /** 按速度积分一步（原版 _integrate）。静态刚体只把速度清零 */
  integrate(dt: number): void {
    if (this.type === RigidBodyType.STATIC) {
      this.vx = 0; this.vy = 0; this.vz = 0;
      this.wx = 0; this.wy = 0; this.wz = 0;
      this.pvx = 0; this.pvy = 0; this.pvz = 0;
      this.pwx = 0; this.pwy = 0; this.pwz = 0;
      return;
    }
    let tX = this.vx * dt, tY = this.vy * dt, tZ = this.vz * dt;
    let rX = this.wx * dt, rY = this.wy * dt, rZ = this.wz * dt;
    const tLenSq = tX * tX + tY * tY + tZ * tZ;
    const rLenSq = rX * rX + rY * rY + rZ * rZ;
    if (tLenSq === 0 && rLenSq === 0) return;
    // 每步的位移和转角有上限，超过时按比例压低速度
    if (tLenSq > MAX_TRANSLATION_PER_STEP * MAX_TRANSLATION_PER_STEP) {
      const l = MAX_TRANSLATION_PER_STEP / Math.sqrt(tLenSq);
      this.vx *= l; this.vy *= l; this.vz *= l;
      tX *= l; tY *= l; tZ *= l;
    }
    if (rLenSq > MAX_ROTATION_PER_STEP * MAX_ROTATION_PER_STEP) {
      const l = MAX_ROTATION_PER_STEP / Math.sqrt(rLenSq);
      this.wx *= l; this.wy *= l; this.wz *= l;
      rX *= l; rY *= l; rZ *= l;
    }
    this.transform.px += tX;
    this.transform.py += tY;
    this.transform.pz += tZ;
    this.applyRotation(rX, rY, rZ);
  }

  /** 按位置修正得到的伪速度移动刚体（原版 _integratePseudoVelocity），然后清零伪速度 */
  integratePseudoVelocity(): void {
    const pLenSq = this.pvx * this.pvx + this.pvy * this.pvy + this.pvz * this.pvz;
    const aLenSq = this.pwx * this.pwx + this.pwy * this.pwy + this.pwz * this.pwz;
    if (pLenSq === 0 && aLenSq === 0) return;
    const tX = this.pvx, tY = this.pvy, tZ = this.pvz;
    const rX = this.pwx, rY = this.pwy, rZ = this.pwz;
    this.pvx = 0; this.pvy = 0; this.pvz = 0;
    this.pwx = 0; this.pwy = 0; this.pwz = 0;
    if (this.type === RigidBodyType.STATIC) return;
    this.transform.px += tX;
    this.transform.py += tY;
    this.transform.pz += tZ;
    this.applyRotation(rX, rY, rZ);
  }

  /** 绕旋转向量 (rX, rY, rZ) 转动（原版 _applyRotation），然后更新世界逆惯性张量 */
  applyRotation(rX: number, rY: number, rZ: number): void {
    // 旋转向量 → 四元数增量 dq。小角度用麦克劳林展开
    const theta = Math.sqrt(rX * rX + rY * rY + rZ * rZ);
    const halfTheta = theta * 0.5;
    let factor: number, cosHalfTheta: number;
    if (halfTheta < 0.5) {
      const ht2 = halfTheta * halfTheta;
      factor = (1 / 2) * (1 - ht2 * (1 / 6) + ht2 * ht2 * (1 / 120));
      cosHalfTheta = 1 - ht2 * (1 / 2) + ht2 * ht2 * (1 / 24);
    } else {
      factor = Math.sin(halfTheta) / theta;
      cosHalfTheta = Math.cos(halfTheta);
    }
    const dqX = rX * factor, dqY = rY * factor, dqZ = rZ * factor, dqW = cosHalfTheta;

    // 当前旋转矩阵 → 四元数 q
    const tf = this.transform;
    const e00 = tf.r00, e11 = tf.r11, e22 = tf.r22;
    const t = e00 + e11 + e22;
    let s: number, qX: number, qY: number, qZ: number, qW: number;
    if (t > 0) {
      s = Math.sqrt(t + 1);
      qW = 0.5 * s;
      s = 0.5 / s;
      qX = (tf.r21 - tf.r12) * s; qY = (tf.r02 - tf.r20) * s; qZ = (tf.r10 - tf.r01) * s;
    } else if (e00 > e11) {
      if (e00 > e22) {
        s = Math.sqrt(e00 - e11 - e22 + 1);
        qX = 0.5 * s;
        s = 0.5 / s;
        qY = (tf.r01 + tf.r10) * s; qZ = (tf.r02 + tf.r20) * s; qW = (tf.r21 - tf.r12) * s;
      } else {
        s = Math.sqrt(e22 - e00 - e11 + 1);
        qZ = 0.5 * s;
        s = 0.5 / s;
        qX = (tf.r02 + tf.r20) * s; qY = (tf.r12 + tf.r21) * s; qW = (tf.r10 - tf.r01) * s;
      }
    } else if (e11 > e22) {
      s = Math.sqrt(e11 - e22 - e00 + 1);
      qY = 0.5 * s;
      s = 0.5 / s;
      qX = (tf.r01 + tf.r10) * s; qZ = (tf.r12 + tf.r21) * s; qW = (tf.r02 - tf.r20) * s;
    } else {
      s = Math.sqrt(e22 - e00 - e11 + 1);
      qZ = 0.5 * s;
      s = 0.5 / s;
      qX = (tf.r02 + tf.r20) * s; qY = (tf.r12 + tf.r21) * s; qW = (tf.r10 - tf.r01) * s;
    }
    // q = dq · q
    if (compat.keepOimoBugs) {
      // 原版 quat_mul 宏没有用临时变量，后面的分量会读到已经更新过的 qX、qY、qZ，转角会被略微放大
      qX = dqW * qX + dqX * qW + dqY * qZ - dqZ * qY;
      qY = dqW * qY - dqX * qZ + dqY * qW + dqZ * qX;
      qZ = dqW * qZ + dqX * qY - dqY * qX + dqZ * qW;
      qW = dqW * qW - dqX * qX - dqY * qY - dqZ * qZ;
    } else {
      const nX = dqW * qX + dqX * qW + dqY * qZ - dqZ * qY;
      const nY = dqW * qY - dqX * qZ + dqY * qW + dqZ * qX;
      const nZ = dqW * qZ + dqX * qY - dqY * qX + dqZ * qW;
      const nW = dqW * qW - dqX * qX - dqY * qY - dqZ * qZ;
      qX = nX; qY = nY; qZ = nZ; qW = nW;
    }
    let l = qX * qX + qY * qY + qZ * qZ + qW * qW;
    if (l > 1e-32) l = 1 / Math.sqrt(l);
    qX *= l; qY *= l; qZ *= l; qW *= l;

    // 四元数 → 旋转矩阵
    const x2 = 2 * qX, y2 = 2 * qY, z2 = 2 * qZ;
    const xx = qX * x2, yy = qY * y2, zz = qZ * z2;
    const xy = qX * y2, yz = qY * z2, xz = qX * z2;
    const qwx = qW * x2, qwy = qW * y2, qwz = qW * z2;
    tf.r00 = 1 - yy - zz; tf.r01 = xy - qwz; tf.r02 = xz + qwy;
    tf.r10 = xy + qwz; tf.r11 = 1 - xx - zz; tf.r12 = yz - qwx;
    tf.r20 = xz - qwy; tf.r21 = yz + qwx; tf.r22 = 1 - xx - yy;
    this.updateInvInertia();
  }

  /** 根据形状重算质量和惯性张量（原版 updateMass），然后唤醒刚体 */
  updateMass(): void {
    let totalMass = 0;
    let i00 = 0, i01 = 0, i02 = 0, i10 = 0, i11 = 0, i12 = 0, i20 = 0, i21 = 0, i22 = 0;
    for (let s = this.shapeList; s !== null; s = s.next) {
      const g = s.geom;
      g.updateMass();
      const mass = s.density * g.volume;
      // 转到刚体坐标系：I = R · (I/m) · Rᵀ · m
      const r = s.localTransform;
      const a00 = r.r00 * g.ic00 + r.r01 * g.ic10 + r.r02 * g.ic20;
      const a01 = r.r00 * g.ic01 + r.r01 * g.ic11 + r.r02 * g.ic21;
      const a02 = r.r00 * g.ic02 + r.r01 * g.ic12 + r.r02 * g.ic22;
      const a10 = r.r10 * g.ic00 + r.r11 * g.ic10 + r.r12 * g.ic20;
      const a11 = r.r10 * g.ic01 + r.r11 * g.ic11 + r.r12 * g.ic21;
      const a12 = r.r10 * g.ic02 + r.r11 * g.ic12 + r.r12 * g.ic22;
      const a20 = r.r20 * g.ic00 + r.r21 * g.ic10 + r.r22 * g.ic20;
      const a21 = r.r20 * g.ic01 + r.r21 * g.ic11 + r.r22 * g.ic21;
      const a22 = r.r20 * g.ic02 + r.r21 * g.ic12 + r.r22 * g.ic22;
      let j00 = a00 * r.r00 + a01 * r.r01 + a02 * r.r02;
      let j01 = a00 * r.r10 + a01 * r.r11 + a02 * r.r12;
      let j02 = a00 * r.r20 + a01 * r.r21 + a02 * r.r22;
      let j10 = a10 * r.r00 + a11 * r.r01 + a12 * r.r02;
      let j11 = a10 * r.r10 + a11 * r.r11 + a12 * r.r12;
      let j12 = a10 * r.r20 + a11 * r.r21 + a12 * r.r22;
      let j20 = a20 * r.r00 + a21 * r.r01 + a22 * r.r02;
      let j21 = a20 * r.r10 + a21 * r.r11 + a22 * r.r12;
      let j22 = a20 * r.r20 + a21 * r.r21 + a22 * r.r22;
      j00 *= mass; j01 *= mass; j02 *= mass;
      j10 *= mass; j11 *= mass; j12 *= mass;
      j20 *= mass; j21 *= mass; j22 *= mass;
      // 平行轴定理：加上形状重心偏移的贡献
      const px = r.px, py = r.py, pz = r.pz;
      const xx = px * px, yy = py * py, zz = pz * pz;
      const xy = -px * py, yz = -py * pz, zx = -pz * px;
      j00 += (yy + zz) * mass; j01 += xy * mass; j02 += zx * mass;
      j10 += xy * mass; j11 += (xx + zz) * mass; j12 += yz * mass;
      j20 += zx * mass; j21 += yz * mass; j22 += (xx + yy) * mass;
      totalMass += mass;
      i00 += j00; i01 += j01; i02 += j02;
      i10 += j10; i11 += j11; i12 += j12;
      i20 += j20; i21 += j21; i22 += j22;
    }
    this.mass = totalMass;
    this.li00 = i00; this.li01 = i01; this.li02 = i02;
    this.li10 = i10; this.li11 = i11; this.li12 = i12;
    this.li20 = i20; this.li21 = i21; this.li22 = i22;
    this.completeMassData();
    this.wakeUp();
  }

  /** 由 mass 和局部惯性张量算出逆质量、逆惯性张量（原版 completeMassData） */
  private completeMassData(): void {
    const e00 = this.li00, e01 = this.li01, e02 = this.li02;
    const e10 = this.li10, e11 = this.li11, e12 = this.li12;
    const e20 = this.li20, e21 = this.li21, e22 = this.li22;
    // 余子式和行列式（原版 mat3_det、mat3_inv）
    const d00 = e11 * e22 - e12 * e21;
    const d01 = e10 * e22 - e12 * e20;
    const d02 = e10 * e21 - e11 * e20;
    const det = e00 * d00 - e01 * d01 + e02 * d02;
    if (this.mass > 0 && det > 0 && this.type === RigidBodyType.DYNAMIC) {
      this.invMass = 1 / this.mass;
      const d10 = e01 * e22 - e02 * e21;
      const d11 = e00 * e22 - e02 * e20;
      const d12 = e00 * e21 - e01 * e20;
      const d20 = e01 * e12 - e02 * e11;
      const d21 = e00 * e12 - e02 * e10;
      const d22 = e00 * e11 - e01 * e10;
      let inv = e00 * d00 - e01 * d01 + e02 * d02;
      if (inv < -1e-32 || inv > 1e-32) inv = 1 / inv;
      this.iliw00 = d00 * inv; this.iliw01 = -d10 * inv; this.iliw02 = d20 * inv;
      this.iliw10 = -d01 * inv; this.iliw11 = d11 * inv; this.iliw12 = -d21 * inv;
      this.iliw20 = d02 * inv; this.iliw21 = -d12 * inv; this.iliw22 = d22 * inv;
    } else {
      this.invMass = 0;
      this.iliw00 = 0; this.iliw01 = 0; this.iliw02 = 0;
      this.iliw10 = 0; this.iliw11 = 0; this.iliw12 = 0;
      this.iliw20 = 0; this.iliw21 = 0; this.iliw22 = 0;
      if (this.type === RigidBodyType.DYNAMIC) {
        // 动态刚体质量的下限，防止逆质量溢出。
        this.invMass = 1e-9;
        this.iliw00 = 1e-9; this.iliw11 = 1e-9; this.iliw22 = 1e-9;
      }
    }
    // 局部逆惯性张量的三行分别乘 rotFactor
    const x = this.rotFactorX, y = this.rotFactorY, z = this.rotFactorZ;
    this.ili00 = this.iliw00 * x; this.ili01 = this.iliw01 * x; this.ili02 = this.iliw02 * x;
    this.ili10 = this.iliw10 * y; this.ili11 = this.iliw11 * y; this.ili12 = this.iliw12 * y;
    this.ili20 = this.iliw20 * z; this.ili21 = this.iliw21 * z; this.ili22 = this.iliw22 * z;
    this.updateInvInertia();
  }

  /** 世界逆惯性张量 = R · ili · Rᵀ，再按行乘 rotFactor（原版 updateInvInertia） */
  updateInvInertia(): void {
    const r = this.transform;
    const a00 = r.r00 * this.ili00 + r.r01 * this.ili10 + r.r02 * this.ili20;
    const a01 = r.r00 * this.ili01 + r.r01 * this.ili11 + r.r02 * this.ili21;
    const a02 = r.r00 * this.ili02 + r.r01 * this.ili12 + r.r02 * this.ili22;
    const a10 = r.r10 * this.ili00 + r.r11 * this.ili10 + r.r12 * this.ili20;
    const a11 = r.r10 * this.ili01 + r.r11 * this.ili11 + r.r12 * this.ili21;
    const a12 = r.r10 * this.ili02 + r.r11 * this.ili12 + r.r12 * this.ili22;
    const a20 = r.r20 * this.ili00 + r.r21 * this.ili10 + r.r22 * this.ili20;
    const a21 = r.r20 * this.ili01 + r.r21 * this.ili11 + r.r22 * this.ili21;
    const a22 = r.r20 * this.ili02 + r.r21 * this.ili12 + r.r22 * this.ili22;
    const x = this.rotFactorX, y = this.rotFactorY, z = this.rotFactorZ;
    this.ii00 = (a00 * r.r00 + a01 * r.r01 + a02 * r.r02) * x;
    this.ii01 = (a00 * r.r10 + a01 * r.r11 + a02 * r.r12) * x;
    this.ii02 = (a00 * r.r20 + a01 * r.r21 + a02 * r.r22) * x;
    this.ii10 = (a10 * r.r00 + a11 * r.r01 + a12 * r.r02) * y;
    this.ii11 = (a10 * r.r10 + a11 * r.r11 + a12 * r.r12) * y;
    this.ii12 = (a10 * r.r20 + a11 * r.r21 + a12 * r.r22) * y;
    this.ii20 = (a20 * r.r00 + a21 * r.r01 + a22 * r.r02) * z;
    this.ii21 = (a20 * r.r10 + a21 * r.r11 + a22 * r.r12) * z;
    this.ii22 = (a20 * r.r20 + a21 * r.r21 + a22 * r.r22) * z;
  }

  /** 外部直接改了变换后调用（原版 updateTransformExt） */
  updateTransformExt(): void {
    this.ptransform.copyFrom(this.transform);
    this.syncShapes();
    this.wakeUp();
  }

  /** 形状增删或改动后调用：重算质量并同步形状 */
  shapeModified(): void {
    this.updateMass();
    this.syncShapes();
  }

  syncShapes(): void {
    // 子步模式下同步预测距离，覆盖新增、瞬移和形状变化。
    if (this.type !== RigidBodyType.STATIC && this.world !== null) {
      const t = this.world.speculativeTick();
      if (t > 0) {
        const w = this.world;
        this.syncShapesSpeculative(t, w.speculativeDistance, w.maxSpeculativeDistance);
        return;
      }
    }
    for (let s = this.shapeList; s !== null; s = s.next) s.sync(this.ptransform, this.transform);
  }

  /** 按当前速度同步形状，并将包围盒扩展到预测接触范围。t 为下一 tick 的时长。 */
  syncShapesSpeculative(t: number, base: number, max: number): void {
    this.speculativeStale = false;
    const speed = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
    const angularSpeed = Math.sqrt(this.wx * this.wx + this.wy * this.wy + this.wz * this.wz);
    const linearReach = speed * t;
    for (let s = this.shapeList; s !== null; s = s.next) s.syncSpeculative(this.ptransform, this.transform, base, max, linearReach, angularSpeed, t);
  }

  /** 位置修正：沿位置冲量平移（原版 _applyLinearPositionImpulse） */
  applyLinearPositionImpulse(ix: number, iy: number, iz: number): void {
    this.transform.px += ix * this.invMass;
    this.transform.py += iy * this.invMass;
    this.transform.pz += iz * this.invMass;
  }

  /** 位置修正：按角位置冲量转动（原版 _applyAngularPositionImpulse） */
  applyAngularPositionImpulse(ix: number, iy: number, iz: number): void {
    this.applyRotation(
      this.ii00 * ix + this.ii01 * iy + this.ii02 * iz,
      this.ii10 * ix + this.ii11 * iy + this.ii12 * iz,
      this.ii20 * ix + this.ii21 * iy + this.ii22 * iz,
    );
  }

  /** 速度低于休眠阈值（原版 _isSleepy） */
  isSleepy(): boolean {
    return (
      this.autoSleep &&
      this.vx * this.vx + this.vy * this.vy + this.vz * this.vz < this.sleepingVelocityThreshold * this.sleepingVelocityThreshold &&
      this.wx * this.wx + this.wy * this.wy + this.wz * this.wz <
        this.sleepingAngularVelocityThreshold * this.sleepingAngularVelocityThreshold
    );
  }

  /** 平移（原版 _applyTranslation，NGS 位置修正用） */
  applyTranslation(tX: number, tY: number, tZ: number): void {
    this.transform.px += tX;
    this.transform.py += tY;
    this.transform.pz += tZ;
  }

  /** 没有任何接触和关节（原版 _isAlone） */
  isAlone(): boolean {
    return this.numContactLinks === 0 && this.numJointLinks === 0;
  }

  /** 添加形状。刚体已经在世界里时，形状也立即加入世界 */
  addShape(shape: Shape): void {
    if (this.shapeList === null) {
      this.shapeList = shape;
      this.shapeListLast = shape;
    } else {
      this.shapeListLast!.next = shape;
      shape.prev = this.shapeListLast;
      this.shapeListLast = shape;
    }
    this.numShapes++;
    shape.rigidBody = this;
    if (this.world !== null) this.world.addShape(shape);
    this.shapeModified();
  }

  /** 批量添加形状，完成后统一重算质量。 */
  addShapes(shapes: readonly Shape[]): void {
    for (const shape of shapes) {
      if (this.shapeList === null) {
        this.shapeList = shape;
      } else {
        this.shapeListLast!.next = shape;
        shape.prev = this.shapeListLast;
      }
      this.shapeListLast = shape;
      this.numShapes++;
      shape.rigidBody = this;
      if (this.world !== null) this.world.addShape(shape);
    }
    this.shapeModified();
  }
  removeShape(shape: Shape): void {
    this.unlinkShape(shape);
    this.shapeModified();
  }
  /** 批量移除形状，完成后统一重算质量。 */
  removeShapes(shapes: readonly Shape[]): void {
    for (const shape of shapes) this.unlinkShape(shape);
    this.shapeModified();
  }
  private unlinkShape(shape: Shape): void {
    // 先从世界里移除（删除代理和相关接触），再从链表里移除
    if (this.world !== null) this.world.removeShape(shape);
    const prev = shape.prev, next = shape.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (shape === this.shapeList) this.shapeList = next;
    if (shape === this.shapeListLast) this.shapeListLast = prev;
    shape.next = null;
    shape.prev = null;
    this.numShapes--;
    shape.rigidBody = null;
  }

  /** 在世界坐标 (px, py, pz) 处施加冲量：改变线速度和角速度，并唤醒刚体（原版 applyImpulse） */
  applyImpulseAt(ix: number, iy: number, iz: number, px: number, py: number, pz: number): void {
    this.vx += ix * this.invMass;
    this.vy += iy * this.invMass;
    this.vz += iz * this.invMass;
    px -= this.transform.px;
    py -= this.transform.py;
    pz -= this.transform.pz;
    const ax = py * iz - pz * iy;
    const ay = pz * ix - px * iz;
    const az = px * iy - py * ix;
    this.wx += this.ii00 * ax + this.ii01 * ay + this.ii02 * az;
    this.wy += this.ii10 * ax + this.ii11 * ay + this.ii12 * az;
    this.wz += this.ii20 * ax + this.ii21 * ay + this.ii22 * az;
    this.sleeping = false;
    this.sleepTime = 0;
    this.speculativeStale = true;
  }

  /** 在重心施加冲量，不改变角速度（原版 applyLinearImpulse） */
  applyLinearImpulse(ix: number, iy: number, iz: number): void {
    this.vx += ix * this.invMass;
    this.vy += iy * this.invMass;
    this.vz += iz * this.invMass;
    this.sleeping = false;
    this.sleepTime = 0;
    this.speculativeStale = true;
  }

  /** 施加角冲量，不改变线速度（原版 applyAngularImpulse） */
  applyAngularImpulse(ix: number, iy: number, iz: number): void {
    this.wx += this.ii00 * ix + this.ii01 * iy + this.ii02 * iz;
    this.wy += this.ii10 * ix + this.ii11 * iy + this.ii12 * iz;
    this.wz += this.ii20 * ix + this.ii21 * iy + this.ii22 * iz;
    this.sleeping = false;
    this.sleepTime = 0;
    this.speculativeStale = true;
  }

  /** 在世界坐标 (px, py, pz) 处施力，累加到下一步，每步结束后清零（原版 applyForce） */
  applyForceAt(fx: number, fy: number, fz: number, px: number, py: number, pz: number): void {
    this.fx += fx;
    this.fy += fy;
    this.fz += fz;
    px -= this.transform.px;
    py -= this.transform.py;
    pz -= this.transform.pz;
    this.tx += py * fz - pz * fy;
    this.ty += pz * fx - px * fz;
    this.tz += px * fy - py * fx;
    this.sleeping = false;
    this.sleepTime = 0;
  }

  /** 在重心施力（原版 applyForceToCenter） */
  applyForceToCenter(fx: number, fy: number, fz: number): void {
    this.fx += fx;
    this.fy += fy;
    this.fz += fz;
    this.sleeping = false;
    this.sleepTime = 0;
  }

  /** 施加力矩，累加到下一步（原版 applyTorque） */
  applyTorque(tx: number, ty: number, tz: number): void {
    this.tx += tx;
    this.ty += ty;
    this.tz += tz;
    this.sleeping = false;
    this.sleepTime = 0;
  }

  setType(type: number): void {
    this.type = type;
    this.updateMass();
  }

  wakeUp(): void {
    this.sleeping = false;
    this.sleepTime = 0;
  }

  sleep(): void {
    this.sleeping = true;
    this.sleepTime = 0;
  }
}
