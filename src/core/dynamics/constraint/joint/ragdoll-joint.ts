// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/{RagdollJoint,RagdollJointConfig}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { Vec3 } from "../../../common/vec3.js";
import { MIN_RAGDOLL_MAX_SWING_ANGLE } from "../../../settings.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { TimeStep } from "../../time-step.js";
import { JointConfig, toLocalVector } from "./joint-config.js";
import { Joint, JointType } from "./joint.js";
import type { JointSolverInfo } from "./joint-solver-info.js";
import { RotationalLimitMotor } from "./rotational-limit-motor.js";
import { SpringDamper } from "./spring-damper.js";

export class RagdollJointConfig extends JointConfig {
  /** 两个刚体的扭转轴（各自的局部坐标） */
  readonly localTwistAxis1 = new Vec3(1, 0, 0);
  readonly localTwistAxis2 = new Vec3(1, 0, 0);
  /** 刚体 1 的第一个摆动轴（局部坐标）。第二个摆动轴和它、扭转轴都垂直，创建关节时自动算出 */
  readonly localSwingAxis1 = new Vec3(0, 1, 0);
  /** 绕扭转轴的弹簧阻尼、限制和马达 */
  readonly twistSpringDamper = new SpringDamper();
  readonly twistLimitMotor = new RotationalLimitMotor();
  /** 摆动方向的弹簧阻尼 */
  readonly swingSpringDamper = new SpringDamper();
  /** 绕两个摆动轴各自最多转多少（弧度，要大于 0）。原版的 π 是截断的 3.14159265358979 */
  maxSwingAngle1 = 3.14159265358979;
  maxSwingAngle2 = 3.14159265358979;

  /**
   * 两个刚体在世界坐标的 anchor 处连起来，扭转轴是 twistAxis、第一个摆动轴是 swingAxis（世界坐标）。
   */
  init(
    rigidBody1: RigidBody, rigidBody2: RigidBody, anchorX: number, anchorY: number, anchorZ: number,
    twistAxisX: number, twistAxisY: number, twistAxisZ: number, swingAxisX: number, swingAxisY: number, swingAxisZ: number,
  ): this {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    toLocalVector(rigidBody1, twistAxisX, twistAxisY, twistAxisZ, this.localTwistAxis1);
    toLocalVector(rigidBody2, twistAxisX, twistAxisY, twistAxisZ, this.localTwistAxis2);
    toLocalVector(rigidBody1, swingAxisX, swingAxisY, swingAxisZ, this.localSwingAxis1);
    return this;
  }
}

/**
 * 布娃娃关节：模拟四肢。固定相对位置；两个扭转轴之间的夹角（摆动）限制在一个椭圆锥里，
 * 绕扭转轴的转动（扭转）有限制、马达和弹簧（3 行平移 + 摆动 1 行 + 扭转 1 行）
 */
export class RagdollJoint extends Joint {
  readonly twistSd: SpringDamper;
  readonly swingSd: SpringDamper;
  readonly twistLm: RotationalLimitMotor;
  /** 椭圆锥的两个半角（弧度），不小于 MIN_RAGDOLL_MAX_SWING_ANGLE */
  maxSwingAngle1: number;
  maxSwingAngle2: number;
  /** 摆出锥外时把它推回去的方向（世界坐标）。在锥里时不更新，保留上一次的值（和原版一样） */
  swingAxisX = 0; swingAxisY = 0; swingAxisZ = 0;
  /** 扭转约束的方向：两个扭转轴的角平分线（世界坐标） */
  twistAxisX = 0; twistAxisY = 0; twistAxisZ = 0;
  /** 两个锚点之间的偏差 */
  linearErrorX = 0; linearErrorY = 0; linearErrorZ = 0;
  /** 摆出锥外多少（弧度），在锥里时是 0 */
  swingError = 0;
  /** 摆动那一行用的限制：只能往回推（下限 -1、上限 0，误差总是正的） */
  readonly dummySwingLm = new RotationalLimitMotor();
  /** 两个扭转轴的夹角、绕扭转轴转过的角度（弧度） */
  swingAngle = 0;
  twistAngle = 0;

  constructor(config: RagdollJointConfig) {
    super(config, JointType.RAGDOLL);
    const t1 = config.localTwistAxis1, s1 = config.localSwingAxis1, t2 = config.localTwistAxis2;
    this.lbX1X = t1.x; this.lbX1Y = t1.y; this.lbX1Z = t1.z;
    this.lbY1X = s1.x; this.lbY1Y = s1.y; this.lbY1Z = s1.z;
    this.lbX2X = t2.x; this.lbX2Y = t2.y; this.lbX2Z = t2.z;
    this.buildLocalBasesFromXY1X2();
    this.twistSd = config.twistSpringDamper.clone();
    this.twistLm = config.twistLimitMotor.clone();
    this.swingSd = config.swingSpringDamper.clone();
    this.maxSwingAngle1 = config.maxSwingAngle1;
    this.maxSwingAngle2 = config.maxSwingAngle2;
    if (this.maxSwingAngle1 < MIN_RAGDOLL_MAX_SWING_ANGLE) this.maxSwingAngle1 = MIN_RAGDOLL_MAX_SWING_ANGLE;
    if (this.maxSwingAngle2 < MIN_RAGDOLL_MAX_SWING_ANGLE) this.maxSwingAngle2 = MIN_RAGDOLL_MAX_SWING_ANGLE;
    this.dummySwingLm.lowerLimit = -1;
    this.dummySwingLm.upperLimit = 0;
  }

  private getInfo(info: JointSolverInfo, timeStep: TimeStep | null, isPositionPart: boolean): void {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsX = this.linearErrorX * erp, linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    // 平移部分的角度项和球关节一样：-[r]× 的各行，对角线是 -0
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const swingMass = this.computeEffectiveInertiaMoment(this.swingAxisX, this.swingAxisY, this.swingAxisZ);
    const twistMass = this.computeEffectiveInertiaMoment(this.bX2X, this.bX2Y, this.bX2Z);

    let row = info.addRow(this.impulses[0]);
    row.equalLimit(linRhsX, 0);
    let j = row.jacobian;
    j.lin1X = 1; j.lin1Y = 0; j.lin1Z = 0;
    j.lin2X = 1; j.lin2Y = 0; j.lin2Z = 0;
    j.ang1X = -0; j.ang1Y = r1Z; j.ang1Z = -r1Y;
    j.ang2X = -0; j.ang2Y = r2Z; j.ang2Z = -r2Y;

    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, 0);
    j = row.jacobian;
    j.lin1X = 0; j.lin1Y = 1; j.lin1Z = 0;
    j.lin2X = 0; j.lin2Y = 1; j.lin2Z = 0;
    j.ang1X = -r1Z; j.ang1Y = -0; j.ang1Z = r1X;
    j.ang2X = -r2Z; j.ang2Y = -0; j.ang2Z = r2X;

    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, 0);
    j = row.jacobian;
    j.lin1X = 0; j.lin1Y = 0; j.lin1Z = 1;
    j.lin2X = 0; j.lin2Y = 0; j.lin2Z = 1;
    j.ang1X = r1Y; j.ang1Y = -r1X; j.ang1Z = -0;
    j.ang2X = r2Y; j.ang2Y = -r2X; j.ang2Z = -0;

    // 摆动：摆出锥外时往回推
    if (this.swingError > 0 && (this.swingSd.frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.swingError, this.dummySwingLm, swingMass, this.swingSd, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.swingAxisX; j.ang1Y = this.swingAxisY; j.ang1Z = this.swingAxisZ;
      j.ang2X = this.swingAxisX; j.ang2Y = this.swingAxisY; j.ang2Z = this.swingAxisZ;
    }

    // 扭转：限制、马达、弹簧
    if (this.twistSd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[4]);
      this.setSolverInfoRowAngular(row, this.twistAngle, this.twistLm, twistMass, this.twistSd, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.twistAxisX; j.ang1Y = this.twistAxisY; j.ang1Z = this.twistAxisZ;
      j.ang2X = this.twistAxisX; j.ang2Y = this.twistAxisY; j.ang2Z = this.twistAxisZ;
    }
  }

  /** 同步锚点和坐标系，然后更新摆动角、扭转角、约束方向和偏差（原版 _syncAnchors + computeErrors） */
  override syncAnchors(): void {
    super.syncAnchors();
    const x1X = this.bX1X, x1Y = this.bX1Y, x1Z = this.bX1Z;
    const x2X = this.bX2X, x2Y = this.bX2Y, x2Z = this.bX2Z;

    // 摆动 q = 从 X1 转到 X2 的最短弧
    let qX: number, qY: number, qZ: number, qW: number;
    let d = x1X * x2X + x1Y * x2Y + x1Z * x2Z;
    if (d < -0.999999999) {
      const x2 = x1X * x1X, y2 = x1Y * x1Y, z2 = x1Z * x1Z;
      let e: number;
      if (x2 < y2) {
        if (x2 < z2) {
          e = 1 / Math.sqrt(y2 + z2);
          qX = 0; qY = x1Z * e; qZ = -x1Y * e;
        } else {
          e = 1 / Math.sqrt(x2 + y2);
          qX = x1Y * e; qY = -x1X * e; qZ = 0;
        }
      } else if (y2 < z2) {
        e = 1 / Math.sqrt(z2 + x2);
        qX = -x1Z * e; qY = 0; qZ = x1X * e;
      } else {
        e = 1 / Math.sqrt(x2 + y2);
        qX = x1Y * e; qY = -x1X * e; qZ = 0;
      }
      qW = 0;
    } else {
      let cX = x1Y * x2Z - x1Z * x2Y, cY = x1Z * x2X - x1X * x2Z, cZ = x1X * x2Y - x1Y * x2X;
      const w = Math.sqrt((1 + d) * 0.5);
      d = 0.5 / w;
      cX *= d; cY *= d; cZ *= d;
      qX = cX; qY = cY; qZ = cZ; qW = w;
    }
    const x2 = 2 * qX, y2 = 2 * qY, z2 = 2 * qZ;
    const xx = qX * x2, yy = qY * y2, zz = qZ * z2;
    const xy = qX * y2, yz = qY * z2, xz = qX * z2;
    const wx = qW * x2, wy = qW * y2, wz = qW * z2;
    const s00 = 1 - yy - zz, s01 = xy - wz, s02 = xz + wy;
    const s10 = xy + wz, s11 = 1 - xx - zz, s12 = yz - wx;
    const s20 = xz - wy, s21 = yz + wx, s22 = 1 - xx - yy;
    // 原版 safeAcos：超出 [-1, 1] 时取端点（原版的 π 是截断的 3.14159265358979）
    this.swingAngle = (qW <= -1 ? 3.14159265358979 : qW >= 1 ? 0 : Math.acos(qW)) * 2;

    // 扭转角：把 Y2 按摆动反着转回来（乘摆动矩阵的转置），看它在 Y1、Z1 平面上转了多少
    const yInX = s00 * this.bY2X + s10 * this.bY2Y + s20 * this.bY2Z;
    const yInY = s01 * this.bY2X + s11 * this.bY2Y + s21 * this.bY2Z;
    const yInZ = s02 * this.bY2X + s12 * this.bY2Y + s22 * this.bY2Z;
    this.twistAngle = Math.atan2(
      this.bZ1X * yInX + this.bZ1Y * yInY + this.bZ1Z * yInZ,
      this.bY1X * yInX + this.bY1Y * yInY + this.bY1Z * yInZ,
    );

    // 扭转约束的方向：X1、X2 的角平分线
    this.twistAxisX = this.bX1X + this.bX2X;
    this.twistAxisY = this.bX1Y + this.bX2Y;
    this.twistAxisZ = this.bX1Z + this.bX2Z;
    let l = this.twistAxisX * this.twistAxisX + this.twistAxisY * this.twistAxisY + this.twistAxisZ * this.twistAxisZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.twistAxisX *= l; this.twistAxisY *= l; this.twistAxisZ *= l;

    // 摆动向量：方向是摆动的转轴，长度是摆动角；再换到刚体 1 的约束坐标系，只要 Y1、Z1 两个分量
    let vX = qX, vY = qY, vZ = qZ;
    let invLen = Math.sqrt(vX * vX + vY * vY + vZ * vZ);
    if (invLen > 0) invLen = 1 / invLen;
    vX *= invLen * this.swingAngle;
    vY *= invLen * this.swingAngle;
    vZ *= invLen * this.swingAngle;
    const x = this.bY1X * vX + this.bY1Y * vY + this.bY1Z * vZ;
    const y = this.bZ1X * vX + this.bZ1Y * vY + this.bZ1Z * vZ;

    // 椭圆锥 x²/a² + y²/b² <= 1：沿 (x, y) 方向和椭圆的交点 (x0, y0)，在那里的法线方向上量出界多少
    const a = this.maxSwingAngle1, b = this.maxSwingAngle2;
    const invA2 = 1 / (a * a), invB2 = 1 / (b * b);
    const w1 = x * x * invA2 + y * y * invB2;
    if (w1 === 0) {
      this.swingAxisX = 0; this.swingAxisY = 0; this.swingAxisZ = 0;
      this.swingError = 0;
    } else {
      const t = Math.sqrt(1 / w1);
      const x0 = x * t, y0 = y * t;
      let nx = x0 * invA2, ny = y0 * invB2;
      invLen = 1 / Math.sqrt(nx * nx + ny * ny);
      nx *= invLen;
      ny *= invLen;
      const depth = (x - x0) * nx + (y - y0) * ny;
      if (depth > 0) {
        this.swingError = depth;
        // 法线 (0, nx, ny) 换到世界坐标（乘刚体 1 的约束坐标系），再按摆动转过去。
        // 零乘积项保留浮点结果的零符号。
        const n0 = 0;
        const nX = this.bX1X * n0 + this.bY1X * nx + this.bZ1X * ny;
        const nY = this.bX1Y * n0 + this.bY1Y * nx + this.bZ1Y * ny;
        const nZ = this.bX1Z * n0 + this.bY1Z * nx + this.bZ1Z * ny;
        this.swingAxisX = s00 * nX + s01 * nY + s02 * nZ;
        this.swingAxisY = s10 * nX + s11 * nY + s12 * nZ;
        this.swingAxisZ = s20 * nX + s21 * nY + s22 * nZ;
      } else {
        this.swingError = 0;
      }
    }

    this.linearErrorX = this.a2X - this.a1X;
    this.linearErrorY = this.a2Y - this.a1Y;
    this.linearErrorZ = this.a2Z - this.a1Z;
  }

  override getVelocitySolverInfo(timeStep: TimeStep, info: JointSolverInfo): void {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }

  override getPositionSolverInfo(info: JointSolverInfo): void {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
}
