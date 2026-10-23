// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/{PrismaticJoint,PrismaticJointConfig}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { Vec3 } from "../../../common/vec3.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { TimeStep } from "../../time-step.js";
import { BasisTracker } from "./basis-tracker.js";
import { JointConfig, toLocalVector } from "./joint-config.js";
import { Joint, JointType } from "./joint.js";
import type { JointSolverInfo } from "./joint-solver-info.js";
import { SpringDamper } from "./spring-damper.js";
import { TranslationalLimitMotor } from "./translational-limit-motor.js";

export class PrismaticJointConfig extends JointConfig {
  /** 滑动方向在两个刚体上的局部方向 */
  readonly localAxis1 = new Vec3(1, 0, 0);
  readonly localAxis2 = new Vec3(1, 0, 0);
  /** 平移方向的限制和马达 */
  readonly limitMotor = new TranslationalLimitMotor();
  /** 平移方向的弹簧阻尼 */
  readonly springDamper = new SpringDamper();

  /** 两个刚体在世界坐标的 anchor 处连起来，沿世界坐标的 axis 滑动 */
  init(
    rigidBody1: RigidBody, rigidBody2: RigidBody,
    anchorX: number, anchorY: number, anchorZ: number, axisX: number, axisY: number, axisZ: number,
  ): this {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    toLocalVector(rigidBody1, axisX, axisY, axisZ, this.localAxis1);
    toLocalVector(rigidBody2, axisX, axisY, axisZ, this.localAxis2);
    return this;
  }
}

/** 滑动关节：两个刚体不能相对转动，只能沿滑动方向相对平移（3 行平移 + 3 行转动约束） */
export class PrismaticJoint extends Joint {
  readonly sd: SpringDamper;
  readonly lm: TranslationalLimitMotor;
  readonly basis: BasisTracker;
  /** 沿滑动方向的位移，同步锚点时更新 */
  translation = 0;
  /** 两个锚点之间的偏差沿公共坐标系的 y、z 分解 */
  linearErrorY = 0;
  linearErrorZ = 0;
  /** 两个刚体的相对转动（转动向量，世界坐标） */
  angularErrorX = 0; angularErrorY = 0; angularErrorZ = 0;

  constructor(config: PrismaticJointConfig) {
    super(config, JointType.PRISMATIC);
    const a1 = config.localAxis1, a2 = config.localAxis2;
    this.lbX1X = a1.x; this.lbX1Y = a1.y; this.lbX1Z = a1.z;
    this.lbX2X = a2.x; this.lbX2Y = a2.y; this.lbX2Z = a2.z;
    this.buildLocalBasesFromX();
    this.basis = new BasisTracker(this);
    this.sd = config.springDamper.clone();
    this.lm = config.limitMotor.clone();
  }

  private getInfo(info: JointSolverInfo, timeStep: TimeStep | null, isPositionPart: boolean): void {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const angRhsX = this.angularErrorX * erp, angRhsY = this.angularErrorY * erp, angRhsZ = this.angularErrorZ * erp;
    const b = this.basis;
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;

    // 沿滑动方向：限制、马达、弹簧。有弹簧时位置修正阶段不处理这一行
    let row, j;
    if (this.sd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[0]);
      this.setSolverInfoRowLinear(row, this.translation, this.lm, 1 / (this.b1.invMass + this.b2.invMass), this.sd, timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = b.xX; j.lin1Y = b.xY; j.lin1Z = b.xZ;
      j.lin2X = b.xX; j.lin2Y = b.xY; j.lin2Z = b.xZ;
      j.ang1X = r1Y * b.xZ - r1Z * b.xY; j.ang1Y = r1Z * b.xX - r1X * b.xZ; j.ang1Z = r1X * b.xY - r1Y * b.xX;
      j.ang2X = r2Y * b.xZ - r2Z * b.xY; j.ang2Y = r2Z * b.xX - r2X * b.xZ; j.ang2Z = r2X * b.xY - r2Y * b.xX;
    }

    // 垂直于滑动方向不能动
    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, 0);
    j = row.jacobian;
    j.lin1X = b.yX; j.lin1Y = b.yY; j.lin1Z = b.yZ;
    j.lin2X = b.yX; j.lin2Y = b.yY; j.lin2Z = b.yZ;
    j.ang1X = r1Y * b.yZ - r1Z * b.yY; j.ang1Y = r1Z * b.yX - r1X * b.yZ; j.ang1Z = r1X * b.yY - r1Y * b.yX;
    j.ang2X = r2Y * b.yZ - r2Z * b.yY; j.ang2Y = r2Z * b.yX - r2X * b.yZ; j.ang2Z = r2X * b.yY - r2Y * b.yX;

    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, 0);
    j = row.jacobian;
    j.lin1X = b.zX; j.lin1Y = b.zY; j.lin1Z = b.zZ;
    j.lin2X = b.zX; j.lin2Y = b.zY; j.lin2Z = b.zZ;
    j.ang1X = r1Y * b.zZ - r1Z * b.zY; j.ang1Y = r1Z * b.zX - r1X * b.zZ; j.ang1Z = r1X * b.zY - r1Y * b.zX;
    j.ang2X = r2Y * b.zZ - r2Z * b.zY; j.ang2Y = r2Z * b.zX - r2X * b.zZ; j.ang2Z = r2X * b.zY - r2Y * b.zX;

    // 不能相对转动：世界坐标的 x、y、z 三个方向
    row = info.addRow(this.impulses[3]);
    row.equalLimit(angRhsX, 0);
    j = row.jacobian;
    j.ang1X = 1; j.ang1Y = 0; j.ang1Z = 0;
    j.ang2X = 1; j.ang2Y = 0; j.ang2Z = 0;

    row = info.addRow(this.impulses[4]);
    row.equalLimit(angRhsY, 0);
    j = row.jacobian;
    j.ang1X = 0; j.ang1Y = 1; j.ang1Z = 0;
    j.ang2X = 0; j.ang2Y = 1; j.ang2Z = 0;

    row = info.addRow(this.impulses[5]);
    row.equalLimit(angRhsZ, 0);
    j = row.jacobian;
    j.ang1X = 0; j.ang1Y = 0; j.ang1Z = 1;
    j.ang2X = 0; j.ang2Y = 0; j.ang2Z = 1;
  }

  /** 同步锚点和坐标系，然后更新公共坐标系、相对转动和位移（原版 _syncAnchors + computeErrors） */
  override syncAnchors(): void {
    super.syncAnchors();
    const b = this.basis;
    b.trackByX();

    // 相对转动 = 刚体 2 的坐标系 × 刚体 1 的坐标系的转置（两个坐标系的轴作为矩阵的列），再转成四元数
    const a00 = this.bX1X, a01 = this.bY1X, a02 = this.bZ1X;
    const a10 = this.bX1Y, a11 = this.bY1Y, a12 = this.bZ1Y;
    const a20 = this.bX1Z, a21 = this.bY1Z, a22 = this.bZ1Z;
    const c00 = this.bX2X, c01 = this.bY2X, c02 = this.bZ2X;
    const c10 = this.bX2Y, c11 = this.bY2Y, c12 = this.bZ2Y;
    const c20 = this.bX2Z, c21 = this.bY2Z, c22 = this.bZ2Z;
    const m00 = c00 * a00 + c01 * a01 + c02 * a02;
    const m01 = c00 * a10 + c01 * a11 + c02 * a12;
    const m02 = c00 * a20 + c01 * a21 + c02 * a22;
    const m10 = c10 * a00 + c11 * a01 + c12 * a02;
    const m11 = c10 * a10 + c11 * a11 + c12 * a12;
    const m12 = c10 * a20 + c11 * a21 + c12 * a22;
    const m20 = c20 * a00 + c21 * a01 + c22 * a02;
    const m21 = c20 * a10 + c21 * a11 + c22 * a12;
    const m22 = c20 * a20 + c21 * a21 + c22 * a22;
    let qX: number, qY: number, qZ: number, qW: number, s: number;
    const t = m00 + m11 + m22;
    if (t > 0) {
      s = Math.sqrt(t + 1);
      qW = 0.5 * s;
      s = 0.5 / s;
      qX = (m21 - m12) * s; qY = (m02 - m20) * s; qZ = (m10 - m01) * s;
    } else if (m00 > m11) {
      if (m00 > m22) {
        s = Math.sqrt(m00 - m11 - m22 + 1);
        qX = 0.5 * s;
        s = 0.5 / s;
        qY = (m01 + m10) * s; qZ = (m02 + m20) * s; qW = (m21 - m12) * s;
      } else {
        s = Math.sqrt(m22 - m00 - m11 + 1);
        qZ = 0.5 * s;
        s = 0.5 / s;
        qX = (m02 + m20) * s; qY = (m12 + m21) * s; qW = (m10 - m01) * s;
      }
    } else if (m11 > m22) {
      s = Math.sqrt(m11 - m22 - m00 + 1);
      qY = 0.5 * s;
      s = 0.5 / s;
      qX = (m01 + m10) * s; qZ = (m12 + m21) * s; qW = (m02 - m20) * s;
    } else {
      s = Math.sqrt(m22 - m00 - m11 + 1);
      qZ = 0.5 * s;
      s = 0.5 / s;
      qX = (m02 + m20) * s; qY = (m12 + m21) * s; qW = (m10 - m01) * s;
    }
    // 转动向量 = 转轴 × 转角。原版 safeAcos：超出 [-1, 1] 时取端点（原版的 π 是截断的 3.14159265358979）
    const theta = (qW <= -1 ? 3.14159265358979 : qW >= 1 ? 0 : Math.acos(qW)) * 2;
    let l = qX * qX + qY * qY + qZ * qZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.angularErrorX = qX * l * theta;
    this.angularErrorY = qY * l * theta;
    this.angularErrorZ = qZ * l * theta;

    // 两个锚点之间的偏差投影到公共坐标系
    const dX = this.a2X - this.a1X, dY = this.a2Y - this.a1Y, dZ = this.a2Z - this.a1Z;
    this.translation = dX * b.xX + dY * b.xY + dZ * b.xZ;
    this.linearErrorY = dX * b.yX + dY * b.yY + dZ * b.yZ;
    this.linearErrorZ = dX * b.zX + dY * b.zY + dZ * b.zZ;
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
