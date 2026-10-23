// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/{CylindricalJoint,CylindricalJointConfig}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { Vec3 } from "../../../common/vec3.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { TimeStep } from "../../time-step.js";
import { BasisTracker } from "./basis-tracker.js";
import { JointConfig, toLocalVector } from "./joint-config.js";
import { Joint, JointType } from "./joint.js";
import type { JointSolverInfo } from "./joint-solver-info.js";
import { RotationalLimitMotor } from "./rotational-limit-motor.js";
import { SpringDamper } from "./spring-damper.js";
import { TranslationalLimitMotor } from "./translational-limit-motor.js";

export class CylindricalJointConfig extends JointConfig {
  /** 轴在两个刚体上的局部方向 */
  readonly localAxis1 = new Vec3(1, 0, 0);
  readonly localAxis2 = new Vec3(1, 0, 0);
  /** 沿轴平移的限制和马达、弹簧阻尼 */
  readonly translationalLimitMotor = new TranslationalLimitMotor();
  readonly translationalSpringDamper = new SpringDamper();
  /** 绕轴转动的限制和马达、弹簧阻尼 */
  readonly rotationalLimitMotor = new RotationalLimitMotor();
  readonly rotationalSpringDamper = new SpringDamper();

  /** 两个刚体在世界坐标的 anchor 处连起来，可以沿世界坐标的 axis 平移、绕它转动 */
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

/** 圆柱关节：两个刚体的轴重合，可以沿轴平移、绕轴转动（2 行平移 + 2 行转动约束，加上沿轴、绕轴各一行限制和马达） */
export class CylindricalJoint extends Joint {
  readonly translSd: SpringDamper;
  readonly translLm: TranslationalLimitMotor;
  readonly rotSd: SpringDamper;
  readonly rotLm: RotationalLimitMotor;
  readonly basis: BasisTracker;
  /** 绕轴的转角（弧度）、沿轴的位移，同步锚点时更新 */
  angle = 0;
  translation = 0;
  /** 两个刚体的轴之间的偏差、两个锚点之间的偏差，沿公共坐标系的 y、z 分解 */
  angularErrorY = 0;
  angularErrorZ = 0;
  linearErrorY = 0;
  linearErrorZ = 0;

  constructor(config: CylindricalJointConfig) {
    super(config, JointType.CYLINDRICAL);
    const a1 = config.localAxis1, a2 = config.localAxis2;
    this.lbX1X = a1.x; this.lbX1Y = a1.y; this.lbX1Z = a1.z;
    this.lbX2X = a2.x; this.lbX2Y = a2.y; this.lbX2Z = a2.z;
    this.buildLocalBasesFromX();
    this.basis = new BasisTracker(this);
    this.translSd = config.translationalSpringDamper.clone();
    this.translLm = config.translationalLimitMotor.clone();
    this.rotSd = config.rotationalSpringDamper.clone();
    this.rotLm = config.rotationalLimitMotor.clone();
  }

  private getInfo(info: JointSolverInfo, timeStep: TimeStep | null, isPositionPart: boolean): void {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const angRhsY = this.angularErrorY * erp, angRhsZ = this.angularErrorZ * erp;
    const b = this.basis;
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const translationalMotorMass = 1 / (this.b1.invMass + this.b2.invMass);
    const rotationalMotorMass = this.computeEffectiveInertiaMoment(b.xX, b.xY, b.xZ);

    // 沿轴：限制、马达、弹簧。有弹簧时位置修正阶段不处理这一行
    let row, j;
    if (this.translSd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[0]);
      this.setSolverInfoRowLinear(row, this.translation, this.translLm, translationalMotorMass, this.translSd, timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = b.xX; j.lin1Y = b.xY; j.lin1Z = b.xZ;
      j.lin2X = b.xX; j.lin2Y = b.xY; j.lin2Z = b.xZ;
      j.ang1X = r1Y * b.xZ - r1Z * b.xY; j.ang1Y = r1Z * b.xX - r1X * b.xZ; j.ang1Z = r1X * b.xY - r1Y * b.xX;
      j.ang2X = r2Y * b.xZ - r2Z * b.xY; j.ang2Y = r2Z * b.xX - r2X * b.xZ; j.ang2Z = r2X * b.xY - r2Y * b.xX;
    }

    // 垂直于轴不能平移
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

    // 绕轴：限制、马达、弹簧
    if (this.rotSd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.angle, this.rotLm, rotationalMotorMass, this.rotSd, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = b.xX; j.ang1Y = b.xY; j.ang1Z = b.xZ;
      j.ang2X = b.xX; j.ang2Y = b.xY; j.ang2Z = b.xZ;
    }

    // 轴的偏差：绕 y、z 不能转
    row = info.addRow(this.impulses[4]);
    row.equalLimit(angRhsY, 0);
    j = row.jacobian;
    j.ang1X = b.yX; j.ang1Y = b.yY; j.ang1Z = b.yZ;
    j.ang2X = b.yX; j.ang2Y = b.yY; j.ang2Z = b.yZ;

    row = info.addRow(this.impulses[5]);
    row.equalLimit(angRhsZ, 0);
    j = row.jacobian;
    j.ang1X = b.zX; j.ang1Y = b.zY; j.ang1Z = b.zZ;
    j.ang2X = b.zX; j.ang2Y = b.zY; j.ang2Z = b.zZ;
  }

  /** 同步锚点和坐标系，然后更新公共坐标系、转角、位移和偏差（原版 _syncAnchors + computeErrors） */
  override syncAnchors(): void {
    super.syncAnchors();
    const b = this.basis;
    b.trackByX();

    // 轴的偏差：X1 转到 X2 的转轴 × 夹角，投影到 y、z（和转动关节一样）
    let errX = this.bX1Y * this.bX2Z - this.bX1Z * this.bX2Y;
    let errY = this.bX1Z * this.bX2X - this.bX1X * this.bX2Z;
    let errZ = this.bX1X * this.bX2Y - this.bX1Y * this.bX2X;
    let cos = this.bX1X * this.bX2X + this.bX1Y * this.bX2Y + this.bX1Z * this.bX2Z;
    // 原版 safeAcos：超出 [-1, 1] 时取端点（原版的 π 是截断的 3.14159265358979）
    const theta = cos <= -1 ? 3.14159265358979 : cos >= 1 ? 0 : Math.acos(cos);
    let l = errX * errX + errY * errY + errZ * errZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    errX *= l; errY *= l; errZ *= l;
    errX *= theta; errY *= theta; errZ *= theta;
    this.angularErrorY = errX * b.yX + errY * b.yY + errZ * b.yZ;
    this.angularErrorZ = errX * b.zX + errY * b.zY + errZ * b.zZ;

    // 绕轴的转角
    const pcX = this.bY1Y * this.bY2Z - this.bY1Z * this.bY2Y;
    const pcY = this.bY1Z * this.bY2X - this.bY1X * this.bY2Z;
    const pcZ = this.bY1X * this.bY2Y - this.bY1Y * this.bY2X;
    cos = this.bY1X * this.bY2X + this.bY1Y * this.bY2Y + this.bY1Z * this.bY2Z;
    this.angle = cos <= -1 ? 3.14159265358979 : cos >= 1 ? 0 : Math.acos(cos);
    if (pcX * b.xX + pcY * b.xY + pcZ * b.xZ < 0) this.angle = -this.angle;

    // 两个锚点之间的偏差投影到公共坐标系（和滑动关节一样）
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
