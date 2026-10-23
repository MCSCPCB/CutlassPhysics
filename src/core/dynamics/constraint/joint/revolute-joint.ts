// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/{RevoluteJoint,RevoluteJointConfig}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { Vec3 } from "../../../common/vec3.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { TimeStep } from "../../time-step.js";
import { BasisTracker } from "./basis-tracker.js";
import { JointConfig, toLocalVector } from "./joint-config.js";
import { Joint, JointType } from "./joint.js";
import type { JointSolverInfo } from "./joint-solver-info.js";
import { RotationalLimitMotor } from "./rotational-limit-motor.js";
import { SpringDamper } from "./spring-damper.js";

export class RevoluteJointConfig extends JointConfig {
  /** 转轴在两个刚体上的局部方向 */
  readonly localAxis1 = new Vec3(1, 0, 0);
  readonly localAxis2 = new Vec3(1, 0, 0);
  /** 转动方向的弹簧阻尼 */
  readonly springDamper = new SpringDamper();
  /** 转动方向的限制和马达 */
  readonly limitMotor = new RotationalLimitMotor();

  /** 两个刚体在世界坐标的 anchor 处连起来，绕世界坐标的 axis 转动 */
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

/** 转动关节（铰链）：锚点重合、转轴重合，只能绕转轴转动（3 行平移 + 3 行转动约束） */
export class RevoluteJoint extends Joint {
  readonly sd: SpringDamper;
  readonly lm: RotationalLimitMotor;
  readonly basis: BasisTracker;
  /** 绕转轴的转角（弧度），同步锚点时更新 */
  angle = 0;
  /** 两个刚体的转轴之间的偏差，沿公共坐标系的 y、z 分解 */
  angularErrorY = 0;
  angularErrorZ = 0;
  /** 两个锚点之间的偏差 */
  linearErrorX = 0; linearErrorY = 0; linearErrorZ = 0;

  constructor(config: RevoluteJointConfig) {
    super(config, JointType.REVOLUTE);
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
    const linRhsX = this.linearErrorX * erp, linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const angRhsY = this.angularErrorY * erp, angRhsZ = this.angularErrorZ * erp;
    // 平移部分的角度项和球关节一样：-[r]× 的各行，对角线是 -0
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const b = this.basis;
    const motorMass = this.computeEffectiveInertiaMoment(b.xX, b.xY, b.xZ);

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

    // 绕转轴：限制、马达、弹簧。有弹簧时位置修正阶段不处理这一行
    if (this.sd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.angle, this.lm, motorMass, this.sd, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = b.xX; j.ang1Y = b.xY; j.ang1Z = b.xZ;
      j.ang2X = b.xX; j.ang2Y = b.xY; j.ang2Z = b.xZ;
    }

    // 转轴的偏差：绕 y、z 不能转
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

  /** 同步锚点和坐标系，然后更新公共坐标系、转角和偏差（原版 _syncAnchors + computeErrors） */
  override syncAnchors(): void {
    super.syncAnchors();
    const b = this.basis;
    b.trackByX();

    // 转轴的偏差：X1 转到 X2 的转轴 × 夹角，投影到 y、z
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

    // 绕 x 的转角：两个刚体的 Y 轴之间的夹角，方向由叉乘和 x 的点积决定
    const pcX = this.bY1Y * this.bY2Z - this.bY1Z * this.bY2Y;
    const pcY = this.bY1Z * this.bY2X - this.bY1X * this.bY2Z;
    const pcZ = this.bY1X * this.bY2Y - this.bY1Y * this.bY2X;
    cos = this.bY1X * this.bY2X + this.bY1Y * this.bY2Y + this.bY1Z * this.bY2Z;
    this.angle = cos <= -1 ? 3.14159265358979 : cos >= 1 ? 0 : Math.acos(cos);
    if (pcX * b.xX + pcY * b.xY + pcZ * b.xZ < 0) this.angle = -this.angle;

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
