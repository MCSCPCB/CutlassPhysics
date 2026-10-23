// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/{UniversalJoint,UniversalJointConfig}.hx（MIT，见 Credits.md）
import { Vec3 } from "../../../common/vec3.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { TimeStep } from "../../time-step.js";
import { JointConfig, toLocalVector } from "./joint-config.js";
import { Joint, JointType } from "./joint.js";
import type { JointSolverInfo } from "./joint-solver-info.js";
import { RotationalLimitMotor } from "./rotational-limit-motor.js";
import { SpringDamper } from "./spring-damper.js";

export class UniversalJointConfig extends JointConfig {
  /** 刚体 1 的转轴、刚体 2 的转轴（各自的局部坐标），两者要垂直 */
  readonly localAxis1 = new Vec3(1, 0, 0);
  readonly localAxis2 = new Vec3(1, 0, 0);
  /** 绕两个转轴的弹簧阻尼、限制和马达 */
  readonly springDamper1 = new SpringDamper();
  readonly springDamper2 = new SpringDamper();
  readonly limitMotor1 = new RotationalLimitMotor();
  readonly limitMotor2 = new RotationalLimitMotor();

  /** 两个刚体在世界坐标的 anchor 处连起来，刚体 1 绕 axis1、刚体 2 绕 axis2（世界坐标）转动 */
  init(
    rigidBody1: RigidBody, rigidBody2: RigidBody, anchorX: number, anchorY: number, anchorZ: number,
    axis1X: number, axis1Y: number, axis1Z: number, axis2X: number, axis2Y: number, axis2Z: number,
  ): this {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    toLocalVector(rigidBody1, axis1X, axis1Y, axis1Z, this.localAxis1);
    toLocalVector(rigidBody2, axis2X, axis2Y, axis2Z, this.localAxis2);
    return this;
  }
}

/**
 * 万向节：刚体 1 的转轴（X1）和刚体 2 的转轴（Z2）保持垂直，可以分别绕它们转动，
 * 不能绕和两者都垂直的方向转（3 行平移 + 3 行转动约束，其中两行是限制和马达）
 */
export class UniversalJoint extends Joint {
  readonly sd1: SpringDamper;
  readonly sd2: SpringDamper;
  readonly lm1: RotationalLimitMotor;
  readonly lm2: RotationalLimitMotor;
  /** 约束的三个转动方向（世界坐标），同步锚点时更新 */
  axisXX = 0; axisXY = 0; axisXZ = 0;
  axisYX = 0; axisYY = 0; axisYZ = 0;
  axisZX = 0; axisZY = 0; axisZZ = 0;
  /** 相对转动的 XYZ 欧拉角：X、Z 是绕两个转轴的转角，Y 是偏差 */
  angleX = 0; angleY = 0; angleZ = 0;
  /** 约束方向退化成零向量时跳过对应的行 */
  xSingular = false; ySingular = false; zSingular = false;
  /** 两个锚点之间的偏差 */
  linearErrorX = 0; linearErrorY = 0; linearErrorZ = 0;

  constructor(config: UniversalJointConfig) {
    super(config, JointType.UNIVERSAL);
    const a1 = config.localAxis1, a2 = config.localAxis2;
    this.lbX1X = a1.x; this.lbX1Y = a1.y; this.lbX1Z = a1.z;
    this.lbZ2X = a2.x; this.lbZ2Y = a2.y; this.lbZ2Z = a2.z;
    this.buildLocalBasesFromX1Z2();
    this.sd1 = config.springDamper1.clone();
    this.sd2 = config.springDamper2.clone();
    this.lm1 = config.limitMotor1.clone();
    this.lm2 = config.limitMotor2.clone();
  }

  private getInfo(info: JointSolverInfo, timeStep: TimeStep | null, isPositionPart: boolean): void {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsX = this.linearErrorX * erp, linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const angRhsY = this.angleY * erp;
    // 平移部分的角度项和球关节一样：-[r]× 的各行，对角线是 -0
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const motorMassX = this.computeEffectiveInertiaMoment(this.axisXX, this.axisXY, this.axisXZ);
    const motorMassZ = this.computeEffectiveInertiaMoment(this.axisZX, this.axisZY, this.axisZZ);

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

    // 绕刚体 1 的转轴：限制、马达、弹簧
    if (!this.xSingular && (this.sd1.frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.angleX, this.lm1, motorMassX, this.sd1, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisXX; j.ang1Y = this.axisXY; j.ang1Z = this.axisXZ;
      j.ang2X = this.axisXX; j.ang2Y = this.axisXY; j.ang2Z = this.axisXZ;
    }

    // 两个转轴之间的偏差：不能转
    if (!this.ySingular) {
      row = info.addRow(this.impulses[4]);
      row.equalLimit(angRhsY, 0);
      j = row.jacobian;
      j.ang1X = this.axisYX; j.ang1Y = this.axisYY; j.ang1Z = this.axisYZ;
      j.ang2X = this.axisYX; j.ang2Y = this.axisYY; j.ang2Z = this.axisYZ;
    }

    // 绕刚体 2 的转轴：限制、马达、弹簧
    if (!this.zSingular && (this.sd2.frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[5]);
      this.setSolverInfoRowAngular(row, this.angleZ, this.lm2, motorMassZ, this.sd2, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisZX; j.ang1Y = this.axisZY; j.ang1Z = this.axisZZ;
      j.ang2X = this.axisZX; j.ang2Y = this.axisZY; j.ang2Z = this.axisZZ;
    }
  }

  /** 同步锚点和坐标系，然后更新约束方向、欧拉角和偏差 */
  override syncAnchors(): void {
    super.syncAnchors();

    // 欧拉角的转轴：X = X1，Z = Z2，Y = Z × X。
    // 约束方向：绕欧拉角 X 轴的转动要和 Y、Z 都垂直，所以取 Y × Z；Z 方向同理取 X × Y
    const aXX = this.bX1X, aXY = this.bX1Y, aXZ = this.bX1Z;
    const aZX = this.bZ2X, aZY = this.bZ2Y, aZZ = this.bZ2Z;
    const aYX = aZY * aXZ - aZZ * aXY, aYY = aZZ * aXX - aZX * aXZ, aYZ = aZX * aXY - aZY * aXX;
    this.axisXX = aYY * aZZ - aYZ * aZY; this.axisXY = aYZ * aZX - aYX * aZZ; this.axisXZ = aYX * aZY - aYY * aZX;
    this.axisYX = aYX; this.axisYY = aYY; this.axisYZ = aYZ;
    this.axisZX = aXY * aYZ - aXZ * aYY; this.axisZY = aXZ * aYX - aXX * aYZ; this.axisZZ = aXX * aYY - aXY * aYX;
    let l = this.axisXX * this.axisXX + this.axisXY * this.axisXY + this.axisXZ * this.axisXZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisXX *= l; this.axisXY *= l; this.axisXZ *= l;
    l = this.axisYX * this.axisYX + this.axisYY * this.axisYY + this.axisYZ * this.axisYZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisYX *= l; this.axisYY *= l; this.axisYZ *= l;
    l = this.axisZX * this.axisZX + this.axisZY * this.axisZY + this.axisZZ * this.axisZZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisZX *= l; this.axisZY *= l; this.axisZZ *= l;
    this.xSingular = this.axisXX * this.axisXX + this.axisXY * this.axisXY + this.axisXZ * this.axisXZ === 0;
    this.ySingular = this.axisYX * this.axisYX + this.axisYY * this.axisYY + this.axisYZ * this.axisYZ === 0;
    this.zSingular = this.axisZX * this.axisZX + this.axisZY * this.axisZY + this.axisZZ * this.axisZZ === 0;

    // 相对转动 = 刚体 1 的坐标系的转置 × 刚体 2 的坐标系（只算欧拉角用到的 7 个元素）
    const m00 = this.bX1X * this.bX2X + this.bX1Y * this.bX2Y + this.bX1Z * this.bX2Z;
    const m01 = this.bX1X * this.bY2X + this.bX1Y * this.bY2Y + this.bX1Z * this.bY2Z;
    const m02 = this.bX1X * this.bZ2X + this.bX1Y * this.bZ2Y + this.bX1Z * this.bZ2Z;
    const m11 = this.bY1X * this.bY2X + this.bY1Y * this.bY2Y + this.bY1Z * this.bY2Z;
    const m12 = this.bY1X * this.bZ2X + this.bY1Y * this.bZ2Y + this.bY1Z * this.bZ2Z;
    const m21 = this.bZ1X * this.bY2X + this.bZ1Y * this.bY2Y + this.bZ1Z * this.bY2Z;
    const m22 = this.bZ1X * this.bZ2X + this.bZ1Y * this.bZ2Y + this.bZ1Z * this.bZ2Z;
    // 旋转矩阵转 XYZ 欧拉角。π/2 是截断的 1.570796326794895
    const sy = m02;
    if (sy <= -1) {
      const xSubZ = Math.atan2(m21, m11);
      this.angleX = xSubZ * 0.5;
      this.angleY = -1.570796326794895;
      this.angleZ = -xSubZ * 0.5;
    } else if (sy >= 1) {
      const xAddZ = Math.atan2(m21, m11);
      this.angleX = xAddZ * 0.5;
      this.angleY = 1.570796326794895;
      this.angleZ = xAddZ * 0.5;
    } else {
      this.angleX = Math.atan2(-m12, m22);
      this.angleY = Math.asin(sy);
      this.angleZ = Math.atan2(-m01, m00);
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
