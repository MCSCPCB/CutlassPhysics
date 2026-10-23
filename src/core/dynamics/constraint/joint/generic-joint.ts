// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/{GenericJoint,GenericJointConfig}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { Mat3 } from "../../../common/mat3.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { TimeStep } from "../../time-step.js";
import { JointConfig } from "./joint-config.js";
import { Joint, JointType } from "./joint.js";
import type { JointSolverInfo } from "./joint-solver-info.js";
import { RotationalLimitMotor } from "./rotational-limit-motor.js";
import { SpringDamper } from "./spring-damper.js";
import { TranslationalLimitMotor } from "./translational-limit-motor.js";

type Three<T> = [T, T, T];

export class GenericJointConfig extends JointConfig {
  /** 两个刚体的约束坐标系（各自的局部坐标，三列是 X、Y、Z 轴），要是右手系，不会自动归一化、正交化 */
  readonly localBasis1 = new Mat3();
  readonly localBasis2 = new Mat3();
  /** 沿刚体 1 约束坐标系三个轴的平移：限制和马达、弹簧阻尼。默认上下限都是 0（锁死） */
  readonly translationalLimitMotors: Three<TranslationalLimitMotor> = [
    new TranslationalLimitMotor().setLimits(0, 0), new TranslationalLimitMotor().setLimits(0, 0), new TranslationalLimitMotor().setLimits(0, 0),
  ];
  readonly translationalSpringDampers: Three<SpringDamper> = [new SpringDamper(), new SpringDamper(), new SpringDamper()];
  /** 绕相对转动的 XYZ 欧拉角三个转轴的转动：限制和马达、弹簧阻尼。默认上下限都是 0（锁死） */
  readonly rotationalLimitMotors: Three<RotationalLimitMotor> = [
    new RotationalLimitMotor().setLimits(0, 0), new RotationalLimitMotor().setLimits(0, 0), new RotationalLimitMotor().setLimits(0, 0),
  ];
  readonly rotationalSpringDampers: Three<SpringDamper> = [new SpringDamper(), new SpringDamper(), new SpringDamper()];

  /** 两个刚体在世界坐标的 anchor 处连起来，约束坐标系是世界坐标的 basis1、basis2（三列是 X、Y、Z 轴） */
  init(rigidBody1: RigidBody, rigidBody2: RigidBody, anchorX: number, anchorY: number, anchorZ: number, basis1: Mat3, basis2: Mat3): this {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    // 局部坐标系 = 刚体旋转矩阵的转置 × 世界坐标系
    const r1 = rigidBody1.transform, r2 = rigidBody2.transform, w1 = basis1, w2 = basis2;
    this.localBasis1.init(
      r1.r00 * w1.e00 + r1.r10 * w1.e10 + r1.r20 * w1.e20, r1.r00 * w1.e01 + r1.r10 * w1.e11 + r1.r20 * w1.e21, r1.r00 * w1.e02 + r1.r10 * w1.e12 + r1.r20 * w1.e22,
      r1.r01 * w1.e00 + r1.r11 * w1.e10 + r1.r21 * w1.e20, r1.r01 * w1.e01 + r1.r11 * w1.e11 + r1.r21 * w1.e21, r1.r01 * w1.e02 + r1.r11 * w1.e12 + r1.r21 * w1.e22,
      r1.r02 * w1.e00 + r1.r12 * w1.e10 + r1.r22 * w1.e20, r1.r02 * w1.e01 + r1.r12 * w1.e11 + r1.r22 * w1.e21, r1.r02 * w1.e02 + r1.r12 * w1.e12 + r1.r22 * w1.e22,
    );
    this.localBasis2.init(
      r2.r00 * w2.e00 + r2.r10 * w2.e10 + r2.r20 * w2.e20, r2.r00 * w2.e01 + r2.r10 * w2.e11 + r2.r20 * w2.e21, r2.r00 * w2.e02 + r2.r10 * w2.e12 + r2.r20 * w2.e22,
      r2.r01 * w2.e00 + r2.r11 * w2.e10 + r2.r21 * w2.e20, r2.r01 * w2.e01 + r2.r11 * w2.e11 + r2.r21 * w2.e21, r2.r01 * w2.e02 + r2.r11 * w2.e12 + r2.r21 * w2.e22,
      r2.r02 * w2.e00 + r2.r12 * w2.e10 + r2.r22 * w2.e20, r2.r02 * w2.e01 + r2.r12 * w2.e11 + r2.r22 * w2.e21, r2.r02 * w2.e02 + r2.r12 * w2.e12 + r2.r22 * w2.e22,
    );
    return this;
  }
}

/**
 * 通用关节（六自由度关节）：沿刚体 1 约束坐标系的三个轴平移、绕相对转动的 XYZ 欧拉角三个转轴转动，
 * 每个方向都可以锁死、放开、加限制、马达或弹簧（3 行平移 + 3 行转动约束）
 */
export class GenericJoint extends Joint {
  readonly translSds: Three<SpringDamper>;
  readonly rotSds: Three<SpringDamper>;
  readonly translLms: Three<TranslationalLimitMotor>;
  readonly rotLms: Three<RotationalLimitMotor>;
  /** 约束的三个转动方向（世界坐标），同步锚点时更新 */
  axisXX = 0; axisXY = 0; axisXZ = 0;
  axisYX = 0; axisYY = 0; axisYZ = 0;
  axisZX = 0; axisZY = 0; axisZZ = 0;
  /** 相对转动的 XYZ 欧拉角 */
  angleX = 0; angleY = 0; angleZ = 0;
  /** 约束方向退化成零向量时跳过对应的行 */
  xSingular = false; ySingular = false; zSingular = false;
  /** 两个锚点之间的偏差沿刚体 1 约束坐标系三个轴的分量 */
  translationX = 0; translationY = 0; translationZ = 0;

  constructor(config: GenericJointConfig) {
    super(config, JointType.GENERIC);
    const m1 = config.localBasis1, m2 = config.localBasis2;
    this.lbX1X = m1.e00; this.lbX1Y = m1.e10; this.lbX1Z = m1.e20;
    this.lbY1X = m1.e01; this.lbY1Y = m1.e11; this.lbY1Z = m1.e21;
    this.lbZ1X = m1.e02; this.lbZ1Y = m1.e12; this.lbZ1Z = m1.e22;
    this.lbX2X = m2.e00; this.lbX2Y = m2.e10; this.lbX2Z = m2.e20;
    this.lbY2X = m2.e01; this.lbY2Y = m2.e11; this.lbY2Z = m2.e21;
    this.lbZ2X = m2.e02; this.lbZ2Y = m2.e12; this.lbZ2Z = m2.e22;
    const tl = config.translationalLimitMotors, ts = config.translationalSpringDampers;
    const rl = config.rotationalLimitMotors, rs = config.rotationalSpringDampers;
    this.translLms = [tl[0].clone(), tl[1].clone(), tl[2].clone()];
    this.translSds = [ts[0].clone(), ts[1].clone(), ts[2].clone()];
    this.rotLms = [rl[0].clone(), rl[1].clone(), rl[2].clone()];
    this.rotSds = [rs[0].clone(), rs[1].clone(), rs[2].clone()];
  }

  private getInfo(info: JointSolverInfo, timeStep: TimeStep | null, isPositionPart: boolean): void {
    const translMotorMass = 1 / (this.b1.invMass + this.b2.invMass);
    const motorMassX = this.computeEffectiveInertiaMoment(this.axisXX, this.axisXY, this.axisXZ);
    const motorMassY = this.computeEffectiveInertiaMoment(this.axisYX, this.axisYY, this.axisYZ);
    const motorMassZ = this.computeEffectiveInertiaMoment(this.axisZX, this.axisZY, this.axisZZ);
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    // 刚体 1 的约束坐标系
    const xX = this.bX1X, xY = this.bX1Y, xZ = this.bX1Z;
    const yX = this.bY1X, yY = this.bY1Y, yZ = this.bY1Z;
    const zX = this.bZ1X, zY = this.bZ1Y, zZ = this.bZ1Z;

    // 沿三个轴平移：限制、马达、弹簧。有弹簧时位置修正阶段不处理这一行
    let row, j;
    if (this.translSds[0].frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[0]);
      this.setSolverInfoRowLinear(row, this.translationX, this.translLms[0], translMotorMass, this.translSds[0], timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = xX; j.lin1Y = xY; j.lin1Z = xZ;
      j.lin2X = xX; j.lin2Y = xY; j.lin2Z = xZ;
      j.ang1X = r1Y * xZ - r1Z * xY; j.ang1Y = r1Z * xX - r1X * xZ; j.ang1Z = r1X * xY - r1Y * xX;
      j.ang2X = r2Y * xZ - r2Z * xY; j.ang2Y = r2Z * xX - r2X * xZ; j.ang2Z = r2X * xY - r2Y * xX;
    }
    if (this.translSds[1].frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[1]);
      this.setSolverInfoRowLinear(row, this.translationY, this.translLms[1], translMotorMass, this.translSds[1], timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = yX; j.lin1Y = yY; j.lin1Z = yZ;
      j.lin2X = yX; j.lin2Y = yY; j.lin2Z = yZ;
      j.ang1X = r1Y * yZ - r1Z * yY; j.ang1Y = r1Z * yX - r1X * yZ; j.ang1Z = r1X * yY - r1Y * yX;
      j.ang2X = r2Y * yZ - r2Z * yY; j.ang2Y = r2Z * yX - r2X * yZ; j.ang2Z = r2X * yY - r2Y * yX;
    }
    if (this.translSds[2].frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[2]);
      this.setSolverInfoRowLinear(row, this.translationZ, this.translLms[2], translMotorMass, this.translSds[2], timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = zX; j.lin1Y = zY; j.lin1Z = zZ;
      j.lin2X = zX; j.lin2Y = zY; j.lin2Z = zZ;
      j.ang1X = r1Y * zZ - r1Z * zY; j.ang1Y = r1Z * zX - r1X * zZ; j.ang1Z = r1X * zY - r1Y * zX;
      j.ang2X = r2Y * zZ - r2Z * zY; j.ang2Y = r2Z * zX - r2X * zZ; j.ang2Z = r2X * zY - r2Y * zX;
    }

    // 绕欧拉角的三个转轴转动：限制、马达、弹簧
    if (!this.xSingular && (this.rotSds[0].frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.angleX, this.rotLms[0], motorMassX, this.rotSds[0], timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisXX; j.ang1Y = this.axisXY; j.ang1Z = this.axisXZ;
      j.ang2X = this.axisXX; j.ang2Y = this.axisXY; j.ang2Z = this.axisXZ;
    }
    if (!this.ySingular && (this.rotSds[1].frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[4]);
      this.setSolverInfoRowAngular(row, this.angleY, this.rotLms[1], motorMassY, this.rotSds[1], timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisYX; j.ang1Y = this.axisYY; j.ang1Z = this.axisYZ;
      j.ang2X = this.axisYX; j.ang2Y = this.axisYY; j.ang2Z = this.axisYZ;
    }
    if (!this.zSingular && (this.rotSds[2].frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[5]);
      this.setSolverInfoRowAngular(row, this.angleZ, this.rotLms[2], motorMassZ, this.rotSds[2], timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisZX; j.ang1Y = this.axisZY; j.ang1Z = this.axisZZ;
      j.ang2X = this.axisZX; j.ang2Y = this.axisZY; j.ang2Z = this.axisZZ;
    }
  }

  /** 同步锚点和坐标系，然后更新约束方向、欧拉角和位移（原版 _syncAnchors + updateConstraintAxes + computeErrors） */
  override syncAnchors(): void {
    super.syncAnchors();

    // 欧拉角的转轴：X = X1，Z = Z2，Y = Z × X（和万向节一样）。
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
    // 旋转矩阵转 XYZ 欧拉角（原版 mat3_toEulerXyz）。原版的 π/2 是截断的 1.570796326794895
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

    // 两个锚点之间的偏差投影到刚体 1 的约束坐标系
    const dX = this.a2X - this.a1X, dY = this.a2Y - this.a1Y, dZ = this.a2Z - this.a1Z;
    this.translationX = dX * this.bX1X + dY * this.bX1Y + dZ * this.bX1Z;
    this.translationY = dX * this.bY1X + dY * this.bY1Y + dZ * this.bY1Z;
    this.translationZ = dX * this.bZ1X + dY * this.bZ1Y + dZ * this.bZ1Z;
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
