// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/{Joint,JointType,JointMacro}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { ANGULAR_SLOP, LINEAR_SLOP, MIN_SPRING_DAMPER_DAMPING_RATIO, MAX_JACOBIAN_ROWS, VELOCITY_BAUMGARTE, JOINT_WARM_STARTING_FACTOR, JOINT_WARM_STARTING_FACTOR_FOR_BAUMGARTE, DEFAULT_JOINT_POSITION_CORRECTION_ALGORITHM } from "../../../settings.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { TimeStep } from "../../time-step.js";
import type { World } from "../../world.js";
import type { ConstraintSolver } from "../constraint-solver.js";
import { PositionCorrectionAlgorithm } from "../position-correction-algorithm.js";
import { compat } from "../../../compat.js";
import { DirectJointConstraintSolver } from "../solver/direct-joint-constraint-solver.js";
import { PgsJointConstraintSolver } from "../solver/pgs-joint-constraint-solver.js";
import { ConstraintSolverType, type JointConfig } from "./joint-config.js";
import { JointImpulse } from "./joint-impulse.js";
import { JointLink } from "./joint-link.js";
import { ARC, arc, mat3FromQuat, PERP, perp, ROT } from "./joint-math.js";
import type { JointSolverInfo, JointSolverInfoRow } from "./joint-solver-info.js";
import type { RotationalLimitMotor } from "./rotational-limit-motor.js";
import type { SpringDamper } from "./spring-damper.js";
import type { TranslationalLimitMotor } from "./translational-limit-motor.js";

/** 关节类型，编号和原版一致 */
export const JointType = {
  SPHERICAL: 0,
  REVOLUTE: 1,
  CYLINDRICAL: 2,
  PRISMATIC: 3,
  UNIVERSAL: 4,
  RAGDOLL: 5,
  GENERIC: 6,
} as const;

/**
 * 软约束的参数（原版 JointMacro.computeSoftConstraintParameters）：由弹簧频率、阻尼比算出 CFM 系数和 ERP，
 * 写到 SOFT 里（不 new 对象）。CFM = CFM 系数 / 质量
 */
export const SOFT = { cfmFactor: 0, erp: 0 };
export function computeSoftConstraintParameters(frequency: number, dampingRatio: number, dt: number, useSymplecticEuler: boolean): void {
  // 原版的 π 是截断的 3.14159265358979
  const omega = 2 * 3.14159265358979 * frequency;
  let zeta = dampingRatio;
  if (zeta < MIN_SPRING_DAMPER_DAMPING_RATIO) zeta = MIN_SPRING_DAMPER_DAMPING_RATIO;
  const h = dt;
  const c = 2 * zeta * omega;
  const k = omega * omega;
  if (useSymplecticEuler) {
    SOFT.cfmFactor = 1 / (h * c);
    SOFT.erp = k / c;
  } else {
    SOFT.cfmFactor = 1 / (h * (h * k + c));
    SOFT.erp = k / (h * k + c);
  }
}

/** 关节基类：把两个刚体按某种方式连起来 */
export abstract class Joint {
  b1: RigidBody;
  b2: RigidBody;
  readonly link1: JointLink;
  readonly link2: JointLink;
  positionCorrectionAlgorithm: number = DEFAULT_JOINT_POSITION_CORRECTION_ALGORITHM;
  /** 两个刚体是否可以互相碰撞 */
  allowCollision: boolean;
  prev: Joint | null = null;
  next: Joint | null = null;
  world: World | null = null;

  /** 锚点：刚体局部坐标、相对重心的世界方向、世界坐标 */
  la1X: number; la1Y: number; la1Z: number;
  la2X: number; la2Y: number; la2Z: number;
  ra1X = 0; ra1Y = 0; ra1Z = 0;
  ra2X = 0; ra2Y = 0; ra2Z = 0;
  a1X = 0; a1Y = 0; a1Z = 0;
  a2X = 0; a2Y = 0; a2Z = 0;

  /** 关节坐标系：两个刚体上的局部坐标、世界坐标 */
  lbX1X = 0; lbX1Y = 0; lbX1Z = 0;
  lbY1X = 0; lbY1Y = 0; lbY1Z = 0;
  lbZ1X = 0; lbZ1Y = 0; lbZ1Z = 0;
  lbX2X = 0; lbX2Y = 0; lbX2Z = 0;
  lbY2X = 0; lbY2Y = 0; lbY2Z = 0;
  lbZ2X = 0; lbZ2Y = 0; lbZ2Z = 0;
  bX1X = 0; bX1Y = 0; bX1Z = 0;
  bY1X = 0; bY1Y = 0; bY1Z = 0;
  bZ1X = 0; bZ1Y = 0; bZ1Z = 0;
  bX2X = 0; bX2Y = 0; bX2Z = 0;
  bY2X = 0; bY2Y = 0; bY2Z = 0;
  bZ2X = 0; bZ2Y = 0; bZ2Z = 0;

  readonly impulses: JointImpulse[] = [];
  /** 上一步作用在刚体 1 上的约束力、力矩（求解器算出来） */
  appliedForceX = 0; appliedForceY = 0; appliedForceZ = 0;
  appliedTorqueX = 0; appliedTorqueY = 0; appliedTorqueZ = 0;
  /** 约束力、力矩超过它时关节断开，0 表示不会断 */
  breakForce: number;
  breakTorque: number;
  readonly type: number;
  readonly solver: ConstraintSolver;
  userData: unknown = null;

  protected constructor(config: JointConfig, type: number) {
    this.link1 = new JointLink(this);
    this.link2 = new JointLink(this);
    this.type = type;
    if (config.rigidBody1 === null || config.rigidBody2 === null) throw new Error("关节配置缺少刚体");
    this.b1 = config.rigidBody1;
    this.b2 = config.rigidBody2;
    this.allowCollision = config.allowCollision;
    this.breakForce = config.breakForce;
    this.breakTorque = config.breakTorque;
    this.solver = config.solverType === ConstraintSolverType.DIRECT ? new DirectJointConstraintSolver(this) : new PgsJointConstraintSolver(this);
    this.la1X = config.localAnchor1.x; this.la1Y = config.localAnchor1.y; this.la1Z = config.localAnchor1.z;
    this.la2X = config.localAnchor2.x; this.la2Y = config.localAnchor2.y; this.la2Z = config.localAnchor2.z;
    for (let i = 0; i < MAX_JACOBIAN_ROWS; i++) this.impulses.push(new JointImpulse());
  }

  /**
   * 由两个刚体上的局部 X 轴构造两组局部坐标系（原版 buildLocalBasesFromX）：
   * 刚体 1 的 Y、Z 取和 X1 垂直的任意方向，刚体 2 的坐标系是刚体 1 的绕最短弧从 X1 转到 X2
   */
  protected buildLocalBasesFromX(): void {
    if (this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z === 0) {
      this.lbX1X = 1; this.lbX1Y = 0; this.lbX1Z = 0;
    } else {
      let l = this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbX1X *= l; this.lbX1Y *= l; this.lbX1Z *= l;
    }
    if (this.lbX2X * this.lbX2X + this.lbX2Y * this.lbX2Y + this.lbX2Z * this.lbX2Z === 0) {
      this.lbX2X = 1; this.lbX2Y = 0; this.lbX2Z = 0;
    } else {
      let l = this.lbX2X * this.lbX2X + this.lbX2Y * this.lbX2Y + this.lbX2Z * this.lbX2Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbX2X *= l; this.lbX2Y *= l; this.lbX2Z *= l;
    }
    arc(this.lbX1X, this.lbX1Y, this.lbX1Z, this.lbX2X, this.lbX2Y, this.lbX2Z);
    mat3FromQuat(ARC.x, ARC.y, ARC.z, ARC.w);
    perp(this.lbX1X, this.lbX1Y, this.lbX1Z);
    this.lbY1X = PERP.x; this.lbY1Y = PERP.y; this.lbY1Z = PERP.z;
    this.lbZ1X = this.lbX1Y * this.lbY1Z - this.lbX1Z * this.lbY1Y;
    this.lbZ1Y = this.lbX1Z * this.lbY1X - this.lbX1X * this.lbY1Z;
    this.lbZ1Z = this.lbX1X * this.lbY1Y - this.lbX1Y * this.lbY1X;
    const m = ROT;
    this.lbX2X = m.m00 * this.lbX1X + m.m01 * this.lbX1Y + m.m02 * this.lbX1Z;
    this.lbX2Y = m.m10 * this.lbX1X + m.m11 * this.lbX1Y + m.m12 * this.lbX1Z;
    this.lbX2Z = m.m20 * this.lbX1X + m.m21 * this.lbX1Y + m.m22 * this.lbX1Z;
    this.lbY2X = m.m00 * this.lbY1X + m.m01 * this.lbY1Y + m.m02 * this.lbY1Z;
    this.lbY2Y = m.m10 * this.lbY1X + m.m11 * this.lbY1Y + m.m12 * this.lbY1Z;
    this.lbY2Z = m.m20 * this.lbY1X + m.m21 * this.lbY1Y + m.m22 * this.lbY1Z;
    this.lbZ2X = m.m00 * this.lbZ1X + m.m01 * this.lbZ1Y + m.m02 * this.lbZ1Z;
    this.lbZ2Y = m.m10 * this.lbZ1X + m.m11 * this.lbZ1Y + m.m12 * this.lbZ1Z;
    this.lbZ2Z = m.m20 * this.lbZ1X + m.m21 * this.lbZ1Y + m.m22 * this.lbZ1Z;
  }

  /**
   * 由刚体 1 的局部 X 轴、刚体 2 的局部 Z 轴构造两组局部坐标系（原版 buildLocalBasesFromX1Z2，万向节用）：
   * 按刚体当前的朝向算出世界坐标的 Y = Z2 × X1（平行时取和 X1 垂直的方向），再补齐两组坐标系
   */
  protected buildLocalBasesFromX1Z2(): void {
    if (this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z === 0) {
      this.lbX1X = 1; this.lbX1Y = 0; this.lbX1Z = 0;
    } else {
      let l = this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbX1X *= l; this.lbX1Y *= l; this.lbX1Z *= l;
    }
    if (this.lbZ2X * this.lbZ2X + this.lbZ2Y * this.lbZ2Y + this.lbZ2Z * this.lbZ2Z === 0) {
      this.lbZ2X = 0; this.lbZ2Y = 0; this.lbZ2Z = 1;
    } else {
      let l = this.lbZ2X * this.lbZ2X + this.lbZ2Y * this.lbZ2Y + this.lbZ2Z * this.lbZ2Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbZ2X *= l; this.lbZ2Y *= l; this.lbZ2Z *= l;
    }
    const tf1 = this.b1.transform, tf2 = this.b2.transform;
    // 世界坐标的 X1、Z2、Y = Z2 × X1
    const x1X = tf1.r00 * this.lbX1X + tf1.r01 * this.lbX1Y + tf1.r02 * this.lbX1Z;
    const x1Y = tf1.r10 * this.lbX1X + tf1.r11 * this.lbX1Y + tf1.r12 * this.lbX1Z;
    const x1Z = tf1.r20 * this.lbX1X + tf1.r21 * this.lbX1Y + tf1.r22 * this.lbX1Z;
    const z2X = tf2.r00 * this.lbZ2X + tf2.r01 * this.lbZ2Y + tf2.r02 * this.lbZ2Z;
    const z2Y = tf2.r10 * this.lbZ2X + tf2.r11 * this.lbZ2Y + tf2.r12 * this.lbZ2Z;
    const z2Z = tf2.r20 * this.lbZ2X + tf2.r21 * this.lbZ2Y + tf2.r22 * this.lbZ2Z;
    let yX = z2Y * x1Z - z2Z * x1Y, yY = z2Z * x1X - z2X * x1Z, yZ = z2X * x1Y - z2Y * x1X;
    if (yX * yX + yY * yY + yZ * yZ === 0) {
      perp(x1X, x1Y, x1Z);
      yX = PERP.x; yY = PERP.y; yZ = PERP.z;
    }
    // Z1 = X1 × Y，X2 = Y × Z2
    const z1X = x1Y * yZ - x1Z * yY, z1Y = x1Z * yX - x1X * yZ, z1Z = x1X * yY - x1Y * yX;
    const x2X = yY * z2Z - yZ * z2Y, x2Y = yZ * z2X - yX * z2Z, x2Z = yX * z2Y - yY * z2X;
    // 转回刚体的局部坐标（乘旋转矩阵的转置）
    this.lbX1X = tf1.r00 * x1X + tf1.r10 * x1Y + tf1.r20 * x1Z;
    this.lbX1Y = tf1.r01 * x1X + tf1.r11 * x1Y + tf1.r21 * x1Z;
    this.lbX1Z = tf1.r02 * x1X + tf1.r12 * x1Y + tf1.r22 * x1Z;
    this.lbY1X = tf1.r00 * yX + tf1.r10 * yY + tf1.r20 * yZ;
    this.lbY1Y = tf1.r01 * yX + tf1.r11 * yY + tf1.r21 * yZ;
    this.lbY1Z = tf1.r02 * yX + tf1.r12 * yY + tf1.r22 * yZ;
    this.lbZ1X = tf1.r00 * z1X + tf1.r10 * z1Y + tf1.r20 * z1Z;
    this.lbZ1Y = tf1.r01 * z1X + tf1.r11 * z1Y + tf1.r21 * z1Z;
    this.lbZ1Z = tf1.r02 * z1X + tf1.r12 * z1Y + tf1.r22 * z1Z;
    this.lbX2X = tf2.r00 * x2X + tf2.r10 * x2Y + tf2.r20 * x2Z;
    this.lbX2Y = tf2.r01 * x2X + tf2.r11 * x2Y + tf2.r21 * x2Z;
    this.lbX2Z = tf2.r02 * x2X + tf2.r12 * x2Y + tf2.r22 * x2Z;
    this.lbY2X = tf2.r00 * yX + tf2.r10 * yY + tf2.r20 * yZ;
    this.lbY2Y = tf2.r01 * yX + tf2.r11 * yY + tf2.r21 * yZ;
    this.lbY2Z = tf2.r02 * yX + tf2.r12 * yY + tf2.r22 * yZ;
    this.lbZ2X = tf2.r00 * z2X + tf2.r10 * z2Y + tf2.r20 * z2Z;
    this.lbZ2Y = tf2.r01 * z2X + tf2.r11 * z2Y + tf2.r21 * z2Z;
    this.lbZ2Z = tf2.r02 * z2X + tf2.r12 * z2Y + tf2.r22 * z2Z;
  }

  /**
   * 由局部 X1、Y1 和 X2 构造关节坐标系。
   * Y1 与 X1 正交化，两组坐标系通过最短弧关联；X2 归一化，零向量取 (1, 0, 0)。
   */
  protected buildLocalBasesFromXY1X2(): void {
    if (this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z === 0) {
      this.lbX1X = 1; this.lbX1Y = 0; this.lbX1Z = 0;
    } else {
      let l = this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbX1X *= l; this.lbX1Y *= l; this.lbX1Z *= l;
    }
    // Z1 = X1 × Y1，再用 Y1 = Z1 × X1 把 Y1 正交化
    this.lbZ1X = this.lbX1Y * this.lbY1Z - this.lbX1Z * this.lbY1Y;
    this.lbZ1Y = this.lbX1Z * this.lbY1X - this.lbX1X * this.lbY1Z;
    this.lbZ1Z = this.lbX1X * this.lbY1Y - this.lbX1Y * this.lbY1X;
    if (this.lbZ1X * this.lbZ1X + this.lbZ1Y * this.lbZ1Y + this.lbZ1Z * this.lbZ1Z === 0) {
      perp(this.lbX1X, this.lbX1Y, this.lbX1Z);
      this.lbY1X = PERP.x; this.lbY1Y = PERP.y; this.lbY1Z = PERP.z;
      this.lbZ1X = this.lbX1Y * this.lbY1Z - this.lbX1Z * this.lbY1Y;
      this.lbZ1Y = this.lbX1Z * this.lbY1X - this.lbX1X * this.lbY1Z;
      this.lbZ1Z = this.lbX1X * this.lbY1Y - this.lbX1Y * this.lbY1X;
    } else {
      let l = this.lbZ1X * this.lbZ1X + this.lbZ1Y * this.lbZ1Y + this.lbZ1Z * this.lbZ1Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbZ1X *= l; this.lbZ1Y *= l; this.lbZ1Z *= l;
      this.lbY1X = this.lbZ1Y * this.lbX1Z - this.lbZ1Z * this.lbX1Y;
      this.lbY1Y = this.lbZ1Z * this.lbX1X - this.lbZ1X * this.lbX1Z;
      this.lbY1Z = this.lbZ1X * this.lbX1Y - this.lbZ1Y * this.lbX1X;
    }
    if (!compat.keepOimoBugs) {
      if (this.lbX2X * this.lbX2X + this.lbX2Y * this.lbX2Y + this.lbX2Z * this.lbX2Z === 0) {
        this.lbX2X = 1; this.lbX2Y = 0; this.lbX2Z = 0;
      } else {
        let l = this.lbX2X * this.lbX2X + this.lbX2Y * this.lbX2Y + this.lbX2Z * this.lbX2Z;
        if (l > 0) l = 1 / Math.sqrt(l);
        this.lbX2X *= l; this.lbX2Y *= l; this.lbX2Z *= l;
      }
    }
    arc(this.lbX1X, this.lbX1Y, this.lbX1Z, this.lbX2X, this.lbX2Y, this.lbX2Z);
    mat3FromQuat(ARC.x, ARC.y, ARC.z, ARC.w);
    const m = ROT;
    this.lbX2X = m.m00 * this.lbX1X + m.m01 * this.lbX1Y + m.m02 * this.lbX1Z;
    this.lbX2Y = m.m10 * this.lbX1X + m.m11 * this.lbX1Y + m.m12 * this.lbX1Z;
    this.lbX2Z = m.m20 * this.lbX1X + m.m21 * this.lbX1Y + m.m22 * this.lbX1Z;
    this.lbY2X = m.m00 * this.lbY1X + m.m01 * this.lbY1Y + m.m02 * this.lbY1Z;
    this.lbY2Y = m.m10 * this.lbY1X + m.m11 * this.lbY1Y + m.m12 * this.lbY1Z;
    this.lbY2Z = m.m20 * this.lbY1X + m.m21 * this.lbY1Y + m.m22 * this.lbY1Z;
    this.lbZ2X = m.m00 * this.lbZ1X + m.m01 * this.lbZ1Y + m.m02 * this.lbZ1Z;
    this.lbZ2Y = m.m10 * this.lbZ1X + m.m11 * this.lbZ1Y + m.m12 * this.lbZ1Z;
    this.lbZ2Z = m.m20 * this.lbZ1X + m.m21 * this.lbZ1Y + m.m22 * this.lbZ1Z;
  }

  /**
   * 填一行平移方向的约束（原版 setSolverInfoRowLinear）：限制、马达、弹簧。
   * diff 是当前位移；mass 是沿这个方向的有效质量（马达和弹簧用）
   */
  protected setSolverInfoRowLinear(
    row: JointSolverInfoRow, diff: number, lm: TranslationalLimitMotor, mass: number, sd: SpringDamper,
    timeStep: TimeStep | null, isPositionPart: boolean,
  ): void {
    let cfmFactor: number, erp: number;
    let slop = LINEAR_SLOP;
    if (isPositionPart) {
      cfmFactor = 0;
      erp = 1;
    } else {
      if (sd.frequency > 0) {
        // 软约束：不留容差
        slop = 0;
        computeSoftConstraintParameters(sd.frequency, sd.dampingRatio, timeStep!.dt, sd.useSymplecticEuler);
        cfmFactor = SOFT.cfmFactor;
        erp = SOFT.erp;
      } else {
        cfmFactor = 0;
        erp = this.getErp(timeStep, false);
      }
      if (lm.motorForce > 0) row.motor(lm.motorSpeed, lm.motorForce * timeStep!.dt);
      else row.motor(0, 0);
    }
    const lower = lm.lowerLimit, upper = lm.upperLimit;
    let minImp: number, maxImp: number, error: number;
    if (lower > upper) {
      // 不限制
      minImp = 0; maxImp = 0; error = 0;
    } else if (lower === upper) {
      // 锁死
      minImp = -Infinity; maxImp = Infinity; error = diff - lower;
    } else if (diff < lower) {
      // 超过下限
      minImp = -Infinity; maxImp = 0;
      error = diff - lower + slop;
      if (error > 0) error = 0;
    } else if (diff > upper) {
      // 超过上限
      minImp = 0; maxImp = Infinity;
      error = diff - upper - slop;
      if (error < 0) error = 0;
    } else {
      minImp = 0; maxImp = 0; error = 0;
    }
    row.minImpulse = minImp;
    row.maxImpulse = maxImp;
    row.cfm = cfmFactor * (mass === 0 ? 0 : 1 / mass);
    row.rhs = error * erp;
  }

  /**
   * 填一行转动方向的约束（原版 setSolverInfoRowAngular）：限制、马达、弹簧。
   * diff 是当前转角，先换算到 [限制中点 - π, 限制中点 + π]；mass 是绕这个轴的有效转动惯量（马达和弹簧用）
   */
  protected setSolverInfoRowAngular(
    row: JointSolverInfoRow, diff: number, lm: RotationalLimitMotor, mass: number, sd: SpringDamper,
    timeStep: TimeStep | null, isPositionPart: boolean,
  ): void {
    let cfmFactor: number, erp: number;
    let slop = ANGULAR_SLOP;
    if (isPositionPart) {
      cfmFactor = 0;
      erp = 1;
    } else {
      if (sd.frequency > 0) {
        // 软约束：不留容差
        slop = 0;
        computeSoftConstraintParameters(sd.frequency, sd.dampingRatio, timeStep!.dt, sd.useSymplecticEuler);
        cfmFactor = SOFT.cfmFactor;
        erp = SOFT.erp;
      } else {
        cfmFactor = 0;
        erp = this.getErp(timeStep, false);
      }
      if (lm.motorTorque > 0) row.motor(lm.motorSpeed, lm.motorTorque * timeStep!.dt);
      else row.motor(0, 0);
    }
    const lower = lm.lowerLimit, upper = lm.upperLimit;
    // 原版的 π、2π 是截断的 3.14159265358979、6.28318530717958
    const mid = (lower + upper) * 0.5;
    diff -= mid;
    diff = ((diff + 3.14159265358979) % 6.28318530717958 + 6.28318530717958) % 6.28318530717958 - 3.14159265358979;
    diff += mid;
    let minImp: number, maxImp: number, error: number;
    if (lower > upper) {
      // 不限制
      minImp = 0; maxImp = 0; error = 0;
    } else if (lower === upper) {
      // 锁死
      minImp = -Infinity; maxImp = Infinity; error = diff - lower;
    } else if (diff < lower) {
      // 超过下限
      minImp = -Infinity; maxImp = 0;
      error = diff - lower + slop;
      if (error > 0) error = 0;
    } else if (diff > upper) {
      // 超过上限
      minImp = 0; maxImp = Infinity;
      error = diff - upper - slop;
      if (error < 0) error = 0;
    } else {
      minImp = 0; maxImp = 0; error = 0;
    }
    row.minImpulse = minImp;
    row.maxImpulse = maxImp;
    row.cfm = cfmFactor * (mass === 0 ? 0 : 1 / mass);
    row.rhs = error * erp;
  }

  /** 位置修正的 ERP：Baumgarte 时是 VELOCITY_BAUMGARTE / dt，否则 0（位置修正阶段总是 1） */
  protected getErp(timeStep: TimeStep | null, isPositionPart: boolean): number {
    if (isPositionPart) return 1;
    if (this.positionCorrectionAlgorithm === PositionCorrectionAlgorithm.BAUMGARTE) return timeStep!.invDt * VELOCITY_BAUMGARTE;
    return 0;
  }

  /**
   * 两个刚体绕世界坐标的单位轴 axis 转动的有效转动惯量（原版 computeEffectiveInertiaMoment）。
   * 锚点不在重心上时，把绕锚点转动需要的平动也算进去
   */
  protected computeEffectiveInertiaMoment(axisX: number, axisY: number, axisZ: number): number {
    const b1 = this.b1, b2 = this.b2;
    const ia1X = b1.ii00 * axisX + b1.ii01 * axisY + b1.ii02 * axisZ;
    const ia1Y = b1.ii10 * axisX + b1.ii11 * axisY + b1.ii12 * axisZ;
    const ia1Z = b1.ii20 * axisX + b1.ii21 * axisY + b1.ii22 * axisZ;
    const ia2X = b2.ii00 * axisX + b2.ii01 * axisY + b2.ii02 * axisZ;
    const ia2Y = b2.ii10 * axisX + b2.ii11 * axisY + b2.ii12 * axisZ;
    const ia2Z = b2.ii20 * axisX + b2.ii21 * axisY + b2.ii22 * axisZ;
    let invI1 = ia1X * axisX + ia1Y * axisY + ia1Z * axisZ;
    let invI2 = ia2X * axisX + ia2Y * axisY + ia2Z * axisZ;
    if (invI1 > 0) {
      const dot = axisX * this.ra1X + axisY * this.ra1Y + axisZ * this.ra1Z;
      const projsq = this.ra1X * this.ra1X + this.ra1Y * this.ra1Y + this.ra1Z * this.ra1Z - dot * dot;
      if (projsq > 0) {
        if (b1.invMass > 0) invI1 = 1 / (1 / invI1 + b1.mass * projsq);
        else invI1 = 0;
      }
    }
    if (invI2 > 0) {
      const dot = axisX * this.ra2X + axisY * this.ra2Y + axisZ * this.ra2Z;
      const projsq = this.ra2X * this.ra2X + this.ra2Y * this.ra2Y + this.ra2Z * this.ra2Z - dot * dot;
      if (projsq > 0) {
        if (b2.invMass > 0) invI2 = 1 / (1 / invI2 + b2.mass * projsq);
        else invI2 = 0;
      }
    }
    return invI1 + invI2 === 0 ? 0 : 1 / (invI1 + invI2);
  }

  /** 热启动时上一步冲量保留的比例（原版 _getWarmStartingFactor） */
  getWarmStartingFactor(): number {
    return this.positionCorrectionAlgorithm === PositionCorrectionAlgorithm.BAUMGARTE
      ? JOINT_WARM_STARTING_FACTOR_FOR_BAUMGARTE
      : JOINT_WARM_STARTING_FACTOR;
  }

  /** 按当前变换更新锚点及约束坐标系，供约束求解使用。 */
  syncAnchors(): void {
    const tf1 = this.b1.transform, tf2 = this.b2.transform;
    this.ra1X = tf1.r00 * this.la1X + tf1.r01 * this.la1Y + tf1.r02 * this.la1Z;
    this.ra1Y = tf1.r10 * this.la1X + tf1.r11 * this.la1Y + tf1.r12 * this.la1Z;
    this.ra1Z = tf1.r20 * this.la1X + tf1.r21 * this.la1Y + tf1.r22 * this.la1Z;
    this.ra2X = tf2.r00 * this.la2X + tf2.r01 * this.la2Y + tf2.r02 * this.la2Z;
    this.ra2Y = tf2.r10 * this.la2X + tf2.r11 * this.la2Y + tf2.r12 * this.la2Z;
    this.ra2Z = tf2.r20 * this.la2X + tf2.r21 * this.la2Y + tf2.r22 * this.la2Z;
    this.a1X = this.ra1X + tf1.px; this.a1Y = this.ra1Y + tf1.py; this.a1Z = this.ra1Z + tf1.pz;
    this.a2X = this.ra2X + tf2.px; this.a2Y = this.ra2Y + tf2.py; this.a2Z = this.ra2Z + tf2.pz;
    this.bX1X = tf1.r00 * this.lbX1X + tf1.r01 * this.lbX1Y + tf1.r02 * this.lbX1Z;
    this.bX1Y = tf1.r10 * this.lbX1X + tf1.r11 * this.lbX1Y + tf1.r12 * this.lbX1Z;
    this.bX1Z = tf1.r20 * this.lbX1X + tf1.r21 * this.lbX1Y + tf1.r22 * this.lbX1Z;
    this.bY1X = tf1.r00 * this.lbY1X + tf1.r01 * this.lbY1Y + tf1.r02 * this.lbY1Z;
    this.bY1Y = tf1.r10 * this.lbY1X + tf1.r11 * this.lbY1Y + tf1.r12 * this.lbY1Z;
    this.bY1Z = tf1.r20 * this.lbY1X + tf1.r21 * this.lbY1Y + tf1.r22 * this.lbY1Z;
    this.bZ1X = tf1.r00 * this.lbZ1X + tf1.r01 * this.lbZ1Y + tf1.r02 * this.lbZ1Z;
    this.bZ1Y = tf1.r10 * this.lbZ1X + tf1.r11 * this.lbZ1Y + tf1.r12 * this.lbZ1Z;
    this.bZ1Z = tf1.r20 * this.lbZ1X + tf1.r21 * this.lbZ1Y + tf1.r22 * this.lbZ1Z;
    this.bX2X = tf2.r00 * this.lbX2X + tf2.r01 * this.lbX2Y + tf2.r02 * this.lbX2Z;
    this.bX2Y = tf2.r10 * this.lbX2X + tf2.r11 * this.lbX2Y + tf2.r12 * this.lbX2Z;
    this.bX2Z = tf2.r20 * this.lbX2X + tf2.r21 * this.lbX2Y + tf2.r22 * this.lbX2Z;
    this.bY2X = tf2.r00 * this.lbY2X + tf2.r01 * this.lbY2Y + tf2.r02 * this.lbY2Z;
    this.bY2Y = tf2.r10 * this.lbY2X + tf2.r11 * this.lbY2Y + tf2.r12 * this.lbY2Z;
    this.bY2Z = tf2.r20 * this.lbY2X + tf2.r21 * this.lbY2Y + tf2.r22 * this.lbY2Z;
    this.bZ2X = tf2.r00 * this.lbZ2X + tf2.r01 * this.lbZ2Y + tf2.r02 * this.lbZ2Z;
    this.bZ2Y = tf2.r10 * this.lbZ2X + tf2.r11 * this.lbZ2Y + tf2.r12 * this.lbZ2Z;
    this.bZ2Z = tf2.r20 * this.lbZ2X + tf2.r21 * this.lbZ2Y + tf2.r22 * this.lbZ2Z;
  }

  /** 初始化速度约束行，子类随后追加各自由度的约束。 */
  getVelocitySolverInfo(_timeStep: TimeStep, info: JointSolverInfo): void {
    info.b1 = this.b1;
    info.b2 = this.b2;
    info.numRows = 0;
  }

  /** 位置修正要用的约束行（原版 _getPositionSolverInfo） */
  getPositionSolverInfo(info: JointSolverInfo): void {
    info.b1 = this.b1;
    info.b2 = this.b2;
    info.numRows = 0;
  }

  /** 约束力或力矩超过阈值时把关节从世界里删掉（原版 _checkDestruction） */
  checkDestruction(): void {
    const forceSq = this.appliedForceX * this.appliedForceX + this.appliedForceY * this.appliedForceY + this.appliedForceZ * this.appliedForceZ;
    const torqueSq = this.appliedTorqueX * this.appliedTorqueX + this.appliedTorqueY * this.appliedTorqueY + this.appliedTorqueZ * this.appliedTorqueZ;
    if (this.breakForce > 0 && forceSq > this.breakForce * this.breakForce) {
      this.world!.removeJoint(this);
      return;
    }
    if (this.breakTorque > 0 && torqueSq > this.breakTorque * this.breakTorque) {
      this.world!.removeJoint(this);
      return;
    }
  }

  /** 挂到两个刚体的关节链表上，并唤醒它们（原版 _attachLinks） */
  attachLinks(): void {
    const b1 = this.b1, b2 = this.b2, l1 = this.link1, l2 = this.link2;
    l1.other = b2;
    l2.other = b1;
    if (b1.jointLinkList === null) {
      b1.jointLinkList = l1;
      b1.jointLinkListLast = l1;
    } else {
      b1.jointLinkListLast!.next = l1;
      l1.prev = b1.jointLinkListLast;
      b1.jointLinkListLast = l1;
    }
    if (b2.jointLinkList === null) {
      b2.jointLinkList = l2;
      b2.jointLinkListLast = l2;
    } else {
      b2.jointLinkListLast!.next = l2;
      l2.prev = b2.jointLinkListLast;
      b2.jointLinkListLast = l2;
    }
    b1.numJointLinks++;
    b2.numJointLinks++;
    b1.wakeUp();
    b2.wakeUp();
  }

  /** 从两个刚体的关节链表上摘下来，并唤醒它们（原版 _detachLinks） */
  detachLinks(): void {
    const b1 = this.b1, b2 = this.b2;
    removeLink(b1, this.link1);
    removeLink(b2, this.link2);
    this.link1.other = null;
    this.link2.other = null;
    b1.numJointLinks--;
    b2.numJointLinks--;
    b1.wakeUp();
    b2.wakeUp();
  }
}

function removeLink(b: RigidBody, l: JointLink): void {
  const prev = l.prev, next = l.next;
  if (prev !== null) prev.next = next;
  if (next !== null) next.prev = prev;
  if (l === b.jointLinkList) b.jointLinkList = b.jointLinkList.next;
  if (l === b.jointLinkListLast) b.jointLinkListLast = b.jointLinkListLast.prev;
  l.next = null;
  l.prev = null;
}
