// 移植自 OimoPhysics src/oimo/dynamics/constraint/info/joint/{JointSolverInfo,JointSolverInfoRow}.hx、
// src/oimo/dynamics/constraint/solver/common/JointSolverMassDataRow.hx（MIT，见 Credits.md）
import { MAX_JACOBIAN_ROWS } from "../../../settings.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import { JacobianRow } from "../contact/contact-solver-info.js";
import type { JointImpulse } from "./joint-impulse.js";

/** 关节的一行约束交给求解器的数据 */
export class JointSolverInfoRow {
  readonly jacobian = new JacobianRow();
  /** 目标相对速度（位置修正） */
  rhs = 0;
  /** 约束力混合（软约束） */
  cfm = 0;
  /** 冲量的范围：等式约束是 (-∞, ∞)，限制是单边 */
  minImpulse = 0;
  maxImpulse = 0;
  /** 马达的目标速度和每步最大冲量 */
  motorSpeed = 0;
  motorMaxImpulse = 0;
  impulse: JointImpulse | null = null;

  clear(): void {
    this.jacobian.clear();
    this.rhs = 0;
    this.cfm = 0;
    this.minImpulse = 0;
    this.maxImpulse = 0;
    this.motorSpeed = 0;
    this.motorMaxImpulse = 0;
    this.impulse = null;
  }

  /** 等式约束（两边都不限制冲量） */
  equalLimit(rhs: number, cfm: number): void {
    this.rhs = rhs;
    this.cfm = cfm;
    this.minImpulse = -Infinity;
    this.maxImpulse = Infinity;
  }

  motor(speed: number, maxImpulse: number): void {
    this.motorSpeed = speed;
    this.motorMaxImpulse = maxImpulse;
  }
}

export class JointSolverInfo {
  b1: RigidBody | null = null;
  b2: RigidBody | null = null;
  numRows = 0;
  readonly rows: JointSolverInfoRow[] = [];

  constructor() {
    for (let i = 0; i < MAX_JACOBIAN_ROWS; i++) this.rows.push(new JointSolverInfoRow());
  }

  addRow(impulse: JointImpulse): JointSolverInfoRow {
    const row = this.rows[this.numRows++];
    row.clear();
    row.impulse = impulse;
    return row;
  }
}

/** 一行约束的质量数据：单位冲量引起的速度变化，以及有效质量 */
export class JointSolverMassDataRow {
  invMLin1X = 0; invMLin1Y = 0; invMLin1Z = 0;
  invMLin2X = 0; invMLin2Y = 0; invMLin2Z = 0;
  invMAng1X = 0; invMAng1Y = 0; invMAng1Z = 0;
  invMAng2X = 0; invMAng2Y = 0; invMAng2Z = 0;
  mass = 0;
  massWithoutCfm = 0;
}
