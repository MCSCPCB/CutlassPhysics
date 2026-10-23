// 移植自 OimoPhysics src/oimo/dynamics/constraint/solver/pgs/PgsJointConstraintSolver.hx（MIT，见 Credits.md）
// 速度和位置求解使用展开的标量运算。
import { MAX_JACOBIAN_ROWS, POSITION_NGS_BAUMGARTE, POSITION_SPLIT_IMPULSE_BAUMGARTE } from "../../../settings.js";
import type { TimeStep } from "../../time-step.js";
import { ConstraintSolver } from "../constraint-solver.js";
import type { Joint } from "../joint/joint.js";
import { JointSolverInfo, JointSolverMassDataRow } from "../joint/joint-solver-info.js";

/** 关节约束的投影高斯-赛德尔（顺序冲量）求解器 */
export class PgsJointConstraintSolver extends ConstraintSolver {
  private readonly joint: Joint;
  private readonly info = new JointSolverInfo();
  private readonly massData: JointSolverMassDataRow[] = [];

  constructor(joint: Joint) {
    super();
    this.joint = joint;
    for (let i = 0; i < MAX_JACOBIAN_ROWS; i++) this.massData.push(new JointSolverMassDataRow());
  }

  preSolveVelocity(timeStep: TimeStep): void {
    this.joint.syncAnchors();
    this.joint.getVelocitySolverInfo(timeStep, this.info);
    const info = this.info;
    const b1 = (this.b1 = info.b1!), b2 = (this.b2 = info.b2!);
    const invM1 = b1.invMass, invM2 = b2.invMass;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], j = row.jacobian;
      j.updateSparsity();
      if (j.isLinearSet()) {
        md.invMLin1X = j.lin1X * invM1; md.invMLin1Y = j.lin1Y * invM1; md.invMLin1Z = j.lin1Z * invM1;
        md.invMLin2X = j.lin2X * invM2; md.invMLin2Y = j.lin2Y * invM2; md.invMLin2Z = j.lin2Z * invM2;
      } else {
        md.invMLin1X = 0; md.invMLin1Y = 0; md.invMLin1Z = 0;
        md.invMLin2X = 0; md.invMLin2Y = 0; md.invMLin2Z = 0;
      }
      if (j.isAngularSet()) {
        md.invMAng1X = b1.ii00 * j.ang1X + b1.ii01 * j.ang1Y + b1.ii02 * j.ang1Z;
        md.invMAng1Y = b1.ii10 * j.ang1X + b1.ii11 * j.ang1Y + b1.ii12 * j.ang1Z;
        md.invMAng1Z = b1.ii20 * j.ang1X + b1.ii21 * j.ang1Y + b1.ii22 * j.ang1Z;
        md.invMAng2X = b2.ii00 * j.ang2X + b2.ii01 * j.ang2Y + b2.ii02 * j.ang2Z;
        md.invMAng2Y = b2.ii10 * j.ang2X + b2.ii11 * j.ang2Y + b2.ii12 * j.ang2Z;
        md.invMAng2Z = b2.ii20 * j.ang2X + b2.ii21 * j.ang2Y + b2.ii22 * j.ang2Z;
      } else {
        md.invMAng1X = 0; md.invMAng1Y = 0; md.invMAng1Z = 0;
        md.invMAng2X = 0; md.invMAng2Y = 0; md.invMAng2Z = 0;
      }
      // 求和顺序：第一项 + (第二项) + (第三项) + (第四项)
      md.massWithoutCfm =
        md.invMLin1X * j.lin1X + md.invMLin1Y * j.lin1Y + md.invMLin1Z * j.lin1Z +
        (md.invMLin2X * j.lin2X + md.invMLin2Y * j.lin2Y + md.invMLin2Z * j.lin2Z) +
        (md.invMAng1X * j.ang1X + md.invMAng1Y * j.ang1Y + md.invMAng1Z * j.ang1Z) +
        (md.invMAng2X * j.ang2X + md.invMAng2Y * j.ang2Y + md.invMAng2Z * j.ang2Z);
      md.mass = md.massWithoutCfm + row.cfm;
      if (md.massWithoutCfm !== 0) md.massWithoutCfm = 1 / md.massWithoutCfm;
      if (md.mass !== 0) md.mass = 1 / md.mass;
    }
  }

  warmStart(timeStep: TimeStep): void {
    const info = this.info;
    let factor = this.joint.getWarmStartingFactor();
    // 变步长时按比例缩放
    factor *= timeStep.dtRatio;
    if (factor <= 0) {
      for (let i = 0; i < info.numRows; i++) info.rows[i].impulse!.clear();
      return;
    }
    const b1 = this.b1!, b2 = this.b2!;
    let lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz;
    let lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    let av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz;
    let av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    for (let i = 0; i < info.numRows; i++) {
      const md = this.massData[i], imp = info.rows[i].impulse!;
      imp.impulse *= factor;
      imp.impulseM *= factor;
      const impulse = imp.impulse + imp.impulseM;
      lv1X += md.invMLin1X * impulse; lv1Y += md.invMLin1Y * impulse; lv1Z += md.invMLin1Z * impulse;
      lv2X += md.invMLin2X * -impulse; lv2Y += md.invMLin2Y * -impulse; lv2Z += md.invMLin2Z * -impulse;
      av1X += md.invMAng1X * impulse; av1Y += md.invMAng1Y * impulse; av1Z += md.invMAng1Z * impulse;
      av2X += md.invMAng2X * -impulse; av2Y += md.invMAng2Y * -impulse; av2Z += md.invMAng2Z * -impulse;
    }
    b1.vx = lv1X; b1.vy = lv1Y; b1.vz = lv1Z;
    b2.vx = lv2X; b2.vy = lv2Y; b2.vz = lv2Z;
    b1.wx = av1X; b1.wy = av1Y; b1.wz = av1Z;
    b2.wx = av2X; b2.wy = av2Y; b2.wz = av2Z;
  }

  solveVelocity(): void {
    const info = this.info, b1 = this.b1!, b2 = this.b2!;
    let lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz;
    let lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    let av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz;
    let av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;

    // 先解马达
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse!, j = row.jacobian;
      if (row.motorMaxImpulse === 0) continue;
      let rv = 0;
      rv += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rv -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rv += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rv -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseM = (-row.motorSpeed - rv) * md.massWithoutCfm;
      const oldImpulseM = imp.impulseM;
      imp.impulseM += impulseM;
      if (imp.impulseM < -row.motorMaxImpulse) imp.impulseM = -row.motorMaxImpulse;
      else if (imp.impulseM > row.motorMaxImpulse) imp.impulseM = row.motorMaxImpulse;
      impulseM = imp.impulseM - oldImpulseM;
      if (j.isLinearSet()) {
        lv1X += md.invMLin1X * impulseM; lv1Y += md.invMLin1Y * impulseM; lv1Z += md.invMLin1Z * impulseM;
        lv2X += md.invMLin2X * -impulseM; lv2Y += md.invMLin2Y * -impulseM; lv2Z += md.invMLin2Z * -impulseM;
      }
      if (j.isAngularSet()) {
        av1X += md.invMAng1X * impulseM; av1Y += md.invMAng1Y * impulseM; av1Z += md.invMAng1Z * impulseM;
        av2X += md.invMAng2X * -impulseM; av2Y += md.invMAng2Y * -impulseM; av2Z += md.invMAng2Z * -impulseM;
      }
    }

    // 再解约束本身
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse!, j = row.jacobian;
      let rv = 0;
      rv += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rv -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rv += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rv -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulse = (row.rhs - rv - imp.impulse * row.cfm) * md.mass;
      const oldImpulse = imp.impulse;
      imp.impulse += impulse;
      if (imp.impulse < row.minImpulse) imp.impulse = row.minImpulse;
      else if (imp.impulse > row.maxImpulse) imp.impulse = row.maxImpulse;
      impulse = imp.impulse - oldImpulse;
      if (j.isLinearSet()) {
        lv1X += md.invMLin1X * impulse; lv1Y += md.invMLin1Y * impulse; lv1Z += md.invMLin1Z * impulse;
        lv2X += md.invMLin2X * -impulse; lv2Y += md.invMLin2Y * -impulse; lv2Z += md.invMLin2Z * -impulse;
      }
      if (j.isAngularSet()) {
        av1X += md.invMAng1X * impulse; av1Y += md.invMAng1Y * impulse; av1Z += md.invMAng1Z * impulse;
        av2X += md.invMAng2X * -impulse; av2Y += md.invMAng2Y * -impulse; av2Z += md.invMAng2Z * -impulse;
      }
    }
    b1.vx = lv1X; b1.vy = lv1Y; b1.vz = lv1Z;
    b2.vx = lv2X; b2.vy = lv2Y; b2.vz = lv2Z;
    b1.wx = av1X; b1.wy = av1Y; b1.wz = av1Z;
    b2.wx = av2X; b2.wy = av2Y; b2.wz = av2Z;
  }

  /** 由累积冲量算出这一步的约束力、力矩（断开判断用） */
  postSolveVelocity(timeStep: TimeStep): void {
    const info = this.info;
    let linX = 0, linY = 0, linZ = 0, angX = 0, angY = 0, angZ = 0;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse!, j = row.jacobian;
      if (j.isLinearSet()) {
        // 假定这一行是平移约束
        linX += j.lin1X * imp.impulse; linY += j.lin1Y * imp.impulse; linZ += j.lin1Z * imp.impulse;
      } else if (j.isAngularSet()) {
        // 假定这一行是旋转约束
        angX += j.ang1X * imp.impulse; angY += j.ang1Y * imp.impulse; angZ += j.ang1Z * imp.impulse;
      }
    }
    const joint = this.joint, invDt = timeStep.invDt;
    joint.appliedForceX = linX * invDt; joint.appliedForceY = linY * invDt; joint.appliedForceZ = linZ * invDt;
    joint.appliedTorqueX = angX * invDt; joint.appliedTorqueY = angY * invDt; joint.appliedTorqueZ = angZ * invDt;
  }

  /** 位置修正阶段的约束行和有效质量（不含 CFM） */
  private updatePositionData(): void {
    this.joint.syncAnchors();
    this.joint.getPositionSolverInfo(this.info);
    const info = this.info;
    const b1 = (this.b1 = info.b1!), b2 = (this.b2 = info.b2!);
    const invM1 = b1.invMass, invM2 = b2.invMass;
    for (let i = 0; i < info.numRows; i++) {
      const md = this.massData[i], j = info.rows[i].jacobian;
      md.invMLin1X = j.lin1X * invM1; md.invMLin1Y = j.lin1Y * invM1; md.invMLin1Z = j.lin1Z * invM1;
      md.invMLin2X = j.lin2X * invM2; md.invMLin2Y = j.lin2Y * invM2; md.invMLin2Z = j.lin2Z * invM2;
      md.invMAng1X = b1.ii00 * j.ang1X + b1.ii01 * j.ang1Y + b1.ii02 * j.ang1Z;
      md.invMAng1Y = b1.ii10 * j.ang1X + b1.ii11 * j.ang1Y + b1.ii12 * j.ang1Z;
      md.invMAng1Z = b1.ii20 * j.ang1X + b1.ii21 * j.ang1Y + b1.ii22 * j.ang1Z;
      md.invMAng2X = b2.ii00 * j.ang2X + b2.ii01 * j.ang2Y + b2.ii02 * j.ang2Z;
      md.invMAng2Y = b2.ii10 * j.ang2X + b2.ii11 * j.ang2Y + b2.ii12 * j.ang2Z;
      md.invMAng2Z = b2.ii20 * j.ang2X + b2.ii21 * j.ang2Y + b2.ii22 * j.ang2Z;
      md.mass =
        md.invMLin1X * j.lin1X + md.invMLin1Y * j.lin1Y + md.invMLin1Z * j.lin1Z +
        (md.invMLin2X * j.lin2X + md.invMLin2Y * j.lin2Y + md.invMLin2Z * j.lin2Z) +
        (md.invMAng1X * j.ang1X + md.invMAng1Y * j.ang1Y + md.invMAng1Z * j.ang1Z) +
        (md.invMAng2X * j.ang2X + md.invMAng2Y * j.ang2Y + md.invMAng2Z * j.ang2Z);
      if (md.mass !== 0) md.mass = 1 / md.mass;
    }
  }

  preSolvePosition(): void {
    this.updatePositionData();
    for (let i = 0; i < this.info.numRows; i++) this.info.rows[i].impulse!.impulseP = 0;
  }

  solvePositionSplitImpulse(): void {
    const info = this.info, b1 = this.b1!, b2 = this.b2!;
    let lv1X = b1.pvx, lv1Y = b1.pvy, lv1Z = b1.pvz;
    let lv2X = b2.pvx, lv2Y = b2.pvy, lv2Z = b2.pvz;
    let av1X = b1.pwx, av1Y = b1.pwy, av1Z = b1.pwz;
    let av2X = b2.pwx, av2Y = b2.pwy, av2Z = b2.pwz;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse!, j = row.jacobian;
      let rv = 0;
      rv += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rv -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rv += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rv -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseP = (row.rhs * POSITION_SPLIT_IMPULSE_BAUMGARTE - rv) * md.mass;
      const oldImpulseP = imp.impulseP;
      imp.impulseP += impulseP;
      if (imp.impulseP < row.minImpulse) imp.impulseP = row.minImpulse;
      else if (imp.impulseP > row.maxImpulse) imp.impulseP = row.maxImpulse;
      impulseP = imp.impulseP - oldImpulseP;
      lv1X += md.invMLin1X * impulseP; lv1Y += md.invMLin1Y * impulseP; lv1Z += md.invMLin1Z * impulseP;
      lv2X += md.invMLin2X * -impulseP; lv2Y += md.invMLin2Y * -impulseP; lv2Z += md.invMLin2Z * -impulseP;
      av1X += md.invMAng1X * impulseP; av1Y += md.invMAng1Y * impulseP; av1Z += md.invMAng1Z * impulseP;
      av2X += md.invMAng2X * -impulseP; av2Y += md.invMAng2Y * -impulseP; av2Z += md.invMAng2Z * -impulseP;
    }
    b1.pvx = lv1X; b1.pvy = lv1Y; b1.pvz = lv1Z;
    b2.pvx = lv2X; b2.pvy = lv2Y; b2.pvz = lv2Z;
    b1.pwx = av1X; b1.pwy = av1Y; b1.pwz = av1Z;
    b2.pwx = av2X; b2.pwy = av2Y; b2.pwz = av2Z;
  }

  /** 非线性高斯-赛德尔：每次迭代重新算约束行，直接修改刚体的位置和朝向 */
  solvePositionNgs(): void {
    this.updatePositionData();
    const info = this.info, b1 = this.b1!, b2 = this.b2!;
    let lv1X = 0, lv1Y = 0, lv1Z = 0;
    let lv2X = 0, lv2Y = 0, lv2Z = 0;
    let av1X = 0, av1Y = 0, av1Z = 0;
    let av2X = 0, av2Y = 0, av2Z = 0;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse!, j = row.jacobian;
      let rv = 0;
      rv += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rv -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rv += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rv -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseP = (row.rhs * POSITION_NGS_BAUMGARTE - rv) * md.mass;
      const oldImpulseP = imp.impulseP;
      imp.impulseP += impulseP;
      if (imp.impulseP < row.minImpulse) imp.impulseP = row.minImpulse;
      else if (imp.impulseP > row.maxImpulse) imp.impulseP = row.maxImpulse;
      impulseP = imp.impulseP - oldImpulseP;
      lv1X += md.invMLin1X * impulseP; lv1Y += md.invMLin1Y * impulseP; lv1Z += md.invMLin1Z * impulseP;
      lv2X += md.invMLin2X * -impulseP; lv2Y += md.invMLin2Y * -impulseP; lv2Z += md.invMLin2Z * -impulseP;
      av1X += md.invMAng1X * impulseP; av1Y += md.invMAng1Y * impulseP; av1Z += md.invMAng1Z * impulseP;
      av2X += md.invMAng2X * -impulseP; av2Y += md.invMAng2Y * -impulseP; av2Z += md.invMAng2Z * -impulseP;
    }
    b1.applyTranslation(lv1X, lv1Y, lv1Z);
    b2.applyTranslation(lv2X, lv2Y, lv2Z);
    b1.applyRotation(av1X, av1Y, av1Z);
    b2.applyRotation(av2X, av2Y, av2Z);
  }

  postSolve(): void {
    this.joint.syncAnchors();
    this.joint.checkDestruction();
  }
}
