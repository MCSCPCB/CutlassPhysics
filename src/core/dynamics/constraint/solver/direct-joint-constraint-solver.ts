// 移植自 OimoPhysics src/oimo/dynamics/constraint/solver/direct/DirectJointConstraintSolver.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { MAX_JACOBIAN_ROWS, POSITION_NGS_BAUMGARTE, POSITION_SPLIT_IMPULSE_BAUMGARTE } from "../../../settings.js";
import type { TimeStep } from "../../time-step.js";
import { ConstraintSolver } from "../constraint-solver.js";
import type { Joint } from "../joint/joint.js";
import { JointSolverInfo, JointSolverMassDataRow } from "../joint/joint-solver-info.js";
import { BoundaryBuilder, BoundarySelector } from "./boundary.js";
import { MassMatrix } from "./mass-matrix.js";

/**
 * 关节约束的直接求解器：把一个关节的所有约束行当成一个混合线性互补问题（MLCP）一次解出来。
 * 依次尝试每一种“哪些行顶在上下限”的情况（上一次成功的先试），解对应的线性方程组，直到满足所有条件
 */
export class DirectJointConstraintSolver extends ConstraintSolver {
  private readonly joint: Joint;
  private readonly info = new JointSolverInfo();
  private readonly massData: JointSolverMassDataRow[] = [];
  private readonly relVels: number[] = [];
  private readonly impulses: number[] = [];
  private readonly dImpulses: number[] = [];
  private readonly dTotalImpulses: number[] = [];
  private readonly massMatrix = new MassMatrix(MAX_JACOBIAN_ROWS);
  private readonly boundaryBuilder = new BoundaryBuilder(MAX_JACOBIAN_ROWS);
  private readonly velBoundarySelector = new BoundarySelector(this.boundaryBuilder.maxBoundaries);
  private readonly posBoundarySelector = new BoundarySelector(this.boundaryBuilder.maxBoundaries);
  /** 冲量换算成两个刚体的速度（或位移、转动）变化时用的临时变量 */
  private l1X = 0; private l1Y = 0; private l1Z = 0;
  private l2X = 0; private l2Y = 0; private l2Z = 0;
  private a1X = 0; private a1Y = 0; private a1Z = 0;
  private a2X = 0; private a2Y = 0; private a2Z = 0;
  private linearSet = false;
  private angularSet = false;

  constructor(joint: Joint) {
    super();
    this.joint = joint;
    for (let i = 0; i < MAX_JACOBIAN_ROWS; i++) {
      this.massData.push(new JointSolverMassDataRow());
      this.relVels.push(0);
      this.impulses.push(0);
      this.dImpulses.push(0);
      this.dTotalImpulses.push(0);
    }
  }

  /** 每一行的冲量换算成速度变化，加到 l1..a2 上（原版 applyImpulses 等共同的循环） */
  private accumulate(impulses: number[]): void {
    const info = this.info;
    this.linearSet = false;
    this.angularSet = false;
    for (let i = 0; i < info.numRows; i++) {
      const j = info.rows[i].jacobian, md = this.massData[i], imp = impulses[i];
      if (j.isLinearSet()) {
        this.l1X += md.invMLin1X * imp; this.l1Y += md.invMLin1Y * imp; this.l1Z += md.invMLin1Z * imp;
        this.l2X += md.invMLin2X * -imp; this.l2Y += md.invMLin2Y * -imp; this.l2Z += md.invMLin2Z * -imp;
        this.linearSet = true;
      }
      if (j.isAngularSet()) {
        this.a1X += md.invMAng1X * imp; this.a1Y += md.invMAng1Y * imp; this.a1Z += md.invMAng1Z * imp;
        this.a2X += md.invMAng2X * -imp; this.a2Y += md.invMAng2Y * -imp; this.a2Z += md.invMAng2Z * -imp;
        this.angularSet = true;
      }
    }
  }

  /** 冲量加到两个刚体的速度上 */
  private applyImpulses(impulses: number[]): void {
    const b1 = this.b1!, b2 = this.b2!;
    this.l1X = b1.vx; this.l1Y = b1.vy; this.l1Z = b1.vz;
    this.l2X = b2.vx; this.l2Y = b2.vy; this.l2Z = b2.vz;
    this.a1X = b1.wx; this.a1Y = b1.wy; this.a1Z = b1.wz;
    this.a2X = b2.wx; this.a2Y = b2.wy; this.a2Z = b2.wz;
    this.accumulate(impulses);
    if (this.linearSet) {
      b1.vx = this.l1X; b1.vy = this.l1Y; b1.vz = this.l1Z;
      b2.vx = this.l2X; b2.vy = this.l2Y; b2.vz = this.l2Z;
    }
    if (this.angularSet) {
      b1.wx = this.a1X; b1.wy = this.a1Y; b1.wz = this.a1Z;
      b2.wx = this.a2X; b2.wy = this.a2Y; b2.wz = this.a2Z;
    }
  }

  /** 位置修正（分离冲量）的冲量加到两个刚体的伪速度上 */
  private applySplitImpulses(impulses: number[]): void {
    const b1 = this.b1!, b2 = this.b2!;
    this.l1X = b1.pvx; this.l1Y = b1.pvy; this.l1Z = b1.pvz;
    this.l2X = b2.pvx; this.l2Y = b2.pvy; this.l2Z = b2.pvz;
    this.a1X = b1.pwx; this.a1Y = b1.pwy; this.a1Z = b1.pwz;
    this.a2X = b2.pwx; this.a2Y = b2.pwy; this.a2Z = b2.pwz;
    this.accumulate(impulses);
    if (this.linearSet) {
      b1.pvx = this.l1X; b1.pvy = this.l1Y; b1.pvz = this.l1Z;
      b2.pvx = this.l2X; b2.pvy = this.l2Y; b2.pvz = this.l2Z;
    }
    if (this.angularSet) {
      b1.pwx = this.a1X; b1.pwy = this.a1Y; b1.pwz = this.a1Z;
      b2.pwx = this.a2X; b2.pwy = this.a2Y; b2.pwz = this.a2Z;
    }
  }

  /** 位置修正（NGS）的冲量直接换算成两个刚体的位移和转动 */
  private applyPositionImpulses(impulses: number[]): void {
    const b1 = this.b1!, b2 = this.b2!;
    this.l1X = 0; this.l1Y = 0; this.l1Z = 0;
    this.l2X = 0; this.l2Y = 0; this.l2Z = 0;
    this.a1X = 0; this.a1Y = 0; this.a1Z = 0;
    this.a2X = 0; this.a2Y = 0; this.a2Z = 0;
    this.accumulate(impulses);
    if (this.linearSet) {
      b1.applyTranslation(this.l1X, this.l1Y, this.l1Z);
      b2.applyTranslation(this.l2X, this.l2Y, this.l2Z);
    }
    if (this.angularSet) {
      b1.applyRotation(this.a1X, this.a1Y, this.a1Z);
      b2.applyRotation(this.a2X, this.a2Y, this.a2Z);
    }
  }

  preSolveVelocity(timeStep: TimeStep): void {
    this.joint.syncAnchors();
    this.joint.getVelocitySolverInfo(timeStep, this.info);
    this.b1 = this.info.b1;
    this.b2 = this.info.b2;
    this.massMatrix.computeInvMass(this.info, this.massData);
    this.boundaryBuilder.buildBoundaries(this.info);
    this.velBoundarySelector.setSize(this.boundaryBuilder.numBoundaries);
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
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse!;
      // 上一步的冲量缩放后夹在这一步的上下限里
      let impulse = imp.impulse * factor;
      if (impulse < row.minImpulse) impulse = row.minImpulse;
      else if (impulse > row.maxImpulse) impulse = row.maxImpulse;
      imp.impulse = impulse;
      if (row.motorMaxImpulse > 0) {
        let impulseM = imp.impulseM * factor;
        const max = row.motorMaxImpulse;
        if (impulseM < -max) impulseM = -max;
        else if (impulseM > max) impulseM = max;
        imp.impulseM = impulseM;
      } else {
        imp.impulseM = 0;
      }
      this.dImpulses[i] = imp.impulse + imp.impulseM;
    }
    this.applyImpulses(this.dImpulses);
  }

  solveVelocity(): void {
    const info = this.info, numRows = info.numRows, b1 = this.b1!, b2 = this.b2!;
    const relVels = this.relVels, impulses = this.impulses, dImpulses = this.dImpulses, dTotalImpulses = this.dTotalImpulses;
    const lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz;
    const lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    const av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz;
    const av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    for (let i = 0; i < numRows; i++) {
      const row = info.rows[i], j = row.jacobian;
      let relVel = 0;
      relVel += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      relVel -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      relVel += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      relVel -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      relVels[i] = relVel;
      impulses[i] = row.impulse!.impulse;
      dTotalImpulses[i] = 0;
    }

    // 先解马达。相对速度是刚体 1 - 刚体 2，马达速度是刚体 2 - 刚体 1，所以目标相对速度是 -马达速度
    const invMass = this.massMatrix.invMassWithoutCfm;
    for (let i = 0; i < numRows; i++) {
      const row = info.rows[i], imp = row.impulse!;
      if (row.motorMaxImpulse > 0) {
        const oldImpulseM = imp.impulseM;
        let impulseM = oldImpulseM + this.massData[i].massWithoutCfm * (-row.motorSpeed - relVels[i]);
        const maxImpulseM = row.motorMaxImpulse;
        if (impulseM < -maxImpulseM) impulseM = -maxImpulseM;
        else if (impulseM > maxImpulseM) impulseM = maxImpulseM;
        imp.impulseM = impulseM;
        const dImpulseM = impulseM - oldImpulseM;
        dTotalImpulses[i] = dImpulseM;
        // 先在相对速度上虚拟地加上马达冲量
        for (let k = 0; k < numRows; k++) relVels[k] += dImpulseM * invMass[i][k];
      }
    }

    // 依次尝试每一种情况，找到满足条件的就用它的冲量
    const builder = this.boundaryBuilder;
    for (let i = 0; i < builder.numBoundaries; i++) {
      const idx = this.velBoundarySelector.getIndex(i);
      if (builder.boundaries[idx].computeImpulses(info, this.massMatrix, relVels, impulses, dImpulses, 1)) {
        for (let k = 0; k < numRows; k++) {
          const dimp = dImpulses[k];
          info.rows[k].impulse!.impulse += dimp;
          dTotalImpulses[k] += dimp;
        }
        // 马达 + 限制的冲量一起加到刚体上
        this.applyImpulses(dTotalImpulses);
        // 下一次先试这种情况
        this.velBoundarySelector.select(idx);
        return;
      }
    }
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

  /** 位置修正阶段的约束行、逆质量矩阵和情况列表 */
  private updatePositionData(): void {
    this.joint.syncAnchors();
    this.joint.getPositionSolverInfo(this.info);
    this.b1 = this.info.b1;
    this.b2 = this.info.b2;
    this.massMatrix.computeInvMass(this.info, this.massData);
    this.boundaryBuilder.buildBoundaries(this.info);
    this.posBoundarySelector.setSize(this.boundaryBuilder.numBoundaries);
  }

  preSolvePosition(): void {
    this.updatePositionData();
    for (let i = 0; i < this.info.numRows; i++) this.info.rows[i].impulse!.impulseP = 0;
  }

  solvePositionSplitImpulse(): void {
    const info = this.info, numRows = info.numRows, b1 = this.b1!, b2 = this.b2!;
    const relVels = this.relVels, impulses = this.impulses, dImpulses = this.dImpulses;
    const lv1X = b1.pvx, lv1Y = b1.pvy, lv1Z = b1.pvz;
    const lv2X = b2.pvx, lv2Y = b2.pvy, lv2Z = b2.pvz;
    const av1X = b1.pwx, av1Y = b1.pwy, av1Z = b1.pwz;
    const av2X = b2.pwx, av2Y = b2.pwy, av2Z = b2.pwz;
    for (let i = 0; i < numRows; i++) {
      const row = info.rows[i], j = row.jacobian;
      let relVel = 0;
      relVel += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      relVel -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      relVel += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      relVel -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      relVels[i] = relVel;
      impulses[i] = row.impulse!.impulseP;
    }
    const builder = this.boundaryBuilder;
    for (let i = 0; i < builder.numBoundaries; i++) {
      const idx = this.posBoundarySelector.getIndex(i);
      if (builder.boundaries[idx].computeImpulses(info, this.massMatrix, relVels, impulses, dImpulses, POSITION_SPLIT_IMPULSE_BAUMGARTE)) {
        for (let k = 0; k < numRows; k++) info.rows[k].impulse!.impulseP += dImpulses[k];
        this.applySplitImpulses(dImpulses);
        this.posBoundarySelector.select(idx);
        return;
      }
    }
  }

  /** 非线性高斯-赛德尔：每次迭代重新算约束行，直接修改刚体的位置和朝向 */
  solvePositionNgs(): void {
    this.updatePositionData();
    const info = this.info, numRows = info.numRows;
    const relVels = this.relVels, impulses = this.impulses, dImpulses = this.dImpulses;
    for (let i = 0; i < numRows; i++) {
      // NGS 时相对速度当作 0
      relVels[i] = 0;
      impulses[i] = info.rows[i].impulse!.impulseP;
    }
    const builder = this.boundaryBuilder;
    for (let i = 0; i < builder.numBoundaries; i++) {
      const idx = this.posBoundarySelector.getIndex(i);
      if (builder.boundaries[idx].computeImpulses(info, this.massMatrix, relVels, impulses, dImpulses, POSITION_NGS_BAUMGARTE)) {
        for (let k = 0; k < numRows; k++) info.rows[k].impulse!.impulseP += dImpulses[k];
        this.applyPositionImpulses(dImpulses);
        this.posBoundarySelector.select(idx);
        return;
      }
    }
  }

  postSolve(): void {
    this.joint.syncAnchors();
    this.joint.checkDestruction();
  }
}
