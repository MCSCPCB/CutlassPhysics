// 移植自 OimoPhysics src/oimo/dynamics/constraint/ConstraintSolver.hx（MIT，见 Credits.md）
import type { RigidBody } from "../rigidbody/rigid-body.js";
import type { TimeStep } from "../time-step.js";

/** 约束求解器基类。岛（Island）每步按固定顺序调用这些方法 */
export abstract class ConstraintSolver {
  b1: RigidBody | null = null;
  b2: RigidBody | null = null;
  addedToIsland = false;
  /** 最近一次软接触子步的步长，用于关节热启动。 */
  softSubstepDt = 0;

  abstract preSolveVelocity(timeStep: TimeStep): void;
  abstract warmStart(timeStep: TimeStep): void;
  abstract solveVelocity(): void;
  /** 速度求解之后（关节在这里算约束力） */
  abstract postSolveVelocity(timeStep: TimeStep): void;
  abstract preSolvePosition(timeStep: TimeStep): void;
  abstract solvePositionSplitImpulse(): void;
  /** 非线性高斯-赛德尔位置修正：直接修改刚体的位置和朝向 */
  abstract solvePositionNgs(timeStep: TimeStep): void;
  abstract postSolve(): void;
}
