// 移植自 OimoPhysics src/oimo/dynamics/Island.hx（MIT，见 Credits.md）
import type { ConstraintSolver } from "./constraint/constraint-solver.js";
import { PositionCorrectionAlgorithm } from "./constraint/position-correction-algorithm.js";
import type { Softness, SoftContactSolver } from "./constraint/solver/soft-contact-solver.js";
import { RigidBodyType } from "./rigidbody/rigid-body-config.js";
import type { RigidBody } from "./rigidbody/rigid-body.js";
import type { TimeStep } from "./time-step.js";
import type { DragConstraint } from "./constraint/drag-constraint.js";

/** 1 / eˣ 的近似，阻尼用 */
function fastInvExp(x: number): number {
  const x2 = x * x;
  return 1 / (1 + x + x2 * (1 / 2 + x * (1 / 6) + x2 * (1 / 24)));
}

/** 子步求解参数，由 World 在每步开始时设置。 */
export class SoftSubstepParams {
  /** 子步数 */
  substeps = 1;
  /**
   * 接触带偏置、松弛各解几遍。这个 tick 有接触的靠近速度超过 impactVelocity（m/s）时，两个阶段都解 impactIterations 遍，
   * 超出常规遍数的部分仅处理被撞刚体的接触。
   */
  iterations = 1;
  relaxIterations = 1;
  tolerance = 0;
  adaptiveIterations = false;
  impactIterations = 0;
  impactVelocity = 0;
  /** 子步结束后的反弹遍数；靠近速度超过 restitutionThreshold（m/s）的接触才反弹 */
  restitutionIterations = 1;
  restitutionThreshold = 0;
  /** 每个 tick 结束时的速度上限（m/s）、角速度上限（rad/s） */
  maxLinearVelocity = Infinity;
  maxAngularVelocity = Infinity;
  /** 关节每个子步的速度迭代次数、位置迭代次数 */
  jointIterations = 0;
  positionIterations = 0;
  /** 推开穿透的最大速度（m/s） */
  maxPushVelocity = 0;
  /** 第一个子步的求解顺序（之后每个子步交替） */
  parity = 0;
  /** 这次 World.step 的编号（接触记下它，求解之后据此判断是否接触） */
  stamp = 0;
  /** 预测接触距离：基础值、上限（米） */
  speculativeDistance = 0;
  maxSpeculativeDistance = 0;
  /** 接触面向外移的距离（米），见 World.softContactSkin */
  skin = 0;
  /** 虚空海（见 World.voidSea）：是否打开、开始的高度、深度系数、阻力系数 */
  sea = false;
  seaStartY = 0;
  seaDepthGradient = 1;
  seaDrag = 1;

  /** 两个动态刚体之间、一边不会动时的软约束参数 */
  constructor(readonly dynamic: Softness, readonly fixed: Softness) {}
}

/** 模拟岛：通过接触连在一起的一组刚体和约束，放在一起求解 */
export class Island {
  private gravityX = 0;
  private gravityY = 0;
  private gravityZ = 0;
  private readonly rigidBodies: RigidBody[] = [];
  private readonly drags: DragConstraint[] = [];
  private readonly solvers: ConstraintSolver[] = [];
  /** 位置修正用 split impulse、NGS 的约束 */
  private readonly solversSi: ConstraintSolver[] = [];
  private readonly solversNgs: ConstraintSolver[] = [];
  /** 软接触求解器列表；关节求解器保存在 solvers 中。 */
  private readonly softContacts: SoftContactSolver[] = [];
  private readonly orderedContacts: SoftContactSolver[] = [];
  /** beginSubsteps 计算的带偏置、松弛及速度迭代次数，供 runSubsteps 使用。 */
  private biasPasses = 0;
  private relaxPasses = 0;
  private solveTotal = 0;
  private conditionedContacts = false;
  private quietContacts = false;
  private isotropicContacts = false;
  private minMass = Infinity;
  private maxMass = 0;
  private extendedBody = false;

  clear(): void {
    this.rigidBodies.length = 0;
    this.drags.length = 0;
    this.solvers.length = 0;
    this.solversSi.length = 0;
    this.solversNgs.length = 0;
    this.softContacts.length = 0;
    this.orderedContacts.length = 0;
    this.minMass = Infinity;
    this.maxMass = 0;
    this.extendedBody = false;
  }

  setGravity(x: number, y: number, z: number): void {
    this.gravityX = x;
    this.gravityY = y;
    this.gravityZ = z;
  }

  addRigidBody(rb: RigidBody): void {
    rb.addedToIsland = true;
    this.rigidBodies.push(rb);
    if (rb.drag !== null) this.drags.push(rb.drag);
    if (rb.type === RigidBodyType.DYNAMIC) {
      if (rb.mass < this.minMass) this.minMass = rb.mass;
      if (rb.mass > this.maxMass) this.maxMass = rb.mass;
      if (Math.max(rb.li00, rb.li11, rb.li22) > 1.5 * Math.min(rb.li00, rb.li11, rb.li22)) this.extendedBody = true;
    }
  }

  /** 大质量比接触保留支撑所需的子步数。 */
  supportSubsteps(base: number, minimum: number): number {
    return base < minimum && this.softContacts.length > 0 && this.maxMass >= this.minMass * 8 ? minimum : base;
  }

  addConstraintSolver(solver: ConstraintSolver, positionCorrection: number): void {
    solver.addedToIsland = true;
    this.solvers.push(solver);
    if (positionCorrection === PositionCorrectionAlgorithm.SPLIT_IMPULSE) this.solversSi.push(solver);
    if (positionCorrection === PositionCorrectionAlgorithm.NGS) this.solversNgs.push(solver);
  }

  /** 将接触加入软接触求解列表。 */
  addSoftContact(solver: SoftContactSolver, owner: ConstraintSolver): void {
    owner.addedToIsland = true;
    this.softContacts.push(solver);
  }

  /** 单独一个刚体（没有接触）的一步 */
  stepSingleRigidBody(timeStep: TimeStep, rb: RigidBody): void {
    const dt = timeStep.dt;
    rb.ptransform.copyFrom(rb.transform);
    rb.lciX = 0; rb.lciY = 0; rb.lciZ = 0;
    rb.aciX = 0; rb.aciY = 0; rb.aciZ = 0;
    // 休眠计时
    if (rb.isSleepy() && !rb.restless) {
      rb.sleepTime += dt;
      if (rb.sleepTime >= rb.sleepingTimeThreshold) rb.sleep();
    } else {
      rb.sleepTime = 0;
    }
    if (rb.sleeping) return;
    if (rb.type === RigidBodyType.DYNAMIC) this.applyForces(rb, dt);
    rb.integrate(dt);
    rb.syncShapes();
  }

  /** 多个刚体和约束的一步：速度求解 → 积分 → 位置修正 */
  step(timeStep: TimeStep, numVelocityIterations: number, numPositionIterations: number): void {
    const dt = timeStep.dt, bodies = this.rigidBodies, solvers = this.solvers, solversSi = this.solversSi, solversNgs = this.solversNgs;
    let sleepIsland = true;
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      rb.ptransform.copyFrom(rb.transform);
      rb.lciX = 0; rb.lciY = 0; rb.lciZ = 0;
      rb.aciX = 0; rb.aciY = 0; rb.aciZ = 0;
      rb.sleeping = false;
      if (rb.isSleepy() && !rb.restless) rb.sleepTime += dt;
      else rb.sleepTime = 0;
      // 任一刚体未达到休眠时长时，整个岛保持活动。
      if (rb.sleepTime < rb.sleepingTimeThreshold) sleepIsland = false;
      if (rb.type === RigidBodyType.DYNAMIC) this.applyForces(rb, dt);
    }
    if (sleepIsland) {
      for (let i = 0; i < bodies.length; i++) bodies[i].sleep();
      return;
    }
    for (let i = 0; i < solvers.length; i++) solvers[i].preSolveVelocity(timeStep);
    for (let i = 0; i < solvers.length; i++) solvers[i].warmStart(timeStep);
    for (let t = 0; t < numVelocityIterations; t++) {
      for (let i = 0; i < solvers.length; i++) solvers[i].solveVelocity();
    }
    for (let i = 0; i < solvers.length; i++) solvers[i].postSolveVelocity(timeStep);
    for (let i = 0; i < bodies.length; i++) bodies[i].integrate(dt);
    for (let i = 0; i < solversSi.length; i++) solversSi[i].preSolvePosition(timeStep);
    for (let t = 0; t < numPositionIterations; t++) {
      for (let i = 0; i < solversSi.length; i++) solversSi[i].solvePositionSplitImpulse();
    }
    for (let i = 0; i < bodies.length; i++) bodies[i].integratePseudoVelocity();
    for (let i = 0; i < solversNgs.length; i++) solversNgs[i].preSolvePosition(timeStep);
    for (let t = 0; t < numPositionIterations; t++) {
      for (let i = 0; i < solversNgs.length; i++) solversNgs[i].solvePositionNgs(timeStep);
    }
    // 关节可能在这里断开、被删掉
    for (let i = 0; i < solvers.length; i++) solvers[i].postSolve();
    for (let i = 0; i < bodies.length; i++) bodies[i].syncShapes();
  }

  /**
   * 单步软接触求解：带偏置求解、积分、松弛。
   * 撞击增加松弛迭代次数，关节使用独立迭代次数及位置修正。
   */
  stepSoft(
    timeStep: TimeStep, iterations: number, relaxIterations: number, impactIterations: number, impactVelocity: number,
    dynamic: Softness, fixed: Softness, maxPushVelocity: number, jointIterations: number, numPositionIterations: number, parity: number,
  ): void {
    const dt = timeStep.dt, invDt = timeStep.invDt, bodies = this.rigidBodies, joints = this.solvers, contacts = this.softContacts;
    const jointsSi = this.solversSi, jointsNgs = this.solversNgs;
    let sleepIsland = true;
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      rb.ptransform.copyFrom(rb.transform);
      rb.lciX = 0; rb.lciY = 0; rb.lciZ = 0;
      rb.aciX = 0; rb.aciY = 0; rb.aciZ = 0;
      rb.sleeping = false;
      if (rb.isSleepy() && !rb.restless) rb.sleepTime += dt;
      else rb.sleepTime = 0;
      if (rb.sleepTime < rb.sleepingTimeThreshold) sleepIsland = false;
      if (rb.type === RigidBodyType.DYNAMIC) this.applyForces(rb, dt);
    }
    if (sleepIsland) {
      for (let i = 0; i < bodies.length; i++) bodies[i].sleep();
      return;
    }
    let approach = 0;
    for (let i = 0; i < contacts.length; i++) {
      const a = contacts[i].prepare();
      if (a > approach) approach = a;
    }
    // 撞击仅增加松弛迭代，限制穿透修正产生的额外分离速度。
    const impact = approach > impactVelocity;
    const passes = iterations;
    const relaxPasses = impact ? relaxIterations + impactIterations : relaxIterations;
    for (let i = 0; i < joints.length; i++) joints[i].preSolveVelocity(timeStep);
    for (let i = 0; i < joints.length; i++) joints[i].warmStart(timeStep);
    for (let i = 0; i < contacts.length; i++) contacts[i].warmStart();
    // 正向、反向交替（每一遍、每一步都换），减少求解顺序带来的偏向
    const total = joints.length > 0 && jointIterations > passes ? jointIterations : passes;
    for (let k = 0; k < total; k++) {
      for (let i = 0; i < joints.length; i++) joints[i].solveVelocity();
      const t = k - (total - passes);
      if (t < 0) continue;
      if (((t + parity) & 1) === 0) for (let i = 0; i < contacts.length; i++) contacts[i].solve(true, false, invDt, dynamic, fixed, maxPushVelocity);
      else for (let i = contacts.length - 1; i >= 0; i--) contacts[i].solve(true, true, invDt, dynamic, fixed, maxPushVelocity);
    }
    for (let i = 0; i < joints.length; i++) joints[i].postSolveVelocity(timeStep);
    for (let i = 0; i < bodies.length; i++) bodies[i].integrate(dt);
    // 松弛阶段移除修正偏置，消除穿透修正产生的分离速度。
    for (let t = 0; t < relaxPasses; t++) {
      if (((t + parity) & 1) === 0) for (let i = contacts.length - 1; i >= 0; i--) contacts[i].solve(false, true, invDt, dynamic, fixed, maxPushVelocity);
      else for (let i = 0; i < contacts.length; i++) contacts[i].solve(false, false, invDt, dynamic, fixed, maxPushVelocity);
    }
    for (let i = 0; i < contacts.length; i++) contacts[i].applyRestitution();
    // 关节的位置修正
    for (let i = 0; i < jointsSi.length; i++) jointsSi[i].preSolvePosition(timeStep);
    for (let t = 0; t < numPositionIterations; t++) {
      for (let i = 0; i < jointsSi.length; i++) jointsSi[i].solvePositionSplitImpulse();
    }
    for (let i = 0; i < bodies.length; i++) bodies[i].integratePseudoVelocity();
    for (let i = 0; i < jointsNgs.length; i++) jointsNgs[i].preSolvePosition(timeStep);
    for (let t = 0; t < numPositionIterations; t++) {
      for (let i = 0; i < jointsNgs.length; i++) jointsNgs[i].solvePositionNgs(timeStep);
    }
    for (let i = 0; i < joints.length; i++) joints[i].postSolve();
    for (let i = 0; i < contacts.length; i++) contacts[i].store();
    for (let i = 0; i < bodies.length; i++) bodies[i].syncShapes();
  }

  /**
   * 准备多个子步共用的接触数据；整个岛休眠时返回 false。
   * 随后由 runSubsteps 求解和积分，finishSubsteps 完成反弹、断开检查及形状同步。
   * tick 为整步时长，sub 为子步时长。
   */
  beginSubsteps(tick: TimeStep, sub: TimeStep, p: SoftSubstepParams): boolean {
    const bodies = this.rigidBodies, joints = this.solvers, contacts = this.softContacts;
    // 地面支撑先于动态接触，顺序不依赖地形加载时机。
    const ordered = this.orderedContacts;
    ordered.length = 0;
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i].constraint;
      if (c.b1!.type === RigidBodyType.STATIC || c.b2!.type === RigidBodyType.STATIC) ordered.push(contacts[i]);
    }
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i].constraint;
      if (c.b1!.type !== RigidBodyType.STATIC && c.b2!.type !== RigidBodyType.STATIC) ordered.push(contacts[i]);
    }
    for (let i = 0; i < contacts.length; i++) contacts[i] = ordered[i];
    // 休眠计时使用求解后的速度。
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      rb.ptransform.copyFrom(rb.transform);
      rb.lciX = 0; rb.lciY = 0; rb.lciZ = 0;
      rb.aciX = 0; rb.aciY = 0; rb.aciZ = 0;
      rb.sleeping = false;
    }
    // prepare 的靠近速度不含本步重力；外力在各子步开始时施加。
    // 撞击标记使用当前步进编号，排除休眠刚体保留的历史标记。
    let impact = false;
    const stamp = p.stamp;
    for (let i = 0; i < contacts.length; i++) {
      if (contacts[i].prepare(sub.dtRatio, true, p.skin, sub.dt, p.restitutionThreshold) > p.impactVelocity) {
        impact = true;
        const c = contacts[i].constraint, b1 = c.b1!, b2 = c.b2!;
        if (b1.type === RigidBodyType.DYNAMIC && !b1.sleeping) b1.impactStamp = stamp;
        if (b2.type === RigidBodyType.DYNAMIC && !b2.sleeping) b2.impactStamp = stamp;
      }
    }
    // 撞击标记决定额外迭代的接触范围。
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i].constraint;
      contacts[i].impacted = impact && (c.b1!.impactStamp === stamp || c.b2!.impactStamp === stamp);
    }
    // 宽展支撑体合并后仍保留大质量比下的冲量传递迭代。
    this.conditionedContacts = (contacts.length > 8 || this.extendedBody) && this.maxMass >= this.minMass * 8;
    const minimumPasses = this.conditionedContacts ? Math.min(40, Math.max(10, Math.ceil(this.maxMass / this.minMass / 4))) : 0;
    const passes = Math.max(minimumPasses, impact ? Math.max(p.impactIterations, p.iterations) : p.iterations);
    this.biasPasses = passes;
    this.relaxPasses = Math.max(minimumPasses, impact ? Math.max(p.impactIterations, p.relaxIterations) : p.relaxIterations);
    this.solveTotal = joints.length > 0 && p.jointIterations > passes ? p.jointIterations : passes;
    let quiet = p.adaptiveIterations && !impact && !this.conditionedContacts && joints.length === 0 && (p.substeps >= 4 || p.substeps === 1);
    let isotropic = true;
    for (let i = 0; quiet && i < bodies.length; i++) {
      const b = bodies[i];
      if (b.type === RigidBodyType.STATIC) continue;
      if (b.type !== RigidBodyType.DYNAMIC || b.drag !== null) { quiet = false; break; }
      const limit = Math.max(b.sleepingVelocityThreshold, b.sleepingAngularVelocityThreshold);
      if (b.vx * b.vx + b.vy * b.vy + b.vz * b.vz + b.wx * b.wx + b.wy * b.wy + b.wz * b.wz >= limit * limit ||
        b.fx !== 0 || b.fy !== 0 || b.fz !== 0 || b.tx !== 0 || b.ty !== 0 || b.tz !== 0 ||
        (p.sea && (p.seaStartY - b.transform.py) * p.seaDepthGradient >= 0)) quiet = false;
      if (b.li01 !== 0 || b.li02 !== 0 || b.li12 !== 0 || b.li00 !== b.li11 || b.li11 !== b.li22) isotropic = false;
    }
    this.quietContacts = quiet;
    this.isotropicContacts = isotropic;
    return true;
  }

  /** 执行 substeps 个子步，仅在 beginSubsteps 返回 true 时调用。 */
  runSubsteps(sub: TimeStep, p: SoftSubstepParams): void {
    const h = sub.dt, invH = sub.invDt, n = p.substeps;
    const bodies = this.rigidBodies, joints = this.solvers, contacts = this.softContacts;
    const jointsSi = this.solversSi, jointsNgs = this.solversNgs;
    const dynamic = p.dynamic, fixed = p.fixed, push = p.maxPushVelocity;
    const reuseAcrossSubsteps = this.relaxPasses > 0 && jointsSi.length === 0 && jointsNgs.length === 0;
    const drags = this.drags;
    const aggregateWarmStart = joints.length === 0 && drags.length === 0;
    let parity = p.parity;
    let quiet = this.quietContacts;
    for (let s = 0; s < n; s++) {
      if (s > 0) sub.dtRatio = 1;
      for (let i = 0; i < bodies.length; i++) if (bodies[i].type === RigidBodyType.DYNAMIC) this.applySoftForces(bodies[i], h, p);
      for (let i = 0; i < joints.length; i++) joints[i].preSolveVelocity(sub);
      const ratio = sub.dtRatio;
      for (let i = 0; i < joints.length; i++) {
        const joint = joints[i];
        sub.dtRatio = s > 0 || joint.softSubstepDt === 0 ? ratio : h / joint.softSubstepDt;
        joint.warmStart(sub);
        joint.softSubstepDt = h;
      }
      for (let i = 0; i < drags.length; i++) drags[i].warmStart();
      // 同一 tick 的力臂与有效质量不变，无关节时按刚体复用接触合速度增量。
      if (aggregateWarmStart) for (let i = 0; i < bodies.length; i++) {
        const b = bodies[i];
        if (b.type !== RigidBodyType.DYNAMIC) continue;
        const vx = b.vx, vy = b.vy, vz = b.vz, wx = b.wx, wy = b.wy, wz = b.wz;
        if (s > 0) {
          b.vx += b.contactDeltaVX; b.vy += b.contactDeltaVY; b.vz += b.contactDeltaVZ;
          b.wx += b.contactDeltaWX; b.wy += b.contactDeltaWY; b.wz += b.contactDeltaWZ;
        }
        b.contactDeltaVX = vx; b.contactDeltaVY = vy; b.contactDeltaVZ = vz;
        b.contactDeltaWX = wx; b.contactDeltaWY = wy; b.contactDeltaWZ = wz;
      }
      if (!aggregateWarmStart || s === 0) for (let i = 0; i < contacts.length; i++) contacts[i].warmStart();
      for (let i = 0; i < drags.length; i++) drags[i].solve(h, (s + 1) / n);
      for (let i = 0; quiet && i < bodies.length; i++) {
        const b = bodies[i], limit = Math.max(b.sleepingVelocityThreshold, b.sleepingAngularVelocityThreshold);
        if (b.type === RigidBodyType.STATIC) continue;
        if (b.vx * b.vx + b.vy * b.vy + b.vz * b.vz + b.wx * b.wx + b.wy * b.wy + b.wz * b.wz >= 4 * limit * limit) quiet = false;
      }
      const passes = quiet ? Math.min(this.biasPasses, this.isotropicContacts && (s > 0 || n === 1) ? 1 : 2) : this.biasPasses;
      const relaxPasses = quiet ? Math.min(this.relaxPasses, n > 1 || this.isotropicContacts ? 1 : 2) : this.relaxPasses;
      const total = quiet ? passes : this.solveTotal;
      for (let k = 0; k < total; k++) {
        for (let i = 0; i < joints.length; i++) joints[i].solveVelocity();
        const t = k - (total - passes);
        if (t < 0) continue;
        const reuse = t > 0 || (s > 0 && reuseAcrossSubsteps);
        let error = 0;
        // 承重组合的额外迭代覆盖全部接触，其余组合仅扩展撞击接触。
        if (((t + parity) & 1) === 0) {
          for (let i = 0; i < contacts.length; i++) if (this.conditionedContacts || t < p.iterations || contacts[i].impacted) error += contacts[i].solve(true, false, invH, dynamic, fixed, push, reuse);
        } else {
          for (let i = contacts.length - 1; i >= 0; i--) if (this.conditionedContacts || t < p.iterations || contacts[i].impacted) error += contacts[i].solve(true, true, invH, dynamic, fixed, push, reuse);
        }
        if (joints.length === 0 && error < p.tolerance) break;
      }
      for (let i = 0; i < joints.length; i++) joints[i].postSolveVelocity(sub);
      for (let i = 0; i < bodies.length; i++) bodies[i].integrate(h);
      for (let t = 0; t < relaxPasses; t++) {
        // 同一轮松弛只改速度；积分后的第一遍重算，其余遍复用。
        const reuse = t > 0;
        let error = 0;
        if (((t + parity) & 1) === 0) {
          for (let i = contacts.length - 1; i >= 0; i--) if (this.conditionedContacts || t < p.relaxIterations || contacts[i].impacted) error += contacts[i].solve(false, true, invH, dynamic, fixed, push, reuse);
        } else {
          for (let i = 0; i < contacts.length; i++) if (this.conditionedContacts || t < p.relaxIterations || contacts[i].impacted) error += contacts[i].solve(false, false, invH, dynamic, fixed, push, reuse);
        }
        if (joints.length === 0 && error < p.tolerance) break;
      }
      if (aggregateWarmStart) for (let i = 0; i < bodies.length; i++) {
        const b = bodies[i];
        if (b.type !== RigidBodyType.DYNAMIC) continue;
        b.contactDeltaVX = b.vx - b.contactDeltaVX;
        b.contactDeltaVY = b.vy - b.contactDeltaVY;
        b.contactDeltaVZ = b.vz - b.contactDeltaVZ;
        b.contactDeltaWX = b.wx - b.contactDeltaWX;
        b.contactDeltaWY = b.wy - b.contactDeltaWY;
        b.contactDeltaWZ = b.wz - b.contactDeltaWZ;
      }
      // 关节的位置修正（每个子步一次）
      for (let i = 0; i < jointsSi.length; i++) jointsSi[i].preSolvePosition(sub);
      for (let t = 0; t < p.positionIterations; t++) {
        for (let i = 0; i < jointsSi.length; i++) jointsSi[i].solvePositionSplitImpulse();
      }
      for (let i = 0; i < bodies.length; i++) bodies[i].integratePseudoVelocity();
      for (let i = 0; i < jointsNgs.length; i++) jointsNgs[i].preSolvePosition(sub);
      for (let t = 0; t < p.positionIterations; t++) {
        for (let i = 0; i < jointsNgs.length; i++) jointsNgs[i].solvePositionNgs(sub);
      }
      parity ^= 1;
    }
  }

  /** 完成本步的反弹、关节断开检查、冲量存储和形状同步。 */
  finishSubsteps(tick: TimeStep, p: SoftSubstepParams): void {
    const dt = tick.dt;
    const bodies = this.rigidBodies, joints = this.solvers, contacts = this.softContacts;
    // 反弹：只要有一个接触要反弹，所有要反弹的接触一起解 restitutionIterations 遍
    let bounce = false;
    for (let i = 0; i < contacts.length; i++) {
      if (contacts[i].wantsRestitution()) {
        bounce = true;
        break;
      }
    }
    if (bounce) {
      for (let t = 0; t < p.restitutionIterations; t++) for (let i = 0; i < contacts.length; i++) contacts[i].applyRestitution();
    }
    // 关节可能在这里断开、被删掉
    for (let i = 0; i < joints.length; i++) joints[i].postSolve();
    for (let i = 0; i < contacts.length; i++) contacts[i].store(p.stamp);
    // 休眠：按求解之后的速度计时，整个岛都够久了才一起睡，睡时速度清零
    let sleepIsland = true;
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      if (rb.type === RigidBodyType.DYNAMIC) this.limitState(rb, p);
      rb.drag?.finish();
      if (rb.isSleepy() && !rb.restless && rb.drag === null) rb.sleepTime += dt;
      else rb.sleepTime = 0;
      if (rb.sleepTime < rb.sleepingTimeThreshold) sleepIsland = false;
    }
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      if (sleepIsland) {
        rb.sleep();
        rb.vx = 0; rb.vy = 0; rb.vz = 0;
        rb.wx = 0; rb.wy = 0; rb.wz = 0;
      }
      // 静态刚体的形状已在外部变换更新时同步。
      if (rb.type !== RigidBodyType.STATIC) rb.syncShapesSpeculative(dt, p.speculativeDistance, p.maxSpeculativeDistance);
    }
  }

  /** 记录孤立刚体的步进起始位姿，供子步积分与异常回滚使用。 */
  beginSingleRigidBodySubsteps(tick: TimeStep, rb: RigidBody): boolean {
    rb.ptransform.copyFrom(rb.transform);
    rb.lciX = 0; rb.lciY = 0; rb.lciZ = 0;
    rb.aciX = 0; rb.aciY = 0; rb.aciZ = 0;
    return !rb.sleeping;
  }

  /** 执行孤立刚体的子步积分。 */
  runSingleRigidBodySubsteps(sub: TimeStep, p: SoftSubstepParams, rb: RigidBody): void {
    const h = sub.dt;
    const dynamic = rb.type === RigidBodyType.DYNAMIC;
    for (let s = 0; s < p.substeps; s++) {
      if (dynamic) this.applySoftForces(rb, h, p);
      rb.drag?.warmStart();
      rb.drag?.solve(h, (s + 1) / p.substeps);
      rb.integrate(h);
    }
  }

  /** 完成孤立刚体的限速、休眠计时和形状同步。 */
  finishSingleRigidBodySubsteps(tick: TimeStep, p: SoftSubstepParams, rb: RigidBody): void {
    rb.drag?.finish();
    if (rb.type === RigidBodyType.DYNAMIC) this.limitState(rb, p);
    if (rb.isSleepy() && !rb.restless && rb.drag === null) {
      rb.sleepTime += tick.dt;
      if (rb.sleepTime >= rb.sleepingTimeThreshold) {
        rb.sleep();
        rb.vx = 0; rb.vy = 0; rb.vz = 0;
        rb.wx = 0; rb.wy = 0; rb.wz = 0;
      }
    } else {
      rb.sleepTime = 0;
    }
    rb.syncShapesSpeculative(tick.dt, p.speculativeDistance, p.maxSpeculativeDistance);
  }

  /**
   * 累加重力、外力和力矩，应用隐式陀螺项及 1 / (1 + h·d) 阻尼。
   * 存在锁定转轴时跳过陀螺项；p.sea 启用时先施加虚空海作用力。
   */
  private applySoftForces(rb: RigidBody, h: number, p: SoftSubstepParams): void {
    const gs = rb.gravityScale, invMass = rb.invMass;
    let vX = rb.vx, vY = rb.vy, vZ = rb.vz, wX = rb.wx, wY = rb.wy, wZ = rb.wz;
    if (p.sea) {
      // 深度按刚体的位置算：深度 = (开始高度 - y) × 深度系数。在海里时速度、角速度按子步衰减，
      // 再沿重力的反方向加 深度 × 子步时长 的速度：深度为 1 时正好抵消重力
      const depth = (p.seaStartY - rb.transform.py) * p.seaDepthGradient;
      if (depth >= 0) {
        const lin = 1 - h * 3.5 * p.seaDrag, ang = 1 - h * 3 * p.seaDrag;
        const lift = -Math.sign(p.seaDepthGradient) * depth * h;
        vX = vX * lin + this.gravityX * lift; vY = vY * lin + this.gravityY * lift; vZ = vZ * lin + this.gravityZ * lift;
        wX *= ang; wY *= ang; wZ *= ang;
      }
    }
    vX += h * (this.gravityX * gs + rb.fx * invMass);
    vY += h * (this.gravityY * gs + rb.fy * invMass);
    vZ += h * (this.gravityZ * gs + rb.fz * invMass);
    if (rb.tx !== 0 || rb.ty !== 0 || rb.tz !== 0) {
      wX += h * (rb.ii00 * rb.tx + rb.ii01 * rb.ty + rb.ii02 * rb.tz);
      wY += h * (rb.ii10 * rb.tx + rb.ii11 * rb.ty + rb.ii12 * rb.tz);
      wZ += h * (rb.ii20 * rb.tx + rb.ii21 * rb.ty + rb.ii22 * rb.tz);
    }
    if (wX * wX + wY * wY + wZ * wZ > 1e-6 && rb.rotFactorX === 1 && rb.rotFactorY === 1 && rb.rotFactorZ === 1) {
      const tf = rb.transform;
      const r00 = tf.r00, r01 = tf.r01, r02 = tf.r02, r10 = tf.r10, r11 = tf.r11, r12 = tf.r12, r20 = tf.r20, r21 = tf.r21, r22 = tf.r22;
      // 角速度转到局部坐标，L = I·ω，f = h·(ω × L)，雅可比 J = I + h·(skew(ω)·I - skew(L))，局部角速度减去 J⁻¹·f
      const oX = r00 * wX + r10 * wY + r20 * wZ, oY = r01 * wX + r11 * wY + r21 * wZ, oZ = r02 * wX + r12 * wY + r22 * wZ;
      const i0 = rb.li00, i1 = rb.li01, i2 = rb.li02, i3 = rb.li11, i4 = rb.li12, i5 = rb.li22;
      const lX = i0 * oX + i1 * oY + i2 * oZ, lY = i1 * oX + i3 * oY + i4 * oZ, lZ = i2 * oX + i4 * oY + i5 * oZ;
      const fX = h * (oY * lZ - oZ * lY), fY = h * (oZ * lX - oX * lZ), fZ = h * (oX * lY - oY * lX);
      const j00 = i0 + h * (-oZ * i1 + oY * i2), j01 = i1 + h * (-oZ * i3 + oY * i4 + lZ), j02 = i2 + h * (-oZ * i4 + oY * i5 - lY);
      const j10 = i1 + h * (oZ * i0 - oX * i2 - lZ), j11 = i3 + h * (oZ * i1 - oX * i4), j12 = i4 + h * (oZ * i2 - oX * i5 + lX);
      const j20 = i2 + h * (-oY * i0 + oX * i1 + lY), j21 = i4 + h * (-oY * i1 + oX * i3 - lX), j22 = i5 + h * (-oY * i2 + oX * i4);
      const c0 = j11 * j22 - j12 * j21, c1 = j12 * j20 - j10 * j22, c2 = j10 * j21 - j11 * j20;
      const det = j00 * c0 + j01 * c1 + j02 * c2;
      if (Math.abs(det) > 1e-12) {
        const inv = 1 / det;
        const dX = (fX * c0 + fY * (j02 * j21 - j01 * j22) + fZ * (j01 * j12 - j02 * j11)) * inv;
        const dY = (fX * c1 + fY * (j00 * j22 - j02 * j20) + fZ * (j02 * j10 - j00 * j12)) * inv;
        const dZ = (fX * c2 + fY * (j01 * j20 - j00 * j21) + fZ * (j00 * j11 - j01 * j10)) * inv;
        const nX = oX - dX, nY = oY - dY, nZ = oZ - dZ;
        wX = r00 * nX + r01 * nY + r02 * nZ;
        wY = r10 * nX + r11 * nY + r12 * nZ;
        wZ = r20 * nX + r21 * nY + r22 * nZ;
      }
    }
    const lin = 1 / (1 + h * rb.linearDamping), ang = 1 / (1 + h * rb.angularDamping);
    rb.vx = vX * lin; rb.vy = vY * lin; rb.vz = vZ * lin;
    rb.wx = wX * ang; rb.wy = wY * ang; rb.wz = wZ * ang;
  }

  /**
   * 结束步进并限制速度。
   * 非有限状态回滚至起始位姿，清零速度并丢弃接触点。
   */
  private limitState(rb: RigidBody, p: SoftSubstepParams): void {
    const tf = rb.transform;
    const sum = rb.vx + rb.vy + rb.vz + rb.wx + rb.wy + rb.wz + tf.px + tf.py + tf.pz +
      tf.r00 + tf.r01 + tf.r02 + tf.r10 + tf.r11 + tf.r12 + tf.r20 + tf.r21 + tf.r22;
    if (sum !== sum || sum === Infinity || sum === -Infinity) {
      tf.copyFrom(rb.ptransform);
      rb.vx = 0; rb.vy = 0; rb.vz = 0;
      rb.wx = 0; rb.wy = 0; rb.wz = 0;
      rb.updateInvInertia();
      for (let l = rb.contactLinkList; l !== null; l = l.next) l.contact!.manifold.clear();
      return;
    }
    const v2 = rb.vx * rb.vx + rb.vy * rb.vy + rb.vz * rb.vz, maxV = p.maxLinearVelocity;
    if (v2 > maxV * maxV) {
      const s = maxV / Math.sqrt(v2);
      rb.vx *= s; rb.vy *= s; rb.vz *= s;
    }
    const w2 = rb.wx * rb.wx + rb.wy * rb.wy + rb.wz * rb.wz, maxW = p.maxAngularVelocity;
    if (w2 > maxW * maxW) {
      const s = maxW / Math.sqrt(w2);
      rb.wx *= s; rb.wy *= s; rb.wz *= s;
    }
  }

  /** 重力、外力、阻尼更新速度 */
  private applyForces(rb: RigidBody, dt: number): void {
    const linScale = fastInvExp(dt * rb.linearDamping);
    const angScale = fastInvExp(dt * rb.angularDamping);
    const gs = rb.gravityScale, invMass = rb.invMass;
    const laX = this.gravityX * gs + rb.fx * invMass;
    const laY = this.gravityY * gs + rb.fy * invMass;
    const laZ = this.gravityZ * gs + rb.fz * invMass;
    const aaX = rb.ii00 * rb.tx + rb.ii01 * rb.ty + rb.ii02 * rb.tz;
    const aaY = rb.ii10 * rb.tx + rb.ii11 * rb.ty + rb.ii12 * rb.tz;
    const aaZ = rb.ii20 * rb.tx + rb.ii21 * rb.ty + rb.ii22 * rb.tz;
    rb.vx = (rb.vx + laX * dt) * linScale;
    rb.vy = (rb.vy + laY * dt) * linScale;
    rb.vz = (rb.vz + laZ * dt) * linScale;
    rb.wx = (rb.wx + aaX * dt) * angScale;
    rb.wy = (rb.wy + aaY * dt) * angScale;
    rb.wz = (rb.wz + aaZ * dt) * angScale;
  }
}
