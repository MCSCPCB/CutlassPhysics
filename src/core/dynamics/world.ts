// 移植自 OimoPhysics src/oimo/dynamics/World.hx（MIT，见 Credits.md）
import type { BroadPhase, BroadPhaseProxyCallback } from "../collision/broadphase/broad-phase.js";
import { BruteForceBroadPhase } from "../collision/broadphase/brute-force-broad-phase.js";
import { BvhBroadPhase } from "../collision/broadphase/bvh-broad-phase.js";
import type { Proxy } from "../collision/broadphase/proxy.js";
import { Aabb } from "../collision/geometry/aabb.js";
import type { ConvexGeometry } from "../collision/geometry/geometry.js";
import { RayCastHit } from "../collision/geometry/ray-cast-hit.js";
import { GjkEpa } from "../collision/narrowphase/gjkepa/gjk-epa.js";
import { Transform } from "../common/transform.js";
import { Vec3 } from "../common/vec3.js";
import { DEFAULT_GRAVITY_Y } from "../settings.js";
import type { AabbTestCallback } from "./callback/aabb-test-callback.js";
import type { RayCastCallback } from "./callback/ray-cast-callback.js";
import type { ConstraintSolver } from "./constraint/constraint-solver.js";
import type { Joint } from "./constraint/joint/joint.js";
import { Softness } from "./constraint/solver/soft-contact-solver.js";
import { ContactManager } from "./contact-manager.js";
import { Island, SoftSubstepParams } from "./island.js";
import { RigidBodyType } from "./rigidbody/rigid-body-config.js";
import type { RigidBody } from "./rigidbody/rigid-body.js";
import type { Shape } from "./rigidbody/shape.js";
import { TimeStep } from "./time-step.js";

/** 宽相算法，编号和 Oimo 一致 */
export const BroadPhaseType = { BRUTE_FORCE: 1, BVH: 2 } as const;

/** 物理世界：管理刚体和关节，每步做碰撞检测和求解 */
export class World {
  rigidBodyList: RigidBody | null = null;
  rigidBodyListLast: RigidBody | null = null;
  jointList: Joint | null = null;
  jointListLast: Joint | null = null;
  readonly broadPhase: BroadPhase;
  readonly contactManager: ContactManager;
  numRigidBodies = 0;
  numJoints = 0;
  numShapes = 0;
  numIslands = 0;
  numVelocityIterations = 10;
  numPositionIterations = 5;
  /**
   * 是否使用软接触求解，默认为 false。
   * 开启后接触使用 softIterations，关节保留独立的速度和位置迭代设置。
   */
  softContacts = false;
  /** 软接触带偏置阶段的迭代次数。 */
  softIterations = 3;
  /** 软接触：积分之后松弛解几遍 */
  softRelaxIterations = 3;
  /** 冲量变化总量低于此值时结束本阶段迭代；0 表示固定遍数。 */
  softTolerance = 0;
  /** 低速且无关节的接触岛减少常规迭代，受力或速度增大时恢复。 */
  softAdaptiveIterations = false;
  /**
   * 撞击迭代次数。子步模式下，超出常规遍数的部分仅处理被撞刚体的接触。
   * 单步模式将此值加到松弛遍数上。
   */
  softImpactIterations = 4;
  /** 软接触：靠近速度超过它（m/s）算撞击 */
  softImpactVelocity = 1.5;
  /** 软接触，多个子步时：子步结束后反弹解几遍；靠近速度达到多少（m/s）才反弹 */
  softRestitutionIterations = 6;
  softRestitutionThreshold = 1;
  /** 软接触，多个子步时：每个 tick 结束时的速度上限（m/s）、角速度上限（rad/s） */
  softMaxLinearVelocity = 60;
  softMaxAngularVelocity = 30;
  /** 动态接触频率上限，单位为 Hz，实际取 min(上限, softHertzStepRatio / dt)。 */
  softContactHertz = 30;
  /** 软接触：接触频率最多是子步频率的几倍 */
  softHertzStepRatio = 0.25;
  /** 软接触：一边不会动（地面）时的接触频率是动态之间的几倍 */
  softFixedHertzScale = 2;
  /** 软接触：阻尼比 */
  softDampingRatio = 10;
  /** 软接触：推开穿透的最大速度（m/s） */
  softMaxPushVelocity = 3;
  /**
   * 每次 step 的软接触子步数，0 表示单步。
   * 正值使碰撞检测和接触准备每步仅执行一次，子步执行求解和积分；仅在 softContacts 开启时有效。
   */
  softSubsteps = 0;
  /** 大质量比接触岛的子步数下限，0 表示不单独调整。 */
  softSupportSubsteps = 0;
  /** 最近一次步进中各岛使用的最大子步数。 */
  lastMaxSubsteps = 0;
  /**
   * 预测接触的基础距离，单位为米，仅在 softSubsteps > 0 时生效。
   * 静态接触按形状运动范围扩展，动态接触按受限的相对运动速度扩展，上限为 maxSpeculativeDistance。
   */
  speculativeDistance = 0.04;
  /** 预测接触距离的上限，单位为米。 */
  maxSpeculativeDistance = Infinity;
  /** 动态接触中用于计算预测距离的相对运动速度上限。 */
  pairSpeculativeSpeed = 1.5;
  /**
   * 虚空海参数，null 表示关闭，仅在 softSubsteps > 0 时有效。
   * 低于 startY 时施加深度相关的反重力冲量及速度阻尼。
   */
  voidSea: { startY: number; depthGradient: number; drag: number } | null = null;
  /**
   * 静态接触面的法向偏移，单位为米，仅在 softSubsteps > 0 时生效。
   * 用于控制地形接缝处的接触余量，不作用于两个动态刚体之间的接触。
   */
  softContactSkin = 0.012;
  private readonly softDynamic = new Softness();
  private readonly softFixed = new Softness();
  /** 软接触：每步翻转一次，让求解顺序在相邻两步之间交替 */
  private softParity = 0;
  /** 可选的毫秒时钟，供子步耗时统计使用，不影响模拟结果。 */
  clock: (() => number) | null = null;
  /**
   * 上一步所有活动岛和孤立刚体的子步循环耗时，单位为毫秒。
   * 不含碰撞检测及步进前后的准备、同步；未提供 clock 或未启用子步时为 0。
   */
  lastSubstepMs = 0;
  private readonly subTimeStep = new TimeStep();
  private readonly substepParams = new SoftSubstepParams(this.softDynamic, this.softFixed);
  /** 步进编号，供接触判断本步是否参与求解。 */
  private stepStamp = 0;
  /** 重力加速度（m/s²） */
  gravityX = 0;
  gravityY = DEFAULT_GRAVITY_Y;
  gravityZ = 0;
  private readonly timeStep = new TimeStep();
  private readonly island = new Island();
  private readonly rigidBodyStack: RigidBody[] = [];
  private readonly solversInIslands: ConstraintSolver[] = [];
  private shapeIdCount = 0;
  private readonly rayCastWrapper = new RayCastWrapper();
  private readonly convexCastWrapper = new ConvexCastWrapper();
  private readonly aabbTestWrapper = new AabbTestWrapper();
  /** wakeTouched 复用的包围盒和回调。 */
  private readonly wakeAabb = new Aabb();
  private readonly wakeCallback: AabbTestCallback = {
    process(shape: Shape): void {
      const b = shape.rigidBody!;
      if (b.sleeping && b.type !== RigidBodyType.STATIC && !b.wakeLocked) b.wakeUp();
    },
  };

  constructor(broadPhaseType: number = BroadPhaseType.BVH) {
    this.broadPhase = broadPhaseType === BroadPhaseType.BRUTE_FORCE ? new BruteForceBroadPhase() : new BvhBroadPhase();
    this.contactManager = new ContactManager(this.broadPhase);
  }

  /** 前进 dt 秒：宽相 → 窄相 → 按岛求解 */
  step(dt: number): void {
    const ts = this.timeStep;
    if (ts.dt > 0) ts.dtRatio = dt / ts.dt;
    ts.dt = dt;
    ts.invDt = 1 / dt;
    const substepped = this.softContacts && this.softSubsteps > 0;
    this.contactManager.speculative = substepped;
    if (substepped) this.wakeContacts(dt);
    this.contactManager.updateContacts();
    this.contactManager.updateManifolds();
    this.clearRestless();
    if (substepped) {
      this.stepStamp++;
      this.solveIslandsSubstepped();
      this.contactManager.updateTouching(this.stepStamp);
    } else {
      this.solveIslands();
    }
  }

  addRigidBody(rb: RigidBody): void {
    if (rb.world !== null) throw new Error("刚体不能同时属于多个世界");
    if (this.rigidBodyList === null) {
      this.rigidBodyList = rb;
      this.rigidBodyListLast = rb;
    } else {
      this.rigidBodyListLast!.next = rb;
      rb.prev = this.rigidBodyListLast;
      this.rigidBodyListLast = rb;
    }
    rb.world = this;
    for (let s = rb.shapeList; s !== null; s = s.next) this.addShape(s);
    this.numRigidBodies++;
    // 子步模式下，新增刚体的形状立即包含预测接触距离。
    if (rb.type !== RigidBodyType.STATIC && this.speculativeTick() > 0) rb.syncShapes();
  }

  /**
   * 移除刚体及其形状和接触，关联关节不自动移除。
   * wakeContacts=false 时保留接触另一侧刚体的休眠状态。
   */
  removeRigidBody(rb: RigidBody, wakeContacts = true): void {
    if (rb.world !== this) throw new Error("刚体不属于这个世界");
    const prev = rb.prev, next = rb.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (rb === this.rigidBodyList) this.rigidBodyList = next;
    if (rb === this.rigidBodyListLast) this.rigidBodyListLast = prev;
    rb.next = null;
    rb.prev = null;
    rb.world = null;
    for (let s = rb.shapeList; s !== null; s = s.next) this.removeShape(s, wakeContacts);
    this.numRigidBodies--;
  }

  /** 加入关节，并唤醒它连着的两个刚体。两个刚体要先加入世界 */
  addJoint(joint: Joint): void {
    if (joint.world !== null) throw new Error("关节不能同时属于多个世界");
    if (this.jointList === null) {
      this.jointList = joint;
      this.jointListLast = joint;
    } else {
      this.jointListLast!.next = joint;
      joint.prev = this.jointListLast;
      this.jointListLast = joint;
    }
    joint.world = this;
    joint.attachLinks();
    joint.syncAnchors();
    this.numJoints++;
  }

  /** 移除关节，并唤醒它连着的两个刚体。关节被拉断时也会自动移除 */
  removeJoint(joint: Joint): void {
    if (joint.world !== this) throw new Error("关节不属于这个世界");
    const prev = joint.prev, next = joint.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (joint === this.jointList) this.jointList = next;
    if (joint === this.jointListLast) this.jointListLast = prev;
    joint.next = null;
    joint.prev = null;
    joint.world = null;
    joint.detachLinks();
    this.numJoints--;
  }

  /**
   * 逐个回调射线命中的形状，顺序与距离无关。
   * 回调期间不支持增删刚体、形状或嵌套查询。最近命中查询使用 RayCastClosest。
   */
  rayCast(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    callback: RayCastCallback,
  ): void {
    const w = this.rayCastWrapper;
    w.beginX = beginX;
    w.beginY = beginY;
    w.beginZ = beginZ;
    w.endX = endX;
    w.endY = endY;
    w.endZ = endZ;
    w.callback = callback;
    this.broadPhase.rayCast(beginX, beginY, beginZ, endX, endY, endZ, w);
  }

  /**
   * 沿平移量扫掠凸体，并逐个回调命中结果；初始重叠不报告。
   * hit 包含比例、碰撞点和法线，回调顺序与距离无关。回调期间不支持修改世界或嵌套查询。
   */
  convexCast(convex: ConvexGeometry, begin: Transform, tx: number, ty: number, tz: number, callback: RayCastCallback): void {
    const w = this.convexCastWrapper;
    w.convex = convex;
    w.begin.copyFrom(begin);
    w.translation.init(tx, ty, tz);
    w.callback = callback;
    this.broadPhase.convexCast(convex, begin, w.translation, w);
  }

  /**
   * 逐个回调包围盒与 aabb 重叠的形状。
   * 查询包含形状的扫掠范围；回调期间不支持修改世界或嵌套查询。
   */
  aabbTest(aabb: Aabb, callback: AabbTestCallback): void {
    const w = this.aabbTestWrapper;
    w.aabb.copyFrom(aabb);
    w.callback = callback;
    this.broadPhase.aabbTest(aabb, w);
  }

  /** 形状加入世界：在宽相里创建代理 */
  addShape(shape: Shape): void {
    shape.proxy = this.broadPhase.createProxy(shape, shape.aabb);
    shape.id = this.shapeIdCount++;
    this.numShapes++;
  }

  /** 移除形状的代理及相关接触；wakeContacts 控制是否唤醒接触另一侧的刚体。 */
  removeShape(shape: Shape, wakeContacts = true): void {
    this.broadPhase.destroyProxy(shape.proxy!);
    shape.proxy = null;
    shape.id = -1;
    let cl = shape.rigidBody!.contactLinkList;
    while (cl !== null) {
      const next = cl.next;
      const c = cl.contact!;
      if (c.s1 === shape || c.s2 === shape) {
        if (wakeContacts) cl.other!.wakeUp();
        this.contactManager.destroyContact(c);
      }
      cl = next;
    }
    this.numShapes--;
  }

  private solveIslands(): void {
    const island = this.island;
    island.setGravity(this.gravityX, this.gravityY, this.gravityZ);
    this.numIslands = 0;
    const soft = this.softContacts;
    if (soft) {
      this.softParity ^= 1;
      const h = this.timeStep.dt;
      const cap = this.softHertzStepRatio / h;
      this.softDynamic.set(Math.min(this.softContactHertz, cap), this.softDampingRatio, h);
      this.softFixed.set(Math.min(this.softContactHertz * this.softFixedHertzScale, cap * this.softFixedHertzScale), this.softDampingRatio, h);
    }
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      // 已经在岛里的、休眠的、静态的刚体不能作为岛的起点
      if (b.addedToIsland || b.sleeping || b.type === RigidBodyType.STATIC) continue;
      if (b.isAlone()) {
        island.stepSingleRigidBody(this.timeStep, b);
      } else {
        this.buildIsland(b);
        if (soft) {
          island.stepSoft(
            this.timeStep, this.softIterations, this.softRelaxIterations, this.softImpactIterations, this.softImpactVelocity,
            this.softDynamic, this.softFixed, this.softMaxPushVelocity, this.numVelocityIterations, this.numPositionIterations, this.softParity,
          );
        } else {
          island.step(this.timeStep, this.numVelocityIterations, this.numPositionIterations);
        }
        island.clear();
      }
      this.numIslands++;
    }
    this.finishIslands();
  }

  /** 清掉岛的标记，以及这一步的力和力矩 */
  private finishIslands(): void {
    for (let b = this.rigidBodyList; b !== null; b = b.next) b.addedToIsland = false;
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      b.fx = 0; b.fy = 0; b.fz = 0;
      b.tx = 0; b.ty = 0; b.tz = 0;
    }
    const solvers = this.solversInIslands;
    for (let i = 0; i < solvers.length; i++) solvers[i].addedToIsland = false;
    solvers.length = 0;
  }

  /**
   * 返回子步模式的 tick 时长，供形状计算预测距离。
   * 尚未推进时为 1/20 秒；未启用 softContacts 或 softSubsteps 时为 0。
   */
  speculativeTick(): number {
    if (!this.softContacts || this.softSubsteps <= 0) return 0;
    return this.timeStep.dt > 0 ? this.timeStep.dt : 0.05;
  }

  /** 窄相检测后清除有接触刚体的 restless 标记，未接触刚体保留流体不平衡标记。 */
  private clearRestless(): void {
    for (let c = this.contactManager.contactList; c !== null; c = c.next) {
      // 子步模式下 touching 要到求解之后才更新，这里按窄相刚给出的接触点判断（含预测接触点，和接触约束用的一样）
      if (c.manifold.numPoints === 0) continue;
      if (c.b1 !== null) c.b1.restless = false;
      if (c.b2 !== null) c.b2.restless = false;
    }
  }

  /** 碰撞检测前按当前速度同步 speculativeStale 刚体的预测距离和包围盒。 */
  private syncStaleBodies(t: number): void {
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.speculativeStale && b.type !== RigidBodyType.STATIC) b.syncShapesSpeculative(t, this.speculativeDistance, this.maxSpeculativeDistance);
    }
  }

  /** 在地形请求之前唤醒可能受撞击的刚体，使其支撑碰撞体在本步恢复。 */
  wakeContacts(dt: number): void {
    if (!this.softContacts || this.softSubsteps <= 0) return;
    this.syncStaleBodies(dt);
    this.wakeTouched();
  }

  /**
   * 唤醒与高速活动刚体包围盒重叠的休眠刚体，跳过 wakeLocked 对象。
   * 速度门槛为自身休眠阈值的两倍；包围盒包含预测接触距离。
   */
  private wakeTouched(): void {
    let sleepers = false;
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.sleeping && b.type !== RigidBodyType.STATIC && !b.wakeLocked) { sleepers = true; break; }
    }
    if (!sleepers) return;
    const a = this.wakeAabb, callback = this.wakeCallback;
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.sleeping || b.type === RigidBodyType.STATIC || b.shapeList === null) continue;
      const lv = b.sleepingVelocityThreshold * 2, av = b.sleepingAngularVelocityThreshold * 2;
      if (!(b.vx * b.vx + b.vy * b.vy + b.vz * b.vz > lv * lv || b.wx * b.wx + b.wy * b.wy + b.wz * b.wz > av * av)) continue;
      // 整个刚体的包围盒：各形状包围盒的并集
      a.copyFrom(b.shapeList.aabb);
      for (let s = b.shapeList.next; s !== null; s = s.next) {
        const t = s.aabb;
        if (t.minX < a.minX) a.minX = t.minX;
        if (t.minY < a.minY) a.minY = t.minY;
        if (t.minZ < a.minZ) a.minZ = t.minZ;
        if (t.maxX > a.maxX) a.maxX = t.maxX;
        if (t.maxY > a.maxY) a.maxY = t.maxY;
        if (t.maxZ > a.maxZ) a.maxZ = t.maxZ;
      }
      this.aabbTest(a, callback);
    }
  }

  /** 按岛执行软接触子步求解。 */
  private solveIslandsSubstepped(): void {
    const island = this.island;
    island.setGravity(this.gravityX, this.gravityY, this.gravityZ);
    this.numIslands = 0;
    const n = this.softSubsteps;
    this.lastMaxSubsteps = n;
    const h = this.timeStep.dt / n;
    const sub = this.subTimeStep;
    // 子步步长变了（子步数或 dt 变了）：第一个子步按比例缩放上一步留下的累积冲量
    const firstRatio = sub.dt > 0 ? h / sub.dt : 1;
    sub.dt = h;
    sub.invDt = 1 / h;
    const cap = this.softHertzStepRatio / h;
    this.softDynamic.set(Math.min(this.softContactHertz, cap), this.softDampingRatio, h);
    this.softFixed.set(Math.min(this.softContactHertz * this.softFixedHertzScale, cap * this.softFixedHertzScale), this.softDampingRatio, h);
    const p = this.substepParams;
    p.substeps = n;
    p.iterations = this.softIterations;
    p.relaxIterations = this.softRelaxIterations;
    p.tolerance = this.softTolerance;
    p.adaptiveIterations = this.softAdaptiveIterations;
    p.impactIterations = this.softImpactIterations;
    p.impactVelocity = this.softImpactVelocity;
    p.jointIterations = this.numVelocityIterations;
    p.positionIterations = this.numPositionIterations;
    p.maxPushVelocity = this.softMaxPushVelocity;
    p.restitutionIterations = this.softRestitutionIterations;
    p.restitutionThreshold = this.softRestitutionThreshold;
    p.maxLinearVelocity = this.softMaxLinearVelocity;
    p.maxAngularVelocity = this.softMaxAngularVelocity;
    const sea = this.voidSea;
    p.sea = sea !== null;
    if (sea !== null) {
      p.seaStartY = sea.startY;
      p.seaDepthGradient = sea.depthGradient;
      p.seaDrag = sea.drag;
    }
    p.parity = this.softParity;
    p.stamp = this.stepStamp;
    p.speculativeDistance = this.speculativeDistance;
    p.maxSpeculativeDistance = this.maxSpeculativeDistance;
    p.skin = this.softContactSkin;
    const clock = this.clock;
    const tick = this.timeStep;
    let minMass = Infinity;
    if (n < this.softSupportSubsteps) {
      for (let b = this.rigidBodyList; b !== null; b = b.next) {
        if (b.type === RigidBodyType.DYNAMIC && b.mass > 0 && b.mass < minMass) minMass = b.mass;
      }
    }
    let substepMs = 0;
    // 只给子步本身计时（run 那一段），每个 tick 只做一次的 begin、finish 不算进去
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.addedToIsland || b.sleeping || b.type === RigidBodyType.STATIC) continue;
      const alone = b.isAlone();
      if (!alone) this.buildIsland(b, true);
      // 较重的自由落体保留积分精度，避免预算切换改变入射时刻。
      const count = alone ? (b.mass >= minMass * 8 ? Math.max(n, this.softSupportSubsteps) : n) : island.supportSubsteps(n, this.softSupportSubsteps);
      if (count !== p.substeps) {
        p.substeps = count;
        const dt = tick.dt / count, limit = this.softHertzStepRatio / dt;
        sub.dt = dt; sub.invDt = 1 / dt;
        this.softDynamic.set(Math.min(this.softContactHertz, limit), this.softDampingRatio, dt);
        this.softFixed.set(Math.min(this.softContactHertz * this.softFixedHertzScale, limit * this.softFixedHertzScale), this.softDampingRatio, dt);
      }
      if (count > this.lastMaxSubsteps) this.lastMaxSubsteps = count;
      sub.dtRatio = firstRatio;
      if (alone) {
        if (island.beginSingleRigidBodySubsteps(tick, b)) {
          const t0 = clock === null ? 0 : clock();
          island.runSingleRigidBodySubsteps(sub, p, b);
          if (clock !== null) substepMs += clock() - t0;
          island.finishSingleRigidBodySubsteps(tick, p, b);
        }
      } else {
        if (island.beginSubsteps(tick, sub, p)) {
          const t0 = clock === null ? 0 : clock();
          island.runSubsteps(sub, p);
          if (clock !== null) substepMs += clock() - t0;
          island.finishSubsteps(tick, p);
        }
        island.clear();
      }
      this.numIslands++;
    }
    this.lastSubstepMs = substepMs;
    // 每个 tick 使用相同的起始顺序，子步内交替正反序。
    sub.dtRatio = 1;
    // 保存基础步长；接触与关节分别记录各自的热启动步长。
    sub.dt = h;
    sub.invDt = 1 / h;
    this.finishIslands();
  }

  /**
   * 从 base 沿接触和关节深度优先构建模拟岛。
   * 预测模式包含尚未接触的流形点；休眠刚体不加入岛，接触求解将其逆质量视为 0。
   */
  private buildIsland(base: RigidBody, speculative = false): void {
    const island = this.island, stack = this.rigidBodyStack;
    island.addRigidBody(base);
    stack.push(base);
    while (stack.length > 0) {
      const rb = stack.pop()!;
      // 静态刚体不往下找
      if (rb.type === RigidBodyType.STATIC) continue;
      for (let cl = rb.contactLinkList; cl !== null; cl = cl.next) {
        const cc = cl.contact!.contactConstraint, ccs = cc.solver;
        if ((speculative ? cc.manifold.numPoints > 0 : cc.isTouching()) && !ccs.addedToIsland) {
          this.solversInIslands.push(ccs);
          if (this.softContacts) island.addSoftContact(cc.softSolver, ccs);
          else island.addConstraintSolver(ccs, cc.positionCorrectionAlgorithm);
          const other = cl.other!;
          if (speculative && other.sleeping && other.type !== RigidBodyType.STATIC) continue;
          if (!other.addedToIsland) {
            island.addRigidBody(other);
            stack.push(other);
          }
        }
      }
      // 关节不管有没有接触都算连着
      for (let jl = rb.jointLinkList; jl !== null; jl = jl.next) {
        const j = jl.joint, js = j.solver;
        if (!js.addedToIsland) {
          this.solversInIslands.push(js);
          island.addConstraintSolver(js, j.positionCorrectionAlgorithm);
          const other = jl.other!;
          if (!other.addedToIsland) {
            island.addRigidBody(other);
            stack.push(other);
          }
        }
      }
    }
  }
}

/** 宽相粗筛出的代理，再用形状的几何体做精确求交 */
class RayCastWrapper implements BroadPhaseProxyCallback {
  readonly rayCastHit = new RayCastHit();
  beginX = 0;
  beginY = 0;
  beginZ = 0;
  endX = 0;
  endY = 0;
  endZ = 0;
  callback: RayCastCallback | null = null;

  process(proxy: Proxy): void {
    const shape = proxy.userData as Shape;
    if (shape.geom.rayCast(this.beginX, this.beginY, this.beginZ, this.endX, this.endY, this.endZ, shape.transform, this.rayCastHit)) {
      this.callback!.process(shape, this.rayCastHit);
    }
  }
}

/** 宽相粗筛出的代理，再用 GJK 做精确的凸体扫掠 */
class ConvexCastWrapper implements BroadPhaseProxyCallback {
  readonly rayCastHit = new RayCastHit();
  readonly begin = new Transform();
  readonly translation = new Vec3();
  private readonly zero = new Vec3();
  convex: ConvexGeometry | null = null;
  callback: RayCastCallback | null = null;

  process(proxy: Proxy): void {
    const shape = proxy.userData as Shape;
    const type = shape.geom.type;
    if (type < 0 || type > 5) return;
    const hit = this.rayCastHit;
    if (GjkEpa.getInstance().convexCast(this.convex!, shape.geom as ConvexGeometry, this.begin, shape.transform, this.translation, this.zero, hit)) {
      this.callback!.process(shape, hit);
    }
  }
}

/** BVH 的代理包围盒是加厚过的，这里再用形状自己的包围盒过滤一次 */
class AabbTestWrapper implements BroadPhaseProxyCallback {
  readonly aabb = new Aabb();
  callback: AabbTestCallback | null = null;

  process(proxy: Proxy): void {
    const shape = proxy.userData as Shape;
    const s = shape.aabb, a = this.aabb;
    if (s.minX < a.maxX && s.maxX > a.minX && s.minY < a.maxY && s.maxY > a.minY && s.minZ < a.maxZ && s.maxZ > a.minZ) {
      this.callback!.process(shape);
    }
  }
}
