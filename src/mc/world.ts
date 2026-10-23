// 物理世界。
import {
  BlockTypes, BlockVolume, ListBlockVolume, type Dimension, LocationInUnloadedChunkError, LocationOutOfWorldBoundariesError, system, UnloadedChunksError, world,
} from "@minecraft/server";
import { Aabb } from "../core/collision/geometry/aabb.js";
import { RayCastHit as CoreRayCastHit } from "../core/collision/geometry/ray-cast-hit.js";
import { Transform } from "../core/common/transform.js";
import type { AabbTestCallback } from "../core/dynamics/callback/aabb-test-callback.js";
import { RayCastClosest } from "../core/dynamics/callback/ray-cast-closest.js";
import type { RayCastCallback } from "../core/dynamics/callback/ray-cast-callback.js";
import type { Contact } from "../core/dynamics/contact.js";
import type { ContactListener } from "../core/dynamics/contact-manager.js";
import { RigidBodyConfig, RigidBodyType } from "../core/dynamics/rigidbody/rigid-body-config.js";
import { RigidBody } from "../core/dynamics/rigidbody/rigid-body.js";
import type { Shape } from "../core/dynamics/rigidbody/shape.js";
import { World } from "../core/dynamics/world.js";
import { VoxelCollider } from "../world/voxel-collider.js";
import { bodyBounds } from "../world/terrain-scan.js";
import { FluidForces, type FluidBody, type FluidEntry, type FluidTerrain } from "../world/fluid-forces.js";
import { FLUID_LAVA, FLUID_WATER } from "../world/block-shapes.js";
import { FluidGrid } from "../world/fluid-grid.js";
import { Body, type BodyOptions } from "./body.js";
import { BlockResolver, type BlockReadError, type BlockSourceHost, DimensionBlockSource } from "./block-source.js";
import { watchBlockEvents } from "./block-events.js";
import { ExplosionQueue, type OcclusionTest, watchExplosions, worldOcclusion } from "./explosions.js";
import { createBlockShapeTable } from "./block-shapes.js";
import { isConvex, makeGeometry, makeJoint, makeShape, matrixToQuaternion, quaternionToMatrix } from "./convert.js";
import type { FluidEntryEvent, PhysicsAfterEvents } from "./events.js";
// 仅在 {@link} 注释中引用，用于让 TypeDoc 解析链接
// eslint-disable-next-line @typescript-eslint/no-unused-vars
import type { Signal } from "./events.js";
import { applyFloating, floatingClusters, type FloatingCluster, type FloatingPoint } from "./floating.js";
import { getBlockPhysics, registeredPhysics, type BlockPhysics } from "./block-physics.js";
import { BlockRegistry, type BlockDefinitions } from "./block-registry.js";
import { Joint, type JointOptions } from "./joints.js";
import type { Quaternion, Vector3 } from "./math.js";
import { DEFAULT_MATERIALS, materialFriction, terrainFriction } from "./materials.js";
import { createAfterEvents, type SignalImpl } from "./signal.js";
import type { BodyBlock, ShapeOptions } from "./shapes.js";
import {
  captureBody, captureJoint, checkCapturedBodies, copyJson, readSnapshot, restoreContents, type PhysicsSnapshot, type RestoreOptions, type SnapshotOptions, type SnapshotShapeCache,
} from "./snapshot.js";
import type { TerrainOptions } from "./terrain.js";

/**
 * 物理世界的时间推进方式。
 *
 * @remarks
 * `"auto"` 表示每个 tick 自动推进一次，内部使用 `system.runInterval`。`"manual"` 表示不自动推进，需要调用 {@link PhysicsWorld.step}。
 */
export type StepMode = "auto" | "manual";

/** 虚空海的参数，见 {@link PhysicsWorldOptions.voidSea} */
export interface VoidSeaOptions {
  /** 虚空海开始施加作用力的高度。 */
  startY: number;
  /** 深度系数，默认 `1` */
  depthGradient?: number;
  /** 阻力系数，默认 `1` */
  drag?: number;
}

/**
 * 流体的参数，见 {@link PhysicsWorldOptions.fluids}。没给的项用默认值。
 */
export interface FluidOptions {
  /** 水的密度，单位为质量/格³，默认为 10.5 / 11。 */
  waterDensity?: number;
  /** 岩浆的密度。默认 `3` */
  lavaDensity?: number;
  /** 和速度平方成正比的阻力系数。默认 `0.5` */
  drag?: number;
  /** 和速度成正比的阻力系数。默认 `0.5` */
  linearDrag?: number;
  /** 岩浆的阻力是水的多少倍。默认 `6` */
  lavaViscosity?: number;
  /** 抑制液面竖直振荡的阻尼，默认为 0.5。 */
  waveDamping?: number;
  /** 是否计算空心船体托住的空气。默认 `true` */
  airPockets?: boolean;
  /** 船体进水、排水的速度倍率。默认 `1` */
  floodRate?: number;
  /** 水流推动刚体的速度，单位为格/秒；`0` 关闭水流。默认 `1.4` */
  currentSpeed?: number;
}

/**
 * 创建 {@link PhysicsWorld} 时使用的选项。
 */
export interface PhysicsWorldOptions {
  /**
   * 世界的重力，单位为格/秒²。
   *
   * @remarks
   * 默认值为 `{ x: 0, y: -11, z: 0 }`。
   */
  gravity?: Vector3;
  /**
   * 世界底部的虚空海。
   *
   * @remarks
   * 位置低于 startY 时，按深度施加反重力冲量及阻力；深度系数乘以深度为 1 时抵消重力。
   * 默认 startY 为 -105，depthGradient 和 drag 为 1，各维度相同。false 表示关闭。
   */
  voidSea?: VoidSeaOptions | false;
  /**
   * 原版爆炸是否推动刚体，默认为 true。
   *
   * @remarks
   * 爆炸冲量在下一步施加，随距离衰减；方块遮挡时保留 35%。
   * 一批冲量的速度变化上限为 12 格/秒，角速度变化上限为 2.5 弧度/秒。
   */
  explosions?: boolean;
  /**
   * 是否破坏被刚体撞碎的世界方块，默认为 true。
   *
   * @remarks
   * 撞击达到 fragileImpactSpeed 时触发 blockBreak，并在下一 tick 破坏方块、掉落物品。
   * false 仅保留事件。刚体上的方块始终只触发事件，不自动移除形状。
   */
  breakFragileBlocks?: boolean;
  /**
   * 流体交互选项，默认开启。
   *
   * @remarks
   * 仅在启用地形时生效，包含浮力、阻力、水流和气腔。参数见 {@link FluidOptions}。
   * 浮力作用于浸没部分的形心；未平衡且无接触的刚体保持活动。false 表示关闭。
   */
  fluids?: FluidOptions | false;
  /**
   * 世界的时间推进方式。
   *
   * @remarks
   * 默认值为 `"auto"`。
   */
  stepMode?: StepMode;
  /**
   * 每次推进的子步数，默认为 4。
   *
   * @remarks
   * 每步仅检测一次碰撞，子步执行求解和积分。
   * 连续 3 次求解超过预算时减少 1 个子步，最少 2 个；连续 100 次低于预算三分之一时增加 1 个，至设定值。
   * 调整子步数不改变单次推进的总时间。
   */
  substeps?: number;
  /**
   * 每次推进的目标求解耗时，单位为毫秒，默认为 12。
   *
   * @remarks
   * 作为子步调整依据，不是耗时上限；计入流体和悬浮计算，不计方块读取。
   * 必须大于 0，Infinity 禁用预算调整。规则见 {@link PhysicsWorldOptions.substeps}。
   */
  timeBudgetMs?: number;
  /**
   * 地形碰撞的选项。
   *
   * @remarks
   * 省略时，世界中没有方块。
   */
  terrain?: TerrainOptions;
  /**
   * 方块形状与物理属性的登记表。
   *
   * @remarks
   * 作用于地形、刚体方块及易碎判定，并随存档保存。{@link blockShapes} 使用相同登记表时解析结果一致。
   * 格式见 {@link BlockDefinitions}。
   */
  blocks?: BlockDefinitions;
}

/**
 * {@link PhysicsWorld.rayCast}、{@link PhysicsWorld.rayCastAll} 和 {@link PhysicsWorld.sweep} 使用的选项。
 */
export interface RayCastOptions {
  /**
   * 最大检测距离，单位为格，须为正的有限数。
   *
   * @remarks
   * 射线默认值为 100。扫掠默认使用 translation 的长度，指定后忽略超出距离的命中。
   */
  maxDistance?: number;
  /**
   * 是否检测方块。
   *
   * @remarks
   * 默认值为 `true`。
   */
  includeBlocks?: boolean;
  /**
   * 是否检测刚体。
   *
   * @remarks
   * 默认值为 `true`。
   */
  includeBodies?: boolean;
  /**
   * 碰撞掩码。仅检测碰撞分组与该掩码有交集的形状。
   *
   * @remarks
   * 省略时检测所有形状。
   */
  collisionMask?: number;
  /**
   * 要忽略的刚体，例如发射者自身。
   */
  excludeBodies?: Body[];
}

/**
 * 射线检测或扫掠的命中结果。
 */
export interface RayCastHit {
  /**
   * 命中点，世界坐标。
   */
  point: Vector3;
  /**
   * 命中面的法线，指向射线来向的一侧。
   */
  normal: Vector3;
  /**
   * 起点到命中点的距离，单位为格。
   */
  distance: number;
  /**
   * 命中的刚体。
   *
   * @remarks
   * 命中方块时为 `undefined`。
   */
  body?: Body;
  /**
   * 命中的方块坐标。
   *
   * @remarks
   * 命中刚体时为 `undefined`。
   */
  blockLocation?: Vector3;
}

/**
 * {@link PhysicsWorld.queryBox} 使用的选项。
 */
export interface QueryOptions {
  /**
   * 是否返回刚体。
   *
   * @remarks
   * 默认值为 `true`。
   */
  includeBodies?: boolean;
  /**
   * 碰撞掩码。仅检测碰撞分组与该掩码有交集的形状。
   *
   * @remarks
   * 省略时检测所有形状。
   */
  collisionMask?: number;
}

/**
 * 包含刚体、关节和地形碰撞的物理世界。
 *
 * @remarks
 * 每个实例对应一个维度。创建时启动配置的运行模式，{@link PhysicsWorld.destroy} 释放相关资源。
 *
 * @example read-result.ts
 * {@includeCode ../../examples/getting-started/read-result.ts}
 */
export class PhysicsWorld {
  private readonly dimension: Dimension;
  private readonly core = new World();
  private readonly bodySet = new Set<Body>();
  private readonly jointSet = new Set<Joint>();
  private readonly terrainSettings: Omit<TerrainOptions, "collision"> | null;
  private readonly customCollision: boolean;
  /** 方块登记表；没有给 blocks 时为 null */
  private readonly registry: BlockRegistry | null;
  /** 易碎方块查询的速度下限，取内置阈值 4 与登记表最小阈值的较小值。 */
  private readonly fragileMinSpeed: number;
  /** 上一次通知过的睡眠状态，用来只在变化时发事件 */
  private readonly sleepState = new Map<Body, boolean>();
  /** 见 revision */
  private revisionValue = 0;
  private readonly events = createAfterEvents();
  private substepCount: number;
  /** 负载太高时实际用的子步数，以及连续超出预算、连续远低于预算的次数（规则见 PhysicsWorldOptions.substeps） */
  private activeSubsteps: number;
  private slowSteps = 0;
  private fastSteps = 0;
  private readonly collider: VoxelCollider | null = null;
  private readonly source: DimensionBlockSource | null = null;
  private readonly freezeOnUnloaded: boolean;
  private unwatchBlocks: (() => void) | null = null;
  /** 排队的爆炸，下一次推进前施加（见 explosions.ts） */
  private readonly explosionQueue = new ExplosionQueue();
  private occlusion: OcclusionTest | null = null;
  private unwatchExplosions: (() => void) | null = null;
  /** 流体计算。没有地形或 fluids 为 false 时为 null */
  private readonly fluids: FluidForces | null = null;
  /** 每个刚体的流体状态。第一次碰到流体时才建 */
  private readonly fluidBodies = new Map<Body, FluidRecord>();
  /** 带悬浮方块的刚体和它们的方块簇（见 floating.ts） */
  private readonly floatingBodies = new Map<Body, FloatingCluster[]>();
  /** 带易碎方块的刚体（见 onBeginContact） */
  private readonly fragileBodies = new Set<Body>();
  /** 撞碎世界里的易碎方块；已经排队、下一 tick 破坏的方块位置 */
  private readonly breakFragile: boolean;
  private readonly pendingBreaks = new Set<string>();
  private runId = -1;
  /** checkChunks 轮到第几个刚体 */
  private chunkCursor = 0;
  private validFlag = true;
  private totalSteps = 0;
  private tickCount = 0;
  private gravityValue: Vector3;
  private stepModeValue: StepMode = "manual";
  private timeBudget: number;
  private pausedValue = false;
  /** 步进过程中产生的事件，步进结束后再依次触发，避免订阅者在内核遍历接触时增删刚体 */
  private pending: (() => void)[] = [];
  private readonly contactListener: ContactListener = {
    beginContact: (c) => this.onBeginContact(c),
    endContact: (c) => this.onEndContact(c),
  };
  /**
   * 创建一个物理世界。
   *
   * @param dimension
   * 物理世界对应的维度。地形碰撞从该维度读取方块。
   * @param options
   * 物理世界的选项。
   *
   * @example create-world.ts
   * {@includeCode ../../examples/getting-started/create-world.ts}
   */
  constructor(dimension: Dimension, options: PhysicsWorldOptions = {}) {
    if (dimension === undefined || dimension === null) throw new Error("创建物理世界需要一个维度");
    this.dimension = dimension;
    const g = options.gravity ?? DEFAULT_GRAVITY;
    finite3("gravity", g);
    this.gravityValue = { x: g.x, y: g.y, z: g.z };
    this.core.gravityX = g.x; this.core.gravityY = g.y; this.core.gravityZ = g.z;
    const sea = options.voidSea === undefined ? { startY: VOID_SEA_START_Y } : options.voidSea;
    if (sea !== false) {
      const depthGradient = sea.depthGradient ?? 1, drag = sea.drag ?? 1;
      if (!Number.isFinite(sea.startY)) throw new Error(`voidSea.startY 必须是有限数，收到 ${sea.startY}`);
      if (!Number.isFinite(depthGradient)) throw new Error(`voidSea.depthGradient 必须是有限数，收到 ${depthGradient}`);
      if (!(drag >= 0) || !Number.isFinite(drag)) throw new Error(`voidSea.drag 必须是不小于 0 的有限数，收到 ${drag}`);
      this.core.voidSea = { startY: sea.startY, depthGradient, drag };
    }
    const substeps = options.substeps ?? DEFAULT_SUBSTEPS;
    if (!Number.isInteger(substeps) || substeps < 1) throw new Error(`substeps 必须是不小于 1 的整数，收到 ${substeps}`);
    this.timeBudget = checkBudget(options.timeBudgetMs ?? DEFAULT_TIME_BUDGET_MS);
    this.substepCount = substeps;
    this.activeSubsteps = substeps;
    // 接触用软接触求解（见 core/dynamics/constraint/solver/soft-contact-solver.ts），每个 tick 只检测一次碰撞，子步在内核里做，
    // 每个子步带偏置、松弛各解一遍；撞击、反弹、限速、休眠的参数见 core World 的 soft* 默认值。关节仍然用 PGS
    this.core.numVelocityIterations = SOLVER_VELOCITY_ITERATIONS;
    this.core.softContacts = true;
    this.core.softIterations = 1;
    this.core.softRelaxIterations = 1;
    this.core.softSubsteps = substeps;
    // 地形接触面内移 0.01 格，减少接缝处的抖动。
    this.core.softContactSkin = -0.01;
    this.core.contactManager.maxRestitution = true;
    this.core.contactManager.listener = this.contactListener;

    // 登记表在世界里另存一份校验过的副本：使用者之后改了原对象也不影响
    this.registry = options.blocks === undefined ? null : new BlockRegistry(options.blocks);
    this.fragileMinSpeed = Math.min(FRAGILE_MIN_SPEED, this.registry?.minFragileSpeed ?? Infinity);
    const t = options.terrain;
    this.customCollision = t?.collision !== undefined;
    this.terrainSettings = t === undefined ? null : Object.fromEntries(Object.entries(t).filter(([k, v]) => k !== "collision" && v !== undefined));
    if (t?.material !== undefined) this.terrainSettings!.material = { ...t.material };
    this.freezeOnUnloaded = (t?.unloadedChunkPolicy ?? "freeze") === "freeze";
    if (t !== undefined) {
      // 以公开选项名报告错误；仅接受可序列化的有限值。
      if (t.margin !== undefined && (!(t.margin >= 0) || !Number.isFinite(t.margin))) throw new Error(`margin 必须是不小于 0 的有限数，收到 ${t.margin}`);
      checkInteger("preloadTicks", t.preloadTicks, 0);
      checkInteger("maxChunksPerTick", t.maxChunksPerTick, 1);
      checkInteger("unloadDelayTicks", t.unloadDelayTicks, 0);
      checkInteger("revalidateBlocksPerTick", t.revalidateBlocksPerTick, 0);
      const table = createBlockShapeTable();
      const uniform = t.material;
      const resolver = new BlockResolver(table, {
        registry: this.registry ?? undefined, collision: t.collision, unknownBlock: t.unknownBlock, fixedMaterial: uniform === undefined ? undefined : 0,
      });
      this.source = new DimensionBlockSource(dimension, HOST, resolver);
      this.source.unloadedAsAir = !this.freezeOnUnloaded;
      this.collider = new VoxelCollider(this.core, this.source, {
        table,
        materials: uniform === undefined
          ? (this.registry?.materials ?? DEFAULT_MATERIALS).map((m) => ({ friction: terrainFriction(m), restitution: m.restitution }))
          : [{ friction: uniform.friction, restitution: uniform.restitution }],
        margin: t.margin,
        preloadTicks: t.preloadTicks,
        maxChunksPerUpdate: t.maxChunksPerTick,
        unloadDelay: t.unloadDelayTicks,
        revalidateBlocksPerUpdate: t.revalidateBlocksPerTick,
        suspend: this.freezeOnUnloaded,
        fluids: options.fluids !== false,
      });
      if (t.watchBlockEvents ?? true) {
        const collider = this.collider;
        this.unwatchBlocks = watchBlockEvents(dimension.id, (x0, y0, z0, x1, y1, z1, cachedOnly) => collider.blocksChanged(x0, y0, z0, x1, y1, z1, cachedOnly));
      }
    }
    if (options.explosions ?? true) {
      this.occlusion = worldOcclusion(dimension);
      this.unwatchExplosions = watchExplosions(world.afterEvents.explosion, dimension.id, this.explosionQueue);
    }
    if (options.fluids !== undefined && options.fluids !== false) checkFluids(options.fluids);
    if (this.collider !== null && options.fluids !== false) this.fluids = new FluidForces({ waterDensity: DEFAULT_WATER_DENSITY, ...options.fluids });
    if (options.breakFragileBlocks !== undefined && typeof options.breakFragileBlocks !== "boolean") {
      throw new Error(`breakFragileBlocks 必须是布尔值，收到 ${String(options.breakFragileBlocks)}`);
    }
    this.breakFragile = options.breakFragileBlocks ?? true;
    this.stepMode = options.stepMode ?? "auto";
  }

  private check(): void {
    if (!this.validFlag) throw new Error("物理世界已经销毁");
  }

  /** 方块的物理属性：内置数据，登记表覆盖 */
  private blockPhysics(typeId: string, states: Readonly<Record<string, boolean | number | string>>): BlockPhysics {
    return this.registry === null ? getBlockPhysics(typeId, states) : registeredPhysics(this.registry, typeId, states);
  }

  /**
   * 生成独立的刚体和关节快照，默认省略 userData。
   *
   * @remarks
   * 不包含地形缓存、事件订阅和求解缓存。冻结状态保留速度，恢复时重新请求地形。
   * 世界动态属性存储见 {@link PhysicsWorldStore}。
   *
   * @param options
   * 用 `encodeUserData` 把每个刚体的 userData 转成可存的 JSON 值；省略时不保存 userData。
   * @returns
   * 可以直接 `JSON.stringify` 的普通对象，和世界不共享任何对象。
   * @throws
   * 模拟发散（出现 NaN、无穷大）时，或 `encodeUserData` 返回了 JSON 不能表示的值时。
   */
  createSnapshot(options: SnapshotOptions = {}): PhysicsSnapshot {
    return this.captureSnapshot(options);
  }

  /**
   * 可能影响存档的状态变更计数，供自动保存判断是否跳过捕获。
   * @internal
   */
  get revision(): number {
    return this.revisionValue;
  }

  /** @internal Body、Joint 改了会进存档的状态时调用 */
  markChanged(): void {
    this.revisionValue++;
  }

  /** @internal 持久化复用不变形状；仅供存储器使用，不能向外暴露共享描述。 */
  captureSnapshot(options: SnapshotOptions, cache?: SnapshotShapeCache): PhysicsSnapshot {
    this.check();
    const bodies = [...this.bodySet], ids = new Map(bodies.map((b, i) => [b, i]));
    // 捕获时复用已校验配置，仅检查运动状态中的非有限数。
    return checkCapturedBodies({
      format: "cutlass-physics", schema: 1, dimensionId: this.dimension.id,
      settings: {
        gravity: this.gravity, voidSea: this.core.voidSea === null ? false : { ...this.core.voidSea },
        fluids: this.fluids === null ? false : { waterDensity: this.fluids.waterDensity, lavaDensity: this.fluids.lavaDensity, drag: this.fluids.drag, linearDrag: this.fluids.linearDrag, lavaViscosity: this.fluids.lavaViscosity, waveDamping: this.fluids.waveDamping, airPockets: this.fluids.airPockets, floodRate: this.fluids.floodRate, currentSpeed: this.fluids.currentSpeed },
        explosions: this.occlusion !== null, breakFragileBlocks: this.breakFragile,
        substeps: this.substepCount, timeBudgetMs: this.timeBudget === Infinity ? null : this.timeBudget,
        stepMode: this.stepModeValue, paused: this.pausedValue, terrain: copyJson(this.terrainSettings) as PhysicsSnapshot["settings"]["terrain"], customCollision: this.customCollision,
        ...(this.registry === null ? {} : { blocks: this.registry.toJSON() }),
      },
      bodies: bodies.map((b) => captureBody(b, options, cache)),
      joints: [...this.jointSet].filter((j) => j.core.world !== null).map((j) => captureJoint(j, ids)),
    });
  }

  /**
   * 校验快照并恢复为新世界。
   *
   * @remarks
   * 失败时释放新建资源。地形重新读取，原休眠刚体在确认支撑后可再次休眠。
   * 事件订阅不包含在快照中。
   *
   * @param dimension 恢复到的维度，必须与保存时相同。
   * @param input {@link PhysicsWorld.createSnapshot} 的结果或经过 JSON 往返后的对象。
   * @param options 覆盖已保存的世界设置。原世界使用 terrain.collision 时须重新提供；decodeUserData 用于恢复自定义数据。
   * @returns 新的物理世界，推进方式与保存时一致。
   * @throws 快照格式或版本无效、数据损坏、维度不符或缺少 terrain.collision 时抛出错误。
   */
  static fromSnapshot(dimension: Dimension, input: unknown, options: RestoreOptions = {}): PhysicsWorld {
    const snapshot = readSnapshot(input), s = snapshot.settings;
    if (snapshot.dimensionId !== dimension.id) throw new Error("物理存档的维度与目标维度不同");
    if (s.customCollision && options.terrain?.collision === undefined) throw new Error("恢复此存档需要重新提供 terrain.collision");
    const terrain = options.terrain === undefined ? s.terrain ?? undefined : { ...s.terrain, ...options.terrain };
    const w = new PhysicsWorld(dimension, {
      gravity: options.gravity ?? s.gravity, voidSea: options.voidSea ?? s.voidSea, fluids: options.fluids ?? s.fluids, substeps: options.substeps ?? s.substeps,
      explosions: options.explosions ?? s.explosions, breakFragileBlocks: options.breakFragileBlocks ?? s.breakFragileBlocks,
      timeBudgetMs: options.timeBudgetMs ?? s.timeBudgetMs ?? Infinity, terrain, stepMode: "manual", blocks: options.blocks ?? s.blocks,
    });
    try {
      restoreContents(w, snapshot, options, terrain !== undefined);
      // 记录恢复后的休眠状态，避免首步重复触发 bodySleep。
      for (const b of w.bodySet) if (b.core.sleeping) w.sleepState.set(b, true);
      w.paused = s.paused;
      w.stepMode = options.stepMode ?? s.stepMode;
      return w;
    } catch (error) {
      w.destroy();
      throw error;
    }
  }

  /**
   * 物理世界是否有效。
   *
   * @remarks
   * 调用 {@link PhysicsWorld.destroy} 后为 `false`，此后访问该世界会抛出错误。
   */
  get isValid(): boolean {
    return this.validFlag;
  }

  /**
   * 物理世界发出的事件。
   */
  get afterEvents(): PhysicsAfterEvents {
    return this.events;
  }

  /**
   * 世界的重力，单位为格/秒²。
   *
   * @remarks
   * 设置该值会唤醒所有刚体。
   */
  get gravity(): Vector3 {
    this.check();
    return { x: this.gravityValue.x, y: this.gravityValue.y, z: this.gravityValue.z };
  }

  set gravity(value: Vector3) {
    this.check();
    finite3("gravity", value);
    this.gravityValue = { x: value.x, y: value.y, z: value.z };
    this.core.gravityX = value.x; this.core.gravityY = value.y; this.core.gravityZ = value.z;
    for (const b of this.bodySet) b.core.wakeUp();
    this.revisionValue++;
  }

  /**
   * 世界的时间推进方式。
   *
   * @remarks
   * 设为 `"manual"` 会停止自动推进。
   */
  get stepMode(): StepMode {
    this.check();
    return this.stepModeValue;
  }

  set stepMode(value: StepMode) {
    this.check();
    if (value !== "auto" && value !== "manual") throw new Error(`stepMode 必须是 "auto" 或 "manual"，收到 ${String(value)}`);
    if (value === this.stepModeValue) return;
    this.stepModeValue = value;
    this.revisionValue++;
    if (value === "auto") {
      this.runId = system.runInterval(() => {
        if (this.validFlag && !this.pausedValue) this.step(TICK_SECONDS);
      }, 1);
    } else {
      system.clearRun(this.runId);
      this.runId = -1;
    }
  }

  /**
   * 每次推进拆分的子步数。
   */
  get substeps(): number {
    this.check();
    return this.substepCount;
  }

  set substeps(value: number) {
    this.check();
    if (!Number.isInteger(value) || value < 1) throw new Error(`substeps 必须是不小于 1 的整数，收到 ${value}`);
    this.substepCount = value;
    this.activeSubsteps = value;
    this.slowSteps = 0;
    this.fastSteps = 0;
    this.revisionValue++;
  }

  /**
   * 每次推进的目标求解耗时，单位为毫秒，用来增减子步。详见 {@link PhysicsWorldOptions.timeBudgetMs}。
   */
  get timeBudgetMs(): number {
    this.check();
    return this.timeBudget;
  }

  set timeBudgetMs(value: number) {
    this.check();
    this.timeBudget = checkBudget(value);
    this.revisionValue++;
  }

  /**
   * 世界中有效刚体的数量。
   */
  get bodyCount(): number {
    this.check();
    return this.bodySet.size;
  }

  /**
   * 世界中有效关节的数量。
   */
  get jointCount(): number {
    this.check();
    return this.jointSet.size;
  }

  /**
   * 世界中未休眠的刚体数量。
   */
  get awakeBodyCount(): number {
    this.check();
    let n = 0;
    // 等待地形的冻结状态不计为休眠。
    for (const b of this.bodySet) if (b.core.type !== RigidBodyType.STATIC && (!b.core.sleeping || this.isWaiting(b))) n++;
    return n;
  }

  /**
   * 世界是否处于暂停状态。
   *
   * @remarks
   * 暂停时不会自动推进，但仍可以调用 {@link PhysicsWorld.step}。
   */
  get paused(): boolean {
    this.check();
    return this.pausedValue;
  }

  set paused(value: boolean) {
    this.check();
    this.pausedValue = !!value;
    this.revisionValue++;
  }

  /**
   * 推进物理世界。
   *
   * @remarks
   * 在 auto 模式下调用会额外推进一步。
   *
   * @param deltaTime
   * 推进的时间，单位为秒，必须大于 `0`，由各个子步平均分摊。默认值为 `1 / 20`，即一个 tick。
   * @returns
   * 这次推进的子步数。负载太高时，该值小于 {@link PhysicsWorld.substeps}，规则见 {@link PhysicsWorldOptions.substeps}。
   * @throws
   * `deltaTime` 不合法时；推进完成后有事件回调抛出错误时，抛出第一个错误（见 {@link Signal}）。
   *
   * @example manual-step.ts
   * {@includeCode ../../examples/getting-started/manual-step.ts}
   */
  step(deltaTime: number = TICK_SECONDS): number {
    this.check();
    if (!(deltaTime > 0) || !Number.isFinite(deltaTime)) throw new Error(`deltaTime 必须是大于零的有限数，收到 ${deltaTime}`);
    const start = now();
    // 碰撞事件延迟至步进结束后派发。
    this.tickCount++;
    // 先施加爆炸冲量，使被唤醒刚体按活动范围请求地形。
    if (this.occlusion !== null && this.explosionQueue.size > 0) this.applyExplosions(this.occlusion);
    if (this.collider !== null && this.freezeOnUnloaded) this.checkChunks(this.collider);
    // 更新地形并调整等待状态。
    this.collider?.update(deltaTime);
    // 求解预算包含流体计算，不包含地形读取。
    const solveStart = this.collider === null ? start : now();
    // 悬浮、流体的力按这一步开头的速度算，写进 fx..tz，子步按这个常力积分
    if (this.floatingBodies.size > 0) this.applyFloating();
    if (this.fluids !== null) this.applyFluids(this.fluids, this.collider!, deltaTime);
    // 子步数按最近的耗时调整（见 adjustSubsteps）；每次都推进完整 deltaTime
    const done = this.activeSubsteps;
    // 推进前记录插值起点。
    for (const b of this.bodySet) b.recordPrevious();
    this.core.softSubsteps = done;
    this.core.step(deltaTime);
    // 存在参与推进的刚体时标记存档变化；全部休眠时状态保持不变。
    if (this.core.numIslands > 0) this.revisionValue++;
    this.totalSteps += done;
    this.detectBreaks();
    this.detectSleep();
    const end = now();
    this.adjustSubsteps(end - solveStart);
    try {
      this.flush();
    } finally {
      // 其他事件的回调错误不阻断 stepEnd。
      this.events.stepEnd.emit({ deltaTime, stepCount: this.totalSteps, elapsedMs: end - start });
    }
    return done;
  }

  /** 对活动且地形就绪的动态刚体施加悬浮升力和阻力。 */
  private applyFloating(): void {
    const gx = this.core.gravityX, gy = this.core.gravityY, gz = this.core.gravityZ;
    for (const [b, clusters] of this.floatingBodies) {
      const rb = b.core;
      if (rb.type !== RigidBodyType.DYNAMIC || rb.sleeping || this.isWaiting(b)) continue;
      applyFloating(rb, clusters, gx, gy, gz);
    }
  }

  /** 计算活动且地形就绪的动态刚体的流体作用。受力不平衡且无接触的刚体保持活动。 */
  private applyFluids(fluids: FluidForces, collider: VoxelCollider, dt: number): void {
    // 缓存里一格流体都没有：只把碰过流体的刚体复位
    if (!collider.hasAnyFluid()) {
      for (const [b, r] of this.fluidBodies) { b.core.restless = false; r.body.submerged = 0; r.inWater = false; r.inLava = false; }
      return;
    }
    const gx = this.core.gravityX, gy = this.core.gravityY, gz = this.core.gravityZ;
    const box = fluidBounds;
    for (const b of this.bodySet) {
      const rb = b.core;
      rb.restless = false;
      if (rb.type !== RigidBodyType.DYNAMIC || rb.sleeping || this.isWaiting(b)) continue;
      if (!bodyBounds(rb, box)) continue;
      let record = this.fluidBodies.get(b);
      // 周围和气腔内均无流体时，跳过流体数据准备。
      if (!collider.hasFluidNear(Math.floor(box[0]), Math.floor(box[1]) - 1, Math.floor(box[2]), Math.floor(box[3]), Math.floor(box[4]), Math.floor(box[5])) &&
        (record === undefined || fluids.fluidInside(record.body) <= 0)) {
        if (record !== undefined) { record.body.submerged = 0; record.inWater = false; record.inLava = false; }
        continue;
      }
      if (record === undefined) { record = { body: newFluidBody(), inWater: false, inLava: false, lastLava: -Infinity }; this.fluidBodies.set(b, record); }
      const body = record.body;
      body.grid.update(rb);
      const t = rb.transform;
      body.px = t.px; body.py = t.py; body.pz = t.pz;
      body.r00 = t.r00; body.r01 = t.r01; body.r02 = t.r02;
      body.r10 = t.r10; body.r11 = t.r11; body.r12 = t.r12;
      body.r20 = t.r20; body.r21 = t.r21; body.r22 = t.r22;
      const q = matrixToQuaternion(t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22);
      body.qx = q.x; body.qy = q.y; body.qz = q.z; body.qw = q.w;
      body.vx = rb.vx; body.vy = rb.vy; body.vz = rb.vz; body.wx = rb.wx; body.wy = rb.wy; body.wz = rb.wz;
      body.mass = rb.mass; body.invMass = rb.invMass;
      body.ii00 = rb.ii00; body.ii11 = rb.ii11; body.ii22 = rb.ii22;
      body.gravityScale = rb.gravityScale;
      body.minX = box[0]; body.minY = box[1]; body.minZ = box[2]; body.maxX = box[3]; body.maxY = box[4]; body.maxZ = box[5];
      body.fx = rb.fx; body.fy = rb.fy; body.fz = rb.fz; body.tx = rb.tx; body.ty = rb.ty; body.tz = rb.tz;
      const acted = fluids.apply(body, collider, gx, gy, gz, dt);
      rb.fx = body.fx; rb.fy = body.fy; rb.fz = body.fz; rb.tx = body.tx; rb.ty = body.ty; rb.tz = body.tz;
      // 有接触的刚体由 core 在窄相之后清掉 restless（按这一步的接触判断）
      if (acted && fluids.lastUnbalance > FLUID_RESTLESS) rb.restless = true;
      // 气腔液面查询从下一 tick 起将请求范围向上扩展 64 格。
      collider.setFluidReach(rb, fluids.airPockets && fluids.hasPockets(body));
      this.detectFluidEntry(b, record, fluids, collider);
    }
  }

  /**
   * 入水事件：上一次处理时不在水里、这次在，就发 waterEntry；岩浆要离开满 4 个 tick 才能再发 lavaEntry。
   * 没人订阅时只记状态，不收集接触点
   */
  private detectFluidEntry(b: Body, record: FluidRecord, fluids: FluidForces, terrain: FluidTerrain): void {
    const kinds = fluids.lastKinds;
    const inWater = (kinds & (1 << (FLUID_WATER - 1))) !== 0, inLava = (kinds & (1 << (FLUID_LAVA - 1))) !== 0;
    const water = inWater && !record.inWater, lava = inLava && !record.inLava && this.tickCount - record.lastLava > LAVA_ENTRY_REARM;
    record.inWater = inWater;
    record.inLava = inLava;
    if (inLava) record.lastLava = this.tickCount;
    if (water && this.events.waterEntry.hasSubscribers) this.queueFluidEntry(this.events.waterEntry, b, record.body, fluids, terrain, FLUID_WATER);
    if (lava && this.events.lavaEntry.hasSubscribers) this.queueFluidEntry(this.events.lavaEntry, b, record.body, fluids, terrain, FLUID_LAVA);
  }

  private queueFluidEntry(signal: SignalImpl<FluidEntryEvent>, b: Body, body: FluidBody, fluids: FluidForces, terrain: FluidTerrain, kind: number): void {
    const e = fluidEntry;
    if (!fluids.collectEntry(body, terrain, kind, e)) return;
    const event: FluidEntryEvent = {
      body: b, fastestContactVelocityY: e.fastestVelocityY,
      point: { x: e.pointX, y: e.pointY, z: e.pointZ },
      minX: e.minX, maxX: e.maxX, minZ: e.minZ, maxZ: e.maxZ,
      bodySizeX: body.maxX - body.minX, bodySizeZ: body.maxZ - body.minZ,
    };
    this.pending.push(() => signal.emit(event));
  }

  /** 施加已排队的爆炸冲量，跳过等待地形的刚体。 */
  private applyExplosions(occlusion: OcclusionTest): void {
    const cores: RigidBody[] = [];
    for (const b of this.bodySet) if (!this.isWaiting(b)) cores.push(b.core);
    this.explosionQueue.apply(cores, occlusion);
  }

  /**
   * 刚体最近的浸没体积，单位为格³。休眠期间保留最后一次计算结果。
   * @internal
   */
  submergedVolume(b: Body): number {
    return this.fluidBodies.get(b)?.body.submerged ?? 0;
  }

  /** @internal 刚体是否在等地形（见 TerrainOptions.unloadedChunkPolicy） */
  isWaiting(b: Body): boolean {
    return this.collider !== null && this.collider.isWaiting(b.core);
  }

  /** 轮询动态刚体所在区块。未加载时停用并释放地形请求，加载后恢复。 */
  private checkChunks(collider: VoxelCollider): void {
    const n = this.bodySet.size;
    if (n === 0 || typeof this.dimension.isChunkLoaded !== "function") return;
    const count = Math.min(CHUNK_CHECKS_PER_TICK, n);
    const first = this.chunkCursor % n;
    let i = 0, checked = 0;
    for (const b of this.bodySet) {
      const k = (i++ - first + n) % n;
      if (k >= count) continue;
      checked++;
      if (b.core.type !== RigidBodyType.DYNAMIC) continue;
      const t = b.core.transform;
      let loaded: boolean;
      try {
        loaded = this.dimension.isChunkLoaded({ x: t.px, y: t.py, z: t.pz });
      } catch {
        loaded = false;
      }
      if (loaded === collider.isParked(b.core)) {
        collider.setParked(b.core, !loaded);
        this.revisionValue++;
      }
      if (checked === count) break;
    }
    this.chunkCursor = first + count;
  }

  /** 连续 3 次求解超出预算就减 1 个子步，最少 2 个；连续 100 次不到预算的三分之一就加 1 个，最多回到 substeps */
  private adjustSubsteps(ms: number): void {
    const budget = this.timeBudget;
    if (ms > budget) {
      this.fastSteps = 0;
      if (++this.slowSteps >= 3 && this.activeSubsteps > MIN_SUBSTEPS) {
        this.activeSubsteps--;
        this.slowSteps = 0;
      }
    } else {
      this.slowSteps = 0;
      if (ms * 3 < budget && ++this.fastSteps >= 100 && this.activeSubsteps < this.substepCount) {
        this.activeSubsteps++;
        this.fastSteps = 0;
      }
    }
  }

  /** 内核在关节受力超过阈值时自己把关节摘掉；这里把对应的包装对象失效，并发出 jointBreak */
  private detectBreaks(): void {
    if (this.jointSet.size === 0) return;
    let broken: Joint[] | null = null;
    for (const j of this.jointSet) {
      if (j.core.world === null) (broken ??= []).push(j);
    }
    if (broken === null) return;
    const signal = this.events.jointBreak;
    for (const j of broken) {
      this.jointSet.delete(j);
      j.invalidate();
      if (signal.hasSubscribers) this.pending.push(() => signal.emit({ joint: j }));
    }
  }

  /** 睡眠状态的变化转成事件。等地形的刚体不算休眠 */
  private detectSleep(): void {
    for (const b of this.bodySet) {
      if (this.isWaiting(b)) continue;
      const asleep = b.core.sleeping;
      const was = this.sleepState.get(b) ?? false;
      if (asleep === was) continue;
      this.sleepState.set(b, asleep);
      const signal = asleep ? this.events.bodySleep : this.events.bodyWake;
      if (signal.hasSubscribers) this.pending.push(() => signal.emit({ body: b }));
    }
  }

  /** 把步进过程中攒下的事件依次触发 */
  private flush(): void {
    const list = this.pending;
    if (list.length === 0) return;
    this.pending = [];
    let failed = false;
    let firstError: unknown;
    for (const fn of list) {
      try {
        fn();
      } catch (e) {
        if (!failed) {
          failed = true;
          firstError = e;
        }
      }
    }
    if (failed) throw firstError;
  }

  /**
   * 创建一个刚体。
   *
   * @param options 刚体选项。
   * @returns 新创建的刚体。
   * @throws 选项无效时抛出错误，创建过程不保留部分结果。
   * 形状列表须非空，类型、尺寸、密度及 block 数据须有效；位置、速度和 gravityScale 须为有限数，阻尼须非负。
   * 形状错误包含该形状在 shapes 中的下标。
   *
   * @example drop-a-crate.ts
   * {@includeCode ../../examples/getting-started/drop-a-crate.ts}
   *
   * @example create-body-error.ts
   * {@includeCode ../../examples/world/create-body-error.ts}
   */
  createBody(options: BodyOptions): Body {
    this.check();
    if (!Array.isArray(options.shapes) || options.shapes.length === 0) throw new Error("刚体至少需要一个形状");
    const type = options.type ?? "dynamic";
    const t = type === "static" ? RigidBodyType.STATIC : type === "kinematic" ? RigidBodyType.KINEMATIC : type === "dynamic" ? RigidBodyType.DYNAMIC : -1;
    if (t < 0) throw new Error(`未知的刚体类型：${String(type)}`);
    const config = new RigidBodyConfig();
    config.type = t;
    const loc = options.location;
    if (loc !== undefined) {
      finite3("location", loc);
      config.transform.px = loc.x; config.transform.py = loc.y; config.transform.pz = loc.z;
    }
    if (options.rotation !== undefined) {
      const m = quaternionToMatrix(options.rotation);
      const tf = config.transform;
      tf.r00 = m[0]; tf.r01 = m[1]; tf.r02 = m[2]; tf.r10 = m[3]; tf.r11 = m[4]; tf.r12 = m[5]; tf.r20 = m[6]; tf.r21 = m[7]; tf.r22 = m[8];
    }
    if (options.linearVelocity !== undefined) {
      finite3("linearVelocity", options.linearVelocity);
      config.vx = options.linearVelocity.x; config.vy = options.linearVelocity.y; config.vz = options.linearVelocity.z;
    }
    if (options.angularVelocity !== undefined) {
      finite3("angularVelocity", options.angularVelocity);
      config.wx = options.angularVelocity.x; config.wy = options.angularVelocity.y; config.wz = options.angularVelocity.z;
    }
    config.linearDamping = options.linearDamping === undefined ? DEFAULT_DAMPING : nonNegative("linearDamping", options.linearDamping);
    config.angularDamping = options.angularDamping === undefined ? DEFAULT_DAMPING : nonNegative("angularDamping", options.angularDamping);
    config.autoSleep = options.allowSleep ?? true;
    config.sleepingVelocityThreshold = SLEEP_LINEAR;
    config.sleepingAngularVelocityThreshold = SLEEP_ANGULAR;
    config.sleepingTimeThreshold = SLEEP_SECONDS;
    // 形状全部转换成功后创建刚体。
    const shapes: Shape[] = options.shapes.map((o, i) => {
      try {
        return makeShape(withBodyFriction(o));
      } catch (e) {
        const error = new Error(`shapes[${i}]：${(e as Error).message}`);
        (error as Error & { cause?: unknown }).cause = e;
        throw error;
      }
    });
    const core = new RigidBody(config);
    core.addShapes(shapes);
    // 方块形状：记下方块，挑出悬浮、易碎的方块。一个方块可能有几个盒子（楼梯），悬浮按方块算一次，
    // 位置取这个方块各个盒子按体积加权的中心
    let floatingBlocks: Map<string, { x: number; y: number; z: number; w: number; physics: BlockPhysics }> | null = null;
    let fragile = false;
    for (let i = 0; i < shapes.length; i++) {
      const given = options.shapes[i].block;
      if (given === undefined) continue;
      checkBodyBlock(given, i);
      const block: BodyBlock = given.states === undefined
        ? { location: { x: given.location.x, y: given.location.y, z: given.location.z }, typeId: given.typeId }
        : { location: { x: given.location.x, y: given.location.y, z: given.location.z }, typeId: given.typeId, states: { ...given.states } };
      const physics = this.blockPhysics(block.typeId, block.states ?? {});
      shapes[i].userData = { block, physics, index: i } satisfies ShapeBlock;
      if (physics.fragileImpactSpeed !== undefined) fragile = true;
      if (physics.floatingMaterial === undefined) continue;
      const s = shapes[i], t = s.localTransform, v = s.geom.volume, key = `${block.location.x},${block.location.y},${block.location.z}`;
      floatingBlocks ??= new Map();
      const e = floatingBlocks.get(key);
      if (e === undefined) floatingBlocks.set(key, { x: t.px * v, y: t.py * v, z: t.pz * v, w: v, physics });
      else { e.x += t.px * v; e.y += t.py * v; e.z += t.pz * v; e.w += v; }
    }
    const floating: FloatingPoint[] = [];
    if (floatingBlocks !== null) {
      for (const e of floatingBlocks.values()) {
        floating.push({ x: e.x / e.w, y: e.y / e.w, z: e.z / e.w, material: e.physics.floatingMaterial!, scale: e.physics.floatingScale ?? 1 });
      }
    }
    if (options.gravityScale !== undefined) {
      if (!Number.isFinite(options.gravityScale)) throw new Error(`gravityScale 必须是有限的数，收到 ${options.gravityScale}`);
      core.gravityScale = options.gravityScale;
    }
    const body = new Body(this, core, options.userData);
    body.recordPrevious();
    this.core.addRigidBody(core);
    this.bodySet.add(body);
    if (floating.length > 0) this.floatingBodies.set(body, floatingClusters(floating));
    if (fragile) this.fragileBodies.add(body);
    this.revisionValue++;
    return body;
  }

  /** 内部使用：Body.remove 调用
   * @internal */
  removeBody(body: Body): void {
    if (!this.bodySet.has(body)) return;
    // 移除刚体前先移除关联关节。
    for (const j of [...this.jointSet]) if (j.bodyA === body || j.bodyB === body) this.removeJoint(j);
    this.bodySet.delete(body);
    this.sleepState.delete(body);
    this.fluidBodies.delete(body);
    this.floatingBodies.delete(body);
    this.fragileBodies.delete(body);
    if (body.core.world !== null) this.core.removeRigidBody(body.core);
    body.invalidate();
    this.revisionValue++;
  }

  /**
   * 创建一个关节，连接两个刚体。
   *
   * @param options
   * 关节的选项。
   * @returns
   * 新创建的关节。
   * @throws
   * 两个刚体不在同一个世界，或两者均不是动态刚体时抛出错误。
   */
  createJoint(options: JointOptions): Joint {
    this.check();
    this.checkJointBodies(options);
    if (options.bodyA.core.type !== RigidBodyType.DYNAMIC && options.bodyB.core.type !== RigidBodyType.DYNAMIC) {
      throw new Error("关节连接的两个刚体至少要有一个是动态刚体");
    }
    return this.addJoint(options);
  }

  private checkJointBodies(options: JointOptions): void {
    const a = options.bodyA, b = options.bodyB;
    if (!(this.bodySet.has(a) && this.bodySet.has(b))) throw new Error("关节连接的两个刚体都必须在这个世界里，并且有效");
    if (a === b) throw new Error("关节不能把一个刚体连到它自己");
  }

  /**
   * 恢复关节，允许两端均为非动态刚体。
   * @internal
   */
  addJoint(options: JointOptions): Joint {
    this.check();
    this.checkJointBodies(options);
    const a = options.bodyA, b = options.bodyB;
    const coreJoint = makeJoint(options, a.core, b.core);
    const joint = new Joint(this, coreJoint, a, b, options.type);
    this.core.addJoint(coreJoint);
    this.jointSet.add(joint);
    this.revisionValue++;
    return joint;
  }

  /** 内部使用：Joint.remove 调用
   * @internal */
  removeJoint(joint: Joint): void {
    if (!this.jointSet.has(joint)) return;
    this.jointSet.delete(joint);
    if (joint.core.world !== null) this.core.removeJoint(joint.core);
    joint.invalidate();
    this.revisionValue++;
  }

  /**
   * 遍历世界中所有有效的刚体。
   *
   * @remarks
   * 与 {@link PhysicsWorld.getBodies} 相比，不会创建数组。在回调中移除当前刚体是安全的。
   *
   * @param callback
   * 对每个刚体调用的函数。
   *
   * @example for-each-body.ts
   * {@includeCode ../../examples/display-sync/for-each-body.ts}
   */
  forEachBody(callback: (body: Body) => void): void {
    this.check();
    // 在回调里移除当前刚体是安全的：Set 遍历允许删除当前项
    for (const b of this.bodySet) callback(b);
  }

  /**
   * 返回世界中所有有效刚体组成的新数组。
   */
  getBodies(): Body[] {
    this.check();
    return [...this.bodySet];
  }

  /**
   * 执行射线检测，返回最近的命中。
   *
   * @param origin
   * 射线的起点。
   * @param direction
   * 射线的方向。无需归一化，不能为零向量。
   * @param options
   * 检测选项。
   * @returns
   * 最近的命中结果。没有命中时为 `undefined`。
   *
   * @example ray-cast.ts
   * {@includeCode ../../examples/world/ray-cast.ts}
   */
  rayCast(origin: Vector3, direction: Vector3, options?: RayCastOptions): RayCastHit | undefined {
    this.check();
    const hits = this.rayCastAll(origin, direction, options);
    return hits[0];
  }

  /**
   * 执行射线检测，返回路径上的所有命中。
   *
   * @remarks
   * 结果按距离从近到远排列。
   *
   * @param origin
   * 射线的起点。
   * @param direction
   * 射线的方向。无需归一化，不能为零向量。
   * @param options
   * 检测选项。
   * @returns
   * 所有命中结果。
   */
  rayCastAll(origin: Vector3, direction: Vector3, options?: RayCastOptions): RayCastHit[] {
    this.check();
    finite3("origin", origin);
    finite3("direction", direction);
    const len = Math.hypot(direction.x, direction.y, direction.z);
    if (!(len > 0)) throw new Error("direction 不能是零向量");
    const max = options?.maxDistance ?? 100;
    if (!(max > 0) || !Number.isFinite(max)) throw new Error(`maxDistance 必须是大于零的有限数，收到 ${max}`);
    const dx = direction.x / len, dy = direction.y / len, dz = direction.z / len;
    const mask = options?.collisionMask;
    const exclude = options?.excludeBodies;
    const includeBlocks = options?.includeBlocks ?? true, includeBodies = options?.includeBodies ?? true;
    const out: RayCastHit[] = [];
    const callback: RayCastCallback = {
      process: (shape: Shape, hit: CoreRayCastHit) => {
        const rb = shape.rigidBody!;
        const owner = rb.userData instanceof Body ? rb.userData : undefined;
        if (owner === undefined && !includeBlocks) return;
        if (owner !== undefined && !includeBodies) return;
        if (owner !== undefined && exclude !== undefined && exclude.includes(owner)) return;
        if (mask !== undefined && (shape.collisionGroup & mask) === 0) return;
        const result: RayCastHit = {
          point: { x: hit.positionX, y: hit.positionY, z: hit.positionZ },
          normal: { x: hit.normalX, y: hit.normalY, z: hit.normalZ },
          distance: hit.fraction * max,
        };
        if (owner !== undefined) {
          result.body = owner;
        } else {
          // 方块的碰撞体没有逐块的身份，用命中点往里一点所在的方块坐标
          result.blockLocation = {
            x: Math.floor(hit.positionX - hit.normalX * 1e-4),
            y: Math.floor(hit.positionY - hit.normalY * 1e-4),
            z: Math.floor(hit.positionZ - hit.normalZ * 1e-4),
          };
        }
        out.push(result);
      },
    };
    this.core.rayCast(origin.x, origin.y, origin.z, origin.x + dx * max, origin.y + dy * max, origin.z + dz * max, callback);
    out.sort((p, q) => p.distance - q.distance);
    return out;
  }

  /**
   * 返回包围盒与指定范围重叠的刚体。
   *
   * @remarks
   * 仅比较包围盒，因此结果是近似的。
   *
   * @param min
   * 范围的最小角。
   * @param max
   * 范围的最大角。
   * @param options
   * 查询选项。
   * @returns
   * 包围盒与该范围重叠的刚体。
   *
   * @example query-box.ts
   * {@includeCode ../../examples/world/query-box.ts}
   */
  queryBox(min: Vector3, max: Vector3, options?: QueryOptions): Body[] {
    this.check();
    finite3("min", min);
    finite3("max", max);
    if (options?.includeBodies === false) return [];
    const box = new Aabb();
    box.minX = Math.min(min.x, max.x); box.minY = Math.min(min.y, max.y); box.minZ = Math.min(min.z, max.z);
    box.maxX = Math.max(min.x, max.x); box.maxY = Math.max(min.y, max.y); box.maxZ = Math.max(min.z, max.z);
    const mask = options?.collisionMask;
    const found = new Set<Body>();
    const callback: AabbTestCallback = {
      process: (shape: Shape) => {
        const owner = shape.rigidBody!.userData;
        if (!(owner instanceof Body)) return;
        if (mask !== undefined && (shape.collisionGroup & mask) === 0) return;
        found.add(owner);
      },
    };
    this.core.aabbTest(box, callback);
    return [...found];
  }

  /**
   * 沿直线扫掠形状，返回最近的命中。
   *
   * @param shape
   * 要扫过的形状。仅支持凸形：`box`、`sphere`、`capsule`、`cylinder`、`cone` 和 `convexHull`。
   * @param location
   * 形状的起始位置。
   * @param translation
   * 位移向量，不是方向。
   * @param rotation
   * 形状的起始朝向。默认值为没有旋转。
   * @param options
   * 检测选项。
   * @returns
   * 命中结果。没有命中时为 `undefined`。
   */
  sweep(
    shape: ShapeOptions, location: Vector3, translation: Vector3,
    rotation?: Quaternion, options?: RayCastOptions,
  ): RayCastHit | undefined {
    this.check();
    finite3("location", location);
    finite3("translation", translation);
    const geometry = makeGeometry(shape);
    if (!isConvex(geometry)) throw new Error("sweep 只支持凸形：box、sphere、capsule、cylinder、cone、convexHull");
    const begin = new Transform();
    begin.px = location.x; begin.py = location.y; begin.pz = location.z;
    if (rotation !== undefined) {
      const m = quaternionToMatrix(rotation);
      begin.r00 = m[0]; begin.r01 = m[1]; begin.r02 = m[2]; begin.r10 = m[3]; begin.r11 = m[4]; begin.r12 = m[5]; begin.r20 = m[6]; begin.r21 = m[7]; begin.r22 = m[8];
    }
    const distance = Math.hypot(translation.x, translation.y, translation.z);
    const mask = options?.collisionMask;
    const exclude = options?.excludeBodies;
    const includeBlocks = options?.includeBlocks ?? true, includeBodies = options?.includeBodies ?? true;
    const maxDistance = options?.maxDistance ?? Infinity;
    const best = new RayCastClosest();
    const filter: RayCastCallback = {
      process: (sh: Shape, hit: CoreRayCastHit) => {
        const owner = sh.rigidBody!.userData instanceof Body ? sh.rigidBody!.userData : undefined;
        if (owner === undefined && !includeBlocks) return;
        if (owner !== undefined && !includeBodies) return;
        if (owner !== undefined && exclude !== undefined && exclude.includes(owner)) return;
        if (mask !== undefined && (sh.collisionGroup & mask) === 0) return;
        best.process(sh, hit);
      },
    };
    this.core.convexCast(geometry, begin, translation.x, translation.y, translation.z, filter);
    if (!best.hit) return undefined;
    const d = best.fraction * distance;
    if (d > maxDistance) return undefined;
    const rb = best.shape!.rigidBody!;
    const owner = rb.userData instanceof Body ? rb.userData : undefined;
    const result: RayCastHit = {
      point: { x: best.positionX, y: best.positionY, z: best.positionZ },
      // 反转 GJK 扫掠法线，使其朝向运动来向。
      normal: { x: -best.normalX + 0, y: -best.normalY + 0, z: -best.normalZ + 0 },
      distance: d,
    };
    if (owner !== undefined) result.body = owner;
    else {
      // 命中点往被撞物体里面一点（沿运动方向）所在的方块
      result.blockLocation = {
        x: Math.floor(best.positionX + best.normalX * 1e-4),
        y: Math.floor(best.positionY + best.normalY * 1e-4),
        z: Math.floor(best.positionZ + best.normalZ * 1e-4),
      };
    }
    return result;
  }

  /**
   * 使指定范围的地形碰撞缓存失效。
   *
   * @remarks
   * 范围自动外扩 1 格，覆盖邻接形状的变化；未缓存区域在后续读取时获取当前数据。
   * 未启用地形时无效果。
   *
   * @param min
   * 范围的最小方块坐标。
   * @param max
   * 范围的最大方块坐标，包含在内。
   *
   * @example invalidate-blocks.ts
   * {@includeCode ../../examples/terrain/invalidate-blocks.ts}
   */
  invalidateBlocks(min: Vector3, max: Vector3): void {
    this.check();
    finite3("min", min);
    finite3("max", max);
    this.collider?.blocksChanged(
      Math.min(min.x, max.x), Math.min(min.y, max.y), Math.min(min.z, max.z),
      Math.max(min.x, max.x), Math.max(min.y, max.y), Math.max(min.z, max.z),
    );
  }

  /**
   * 销毁物理世界。
   *
   * @remarks
   * 停止自动推进，移除所有刚体和关节，并取消所有事件订阅。世界及其创建的所有对象随即失效。重复调用没有额外效果。
   */
  destroy(): void {
    if (!this.validFlag) return;
    this.validFlag = false;
    if (this.runId >= 0) {
      system.clearRun(this.runId);
      this.runId = -1;
    }
    this.unwatchBlocks?.();
    this.unwatchBlocks = null;
    this.unwatchExplosions?.();
    this.unwatchExplosions = null;
    this.explosionQueue.clear();
    this.core.contactManager.listener = null;
    for (const j of this.jointSet) j.invalidate();
    for (const b of this.bodySet) b.invalidate();
    this.jointSet.clear();
    this.bodySet.clear();
    this.sleepState.clear();
    this.fluidBodies.clear();
    this.floatingBodies.clear();
    this.fragileBodies.clear();
    this.pendingBreaks.clear();
    this.collider?.dispose();
    for (const signal of Object.values(this.events)) signal.clear();
    this.pending = [];
  }

  // ---- 接触事件 ----

  private onBeginContact(c: Contact): void {
    const owner1 = c.b1!.userData, owner2 = c.b2!.userData;
    const b1 = owner1 instanceof Body ? owner1 : undefined, b2 = owner2 instanceof Body ? owner2 : undefined;
    // 仅对达到最小易碎速度的撞击检查方块。
    const speed = c.contactConstraint.approachSpeed;
    if (speed >= this.fragileMinSpeed && c.manifold.numPoints > 0) this.checkFragile(c, b1, b2, speed);
    const m = c.manifold;
    let px = 0, py = 0, pz = 0;
    const n = m.numPoints;
    for (let i = 0; i < n; i++) {
      const p = m.points[i];
      px += (p.p1X + p.p2X) * 0.5; py += (p.p1Y + p.p2Y) * 0.5; pz += (p.p1Z + p.p2Z) * 0.5;
    }
    if (n > 0) { px /= n; py /= n; pz /= n; }
    // 法线方向：流形的法线从 b2 指向 b1（b1 在上方时为 +Y）
    const nx = m.nX, ny = m.nY, nz = m.nZ;
    // 接触状态在求解后确定，撞击速度使用求解前的法向靠近速度。
    const rel = c.contactConstraint.approachSpeed;
    if (b1 !== undefined && b2 !== undefined) {
      const signal = this.events.bodyCollisionStart;
      if (!signal.hasSubscribers) return;
      // 事件的法线是从 A 指向 B，也就是从 b1 指向 b2，与流形的方向相反
      const event = { bodyA: b1, bodyB: b2, point: { x: px, y: py, z: pz }, normal: { x: -nx, y: -ny, z: -nz }, impactSpeed: rel };
      this.pending.push(() => signal.emit(event));
      return;
    }
    const body = b1 ?? b2;
    if (body === undefined) return;
    const signal = this.events.blockCollisionStart;
    if (!signal.hasSubscribers) return;
    // 事件的法线是从方块指向刚体。刚体是 b1 时流形法线正好指向刚体；刚体是 b2 时要反过来
    const sign = b1 !== undefined ? 1 : -1;
    const normal = { x: nx * sign, y: ny * sign, z: nz * sign };
    // 使用方块侧接触点并向内偏移，避免预测接触的平均点落在方块外。
    let bx = 0, by = 0, bz = 0;
    for (let i = 0; i < n; i++) {
      const p = m.points[i];
      if (b1 !== undefined) { bx += p.p2X; by += p.p2Y; bz += p.p2Z; } else { bx += p.p1X; by += p.p1Y; bz += p.p1Z; }
    }
    if (n > 0) { bx /= n; by /= n; bz /= n; }
    const event = {
      body,
      blockLocation: {
        x: Math.floor(bx - normal.x * 1e-3), y: Math.floor(by - normal.y * 1e-3), z: Math.floor(bz - normal.z * 1e-3),
      },
      point: { x: px, y: py, z: pz },
      normal,
      impactSpeed: rel,
    };
    this.pending.push(() => signal.emit(event));
  }

  /**
   * 易碎方块：刚体上的方块形状撞得够快时发事件；刚体撞上世界里的易碎方块时发事件，并在下一 tick 破坏方块。
   * 世界方块按接触点沿法线前后几个位置找（接触点可能落在方块表面之外或者稍微陷进刚体里）
   */
  private checkFragile(c: Contact, b1: Body | undefined, b2: Body | undefined, speed: number): void {
    // 接触点的平均位置
    const m = c.manifold, n = m.numPoints;
    let px = 0, py = 0, pz = 0;
    for (let i = 0; i < n; i++) {
      const p = m.points[i];
      px += (p.p1X + p.p2X) * 0.5; py += (p.p1Y + p.p2Y) * 0.5; pz += (p.p1Z + p.p2Z) * 0.5;
    }
    px /= n; py /= n; pz /= n;
    const signal = this.events.blockBreak;
    for (let side = 0; side < 2; side++) {
      const body = side === 0 ? b1 : b2;
      if (body === undefined || !this.fragileBodies.has(body)) continue;
      const info = (side === 0 ? c.s1 : c.s2)!.userData as ShapeBlock | null;
      const threshold = info?.physics.fragileImpactSpeed;
      if (threshold === undefined || speed < threshold) continue;
      if (!signal.hasSubscribers) continue;
      const l = info!.block.location;
      const event = { body, blockLocation: { x: l.x, y: l.y, z: l.z }, typeId: info!.block.typeId, shapeIndex: info!.index, impactSpeed: speed };
      this.pending.push(() => signal.emit(event));
    }
    // 世界里的方块：一边是刚体、另一边是地形
    const body = b1 !== undefined && b2 === undefined ? b1 : b2 !== undefined && b1 === undefined ? b2 : undefined;
    if (body === undefined) return;
    // 法线从地形指向刚体；刚体是 b2 时流形的法线要反过来
    const sign = b1 !== undefined ? 1 : -1;
    const nx = m.nX * sign, ny = m.nY * sign, nz = m.nZ * sign;
    let lastX = NaN, lastY = NaN, lastZ = NaN;
    for (let i = 0; i < FRAGILE_PROBES.length; i++) {
      const s = FRAGILE_PROBES[i];
      const x = Math.floor(px + nx * s), y = Math.floor(py + ny * s), z = Math.floor(pz + nz * s);
      if (x === lastX && y === lastY && z === lastZ) continue;
      lastX = x; lastY = y; lastZ = z;
      let block;
      try {
        block = this.dimension.getBlock({ x, y, z });
      } catch {
        continue;
      }
      if (block === undefined || block.isAir || block.isLiquid) continue;
      const threshold = this.blockPhysics(block.typeId, getStates(block)).fragileImpactSpeed;
      if (threshold === undefined || speed < threshold) continue;
      const key = `${x},${y},${z}`;
      if (this.pendingBreaks.has(key)) return;
      this.pendingBreaks.add(key);
      const typeId = block.typeId;
      if (signal.hasSubscribers) {
        const event = { body, blockLocation: { x, y, z }, typeId, impactSpeed: speed };
        this.pending.push(() => signal.emit(event));
      }
      if (this.breakFragile) this.queueBreak(x, y, z, typeId, key);
      else this.pendingBreaks.delete(key);
      return;
    }
  }

  /** 下一 tick 破坏仍为原类型的方块，并生成掉落物。 */
  private queueBreak(x: number, y: number, z: number, typeId: string, key: string): void {
    const dimension = this.dimension;
    system.run(() => {
      this.pendingBreaks.delete(key);
      if (!this.validFlag) return;
      let block;
      try {
        block = dimension.getBlock({ x, y, z });
      } catch {
        return;
      }
      if (block === undefined || block.typeId !== typeId) return;
      try {
        dimension.runCommand(`setblock ${x} ${y} ${z} minecraft:air destroy`);
      } catch {
        // 破坏失败（区块刚好卸载等）就不动世界
        return;
      }
      this.collider?.blocksChanged(x - 1, y - 1, z - 1, x + 1, y + 1, z + 1);
    });
  }

  private onEndContact(c: Contact): void {
    const owner1 = c.b1!.userData, owner2 = c.b2!.userData;
    if (!(owner1 instanceof Body && owner2 instanceof Body)) return;
    const signal = this.events.bodyCollisionEnd;
    if (!signal.hasSubscribers) return;
    const event = { bodyA: owner1, bodyB: owner2 };
    this.pending.push(() => signal.emit(event));
  }
}

// ---- 内部常量与函数 ----

/** 形状上记的方块信息（Shape.userData）：方块、它的物理属性、在 shapes 里的下标 */
interface ShapeBlock {
  readonly block: BodyBlock;
  readonly physics: BlockPhysics;
  readonly index: number;
}

/** 内置易碎方块的最小撞击速度阈值。 */
const FRAGILE_MIN_SPEED = 4;
/** 找被撞的世界方块：接触点沿法线（从地形指向刚体）的偏移，先找接触点所在的格，再往两边找 */
const FRAGILE_PROBES = [0, -0.08, 0.08, -0.25, 0.25];

/** 方块的状态；读不到时按没有状态 */
function getStates(block: { permutation: { getAllStates(): Record<string, boolean | number | string> } }): Record<string, boolean | number | string> {
  try {
    return block.permutation.getAllStates();
  } catch {
    return {};
  }
}

/** ShapeOptions.block：位置是整数、类型是字符串、状态是普通对象 */
function checkBodyBlock(block: BodyBlock, i: number): void {
  if (block === null || typeof block !== "object" || typeof block.typeId !== "string") throw new Error(`shapes[${i}].block 必须有 typeId`);
  const l = block.location;
  if (l === null || typeof l !== "object" || !Number.isInteger(l.x) || !Number.isInteger(l.y) || !Number.isInteger(l.z)) {
    throw new Error(`shapes[${i}].block.location 必须是整数坐标`);
  }
  if (block.states !== undefined && (block.states === null || typeof block.states !== "object")) throw new Error(`shapes[${i}].block.states 必须是对象`);
}

const TICK_SECONDS = 1 / 20;
const DEFAULT_GRAVITY: Vector3 = { x: 0, y: -11, z: 0 };
const DEFAULT_SUBSTEPS = 4;
/** 负载太高时子步数最少减到几个 */
const MIN_SUBSTEPS = 2;
/** 虚空海的默认开始高度，各维度相同。 */
const VOID_SEA_START_Y = -105;
const DEFAULT_TIME_BUDGET_MS = 12;
/** 线速度、角速度的默认阻尼 */
const DEFAULT_DAMPING = 0.09;
/** 水的默认密度：默认重力下每格浸没体积的浮力是 10.5，比重量 11 小一点 */
const DEFAULT_WATER_DENSITY = 10.5 / 11;
/** 每个子步的关节速度迭代次数，不用于软接触求解。 */
const SOLVER_VELOCITY_ITERATIONS = 8;
/** 每 tick 查几个刚体所在的区块有没有加载 */
const CHUNK_CHECKS_PER_TICK = 4;
/** 浮力、重力的合力或合力矩不平衡超过重量的这么多，而且这一步没有接触，刚体这个 tick 不睡 */
const FLUID_RESTLESS = 0.005;
/** 离开岩浆满这么多 tick 才能再发 lavaEntry */
const LAVA_ENTRY_REARM = 4;
const fluidBounds = [0, 0, 0, 0, 0, 0];
const fluidEntry: FluidEntry = { fastestVelocityY: 0, minX: 0, maxX: 0, minZ: 0, maxZ: 0, pointX: 0, pointY: 0, pointZ: 0 };

/** 一个刚体的流体状态：流体计算用的数据，以及上一次处理时在不在水里、岩浆里 */
interface FluidRecord {
  body: FluidBody;
  inWater: boolean;
  inLava: boolean;
  /** 最后一次在岩浆里的 tick */
  lastLava: number;
}

function newFluidBody(): FluidBody {
  return {
    grid: new FluidGrid(), px: 0, py: 0, pz: 0, r00: 1, r01: 0, r02: 0, r10: 0, r11: 1, r12: 0, r20: 0, r21: 0, r22: 1,
    qx: 0, qy: 0, qz: 0, qw: 1, vx: 0, vy: 0, vz: 0, wx: 0, wy: 0, wz: 0, mass: 0, invMass: 0, ii00: 0, ii11: 0, ii22: 0, gravityScale: 1,
    minX: 0, minY: 0, minZ: 0, maxX: 0, maxY: 0, maxZ: 0, fx: 0, fy: 0, fz: 0, tx: 0, ty: 0, tz: 0, submerged: 0,
  };
}
const SLEEP_LINEAR = 0.06;
const SLEEP_ANGULAR = 0.08;
const SLEEP_SECONDS = 0.6;

/** 毫秒精度的预算计时。 */
function now(): number {
  return Date.now();
}

function checkBudget(v: number): number {
  if (!(v > 0)) throw new Error(`timeBudgetMs 必须大于零（Infinity 表示不限制），收到 ${v}`);
  return v;
}

/** 校验可选整数，提供时须不小于 min。 */
function checkInteger(name: string, v: number | undefined, min: number): void {
  if (v !== undefined && (!Number.isSafeInteger(v) || v < min)) throw new Error(`${name} 必须是不小于 ${min} 的整数，收到 ${v}`);
}

/** fluids 选项：数值项必须是不小于 0 的有限数，airPockets 必须是布尔值 */
function checkFluids(f: FluidOptions): void {
  if (f === null || typeof f !== "object") throw new Error("fluids 必须是对象或 false");
  for (const k of ["waterDensity", "lavaDensity", "drag", "linearDrag", "lavaViscosity", "waveDamping", "floodRate", "currentSpeed"] as const) {
    const v = f[k];
    if (v !== undefined && (!(v >= 0) || !Number.isFinite(v))) throw new Error(`fluids.${k} 必须是不小于 0 的有限数，收到 ${v}`);
  }
  if (f.airPockets !== undefined && typeof f.airPockets !== "boolean") throw new Error(`fluids.airPockets 必须是布尔值，收到 ${f.airPockets}`);
}

function finite3(name: string, v: Vector3): void {
  if (!Number.isFinite(v.x) || !Number.isFinite(v.y) || !Number.isFinite(v.z)) {
    throw new Error(`${name} 必须是有限的数，收到 (${v.x}, ${v.y}, ${v.z})`);
  }
}

function nonNegative(name: string, v: number): number {
  if (!(v >= 0) || !Number.isFinite(v)) throw new Error(`${name} 必须是不小于零的有限数，收到 ${v}`);
  return v;
}

/** 省略摩擦时使用普通材质的形状摩擦，显式值保持不变。 */
function withBodyFriction(o: ShapeOptions): ShapeOptions {
  if (o.friction !== undefined) return o;
  return { ...o, friction: materialFriction(DEFAULT_MATERIALS[0]) };
}

/** 向 Minecraft 询问读方块时抛出的错误属于哪一类 */
const HOST: BlockSourceHost = {
  createListVolume: (locations) => new ListBlockVolume(locations),
  createVolume: (x0, y0, z0, x1, y1, z1) => new BlockVolume({ x: x0, y: y0, z: z0 }, { x: x1, y: y1, z: z1 }),
  classifyError: (e): BlockReadError => {
    if (e instanceof UnloadedChunksError || e instanceof LocationInUnloadedChunkError) return "unloaded";
    if (e instanceof LocationOutOfWorldBoundariesError) return "outOfBounds";
    return "other";
  },
  typeExists: (typeId) => BlockTypes.get(typeId) !== undefined,
};
