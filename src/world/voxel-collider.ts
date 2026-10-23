// 按 8×8×8 块缓存地形，每块构建一个静态刚体。
// update 在物理步进前复查变化、更新请求并读取队列。
// 当前地形未就绪时冻结相连刚体，地形就绪后恢复。
// 地形变化仅唤醒休眠范围与变化单元相交的刚体。
// 无活动引用的块退出碰撞世界，无引用的缓存按 unloadDelay 回收。
// 流体码独立缓存，仅流体变化时保留静态碰撞体。
import { Aabb } from "../core/collision/geometry/aabb.js";
import { BoxGeometry } from "../core/collision/geometry/box-geometry.js";
import type { DetectorResult } from "../core/collision/narrowphase/detector-result.js";
import type { AabbTestCallback } from "../core/dynamics/callback/aabb-test-callback.js";
import { RigidBody } from "../core/dynamics/rigidbody/rigid-body.js";
import { RigidBodyConfig, RigidBodyType } from "../core/dynamics/rigidbody/rigid-body-config.js";
import { ShapeConfig } from "../core/dynamics/rigidbody/shape-config.js";
import { Shape } from "../core/dynamics/rigidbody/shape.js";
import type { World } from "../core/dynamics/world.js";
import { type BlockSource, BlockShapeTable, SHAPE_FULL_CUBE, SHAPE_NONE, shapeOf } from "./block-shapes.js";
import { CHUNK_CELLS, CHUNK_SIZE, chunkBoxes, type ChunkBox } from "./chunk-boxes.js";
import { bodyBounds, chunkKey, chunkOrigin, fluidRange, predictedChunks, rangeChunks, sleepingChunks } from "./terrain-scan.js";

/** 一种方块材质在碰撞体上的参数 */
export interface VoxelMaterial {
  friction: number;
  restitution: number;
}

export interface VoxelColliderOptions {
  /** 特殊形状表。默认为空表，即只有空气和完整方块 */
  table?: BlockShapeTable;
  /** 各种材质的参数，下标是方块编号里的材质编号。默认只有 0 号材质：摩擦 0.6、弹性 0 */
  materials?: readonly VoxelMaterial[];
  /** 请求范围的基础外扩（格）。默认 1 */
  margin?: number;
  /** 预读的时长（tick 数），0 关闭。默认 3 */
  preloadTicks?: number;
  /** 每次 update 的高优先读取预算，默认为 4。普通队列另有 2 个名额，并使用剩余预算。 */
  maxChunksPerUpdate?: number;
  /** 每次 update 的复查预算，默认为 12，每个刚体至少检查 2 格；0 表示关闭。 */
  revalidateBlocksPerUpdate?: number;
  /** 无引用块的缓存保留时间，单位为 update 次数，默认为 400。 */
  unloadDelay?: number;
  /** 当前地形未就绪时是否冻结刚体，默认为 true；false 将未读取区域视为空。 */
  suspend?: boolean;
  /**
   * 是否为活动刚体附加流体请求，默认为 false。
   * 形状包围盒水平外扩 1 格、向下 1.5 格，setFluidReach 启用时向上扩展 64 格。
   * 使用普通队列，仅计总引用，不影响地形就绪状态。
   */
  fluids?: boolean;
}

/** 一个 8³ 块 */
interface Chunk {
  ox: number; oy: number; oz: number;
  /** 方块编号和类型令牌，下标见 chunk-boxes.ts */
  cells: Int32Array;
  types: Int32Array;
  /** 流体码（见 block-shapes.ts）和有流体的格数 */
  fluids: Uint8Array;
  fluidCount: number;
  /** 待重读的单元集合；非空时该块未就绪。 */
  dirty: Set<number>;
  /** 静态体。块里没有碰撞时为 null */
  body: RigidBody | null;
  /** 静态体在世界里 */
  attached: boolean;
  lastUsed: number;
}

/** 每个动态刚体的请求 */
interface BodyState {
  body: RigidBody;
  /** 当前地形请求及其成员集合；休眠刚体使用休眠范围。 */
  keys: number[];
  keySet: Set<number>;
  /** 当前请求是否处于活动状态，包括等待地形的刚体。 */
  active: boolean;
  /** 预读请求（去掉当前请求里已有的块） */
  prefetch: number[];
  prefetchSet: Set<number>;
  /** 在等地形 */
  waiting: boolean;
  /** 当前范围外的流体请求及对应整数范围，fluidUp 启用时向上扩展 64 格。 */
  fluid: number[];
  fluidRange: number[];
  fluidKeys: number[] | null;
  fluidSet: Set<number>;
  fluidUp: boolean;
  /** 复查游标 */
  cursor: number;
  /** 最近一次处理它的 update 序号，用来发现离开世界的刚体 */
  seen: number;
  /** 计算休眠范围时的位姿：3 个位置分量及 9 个旋转矩阵分量。 */
  pose: number[];
}

/** 删除没有引用的块：每隔多少个 update 检查一次 */
const PRUNE_INTERVAL = 20;
/** 当前名额之外，普通队列另有的名额 */
const NORMAL_BUILDS = 2;
/** 用于固定 tick 估算的时长，按 20 TPS 计算。 */
const TICK = 0.05;
/** 内部面探测的边界偏移，薄盒取半边长的四分之一。 */
const FACE_INSET = 0.00001;

const scratchCells = new Int32Array(CHUNK_CELLS);
const scratchTypes = new Int32Array(CHUNK_CELLS);
const scratchFluids = new Uint8Array(CHUNK_CELLS);
const boxScratch: ChunkBox[] = [];
const fluidScratch = [0, 0, 0, 0, 0, 0];
const keyScratch: number[] = [];
const origin = [0, 0, 0];
const bounds = [0, 0, 0, 0, 0, 0];
const dirtyScratch: number[] = [];
const changedScratch: number[] = [];
const EMPTY: readonly number[] = [];
const EMPTY_SET: ReadonlySet<number> = new Set();

/** 两组键的内容是否相同（set 是第一组的成员集合，count 是它的个数） */
function sameKeys(set: ReadonlySet<number>, count: number, next: readonly number[]): boolean {
  if (count !== next.length) return false;
  for (let i = 0; i < next.length; i++) if (!set.has(next[i])) return false;
  return true;
}

export class VoxelCollider {
  readonly world: World;
  readonly source: BlockSource;
  readonly table: BlockShapeTable;
  /** 材质参数，下标是材质编号。0 号是默认材质 */
  readonly materials: VoxelMaterial[];
  readonly margin: number;
  readonly preloadTicks: number;
  readonly maxChunksPerUpdate: number;
  readonly revalidateBlocksPerUpdate: number;
  readonly unloadDelay: number;
  readonly suspend: boolean;
  readonly fluids: boolean;
  /** 已缓存的、有流体的块数 */
  private fluidChunks = 0;

  private readonly chunks = new Map<number, Chunk>();
  /** 所有请求（当前、睡眠、预读）对每个块的引用数，和其中 active 请求的引用数 */
  private readonly refs = new Map<number, number>();
  private readonly activeRefs = new Map<number, number>();
  /** 两条队列：Set 的插入顺序就是先进先出的顺序 */
  private readonly high = new Set<number>();
  private readonly normal = new Set<number>();
  private readonly states = new Map<RigidBody, BodyState>();
  /** 停用的刚体（见 setParked） */
  private readonly parked = new Set<RigidBody>();
  private updateCount = 0;
  private lastPrune = 0;
  /** chunkAt 上一次找到的块（删掉块时清空） */
  private lastChunk: Chunk | null = null;
  private readonly group = new Set<RigidBody>();
  private readonly groupQueue: RigidBody[] = [];
  private readonly wakeAabb = new Aabb();
  private readonly wakeQueue: RigidBody[] = [];
  private readonly wakeCallback: AabbTestCallback = {
    process: (shape) => {
      const b = shape.rigidBody!;
      if (!b.sleeping || b.type !== RigidBodyType.DYNAMIC || b.wakeLocked) return;
      b.wakeUp();
      this.wakeQueue.push(b);
    },
  };
  /**
   * 过滤地形内部面的接触点。
   * 将接触点限制在盒内并沿法线外推；探测点位于相邻盒内时丢弃该接触。
   * 探测点仍在当前盒内时保留。
   */
  private readonly filterContact = (shape: Shape, result: DetectorResult, first: boolean): void => {
    const g = shape.geom as BoxGeometry, t = shape.transform;
    // 法线改成从地形指向另一个形状
    const nx = first ? -result.nx : result.nx, ny = first ? -result.ny : result.ny, nz = first ? -result.nz : result.nz;
    const ix = Math.min(FACE_INSET, g.hx / 4), iy = Math.min(FACE_INSET, g.hy / 4), iz = Math.min(FACE_INSET, g.hz / 4);
    const ox = nx * ix * 2, oy = ny * iy * 2, oz = nz * iz * 2;
    for (let i = 0; i < result.numPoints;) {
      const p = result.points[i];
      const lx = (first ? p.p1x : p.p2x) - t.px, ly = (first ? p.p1y : p.p2y) - t.py, lz = (first ? p.p1z : p.p2z) - t.pz;
      const x = Math.max(-g.hx + ix, Math.min(g.hx - ix, lx)) + ox;
      const y = Math.max(-g.hy + iy, Math.min(g.hy - iy, ly)) + oy;
      const z = Math.max(-g.hz + iz, Math.min(g.hz - iz, lz)) + oz;
      const own = Math.abs(x) < g.hx && Math.abs(y) < g.hy && Math.abs(z) < g.hz;
      if (!own && this.containsPoint(t.px + x, t.py + y, t.pz + z)) {
        result.points[i] = result.points[--result.numPoints];
        result.points[result.numPoints] = p;
      } else {
        i++;
      }
    }
  };

  constructor(world: World, source: BlockSource, options: VoxelColliderOptions = {}) {
    this.world = world;
    this.source = source;
    this.table = options.table ?? new BlockShapeTable();
    this.materials = options.materials?.map((m) => ({ ...m })) ?? [{ friction: 0.6, restitution: 0 }];
    if (this.materials.length === 0) throw new Error("materials 至少要有一项（0 号材质）");
    this.margin = options.margin ?? 1;
    this.preloadTicks = options.preloadTicks ?? 3;
    this.maxChunksPerUpdate = options.maxChunksPerUpdate ?? 4;
    this.revalidateBlocksPerUpdate = options.revalidateBlocksPerUpdate ?? 12;
    this.unloadDelay = options.unloadDelay ?? 400;
    this.suspend = options.suspend ?? true;
    this.fluids = options.fluids ?? false;
    if (!(this.margin >= 0) || !Number.isFinite(this.margin)) throw new Error(`margin 必须是不小于 0 的有限数，收到 ${this.margin}`);
    if (!Number.isSafeInteger(this.preloadTicks) || this.preloadTicks < 0) throw new Error(`preloadTicks 必须是不小于 0 的整数，收到 ${this.preloadTicks}`);
    if (!Number.isSafeInteger(this.maxChunksPerUpdate) || this.maxChunksPerUpdate < 1) throw new Error(`maxChunksPerUpdate 必须是不小于 1 的整数，收到 ${this.maxChunksPerUpdate}`);
    if (!Number.isSafeInteger(this.revalidateBlocksPerUpdate) || this.revalidateBlocksPerUpdate < 0) throw new Error(`revalidateBlocksPerUpdate 必须是不小于 0 的整数，收到 ${this.revalidateBlocksPerUpdate}`);
    if (!Number.isSafeInteger(this.unloadDelay) || this.unloadDelay < 0) throw new Error(`unloadDelay 必须是不小于 0 的整数，收到 ${this.unloadDelay}`);
  }

  /** 缓存的块数 */
  get cachedChunks(): number {
    return this.chunks.size;
  }

  /** 挂在世界上的块数（包括没有碰撞的块） */
  get attachedChunks(): number {
    let n = 0;
    for (const c of this.chunks.values()) if (c.attached) n++;
    return n;
  }

  /** 排队等着读的块数 */
  get pendingChunks(): number {
    return this.high.size + this.normal.size;
  }

  /** 刚体是否等待地形，包括已停用的刚体。 */
  isWaiting(body: RigidBody): boolean {
    return this.states.get(body)?.waiting === true;
  }

  /**
   * 切换刚体停用状态。
   * 停用时释放地形请求并冻结关节相连的一组刚体；恢复时等待当前地形就绪。suspend=false 时无效果。
   */
  setParked(body: RigidBody, parked: boolean): void {
    if (!this.suspend) return;
    if (parked) this.parked.add(body);
    else this.parked.delete(body);
  }

  isParked(body: RigidBody): boolean {
    return this.parked.has(body);
  }

  /**
   * 一格的流体码（见 block-shapes.ts）。只查缓存，块没读到时返回 0（当作干的）。
   * 刚体当前范围、睡眠范围每侧外扩 margin，margin 不小于 1 时覆盖流体计算要查的格子
   */
  fluidAt(x: number, y: number, z: number): number {
    const c = this.chunkAt(x, y, z);
    return c === null ? 0 : c.fluids[(x - c.ox) + (z - c.oz) * CHUNK_SIZE + (y - c.oy) * 64];
  }

  /** 已缓存块中是否存在流体。 */
  hasAnyFluid(): boolean {
    return this.fluidChunks > 0;
  }

  /** 一格没有碰撞（只查缓存，块没读到时返回 false） */
  isTerrainEmpty(x: number, y: number, z: number): boolean {
    const c = this.chunkAt(x, y, z);
    return c !== null && shapeOf(c.cells[(x - c.ox) + (z - c.oz) * CHUNK_SIZE + (y - c.oy) * 64]) === SHAPE_NONE;
  }

  /** 方块坐标闭区间里有没有流体：只看覆盖到的已缓存块的流体格数 */
  hasFluidNear(minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): boolean {
    const x0 = Math.floor(minX / CHUNK_SIZE), y0 = Math.floor(minY / CHUNK_SIZE), z0 = Math.floor(minZ / CHUNK_SIZE);
    const x1 = Math.floor(maxX / CHUNK_SIZE), y1 = Math.floor(maxY / CHUNK_SIZE), z1 = Math.floor(maxZ / CHUNK_SIZE);
    for (let cy = y0; cy <= y1; cy++) {
      for (let cz = z0; cz <= z1; cz++) {
        for (let cx = x0; cx <= x1; cx++) {
          const c = this.chunks.get(chunkKey(cx, cy, cz));
          if (c !== undefined && c.fluidCount > 0) return true;
        }
      }
    }
    return false;
  }

  /** 方块所在的已缓存块，没有时返回 null。连续查同一块时不查表 */
  private chunkAt(x: number, y: number, z: number): Chunk | null {
    let c = this.lastChunk;
    if (c === null || x < c.ox || x >= c.ox + CHUNK_SIZE || y < c.oy || y >= c.oy + CHUNK_SIZE || z < c.oz || z >= c.oz + CHUNK_SIZE) {
      c = this.chunks.get(chunkKey(Math.floor(x / CHUNK_SIZE), Math.floor(y / CHUNK_SIZE), Math.floor(z / CHUNK_SIZE))) ?? null;
      if (c === null) return null;
      this.lastChunk = c;
    }
    return c;
  }
  /**
   * 使方块闭区间内的缓存失效，向外扩展 1 格以覆盖邻接形状。
   * cachedOnly=true 仅处理已缓存块；false 同时重新排队未缓存的被引用块。
   */
  blocksChanged(minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number, cachedOnly = false): void {
    if (!Number.isFinite(minX + minY + minZ + maxX + maxY + maxZ)) return;
    const x0 = Math.floor(minX) - 1, y0 = Math.floor(minY) - 1, z0 = Math.floor(minZ) - 1;
    const x1 = Math.floor(maxX) + 1, y1 = Math.floor(maxY) + 1, z1 = Math.floor(maxZ) + 1;
    if (x0 > x1 || y0 > y1 || z0 > z1) return;
    const cx0 = Math.floor(x0 / CHUNK_SIZE), cy0 = Math.floor(y0 / CHUNK_SIZE), cz0 = Math.floor(z0 / CHUNK_SIZE);
    const cx1 = Math.floor(x1 / CHUNK_SIZE), cy1 = Math.floor(y1 / CHUNK_SIZE), cz1 = Math.floor(z1 / CHUNK_SIZE);
    const span = (cx1 - cx0 + 1) * (cy1 - cy0 + 1) * (cz1 - cz0 + 1);
    // 较大失效范围遍历已有缓存及引用，避免枚举整个坐标范围。
    if (span > this.chunks.size + this.refs.size) {
      keyScratch.length = 0;
      for (const key of this.chunks.keys()) keyScratch.push(key);
      if (!cachedOnly) for (const key of this.refs.keys()) if (!this.chunks.has(key)) keyScratch.push(key);
      for (let i = 0; i < keyScratch.length; i++) {
        const key = keyScratch[i];
        chunkOrigin(key, origin);
        const cx = origin[0] / CHUNK_SIZE, cy = origin[1] / CHUNK_SIZE, cz = origin[2] / CHUNK_SIZE;
        if (cx >= cx0 && cx <= cx1 && cy >= cy0 && cy <= cy1 && cz >= cz0 && cz <= cz1) this.invalidate(key, x0, y0, z0, x1, y1, z1);
      }
      return;
    }
    for (let cy = cy0; cy <= cy1; cy++) for (let cz = cz0; cz <= cz1; cz++) for (let cx = cx0; cx <= cx1; cx++) {
      const key = chunkKey(cx, cy, cz);
      if (key < 0 || (cachedOnly && !this.chunks.has(key))) continue;
      this.invalidate(key, x0, y0, z0, x1, y1, z1);
    }
  }

  /**
   * 更新地形缓存和刚体等待状态。
   * 在 world.step 前调用；dt 为下一步的时长，单位为秒。
   */
  update(dt = TICK): void {
    this.updateCount++;
    if (this.updateCount - this.lastPrune >= PRUNE_INTERVAL) {
      this.prune();
      this.lastPrune = this.updateCount;
    }
    this.revalidate();
    const tick = this.updateCount;
    for (let b = this.world.rigidBodyList; b !== null; b = b.next) {
      if (b.type !== RigidBodyType.DYNAMIC) continue;
      let s = this.states.get(b);
      if (s === undefined) { s = this.newState(b); this.states.set(b, s); }
      s.seen = tick;
      if (this.parked.has(b)) {
        this.setRequest(s, EMPTY, false);
        this.releasePrefetch(s);
        this.releaseFluid(s);
        this.pause(s);
        continue;
      }
      if (s.waiting) {
        b.fx = 0; b.fy = 0; b.fz = 0; b.tx = 0; b.ty = 0; b.tz = 0;
        this.requestCurrent(s, dt);
        if (!this.ready(s)) { this.pause(s); continue; }
        this.resume(s);
      }
      if (b.sleeping) { this.releaseFluid(s); this.requestSleeping(s); continue; }
      this.requestCurrent(s, dt);
      this.requestPrefetch(s, dt);
      this.requestFluid(s);
    }
    this.dropLeft(tick);
    const built = this.processQueue(this.high, this.maxChunksPerUpdate);
    this.processQueue(this.normal, Math.max(0, NORMAL_BUILDS + this.maxChunksPerUpdate - built));
    if (this.suspend) this.settle(dt);
  }

  /** 移除全部块并解除刚体的地形等待状态。 */
  dispose(): void {
    for (const c of this.chunks.values()) this.detach(c);
    for (const s of this.states.values()) {
      if (!s.waiting) continue;
      s.body.wakeLocked = false;
      s.body.wakeUp();
    }
    this.chunks.clear();
    this.fluidChunks = 0;
    this.lastChunk = null;
    this.refs.clear();
    this.activeRefs.clear();
    this.high.clear();
    this.normal.clear();
    this.states.clear();
    this.parked.clear();
  }

  /** 按当前请求的就绪状态冻结或恢复刚体，关节相连的一组保持一致。 */
  private settle(dt: number): void {
    const group = this.group, queue = this.groupQueue;
    group.clear();
    queue.length = 0;
    for (const s of this.states.values()) {
      const b = s.body;
      if (this.parked.has(b)) {
        if (!group.has(b)) { group.add(b); queue.push(b); }
        continue;
      }
      if (b.sleeping && !s.waiting) continue;
      // 本 tick 唤醒的刚体切换到当前运动范围。
      if (!s.active) this.requestCurrent(s, dt);
      if (!this.ready(s) && !group.has(b)) { group.add(b); queue.push(b); }
    }
    for (let i = 0; i < queue.length; i++) {
      for (let l = queue[i].jointLinkList; l !== null; l = l.next) {
        const other = l.other!;
        if (other.type !== RigidBodyType.DYNAMIC || group.has(other) || !this.states.has(other)) continue;
        group.add(other);
        queue.push(other);
      }
    }
    for (const s of this.states.values()) {
      if (group.has(s.body)) this.pause(s);
      else if (s.waiting) this.resume(s);
    }
  }
  private newState(body: RigidBody): BodyState {
    return {
      body, keys: [], keySet: new Set(), active: false, prefetch: [], prefetchSet: new Set(), fluid: [], fluidSet: new Set(), fluidRange: [0, 0, 0, 0, 0, 0], fluidKeys: null, fluidUp: false, waiting: false, cursor: 0, seen: 0,
      pose: [NaN, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
  }

  /** 当前请求的块是否全部缓存且未失效。 */
  private ready(s: BodyState): boolean {
    const keys = s.keys;
    for (let i = 0; i < keys.length; i++) {
      const c = this.chunks.get(keys[i]);
      if (c === undefined || c.dirty.size > 0) return false;
    }
    return true;
  }

  /** 冻结刚体，保留速度、锁定唤醒并清除本步外力。 */
  private pause(s: BodyState): void {
    const b = s.body;
    s.waiting = true;
    b.wakeLocked = true;
    if (!b.sleeping) b.sleep();
    b.fx = 0; b.fy = 0; b.fz = 0; b.tx = 0; b.ty = 0; b.tz = 0;
  }

  private resume(s: BodyState): void {
    s.waiting = false;
    s.body.wakeLocked = false;
    this.wakeWithNeighbors(s.body);
  }

  private requestCurrent(s: BodyState, dt: number): void {
    predictedChunks(s.body, dt, this.margin, false, keyScratch);
    this.setRequest(s, keyScratch, true);
  }

  /** 睡眠范围：位姿没变就沿用上一次的 */
  private requestSleeping(s: BodyState): void {
    const tf = s.body.transform, p = s.pose;
    if (!s.active && p[0] === tf.px && p[1] === tf.py && p[2] === tf.pz && p[3] === tf.r00 && p[4] === tf.r01 && p[5] === tf.r02 &&
      p[6] === tf.r10 && p[7] === tf.r11 && p[8] === tf.r12 && p[9] === tf.r20 && p[10] === tf.r21 && p[11] === tf.r22) return;
    p[0] = tf.px; p[1] = tf.py; p[2] = tf.pz; p[3] = tf.r00; p[4] = tf.r01; p[5] = tf.r02;
    p[6] = tf.r10; p[7] = tf.r11; p[8] = tf.r12; p[9] = tf.r20; p[10] = tf.r21; p[11] = tf.r22;
    this.releasePrefetch(s);
    sleepingChunks(s.body, this.margin, keyScratch);
    this.setRequest(s, keyScratch, false);
  }

  /** 更新当前范围外的预读请求。使用普通队列，仅计总引用，不影响就绪状态。 */
  private requestPrefetch(s: BodyState, dt: number): void {
    if (this.preloadTicks === 0) return;
    predictedChunks(s.body, dt * this.preloadTicks, this.margin, true, keyScratch);
    let n = 0;
    for (let i = 0; i < keyScratch.length; i++) if (!s.keySet.has(keyScratch[i])) keyScratch[n++] = keyScratch[i];
    keyScratch.length = n;
    if (sameKeys(s.prefetchSet, s.prefetch.length, keyScratch)) return;
    const old = s.prefetch, oldSet = s.prefetchSet;
    const next = keyScratch.slice(), nextSet = new Set(next);
    for (let i = 0; i < next.length; i++) if (!oldSet.has(next[i])) this.addRef(next[i]);
    for (let i = 0; i < old.length; i++) if (!nextSet.has(old[i])) this.releaseRef(old[i]);
    s.prefetch = next;
    s.prefetchSet = nextSet;
  }

  /** 是否将流体请求向上扩展 64 格，供气腔液面查询使用；仅影响后续请求。 */
  setFluidReach(body: RigidBody, up: boolean): void {
    const s = this.states.get(body);
    if (s !== undefined) s.fluidUp = up;
  }

  /** 更新当前范围外的流体请求。使用普通队列，仅计总引用，不影响就绪状态。 */
  private requestFluid(s: BodyState): void {
    if (!this.fluids) return;
    const r = fluidScratch;
    if (!fluidRange(s.body, s.fluidUp, r)) { this.releaseFluid(s); return; }
    // 整数范围和当前请求未变化时复用请求。
    const p = s.fluidRange;
    if (s.fluidKeys === s.keys && p[0] === r[0] && p[1] === r[1] && p[2] === r[2] && p[3] === r[3] && p[4] === r[4] && p[5] === r[5]) return;
    p[0] = r[0]; p[1] = r[1]; p[2] = r[2]; p[3] = r[3]; p[4] = r[4]; p[5] = r[5];
    s.fluidKeys = s.keys;
    rangeChunks(r, keyScratch);
    let n = 0;
    for (let i = 0; i < keyScratch.length; i++) if (!s.keySet.has(keyScratch[i])) keyScratch[n++] = keyScratch[i];
    keyScratch.length = n;
    if (sameKeys(s.fluidSet, s.fluid.length, keyScratch)) return;
    const old = s.fluid, oldSet = s.fluidSet;
    const next = keyScratch.slice(), nextSet = new Set(next);
    for (let i = 0; i < next.length; i++) if (!oldSet.has(next[i])) this.addRef(next[i]);
    for (let i = 0; i < old.length; i++) if (!nextSet.has(old[i])) this.releaseRef(old[i]);
    s.fluid = next;
    s.fluidSet = nextSet;
  }

  private releaseFluid(s: BodyState): void {
    s.fluidKeys = null;
    if (s.fluid.length === 0) return;
    for (let i = 0; i < s.fluid.length; i++) this.releaseRef(s.fluid[i]);
    s.fluid = EMPTY as number[];
    s.fluidSet = EMPTY_SET as Set<number>;
  }

  private releasePrefetch(s: BodyState): void {
    for (let i = 0; i < s.prefetch.length; i++) this.releaseRef(s.prefetch[i]);
    s.prefetch = EMPTY as number[];
    s.prefetchSet = EMPTY_SET as Set<number>;
  }

  /** 替换当前请求，先更新活动引用，再更新总引用。等待地形的请求计为活动。 */
  private setRequest(s: BodyState, next: readonly number[], active: boolean): void {
    const same = sameKeys(s.keySet, s.keys.length, next);
    if (same && s.active === active) return;
    const old = s.keys, oldSet = s.keySet;
    const keys = same ? old : next.slice(), keySet = same ? oldSet : new Set(keys);
    if (s.active) for (let i = 0; i < old.length; i++) if (!active || !keySet.has(old[i])) this.releaseActive(old[i]);
    if (active) for (let i = 0; i < keys.length; i++) if (!s.active || !oldSet.has(keys[i])) this.addActive(keys[i]);
    if (!same) {
      for (let i = 0; i < keys.length; i++) if (!oldSet.has(keys[i])) this.addRef(keys[i]);
      for (let i = 0; i < old.length; i++) if (!keySet.has(old[i])) this.releaseRef(old[i]);
    }
    s.keys = keys;
    s.keySet = keySet;
    s.active = active;
  }

  private addRef(key: number): void {
    const n = this.refs.get(key) ?? 0;
    this.refs.set(key, n + 1);
    if (n === 0) this.enqueue(key, false);
  }

  private releaseRef(key: number): void {
    const n = this.refs.get(key) ?? 0;
    if (n > 1) { this.refs.set(key, n - 1); return; }
    this.refs.delete(key);
    this.high.delete(key);
    this.normal.delete(key);
    const c = this.chunks.get(key);
    if (c !== undefined) c.lastUsed = this.updateCount;
  }

  private addActive(key: number): void {
    const n = this.activeRefs.get(key) ?? 0;
    this.activeRefs.set(key, n + 1);
    if (n > 0) return;
    const c = this.chunks.get(key);
    if (c !== undefined && c.dirty.size === 0) this.attach(c);
    else this.enqueue(key, true);
  }

  private releaseActive(key: number): void {
    const n = this.activeRefs.get(key) ?? 0;
    if (n > 1) { this.activeRefs.set(key, n - 1); return; }
    this.activeRefs.delete(key);
    const c = this.chunks.get(key);
    if (c !== undefined) this.detach(c);
    if (this.high.delete(key) && this.refs.has(key)) this.normal.add(key);
  }

  /** 未缓存或已失效的块入队；高优先请求升级队列，已入队项保留顺序。 */
  private enqueue(key: number, highPriority: boolean): void {
    const c = this.chunks.get(key);
    if (c !== undefined && c.dirty.size === 0) return;
    if (highPriority) {
      this.normal.delete(key);
      this.high.add(key);
    } else if (!this.high.has(key)) {
      this.normal.add(key);
    }
  }

  /** 释放离开世界或不再动态的刚体请求，并解除其地形等待状态。 */
  private dropLeft(tick: number): void {
    for (const [b, s] of this.states) {
      if (s.seen === tick) continue;
      this.setRequest(s, EMPTY, false);
      this.releasePrefetch(s);
      this.releaseFluid(s);
      if (s.waiting) {
        b.wakeLocked = false;
        b.wakeUp();
      }
      this.states.delete(b);
      this.parked.delete(b);
    }
  }
  /**
   * 按队列顺序尝试读取最多 budget 个块，返回尝试次数。
   * 读取失败的块移至队尾，并停止本次队列读取。
   */
  private processQueue(queue: Set<number>, budget: number): number {
    if (budget <= 0) return 0;
    let built = 0;
    for (const key of queue) {
      queue.delete(key);
      const c = this.chunks.get(key);
      if (c !== undefined && c.dirty.size === 0) continue;
      built++;
      if (!this.scan(key, c)) {
        queue.add(key);
        break;
      }
      if (built >= budget) break;
    }
    return built;
  }

  /** 读一块：没缓存的整块读，缓存了的只重读脏格。读完返回 true */
  private scan(key: number, old: Chunk | undefined): boolean {
    if (old === undefined) {
      chunkOrigin(key, origin);
      const ox = origin[0], oy = origin[1], oz = origin[2];
      const cells = new Int32Array(CHUNK_CELLS), types = new Int32Array(CHUNK_CELLS), fluids = new Uint8Array(CHUNK_CELLS);
      if (!this.source.readChunk(ox, oy, oz, cells, types, fluids)) return false;
      const c: Chunk = {
        ox, oy, oz, cells, types, fluids, fluidCount: countFluids(fluids), dirty: new Set(), body: null, attached: false, lastUsed: this.updateCount,
      };
      c.body = this.buildBody(c);
      this.chunks.set(key, c);
      if (c.fluidCount > 0) this.fluidChunks++;
      if (this.activeRefs.has(key)) this.attach(c);
      return true;
    }
    const indices = dirtyScratch;
    indices.length = 0;
    for (const i of old.dirty) indices.push(i);
    scratchCells.set(old.cells);
    scratchTypes.set(old.types);
    scratchFluids.set(old.fluids);
    if (!this.source.refreshCells(old.ox, old.oy, old.oz, scratchCells, scratchTypes, scratchFluids, indices)) return false;
    const changed = changedScratch;
    changed.length = 0;
    let solid = false;
    for (let n = 0; n < indices.length; n++) {
      const i = indices[n];
      if (scratchCells[i] !== old.cells[i]) { solid = true; changed.push(i); }
      else if (scratchFluids[i] !== old.fluids[i]) changed.push(i);
    }
    old.dirty.clear();
    old.types.set(scratchTypes);
    old.lastUsed = this.updateCount;
    if (changed.length > 0) {
      const had = old.fluidCount > 0;
      old.fluids.set(scratchFluids);
      old.fluidCount = countFluids(old.fluids);
      if (had && old.fluidCount === 0) this.fluidChunks--;
      else if (!had && old.fluidCount > 0) this.fluidChunks++;
    }
    if (!solid) {
      // 仅流体变化时保留静态碰撞体。
      if (this.activeRefs.has(key)) this.attach(old);
      if (changed.length > 0) this.wakeChanged(key, old, changed);
      return true;
    }
    // 碰撞数据变化时重建静态体，仅唤醒受变化单元影响的休眠刚体。
    old.cells.set(scratchCells);
    const wasAttached = old.attached;
    if (wasAttached && old.body !== null) this.world.removeRigidBody(old.body, false);
    old.attached = false;
    old.body = this.buildBody(old);
    if (this.activeRefs.has(key)) this.attach(old);
    this.wakeChanged(key, old, changed);
    return true;
  }

  private attach(c: Chunk): void {
    if (c.attached || c.dirty.size > 0) return;
    if (c.body !== null) this.world.addRigidBody(c.body);
    c.attached = true;
  }

  /** 摘下块的静态体，不叫醒接触着它的刚体 */
  private detach(c: Chunk): void {
    if (!c.attached) return;
    if (c.body !== null) this.world.removeRigidBody(c.body, false);
    c.attached = false;
  }

  /** 块里的盒子做成一个静态体，放在块中心；没有盒子时返回 null */
  private buildBody(c: Chunk): RigidBody | null {
    const boxes = boxScratch;
    boxes.length = 0;
    chunkBoxes(c.cells, this.table, c.ox, c.oy, c.oz, boxes);
    if (boxes.length === 0) return null;
    const config = new RigidBodyConfig();
    config.type = RigidBodyType.STATIC;
    const cx = c.ox + CHUNK_SIZE / 2, cy = c.oy + CHUNK_SIZE / 2, cz = c.oz + CHUNK_SIZE / 2;
    config.transform.px = cx; config.transform.py = cy; config.transform.pz = cz;
    const body = new RigidBody(config);
    const shapes: Shape[] = [];
    for (let i = 0; i < boxes.length; i++) {
      const b = boxes[i], hx = b.sizeX / 2, hy = b.sizeY / 2, hz = b.sizeZ / 2;
      const sc = new ShapeConfig(new BoxGeometry(hx, hy, hz));
      sc.transform.px = b.minX + hx - cx; sc.transform.py = b.minY + hy - cy; sc.transform.pz = b.minZ + hz - cz;
      const m = this.materials[b.material] ?? this.materials[0];
      sc.friction = m.friction;
      sc.restitution = m.restitution;
      // 地形属于分组 1；刚体形状的掩码里有第 1 位就会和它碰撞，不受刚体自己的分组影响
      sc.collisionMask = -1;
      const shape = new Shape(sc);
      shape.contactFilter = this.filterContact;
      shapes.push(shape);
    }
    body.addShapes(shapes);
    return body;
  }
  /** 点是否在挂在世界上的块的碰撞里。只查缓存，求解期间不读方块 */
  private containsPoint(x: number, y: number, z: number): boolean {
    const bx = Math.floor(x), by = Math.floor(y), bz = Math.floor(z);
    // 查询范围包含下方一格，以覆盖高出本格的栅栏和墙。
    for (let dy = 0; dy <= 1; dy++) {
      const yy = by - dy;
      const c = this.chunks.get(chunkKey(Math.floor(bx / CHUNK_SIZE), Math.floor(yy / CHUNK_SIZE), Math.floor(bz / CHUNK_SIZE)));
      if (c === undefined || !c.attached) continue;
      const id = shapeOf(c.cells[(bx - c.ox) + (bz - c.oz) * CHUNK_SIZE + (yy - c.oy) * 64]);
      if (id === SHAPE_NONE) continue;
      if (id === SHAPE_FULL_CUBE) { if (dy === 0) return true; continue; }
      const boxes = this.table.get(id);
      if (!boxes) continue;
      const lx = x - bx, ly = y - yy, lz = z - bz;
      for (const b of boxes) if (lx > b[0] && lx < b[3] && ly > b[1] && ly < b[4] && lz > b[2] && lz < b[5]) return true;
    }
    return false;
  }

  /** 静态体重建后，唤醒休眠范围与变化单元相交的请求者；等待地形的刚体保持冻结。 */
  private wakeChanged(key: number, c: Chunk, changed: readonly number[]): void {
    for (const s of this.states.values()) {
      if (s.waiting || !s.keySet.has(key) || !s.body.sleeping) continue;
      if (!bodyBounds(s.body, bounds)) continue;
      const m = this.margin;
      const x0 = Math.floor(bounds[0] - m), y0 = Math.floor(bounds[1] - m), z0 = Math.floor(bounds[2] - m);
      const x1 = Math.floor(bounds[3] + m), y1 = Math.floor(bounds[4] + m), z1 = Math.floor(bounds[5] + m);
      for (let n = 0; n < changed.length; n++) {
        const i = changed[n], x = c.ox + (i & 7), y = c.oy + (i >> 6), z = c.oz + ((i >> 3) & 7);
        if (x < x0 || x > x1 || y < y0 || y > y1 || z < z0 || z > z1) continue;
        this.wakeWithNeighbors(s.body);
        break;
      }
    }
  }

  /** 唤醒刚体及包围盒重叠的休眠动态刚体，跳过锁定唤醒的对象。 */
  private wakeWithNeighbors(body: RigidBody): void {
    if (!body.sleeping) return;
    body.wakeUp();
    const queue = this.wakeQueue, a = this.wakeAabb;
    queue.length = 0;
    queue.push(body);
    for (let i = 0; i < queue.length; i++) {
      const b = queue[i];
      if (b.shapeList === null) continue;
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
      this.world.aabbTest(a, this.wakeCallback);
    }
    queue.length = 0;
  }

  /**
   * 按动态刚体轮询已缓存且未失效的方块，跳过等待地形的刚体。
   * 每个刚体预算为 max(2, floor(总预算 / 刚体数))；范围外扩 1 格，启用流体时向下扩展 2 格。
   * 游标按 x、z、y 递进，发现变化时立即使对应缓存失效。
   */
  private revalidate(): void {
    if (this.revalidateBlocksPerUpdate === 0) return;
    let n = 0;
    for (let b = this.world.rigidBodyList; b !== null; b = b.next) if (b.type === RigidBodyType.DYNAMIC && this.states.get(b)?.waiting !== true) n++;
    if (n === 0) return;
    const budget = Math.max(2, Math.floor(this.revalidateBlocksPerUpdate / n));
    const changed = changedScratch;
    for (let b = this.world.rigidBodyList; b !== null; b = b.next) {
      if (b.type !== RigidBodyType.DYNAMIC) continue;
      const s = this.states.get(b);
      if (s === undefined || s.waiting || !bodyBounds(b, bounds)) continue;
      const x0 = Math.floor(bounds[0]) - 1, y0 = this.fluids ? Math.floor(bounds[1] - 1.5) : Math.floor(bounds[1]) - 1, z0 = Math.floor(bounds[2]) - 1;
      const sx = Math.floor(bounds[3]) + 2 - x0, sy = Math.floor(bounds[4]) + 2 - y0, sz = Math.floor(bounds[5]) + 2 - z0;
      const volume = sx * sy * sz;
      if (!(volume > 0) || !Number.isSafeInteger(volume)) continue;
      let cursor = s.cursor % volume;
      changed.length = 0;
      let c: Chunk | undefined, last = -2;
      for (let k = 0; k < budget; k++) {
        const x = x0 + cursor % sx, z = z0 + Math.floor(cursor / sx) % sz, y = y0 + Math.floor(cursor / (sx * sz));
        cursor = (cursor + 1) % volume;
        const key = chunkKey(Math.floor(x / CHUNK_SIZE), Math.floor(y / CHUNK_SIZE), Math.floor(z / CHUNK_SIZE));
        if (key !== last) { c = this.chunks.get(key); last = key; }
        if (c === undefined || c.dirty.size > 0) continue;
        const i = (x - c.ox) + (z - c.oz) * CHUNK_SIZE + (y - c.oy) * 64;
        if (this.source.changed(x, y, z, c.cells[i], c.types[i], c.fluids[i])) changed.push(x, y, z);
      }
      s.cursor = cursor;
      for (let k = 0; k < changed.length; k += 3) this.blocksChanged(changed[k], changed[k + 1], changed[k + 2], changed[k], changed[k + 1], changed[k + 2], true);
    }
  }

  /** 标记范围内的已缓存单元失效，并重新排队有引用的块；当前请求优先于预读请求。 */
  private invalidate(key: number, x0: number, y0: number, z0: number, x1: number, y1: number, z1: number): void {
    const c = this.chunks.get(key);
    if (c !== undefined) {
      const ax = Math.max(x0, c.ox), ay = Math.max(y0, c.oy), az = Math.max(z0, c.oz);
      const bx = Math.min(x1, c.ox + 7), by = Math.min(y1, c.oy + 7), bz = Math.min(z1, c.oz + 7);
      for (let y = ay; y <= by; y++) for (let z = az; z <= bz; z++) for (let x = ax; x <= bx; x++) c.dirty.add((x - c.ox) + (z - c.oz) * CHUNK_SIZE + (y - c.oy) * 64);
    }
    this.normal.delete(key);
    if (!this.refs.has(key)) return;
    let current = false;
    for (const s of this.states.values()) if (s.keySet.has(key)) { current = true; break; }
    this.enqueue(key, current);
  }

  /** 删掉没有引用、unloadDelay 个 update 没用过的块 */
  private prune(): void {
    for (const [key, c] of this.chunks) {
      if (this.refs.has(key) || this.updateCount - c.lastUsed <= this.unloadDelay) continue;
      this.detach(c);
      if (c.fluidCount > 0) this.fluidChunks--;
      this.chunks.delete(key);
      if (this.lastChunk === c) this.lastChunk = null;
    }
  }
}

function countFluids(fluids: Uint8Array): number {
  let n = 0;
  for (let i = 0; i < CHUNK_CELLS; i++) if (fluids[i] !== 0) n++;
  return n;
}
