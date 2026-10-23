// 从维度读取方块，转换成 world/ 层使用的方块编号（形状编号 | 材质编号 << 16）和类型令牌。
// 空块由 containsBlock 跳过，非空块逐格读取并按类型缓存解析结果。
// 脏格按旧类型分组筛选变化；流体和含水状态逐格复查。
// 体积构造和错误分类由调用方提供。
import type { Block, Dimension } from "@minecraft/server";
import {
  BlockShapeTable, type BlockSource, type LocalBox, FLUID_LAVA, FLUID_NONE, FLUID_WATER, packBlock, SHAPE_FULL_CUBE, SHAPE_NONE,
  SHAPE_UNLOADED, shapeOf, TYPE_AIR, TYPE_RESOLVE,
} from "../world/block-shapes.js";
import { type BlockStates, isStateIndependent, isStateShaped, needsBlockContext, resolveShapeId, SHAPE_COUNT, SHAPE_UNKNOWN } from "./block-shapes.js";
import { contextualBoxes, stateBoxes, type NeighborReader } from "./block-neighbors.js";
import type { BlockRegistry } from "./block-registry.js";
import { MATERIAL_DEFAULT, resolveMaterialId } from "./materials.js";
import type { BlockCollisionResolver, UnknownBlockPolicy } from "./terrain.js";

/** 读取方块时遇到的错误种类 */
export type BlockReadError = "unloaded" | "outOfBounds" | "other";

/** 方块读取所需的 Minecraft 接口。 */
export interface BlockSourceHost {
  /** 创建 BlockVolume（两个角都包含在内） */
  createVolume(minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): unknown;
  /** 创建稀疏原生体积；省略时脏格逐格重读 */
  createListVolume?(locations: { x: number; y: number; z: number }[]): unknown;
  /** 判断 getBlock、containsBlock、getBlocks 抛出的错误属于哪一类 */
  classifyError(error: unknown): BlockReadError;
  /** 判断游戏是否支持指定方块类型。受支持的光源方块参与空块过滤。 */
  typeExists?(typeId: string): boolean;
}

/** 把方块类型和状态变成方块编号 */
export interface BlockResolverOptions {
  /** 用户的方块登记表：登记了的方块先按登记表取形状和材质，没有登记（或没写碰撞盒）的继续往下 */
  registry?: BlockRegistry;
  /** 用户提供的碰撞形状来源，返回 undefined 时改用内置数据 */
  collision?: BlockCollisionResolver;
  /** 未知方块的处理方式，默认为 "fullCube"。 */
  unknownBlock?: UnknownBlockPolicy;
  /** 所有方块统一使用这个材质编号（给定时忽略方块类型）。默认按方块类型取 */
  fixedMaterial?: number;
}

const FULL_CUBE_BOX = [0, 0, 0, 1, 1, 1].join();
const NO_STATES = (): BlockStates => ({});
/** 这些类型本身就是流体：读 liquid_depth 换算成流体码。气泡柱没有深度，按源方块算 */
const FLUID_TYPES = new Map<string, number>([
  ["minecraft:water", FLUID_WATER], ["minecraft:flowing_water", FLUID_WATER], ["minecraft:bubble_column", FLUID_WATER],
  ["minecraft:lava", FLUID_LAVA], ["minecraft:flowing_lava", FLUID_LAVA],
]);
/** 按类型缓存状态无关的方块；自定义 collision 的结果按类型和状态缓存。 */
export class BlockResolver {
  private readonly stateless = new Map<string, number>();
  private readonly custom = new Map<string, number | null>();
  private readonly customShapes = new Map<string, number>();
  private nextShapeId = SHAPE_COUNT;
  private readonly registry: BlockRegistry | undefined;
  private readonly collision: BlockCollisionResolver | undefined;
  private readonly unknownBlock: UnknownBlockPolicy;
  private readonly fixedMaterial: number | undefined;
  /** 类型 → 令牌；令牌 → 类型、方块编号。0 号是空气 */
  private readonly tokens = new Map<string, number>([["minecraft:air", TYPE_AIR]]);
  private readonly tokenTypes: string[] = ["minecraft:air"];
  private readonly tokenIds: number[] = [SHAPE_NONE];

  constructor(readonly table: BlockShapeTable, options: BlockResolverOptions = {}) {
    this.registry = options.registry;
    this.collision = options.collision;
    this.unknownBlock = options.unknownBlock ?? "fullCube";
    this.fixedMaterial = options.fixedMaterial;
  }

  /** 有没有用户提供的 collision（有的话同一种方块的形状可能随状态变，不能按类型批量判断） */
  get hasCustomCollision(): boolean {
    return this.collision !== undefined;
  }

  /** 这个类型有没有登记碰撞盒（判断整块是否为空时，登记了碰撞盒的光源方块不能排除） */
  hasRegisteredCollision(typeId: string): boolean {
    const entry = this.registry?.get(typeId);
    return entry !== undefined && (entry.hasCollision || entry.terrainStateful);
  }

  /** 清除解析缓存，保留已登记形状和类型令牌。 */
  clearCache(): void {
    this.stateless.clear();
    this.custom.clear();
  }

  /**
   * 类型令牌：形状和材质只由类型决定的方块按类型编号（从 1 开始），其余为 TYPE_RESOLVE。空气是 TYPE_AIR。
   * 登记表里登记了碰撞盒、碰撞和材质不随状态变的方块，不受 collision 函数影响，也按类型编号
   */
  token(typeId: string): number {
    const known = this.tokens.get(typeId);
    if (known !== undefined) return known;
    let t = TYPE_RESOLVE;
    const entry = this.registry?.get(typeId);
    let byType: boolean;
    if (entry !== undefined && entry.terrainStateful) byType = false;
    else if (entry !== undefined && entry.hasCollision) byType = true;
    else byType = this.collision === undefined && isStateIndependent(typeId);
    if (byType) {
      t = this.tokenTypes.length;
      this.tokenTypes.push(typeId);
      this.tokenIds.push(this.resolve(typeId, NO_STATES));
    }
    this.tokens.set(typeId, t);
    return t;
  }

  /** 令牌对应的类型和方块编号（令牌必须是 token 给出的、不小于 0 的值） */
  typeOf(token: number): string {
    return this.tokenTypes[token];
  }

  idOf(token: number): number {
    return this.tokenIds[token];
  }

  /** 返回方块编号。typeId 包含命名空间，仅在解析需要状态时调用 getStates。 */
  resolve(typeId: string, getStates: () => BlockStates, context?: () => { read: NeighborReader; pistonExtended?: boolean }): number {
    const entry = this.registry?.get(typeId);
    if (entry === undefined) return this.resolveUnregistered(typeId, getStates, context);
    // 登记表：碰撞盒和材质取登记的；没登记碰撞盒时形状仍按 collision 函数、内置数据，材质用登记表的
    const m = entry.match(entry.terrainStateful ? getStates() : NO_STATES());
    if (m.boxes !== undefined) {
      const shape = this.shapeFromBoxes(m.boxes);
      return packBlock(shape, this.fixedMaterial ?? (shape === SHAPE_NONE ? MATERIAL_DEFAULT : m.material));
    }
    const id = this.resolveUnregistered(typeId, getStates, context);
    const shape = shapeOf(id);
    return packBlock(shape, this.fixedMaterial ?? (shape === SHAPE_NONE ? MATERIAL_DEFAULT : m.material));
  }

  private resolveUnregistered(typeId: string, getStates: () => BlockStates, context?: () => { read: NeighborReader; pistonExtended?: boolean }): number {
    if (this.collision === undefined) {
      const cached = this.stateless.get(typeId);
      if (cached !== undefined) return cached;
      if (isStateIndependent(typeId)) {
        const id = this.pack(resolveShapeId(typeId, {}), typeId);
        this.stateless.set(typeId, id);
        return id;
      }
      return this.builtin(typeId, getStates(), context);
    }
    const states = getStates();
    const key = `${typeId}|${JSON.stringify(states)}`;
    const cached = this.custom.get(key);
    if (cached !== undefined) return cached === null ? this.builtin(typeId, states, context) : cached;
    const boxes = this.collision({ typeId, states: states as Record<string, string | number | boolean> });
    const id = boxes === undefined
      ? null
      : this.pack(this.shapeFromBoxes(boxes.map((b) => [b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z] as const)), typeId);
    this.custom.set(key, id);
    return id === null ? this.builtin(typeId, states, context) : id;
  }

  private builtin(typeId: string, states: BlockStates, context?: () => { read: NeighborReader; pistonExtended?: boolean }): number {
    if (isStateShaped(typeId)) {
      const boxes = stateBoxes(typeId, states);
      if (boxes !== undefined) return this.pack(this.shapeFromBoxes(boxes), typeId);
    }
    if (context && needsBlockContext(typeId)) {
      const c = context(), boxes = contextualBoxes({ typeId, states, pistonExtended: c.pistonExtended }, c.read);
      if (boxes !== undefined) return this.pack(this.shapeFromBoxes(boxes), typeId);
    }
    return this.pack(resolveShapeId(typeId, states), typeId);
  }

  private pack(shape: number, typeId: string): number {
    if (shape === SHAPE_UNKNOWN) shape = this.unknownBlock === "fullCube" ? SHAPE_FULL_CUBE : SHAPE_NONE;
    const material = this.fixedMaterial ?? (shape === SHAPE_NONE ? MATERIAL_DEFAULT : resolveMaterialId(typeId));
    return packBlock(shape, material);
  }

  /** 用户给出的盒子 → 形状编号。相同的盒子共用一个编号 */
  private shapeFromBoxes(boxes: readonly LocalBox[]): number {
    if (boxes.length === 0) return SHAPE_NONE;
    const key = boxes.map((b) => b.join()).join(";");
    if (key === FULL_CUBE_BOX) return SHAPE_FULL_CUBE;
    const known = this.customShapes.get(key);
    if (known !== undefined) return known;
    const id = this.nextShapeId++;
    this.table.register(id, boxes);
    this.customShapes.set(key, id);
    return id;
  }
}
/**
 * 一格的流体码：水、岩浆按 liquid_depth 换算高度；不是空气、不是完整方块的格子含水时算源方块高度的水。
 * 完整方块和空气不读任何接口
 */
function fluidCode(block: Block, id: number): number {
  const typeId = block.typeId;
  const kind = FLUID_TYPES.get(typeId);
  if (kind !== undefined) {
    const depth = typeId === "minecraft:bubble_column" ? 0 : block.permutation.getState("liquid_depth" as never);
    return kind * 32 + fluidLevel(typeof depth === "number" ? depth : 0);
  }
  if (shapeOf(id) === SHAPE_FULL_CUBE || typeId === "minecraft:air") return FLUID_NONE;
  return block.isWaterlogged ? FLUID_WATER * 32 + fluidLevel(0) : FLUID_NONE;
}

/** liquid_depth → 高度（1/16 格）：8 以上是下落的流体，满格；源方块（0）是 14 */
function fluidLevel(depth: number): number {
  const h = depth >= 8 ? 16 : Math.round((8 - Math.max(0, depth)) / 9 * 16);
  return h < 1 ? 1 : h > 16 ? 16 : h;
}

/** 这一格的流体可能在类型不变时改变（流体本身，或者能含水的非完整方块） */
function mayHoldFluid(id: number, type: number, fluid: number): boolean {
  return fluid !== FLUID_NONE || (type !== TYPE_AIR && shapeOf(id) !== SHAPE_FULL_CUBE);
}

/** 从维度读取方块的 BlockSource（读法见文件开头） */
export class DimensionBlockSource implements BlockSource {
  /** 区块未加载时是否当作空气（对应 unloadedChunkPolicy: "fall"） */
  unloadedAsAir = false;
  private minY = Number.NaN;
  private maxY = Number.NaN;
  private empty: { excludeTypes: string[] } | null = null;
  private readonly location = { x: 0, y: 0, z: 0 };
  private readonly groups = new Map<number, number[]>();
  private readonly direct: number[] = [];
  private readonly one = new Int32Array(1);
  private readonly oneType = new Int32Array(1);
  private readonly oneFluid = new Uint8Array(1);

  constructor(
    private readonly dimension: Dimension,
    private readonly host: BlockSourceHost,
    private readonly resolver: BlockResolver,
  ) {}

  /** 维度的高度范围 [minY, maxY]（两端都包含）。第一次用到时才读，读不到时不限制 */
  private heightRange(): void {
    if (!Number.isNaN(this.minY)) return;
    try {
      const r = this.dimension.heightRange;
      this.minY = r.min;
      this.maxY = r.max - 1;
    } catch {
      this.minY = -Infinity;
      this.maxY = Infinity;
    }
  }

  /**
   * 空块过滤排除空气及受支持的 light_block_0～15。
   * 自定义 collision 可为任意方块提供碰撞，因此启用时仅排除空气。
   */
  private emptyFilter(): { excludeTypes: string[] } {
    if (this.empty !== null) return this.empty;
    const types = ["minecraft:air"];
    const exists = this.host.typeExists;
    if (exists !== undefined && !this.resolver.hasCustomCollision) {
      for (let i = 0; i <= 15; i++) {
        const id = `minecraft:light_block_${i}`;
        // 登记表给光源方块登记了碰撞盒时不能排除
        if (this.resolver.hasRegisteredCollision(id)) continue;
        try { if (exists.call(this.host, id)) types.push(id); } catch { /* 当作没有 */ }
      }
    }
    this.empty = { excludeTypes: types };
    return this.empty;
  }

  readChunk(ox: number, oy: number, oz: number, cells: Int32Array, types: Int32Array, fluids: Uint8Array): boolean {
    this.heightRange();
    cells.fill(SHAPE_NONE);
    types.fill(TYPE_AIR);
    fluids.fill(FLUID_NONE);
    const y0 = Math.max(oy, this.minY), y1 = Math.min(oy + 7, this.maxY);
    if (y0 > y1) return true;
    try {
      const volume = this.host.createVolume(ox, y0, oz, ox + 7, y1, oz + 7);
      if (!this.dimension.containsBlock(volume as never, this.emptyFilter(), this.unloadedAsAir)) return true;
    } catch (e) {
      // 8³ 块在同一个区块里：没加载的话逐格读也读不到。其他错误当作「不空」，逐格读
      if (this.host.classifyError(e) === "unloaded") return this.unloadedAsAir;
    }
    for (let y = y0; y <= y1; y++) {
      for (let z = 0; z < 8; z++) {
        for (let x = 0; x < 8; x++) if (!this.readCell(ox + x, y, oz + z, cells, types, fluids, x + z * 8 + (y - oy) * 64)) return false;
      }
    }
    return true;
  }

  refreshCells(ox: number, oy: number, oz: number, cells: Int32Array, types: Int32Array, fluids: Uint8Array, indices: readonly number[]): boolean {
    this.heightRange();
    const groups = this.groups, direct = this.direct;
    for (const g of groups.values()) g.length = 0;
    direct.length = 0;
    const batch = !this.unloadedAsAir && this.host.createListVolume !== undefined;
    for (let n = 0; n < indices.length; n++) {
      const i = indices[n], y = oy + (i >> 6);
      if (y < this.minY || y > this.maxY) { cells[i] = SHAPE_NONE; types[i] = TYPE_AIR; fluids[i] = FLUID_NONE; continue; }
      // 流体深度、非完整方块的含水状态按类型筛选发现不了，逐格读
      const t = types[i];
      if (!batch || t < 0 || mayHoldFluid(cells[i], t, fluids[i])) { direct.push(i); continue; }
      let g = groups.get(t);
      if (g === undefined) { g = []; groups.set(t, g); }
      g.push(i);
    }
    for (const [t, g] of groups) {
      if (g.length === 0) continue;
      if (g.length === 1) { direct.push(g[0]); continue; }
      try {
        // 只剩类型变了的格子：getBlocks 排除旧类型
        const volume = this.groupVolume(ox, oy, oz, g);
        for (const p of this.dimension.getBlocks(volume as never, { excludeTypes: [this.resolver.typeOf(t)] }, false).getBlockLocationIterator()) {
          direct.push((p.x - ox) + (p.z - oz) * 8 + (p.y - oy) * 64);
        }
      } catch {
        for (let n = 0; n < g.length; n++) direct.push(g[n]);
      }
    }
    for (let n = 0; n < direct.length; n++) {
      const i = direct[n];
      if (!this.readCell(ox + (i & 7), oy + (i >> 6), oz + ((i >> 3) & 7), cells, types, fluids, i)) return false;
    }
    return true;
  }

  /** 一组格子的原生体积：正好填满包围盒时用 BlockVolume，否则用 ListBlockVolume */
  private groupVolume(ox: number, oy: number, oz: number, g: readonly number[]): unknown {
    let x0 = 7, y0 = 7, z0 = 7, x1 = 0, y1 = 0, z1 = 0;
    for (let n = 0; n < g.length; n++) {
      const i = g[n], x = i & 7, y = i >> 6, z = (i >> 3) & 7;
      if (x < x0) x0 = x; if (x > x1) x1 = x;
      if (y < y0) y0 = y; if (y > y1) y1 = y;
      if (z < z0) z0 = z; if (z > z1) z1 = z;
    }
    if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) === g.length) return this.host.createVolume(ox + x0, oy + y0, oz + z0, ox + x1, oy + y1, oz + z1);
    return this.host.createListVolume!(g.map((i) => ({ x: ox + (i & 7), y: oy + (i >> 6), z: oz + ((i >> 3) & 7) })));
  }

  changed(x: number, y: number, z: number, id: number, type: number, fluid: number): boolean {
    this.heightRange();
    if (y < this.minY || y > this.maxY) return id !== SHAPE_NONE || fluid !== FLUID_NONE;
    const p = this.location;
    p.x = x; p.y = y; p.z = z;
    let block: Block | undefined;
    try { block = this.dimension.getBlock(p); } catch { return false; }
    if (block === undefined) return false;
    const t = this.resolver.token(block.typeId);
    // 复用按类型缓存的结果；流体深度和含水状态独立比较。
    let now: number;
    if (t >= 0) {
      now = this.resolver.idOf(t);
      if (t !== type && now !== id) return true;
    } else {
      now = this.resolveBlock(block, x, y, z);
      if (now === SHAPE_UNLOADED) return false;
      if (now !== id) return true;
    }
    return fluidCode(block, now) !== fluid;
  }

  /** 一格的方块编号（调试显示用）。区块没加载时返回 SHAPE_UNLOADED */
  getShapeId(x: number, y: number, z: number): number {
    this.heightRange();
    if (y < this.minY || y > this.maxY) return SHAPE_NONE;
    return this.readCell(x, y, z, this.one, this.oneType, this.oneFluid, 0) ? this.one[0] : SHAPE_UNLOADED;
  }

  /** 一格的流体码（流体计算用）。区块没加载时返回 FLUID_NONE */
  fluidAt(x: number, y: number, z: number): number {
    this.heightRange();
    if (y < this.minY || y > this.maxY) return FLUID_NONE;
    return this.readCell(x, y, z, this.one, this.oneType, this.oneFluid, 0) ? this.oneFluid[0] : FLUID_NONE;
  }

  /** 读一格写进 cells[i]、types[i]、fluids[i]。区块没加载时返回 false（unloadedAsAir 时当空气） */
  private readCell(x: number, y: number, z: number, cells: Int32Array, types: Int32Array, fluids: Uint8Array, i: number): boolean {
    const p = this.location;
    p.x = x; p.y = y; p.z = z;
    let block: Block | undefined;
    try {
      block = this.dimension.getBlock(p);
    } catch (e) {
      const kind = this.host.classifyError(e);
      if (kind === "outOfBounds") { cells[i] = SHAPE_NONE; types[i] = TYPE_AIR; fluids[i] = FLUID_NONE; return true; }
      if (kind !== "unloaded") throw e;
    }
    if (block !== undefined) {
      const t = this.resolver.token(block.typeId);
      const id = t >= 0 ? this.resolver.idOf(t) : this.resolveBlock(block, x, y, z);
      if (id !== SHAPE_UNLOADED) { cells[i] = id; types[i] = t; fluids[i] = fluidCode(block, id); return true; }
    }
    if (!this.unloadedAsAir) return false;
    cells[i] = SHAPE_NONE;
    types[i] = TYPE_AIR;
    fluids[i] = FLUID_NONE;
    return true;
  }

  /** 完整解析一格（依赖状态、邻居、自定义解析的方块）。邻居读不到（没加载、越界）当作不连接。区块没加载时返回 SHAPE_UNLOADED */
  private resolveBlock(block: Block, x: number, y: number, z: number): number {
    try {
      return this.resolver.resolve(block.typeId, () => block.permutation.getAllStates(), () => ({
        pistonExtended: block.typeId === "minecraft:piston" || block.typeId === "minecraft:sticky_piston"
          ? ["Expanded", "Expanding"].includes(block.getComponent("minecraft:piston")?.state ?? "") : undefined,
        read: (dx, dy, dz) => {
          if (y + dy < this.minY || y + dy > this.maxY) return undefined;
          let other: Block | undefined;
          try { other = this.dimension.getBlock({ x: x + dx, y: y + dy, z: z + dz }); }
          catch (error) {
            const kind = this.host.classifyError(error);
            if (kind !== "outOfBounds" && kind !== "unloaded") throw error;
          }
          if (!other) return undefined;
          return { typeId: other.typeId, states: isStateIndependent(other.typeId) ? {} : other.permutation.getAllStates() };
        },
      }));
    } catch (error) {
      const kind = this.host.classifyError(error);
      if (kind === "unloaded") return SHAPE_UNLOADED;
      if (kind === "outOfBounds") return SHAPE_NONE;
      throw error;
    }
  }
}
