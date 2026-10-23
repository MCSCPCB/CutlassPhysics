// 方块类型到碰撞盒和物理属性的映射。
// 登记表为可序列化数据，可随物理存档保存。
// 登记的字段覆盖内置值，没写的字段沿用内置值；内置数据也没有的方块用默认值。
import type { LocalBox } from "../world/block-shapes.js";
import { DEFAULT_MATERIALS, resolveMaterialId, type MaterialSpec } from "./materials.js";
import type { BlockBox, BlockCollision } from "./terrain.js";

/**
 * 方块状态的值，和 `block.permutation.getAllStates()` 的值一致。
 */
export type BlockStateValue = boolean | number | string;

/**
 * 方块或变体的可选属性。
 *
 * @remarks
 * 变体中省略的字段沿用默认项，默认项中省略的字段沿用内置数据。
 */
export interface BlockProperties {
  /**
   * 轴对齐碰撞盒。
   *
   * @remarks
   * 坐标相对于方块最小角，单位为格；x 向东、y 向上、z 向南。空数组表示无碰撞。
   * 每个盒子必须有体积，坐标范围为 [-1, 2]。同时影响地形和 {@link blockShapes}。
   * 省略时使用内置数据；未知方块按 {@link TerrainOptions.unknownBlock} 处理。
   */
  collision?: BlockCollision;
  /**
   * 单个方块的质量。
   *
   * @remarks
   * 仅影响 {@link blockShapes}，与碰撞盒体积无关。必须为非负有限数，0 表示不生成形状。
   * 省略时使用内置值，未知方块默认为 1。
   */
  mass?: number;
  /**
   * 摩擦倍率。
   *
   * @remarks
   * 普通方块为 `1`，冰为 `0`，蜂蜜块等为 `1.65`。接触时的摩擦 = 基础摩擦 × 双方倍率的乘积，
   * 基础摩擦在两个刚体之间为 `0.525`，刚体和地形之间为 `0.5125`。同时作用于地形和 {@link blockShapes} 生成的形状。
   * 必须是不小于 `0` 的有限数。设置了 {@link TerrainOptions.material} 时，地形忽略这一项。
   */
  friction?: number;
  /**
   * 弹性。
   *
   * @remarks
   * 接触时取双方较大的值。同时作用于地形和 {@link blockShapes} 生成的形状。必须是不小于 `0` 的有限数。
   * 设置了 {@link TerrainOptions.material} 时，地形忽略这一项。
   */
  restitution?: number;
  /**
   * 易碎速度，单位为格/秒。
   *
   * @remarks
   * 刚体以不低于这个速度撞上这个方块时触发 {@link PhysicsAfterEvents.blockBreak}，世界里的方块还会被破坏
   * （见 {@link PhysicsWorldOptions.breakFragileBlocks}）。必须是大于 `0` 的有限数。
   * 省略时用内置值，没有内置值的方块不易碎。
   */
  fragileImpactSpeed?: number;
}

/**
 * 登记表里方块的一个变体：方块状态符合条件时，覆盖默认项里的字段。
 */
export interface BlockVariant extends BlockProperties {
  /**
   * 变体的状态匹配条件。
   *
   * @remarks
   * 至少包含一个状态，所有列出的状态均须匹配。数组值表示匹配其中任一值，未列出的状态不受限制。
   * 键和值类型与 block.permutation.getAllStates() 一致，原版状态键兼容 minecraft: 前缀。
   */
  when: Readonly<Record<string, BlockStateValue | readonly BlockStateValue[]>>;
}

/**
 * 登记表里一个方块类型的定义。
 *
 * @remarks
 * 顶层的字段是默认项；`variants` 按顺序检查，第一个条件符合的变体覆盖默认项里对应的字段。
 */
export interface BlockDefinition extends BlockProperties {
  /**
   * 随方块状态变化的部分，例如开关的门板、分层的雪。
   */
  variants?: readonly BlockVariant[];
}

/**
 * 方块类型标识符到定义的映射。
 *
 * @remarks
 * 无命名空间的键默认为 minecraft:。已登记字段覆盖内置数据，未登记的方块使用内置数据。
 * 创建世界时校验并复制登记表；非法项抛出错误。登记表随物理存档保存和恢复。
 *
 * @example block-definitions.ts
 * {@includeCode ../../examples/terrain/block-definitions.ts}
 */
export type BlockDefinitions = Readonly<Record<string, BlockDefinition>>;

/**
 * 方块状态符合条件之后得到的结果（字段已经合并了默认项）
 * @internal
 */
export interface BlockMatch {
  /** 碰撞盒；undefined 表示没有登记，用内置数据 */
  readonly boxes: readonly LocalBox[] | undefined;
  /** 地形材质编号（对应 {@link BlockRegistry.materials}） */
  readonly material: number;
  readonly mass: number | undefined;
  readonly friction: number | undefined;
  readonly restitution: number | undefined;
  readonly fragileImpactSpeed: number | undefined;
}

interface Condition {
  readonly key: string;
  readonly values: readonly BlockStateValue[];
}

interface Variant {
  readonly conditions: readonly Condition[];
  readonly match: BlockMatch;
}

const BOX_MIN = -1, BOX_MAX = 2;

/**
 * 一个方块类型的定义（已校验、合并好的）
 * @internal
 */
export class BlockEntry {
  /** 碰撞或地形材质随状态变化，不能只按类型缓存 */
  readonly terrainStateful: boolean;

  constructor(readonly typeId: string, private readonly base: BlockMatch, private readonly variants: readonly Variant[], terrainStateful: boolean) {
    this.terrainStateful = terrainStateful;
  }

  /** 默认项是否登记了碰撞盒 */
  get hasCollision(): boolean {
    return this.base.boxes !== undefined;
  }

  /** 只按类型就能确定地形形状和材质（可以按类型缓存）：登记了碰撞盒、没有变体改碰撞或材质；或者什么都没改 */
  get terrainByType(): boolean {
    return !this.terrainStateful;
  }

  /** 状态符合的第一个变体，没有符合的返回默认项 */
  match(states: Readonly<Record<string, BlockStateValue>>): BlockMatch {
    const variants = this.variants;
    for (let i = 0; i < variants.length; i++) {
      const conditions = variants[i].conditions;
      let ok = true;
      for (let j = 0; j < conditions.length; j++) {
        if (!matches(readState(states, conditions[j].key), conditions[j].values)) { ok = false; break; }
      }
      if (ok) return variants[i].match;
    }
    return this.base;
  }
}

/** 读取状态值，兼容带有或省略 minecraft: 前缀的键。 */
function readState(states: Readonly<Record<string, BlockStateValue>>, key: string): BlockStateValue | undefined {
  const v = states[key];
  if (v !== undefined) return v;
  if (key.startsWith("minecraft:")) return states[key.slice(10)];
  return key.includes(":") ? undefined : states[`minecraft:${key}`];
}

function matches(value: BlockStateValue | undefined, wanted: readonly BlockStateValue[]): boolean {
  if (value === undefined) return false;
  for (let i = 0; i < wanted.length; i++) if (wanted[i] === value) return true;
  return false;
}

function fail(typeId: string, path: string, message: string): never {
  throw new Error(`方块登记表 ${typeId} 的 ${path} ${message}`);
}

function finiteNumber(v: unknown): v is number {
  return typeof v === "number" && Number.isFinite(v);
}

/** 校验并复制一个盒子，返回内部使用的 [minX, minY, minZ, maxX, maxY, maxZ] */
function checkBox(typeId: string, path: string, box: BlockBox): { local: LocalBox; copy: BlockBox } {
  if (box === null || typeof box !== "object" || box.min === null || typeof box.min !== "object" || box.max === null || typeof box.max !== "object") {
    fail(typeId, path, "必须是 { min, max }");
  }
  const lo = [box.min.x, box.min.y, box.min.z], hi = [box.max.x, box.max.y, box.max.z];
  for (let i = 0; i < 3; i++) {
    const axis = "xyz"[i];
    if (!finiteNumber(lo[i]) || !finiteNumber(hi[i])) fail(typeId, path, `的 ${axis} 必须是有限数`);
    if (lo[i] < BOX_MIN || hi[i] > BOX_MAX) {
      fail(typeId, path, `的 ${axis} 必须在 ${BOX_MIN} 到 ${BOX_MAX} 之间（最多越出方块 1 格），收到 ${lo[i]} ~ ${hi[i]}。坐标的单位是格（1 是一个方块的边长），不是像素`);
    }
    if (!(hi[i] > lo[i])) fail(typeId, path, `在 ${axis} 方向必须有体积（max 大于 min），收到 ${lo[i]} ~ ${hi[i]}`);
  }
  return {
    local: [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]],
    copy: { min: { x: lo[0], y: lo[1], z: lo[2] }, max: { x: hi[0], y: hi[1], z: hi[2] } },
  };
}

/** 校验一组字段，返回复制后的数据和内部格式的盒子 */
function checkProperties(typeId: string, path: string, p: BlockProperties): { copy: BlockProperties; boxes: readonly LocalBox[] | undefined } {
  if (p === null || typeof p !== "object") fail(typeId, path, "必须是对象");
  const copy: { -readonly [K in keyof BlockProperties]: BlockProperties[K] } = {};
  let boxes: LocalBox[] | undefined;
  if (p.collision !== undefined) {
    if (!Array.isArray(p.collision)) fail(typeId, `${path}.collision`, "必须是盒子数组");
    boxes = [];
    copy.collision = [];
    p.collision.forEach((b, i) => {
      const checked = checkBox(typeId, `${path}.collision[${i}]`, b);
      boxes!.push(checked.local);
      copy.collision!.push(checked.copy);
    });
  }
  const numbers = [
    ["mass", 0, true], ["friction", 0, true], ["restitution", 0, true], ["fragileImpactSpeed", 0, false],
  ] as const;
  for (const [key, min, inclusive] of numbers) {
    const v = p[key];
    if (v === undefined) continue;
    if (!finiteNumber(v) || (inclusive ? v < min : v <= min)) fail(typeId, `${path}.${key}`, `必须是${inclusive ? "不小于" : "大于"} ${min} 的有限数，收到 ${String(v)}`);
    copy[key] = v;
  }
  return { copy, boxes };
}

function checkCondition(typeId: string, path: string, when: BlockVariant["when"]): { conditions: Condition[]; copy: Record<string, BlockStateValue | BlockStateValue[]> } {
  if (when === null || typeof when !== "object" || Array.isArray(when)) fail(typeId, path, "必须是对象：状态名 → 要求的值");
  const keys = Object.keys(when);
  if (keys.length === 0) fail(typeId, path, "至少要写一项状态");
  const conditions: Condition[] = [], copy: Record<string, BlockStateValue | BlockStateValue[]> = {};
  for (const key of keys) {
    const raw = when[key];
    const list = Array.isArray(raw) ? raw as readonly unknown[] : [raw];
    if (list.length === 0) fail(typeId, `${path}.${key}`, "不能是空数组");
    for (const v of list) {
      if (typeof v !== "boolean" && typeof v !== "string" && !finiteNumber(v)) fail(typeId, `${path}.${key}`, `的值必须是布尔、有限数或字符串，收到 ${String(v)}`);
    }
    conditions.push({ key, values: list as BlockStateValue[] });
    copy[key] = Array.isArray(raw) ? [...raw as BlockStateValue[]] : raw as BlockStateValue;
  }
  return { conditions, copy };
}

/**
 * 登记表的内部形式：校验过、深拷贝过的数据，加上按状态匹配的接口。
 * @internal
 */
export class BlockRegistry {
  /** 地形材质表：内置的 6 种加上登记表里出现的摩擦、弹性组合，下标是方块编号里的材质编号 */
  readonly materials: readonly MaterialSpec[];
  /** 登记表里最小的易碎速度，没有易碎方块时为 Infinity */
  readonly minFragileSpeed: number;
  private readonly entries = new Map<string, BlockEntry>();
  private readonly json: Record<string, BlockDefinition> = {};

  constructor(definitions: BlockDefinitions) {
    if (definitions === null || typeof definitions !== "object" || Array.isArray(definitions)) throw new Error("方块登记表必须是对象：方块类型 → 定义");
    const materials: MaterialSpec[] = [...DEFAULT_MATERIALS];
    const materialOf = (typeId: string, friction: number | undefined, restitution: number | undefined): number => {
      const builtin = DEFAULT_MATERIALS[resolveMaterialId(typeId)];
      const frictionScale = friction ?? builtin.frictionScale, r = restitution ?? builtin.restitution;
      let id = materials.findIndex((m) => m.frictionScale === frictionScale && m.restitution === r);
      if (id < 0) {
        id = materials.length;
        materials.push({ frictionScale, restitution: r });
      }
      return id;
    };
    let minFragile = Infinity;
    for (const key of Object.keys(definitions)) {
      if (key.length === 0 || key.startsWith(":") || key.endsWith(":")) throw new Error(`方块登记表的方块类型不合法：${JSON.stringify(key)}`);
      const typeId = qualified(key);
      if (typeId === "minecraft:air") throw new Error("方块登记表不能登记空气");
      if (this.entries.has(typeId)) throw new Error(`方块登记表里 ${typeId} 登记了两次`);
      const definition = definitions[key];
      const base = checkProperties(typeId, "默认项", definition);
      const merge = (over: { copy: BlockProperties; boxes: readonly LocalBox[] | undefined }, under?: BlockMatch): BlockMatch => {
        const friction = over.copy.friction ?? under?.friction, restitution = over.copy.restitution ?? under?.restitution;
        return {
          boxes: over.boxes ?? under?.boxes,
          material: materialOf(typeId, friction, restitution),
          mass: over.copy.mass ?? under?.mass,
          friction, restitution,
          fragileImpactSpeed: over.copy.fragileImpactSpeed ?? under?.fragileImpactSpeed,
        };
      };
      const baseMatch = merge(base);
      const json: { -readonly [K in keyof BlockDefinition]: BlockDefinition[K] } = { ...base.copy };
      const variants: Variant[] = [];
      let terrainStateful = false;
      if (definition.variants !== undefined) {
        if (!Array.isArray(definition.variants)) fail(typeId, "variants", "必须是数组");
        const jsonVariants: BlockVariant[] = [];
        definition.variants.forEach((v, i) => {
          const path = `variants[${i}]`;
          const props = checkProperties(typeId, path, v);
          const cond = checkCondition(typeId, `${path}.when`, v.when);
          variants.push({ conditions: cond.conditions, match: merge(props, baseMatch) });
          jsonVariants.push({ ...props.copy, when: cond.copy });
          if (props.copy.collision !== undefined || props.copy.friction !== undefined || props.copy.restitution !== undefined) terrainStateful = true;
        });
        json.variants = jsonVariants;
      }
      this.entries.set(typeId, new BlockEntry(typeId, baseMatch, variants, terrainStateful));
      this.json[typeId] = json;
      for (const m of [baseMatch, ...variants.map((v) => v.match)]) {
        if (m.fragileImpactSpeed !== undefined && m.fragileImpactSpeed < minFragile) minFragile = m.fragileImpactSpeed;
      }
    }
    if (materials.length > 255) throw new Error(`方块登记表里不同的摩擦、弹性组合有 ${materials.length - DEFAULT_MATERIALS.length} 种，超过了上限 ${255 - DEFAULT_MATERIALS.length}`);
    this.materials = materials;
    this.minFragileSpeed = minFragile;
  }

  /** 方块类型的定义（带不带命名空间都行），没有登记返回 undefined */
  get(typeId: string): BlockEntry | undefined {
    return this.entries.get(typeId.includes(":") ? typeId : `minecraft:${typeId}`);
  }

  /** 登记表的副本，键都带命名空间。可以直接存成 JSON */
  toJSON(): BlockDefinitions {
    return JSON.parse(JSON.stringify(this.json)) as BlockDefinitions;
  }
}

/** 按登记表对象缓存校验结果，供 getBlockPhysics 和 blockShapes 复用。登记表在首次使用后须保持不变。 */
const cache = new WeakMap<object, BlockRegistry>();

/**
 * 登记表对象 → 校验过的内部形式（同一个对象只校验一次）。
 * @internal
 */
export function registryFor(definitions: BlockDefinitions): BlockRegistry {
  if (definitions === null || typeof definitions !== "object") return new BlockRegistry(definitions);
  let registry = cache.get(definitions);
  if (registry === undefined) {
    registry = new BlockRegistry(definitions);
    cache.set(definitions, registry);
  }
  return registry;
}

function qualified(typeId: string): string {
  return typeId.includes(":") ? typeId : `minecraft:${typeId}`;
}
