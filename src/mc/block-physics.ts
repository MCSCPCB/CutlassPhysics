// 方块的物理属性表：质量、浮力体积、摩擦倍率、弹性、易碎速度、悬浮材质。
// 按规则顺序叠加：先取默认值，再依次套用匹配上的规则（列出的方块或名字模式），规则可以带只在某些状态下生效的变体。
// 碰撞形状不在这里（见 block-shapes.ts）。使用者的方块登记表（block-registry.ts）覆盖这里的结果。
import { registryFor, type BlockDefinitions, type BlockRegistry } from "./block-registry.js";

/** 悬浮材质：带这种材质的方块抵消一部分重力，并受随速度变化的阻力（见 floating.ts） */
export interface FloatingMaterial {
  /** 升力上限为对应方块对刚体的有效重量。 */
  readonly preventSelfLift: boolean;
  /** 阻力按重力大小缩放 */
  readonly scaleFrictionWithGravity: boolean;
  /** 每个方块的升力（以一格质量的重量为单位） */
  readonly liftStrength: number;
  /** 慢速阻力过渡到快速阻力的速度（格/秒） */
  readonly transitionSpeed: number;
  /** 慢速时竖直方向的阻力 */
  readonly slowVerticalFriction: number;
  /** 快速时竖直方向的阻力 */
  readonly fastVerticalFriction: number;
  /** 慢速时水平方向的阻力 */
  readonly slowHorizontalFriction: number;
  /** 快速时水平方向的阻力 */
  readonly fastHorizontalFriction: number;
}

/**
 * 一个方块的物理属性。
 */
export interface BlockPhysics {
  /** 一个方块的质量 */
  readonly mass: number;
  /** 完全浸没时排开的流体体积，单位为格³。 */
  readonly buoyancyVolume: number;
  /** 摩擦倍率：普通方块为 1，冰为 0，蜂蜜块等为 1.65 */
  readonly friction: number;
  /** 弹性 */
  readonly restitution: number;
  /** 触发方块破碎的最小撞击速度，单位为格/秒。省略表示无易碎属性。 */
  readonly fragileImpactSpeed?: number;
  /** 悬浮材质。只有末地石一类会悬浮的方块才有 */
  readonly floatingMaterial?: FloatingMaterial;
  /** 悬浮强度，半砖为一半。只有末地石一类会悬浮的方块才有 */
  readonly floatingScale?: number;
}

/** 方块状态，与 BlockPermutation.getAllStates() 的返回值一致 */
export type BlockPhysicsStates = Readonly<Record<string, boolean | number | string>>;

interface Definition {
  mass?: number;
  buoyancyVolume?: number;
  friction?: number;
  restitution?: number;
  fragileImpactSpeed?: number;
  floatingMaterial?: FloatingMaterial;
  floatingScale?: number;
  massMultiplier?: number;
  buoyancyVolumeMultiplier?: number;
  floatingScaleMultiplier?: number;
}

interface Rule {
  readonly blocks?: readonly string[];
  readonly pattern?: RegExp;
  readonly definition: Definition;
  readonly variants?: readonly { readonly when: (states: BlockPhysicsStates) => boolean; readonly definition: Definition }[];
}

const END_STONE: FloatingMaterial = {
  preventSelfLift: true,
  scaleFrictionWithGravity: true,
  liftStrength: 2,
  transitionSpeed: 2,
  slowVerticalFriction: 0.4,
  fastVerticalFriction: 0.02,
  slowHorizontalFriction: 0.3,
  fastHorizontalFriction: 0.01,
};

const MATERIALS = {
  timber: { mass: 0.5, buoyancyVolume: 1 },
  leaves: { mass: 0.25, buoyancyVolume: 1 },
  light: { mass: 0.5 },
  superLight: { mass: 0.25 },
  stone: { mass: 2 },
  heavyStorage: { mass: 4 },
  bedrock: { mass: 1000 },
  ice: { friction: 0 },
  sticky: { friction: 1.65 },
  mud: { friction: 0.25 },
  grindstone: { friction: 0.05 },
  slime: { restitution: 0.5 },
  endStone: { floatingMaterial: END_STONE, floatingScale: 1 },
} satisfies Record<string, Definition>;

const DEFAULT: BlockPhysics = { mass: 1, buoyancyVolume: 1, friction: 1, restitution: 0 };

/** 读取状态值，兼容带有或省略 minecraft: 前缀的键。 */
function state(states: BlockPhysicsStates, name: string): boolean | number | string | undefined {
  const v = states[name];
  if (v !== undefined) return v;
  const i = name.indexOf(":");
  return i >= 0 ? states[name.slice(i + 1)] : states[`minecraft:${name}`];
}

const WOODS = "oak|spruce|birch|jungle|acacia|dark_oak|mangrove|cherry|pale_oak|crimson|warped|bamboo";
const COLORS = "white|orange|magenta|light_blue|yellow|lime|pink|gray|light_gray|cyan|purple|blue|brown|green|red|black";
const re = (source: string) => new RegExp(`^(?:${source})$`);
const doubled: Definition = { massMultiplier: 2, buoyancyVolumeMultiplier: 2, floatingScaleMultiplier: 2 };

/** 规则按顺序应用，后续规则覆盖先前结果。 */
const RULES: readonly Rule[] = [
  {
    pattern: re(`minecraft:(?:stripped_)?(?:${WOODS})_(?:planks|log|wood|stem|hyphae|block)`),
    blocks: ["minecraft:planks", "minecraft:log", "minecraft:log2", "minecraft:wood", "minecraft:barrel", "minecraft:chest", "minecraft:trapped_chest"],
    definition: MATERIALS.timber,
  },
  {
    blocks: ["minecraft:piston_arm_collision", "minecraft:sticky_piston_arm_collision", "minecraft:piston_head", "minecraft:amethyst_cluster"],
    definition: MATERIALS.light,
  },
  { pattern: re("minecraft:.*_(?:slab|stairs)"), definition: { mass: 0.5, buoyancyVolume: 0.5 } },
  {
    pattern: re(`minecraft:(?:${WOODS}|bamboo_mosaic)_(?:double_)?(?:slab|stairs)`),
    blocks: ["minecraft:wooden_slab", "minecraft:double_wooden_slab"],
    definition: { mass: 0.25 },
  },
  {
    pattern: re("minecraft:(?:stone|normal_stone|cobblestone|mossy_cobblestone|granite|diorite|andesite|cobbled_deepslate|tuff|blackstone)_(?:double_)?slab"),
    definition: { mass: 1 },
  },
  {
    pattern: re(`minecraft:(?:(?:.*_)?(?:fence|fence_gate|trapdoor|door|candle)|(?:${COLORS})_carpet|carpet|glass_pane|.*_stained_glass_pane)`),
    blocks: [
      "minecraft:ladder", "minecraft:iron_bars", "minecraft:cocoa", "minecraft:moss_carpet", "minecraft:bamboo", "minecraft:sea_pickle",
      "minecraft:chain", "minecraft:iron_chain", "minecraft:lightning_rod", "minecraft:flower_pot", "minecraft:end_rod", "minecraft:lantern",
      "minecraft:soul_lantern", "minecraft:unpowered_repeater", "minecraft:powered_repeater", "minecraft:unpowered_comparator",
      "minecraft:powered_comparator", "minecraft:repeater", "minecraft:comparator",
    ],
    definition: { mass: 0.25, buoyancyVolume: 0.25 },
  },
  {
    pattern: re("minecraft:(?:(?:oak|spruce|birch|jungle|acacia|dark_oak|mangrove|cherry|pale_oak)_sapling|(?:.*_)?wool)"),
    blocks: [
      "minecraft:sapling", "minecraft:azalea", "minecraft:flowering_azalea", "minecraft:lily_pad", "minecraft:waterlily",
      "minecraft:big_dripleaf", "minecraft:turtle_egg", "minecraft:snow_layer", "minecraft:mangrove_propagule",
    ],
    definition: MATERIALS.superLight,
  },
  {
    pattern: re("minecraft:(?:.*_)?leaves"),
    blocks: ["minecraft:leaves2", "minecraft:azalea_leaves_flowered"],
    definition: MATERIALS.leaves,
    // persistent 为 true 的树叶无易碎属性。
    variants: [{ when: (s) => !state(s, "persistent_bit") && !state(s, "persistent"), definition: { fragileImpactSpeed: 4 } }],
  },
  {
    blocks: [
      "minecraft:stone", "minecraft:andesite", "minecraft:diorite", "minecraft:granite", "minecraft:tuff", "minecraft:deepslate",
      "minecraft:cobblestone", "minecraft:mossy_cobblestone", "minecraft:cobbled_deepslate", "minecraft:infested_cobblestone",
      "minecraft:basalt", "minecraft:polished_basalt", "minecraft:smooth_basalt", "minecraft:blackstone", "minecraft:obsidian",
      "minecraft:crying_obsidian",
    ],
    definition: MATERIALS.stone,
  },
  {
    blocks: [
      "minecraft:coal_block", "minecraft:diamond_block", "minecraft:dried_kelp_block", "minecraft:emerald_block", "minecraft:gold_block",
      "minecraft:iron_block", "minecraft:lapis_block", "minecraft:netherite_block", "minecraft:raw_copper_block", "minecraft:raw_gold_block",
      "minecraft:raw_iron_block", "minecraft:redstone_block", "minecraft:hay_block", "minecraft:anvil", "minecraft:chipped_anvil",
      "minecraft:damaged_anvil", "minecraft:heavy_core",
    ],
    definition: MATERIALS.heavyStorage,
  },
  { blocks: ["minecraft:bedrock"], definition: MATERIALS.bedrock },
  { blocks: ["minecraft:ice", "minecraft:frosted_ice", "minecraft:packed_ice", "minecraft:blue_ice"], definition: MATERIALS.ice },
  { blocks: ["minecraft:honey_block", "minecraft:soul_sand", "minecraft:soul_soil", "minecraft:cactus"], definition: MATERIALS.sticky },
  { blocks: ["minecraft:mud"], definition: MATERIALS.mud },
  { blocks: ["minecraft:grindstone"], definition: MATERIALS.grindstone },
  { blocks: ["minecraft:slime", "minecraft:slime_block"], definition: MATERIALS.slime },
  {
    blocks: [
      "minecraft:end_stone", "minecraft:end_bricks", "minecraft:end_stone_bricks", "minecraft:end_brick_stairs",
      "minecraft:end_stone_brick_stairs", "minecraft:end_stone_brick_wall", "minecraft:end_stone_brick_slab", "minecraft:end_stone_brick_double_slab",
    ],
    definition: MATERIALS.endStone,
  },
  { pattern: re("minecraft:end_stone_brick_(?:double_)?slab"), definition: { floatingScale: 0.5 } },
  // 双层台阶：Java 版的台阶用 type=double 状态，基岩版有的用单独的 double_slab 方块
  { pattern: re("minecraft:.*_slab"), definition: {}, variants: [{ when: (s) => state(s, "type") === "double", definition: doubled }] },
  { pattern: re("minecraft:.*double_.*slab"), definition: {}, variants: [{ when: (s) => state(s, "type") !== "double", definition: doubled }] },
  {
    pattern: re("minecraft:(?:sticky_)?piston"),
    definition: {},
    variants: [{ when: (s) => state(s, "extended") === true || state(s, "extended_bit") === true, definition: { mass: 0.5 } }],
  },
  {
    blocks: [
      "minecraft:bamboo", "minecraft:cactus", "minecraft:frosted_ice", "minecraft:ice", "minecraft:lily_pad", "minecraft:waterlily",
      "minecraft:melon", "minecraft:melon_block", "minecraft:pumpkin",
    ],
    definition: { fragileImpactSpeed: 4 },
  },
];

const ruleSets = RULES.map((r) => ({ rule: r, blocks: new Set(r.blocks) }));
const rulesByType = new Map<string, readonly Rule[]>();
const staticByType = new Map<string, BlockPhysics>();
const NO_STATES: BlockPhysicsStates = {};

function rulesOf(typeId: string): readonly Rule[] {
  let rules = rulesByType.get(typeId);
  if (rules === undefined) {
    rules = ruleSets.filter((r) => r.blocks.has(typeId) || r.rule.pattern?.test(typeId) === true).map((r) => r.rule);
    rulesByType.set(typeId, rules);
  }
  return rules;
}

function apply(p: { -readonly [K in keyof BlockPhysics]: BlockPhysics[K] }, d: Definition): void {
  if (d.mass !== undefined) p.mass = d.mass;
  if (d.buoyancyVolume !== undefined) p.buoyancyVolume = d.buoyancyVolume;
  if (d.friction !== undefined) p.friction = d.friction;
  if (d.restitution !== undefined) p.restitution = d.restitution;
  if (d.fragileImpactSpeed !== undefined) p.fragileImpactSpeed = d.fragileImpactSpeed;
  if (d.floatingMaterial !== undefined) p.floatingMaterial = d.floatingMaterial;
  if (d.floatingScale !== undefined) p.floatingScale = d.floatingScale;
  if (d.massMultiplier !== undefined) p.mass *= d.massMultiplier;
  if (d.buoyancyVolumeMultiplier !== undefined) p.buoyancyVolume *= d.buoyancyVolumeMultiplier;
  if (d.floatingScaleMultiplier !== undefined && p.floatingMaterial !== undefined) p.floatingScale = (p.floatingScale ?? 1) * d.floatingScaleMultiplier;
}

/**
 * 查询方块的物理属性。
 *
 * @param typeId 方块类型标识符，省略命名空间时默认为 minecraft:。
 * @param states 方块状态，供内置属性和登记表变体匹配使用。
 * @param blocks 方块登记表，已登记字段覆盖内置值。
 * @returns 只读属性对象；状态无关的方块复用同一对象。
 *
 * @example block-physics.ts
 * {@includeCode ../../examples/body/block-physics.ts}
 */
export function getBlockPhysics(typeId: string, states: BlockPhysicsStates = NO_STATES, blocks?: BlockDefinitions): BlockPhysics {
  const builtin = builtinPhysics(typeId, states);
  if (blocks === undefined) return builtin;
  return registeredPhysics(registryFor(blocks), typeId, states, builtin);
}

/**
 * 登记表覆盖内置属性。没有登记的方块返回内置属性本身
 * @internal
 */
export function registeredPhysics(registry: BlockRegistry, typeId: string, states: BlockPhysicsStates, builtin = builtinPhysics(typeId, states)): BlockPhysics {
  const entry = registry.get(typeId);
  if (entry === undefined) return builtin;
  const m = entry.match(states);
  if (m.mass === undefined && m.friction === undefined && m.restitution === undefined && m.fragileImpactSpeed === undefined) return builtin;
  // 同一个变体（或默认项）和同一个内置结果只合并一次：不随状态变化的方块每次返回同一个对象
  let byBuiltin = merged.get(m);
  if (byBuiltin === undefined) {
    byBuiltin = new WeakMap();
    merged.set(m, byBuiltin);
  }
  const known = byBuiltin.get(builtin);
  if (known !== undefined) return known;
  const p: { -readonly [K in keyof BlockPhysics]: BlockPhysics[K] } = { ...builtin };
  if (m.mass !== undefined) p.mass = m.mass;
  if (m.friction !== undefined) p.friction = m.friction;
  if (m.restitution !== undefined) p.restitution = m.restitution;
  if (m.fragileImpactSpeed !== undefined) p.fragileImpactSpeed = m.fragileImpactSpeed;
  byBuiltin.set(builtin, p);
  return p;
}

const merged = new WeakMap<object, WeakMap<BlockPhysics, BlockPhysics>>();

function builtinPhysics(typeId: string, states: BlockPhysicsStates): BlockPhysics {
  const id = typeId.includes(":") ? typeId : `minecraft:${typeId}`;
  const cached = staticByType.get(id);
  if (cached !== undefined) return cached;
  const rules = rulesOf(id);
  let stateful = false;
  for (let i = 0; i < rules.length; i++) if (rules[i].variants !== undefined) stateful = true;
  const p: { -readonly [K in keyof BlockPhysics]: BlockPhysics[K] } = { ...DEFAULT };
  for (let i = 0; i < rules.length; i++) {
    const r = rules[i];
    apply(p, r.definition);
    const v = r.variants?.find((x) => x.when(states));
    if (v !== undefined) apply(p, v.definition);
  }
  if (!stateful) staticByType.set(id, p);
  return p;
}
