// 方块表面的材质：由方块类型得到材质编号，并给出每种材质的摩擦和弹性。
// 接触规则：
//   - 两个刚体接触时的摩擦 = 0.525 × 两边的摩擦倍率；刚体和地形接触时 = 0.5125 × 两边的摩擦倍率。
//     接触摩擦取两侧几何平均：刚体形状为 0.525 × m²，地形为 0.5125² / 0.525 × m²，m 为摩擦倍率。
//   - 恢复系数取两侧较大值，对应 ContactManager.maxRestitution

/** 材质编号。同一区段内只有材质相同的相邻方块才会合并成一个盒子 */
export const MATERIAL_DEFAULT = 0;
export const MATERIAL_ICE = 1;
export const MATERIAL_SLIME = 2;
/** 蜂蜜块、灵魂沙、灵魂土、仙人掌 */
export const MATERIAL_STICKY = 3;
export const MATERIAL_MUD = 4;
export const MATERIAL_GRINDSTONE = 5;

/**
 * 材质参数。
 *
 * @remarks
 * `frictionScale` 是摩擦倍率：普通方块为 1，冰为 0，蜂蜜块等为 1.65，泥为 0.25，砂轮为 0.05。
 * `restitution` 是该材质的恢复系数，接触时取两侧较大值。
 */
export interface MaterialSpec {
  readonly frictionScale: number;
  readonly restitution: number;
}

export const DEFAULT_MATERIALS: readonly MaterialSpec[] = [
  { frictionScale: 1, restitution: 0 },
  { frictionScale: 0, restitution: 0 },
  { frictionScale: 1, restitution: 0.5 },
  { frictionScale: 1.65, restitution: 0 },
  { frictionScale: 0.25, restitution: 0 },
  { frictionScale: 0.05, restitution: 0 },
];

/** 两个刚体接触、摩擦倍率都是 1 时的摩擦 */
export const BASE_FRICTION = 0.525;

/** 刚体和地形接触、摩擦倍率都是 1 时的摩擦：刚体一侧 0.525、地形一侧 0.5 取平均 */
export const TERRAIN_FRICTION = 0.5125;

/** 刚体形状的摩擦：两个刚体取几何平均后正好是 BASE_FRICTION × 两边的倍率 */
export function materialFriction(m: MaterialSpec): number {
  return BASE_FRICTION * m.frictionScale * m.frictionScale;
}

/** 地形方块的形状摩擦：和摩擦为 materialFriction 的刚体形状取几何平均后正好是 TERRAIN_FRICTION × 两边的倍率 */
export function terrainFriction(m: MaterialSpec): number {
  return (TERRAIN_FRICTION * TERRAIN_FRICTION / BASE_FRICTION) * m.frictionScale * m.frictionScale;
}

const BY_NAME = new Map<string, number>([
  ["ice", MATERIAL_ICE],
  ["packed_ice", MATERIAL_ICE],
  ["blue_ice", MATERIAL_ICE],
  ["frosted_ice", MATERIAL_ICE],
  ["slime", MATERIAL_SLIME],
  ["slime_block", MATERIAL_SLIME],
  ["honey_block", MATERIAL_STICKY],
  ["soul_sand", MATERIAL_STICKY],
  ["soul_soil", MATERIAL_STICKY],
  ["cactus", MATERIAL_STICKY],
  ["mud", MATERIAL_MUD],
  ["grindstone", MATERIAL_GRINDSTONE],
]);

/** 由方块类型得到材质编号，没有特殊材质的方块为 {@link MATERIAL_DEFAULT} */
export function resolveMaterialId(typeId: string): number {
  const name = typeId.startsWith("minecraft:") ? typeId.slice(10) : typeId;
  return BY_NAME.get(name) ?? MATERIAL_DEFAULT;
}
