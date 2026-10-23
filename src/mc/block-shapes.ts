// 方块碰撞形状的解析：由方块类型和状态得到形状编号，并建立 world/ 层使用的形状表。
// 本文件不访问 Minecraft API，状态由调用方通过 BlockPermutation.getAllStates() 取得后传入。
import { BlockShapeTable, type LocalBox, SHAPE_NONE } from "../world/block-shapes.js";
import { BLOCKS, PROFILES, SHAPES } from "./data/block-collision.js";

/** 方块状态，与 BlockPermutation.getAllStates() 的返回值一致 */
export type BlockStates = Readonly<Record<string, boolean | number | string>>;

/** 无法识别的方块（自定义方块，或不在数据中的类型） */
export const SHAPE_UNKNOWN = -2;

const WEST: LocalBox = [0, 0, 0, 0.1875, 1, 1];
const EAST: LocalBox = [0.8125, 0, 0, 1, 1, 1];
const NORTH: LocalBox = [0, 0, 0, 1, 1, 0.1875];
const SOUTH: LocalBox = [0, 0, 0.8125, 1, 1, 1];

/** 补充形状，编号接在 SHAPES 之后。门的形状由 {@link resolveDoorShape} 按状态解析。 */
export const DOOR_SHAPE_WEST = SHAPES.length;
export const DOOR_SHAPE_EAST = SHAPES.length + 1;
export const DOOR_SHAPE_NORTH = SHAPES.length + 2;
export const DOOR_SHAPE_SOUTH = SHAPES.length + 3;
/** 形状编号的上界（不含） */
export const SHAPE_COUNT = SHAPES.length + 4;

/** 返回形状编号对应的盒子，每个盒子为 [minX, minY, minZ, maxX, maxY, maxZ] */
export function shapeBoxes(id: number): readonly LocalBox[] {
  if (id === DOOR_SHAPE_WEST) return [WEST];
  if (id === DOOR_SHAPE_EAST) return [EAST];
  if (id === DOOR_SHAPE_NORTH) return [NORTH];
  if (id === DOOR_SHAPE_SOUTH) return [SOUTH];
  const flat = SHAPES[id];
  if (flat === undefined) throw new Error(`形状编号 ${id} 不存在`);
  const out: LocalBox[] = [];
  for (let i = 0; i < flat.length; i += 6) {
    out.push([flat[i], flat[i + 1], flat[i + 2], flat[i + 3], flat[i + 4], flat[i + 5]]);
  }
  return out;
}

/** 建立包含全部形状的形状表。编号 0（无碰撞）和 1（完整方块）由 world/ 内置，不登记 */
export function createBlockShapeTable(): BlockShapeTable {
  const table = new BlockShapeTable();
  for (let id = 2; id < SHAPE_COUNT; id++) table.register(id, shapeBoxes(id));
  return table;
}

/** 状态值统一为数字或字符串：布尔值（字节类状态）转为 0 / 1 */
function normalize(v: boolean | number | string | undefined): number | string | undefined {
  return typeof v === "boolean" ? (v ? 1 : 0) : v;
}

/** 读取状态值，兼容带有或省略 minecraft: 前缀的键。 */
export function readState(states: BlockStates, key: string): number | string | undefined {
  let v = states[key];
  if (v === undefined) v = key.startsWith("minecraft:") ? states[key.slice(10)] : states[`minecraft:${key}`];
  return normalize(v);
}

/** 门的方块名：wooden_door、iron_door，以及各种木材、铜的 xxx_door（活板门的名字是 xxx_trapdoor，不会匹配） */
export function isDoor(name: string): boolean {
  return name === "wooden_door" || name === "iron_door" || name.endsWith("_door");
}

/**
 * 门的形状。
 *
 * @remarks
 * 由朝向（`minecraft:cardinal_direction`）、开关（`open_bit`）、铰链（`door_hinge_bit`）决定，与上下半无关。
 * 关闭时薄板的位置：south 为西侧，west 为北侧，north 为东侧，east 为南侧。
 * 打开时铰链在左（`door_hinge_bit` = 0）：south 为北侧，west 为东侧，north 为南侧，east 为西侧；
 * 铰链在右（`door_hinge_bit` = 1）：south 为南侧，west 为西侧，north 为北侧，east 为东侧。
 */
export function resolveDoorShape(states: BlockStates): number {
  const dir = readState(states, "minecraft:cardinal_direction");
  const open = readState(states, "open_bit");
  const hinge = readState(states, "door_hinge_bit");
  if (dir === undefined || open === undefined || hinge === undefined) return SHAPE_UNKNOWN;
  // 顺序：south, west, north, east
  const closed = [DOOR_SHAPE_WEST, DOOR_SHAPE_NORTH, DOOR_SHAPE_EAST, DOOR_SHAPE_SOUTH];
  const openLeft = [DOOR_SHAPE_NORTH, DOOR_SHAPE_EAST, DOOR_SHAPE_SOUTH, DOOR_SHAPE_WEST];
  const openRight = [DOOR_SHAPE_SOUTH, DOOR_SHAPE_WEST, DOOR_SHAPE_NORTH, DOOR_SHAPE_EAST];
  const i = dir === "south" ? 0 : dir === "west" ? 1 : dir === "north" ? 2 : dir === "east" ? 3 : -1;
  if (i < 0) return SHAPE_UNKNOWN;
  if (open === 0) return closed[i];
  return hinge === 0 ? openLeft[i] : openRight[i];
}

const hasOwn = Object.prototype.hasOwnProperty;

/** 方块的形状不随状态变化（可以只按类型缓存）。门例外：它的形状由状态决定 */
export function isStateIndependent(typeId: string): boolean {
  const name = typeId.startsWith("minecraft:") ? typeId.slice(10) : typeId;
  // 仅接受数值形状编号，排除继承属性。
  return typeof BLOCKS[name] === "number" && !needsBlockContext(typeId) && !isStateShaped(typeId);
}

/**
 * 形状由游戏算好的连接、拐角状态决定的原版方块：栅栏、玻璃板、铁栏杆读 `minecraft:connection_*`，楼梯读 `minecraft:corner`。
 * 生成的数据表里没有这些状态，形状由 block-neighbors.ts 的 stateBoxes 构造
 */
export function isStateShaped(typeId: string): boolean {
  if (typeId.includes(":") && !typeId.startsWith("minecraft:")) return false;
  const n = typeId.replace(/^minecraft:/, "");
  return n === "fence" || n.endsWith("_fence") || n.endsWith("_stairs") || n === "glass_pane" ||
    n.endsWith("_stained_glass_pane") || n.endsWith("_bars");
}

/** 这些原版方块还依赖邻居或组件；不能只用类型和排列状态缓存。 */
export function needsBlockContext(typeId: string): boolean {
  if (typeId.includes(":") && !typeId.startsWith("minecraft:")) return false;
  const n = typeId.replace(/^minecraft:/, "");
  return n === "chorus_plant" || isDoor(n) ||
    n === "piston" || n === "sticky_piston" || n === "piston_arm_collision" || n === "sticky_piston_arm_collision";
}

/**
 * 由方块类型和状态得到形状编号。
 *
 * @param typeId 方块类型，带不带 `minecraft:` 前缀均可
 * @param states 方块状态
 * @returns 形状编号（0 为无碰撞，1 为完整方块，其余见 {@link shapeBoxes}）；无法识别时返回 {@link SHAPE_UNKNOWN}
 */
export function resolveShapeId(typeId: string, states: BlockStates): number {
  const name = typeId.startsWith("minecraft:") ? typeId.slice(10) : typeId;
  // 仅匹配自身属性，排除 constructor、__proto__ 等继承名称。
  if (!hasOwn.call(BLOCKS, name)) return SHAPE_UNKNOWN;
  const entry = BLOCKS[name];
  if (typeof entry === "number") {
    return entry === SHAPE_NONE || !isDoor(name) ? entry : resolveDoorShape(states);
  }
  const profile = PROFILES[entry[0]];
  let at = 0;
  for (let j = 0; j < profile.keys.length; j++) {
    const v = readState(states, profile.keys[j]);
    const index = v === undefined ? -1 : profile.values[j].indexOf(v);
    if (index < 0) return SHAPE_UNKNOWN;
    at = at * profile.values[j].length + index;
  }
  return profile.shapes[at];
}
