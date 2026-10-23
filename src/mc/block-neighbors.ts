// 特殊方块的形状：栅栏、玻璃板、铁栏杆、楼梯按游戏算好的状态构造；门、活塞、紫颂植物还要看邻居或组件。
// 普通方块仍使用生成的数据表。
import type { LocalBox } from "../world/block-shapes.js";
import { isDoor, readState, resolveShapeId, shapeBoxes, SHAPE_UNKNOWN, type BlockStates } from "./block-shapes.js";

export interface NeighborBlock {
  typeId: string;
  states: BlockStates;
  pistonExtended?: boolean;
}
/** 由读取层处理未加载错误；这里的 undefined 只表示世界边界之外。 */
export type NeighborReader = (dx: number, dy: number, dz: number) => NeighborBlock | undefined;
/** 方向编号 0 东（+x）、1 西（-x）、2 南（+z）、3 北（-z），和楼梯的 weirdo_direction 一致。从上往下看顺时针、逆时针转 90 度后的方向编号 */
const clockwise = [2, 3, 1, 0] as const;
const counterClockwise = [3, 2, 0, 1] as const;
const connectionKeys = ["minecraft:connection_east", "minecraft:connection_west", "minecraft:connection_south", "minecraft:connection_north"] as const;
const full: LocalBox = [0, 0, 0, 1, 1, 1];
const nameOf = (b: NeighborBlock) => b.typeId.replace(/^minecraft:/, "");
const fence = (n: string) => n === "fence" || n.endsWith("_fence");
const pane = (n: string) => n.endsWith("_bars") || n === "glass_pane" || n.endsWith("_stained_glass_pane");

/**
 * 按 weirdo_direction、upside_down_bit 和 minecraft:corner 构造楼梯。
 * left 拐角位于朝向逆时针旋转 90 度的一侧，right 位于顺时针一侧。
 */
function stairBoxes(states: BlockStates): readonly LocalBox[] | undefined {
  const up = readState(states, "upside_down_bit"), d = readState(states, "weirdo_direction");
  if (typeof d !== "number" || !Number.isInteger(d) || d < 0 || d > 3) return undefined;
  const shape = readState(states, "minecraft:corner");
  const outer = shape === "outer_left" || shape === "outer_right";
  const corner = shape === "outer_left" || shape === "inner_left" ? counterClockwise[d]
    : shape === "outer_right" || shape === "inner_right" ? clockwise[d] : -1;
  const y0 = up === 1 ? 0 : 0.5, y1 = up === 1 ? 0.5 : 1;
  const boxes: LocalBox[] = [up === 1 ? [0, 0.5, 0, 1, 1, 1] : [0, 0, 0, 1, 0.5, 1]];
  // 四个半格方柱表达楼梯上半层，横向相邻的柱随后合并，避免额外内部面。
  const occupied = [false, false, false, false];
  for (let z = 0; z < 2; z++) for (let x = 0; x < 2; x++) {
    const high = d === 0 ? x === 1 : d === 1 ? x === 0 : d === 2 ? z === 1 : z === 0;
    const side = corner === 0 ? x === 1 : corner === 1 ? x === 0 : corner === 2 ? z === 1 : z === 0;
    occupied[z * 2 + x] = corner < 0 ? high : outer ? high && side : high || side;
  }
  for (let z = 0; z < 2; z++) for (let x = 0; x < 2; x++) {
    if (!occupied[z * 2 + x]) continue;
    const width = x === 0 && occupied[z * 2 + 1] ? 2 : 1;
    const depth = z === 0 && occupied[2 + x] && (width === 1 || occupied[3]) ? 2 : 1;
    boxes.push([x / 2, y0, z / 2, (x + width) / 2, y1, (z + depth) / 2]);
    for (let zz = z; zz < z + depth; zz++) for (let xx = x; xx < x + width; xx++) occupied[zz * 2 + xx] = false;
  }
  return boxes;
}

/**
 * 形状只由自身状态决定的特殊方块（见 isStateShaped）：栅栏、玻璃板、铁栏杆按 minecraft:connection_*（连哪边由游戏算好），
 * 楼梯按 minecraft:corner。不是这几种方块时返回 undefined
 */
export function stateBoxes(typeId: string, states: BlockStates): readonly LocalBox[] | undefined {
  const n = typeId.replace(/^minecraft:/, "");
  if (n.endsWith("_stairs")) return stairBoxes(states);
  if (!fence(n) && !pane(n)) return undefined;
  const isFence = fence(n), lo = isFence ? 0.375 : 0.4375, hi = 1 - lo, height = isFence ? 1.5 : 1;
  const c = connectionKeys.map((k) => readState(states, k) === 1);
  const boxes: LocalBox[] = [];
  // 一字连接合成一个盒子；交叉时第二条只增加未覆盖的部分。
  if (c[0] || c[1]) boxes.push([c[1] ? 0 : lo, 0, lo, c[0] ? 1 : hi, height, hi]);
  else boxes.push([lo, 0, lo, hi, height, hi]);
  if (c[2]) boxes.push([lo, 0, hi, hi, height, 1]);
  if (c[3]) boxes.push([lo, 0, 0, hi, height, lo]);
  return boxes;
}

/** 返回门、活塞、紫颂植物这些要看邻居或组件的形状；undefined 回退到内置排列状态表。 */
export function contextualBoxes(b: NeighborBlock, read: NeighborReader): readonly LocalBox[] | undefined {
  const n = nameOf(b);
  if (isDoor(n)) {
    const upper = readState(b.states, "upper_block_bit") === 1;
    const other = read(0, upper ? -1 : 1, 0);
    if (other?.typeId !== b.typeId) return undefined;
    const lower = upper ? other.states : b.states, top = upper ? b.states : other.states;
    const id = resolveShapeId(b.typeId, { ...lower, door_hinge_bit: readState(top, "door_hinge_bit") ?? readState(lower, "door_hinge_bit") ?? false });
    return id === SHAPE_UNKNOWN ? undefined : shapeBoxes(id);
  }
  if (n === "piston" || n === "sticky_piston" || n.endsWith("piston_arm_collision")) {
    const facing = readState(b.states, "facing_direction");
    if (typeof facing !== "number" || !Number.isInteger(facing) || facing < 0 || facing > 5) return undefined;
    // 活塞的水平朝向和普通方块相反：2 推向 +z（南）、3 推向 -z、4 推向 +x（东）、5 推向 -x；0 向下、1 向上。
    const axis = facing < 2 ? 1 : facing < 4 ? 2 : 0, positive = facing === 1 || facing === 2 || facing === 4;
    if (n.endsWith("piston_arm_collision")) {
      // 头部：推出方向那一侧 1/4 厚的薄板，加一根伸回底座 1/4 格的杆
      const plate = [...full], rod = [0.375, 0.375, 0.375, 0.625, 0.625, 0.625];
      plate[axis + (positive ? 0 : 3)] = positive ? 0.75 : 0.25;
      rod[axis] = positive ? -0.25 : 0.25; rod[axis + 3] = positive ? 0.75 : 1.25;
      return [plate as unknown as LocalBox, rod as unknown as LocalBox];
    }
    if (!b.pistonExtended) return [full];
    const base = [...full];
    base[axis + (positive ? 3 : 0)] = positive ? 0.75 : 0.25;
    return [base as unknown as LocalBox];
  }
  if (n === "chorus_plant") {
    const center: LocalBox = [0.1875, 0.1875, 0.1875, 0.8125, 0.8125, 0.8125];
    const boxes: LocalBox[] = [center];
    for (let axis = 0; axis < 3; axis++) for (const sign of [-1, 1]) {
      const other = read(axis === 0 ? sign : 0, axis === 1 ? sign : 0, axis === 2 ? sign : 0);
      const type = other && nameOf(other);
      if (type === "chorus_plant" || type === "chorus_flower" || (axis === 1 && sign === -1 && type === "end_stone")) {
        const arm = [...center];
        arm[axis] = sign === 1 ? 0.8125 : 0; arm[axis + 3] = sign === 1 ? 1 : 0.1875;
        boxes.push(arm as unknown as LocalBox);
      }
    }
    return boxes;
  }
  return undefined;
}
