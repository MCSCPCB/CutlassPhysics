// 方块的碰撞形状。world/ 层只处理形状编号，方块到形状编号的映射由 mc/ 层负责。

/** 无碰撞的方块，例如空气 */
export const SHAPE_NONE = 0;
/** 完整的 1×1×1 立方体。同一区段内相邻的完整方块会合并为一个盒子 */
export const SHAPE_FULL_CUBE = 1;
/** 该位置所在区块尚未加载，读取可重试。 */
export const SHAPE_UNLOADED = -1;

/** 方块编号中材质所在的位置：编号 = 形状编号 | (材质编号 << MATERIAL_SHIFT)。形状编号占低 16 位 */
export const MATERIAL_SHIFT = 16;
const SHAPE_MASK = (1 << MATERIAL_SHIFT) - 1;
/** 材质编号的上限（不含）。区段内部用一个字节记录材质，0 留给空 */
export const MAX_MATERIALS = 255;

/** 把形状编号和材质编号合成方块编号。材质 0 是默认材质，此时方块编号就是形状编号 */
export function packBlock(shapeId: number, materialId: number): number {
  return shapeId | (materialId << MATERIAL_SHIFT);
}

/** 方块编号中的形状编号 */
export function shapeOf(blockId: number): number {
  return blockId & SHAPE_MASK;
}

/** 方块编号中的材质编号 */
export function materialOf(blockId: number): number {
  return blockId >> MATERIAL_SHIFT;
}

/**
 * 类型令牌：读方块时和方块编号一起记下，复查时用来跳过没变的格子。
 * 0 表示空气；大于 0 表示一种形状和材质不随状态、邻居变化的方块类型（令牌相同，方块编号就相同）；
 * TYPE_RESOLVE 表示每次都要完整解析（依赖状态、邻居或使用者的解析函数）
 */
export const TYPE_AIR = 0;
export const TYPE_RESOLVE = -1;

/**
 * 流体码：0 表示没有流体；其余是种类 × 32 + 高度（1 到 16 的 1/16 格）。
 * 水和岩浆的源方块、流动方块、气泡柱都算；含水的非完整方块（楼梯、栅栏、海带这类）算源方块高度的水
 */
export const FLUID_NONE = 0;
export const FLUID_WATER = 1;
export const FLUID_LAVA = 2;

/** 流体码的种类（0、{@link FLUID_WATER}、{@link FLUID_LAVA}） */
export function fluidKind(code: number): number {
  return code >> 5;
}

/** 流体码的高度（1/16 格），没有流体时是 0 */
export function fluidHeight(code: number): number {
  return (code & 31) / 16;
}

/**
 * 方块读取接口，由 mc 层实现。
 * 8³ 块的下标为 x + z × 8 + y × 64，坐标相对于块最小角。
 */
export interface BlockSource {
  /**
   * 读取最小角为 (ox, oy, oz) 的 8³ 块：cells 写方块编号（SHAPE_NONE、SHAPE_FULL_CUBE 或 {@link packBlock} 合成的编号），
   * types 写类型令牌，fluids 写流体码（{@link FLUID_NONE} 表示没有流体）。
   * 读完返回 true；块里有区块没加载时返回 false，cells、types、fluids 的内容作废
   */
  readChunk(ox: number, oy: number, oz: number, cells: Int32Array, types: Int32Array, fluids: Uint8Array): boolean;
  /**
   * 重新读取 indices 里的格子。cells、types、fluids 里是旧值，读到的新值原地写回（没变的格子可以不写）。
   * 有区块没加载时返回 false，cells、types、fluids 的内容作废
   */
  refreshCells(ox: number, oy: number, oz: number, cells: Int32Array, types: Int32Array, fluids: Uint8Array, indices: readonly number[]): boolean;
  /** 复查方块编号、类型令牌和流体码，任一变化时返回 true。读取失败视为未变化。 */
  changed(x: number, y: number, z: number, id: number, type: number, fluid: number): boolean;
}

/** 盒子的最小角和最大角：[minX, minY, minZ, maxX, maxY, maxZ]，坐标相对于方块的最小角，单位为格 */
export type LocalBox = readonly [number, number, number, number, number, number];

/**
 * 特殊形状表（台阶、栅栏等），将形状编号映射为若干个局部盒子。
 * SHAPE_NONE 和 SHAPE_FULL_CUBE 为内置编号，不能登记；编号必须小于 65536（高位留给材质）。
 */
export class BlockShapeTable {
  private readonly boxes = new Map<number, readonly LocalBox[]>();

  register(id: number, boxes: readonly LocalBox[]): void {
    if (!Number.isInteger(id) || id <= SHAPE_FULL_CUBE || id > SHAPE_MASK) {
      throw new Error(`形状编号必须是大于 ${SHAPE_FULL_CUBE}、不超过 ${SHAPE_MASK} 的整数，收到 ${id}`);
    }
    if (this.boxes.has(id)) throw new Error(`形状编号 ${id} 已经登记过`);
    if (boxes.length === 0) throw new Error(`形状 ${id} 至少要有一个盒子`);
    for (const b of boxes) {
      if (!(b[3] > b[0] && b[4] > b[1] && b[5] > b[2])) throw new Error(`形状 ${id} 的盒子必须有体积：[${b.join(", ")}]`);
    }
    this.boxes.set(id, boxes.map((b) => [...b] as unknown as LocalBox));
  }

  /** 返回已登记的盒子；编号未登记时返回 undefined */
  get(id: number): readonly LocalBox[] | undefined {
    return this.boxes.get(id);
  }
}
