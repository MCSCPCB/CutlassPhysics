// 把一个 8×8×8 块内的方块转换为盒子：同材质的完整方块贪心合并（先沿 x，再沿 z，再沿 y），其他形状每格各自出盒子，不合并。
// 合并后平地、墙面只产生少量大盒子，盒子内部没有接缝。
import { type BlockShapeTable, MAX_MATERIALS, materialOf, packBlock, SHAPE_FULL_CUBE, SHAPE_NONE, shapeOf } from "./block-shapes.js";

/** 块的边长（格） */
export const CHUNK_SIZE = 8;
/** 块内的格数。格下标为 x + z × 8 + y × 64（x 最快、z 次之、y 最慢） */
export const CHUNK_CELLS = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;

/** 一个盒子：最小角（世界坐标）、三个方向的边长、材质编号 */
export interface ChunkBox {
  minX: number;
  minY: number;
  minZ: number;
  sizeX: number;
  sizeY: number;
  sizeZ: number;
  material: number;
}

/**
 * 合并用的临时数据，只在 chunkBoxes 内同步使用。groups 每格存一个编码 sizeX + sizeZ × 9 + sizeY × 81：
 * 合并时把相邻格的编码累加到后一格上、前一格清 0，最后编码不为 0 的格就是一个盒子的最大角
 */
const voxels = new Uint8Array(CHUNK_CELLS);
const groups = new Uint32Array(CHUNK_CELLS);
const present = new Uint8Array(MAX_MATERIALS);
const STRIDE = CHUNK_SIZE + 1;
const STRIDE_XZ = STRIDE * STRIDE;
const SINGLE = 1 + STRIDE + STRIDE_XZ;

/**
 * 把块内的方块编号转换为盒子，追加到 out，最后按材质、minY、minZ、minX、sizeY、sizeZ、sizeX 排序。
 * (ox, oy, oz) 是块最小角的方块坐标。形状编号没登记或材质编号超出上限时抛错
 */
export function chunkBoxes(cells: Int32Array, table: BlockShapeTable, ox: number, oy: number, oz: number, out: ChunkBox[]): void {
  const first = out.length;
  present.fill(0);
  for (let i = 0; i < CHUNK_CELLS; i++) {
    const id = cells[i], shape = shapeOf(id);
    if (shape === SHAPE_NONE) continue;
    const material = materialOf(id);
    const x = i & 7, z = (i >> 3) & 7, y = i >> 6;
    if (material >= MAX_MATERIALS) throw new Error(`方块 (${ox + x}, ${oy + y}, ${oz + z}) 的材质编号 ${material} 超出上限 ${MAX_MATERIALS - 1}`);
    if (shape === SHAPE_FULL_CUBE) {
      present[material] = 1;
      continue;
    }
    const boxes = table.get(shape);
    if (boxes === undefined) throw new Error(`方块 (${ox + x}, ${oy + y}, ${oz + z}) 的形状编号 ${shape} 没有登记`);
    for (const b of boxes) {
      out.push({ minX: ox + x + b[0], minY: oy + y + b[1], minZ: oz + z + b[2], sizeX: b[3] - b[0], sizeY: b[4] - b[1], sizeZ: b[5] - b[2], material });
    }
  }
  for (let m = 0; m < MAX_MATERIALS; m++) {
    if (present[m] === 0) continue;
    const full = packBlock(SHAPE_FULL_CUBE, m);
    for (let i = 0; i < CHUNK_CELLS; i++) voxels[i] = cells[i] === full ? 1 : 0;
    mergeVoxels();
    for (let i = 0; i < CHUNK_CELLS; i++) {
      const g = groups[i];
      if (g === 0) continue;
      const sizeX = g % STRIDE, sizeZ = Math.floor(g / STRIDE) % STRIDE, sizeY = Math.floor(g / STRIDE_XZ) % STRIDE;
      out.push({
        minX: ox + (i & 7) + 1 - sizeX, minY: oy + (i >> 6) + 1 - sizeY, minZ: oz + ((i >> 3) & 7) + 1 - sizeZ,
        sizeX, sizeY, sizeZ, material: m,
      });
    }
  }
  sortBoxes(out, first);
}

/** voxels → groups：先沿 x 连成一段，再把 sizeX 相同的相邻行沿 z 合并，再把 sizeX、sizeZ 都相同的相邻层沿 y 合并 */
function mergeVoxels(): void {
  for (let i = 0; i < CHUNK_CELLS; i++) groups[i] = voxels[i] === 0 ? 0 : SINGLE;
  for (let row = 0; row < CHUNK_CELLS; row += CHUNK_SIZE) {
    for (let x = 1; x < CHUNK_SIZE; x++) {
      const i = row + x;
      if (voxels[i] === 0 || voxels[i - 1] === 0 || groups[i - 1] === 0) continue;
      groups[i] += groups[i - 1] % STRIDE;
      groups[i - 1] = 0;
    }
  }
  for (let y = 0; y < CHUNK_SIZE; y++) {
    for (let z = 1; z < CHUNK_SIZE; z++) {
      const row = y * 64 + z * CHUNK_SIZE;
      for (let x = 0; x < CHUNK_SIZE; x++) {
        const i = row + x, p = i - CHUNK_SIZE, pg = groups[p], cg = groups[i];
        if (voxels[i] === 0 || voxels[p] === 0 || pg === 0 || cg === 0 || pg % STRIDE !== cg % STRIDE) continue;
        groups[i] += (Math.floor(pg / STRIDE) % STRIDE) * STRIDE;
        groups[p] = 0;
      }
    }
  }
  for (let i = 64; i < CHUNK_CELLS; i++) {
    const p = i - 64, pg = groups[p], cg = groups[i];
    if (voxels[i] === 0 || voxels[p] === 0 || pg === 0 || cg === 0 || pg % STRIDE !== cg % STRIDE ||
      Math.floor(pg / STRIDE) % STRIDE !== Math.floor(cg / STRIDE) % STRIDE) continue;
    groups[i] += (Math.floor(pg / STRIDE_XZ) % STRIDE) * STRIDE_XZ;
    groups[p] = 0;
  }
}

/** 只排 out[first..] 这一段 */
function sortBoxes(out: ChunkBox[], first: number): void {
  const part = out.splice(first);
  part.sort((a, b) => a.material - b.material || a.minY - b.minY || a.minZ - b.minZ || a.minX - b.minX ||
    a.sizeY - b.sizeY || a.sizeZ - b.sizeZ || a.sizeX - b.sizeX);
  for (let i = 0; i < part.length; i++) out.push(part[i]);
}
