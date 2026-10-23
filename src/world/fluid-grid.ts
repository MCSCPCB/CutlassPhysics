// 将刚体形状体素化为流体计算格网。
// 格网在刚体局部坐标系中，原点对齐形状总包围盒的最小角。
// 格子坐标是从 0 开始的整数，格子 (i, j, k) 的局部范围是 origin + [i, i + 1] × [j, j + 1] × [k, k + 1]。
// 每个占用格记录体积、局部包围盒及是否满格。
// 轴对齐盒体按相交体积计算，其他形状使用 4³ 采样；形状变化时重建。
import { Aabb } from "../core/collision/geometry/aabb.js";
import { BoxGeometry } from "../core/collision/geometry/box-geometry.js";
import { CapsuleGeometry } from "../core/collision/geometry/capsule-geometry.js";
import { ConeGeometry } from "../core/collision/geometry/cone-geometry.js";
import { ConvexHullGeometry } from "../core/collision/geometry/convex-hull-geometry.js";
import { CylinderGeometry } from "../core/collision/geometry/cylinder-geometry.js";
import type { Geometry } from "../core/collision/geometry/geometry.js";
import { SphereGeometry } from "../core/collision/geometry/sphere-geometry.js";
import { GjkEpa } from "../core/collision/narrowphase/gjkepa/gjk-epa.js";
import { Transform } from "../core/common/transform.js";
import type { RigidBody } from "../core/dynamics/rigidbody/rigid-body.js";
import type { Shape } from "../core/dynamics/rigidbody/shape.js";

/** 每格每根轴的子采样点数 */
const SUB = 4;
const INSIDE_EPS = 1e-9;
/** 格网的最大格数；超出时不参与流体计算。 */
export const FLUID_GRID_MAX_CELLS = 40000;
/** 格数取整的容差：0.9999999999 格长的盒子算 1 格 */
const SIZE_EPS = 1e-9;

const shapeAabb = new Aabb();
const probe = new SphereGeometry(0);
const probeTf = new Transform();
/** 一格的结果：占用体积和包围盒（相对格子最小角） */
const cell = [0, 0, 0, 0, 0, 0, 0];

/** 一个刚体的流体格网 */
export class FluidGrid {
  /** 每次重建加 1 */
  version = 0;
  /** 格网最小角的局部坐标 */
  originX = 0; originY = 0; originZ = 0;
  /** 每根轴的格数 */
  sx = 0; sy = 0; sz = 0;
  /** 每格的编号：没占用是 0，占用的是格下标 + 1。格下标是 x + z·sx + y·sx·sz */
  cells = new Int32Array(0);
  /** 按编号保存占用体积、满格标志和局部包围盒；每个包围盒含 6 个分量，编号 0 保留。 */
  volume = new Float64Array(1);
  full = new Uint8Array(1);
  bounds = new Float64Array(6);
  /** 占用的格数和占用格的范围（格子坐标，max 不含） */
  count = 0;
  minX = 0; minY = 0; minZ = 0; maxX = 0; maxY = 0; maxZ = 0;
  /** 上一次重建时的形状和局部变换，用来发现形状变了（几何体是只读的，换几何体就是换形状） */
  private readonly shapes: Shape[] = [];
  private readonly poses: number[] = [];

  /** 形状变了就重建。格网超过 FLUID_GRID_MAX_CELLS 格时是空的 */
  update(body: RigidBody): void {
    if (this.version > 0 && this.same(body)) return;
    this.remember(body);
    this.version++;
    this.count = 0;
    this.minX = this.minY = this.minZ = this.maxX = this.maxY = this.maxZ = 0;
    let minX = Infinity, minY = Infinity, minZ = Infinity, maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (let s: Shape | null = body.shapeList; s !== null; s = s.next) {
      s.geom.computeAabb(shapeAabb, s.localTransform);
      if (shapeAabb.minX < minX) minX = shapeAabb.minX;
      if (shapeAabb.minY < minY) minY = shapeAabb.minY;
      if (shapeAabb.minZ < minZ) minZ = shapeAabb.minZ;
      if (shapeAabb.maxX > maxX) maxX = shapeAabb.maxX;
      if (shapeAabb.maxY > maxY) maxY = shapeAabb.maxY;
      if (shapeAabb.maxZ > maxZ) maxZ = shapeAabb.maxZ;
    }
    const sx = cellsAlong(maxX - minX), sy = cellsAlong(maxY - minY), sz = cellsAlong(maxZ - minZ);
    if (body.shapeList === null || !(sx * sy * sz <= FLUID_GRID_MAX_CELLS)) {
      this.sx = this.sy = this.sz = 0;
      this.cells = new Int32Array(0);
      return;
    }
    this.originX = minX; this.originY = minY; this.originZ = minZ;
    this.sx = sx; this.sy = sy; this.sz = sz;
    const n = sx * sy * sz;
    this.cells = new Int32Array(n);
    this.volume = new Float64Array(n + 1);
    this.full = new Uint8Array(n + 1);
    this.bounds = new Float64Array((n + 1) * 6);
    for (let s: Shape | null = body.shapeList; s !== null; s = s.next) this.rasterize(s);
    let lx = Infinity, ly = Infinity, lz = Infinity, hx = -Infinity, hy = -Infinity, hz = -Infinity, count = 0;
    let idx = 0;
    for (let y = 0; y < sy; y++) {
      for (let z = 0; z < sz; z++) {
        for (let x = 0; x < sx; x++, idx++) {
          if (!(this.volume[idx + 1] > 0)) continue;
          this.cells[idx] = idx + 1;
          count++;
          if (x < lx) lx = x;
          if (y < ly) ly = y;
          if (z < lz) lz = z;
          if (x + 1 > hx) hx = x + 1;
          if (y + 1 > hy) hy = y + 1;
          if (z + 1 > hz) hz = z + 1;
        }
      }
    }
    this.count = count;
    if (count > 0) { this.minX = lx; this.minY = ly; this.minZ = lz; this.maxX = hx; this.maxY = hy; this.maxZ = hz; }
  }

  /** 格子 (x, y, z) 的编号，格网外是 0 */
  get(x: number, y: number, z: number): number {
    if (x < 0 || y < 0 || z < 0 || x >= this.sx || y >= this.sy || z >= this.sz) return 0;
    return this.cells[x + this.sx * (z + this.sz * y)];
  }

  /** 格子 (x, y, z) 是否满格 */
  isFull(x: number, y: number, z: number): boolean {
    const id = this.get(x, y, z);
    return id !== 0 && this.full[id] === 1;
  }

  private same(body: RigidBody): boolean {
    const shapes = this.shapes, poses = this.poses;
    let n = 0;
    for (let s: Shape | null = body.shapeList; s !== null; s = s.next, n++) {
      if (n >= shapes.length || shapes[n] !== s) return false;
      const t = s.localTransform, o = n * 12;
      if (poses[o] !== t.px || poses[o + 1] !== t.py || poses[o + 2] !== t.pz ||
        poses[o + 3] !== t.r00 || poses[o + 4] !== t.r01 || poses[o + 5] !== t.r02 ||
        poses[o + 6] !== t.r10 || poses[o + 7] !== t.r11 || poses[o + 8] !== t.r12 ||
        poses[o + 9] !== t.r20 || poses[o + 10] !== t.r21 || poses[o + 11] !== t.r22) return false;
    }
    return n === shapes.length;
  }

  private remember(body: RigidBody): void {
    const shapes = this.shapes, poses = this.poses;
    shapes.length = 0; poses.length = 0;
    for (let s: Shape | null = body.shapeList; s !== null; s = s.next) {
      const t = s.localTransform;
      shapes.push(s);
      poses.push(t.px, t.py, t.pz, t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22);
    }
  }

  /** 将形状写入格网；同一格被多个形状占用时取最大占用体积。 */
  private rasterize(s: Shape): void {
    const t = s.localTransform, g = s.geom;
    const aligned = g instanceof BoxGeometry && t.r01 === 0 && t.r02 === 0 && t.r10 === 0 && t.r12 === 0 && t.r20 === 0 && t.r21 === 0;
    g.computeAabb(shapeAabb, t);
    // 只看形状包围盒碰到的格子
    const i0 = clampCell(Math.floor(shapeAabb.minX - this.originX), this.sx), i1 = clampCell(Math.ceil(shapeAabb.maxX - this.originX) - 1, this.sx);
    const j0 = clampCell(Math.floor(shapeAabb.minY - this.originY), this.sy), j1 = clampCell(Math.ceil(shapeAabb.maxY - this.originY) - 1, this.sy);
    const k0 = clampCell(Math.floor(shapeAabb.minZ - this.originZ), this.sz), k1 = clampCell(Math.ceil(shapeAabb.maxZ - this.originZ) - 1, this.sz);
    const b = this.bounds;
    for (let j = j0; j <= j1; j++) {
      for (let k = k0; k <= k1; k++) {
        for (let i = i0; i <= i1; i++) {
          const id = i + this.sx * (k + this.sz * j) + 1;
          if (this.full[id] === 1) continue;
          const cx = this.originX + i, cy = this.originY + j, cz = this.originZ + k;
          const hit = aligned ? boxCell(g as BoxGeometry, t, cx, cy, cz) : sampleCell(g, t, cx, cy, cz);
          if (!hit || cell[0] <= this.volume[id]) continue;
          const o = id * 6;
          // 体积是 1 时包围盒一定是整格
          if (cell[0] >= 1 - INSIDE_EPS) {
            this.full[id] = 1;
            this.volume[id] = 1;
            b[o] = 0; b[o + 1] = 0; b[o + 2] = 0; b[o + 3] = 1; b[o + 4] = 1; b[o + 5] = 1;
          } else {
            this.volume[id] = cell[0];
            b[o] = cell[1]; b[o + 1] = cell[2]; b[o + 2] = cell[3]; b[o + 3] = cell[4]; b[o + 4] = cell[5]; b[o + 5] = cell[6];
          }
        }
      }
    }
  }
}

/** 长度转换为格数，容差内接近整数的值按整数处理，至少为 1。 */
function cellsAlong(length: number): number {
  const n = Math.ceil(length - SIZE_EPS);
  return n < 1 ? 1 : n;
}

function clampCell(i: number, n: number): number {
  return i < 0 ? 0 : i >= n ? n - 1 : i;
}

/** 轴对齐的盒子和格子 [cx, cx + 1] × [cy, cy + 1] × [cz, cz + 1] 相交，结果写进 cell。不相交返回 false */
function boxCell(g: BoxGeometry, t: Transform, cx: number, cy: number, cz: number): boolean {
  // 局部变换的对角线可能是 -1（转了 180°），盒子本身对称，只看中心和半长
  const x0 = Math.max(t.px - g.hx, cx) - cx, x1 = Math.min(t.px + g.hx, cx + 1) - cx;
  const y0 = Math.max(t.py - g.hy, cy) - cy, y1 = Math.min(t.py + g.hy, cy + 1) - cy;
  const z0 = Math.max(t.pz - g.hz, cz) - cz, z1 = Math.min(t.pz + g.hz, cz + 1) - cz;
  if (!(x1 > x0 && y1 > y0 && z1 > z0)) return false;
  cell[0] = (x1 - x0) * (y1 - y0) * (z1 - z0);
  cell[1] = x0; cell[2] = y0; cell[3] = z0; cell[4] = x1; cell[5] = y1; cell[6] = z1;
  return true;
}

/** 4³ 子采样一格，结果写进 cell：体积按落在形状里的点数估，包围盒是这些点各自代表的小格的并。没有点在里面返回 false */
function sampleCell(g: Geometry, t: Transform, cx: number, cy: number, cz: number): boolean {
  let n = 0;
  let b0 = 1, b1 = 1, b2 = 1, b3 = 0, b4 = 0, b5 = 0;
  const step = 1 / SUB;
  for (let iz = 0; iz < SUB; iz++) {
    for (let iy = 0; iy < SUB; iy++) {
      for (let ix = 0; ix < SUB; ix++) {
        if (!inside(g, t, cx + (ix + 0.5) * step, cy + (iy + 0.5) * step, cz + (iz + 0.5) * step)) continue;
        n++;
        const x0 = ix * step, y0 = iy * step, z0 = iz * step;
        if (x0 < b0) b0 = x0;
        if (y0 < b1) b1 = y0;
        if (z0 < b2) b2 = z0;
        if (x0 + step > b3) b3 = x0 + step;
        if (y0 + step > b4) b4 = y0 + step;
        if (z0 + step > b5) b5 = z0 + step;
      }
    }
  }
  if (n === 0) return false;
  cell[0] = n / (SUB * SUB * SUB);
  cell[1] = b0; cell[2] = b1; cell[3] = b2; cell[4] = b3; cell[5] = b4; cell[6] = b5;
  return true;
}

/** 刚体局部坐标里的点 (x, y, z) 在不在形状里。胶囊、圆柱、圆锥的轴沿形状局部 y 轴 */
function inside(g: Geometry, t: Transform, x: number, y: number, z: number): boolean {
  const dx = x - t.px, dy = y - t.py, dz = z - t.pz;
  const lx = t.r00 * dx + t.r10 * dy + t.r20 * dz;
  const ly = t.r01 * dx + t.r11 * dy + t.r21 * dz;
  const lz = t.r02 * dx + t.r12 * dy + t.r22 * dz;
  if (g instanceof BoxGeometry) {
    return Math.abs(lx) <= g.hx + INSIDE_EPS && Math.abs(ly) <= g.hy + INSIDE_EPS && Math.abs(lz) <= g.hz + INSIDE_EPS;
  }
  if (g instanceof SphereGeometry) {
    const r = g.radius + INSIDE_EPS;
    return lx * lx + ly * ly + lz * lz <= r * r;
  }
  if (g instanceof CapsuleGeometry) {
    const ey = ly < -g.halfHeight ? ly + g.halfHeight : ly > g.halfHeight ? ly - g.halfHeight : 0;
    const r = g.radius + INSIDE_EPS;
    return lx * lx + ey * ey + lz * lz <= r * r;
  }
  if (g instanceof CylinderGeometry) {
    if (Math.abs(ly) > g.halfHeight + INSIDE_EPS) return false;
    const r = g.radius + INSIDE_EPS;
    return lx * lx + lz * lz <= r * r;
  }
  if (g instanceof ConeGeometry) {
    if (Math.abs(ly) > g.halfHeight + INSIDE_EPS) return false;
    // 底面在 y = -halfHeight（半径 radius），尖端在 y = +halfHeight
    const r = (g.radius * (g.halfHeight - ly)) / (2 * g.halfHeight) + INSIDE_EPS;
    return lx * lx + lz * lz <= r * r;
  }
  if (g instanceof ConvexHullGeometry) {
    // 点到凸包核心的距离不超过外边距就在里面
    probeTf.px = x; probeTf.py = y; probeTf.pz = z;
    const gjk = GjkEpa.getInstance();
    gjk.computeDistance(probe, g, probeTf, t, null);
    return gjk.distance <= g.gjkMargin + INSIDE_EPS;
  }
  return false;
}
