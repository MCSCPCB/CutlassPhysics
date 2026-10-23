// 流体对刚体的力：浮力、阻力、液面阻尼、水流、气腔。
// apply 在步进前累加流体力，仅处理活动且地形就绪的动态刚体。
// 格网（fluid-grid.ts）在刚体局部坐标里，格子坐标从 0 开始；这里的格子坐标都是格网自己的。
// 力臂相对刚体原点（transform.p）算：格子中心的世界坐标 = 刚体位置 + 旋转 ×（格子坐标 + 0.5 + 格网原点）。
import { FLUID_LAVA, fluidHeight, fluidKind } from "./block-shapes.js";
import { FLUID_GRID_MAX_CELLS, type FluidGrid } from "./fluid-grid.js";

/** 气腔重建的条件：转动超过这么多（四元数点积）且距上次重建满 POCKET_MIN_AGE 个 tick */
const POCKET_ROTATION = 0.99985;
const POCKET_MIN_AGE = 10;
/** 溢流、排水的经验系数 */
const FLOOD_FACTOR = 0.585;

/** 流体计算的参数。不给的项用默认值 */
export interface FluidOptions {
  /** 水的密度（质量 / 格³）。默认 1 */
  waterDensity?: number;
  /** 岩浆的密度。默认 3 */
  lavaDensity?: number;
  /** 二次阻力系数。默认 0.5 */
  drag?: number;
  /** 一次阻力系数。默认 0.5 */
  linearDrag?: number;
  /** 岩浆的阻力倍率。默认 6 */
  lavaViscosity?: number;
  /** 液面阻尼。默认 0.5 */
  waveDamping?: number;
  /** 是否计算气腔（拼成空心船体的刚体）。默认 true */
  airPockets?: boolean;
  /** 气腔进水、排水的速度。默认 1 */
  floodRate?: number;
  /** 水流速度（格 / 秒）。默认 1.4 */
  currentSpeed?: number;
}

/** 流体查询：只查已缓存的数据，读不到时 fluidAt 返回 0、isTerrainEmpty 返回 false */
export interface FluidTerrain {
  fluidAt(x: number, y: number, z: number): number;
  isTerrainEmpty(x: number, y: number, z: number): boolean;
  hasFluidNear(minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number): boolean;
}

/** 刚体的气腔，包括阻止液体进入的碗形空腔和封闭空气的钟形空腔。 */
interface Pockets {
  version: number;
  qx: number; qy: number; qz: number; qw: number;
  age: number;
  x0: number; y0: number; z0: number;
  nx: number; ny: number; nz: number;
  /** 外扩格网里每个格子的气腔格编号，没有是 -1。下标 x + nx·(z + nz·y) */
  index: Int32Array;
  count: number;
  /** 每个气腔格的格子坐标（3 个一组）、浸没深度、当前的水 */
  cells: Int32Array;
  dips: Float64Array;
  fill: Float64Array;
  rooms: number;
  /** 每个房间的气腔格范围 [roomStart[r], roomStart[r + 1])、碗沿、边缘长度、存水量 */
  roomStart: Int32Array;
  roomRim: Float64Array;
  roomEdge: Float64Array;
  water: Float64Array;
}

/** 刚体进入流体时的接触信息 */
export interface FluidEntry {
  fastestVelocityY: number;
  minX: number; maxX: number; minZ: number; maxZ: number;
  pointX: number; pointY: number; pointZ: number;
}

/** 一个刚体在流体计算里要用的状态。位姿和速度是 tick 开头的值，力加到 fx..tz 上 */
export interface FluidBody {
  grid: FluidGrid;
  px: number; py: number; pz: number;
  r00: number; r01: number; r02: number;
  r10: number; r11: number; r12: number;
  r20: number; r21: number; r22: number;
  /** 朝向四元数，气腔重建时用来判断转了多少 */
  qx: number; qy: number; qz: number; qw: number;
  vx: number; vy: number; vz: number;
  wx: number; wy: number; wz: number;
  mass: number;
  invMass: number;
  /** 世界坐标系的逆惯量张量对角线 */
  ii00: number; ii11: number; ii22: number;
  gravityScale: number;
  /** 世界坐标包围盒 */
  minX: number; minY: number; minZ: number; maxX: number; maxY: number; maxZ: number;
  fx: number; fy: number; fz: number;
  tx: number; ty: number; tz: number;
  /** 浸没的体积（格³），apply 里更新。外部用来判断刚体在不在水里 */
  submerged: number;
}

/** 流体对刚体的力 */
export class FluidForces {
  waterDensity = 1;
  lavaDensity = 3;
  drag = 0.5;
  linearDrag = 0.5;
  lavaViscosity = 6;
  waveDamping = 0.5;
  airPockets = true;
  floodRate = 1;
  currentSpeed = 1.4;
  /** 上一次 apply 的结果：浸没体积、气腔体积、腔里的水、浮力的不平衡程度 */
  lastSubmerged = 0;
  lastPocketVolume = 0;
  lastInside = 0;
  lastUnbalance = 0;
  /** 上次 apply 检测到的流体种类，第 k 位表示种类 k + 1。 */
  lastKinds = 0;
  private readonly pockets = new WeakMap<FluidBody, Pockets>();
  /** 上一次 submerged、surfaceAt 查到的流体种类、液面高度、浸没部分的中心 */
  kind = 0;
  middle = 0;
  surface = 0;
  /** 上一次 flowAt 的水流速度 */
  flowX = 0;
  flowZ = 0;
  private readonly surfaces = new Map<number, number>();
  private readonly surfaceKinds = new Map<number, number>();
  private readonly flows = new Map<number, number>();
  private offX = new Float64Array(0);
  private offY = new Float64Array(0);
  private offZ = new Float64Array(0);
  private bottoms = new Float64Array(0);
  private caps = new Float64Array(0);
  private rooms = new Float64Array(0);
  private outside = new Float64Array(0);
  private kinds = new Uint8Array(0);

  constructor(options: FluidOptions = {}) {
    if (options.waterDensity !== undefined) this.waterDensity = Math.max(0, options.waterDensity);
    if (options.lavaDensity !== undefined) this.lavaDensity = Math.max(0, options.lavaDensity);
    if (options.drag !== undefined) this.drag = Math.max(0, options.drag);
    if (options.linearDrag !== undefined) this.linearDrag = Math.max(0, options.linearDrag);
    if (options.lavaViscosity !== undefined) this.lavaViscosity = Math.max(0, options.lavaViscosity);
    if (options.waveDamping !== undefined) this.waveDamping = Math.max(0, options.waveDamping);
    if (options.airPockets !== undefined) this.airPockets = options.airPockets;
    if (options.floodRate !== undefined) this.floodRate = Math.max(0, options.floodRate);
    if (options.currentSpeed !== undefined) this.currentSpeed = Math.max(0, options.currentSpeed);
  }

  /**
   * 采集刚体在指定流体中的接触信息，供入水事件使用。
   * 依赖同一 tick 的 apply 缓存；无对应接触时返回 false。
   */
  collectEntry(body: FluidBody, terrain: FluidTerrain, kind: number, out: FluidEntry): boolean {
    const grid = body.grid, cells = grid.cells, bounds = grid.bounds, full = grid.full;
    const sx = grid.sx, sz = grid.sz, layer = sx * sz;
    const W = 0.5 * (Math.abs(body.r10) + Math.abs(body.r11) + Math.abs(body.r12));
    const ax = Math.abs(body.r10), ay = Math.abs(body.r11), az = Math.abs(body.r12);
    let count = 0, vy = Infinity, sumX = 0, sumY = 0, sumZ = 0;
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (let i = 0; i < cells.length; i++) {
      const id = cells[i];
      if (id === 0) continue;
      const o = id * 6;
      const lx = (i % sx) + grid.originX + (bounds[o] + bounds[o + 3]) * 0.5;
      const ly = Math.floor(i / layer) + grid.originY + (bounds[o + 1] + bounds[o + 4]) * 0.5;
      const lz = (Math.floor(i / sx) % sz) + grid.originZ + (bounds[o + 2] + bounds[o + 5]) * 0.5;
      const rx = body.r00 * lx + body.r01 * ly + body.r02 * lz;
      const ry = body.r10 * lx + body.r11 * ly + body.r12 * lz;
      const rz = body.r20 * lx + body.r21 * ly + body.r22 * lz;
      const half = full[id] === 1 ? W : 0.5 * (ax * (bounds[o + 3] - bounds[o]) + ay * (bounds[o + 4] - bounds[o + 1]) + az * (bounds[o + 5] - bounds[o + 2]));
      if (this.submerged(terrain, body.px + rx, body.py + ry, body.pz + rz, half > 0.01 ? half : 0.01) <= 0 || this.kind !== kind) continue;
      const v = body.vy + body.wz * rx - body.wx * rz;
      if (v < vy) vy = v;
      const x = body.px + rx, z = body.pz + rz;
      sumX += x; sumZ += z; sumY += this.surface === Infinity ? body.py + ry + half : this.surface;
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
      count++;
    }
    if (count === 0) return false;
    out.fastestVelocityY = vy;
    out.minX = minX; out.maxX = maxX; out.minZ = minZ; out.maxZ = maxZ;
    out.pointX = sumX / count; out.pointY = sumY / count + 0.02; out.pointZ = sumZ / count;
    return true;
  }
  /** 上次 apply 建立的格网是否包含气腔；气腔的液面查询向上扩展 64 格。 */
  hasPockets(body: FluidBody): boolean {
    const f = this.pockets.get(body);
    return f !== undefined && f.count > 0;
  }

  /** 气腔里的水（格³） */
  fluidInside(body: FluidBody): number {
    const f = this.pockets.get(body);
    if (f === undefined) return 0;
    let x = 0;
    for (let l = 0; l < f.rooms; l++) x += f.water[l];
    return x;
  }

  /** 一段竖直线 [x − half, x + half] 浸在 (f, l) 这一格柱的流体里的比例，浸没部分的中心记在 middle */
  submerged(terrain: FluidTerrain, f: number, x: number, l: number, half: number): number {
    const h = Math.floor(f), M = Math.floor(l), n = x - half, c = x + half;
    let a = 0, e = 0;
    this.kind = 0;
    for (let y = Math.floor(n); y < c; y++) {
      const i = terrain.fluidAt(h, y, M);
      if (i === 0) continue;
      const r = fluidKind(i), w = y + columnHeight(terrain, h, y, M, i), d = n > y ? n : y, g = c < w ? c : w;
      if (g <= d) continue;
      a += g - d;
      e += (g - d) * (d + g) * 0.5;
      if (r > this.kind) this.kind = r;
    }
    this.middle = a > 0 ? e / a : x;
    const F = a / (2 * half);
    this.surface = F >= 1 - 1e-9 ? 1 / 0 : n + a;
    return F > 1 ? 1 : F;
  }

  /** (f, l) 这一格柱的液面高度（世界坐标）。同一 tick 里查过的记下来 */
  surfaceAt(terrain: FluidTerrain, f: number, x: number, l: number): number {
    const o = Math.floor(f), h = Math.floor(x), M = Math.floor(l), n = columnKey(o, h, M), c = this.surfaces.get(n);
    if (c !== undefined) {
      this.kind = this.surfaceKinds.get(n) ?? 0;
      return c;
    }
    let a = h, e = terrain.fluidAt(o, a, M);
    if (e === 0) { a--; e = terrain.fluidAt(o, a, M); }
    let F = -1 / 0;
    const y = e === 0 ? 0 : fluidKind(e);
    for (let i = 0; i < 64 && e !== 0; i++) {
      const r = terrain.fluidAt(o, a + 1, M);
      F = a + fluidHeight(e);
      if (r === 0 || fluidKind(r) !== y) break;
      a++;
      e = r;
      F = a + 1;
    }
    this.surfaces.set(n, F);
    this.surfaceKinds.set(n, y);
    this.kind = y;
    return F;
  }

  /** (f, x, l) 这一格的水流速度，结果在 flowX、flowZ。同一 tick 里查过的记下来 */
  flowAt(terrain: FluidTerrain, f: number, x: number, l: number): void {
    this.flowX = 0;
    this.flowZ = 0;
    const o = this.currentSpeed;
    if (o <= 0) return;
    const h = columnKey(f, x, l), M = this.flows.get(h);
    if (M !== undefined) {
      this.flowX = (Math.floor(M / 16384) - 4096) / 4096 * o;
      this.flowZ = (M % 16384 - 4096) / 4096 * o;
      return;
    }
    const n = terrain.fluidAt(f, x, l);
    let c = 0, a = 0;
    if (n !== 0) {
      const F = fluidKind(n), y = columnHeight(terrain, f, x, l, n);
      for (let i = 0; i < 4; i++) {
        const r = i === 0 ? 1 : i === 1 ? -1 : 0, w = i === 2 ? 1 : i === 3 ? -1 : 0, d = terrain.fluidAt(f + r, x, l + w);
        let g = 0;
        if (d !== 0 && fluidKind(d) === F) g = y - columnHeight(terrain, f + r, x, l + w, d);
        else if (d === 0 && terrain.isTerrainEmpty(f + r, x, l + w)) {
          const Z = terrain.fluidAt(f + r, x - 1, l + w);
          if (Z !== 0 && fluidKind(Z) === F) g = y - (columnHeight(terrain, f + r, x - 1, l + w, Z) - 1);
        }
        c += r * g;
        a += w * g;
      }
    }
    const e = Math.sqrt(c * c + a * a);
    if (e > 1e-6) { c /= e; a /= e; } else c = a = 0;
    this.flows.set(h, (Math.round(c * 4096) + 4096) * 16384 + Math.round(a * 4096) + 4096);
    this.flowX = c * o;
    this.flowZ = a * o;
  }

  /**
   * 算一个刚体这个 tick 受的流体力，加到 fx..tz 上。附近没有流体、气腔里也没有水时返回 false。
   * dt 是一个 tick 的时长；gx、gy、gz 是世界的重力。
   */
  apply(body: FluidBody, terrain: FluidTerrain, gx: number, gy: number, gz: number, dt: number): boolean {
    this.lastSubmerged = 0;
    this.lastPocketVolume = 0;
    this.lastInside = 0;
    this.lastUnbalance = 0;
    this.lastKinds = 0;
    body.submerged = 0;
    const c = terrain.hasFluidNear(Math.floor(body.minX), Math.floor(body.minY) - 1, Math.floor(body.minZ), Math.floor(body.maxX), Math.floor(body.maxY), Math.floor(body.maxZ));
    if (!c && this.fluidInside(body) <= 0) return false;
    this.surfaces.clear();
    this.surfaceKinds.clear();
    this.flows.clear();
    const grid = body.grid, F = body.px, y = body.py, i = body.pz, r = -grid.originX, w = -grid.originY, d = -grid.originZ;
    const g = body.vx, Z = body.vy, Y = body.vz, T = body.wx, rt = body.wy, lt = body.wz;
    const at = Math.abs(body.r10), k = Math.abs(body.r11), _ = Math.abs(body.r12), W = 0.5 * (at + k + _), P = 2 * W;
    const j = -gx * body.gravityScale, nt = -gy * body.gravityScale, B = -gz * body.gravityScale, J = Math.sqrt(Math.sqrt(j * j + nt * nt + B * B));
    let ot = 0, et = 0, K = 0, Q = 0, dtq = 0, mt = 0, $ = 0, O = 0, St = 0, kt = 0, Xt = 0, At = 0, zt = 0, vt = 0, Zt = 0, Ft = 0, Yt = 0;
    const Ct = this.waterDensity, Lt = this.lavaDensity, Nt = this.waveDamping * J, yt = grid.volume, ns = grid.bounds;
    // 浮力：体积 v 的流体，作用点在刚体局部 (R, X)（Y 是世界高度）
    const buoyancy = (v: number, R: number, G: number, X: number): void => {
      const C = j * v, H = nt * v, it = B * v;
      ot += C; et += H; K += it;
      Q += G * it - X * H; dtq += X * C - R * it; mt += R * H - G * C;
    };
    // 液面阻尼：竖向速度在液面上的那一层被压住
    const wave = (v: number, R: number, X: number): void => {
      const C = Z + lt * R - T * X, H = -v * C;
      O += H; kt -= X * H; At += R * H; zt += v; vt += v * (R * R + X * X);
    };
    const U = this.airPockets && c ? this.pocketsOf(body, W) : this.pockets.get(body);
    if (U !== undefined) {
      const v = U.count;
      if (this.offX.length < v) {
        const s = Math.max(64, v * 2);
        this.offX = new Float64Array(s); this.offY = new Float64Array(s); this.offZ = new Float64Array(s);
        this.bottoms = new Float64Array(s); this.caps = new Float64Array(s); this.rooms = new Float64Array(s);
        this.outside = new Float64Array(s); this.kinds = new Uint8Array(s);
      }
      const R = U.cells, G = U.dips, X = U.fill, C = this.offX, H = this.offY, it = this.offZ;
      const D = this.bottoms, ft = this.caps, b = this.rooms, ht = this.outside, L = this.kinds;
      for (let s = 0; s < v; s++) {
        const S = R[s * 3], V = R[s * 3 + 1], A = R[s * 3 + 2];
        const m = S + 0.5 - r, u = V + 0.5 - w, q = A + 0.5 - d;
        const tt = body.r00 * m + body.r01 * u + body.r02 * q;
        const z = body.r10 * m + body.r11 * u + body.r12 * q;
        const p = body.r20 * m + body.r21 * u + body.r22 * q;
        C[s] = tt; H[s] = z; it[s] = p;
        const ct = y + z - W;
        D[s] = ct;
        const st = (G[s] + y - ct) / P;
        ft[s] = st <= 0 ? 0 : st >= 1 ? 1 : st;
        const gt = grid.get(S, V, A);
        b[s] = gt === 0 ? 1 : 1 - yt[gt];
        ht[s] = c ? this.surfaceAt(terrain, F + tt, y + z, i + p) : -1 / 0;
        L[s] = this.kind;
      }
      // 水位 V 以下，气腔格 s..S 里能装多少水
      const below = (s: number, S: number, V: number): number => {
        let A = 0;
        for (let m = s; m < S; m++) {
          const u = (V - D[m]) / P;
          if (u > 0) A += (u < ft[m] ? u : ft[m]) * b[m];
        }
        return A;
      };
      // 装下 V 格³ 水时的水位
      const level = (s: number, S: number, V: number): number => {
        let A = 1 / 0, m = -1 / 0;
        for (let u = s; u < S; u++) {
          if (D[u] < A) A = D[u];
          if (D[u] + P > m) m = D[u] + P;
        }
        if (below(s, S, m) <= V) return m;
        for (let u = 0; u < 30; u++) {
          const q = (A + m) * 0.5;
          if (below(s, S, q) < V) A = q; else m = q;
        }
        return (A + m) * 0.5;
      };
      for (let s = 0; s < U.rooms; s++) {
        const S = U.roomStart[s], V = U.roomStart[s + 1];
        let A = -1 / 0;
        for (let z = S; z < V; z++) if (ht[z] > A) A = ht[z];
        const m = U.roomRim[s] + y;
        let u = U.water[s], q = u > 0 ? level(S, V, u) : -1 / 0;
        const tt = this.floodRate * FLOOD_FACTOR * J * U.roomEdge[s] * dt;
        if (A > m && q < A) {
          const z = A - (q > m ? q : m);
          u += tt * z * Math.sqrt(z);
          const p = below(S, V, A);
          if (u > p) u = p;
        } else if (q > m && q > A) {
          const z = A > m ? A : m, p = q - z;
          u -= tt * p * Math.sqrt(p);
          const ct = below(S, V, z);
          if (u < ct) u = ct;
        }
        if (u < 1e-6) u = 0;
        U.water[s] = u;
        q = u > 0 ? level(S, V, u) : -1 / 0;
        for (let z = S; z < V; z++) {
          const p = (q - D[z]) / P;
          X[z] = p <= 0 ? 0 : p < ft[z] ? p : ft[z];
        }
      }
      for (let s = 0; s < v; s++) {
        const S = C[s], A = it[s], m = D[s], u = m + P, q = ht[s], tt = L[s] === FLUID_LAVA ? Lt : Ct;
        const z = q > m ? (q < u ? q : u) - m : 0, p = X[s] * P;
        if (z > 0) buoyancy(tt * b[s] * z / P, S, m + z * 0.5 - y, A);
        if (p > 0) buoyancy(-tt * b[s] * p / P, S, m + p * 0.5 - y, A);
        if (z > p) Ft += b[s] * (z - p) / P;
        Yt += b[s] * p / P;
        if (q > m && q < u && p < z) wave(tt * Nt * b[s] / P, S, A);
      }
    }
    const is = this.drag, fs = this.linearDrag, cs = this.lavaViscosity;
    const rs = grid.sx, ls = grid.sy, as = grid.sz, ms = grid.cells;
    let Wt = 0;
    for (let v = 0; v < ls; v++) {
      for (let R = 0; R < as; R++) {
        for (let G = 0; G < rs; G++, Wt++) {
          const X = ms[Wt];
          if (X === 0 || !c) continue;
          const C = G, H = v, it = R, D = ns, o6 = X * 6;
          const ft = C + (D[o6] + D[o6 + 3]) * 0.5 - r, b = H + (D[o6 + 1] + D[o6 + 4]) * 0.5 - w, ht = it + (D[o6 + 2] + D[o6 + 5]) * 0.5 - d;
          const L = body.r00 * ft + body.r01 * b + body.r02 * ht;
          const E = body.r10 * ft + body.r11 * b + body.r12 * ht;
          const N = body.r20 * ft + body.r21 * b + body.r22 * ht;
          const s = yt[X] === 1 ? W : 0.5 * (at * (D[o6 + 3] - D[o6]) + k * (D[o6 + 4] - D[o6 + 1]) + _ * (D[o6 + 5] - D[o6 + 2]));
          const S = this.submerged(terrain, F + L, y + E, i + N, s > 0.01 ? s : 0.01);
          if (S <= 0) continue;
          const V = this.kind, A = this.surface, m = V === FLUID_LAVA ? Lt : Ct;
          this.lastKinds |= 1 << (V - 1); // 记录浸没流体的种类。
          Zt += S * yt[X];
          buoyancy(m * yt[X] * S, L, this.middle - y, N);
          if (A !== 1 / 0) wave(m * Nt * yt[X] / (2 * s), L, N);
          this.flowAt(terrain, Math.floor(F + L), Math.floor(y + E), Math.floor(i + N));
          const u = g + rt * N - lt * E - this.flowX, q = Z + lt * L - T * N, tt = Y + T * E - rt * L - this.flowZ;
          const z = m * (V === FLUID_LAVA ? cs : 1);
          for (let p = 0; p < 3; p++) {
            // 刚体局部第 p 根轴的世界方向
            const ct = p === 0 ? body.r00 : p === 1 ? body.r01 : body.r02;
            const st = p === 0 ? body.r10 : p === 1 ? body.r11 : body.r12;
            const gt = p === 0 ? body.r20 : p === 1 ? body.r21 : body.r22;
            const xt = u * ct + q * st + tt * gt;
            if (xt > -1e-6 && xt < 1e-6) continue;
            const It = D[o6 + p + 3] - D[o6 + p], Vt = s - (st > 0 ? st : -st) * It * 0.5;
            let Mt = 0;
            for (let pt = -1; pt <= 1; pt += 2) {
              const Qt = C + (p === 0 ? pt : 0), $t = H + (p === 1 ? pt : 0), bt = it + (p === 2 ? pt : 0);
              if (grid.isFull(Qt, $t, bt) || U !== undefined && U.index[Qt - U.x0 + U.nx * (bt - U.z0 + U.nz * ($t - U.y0))] >= 0) continue;
              if (A === 1 / 0) { Mt++; continue; }
              const ts = y + E + pt * st * It * 0.5;
              if (Vt < 1e-3) { if (A > ts) Mt++; }
              else {
                const qt = (A - ts + Vt) / (2 * Vt);
                Mt += qt <= 0 ? 0 : qt >= 1 ? 1 : qt;
              }
            }
            if (Mt <= 0) continue;
            const ys = It >= yt[X] ? yt[X] / It : 1, Kt = z * ys * Mt * 0.5 * (fs + is * (xt > 0 ? xt : -xt)), Rt = -Kt * xt;
            const Et = ct * Rt, Ot = st * Rt, Tt = gt * Rt;
            $ += Et; O += Ot; St += Tt;
            kt += E * Tt - N * Ot; Xt += N * Et - L * Tt; At += L * Ot - E * Et;
            zt += Kt;
            const Jt = L * ct + E * st + N * gt;
            vt += Kt * (L * L + E * E + N * N - Jt * Jt);
          }
        }
      }
    }
    this.lastPocketVolume = Ft;
    this.lastSubmerged = Zt + Ft;
    this.lastInside = Yt;
    body.submerged = this.lastSubmerged;
    if (Zt <= 0 && Ft <= 0 && Yt <= 0) return false;
    const Gt = body.mass * (j * j + nt * nt + B * B) ** 0.5;
    if (Gt > 0) {
      const v = ot - j * body.mass, R = et - nt * body.mass, G = K - B * body.mass;
      const X = Math.sqrt(v * v + R * R + G * G), C = Math.sqrt(Q * Q + dtq * dtq + mt * mt);
      this.lastUnbalance = (X > C ? X : C) / Gt;
    }
    const jt = body.invMass;
    if (zt * dt * jt > 0.9) {
      const v = 0.9 / (zt * dt * jt);
      $ *= v; O *= v; St *= v;
    }
    const Bt = body.ii00 + body.ii11 + body.ii22;
    if (vt * dt * Bt > 0.9) {
      const v = 0.9 / (vt * dt * Bt);
      kt *= v; Xt *= v; At *= v;
    }
    body.fx += ot + $; body.fy += et + O; body.fz += K + St;
    body.tx += Q + kt; body.ty += dtq + Xt; body.tz += mt + At;
    return true;
  }

  /** 刚体的气腔。形状没变、转动不超过约 2° 或距上次重建不到 10 tick 时用上次的结果 */
  private pocketsOf(body: FluidBody, half: number): Pockets | undefined {
    const grid = body.grid, o = this.pockets.get(body);
    if (o !== undefined && o.version === grid.version && (o.age++, Math.abs(o.qx * body.qx + o.qy * body.qy + o.qz * body.qz + o.qw * body.qw) >= POCKET_ROTATION || o.age < POCKET_MIN_AGE)) return o.count > 0 ? o : undefined;
    const h = grid.maxX - grid.minX + 2, M = grid.maxY - grid.minY + 2, n = grid.maxZ - grid.minZ + 2;
    if (grid.count === 0 || h * M * n > FLUID_GRID_MAX_CELLS) {
      this.pockets.delete(body);
      return undefined;
    }
    const c = buildPockets(body, half);
    if (o !== undefined) {
      for (let a = 0; a < c.rooms; a++) {
        for (let e = c.roomStart[a]; e < c.roomStart[a + 1]; e++) {
          const F = c.cells[e * 3], y = c.cells[e * 3 + 1], i = c.cells[e * 3 + 2];
          const r = F - o.x0, w = y - o.y0, d = i - o.z0;
          if (r < 0 || w < 0 || d < 0 || r >= o.nx || w >= o.ny || d >= o.nz) continue;
          const g = o.index[r + o.nx * (d + o.nz * w)];
          if (g < 0 || o.fill[g] <= 0) continue;
          const Z = grid.get(F, y, i);
          c.water[a] += o.fill[g] * (Z === 0 ? 1 : 1 - grid.volume[Z]);
          c.fill[e] = o.fill[g];
        }
      }
    }
    this.pockets.set(body, c);
    return c.count > 0 ? c : undefined;
  }
}

/** 一格柱的缓存键 */
function columnKey(x: number, y: number, z: number): number {
  return ((x & 1023) * 1024 + (z & 1023)) * 1024 + (y & 1023);
}

/** 一格流体的有效高度：上面还有同种流体时算满格，否则按流体码的高度 */
function columnHeight(terrain: FluidTerrain, x: number, y: number, z: number, code: number): number {
  const o = terrain.fluidAt(x, y + 1, z);
  return o !== 0 && fluidKind(o) === fluidKind(code) ? 1 : fluidHeight(code);
}

/** 重建气腔：外扩 1 格的格网上做两遍极小极大路径，找出碗形和钟形空腔，再连成房间 */
function buildPockets(body: FluidBody, half: number): Pockets {
  const grid = body.grid;
  const x = grid.minX - 1, l = grid.minY - 1, o = grid.minZ - 1;
  const h = grid.maxX - grid.minX + 2, M = grid.maxY - grid.minY + 2, n = grid.maxZ - grid.minZ + 2, c = h * M * n;
  const i = new Float64Array(c), r = new Float64Array(c), w = new Uint8Array(c);
  let d = 0;
  for (let k = 0; k < M; k++) {
    for (let _ = 0; _ < n; _++) {
      for (let W = 0; W < h; W++, d++) {
        const P = W + x, ot = k + l, et = _ + o;
        const K = body.r10 * (P + 0.5 + grid.originX) + body.r11 * (ot + 0.5 + grid.originY) + body.r12 * (et + 0.5 + grid.originZ);
        i[d] = K - half;
        r[d] = -(K + half);
        w[d] = grid.isFull(P, ot, et) ? 0 : 1;
      }
    }
  }
  const g = flood(h, M, n, w, i), Z = flood(h, M, n, w, r), Y = new Uint8Array(c);
  let T = 0;
  for (let k = 0; k < c; k++) {
    if (w[k] === 1 && (g[k] > i[k] + 1e-9 || Z[k] > r[k] + 1e-9)) { Y[k] = 1; T++; }
  }
  const rt = new Int32Array(c);
  rt.fill(-1);
  const lt = new Int32Array(T), wt = [0], j: number[] = [], nt: number[] = [], B = h * n;
  let J = 0;
  for (let k = 0; k < c; k++) {
    if (Y[k] !== 1 || rt[k] >= 0) continue;
    const _ = J;
    let W = 1 / 0, P = 1 / 0, ot = -1 / 0;
    rt[k] = J;
    lt[J++] = k;
    for (let et = _; et < J; et++) {
      const K = lt[et];
      if (g[K] < W) W = g[K];
      const Q = Math.floor(K / B);
      if (Q < P) P = Q;
      if (Q > ot) ot = Q;
      const dtq = K % h, mt = Math.floor(K / h) % n;
      for (let $ = 0; $ < 6; $++) {
        let O: number;
        if ($ === 0) { if (dtq === 0) continue; O = K - 1; }
        else if ($ === 1) { if (dtq === h - 1) continue; O = K + 1; }
        else if ($ === 2) { if (mt === 0) continue; O = K - h; }
        else if ($ === 3) { if (mt === n - 1) continue; O = K + h; }
        else if ($ === 4) { if (Q === 0) continue; O = K - B; }
        else { if (Q === M - 1) continue; O = K + B; }
        if (Y[O] !== 1 || rt[O] >= 0) continue;
        rt[O] = J;
        lt[J++] = O;
      }
    }
    wt.push(J);
    j.push(W);
    nt.push(4 * Math.sqrt((J - _) / (ot - P + 1)));
  }
  const at: Pockets = {
    version: grid.version, qx: body.qx, qy: body.qy, qz: body.qz, qw: body.qw, age: 0,
    x0: x, y0: l, z0: o, nx: h, ny: M, nz: n, index: rt, count: T,
    cells: new Int32Array(T * 3), dips: new Float64Array(T), fill: new Float64Array(T),
    rooms: j.length, roomStart: Int32Array.from(wt), roomRim: Float64Array.from(j), roomEdge: Float64Array.from(nt),
    water: new Float64Array(j.length),
  };
  for (let k = 0; k < T; k++) {
    const _ = lt[k];
    at.cells[k * 3] = _ % h + x;
    at.cells[k * 3 + 1] = Math.floor(_ / B) + l;
    at.cells[k * 3 + 2] = Math.floor(_ / h) % n + o;
    at.dips[k] = -Z[_];
  }
  return at;
}

/** 从格网边界出发的极小极大路径：每个格子记下能到达的最小高度 */
function flood(sx: number, sy: number, sz: number, open: Uint8Array, height: Float64Array): Float64Array {
  const o = sx * sy * sz, h = new Float64Array(o);
  h.fill(1 / 0);
  const M = new Uint8Array(o);
  let n = new Int32Array(Math.max(64, o)), c = 0;
  const push = (i: number): void => {
    if (c === n.length) {
      const d = new Int32Array(n.length * 2);
      d.set(n);
      n = d;
    }
    let r = c++;
    const w = h[i];
    while (r > 0) {
      const d = r - 1 >> 1;
      if (h[n[d]] <= w) break;
      n[r] = n[d];
      r = d;
    }
    n[r] = i;
  };
  const pop = (): number => {
    const i = n[0], r = n[--c], w = h[r];
    let d = 0;
    for (; ;) {
      let g = d * 2 + 1;
      if (g >= c || (g + 1 < c && h[n[g + 1]] < h[n[g]] && g++, h[n[g]] >= w)) break;
      n[d] = n[g];
      d = g;
    }
    n[d] = r;
    return i;
  };
  let F = 0;
  for (let i = 0; i < sy; i++) {
    for (let r = 0; r < sz; r++) {
      for (let w = 0; w < sx; w++, F++) {
        if (w !== 0 && i !== 0 && r !== 0 && w !== sx - 1 && i !== sy - 1 && r !== sz - 1) continue;
        h[F] = height[F];
        push(F);
      }
    }
  }
  const y = sx * sz;
  while (c > 0) {
    const i = pop();
    if (M[i] === 1) continue;
    M[i] = 1;
    const r = h[i], w = i % sx, d = Math.floor(i / sx) % sz, g = Math.floor(i / y);
    for (let Z = 0; Z < 6; Z++) {
      let Y: number;
      if (Z === 0) { if (w === 0) continue; Y = i - 1; }
      else if (Z === 1) { if (w === sx - 1) continue; Y = i + 1; }
      else if (Z === 2) { if (d === 0) continue; Y = i - sx; }
      else if (Z === 3) { if (d === sz - 1) continue; Y = i + sx; }
      else if (Z === 4) { if (g === 0) continue; Y = i - y; }
      else { if (g === sy - 1) continue; Y = i + y; }
      if (M[Y] === 1 || open[Y] === 0) continue;
      const T = height[Y] > r ? height[Y] : r;
      if (T < h[Y]) { h[Y] = T; push(Y); }
    }
  }
  return h;
}
