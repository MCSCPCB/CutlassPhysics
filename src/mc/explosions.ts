// 原版爆炸推动刚体：爆炸先排队（同一处的合并），下一次推进前按离爆心最近的接触点施加冲量。
// 接触点是刚体各形状上离爆心最近的点。
import { BoxGeometry } from "../core/collision/geometry/box-geometry.js";
import type { ConvexGeometry } from "../core/collision/geometry/geometry.js";
import { SphereGeometry } from "../core/collision/geometry/sphere-geometry.js";
import { GjkEpa, GjkEpaResultState } from "../core/collision/narrowphase/gjkepa/gjk-epa.js";
import { Transform } from "../core/common/transform.js";
import { RigidBodyType } from "../core/dynamics/rigidbody/rigid-body-config.js";
import type { RigidBody } from "../core/dynamics/rigidbody/rigid-body.js";
import type { Shape } from "../core/dynamics/rigidbody/shape.js";

const MIN_RADIUS = 2.5, DEFAULT_RADIUS = 4.25, RADIUS_MARGIN = 1.25, MAX_RADIUS = 8;
const IMPULSE_MULTIPLIER = 2.78, OCCLUDED_SCALE = 0.35, FALLOFF_POWER = 0.68, UPWARD_BIAS = 0.4;
const MAX_EFFECTS_PER_BODY = 16, MAX_OCCLUSION_RAYS_PER_BODY = 4;
const MAX_LINEAR_VELOCITY_CHANGE = 12, MAX_ANGULAR_VELOCITY_CHANGE = 2.5;
// 爆炸冲量的基础强度。
const PUNCH_STRENGTH = 2.1, UPRIGHT_MULTIPLIER = 1.5, BACKEND_COMPENSATION = 1.5;
const CURVE_SCALE = 2, CURVE_EXPONENT = 0.5, CURVE_SLOPE = 0.8, ZERO_MASS_IMPULSE = 1.75;
const CURVE_OFFSET = CURVE_SLOPE / (CURVE_SCALE * CURVE_EXPONENT), CURVE_INVERSE = 1 / (CURVE_EXPONENT - 1);
const CURVE_BASE = Math.pow(CURVE_OFFSET, CURVE_INVERSE), CURVE_BIAS = Math.pow(CURVE_OFFSET, CURVE_EXPONENT * CURVE_INVERSE);
const CURVE_QUADRATIC = ZERO_MASS_IMPULSE + CURVE_SLOPE - 2;
const EPSILON = 1e-8;

function punchCurve(mass: number): number {
  if (!Number.isFinite(mass) || mass <= 0) return 0;
  const m = mass - 1;
  if (mass < 1) return ((CURVE_QUADRATIC * m + CURVE_SLOPE - 1) * m + 1) * mass;
  return CURVE_SCALE * (Math.pow(m + CURVE_BASE, CURVE_EXPONENT) - CURVE_BIAS) + 1;
}

export interface Vec { x: number; y: number; z: number }

/** 由来源实体位置或被炸方块中心确定爆心；半径为最远方块距离加 1.25，限制在 [2.5, 8]。 */
export function explosionOf(source: Vec | undefined, impacted: readonly Vec[]): { center: Vec; radius: number } | undefined {
  let center = source;
  if (center === undefined) {
    if (impacted.length === 0) return undefined;
    let x = 0, y = 0, z = 0;
    for (const b of impacted) { x += b.x + 0.5; y += b.y + 0.5; z += b.z + 0.5; }
    center = { x: x / impacted.length, y: y / impacted.length, z: z / impacted.length };
  }
  if (impacted.length === 0) return { center, radius: DEFAULT_RADIUS };
  let max = 0;
  for (const b of impacted) max = Math.max(max, (b.x + 0.5 - center.x) ** 2 + (b.y + 0.5 - center.y) ** 2 + (b.z + 0.5 - center.z) ** 2);
  return { center, radius: Math.min(MAX_RADIUS, Math.max(MIN_RADIUS, Math.sqrt(max) + RADIUS_MARGIN)) };
}

interface Pending { center: Vec; radius: number; count: number }

const point = new SphereGeometry(0);
const pointTransform = new Transform();

/** 形状上离 c 最近的点；c 在形状里面时就是 c。超过 radius 时返回 undefined */
export function closestOnShape(geom: ConvexGeometry, tf: Transform, c: Vec, radius: number, out: Vec): number | undefined {
  if (geom instanceof BoxGeometry) {
    const dx = c.x - tf.px, dy = c.y - tf.py, dz = c.z - tf.pz;
    const lx = Math.max(-geom.hx, Math.min(geom.hx, tf.r00 * dx + tf.r10 * dy + tf.r20 * dz));
    const ly = Math.max(-geom.hy, Math.min(geom.hy, tf.r01 * dx + tf.r11 * dy + tf.r21 * dz));
    const lz = Math.max(-geom.hz, Math.min(geom.hz, tf.r02 * dx + tf.r12 * dy + tf.r22 * dz));
    out.x = tf.px + tf.r00 * lx + tf.r01 * ly + tf.r02 * lz;
    out.y = tf.py + tf.r10 * lx + tf.r11 * ly + tf.r12 * lz;
    out.z = tf.pz + tf.r20 * lx + tf.r21 * ly + tf.r22 * lz;
  } else {
    const gjk = GjkEpa.getInstance();
    pointTransform.px = c.x; pointTransform.py = c.y; pointTransform.pz = c.z;
    if (gjk.computeDistance(point, geom, pointTransform, tf, null) !== GjkEpaResultState.SUCCEEDED) return undefined;
    if (gjk.distance <= geom.gjkMargin) {
      out.x = c.x; out.y = c.y; out.z = c.z;
    } else {
      // 最近点从核心表面沿着指向 c 的方向移出 gjkMargin
      const p = gjk.closestPoint2, s = geom.gjkMargin / gjk.distance;
      out.x = p.x + (c.x - p.x) * s; out.y = p.y + (c.y - p.y) * s; out.z = p.z + (c.z - p.z) * s;
    }
  }
  const d = Math.hypot(out.x - c.x, out.y - c.y, out.z - c.z);
  return d < radius ? d : undefined;
}

/** 爆心到接触点之间有没有方块挡着（getBlockFromRay，不算液体和可穿过的方块） */
export type OcclusionTest = (center: Vec, contact: Vec) => boolean;

interface RaycastDimension {
  getBlockFromRay(location: Vec, direction: Vec, options?: { maxDistance?: number; includeLiquidBlocks?: boolean; includePassableBlocks?: boolean }): unknown;
}

/** 射线终点向爆心缩进 0.15 格，排除接触表面自身的遮挡。未加载错误按无遮挡处理。 */
export function worldOcclusion(dimension: RaycastDimension): OcclusionTest {
  return (center, contact) => {
    const ox = contact.x - center.x, oy = contact.y - center.y, oz = contact.z - center.z;
    const distance = Math.hypot(ox, oy, oz);
    if (!Number.isFinite(distance) || distance <= 0.2) return false;
    try {
      return dimension.getBlockFromRay(center, { x: ox / distance, y: oy / distance, z: oz / distance }, {
        includeLiquidBlocks: false, includePassableBlocks: false, maxDistance: Math.max(0, distance - 0.15),
      }) !== undefined;
    } catch {
      return false;
    }
  };
}

interface ExplosionEventLike {
  dimension?: { id: string };
  source?: { location: Vec } | undefined;
  getImpactedBlocks(): { location: Vec }[];
}

/** 订阅原版爆炸事件（world.afterEvents.explosion），只处理指定维度的，排进 queue。返回取消订阅的函数 */
export function watchExplosions(
  signal: { subscribe(cb: (e: ExplosionEventLike) => void): unknown; unsubscribe(cb: (e: ExplosionEventLike) => void): void },
  dimensionId: string, queue: ExplosionQueue,
): () => void {
  const cb = (e: ExplosionEventLike): void => {
    try {
      if (e.dimension !== undefined && e.dimension.id !== dimensionId) return;
      const impacted: Vec[] = [];
      for (const b of e.getImpactedBlocks()) {
        try { impacted.push({ x: b.location.x, y: b.location.y, z: b.location.z }); } catch { /* 方块已经不在了 */ }
      }
      let source: Vec | undefined;
      try {
        const l = e.source?.location;
        if (l !== undefined && Number.isFinite(l.x) && Number.isFinite(l.y) && Number.isFinite(l.z)) source = { x: l.x, y: l.y, z: l.z };
      } catch { /* 来源实体已经失效 */ }
      const blast = explosionOf(source, impacted);
      if (blast !== undefined) queue.queue(blast.center, blast.radius);
    } catch {
      // 读事件对象出错时这次爆炸不推刚体，不影响别的事件
    }
  };
  signal.subscribe(cb);
  return () => {
    try { signal.unsubscribe(cb); } catch { /* 退出回调里不允许退订，忽略 */ }
  };
}

/** 排队的爆炸。queue 收集，apply 在下一次推进之前把冲量施加到刚体上 */
export class ExplosionQueue {
  private pending = new Map<string, Pending>();

  get size(): number {
    return this.pending.size;
  }

  queue(center: Vec, radius: number): void {
    if (!Number.isFinite(center.x) || !Number.isFinite(center.y) || !Number.isFinite(center.z) || !(radius > 0) || !Number.isFinite(radius)) return;
    // 同一处（1/4 格、半径 1/2 格以内）的爆炸合并：中心取平均，半径取大的，按次数加强
    const key = `${Math.round(center.x * 4)},${Math.round(center.y * 4)},${Math.round(center.z * 4)}|${Math.round(radius * 2)}`;
    const e = this.pending.get(key);
    if (e === undefined) {
      this.pending.set(key, { center: { x: center.x, y: center.y, z: center.z }, radius, count: 1 });
      return;
    }
    const n = e.count + 1;
    e.center = { x: (e.center.x * e.count + center.x) / n, y: (e.center.y * e.count + center.y) / n, z: (e.center.z * e.count + center.z) / n };
    e.count = n;
    e.radius = Math.max(e.radius, radius);
  }

  clear(): void {
    this.pending.clear();
  }

  apply(bodies: Iterable<RigidBody>, occluded: OcclusionTest): void {
    if (this.pending.size === 0) return;
    const explosions = [...this.pending.values()];
    this.pending.clear();
    const contact: Vec = { x: 0, y: 0, z: 0 }, best: Vec = { x: 0, y: 0, z: 0 };
    for (const rb of bodies) {
      if (rb.type !== RigidBodyType.DYNAMIC || rb.shapeList === null) continue;
      // 包围盒和爆炸球相交的，按离得近排序，最多 16 个
      let minX = Infinity, minY = Infinity, minZ = Infinity, maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
      for (let s: Shape | null = rb.shapeList; s !== null; s = s.next) {
        const a = s.aabb;
        minX = Math.min(minX, a.minX); minY = Math.min(minY, a.minY); minZ = Math.min(minZ, a.minZ);
        maxX = Math.max(maxX, a.maxX); maxY = Math.max(maxY, a.maxY); maxZ = Math.max(maxZ, a.maxZ);
      }
      const near: { e: Pending; score: number }[] = [];
      for (const e of explosions) {
        const c = e.center;
        const dx = c.x - Math.max(minX, Math.min(maxX, c.x)), dy = c.y - Math.max(minY, Math.min(maxY, c.y)), dz = c.z - Math.max(minZ, Math.min(maxZ, c.z));
        const d2 = dx * dx + dy * dy + dz * dz;
        // 包围盒距离预筛；实际接触点另按爆炸半径过滤。
        if (d2 > e.radius * e.radius) continue;
        near.push({ e, score: d2 / (e.radius * e.radius) });
      }
      if (near.length === 0) continue;
      near.sort((a, b) => a.score - b.score);
      if (near.length > MAX_EFFECTS_PER_BODY) near.length = MAX_EFFECTS_PER_BODY;
      let lX = 0, lY = 0, lZ = 0, tX = 0, tY = 0, tZ = 0, contacted = false, rays = 0;
      const tf = rb.transform;
      const upright = Math.max(0, Math.min(1, tf.r11));
      for (const { e } of near) {
        let distance = Infinity;
        for (let s: Shape | null = rb.shapeList; s !== null; s = s.next) {
          const d = closestOnShape(s.geom as ConvexGeometry, s.transform, e.center, e.radius, contact);
          if (d !== undefined && d < distance) { distance = d; best.x = contact.x; best.y = contact.y; best.z = contact.z; }
        }
        if (distance === Infinity) continue;
        let occlusion = 1;
        // 每个刚体最多测 4 条遮挡射线；超出的直接按没挡着算
        if (rays < MAX_OCCLUSION_RAYS_PER_BODY && distance > 0.2 && occluded(e.center, best)) occlusion = OCCLUDED_SCALE;
        if (rays < MAX_OCCLUSION_RAYS_PER_BODY) rays++;
        // 方向：爆心指向接触点，再往上偏 0.4
        let dX = best.x - e.center.x, dY = best.y - e.center.y, dZ = best.z - e.center.z;
        if (Math.hypot(dX, dY, dZ) < EPSILON) { dX = tf.px - e.center.x; dY = tf.py - e.center.y; dZ = tf.pz - e.center.z; }
        dY += UPWARD_BIAS;
        let len = Math.hypot(dX, dY, dZ);
        if (len < EPSILON) { dX = 0; dY = 1; dZ = 0; len = 1; }
        dX /= len; dY /= len; dZ /= len;
        // 接触点沿这个方向的有效质量
        const rX = best.x - tf.px, rY = best.y - tf.py, rZ = best.z - tf.pz;
        const cX = rY * dZ - rZ * dY, cY = rZ * dX - rX * dZ, cZ = rX * dY - rY * dX;
        const k = rb.invMass + cX * (rb.ii00 * cX + rb.ii01 * cY + rb.ii02 * cZ) + cY * (rb.ii10 * cX + rb.ii11 * cY + rb.ii12 * cZ) + cZ * (rb.ii20 * cX + rb.ii21 * cY + rb.ii22 * cZ);
        const punch = BACKEND_COMPENSATION * (1 + UPRIGHT_MULTIPLIER * upright) * PUNCH_STRENGTH * punchCurve(k > 0 ? 1 / k : 0);
        const magnitude = occlusion * Math.pow(Math.max(0, 1 - distance / e.radius), FALLOFF_POWER) *
          Math.min(2, Math.sqrt(Math.max(1, Math.floor(e.count)))) * punch * IMPULSE_MULTIPLIER;
        if (!(magnitude > 0)) continue;
        contacted = true;
        const iX = dX * magnitude, iY = dY * magnitude, iZ = dZ * magnitude;
        lX += iX; lY += iY; lZ += iZ;
        tX += rY * iZ - rZ * iY; tY += rZ * iX - rX * iZ; tZ += rX * iY - rY * iX;
      }
      if (!contacted) continue;
      // 一批冲量合起来限幅：速度变化不超过 12，角速度变化不超过 2.5（按最小的主惯量）
      const maxL = rb.mass * MAX_LINEAR_VELOCITY_CHANGE, l = Math.hypot(lX, lY, lZ);
      if (l > maxL) { const s = maxL / l; lX *= s; lY *= s; lZ *= s; }
      let inertia = Infinity;
      for (const v of [rb.li00, rb.li11, rb.li22]) if (v > 0 && v < inertia) inertia = v;
      const maxT = Number.isFinite(inertia) ? inertia * MAX_ANGULAR_VELOCITY_CHANGE : 0, t = Math.hypot(tX, tY, tZ);
      if (t > maxT) { const s = t > 0 ? maxT / t : 0; tX *= s; tY *= s; tZ *= s; }
      rb.applyLinearImpulse(lX, lY, lZ);
      rb.applyAngularImpulse(tX, tY, tZ);
    }
  }
}
