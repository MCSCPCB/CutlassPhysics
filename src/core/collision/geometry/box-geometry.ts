// 移植自 OimoPhysics src/oimo/collision/geometry/BoxGeometry.hx（MIT，见 Credits.md）
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import type { Aabb } from "./aabb.js";
import { ConvexGeometry, GeometryType } from "./geometry.js";
import type { RayCastHit } from "./ray-cast-hit.js";

/** 盒子，用半边长描述（1×1×1 的方块是 0.5, 0.5, 0.5） */
export class BoxGeometry extends ConvexGeometry {
  readonly hx: number;
  readonly hy: number;
  readonly hz: number;

  constructor(hx: number, hy: number, hz: number) {
    super(GeometryType.BOX);
    this.hx = hx;
    this.hy = hy;
    this.hz = hz;
    this.updateMass();
    // GJK 外边距不超过最短半边长的 20%
    const minHalfExtent = hx < hy ? (hz < hx ? hz : hx) : hz < hy ? hz : hy;
    if (this.gjkMargin > minHalfExtent * 0.2) this.gjkMargin = minHalfExtent * 0.2;
  }

  updateMass(): void {
    this.volume = 8 * (this.hx * this.hy * this.hz);
    const sqx = this.hx * this.hx;
    const sqy = this.hy * this.hy;
    const sqz = this.hz * this.hz;
    this.ic00 = (1 / 3) * (sqy + sqz);
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = (1 / 3) * (sqz + sqx);
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = (1 / 3) * (sqx + sqy);
  }

  computeAabb(aabb: Aabb, tf: Transform): void {
    // 三根半轴转到世界坐标后取绝对值相加，得到包围盒的半尺寸
    let a = tf.r00 * this.hx, b = tf.r01 * this.hy, c = tf.r02 * this.hz;
    const ex = (a < 0 ? -a : a) + (b < 0 ? -b : b) + (c < 0 ? -c : c);
    a = tf.r10 * this.hx;
    b = tf.r11 * this.hy;
    c = tf.r12 * this.hz;
    const ey = (a < 0 ? -a : a) + (b < 0 ? -b : b) + (c < 0 ? -c : c);
    a = tf.r20 * this.hx;
    b = tf.r21 * this.hy;
    c = tf.r22 * this.hz;
    const ez = (a < 0 ? -a : a) + (b < 0 ? -b : b) + (c < 0 ? -c : c);
    aabb.minX = tf.px - ex;
    aabb.minY = tf.py - ey;
    aabb.minZ = tf.pz - ez;
    aabb.maxX = tf.px + ex;
    aabb.maxY = tf.py + ey;
    aabb.maxZ = tf.pz + ez;
  }

  computeLocalSupportingVertex(dir: Vec3, out: Vec3): void {
    // 核心是每边缩进 GJK 外边距的小盒子（外边距超过半边长时缩成 0）
    let mx = this.gjkMargin, my = this.gjkMargin, mz = this.gjkMargin;
    if (!(mx < this.hx)) mx = this.hx;
    if (!(my < this.hy)) my = this.hy;
    if (!(mz < this.hz)) mz = this.hz;
    const cx = this.hx - mx, cy = this.hy - my, cz = this.hz - mz;
    out.x = dir.x > 0 ? cx : -cx;
    out.y = dir.y > 0 ? cy : -cy;
    out.z = dir.z > 0 ? cz : -cz;
  }

  rayCastLocal(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    hit: RayCastHit,
  ): boolean {
    const hx = this.hx, hy = this.hy, hz = this.hz;
    const dx = endX - beginX;
    const dy = endY - beginY;
    const dz = endZ - beginZ;
    // 板块法：每个轴算出线段进入、离开这一对平面的比例，进入取最晚的，离开取最早的
    let tminX = 0, tminY = 0, tminZ = 0;
    let tmaxX = 1, tmaxY = 1, tmaxZ = 1;
    if (dx > -1e-6 && dx < 1e-6) {
      // 和这一对平面平行：起点不在两平面之间就不可能命中
      if (beginX <= -hx || beginX >= hx) return false;
    } else {
      const inv = 1 / dx;
      let t1 = (-hx - beginX) * inv;
      let t2 = (hx - beginX) * inv;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminX = t1;
      if (t2 < 1) tmaxX = t2;
    }
    if (dy > -1e-6 && dy < 1e-6) {
      if (beginY <= -hy || beginY >= hy) return false;
    } else {
      const inv = 1 / dy;
      let t1 = (-hy - beginY) * inv;
      let t2 = (hy - beginY) * inv;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminY = t1;
      if (t2 < 1) tmaxY = t2;
    }
    if (dz > -1e-6 && dz < 1e-6) {
      if (beginZ <= -hz || beginZ >= hz) return false;
    } else {
      const inv = 1 / dz;
      let t1 = (-hz - beginZ) * inv;
      let t2 = (hz - beginZ) * inv;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminZ = t1;
      if (t2 < 1) tmaxZ = t2;
    }
    if (tminX >= 1 || tminY >= 1 || tminZ >= 1 || tmaxX <= 0 || tmaxY <= 0 || tmaxZ <= 0) return false;
    let min = tminX;
    let max = tmaxX;
    // 命中的是最后进入的那一对平面；比例相同时取靠前的轴
    let hitAxis = 0;
    if (tminY > min) {
      min = tminY;
      hitAxis = 1;
    }
    if (tminZ > min) {
      min = tminZ;
      hitAxis = 2;
    }
    if (tmaxY < max) max = tmaxY;
    if (tmaxZ < max) max = tmaxZ;
    if (min > max) return false;
    // 起点在盒子内部（或正好在表面上往里走）
    if (min === 0) return false;
    if (hitAxis === 0) {
      hit.normalX = dx > 0 ? -1 : 1;
      hit.normalY = 0;
      hit.normalZ = 0;
    } else if (hitAxis === 1) {
      hit.normalX = 0;
      hit.normalY = dy > 0 ? -1 : 1;
      hit.normalZ = 0;
    } else {
      hit.normalX = 0;
      hit.normalY = 0;
      hit.normalZ = dz > 0 ? -1 : 1;
    }
    hit.positionX = beginX + min * dx;
    hit.positionY = beginY + min * dy;
    hit.positionZ = beginZ + min * dz;
    hit.fraction = min;
    return true;
  }
}
