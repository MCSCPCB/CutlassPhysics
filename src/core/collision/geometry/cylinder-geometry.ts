// 移植自 OimoPhysics src/oimo/collision/geometry/CylinderGeometry.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import type { Aabb } from "./aabb.js";
import { ConvexGeometry, GeometryType } from "./geometry.js";
import type { RayCastHit } from "./ray-cast-hit.js";

/** 圆柱体：轴沿局部 y 轴，高度从 -halfHeight 到 +halfHeight */
export class CylinderGeometry extends ConvexGeometry {
  readonly radius: number;
  readonly halfHeight: number;

  constructor(radius: number, halfHeight: number) {
    super(GeometryType.CYLINDER);
    this.radius = radius;
    this.halfHeight = halfHeight;
    this.updateMass();
  }

  updateMass(): void {
    const r2 = this.radius * this.radius;
    const h2 = this.halfHeight * this.halfHeight * 4;
    // π 用原版截断的常数
    this.volume = 3.14159265358979 * r2 * this.halfHeight * 2;
    this.ic00 = (1 / 12) * (3 * r2 + h2);
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 0.5 * r2;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = (1 / 12) * (3 * r2 + h2);
  }

  computeAabb(aabb: Aabb, tf: Transform): void {
    // 轴方向贡献 |axis| * 半高，圆面贡献 sqrt(1 - axis²) * 半径
    let ax = tf.r01, ay = tf.r11, az = tf.r21;
    if (ax < 0) ax = -ax;
    if (ay < 0) ay = -ay;
    if (az < 0) az = -az;
    const ax2 = ax * ax, ay2 = ay * ay, az2 = az * az;
    let erX = Math.sqrt(1 - ax2), erY = Math.sqrt(1 - ay2), erZ = Math.sqrt(1 - az2);
    erX *= this.radius;
    erY *= this.radius;
    erZ *= this.radius;
    const ehX = ax * this.halfHeight, ehY = ay * this.halfHeight, ehZ = az * this.halfHeight;
    const maxX = erX + ehX, maxY = erY + ehY, maxZ = erZ + ehZ;
    aabb.minX = tf.px - maxX;
    aabb.minY = tf.py - maxY;
    aabb.minZ = tf.pz - maxZ;
    aabb.maxX = tf.px + maxX;
    aabb.maxY = tf.py + maxY;
    aabb.maxZ = tf.pz + maxZ;
  }

  computeLocalSupportingVertex(dir: Vec3, out: Vec3): void {
    // 核心是半径、半高各缩进 GJK 外边距的小圆柱
    const rx = dir.x, rz = dir.z;
    const len = rx * rx + rz * rz;
    let coreRadius = this.radius - this.gjkMargin;
    if (coreRadius < 0) coreRadius = 0;
    const invLen = len > 0 ? coreRadius / Math.sqrt(len) : 0;
    let coreHeight = this.halfHeight - this.gjkMargin;
    if (coreHeight < 0) coreHeight = 0;
    out.x = rx * invLen;
    out.y = dir.y > 0 ? coreHeight : -coreHeight;
    out.z = rz * invLen;
  }

  rayCastLocal(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    hit: RayCastHit,
  ): boolean {
    const halfH = this.halfHeight;
    const dx = endX - beginX;
    const dy = endY - beginY;
    const dz = endZ - beginZ;
    // 先和上下两个端面之间的板块求交（y 方向）
    let tminy = 0;
    let tmaxy = 1;
    if (dy > -1e-6 && dy < 1e-6) {
      if (beginY <= -halfH || beginY >= halfH) return false;
    } else {
      const inv = 1 / dy;
      let t1 = (-halfH - beginY) * inv;
      let t2 = (halfH - beginY) * inv;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminy = t1;
      if (t2 < 1) tmaxy = t2;
    }
    if (tminy >= 1 || tmaxy <= 0) return false;
    // 再和无限长的圆柱面求交（xz 平面）
    let tminxz: number;
    let tmaxxz: number;
    const a = dx * dx + dz * dz;
    const b = beginX * dx + beginZ * dz;
    const c = beginX * beginX + beginZ * beginZ - this.radius * this.radius;
    const D = b * b - a * c;
    if (D < 0) return false;
    if (a > 0) {
      const sqrtD = Math.sqrt(D);
      tminxz = (-b - sqrtD) / a;
      tmaxxz = (-b + sqrtD) / a;
      if (tminxz >= 1 || tmaxxz <= 0) return false;
    } else {
      // 线段和轴平行：起点不在圆柱里就不可能命中
      if (c >= 0) return false;
      tminxz = 0;
      tmaxxz = 1;
    }
    // 两段区间没有重叠就没命中；进入得晚的那个决定命中的是端面还是侧面
    if (tmaxxz <= tminy || tmaxy <= tminxz) return false;
    let min: number;
    if (tminxz < tminy) {
      min = tminy;
      if (min === 0) return false;
      hit.normalX = 0;
      hit.normalY = dy > 0 ? -1 : 1;
      hit.normalZ = 0;
    } else {
      min = tminxz;
      if (min === 0) return false;
      const nx = beginX + dx * min, nz = beginZ + dz * min;
      let invLen = Math.sqrt(nx * nx + 0 * 0 + nz * nz);
      if (invLen > 0) invLen = 1 / invLen;
      hit.normalX = nx * invLen;
      hit.normalY = 0 * invLen;
      hit.normalZ = nz * invLen;
    }
    hit.positionX = beginX + min * dx;
    hit.positionY = beginY + min * dy;
    hit.positionZ = beginZ + min * dz;
    hit.fraction = min;
    return true;
  }
}
