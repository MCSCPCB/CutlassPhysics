// 移植自 OimoPhysics src/oimo/collision/geometry/SphereGeometry.hx（MIT，见 Credits.md）
import { compat } from "../../compat.js";
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import type { Aabb } from "./aabb.js";
import { ConvexGeometry, GeometryType } from "./geometry.js";
import type { RayCastHit } from "./ray-cast-hit.js";

/** 球体，中心在原点 */
export class SphereGeometry extends ConvexGeometry {
  readonly radius: number;

  constructor(radius: number) {
    super(GeometryType.SPHERE);
    this.radius = radius;
    // 整个球都当作 GJK 外边距，核心是一个点
    this.gjkMargin = radius;
    this.updateMass();
  }

  updateMass(): void {
    // 4π/3 的近似常量，保留原有浮点精度。
    this.volume = 4.188790204786386 * this.radius * this.radius * this.radius;
    this.ic00 = 0.4 * this.radius * this.radius;
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 0.4 * this.radius * this.radius;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = 0.4 * this.radius * this.radius;
  }

  computeAabb(aabb: Aabb, tf: Transform): void {
    const r = this.radius;
    aabb.minX = tf.px - r;
    aabb.minY = tf.py - r;
    aabb.minZ = tf.pz - r;
    aabb.maxX = tf.px + r;
    aabb.maxY = tf.py + r;
    aabb.maxZ = tf.pz + r;
  }

  computeLocalSupportingVertex(_dir: Vec3, out: Vec3): void {
    // 核心是球心一个点
    out.zero();
  }

  rayCastLocal(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    hit: RayCastHit,
  ): boolean {
    // 解 |begin + t·d|² = r²，取较小的根
    const dX = endX - beginX;
    const dY = endY - beginY;
    const dZ = endZ - beginZ;
    const a = dX * dX + dY * dY + dZ * dZ;
    const b = beginX * dX + beginY * dY + beginZ * dZ;
    const D = b * b - a * (beginX * beginX + beginY * beginY + beginZ * beginZ - this.radius * this.radius);
    if (D < 0) return false;
    // 零长度线段不命中；兼容模式跳过此检查。
    if (a === 0 && !compat.keepOimoBugs) return false;
    const t = (-b - Math.sqrt(D)) / a;
    if (t < 0 || t > 1) return false;
    const pX = beginX + dX * t;
    const pY = beginY + dY * t;
    const pZ = beginZ + dZ * t;
    let l = pX * pX + pY * pY + pZ * pZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    hit.positionX = pX;
    hit.positionY = pY;
    hit.positionZ = pZ;
    hit.normalX = pX * l;
    hit.normalY = pY * l;
    hit.normalZ = pZ * l;
    hit.fraction = t;
    return true;
  }
}
