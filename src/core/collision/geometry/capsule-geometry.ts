// 移植自 OimoPhysics src/oimo/collision/geometry/CapsuleGeometry.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { compat } from "../../compat.js";
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import type { Aabb } from "./aabb.js";
import { ConvexGeometry, GeometryType } from "./geometry.js";
import type { RayCastHit } from "./ray-cast-hit.js";

/**
 * 胶囊体：沿局部 y 轴的线段（从 -halfHeight 到 +halfHeight）外面包一层半径为 radius 的球面。
 * 总高度是 2 * (halfHeight + radius)。
 */
export class CapsuleGeometry extends ConvexGeometry {
  readonly radius: number;
  readonly halfHeight: number;

  constructor(radius: number, halfHeight: number) {
    super(GeometryType.CAPSULE);
    this.radius = radius;
    this.halfHeight = halfHeight;
    // 半径部分整个当作 GJK 外边距，核心是中间那条线段
    this.gjkMargin = radius;
    this.updateMass();
  }

  updateMass(): void {
    // 圆柱部分 + 两端合起来的一个球；π 用原版截断的常数
    const r2 = this.radius * this.radius;
    const hh2 = this.halfHeight * this.halfHeight;
    const cylinderVolume = 6.28318530717958 * r2 * this.halfHeight;
    const sphereVolume = (3.14159265358979 * r2 * this.radius * 4) / 3;
    this.volume = cylinderVolume + sphereVolume;
    const invVolume = this.volume === 0 ? 0 : 1 / this.volume;
    const inertiaXZ =
      invVolume * (cylinderVolume * (r2 * 0.25 + hh2 / 3) + sphereVolume * (r2 * 0.4 + this.halfHeight * this.radius * 0.75 + hh2));
    this.ic00 = inertiaXZ;
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = invVolume * (cylinderVolume * r2 * 0.5 + sphereVolume * r2 * 0.4);
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = inertiaXZ;
  }

  computeAabb(aabb: Aabb, tf: Transform): void {
    // 半尺寸 = 半径 + |局部 y 轴| * 半高
    let ax = tf.r01, ay = tf.r11, az = tf.r21;
    if (ax < 0) ax = -ax;
    if (ay < 0) ay = -ay;
    if (az < 0) az = -az;
    ax *= this.halfHeight;
    ay *= this.halfHeight;
    az *= this.halfHeight;
    let ex = this.radius, ey = this.radius, ez = this.radius;
    ex += ax;
    ey += ay;
    ez += az;
    aabb.minX = tf.px - ex;
    aabb.minY = tf.py - ey;
    aabb.minZ = tf.pz - ez;
    aabb.maxX = tf.px + ex;
    aabb.maxY = tf.py + ey;
    aabb.maxZ = tf.pz + ez;
  }

  computeLocalSupportingVertex(dir: Vec3, out: Vec3): void {
    // 核心是中轴线段，取靠近 dir 的那个端点
    if (dir.y > 0) out.init(0, this.halfHeight, 0);
    else out.init(0, -this.halfHeight, 0);
  }

  rayCastLocal(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    hit: RayCastHit,
  ): boolean {
    const halfH = this.halfHeight;
    const dx = endX - beginX;
    const dz = endZ - beginZ;
    // 先和无限长的圆柱面求交（只看 xz 平面）
    let tminxz: number;
    let a = dx * dx + dz * dz;
    let b = beginX * dx + beginZ * dz;
    let c = beginX * beginX + beginZ * beginZ - this.radius * this.radius;
    let D = b * b - a * c;
    if (D < 0) return false;
    if (a > 0) {
      const sqrtD = Math.sqrt(D);
      tminxz = (-b - sqrtD) / a;
      const tmaxxz = (-b + sqrtD) / a;
      if (tminxz >= 1 || tmaxxz <= 0) return false;
    } else {
      // 线段和 y 轴平行：起点不在圆柱里就不可能命中
      if (c >= 0) return false;
      tminxz = 0;
    }
    const crossY = beginY + (endY - beginY) * tminxz;
    if (crossY > -halfH && crossY < halfH) {
      // 进入圆柱面时还在两端的半球之间：命中侧面（起点在内部则不算）
      if (tminxz > 0) {
        const min = tminxz;
        const nx = beginX + dx * min, nz = beginZ + dz * min;
        // 法线是 (x, 0, z) 归一化；原版先求长度（变量名叫 invLen），大于 0 才取倒数
        let invLen = Math.sqrt(nx * nx + 0 * 0 + nz * nz);
        if (invLen > 0) invLen = 1 / invLen;
        hit.normalX = nx * invLen;
        hit.normalY = 0 * invLen;
        hit.normalZ = nz * invLen;
        hit.positionX = beginX + min * dx;
        hit.positionY = crossY;
        hit.positionZ = beginZ + min * dz;
        hit.fraction = min;
        return true;
      }
      return false;
    }
    // 否则和靠近的那一端的半球求交
    const sphereY = crossY < 0 ? -halfH : halfH;
    const sbX = beginX - 0;
    const sbY = beginY - sphereY;
    const sbZ = beginZ - 0;
    const dX = endX - beginX;
    const dY = endY - beginY;
    const dZ = endZ - beginZ;
    a = dX * dX + dY * dY + dZ * dZ;
    b = sbX * dX + sbY * dY + sbZ * dZ;
    c = sbX * sbX + sbY * sbY + sbZ * sbZ - this.radius * this.radius;
    D = b * b - a * c;
    if (D < 0) return false;
    // 零长度线段不命中；兼容模式保留上游行为。
    if (a === 0 && !compat.keepOimoBugs) return false;
    const t = (-b - Math.sqrt(D)) / a;
    if (t < 0 || t > 1) return false;
    let pX = sbX + dX * t;
    let pY = sbY + dY * t;
    let pZ = sbZ + dZ * t;
    let l = pX * pX + pY * pY + pZ * pZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    hit.normalX = pX * l;
    hit.normalY = pY * l;
    hit.normalZ = pZ * l;
    // 加上球心偏移；零偏移项将 -0 规范化为 +0。
    pX += 0;
    pY += sphereY;
    pZ += 0;
    hit.positionX = pX;
    hit.positionY = pY;
    hit.positionZ = pZ;
    hit.fraction = t;
    return true;
  }
}
