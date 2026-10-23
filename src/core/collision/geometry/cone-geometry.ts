// 移植自 OimoPhysics src/oimo/collision/geometry/ConeGeometry.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import type { Aabb } from "./aabb.js";
import { ConvexGeometry, GeometryType } from "./geometry.js";
import type { RayCastHit } from "./ray-cast-hit.js";

/** 圆锥：轴沿局部 y 轴，底面在 y = -halfHeight（半径 radius），尖端在 y = +halfHeight */
export class ConeGeometry extends ConvexGeometry {
  readonly radius: number;
  readonly halfHeight: number;
  /** 半顶角的正弦、余弦 */
  readonly sinTheta: number;
  readonly cosTheta: number;

  constructor(radius: number, halfHeight: number) {
    super(GeometryType.CONE);
    this.radius = radius;
    this.halfHeight = halfHeight;
    this.sinTheta = radius / Math.sqrt(radius * radius + 4 * halfHeight * halfHeight);
    this.cosTheta = (2 * halfHeight) / Math.sqrt(radius * radius + 4 * halfHeight * halfHeight);
    this.updateMass();
  }

  updateMass(): void {
    const r2 = this.radius * this.radius;
    const h2 = this.halfHeight * this.halfHeight * 4;
    // π 用原版截断的常数
    this.volume = (3.14159265358979 * r2 * this.halfHeight * 2) / 3;
    this.ic00 = 0.05 * (3 * r2 + 2 * h2);
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 0.3 * r2;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = 0.05 * (3 * r2 + 2 * h2);
  }

  computeAabb(aabb: Aabb, tf: Transform): void {
    // 圆锥上下不对称：底面圆在 -axis * 半高处，尖端在 +axis * 半高处，分别取最小、最大
    const ax = tf.r01, ay = tf.r11, az = tf.r21;
    const ax2 = ax * ax, ay2 = ay * ay, az2 = az * az;
    let erX = Math.sqrt(1 - ax2), erY = Math.sqrt(1 - ay2), erZ = Math.sqrt(1 - az2);
    erX *= this.radius;
    erY *= this.radius;
    erZ *= this.radius;
    const ehX = ax * this.halfHeight, ehY = ay * this.halfHeight, ehZ = az * this.halfHeight;
    let rminX = -ehX, rminY = -ehY, rminZ = -ehZ;
    rminX -= erX;
    rminY -= erY;
    rminZ -= erZ;
    let rmaxX = -ehX, rmaxY = -ehY, rmaxZ = -ehZ;
    rmaxX += erX;
    rmaxY += erY;
    rmaxZ += erZ;
    let maxX = rminX > rmaxX ? rminX : rmaxX;
    let maxY = rminY > rmaxY ? rminY : rmaxY;
    let maxZ = rminZ > rmaxZ ? rminZ : rmaxZ;
    if (!(maxX > ehX)) maxX = ehX;
    if (!(maxY > ehY)) maxY = ehY;
    if (!(maxZ > ehZ)) maxZ = ehZ;
    let minX = rminX < rmaxX ? rminX : rmaxX;
    let minY = rminY < rmaxY ? rminY : rmaxY;
    let minZ = rminZ < rmaxZ ? rminZ : rmaxZ;
    if (!(minX < ehX)) minX = ehX;
    if (!(minY < ehY)) minY = ehY;
    if (!(minZ < ehZ)) minZ = ehZ;
    aabb.minX = tf.px + minX;
    aabb.minY = tf.py + minY;
    aabb.minZ = tf.pz + minZ;
    aabb.maxX = tf.px + maxX;
    aabb.maxY = tf.py + maxY;
    aabb.maxZ = tf.pz + maxZ;
  }

  computeLocalSupportingVertex(dir: Vec3, out: Vec3): void {
    const dx = dir.x, dy = dir.y, dz = dir.z;
    // dir 落在尖端的法锥里：取（缩进外边距后的）尖端
    if (dy > 0 && dy * dy > this.sinTheta * this.sinTheta * (dx * dx + dy * dy + dz * dz)) {
      out.init(0, this.halfHeight - this.gjkMargin / this.sinTheta, 0);
      if (out.y < 0) out.y = 0;
      return;
    }
    // 否则取（缩进外边距后的）底面圆周上的点
    const rx = dir.x, rz = dir.z;
    const len = rx * rx + rz * rz;
    const height = 2 * this.halfHeight;
    let coreRadius = ((height - this.gjkMargin) / height) * this.radius - this.gjkMargin / this.cosTheta;
    if (coreRadius < 0) coreRadius = 0;
    const invLen = len > 0 ? coreRadius / Math.sqrt(len) : 0;
    let coreHalfHeight = this.halfHeight - this.gjkMargin;
    if (coreHalfHeight < 0) coreHalfHeight = 0;
    out.x = rx * invLen;
    out.y = -coreHalfHeight;
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
    // 先和底面、尖端所在的两个平面之间的板块求交（y 方向）
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
    // 再和无限长的圆锥面求交。坐标先平移到尖端为原点：cos²θ·|p|² = p.y²
    let tminxz: number;
    let tmaxxz: number;
    let p1y = beginY - halfH;
    const cos2 = this.cosTheta * this.cosTheta;
    const a = cos2 * (dx * dx + dy * dy + dz * dz) - dy * dy;
    const b = cos2 * (beginX * dx + p1y * dy + beginZ * dz) - p1y * dy;
    const c = cos2 * (beginX * beginX + p1y * p1y + beginZ * beginZ) - p1y * p1y;
    const D = b * b - a * c;
    if (a !== 0) {
      if (D < 0) return false;
      const sqrtD = Math.sqrt(D);
      if (a < 0) {
        // 线段比圆锥面陡：区间是一段射线
        if (dy > 0) {
          tminxz = 0;
          tmaxxz = (-b + sqrtD) / a;
          if (tmaxxz <= 0) return false;
        } else {
          tminxz = (-b - sqrtD) / a;
          tmaxxz = 1;
          if (tminxz >= 1) return false;
        }
      } else {
        tminxz = (-b - sqrtD) / a;
        tmaxxz = (-b + sqrtD) / a;
        if (tminxz >= 1 || tmaxxz <= 0) return false;
      }
    } else {
      // 线段和圆锥面平行，方程退化成一次
      const t = -c / (2 * b);
      if (b > 0) {
        tminxz = 0;
        tmaxxz = t;
        if (t <= 0) return false;
      } else {
        tminxz = t;
        tmaxxz = 1;
        if (t >= 1) return false;
      }
    }
    // 交点转换回局部坐标，保留上游运算顺序。
    p1y += halfH;
    if (tmaxxz <= tminy || tmaxy <= tminxz) return false;
    let min: number;
    if (tminxz < tminy) {
      // 先碰到底面（或尖端所在的平面）
      min = tminy;
      if (min === 0) return false;
      hit.normalX = 0;
      hit.normalY = dy > 0 ? -1 : 1;
      hit.normalZ = 0;
    } else {
      // 先碰到侧面：法线 = 水平方向 * cosθ + y 轴 * sinθ
      min = tminxz;
      if (min === 0) return false;
      let nx = beginX + dx * min, ny = 0, nz = beginZ + dz * min;
      let invLen = Math.sqrt(nx * nx + ny * ny + nz * nz);
      if (invLen > 0) invLen = 1 / invLen;
      nx *= invLen;
      ny *= invLen;
      nz *= invLen;
      const s = this.cosTheta;
      nx *= s;
      ny *= s;
      nz *= s;
      ny += this.sinTheta;
      hit.normalX = nx;
      hit.normalY = ny;
      hit.normalZ = nz;
    }
    hit.positionX = beginX + min * dx;
    hit.positionY = p1y + min * dy;
    hit.positionZ = beginZ + min * dz;
    hit.fraction = min;
    return true;
  }
}
