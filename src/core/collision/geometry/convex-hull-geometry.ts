// 移植自 OimoPhysics src/oimo/collision/geometry/ConvexHullGeometry.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
// 凸包射线检测使用 GJK。
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import type { Aabb } from "./aabb.js";
import { ConvexGeometry, GeometryType } from "./geometry.js";

/**
 * 由局部顶点定义的凸包。顶点数组按引用保存，构造后视为不可变。
 * 质量属性按顶点包围盒近似。
 */
export class ConvexHullGeometry extends ConvexGeometry {
  readonly vertices: readonly Vec3[];

  constructor(vertices: readonly Vec3[]) {
    super(GeometryType.CONVEX_HULL);
    this.vertices = vertices;
    this.useGjkRayCast = true;
    this.updateMass();
  }

  updateMass(): void {
    const v = this.vertices;
    let minx = v[0].x, miny = v[0].y, minz = v[0].z;
    let maxx = v[0].x, maxy = v[0].y, maxz = v[0].z;
    for (let i = 1; i < v.length; i++) {
      const vx = v[i].x, vy = v[i].y, vz = v[i].z;
      if (vx < minx) minx = vx;
      else if (vx > maxx) maxx = vx;
      if (vy < miny) miny = vy;
      else if (vy > maxy) maxy = vy;
      if (vz < minz) minz = vz;
      else if (vz > maxz) maxz = vz;
    }
    let sizex = maxx - minx, sizey = maxy - miny, sizez = maxz - minz;
    this.volume = sizex * sizey * sizez;
    // 包围盒中心偏离原点带来的平行轴项
    const diffCog = ((minx + maxx) * (minx + maxx) + (miny + maxy) * (miny + maxy) + (minz + maxz) * (minz + maxz)) * 0.25;
    sizex = sizex * sizex * 0.25;
    sizey = sizey * sizey * 0.25;
    sizez = sizez * sizez * 0.25;
    this.ic00 = (1 / 3) * (sizey + sizez) + diffCog;
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = (1 / 3) * (sizez + sizex) + diffCog;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = (1 / 3) * (sizex + sizey) + diffCog;
  }

  computeAabb(aabb: Aabb, tf: Transform): void {
    // 所有顶点转到世界坐标取最小、最大，再向外扩 GJK 外边距
    const v = this.vertices, m = this.gjkMargin;
    let lx = v[0].x, ly = v[0].y, lz = v[0].z;
    let wx = tf.r00 * lx + tf.r01 * ly + tf.r02 * lz;
    let wy = tf.r10 * lx + tf.r11 * ly + tf.r12 * lz;
    let wz = tf.r20 * lx + tf.r21 * ly + tf.r22 * lz;
    wx += tf.px;
    wy += tf.py;
    wz += tf.pz;
    let minX = wx, minY = wy, minZ = wz, maxX = wx, maxY = wy, maxZ = wz;
    for (let i = 1; i < v.length; i++) {
      lx = v[i].x;
      ly = v[i].y;
      lz = v[i].z;
      wx = tf.r00 * lx + tf.r01 * ly + tf.r02 * lz;
      wy = tf.r10 * lx + tf.r11 * ly + tf.r12 * lz;
      wz = tf.r20 * lx + tf.r21 * ly + tf.r22 * lz;
      wx += tf.px;
      wy += tf.py;
      wz += tf.pz;
      if (!(minX < wx)) minX = wx;
      if (!(minY < wy)) minY = wy;
      if (!(minZ < wz)) minZ = wz;
      if (!(maxX > wx)) maxX = wx;
      if (!(maxY > wy)) maxY = wy;
      if (!(maxZ > wz)) maxZ = wz;
    }
    aabb.minX = minX - m;
    aabb.minY = minY - m;
    aabb.minZ = minZ - m;
    aabb.maxX = maxX + m;
    aabb.maxY = maxY + m;
    aabb.maxZ = maxZ + m;
  }

  computeLocalSupportingVertex(dir: Vec3, out: Vec3): void {
    // 和 dir 点积最大的顶点；并列时取靠前的
    const v = this.vertices;
    let maxDot = v[0].x * dir.x + v[0].y * dir.y + v[0].z * dir.z;
    let maxIndex = 0;
    for (let i = 1; i < v.length; i++) {
      const dot = v[i].x * dir.x + v[i].y * dir.y + v[i].z * dir.z;
      if (dot > maxDot) {
        maxDot = dot;
        maxIndex = i;
      }
    }
    out.copyFrom(v[maxIndex]);
  }

  /** 射线检测由 GJK 处理，此解析接口返回 false。 */
  rayCastLocal(): boolean {
    return false;
  }
}
