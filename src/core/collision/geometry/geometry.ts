// 移植自 OimoPhysics src/oimo/collision/geometry/{Geometry,ConvexGeometry,GeometryType}.hx（MIT，见 Credits.md）
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import { DEFAULT_GJK_MARGIN } from "../../settings.js";
import { GjkEpa } from "../narrowphase/gjkepa/gjk-epa.js";
import type { Aabb } from "./aabb.js";
import type { RayCastHit } from "./ray-cast-hit.js";

/** 几何体类型编号，和 Oimo 一致（碰撞矩阵按它查找检测器） */
export const GeometryType = {
  SPHERE: 0,
  BOX: 1,
  CYLINDER: 2,
  CONE: 3,
  CAPSULE: 4,
  CONVEX_HULL: 5,
} as const;

/** 碰撞几何体基类 */
export abstract class Geometry {
  readonly type: number;
  volume = 0;

  /** 惯性张量除以质量（I / m），局部坐标。icIJ 是第 I 行第 J 列 */
  ic00 = 0;
  ic01 = 0;
  ic02 = 0;
  ic10 = 0;
  ic11 = 0;
  ic12 = 0;
  ic20 = 0;
  ic21 = 0;
  ic22 = 0;

  protected constructor(type: number) {
    this.type = type;
  }

  /** 根据尺寸重新计算 volume 和惯性系数 */
  abstract updateMass(): void;

  /** 计算几何体在变换 tf 下的世界坐标包围盒，写入 aabb */
  abstract computeAabb(aabb: Aabb, tf: Transform): void;

  /**
   * 局部坐标下的射线检测：线段从 begin 到 end。
   * 命中时把局部坐标的结果写入 hit 并返回 true；没命中时返回 false，hit 不变。
   * 起点在几何体内部不算命中。
   */
  abstract rayCastLocal(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    hit: RayCastHit,
  ): boolean;

  /**
   * 世界坐标下的射线检测：几何体放在变换 tf 处，线段从 begin 到 end。
   * 命中时把世界坐标的结果写入 hit 并返回 true；没命中时返回 false，hit 不变。
   */
  rayCast(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    tf: Transform, hit: RayCastHit,
  ): boolean {
    // 转到局部坐标：先减位置，再乘旋转矩阵的转置
    beginX -= tf.px;
    beginY -= tf.py;
    beginZ -= tf.pz;
    endX -= tf.px;
    endY -= tf.py;
    endZ -= tf.pz;
    const blX = tf.r00 * beginX + tf.r10 * beginY + tf.r20 * beginZ;
    const blY = tf.r01 * beginX + tf.r11 * beginY + tf.r21 * beginZ;
    const blZ = tf.r02 * beginX + tf.r12 * beginY + tf.r22 * beginZ;
    const elX = tf.r00 * endX + tf.r10 * endY + tf.r20 * endZ;
    const elY = tf.r01 * endX + tf.r11 * endY + tf.r21 * endZ;
    const elZ = tf.r02 * endX + tf.r12 * endY + tf.r22 * endZ;
    if (!this.rayCastLocal(blX, blY, blZ, elX, elY, elZ, hit)) return false;
    // 结果转回世界坐标：命中点先旋转再加位置，法线只旋转
    const lpX = hit.positionX, lpY = hit.positionY, lpZ = hit.positionZ;
    const lnX = hit.normalX, lnY = hit.normalY, lnZ = hit.normalZ;
    hit.positionX = tf.r00 * lpX + tf.r01 * lpY + tf.r02 * lpZ + tf.px;
    hit.positionY = tf.r10 * lpX + tf.r11 * lpY + tf.r12 * lpZ + tf.py;
    hit.positionZ = tf.r20 * lpX + tf.r21 * lpY + tf.r22 * lpZ + tf.pz;
    hit.normalX = tf.r00 * lnX + tf.r01 * lnY + tf.r02 * lnZ;
    hit.normalY = tf.r10 * lnX + tf.r11 * lnY + tf.r12 * lnZ;
    hit.normalZ = tf.r20 * lnX + tf.r21 * lnY + tf.r22 * lnZ;
    return true;
  }
}

/** 凸几何体基类（GJK/EPA 能处理的几何体） */
export abstract class ConvexGeometry extends Geometry {
  /** GJK 外边距：几何体“核心”外面包的一层 */
  gjkMargin = DEFAULT_GJK_MARGIN;
  /** 射线检测用 GJK 做，而不是 rayCastLocal 的解析解（凸包没有解析解） */
  protected useGjkRayCast = false;

  override rayCast(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    tf: Transform, hit: RayCastHit,
  ): boolean {
    if (this.useGjkRayCast) return GjkEpa.getInstance().rayCast(this, tf, beginX, beginY, beginZ, endX, endY, endZ, hit);
    return super.rayCast(beginX, beginY, beginZ, endX, endY, endZ, tf, hit);
  }

  /**
   * 支撑点：几何体“核心”（去掉 GJK 外边距后的部分）上沿局部方向 dir 最远的点，局部坐标，写入 out。
   * dir 不要求是单位向量。
   */
  abstract computeLocalSupportingVertex(dir: Vec3, out: Vec3): void;
}
