// 移植自 OimoPhysics src/oimo/dynamics/rigidbody/Shape.hx（MIT，见 Credits.md）
import type { Proxy } from "../../collision/broadphase/proxy.js";
import { Aabb } from "../../collision/geometry/aabb.js";
import type { Geometry } from "../../collision/geometry/geometry.js";
import { Transform, transformMul } from "../../common/transform.js";
import type { RigidBody } from "./rigid-body.js";
import type { ShapeConfig } from "./shape-config.js";
import type { DetectorResult } from "../../collision/narrowphase/detector-result.js";

/**
 * 形状：把碰撞几何体挂到刚体上，带摩擦、弹性、密度。
 * 几何体可以相对刚体重心有一个局部变换（复合形状）。
 */
export class Shape {
  /** 世界分配的编号，不在世界里时为 -1 */
  id = -1;
  prev: Shape | null = null;
  next: Shape | null = null;
  rigidBody: RigidBody | null = null;
  readonly geom: Geometry;

  /** 相对刚体的局部变换 */
  readonly localTransform = new Transform();
  /** 上一步、这一步的世界变换 */
  readonly ptransform = new Transform();
  readonly transform = new Transform();

  restitution: number;
  friction: number;
  density: number;
  /** 上一步到这一步扫过的包围盒 */
  readonly aabb = new Aabb();
  /** 宽相代理，形状加入世界后才有 */
  proxy: Proxy | null = null;
  collisionGroup: number;
  collisionMask: number;
  /**
   * 预测接触距离，单位为米。包围盒按此扩展，窄相使用两侧距离之和。
   * 未启用子步时为 0。
   */
  speculativeMargin = 0;
  /** 更新流形前的接触过滤器，供适配层剔除内部接触面。默认不过滤。 */
  contactFilter: ((shape: Shape, result: DetectorResult, first: boolean, other: Shape) => void) | null = null;
  /** 使用者的数据，内核不读 */
  userData: unknown = null;

  constructor(config: ShapeConfig) {
    this.localTransform.copyFrom(config.transform);
    this.ptransform.copyFrom(this.localTransform);
    this.transform.copyFrom(this.localTransform);
    this.restitution = config.restitution;
    this.friction = config.friction;
    this.density = config.density;
    this.geom = config.geometry;
    this.collisionGroup = config.collisionGroup;
    this.collisionMask = config.collisionMask;
  }

  /** 按刚体上一步、这一步的变换更新形状的世界变换和扫过的包围盒，并通知宽相 */
  sync(tf1: Transform, tf2: Transform): void {
    this.speculativeMargin = 0;
    transformMul(this.ptransform, this.localTransform, tf1);
    transformMul(this.transform, this.localTransform, tf2);
    const aabb = this.aabb;
    this.geom.computeAabb(aabb, this.ptransform);
    const minX = aabb.minX, minY = aabb.minY, minZ = aabb.minZ;
    const maxX = aabb.maxX, maxY = aabb.maxY, maxZ = aabb.maxZ;
    this.geom.computeAabb(aabb, this.transform);
    aabb.minX = minX < aabb.minX ? minX : aabb.minX;
    aabb.minY = minY < aabb.minY ? minY : aabb.minY;
    aabb.minZ = minZ < aabb.minZ ? minZ : aabb.minZ;
    aabb.maxX = maxX > aabb.maxX ? maxX : aabb.maxX;
    aabb.maxY = maxY > aabb.maxY ? maxY : aabb.maxY;
    aabb.maxZ = maxZ > aabb.maxZ ? maxZ : aabb.maxZ;
    if (this.proxy !== null) {
      const tf = this.transform, ptf = this.ptransform;
      this.rigidBody!.world!.broadPhase.moveProxy(this.proxy, aabb, tf.px - ptf.px, tf.py - ptf.py, tf.pz - ptf.pz);
    }
  }

  /**
   * 同步形状并扩展预测接触包围盒，每 tick 调用一次。
   * 扩展距离为 min(max, base + linearReach + angularSpeed × 半径 × t)。
   * linearReach 为线速度 × t，t 为 tick 时长。
   */
  syncSpeculative(tf1: Transform, tf2: Transform, base: number, max: number, linearReach: number, angularSpeed: number, t: number): void {
    transformMul(this.ptransform, this.localTransform, tf1);
    transformMul(this.transform, this.localTransform, tf2);
    const aabb = this.aabb;
    this.geom.computeAabb(aabb, this.transform);
    // 形状旋转半径上界为包围盒中心到刚体原点的距离加包围盒半对角线。
    const tf = this.rigidBody!.transform;
    const hX = (aabb.maxX - aabb.minX) * 0.5, hY = (aabb.maxY - aabb.minY) * 0.5, hZ = (aabb.maxZ - aabb.minZ) * 0.5;
    const cX = aabb.minX + hX - tf.px, cY = aabb.minY + hY - tf.py, cZ = aabb.minZ + hZ - tf.pz;
    const radius = Math.sqrt(cX * cX + cY * cY + cZ * cZ) + Math.sqrt(hX * hX + hY * hY + hZ * hZ);
    let margin = base + linearReach + angularSpeed * radius * t;
    if (margin > max) margin = max;
    this.speculativeMargin = margin;
    // 和这一步开始时的包围盒合并（扫过的范围），再向外扩
    let minX = aabb.minX, minY = aabb.minY, minZ = aabb.minZ, maxX = aabb.maxX, maxY = aabb.maxY, maxZ = aabb.maxZ;
    this.geom.computeAabb(aabb, this.ptransform);
    if (aabb.minX < minX) minX = aabb.minX;
    if (aabb.minY < minY) minY = aabb.minY;
    if (aabb.minZ < minZ) minZ = aabb.minZ;
    if (aabb.maxX > maxX) maxX = aabb.maxX;
    if (aabb.maxY > maxY) maxY = aabb.maxY;
    if (aabb.maxZ > maxZ) maxZ = aabb.maxZ;
    aabb.minX = minX - margin; aabb.minY = minY - margin; aabb.minZ = minZ - margin;
    aabb.maxX = maxX + margin; aabb.maxY = maxY + margin; aabb.maxZ = maxZ + margin;
    if (this.proxy !== null) {
      const ptf = this.ptransform;
      this.rigidBody!.world!.broadPhase.moveProxy(this.proxy, aabb, this.transform.px - ptf.px, this.transform.py - ptf.py, this.transform.pz - ptf.pz);
    }
  }
}
