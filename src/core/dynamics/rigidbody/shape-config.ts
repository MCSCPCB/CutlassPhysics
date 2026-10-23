// 移植自 OimoPhysics src/oimo/dynamics/rigidbody/ShapeConfig.hx（MIT，见 Credits.md）
import type { Geometry } from "../../collision/geometry/geometry.js";
import { Transform } from "../../common/transform.js";
import {
  DEFAULT_COLLISION_GROUP,
  DEFAULT_COLLISION_MASK,
  DEFAULT_DENSITY,
  DEFAULT_FRICTION,
  DEFAULT_RESTITUTION,
} from "../../settings.js";

/** 创建形状用的配置，可以重复使用 */
export class ShapeConfig {
  /** 相对刚体重心的局部位置和旋转 */
  readonly transform = new Transform();
  friction = DEFAULT_FRICTION;
  restitution = DEFAULT_RESTITUTION;
  /** 密度（kg/m³） */
  density = DEFAULT_DENSITY;
  /** 两个形状只有在 (group1 & mask2) 和 (group2 & mask1) 都不为 0 时才会碰撞 */
  collisionGroup = DEFAULT_COLLISION_GROUP;
  collisionMask = DEFAULT_COLLISION_MASK;
  geometry: Geometry;

  constructor(geometry: Geometry) {
    this.geometry = geometry;
  }
}
