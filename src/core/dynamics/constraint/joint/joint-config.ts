// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/JointConfig.hx（MIT，见 Credits.md）
import { Vec3 } from "../../../common/vec3.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";

/** 约束求解器的类型 */
export const ConstraintSolverType = {
  /** 迭代求解（PGS） */
  ITERATIVE: 0,
  /** 直接求解（更精确、更慢） */
  DIRECT: 1,
} as const;

/** 关节配置的公共部分 */
export class JointConfig {
  rigidBody1: RigidBody | null = null;
  rigidBody2: RigidBody | null = null;
  /** 锚点在两个刚体上的局部坐标 */
  readonly localAnchor1 = new Vec3();
  readonly localAnchor2 = new Vec3();
  /** 是否允许两个刚体互相碰撞 */
  allowCollision = false;
  solverType: number = ConstraintSolverType.ITERATIVE;
  /** 约束力超过它时关节断开，0 表示不会断 */
  breakForce = 0;
  /** 约束力矩超过它时关节断开，0 表示不会断 */
  breakTorque = 0;

  /** 用世界坐标的锚点设置两个刚体和局部锚点 */
  protected initBase(rb1: RigidBody, rb2: RigidBody, anchorX: number, anchorY: number, anchorZ: number): void {
    this.rigidBody1 = rb1;
    this.rigidBody2 = rb2;
    toLocalPoint(rb1, anchorX, anchorY, anchorZ, this.localAnchor1);
    toLocalPoint(rb2, anchorX, anchorY, anchorZ, this.localAnchor2);
  }
}

/** 世界坐标的方向转到刚体的局部坐标 */
export function toLocalVector(rb: RigidBody, x: number, y: number, z: number, out: Vec3): void {
  const tf = rb.transform;
  out.init(
    tf.r00 * x + tf.r10 * y + tf.r20 * z,
    tf.r01 * x + tf.r11 * y + tf.r21 * z,
    tf.r02 * x + tf.r12 * y + tf.r22 * z,
  );
}

/** 世界坐标的点转到刚体的局部坐标 */
export function toLocalPoint(rb: RigidBody, x: number, y: number, z: number, out: Vec3): void {
  const tf = rb.transform;
  const dX = x - tf.px, dY = y - tf.py, dZ = z - tf.pz;
  out.init(
    tf.r00 * dX + tf.r10 * dY + tf.r20 * dZ,
    tf.r01 * dX + tf.r11 * dY + tf.r21 * dZ,
    tf.r02 * dX + tf.r12 * dY + tf.r22 * dZ,
  );
}
