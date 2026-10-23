// 移植自 OimoPhysics src/oimo/dynamics/rigidbody/{RigidBodyConfig,RigidBodyType}.hx（MIT，见 Credits.md）
import { Transform } from "../../common/transform.js";
import {
  SLEEPING_ANGULAR_VELOCITY_THRESHOLD,
  SLEEPING_TIME_THRESHOLD,
  SLEEPING_VELOCITY_THRESHOLD,
} from "../../settings.js";

/** 刚体的运动类型，编号和 Oimo 一致 */
export const RigidBodyType = {
  /** 受力和碰撞影响，正常运动 */
  DYNAMIC: 0,
  /** 不动，质量视为无穷大 */
  STATIC: 1,
  /** 按设定的速度运动，不受力和碰撞影响，但会推开动态刚体 */
  KINEMATIC: 2,
} as const;

/** 创建刚体用的配置，可以重复使用 */
export class RigidBodyConfig {
  /** 重心的世界位置和旋转 */
  readonly transform = new Transform();
  vx = 0;
  vy = 0;
  vz = 0;
  /** 角速度（rad/s） */
  wx = 0;
  wy = 0;
  wz = 0;
  type: number = RigidBodyType.DYNAMIC;
  autoSleep = true;
  linearDamping = 0;
  angularDamping = 0;
  sleepingVelocityThreshold = SLEEPING_VELOCITY_THRESHOLD;
  sleepingAngularVelocityThreshold = SLEEPING_ANGULAR_VELOCITY_THRESHOLD;
  sleepingTimeThreshold = SLEEPING_TIME_THRESHOLD;
}
