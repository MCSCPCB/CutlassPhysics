import { quaternionToEuler, type Body } from "cutlass-physics";

/** 把刚体的朝向换成度，用来设置实体的旋转 */
export function rotationInDegrees(body: Body) {
  const { x: pitch, y: yaw, z: roll } = quaternionToEuler(body.rotation);
  return { pitch, yaw, roll };
}
