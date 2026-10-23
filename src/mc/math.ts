// 对外 API：向量、四元数的类型和欧拉角换算。

/**
 * 表示三维向量的接口。
 *
 * @remarks
 * 与 `@minecraft/server` 中的 `Vector3` 结构相同，两者可以直接互相传递。
 */
export interface Vector3 {
  /** X 分量。 */
  x: number;
  /** Y 分量。 */
  y: number;
  /** Z 分量。 */
  z: number;
}

/**
 * 表示朝向的四元数。
 *
 * @remarks
 *
 * - `{ x: 0, y: 0, z: 0, w: 1 }` 表示没有旋转。
 * - 读取到的四元数已经过归一化。
 * - 传入的四元数会被自动归一化；所有分量均为零时抛出错误。
 */
export interface Quaternion {
  /** X 分量。 */
  x: number;
  /** Y 分量。 */
  y: number;
  /** Z 分量。 */
  z: number;
  /** W 分量。 */
  w: number;
}

/**
 * 以度为单位的欧拉角。
 *
 * @remarks
 * `x` 为俯仰角（pitch），`y` 为偏航角（yaw），`z` 为翻滚角（roll）。旋转顺序为 YXZ，与原版实体一致。
 */
export type EulerDegrees = Vector3;

/**
 * 将欧拉角转换为四元数。
 *
 * @param euler
 * 以度为单位的欧拉角。
 * @returns
 * 经过归一化的四元数。
 */
export function eulerToQuaternion(euler: EulerDegrees): Quaternion {
  // 偏航角从上往下看顺时针为正，对应绕 +Y 轴转 -yaw；俯仰角向下为正，对应绕 +X 轴转 pitch；翻滚角绕 +Z 轴转 roll
  const hy = (-euler.y * Math.PI) / 360, hx = (euler.x * Math.PI) / 360, hz = (euler.z * Math.PI) / 360;
  const sy = Math.sin(hy), cy = Math.cos(hy), sx = Math.sin(hx), cx = Math.cos(hx), sz = Math.sin(hz), cz = Math.cos(hz);
  // q = qY · qX · qZ：先算 qY · qX，再乘 qZ
  const w1 = cy * cx, x1 = cy * sx, y1 = sy * cx, z1 = -sy * sx;
  // 末尾的 +0 将向量分量的 -0 规范化为 +0。
  return {
    x: x1 * cz + y1 * sz + 0,
    y: y1 * cz - x1 * sz + 0,
    z: w1 * sz + z1 * cz + 0,
    w: w1 * cz - z1 * sz,
  };
}

/**
 * 将四元数转换为欧拉角。
 *
 * @remarks
 * 俯仰角接近 ±90 度时发生万向节死锁，此时 z 为零，旋转由偏航角表示。
 *
 * @param rotation
 * 要转换的四元数。
 * @returns
 * 以度为单位的欧拉角。
 *
 * @example quaternion-to-euler.ts
 * {@includeCode ../../examples/display-sync/quaternion-to-euler.ts}
 */
export function quaternionToEuler(rotation: Quaternion): EulerDegrees {
  const len = Math.sqrt(rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z + rotation.w * rotation.w);
  if (!(len > 0)) throw new Error("四元数的所有分量均为零，无法转换");
  const x = rotation.x / len, y = rotation.y / len, z = rotation.z / len, w = rotation.w / len;
  // 旋转矩阵 R = Ry(-yaw) · Rx(pitch) · Rz(roll) 中用到的元素
  const r00 = 1 - 2 * (y * y + z * z), r02 = 2 * (x * z + w * y), r10 = 2 * (x * y + w * z);
  const r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x), r20 = 2 * (x * z - w * y), r22 = 1 - 2 * (x * x + y * y);
  const RAD = 180 / Math.PI;
  const sinPitch = Math.max(-1, Math.min(1, -r12));
  const pitch = Math.asin(sinPitch) * RAD + 0;
  // 俯仰角接近 ±90 度时 r02、r22、r10、r11 都接近 0，改用 r00、r20 求偏航角，翻滚角记为 0
  if (Math.abs(sinPitch) > 0.9999999) {
    return { x: pitch, y: -Math.atan2(-r20, r00) * RAD + 0, z: 0 };
  }
  return { x: pitch, y: -Math.atan2(r02, r22) * RAD + 0, z: Math.atan2(r10, r11) * RAD + 0 };
}
