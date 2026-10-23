// 移植自 OimoPhysics src/oimo/m/M.hx 里关节用到的宏：quat_arc、vec3_perp、mat3_fromQuat（MIT，见 THIRD-PARTY-NOTICES.txt）
// 关节初始化的数学运算，结果写入预分配对象。
// 每步都跑的 BasisTracker 里把同样的计算展开成了标量。

/** 和 v 垂直的单位向量（原版 vec3_perp）：去掉绝对值最小的分量，另外两个交换、取负再归一化 */
export const PERP = { x: 0, y: 0, z: 0 };
export function perp(x1: number, y1: number, z1: number): void {
  const x2 = x1 * x1, y2 = y1 * y1, z2 = z1 * z1;
  let d: number;
  if (x2 < y2) {
    if (x2 < z2) {
      d = 1 / Math.sqrt(y2 + z2);
      PERP.x = 0; PERP.y = z1 * d; PERP.z = -y1 * d;
    } else {
      d = 1 / Math.sqrt(x2 + y2);
      PERP.x = y1 * d; PERP.y = -x1 * d; PERP.z = 0;
    }
  } else if (y2 < z2) {
    d = 1 / Math.sqrt(z2 + x2);
    PERP.x = -z1 * d; PERP.y = 0; PERP.z = x1 * d;
  } else {
    d = 1 / Math.sqrt(x2 + y2);
    PERP.x = y1 * d; PERP.y = -x1 * d; PERP.z = 0;
  }
}

/** 把单位向量 u 转到单位向量 v 的最短旋转（原版 quat_arc），结果放在 ARC */
export const ARC = { x: 0, y: 0, z: 0, w: 0 };
export function arc(ux: number, uy: number, uz: number, vx: number, vy: number, vz: number): void {
  let d = ux * vx + uy * vy + uz * vz;
  if (d < -0.999999999) {
    // 方向相反：绕任意一个垂直轴转 180°
    perp(ux, uy, uz);
    ARC.x = PERP.x; ARC.y = PERP.y; ARC.z = PERP.z; ARC.w = 0;
  } else {
    let cX = uy * vz - uz * vy, cY = uz * vx - ux * vz, cZ = ux * vy - uy * vx;
    const w = Math.sqrt((1 + d) * 0.5);
    d = 0.5 / w;
    cX *= d; cY *= d; cZ *= d;
    ARC.x = cX; ARC.y = cY; ARC.z = cZ; ARC.w = w;
  }
}

/** 四元数转旋转矩阵（原版 mat3_fromQuat），结果放在 ROT（行优先 m00..m22） */
export const ROT = { m00: 0, m01: 0, m02: 0, m10: 0, m11: 0, m12: 0, m20: 0, m21: 0, m22: 0 };
export function mat3FromQuat(x: number, y: number, z: number, w: number): void {
  const x2 = 2 * x, y2 = 2 * y, z2 = 2 * z;
  const xx = x * x2, yy = y * y2, zz = z * z2;
  const xy = x * y2, yz = y * z2, xz = x * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  ROT.m00 = 1 - yy - zz; ROT.m01 = xy - wz; ROT.m02 = xz + wy;
  ROT.m10 = xy + wz; ROT.m11 = 1 - xx - zz; ROT.m12 = yz - wx;
  ROT.m20 = xz - wy; ROT.m21 = yz + wx; ROT.m22 = 1 - xx - yy;
}
