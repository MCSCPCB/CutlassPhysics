// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/gjkepa/SimplexUtil.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
//
// 把原点投影到单纯形（线段、三角形、四面体）上，求离原点最近的点，并返回它所在的 Voronoi 区域。
// 区域用位掩码表示：第 i 位为 1 表示最近点用到了第 i 个顶点（例如 0b011 = 前两个顶点组成的边）。
// out 必须与输入向量不同；标量运算顺序与上游一致。
import type { Vec3 } from "../../../common/vec3.js";

/** 线段 (vec1, vec2) 上离原点最近的点 */
export function projectOrigin2(vec1: Vec3, vec2: Vec3, out: Vec3): number {
  const v1X = vec1.x, v1Y = vec1.y, v1Z = vec1.z;
  const v2X = vec2.x, v2Y = vec2.y, v2Z = vec2.z;
  const v12X = v2X - v1X, v12Y = v2Y - v1Y, v12Z = v2Z - v1Z;
  const d = v12X * v12X + v12Y * v12Y + v12Z * v12Z;
  let t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
  t = -t / d;
  if (t < 0) {
    out.init(v1X, v1Y, v1Z);
    return 1;
  }
  if (t > 1) {
    out.init(v2X, v2Y, v2Z);
    return 2;
  }
  out.init(v1X + v12X * t, v1Y + v12Y * t, v1Z + v12Z * t);
  return 3;
}

/** 三角形 (vec1, vec2, vec3) 上离原点最近的点 */
export function projectOrigin3(vec1: Vec3, vec2: Vec3, vec3: Vec3, out: Vec3): number {
  const v1X = vec1.x, v1Y = vec1.y, v1Z = vec1.z;
  const v2X = vec2.x, v2Y = vec2.y, v2Z = vec2.z;
  const v3X = vec3.x, v3Y = vec3.y, v3Z = vec3.z;
  const v12X = v2X - v1X, v12Y = v2Y - v1Y, v12Z = v2Z - v1Z;
  const v23X = v3X - v2X, v23Y = v3Y - v2Y, v23Z = v3Z - v2Z;
  const v31X = v1X - v3X, v31Y = v1Y - v3Y, v31Z = v1Z - v3Z;
  // 三角形法线，以及三条边在三角形平面内朝外的法线
  let nX = v12Y * v23Z - v12Z * v23Y;
  let nY = v12Z * v23X - v12X * v23Z;
  let nZ = v12X * v23Y - v12Y * v23X;
  const n12X = v12Y * nZ - v12Z * nY, n12Y = v12Z * nX - v12X * nZ, n12Z = v12X * nY - v12Y * nX;
  const n23X = v23Y * nZ - v23Z * nY, n23Y = v23Z * nX - v23X * nZ, n23Z = v23X * nY - v23Y * nX;
  const n31X = v31Y * nZ - v31Z * nY, n31Y = v31Z * nX - v31X * nZ, n31Z = v31X * nY - v31Y * nX;
  const d12 = v1X * n12X + v1Y * n12Y + v1Z * n12Z;
  const d23 = v2X * n23X + v2Y * n23Y + v2Z * n23Z;
  const d31 = v3X * n31X + v3Y * n31Y + v3Z * n31Z;

  // 原点在某条边外侧：投影到那条边上，取最近的
  let mind = -1;
  let minvX = 0, minvY = 0, minvZ = 0;
  let mini = 0;
  if (d12 < 0) {
    const b = projectOrigin2(vec1, vec2, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    mini = b;
    mind = d;
    minvX = out.x; minvY = out.y; minvZ = out.z;
  }
  if (d23 < 0) {
    const b = projectOrigin2(vec2, vec3, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = b << 1;
      mind = d;
      minvX = out.x; minvY = out.y; minvZ = out.z;
    }
  }
  if (d31 < 0) {
    const b = projectOrigin2(vec1, vec3, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = (b & 1) | ((b & 2) << 1);
      mind = d;
      minvX = out.x; minvY = out.y; minvZ = out.z;
    }
  }
  if (mind > 0) {
    out.init(minvX, minvY, minvZ);
    return mini;
  }

  // 原点投影落在三角形内部：投影到平面上
  let l = nX * nX + nY * nY + nZ * nZ;
  if (l > 0) l = 1 / Math.sqrt(l);
  nX = nX * l;
  nY = nY * l;
  nZ = nZ * l;
  const dn = v1X * nX + v1Y * nY + v1Z * nZ;
  let l2 = nX * nX + nY * nY + nZ * nZ;
  l2 = dn / l2;
  out.init(nX * l2, nY * l2, nZ * l2);
  return 7;
}

/** 四面体 (vec1, vec2, vec3, vec4) 上离原点最近的点；原点在内部时返回零向量和 15 */
export function projectOrigin4(vec1: Vec3, vec2: Vec3, vec3: Vec3, vec4: Vec3, out: Vec3): number {
  const v1X = vec1.x, v1Y = vec1.y, v1Z = vec1.z;
  const v2X = vec2.x, v2Y = vec2.y, v2Z = vec2.z;
  const v3X = vec3.x, v3Y = vec3.y, v3Z = vec3.z;
  const v4X = vec4.x, v4Y = vec4.y, v4Z = vec4.z;
  const v12X = v2X - v1X, v12Y = v2Y - v1Y, v12Z = v2Z - v1Z;
  const v13X = v3X - v1X, v13Y = v3Y - v1Y, v13Z = v3Z - v1Z;
  const v14X = v4X - v1X, v14Y = v4Y - v1Y, v14Z = v4Z - v1Z;
  const v23X = v3X - v2X, v23Y = v3Y - v2Y, v23Z = v3Z - v2Z;
  const v24X = v4X - v2X, v24Y = v4Y - v2Y, v24Z = v4Z - v2Z;
  // 四个面的法线
  const n123X = v12Y * v13Z - v12Z * v13Y, n123Y = v12Z * v13X - v12X * v13Z, n123Z = v12X * v13Y - v12Y * v13X;
  const n134X = v13Y * v14Z - v13Z * v14Y, n134Y = v13Z * v14X - v13X * v14Z, n134Z = v13X * v14Y - v13Y * v14X;
  const n142X = v14Y * v12Z - v14Z * v12Y, n142Y = v14Z * v12X - v14X * v12Z, n142Z = v14X * v12Y - v14Y * v12X;
  const n243X = v24Y * v23Z - v24Z * v23Y, n243Y = v24Z * v23X - v24X * v23Z, n243Z = v24X * v23Y - v24Y * v23X;
  // 四面体的朝向（法线朝里还是朝外）
  const sign = v12X * n243X + v12Y * n243Y + v12Z * n243Z > 0 ? 1 : -1;
  const d123 = v1X * n123X + v1Y * n123Y + v1Z * n123Z;
  const d134 = v1X * n134X + v1Y * n134Y + v1Z * n134Z;
  const d142 = v1X * n142X + v1Y * n142Y + v1Z * n142Z;
  const d243 = v2X * n243X + v2Y * n243Y + v2Z * n243Z;

  // 原点在某个面外侧：投影到那个面上，取最近的
  let mind = -1;
  let minvX = 0, minvY = 0, minvZ = 0;
  let mini = 0;
  if (d123 * sign < 0) {
    const b = projectOrigin3(vec1, vec2, vec3, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    mini = b;
    mind = d;
    minvX = out.x; minvY = out.y; minvZ = out.z;
  }
  if (d134 * sign < 0) {
    const b = projectOrigin3(vec1, vec3, vec4, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = (b & 1) | ((b & 6) << 1);
      mind = d;
      minvX = out.x; minvY = out.y; minvZ = out.z;
    }
  }
  if (d142 * sign < 0) {
    const b = projectOrigin3(vec1, vec2, vec4, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = (b & 3) | ((b & 4) << 1);
      mind = d;
      minvX = out.x; minvY = out.y; minvZ = out.z;
    }
  }
  if (d243 * sign < 0) {
    const b = projectOrigin3(vec2, vec3, vec4, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = b << 1;
      mind = d;
      minvX = out.x; minvY = out.y; minvZ = out.z;
    }
  }
  if (mind > 0) {
    out.init(minvX, minvY, minvZ);
    return mini;
  }
  out.zero();
  return 15;
}
