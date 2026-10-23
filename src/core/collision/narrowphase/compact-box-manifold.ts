/** 盒子局部坐标中的接触中点、分离距离和法线；每次检测覆盖结果。 */
export const manifoldPoints = new Array<number>(24).fill(0);
export const manifoldSeparations = new Array<number>(8).fill(0);
export const manifoldNormal = new Array<number>(3).fill(0);
const clipped = new Array<number>(48).fill(0), scratch = new Array<number>(48).fill(0);
const transpose = new Array<number>(9).fill(0), candidates = new Array<number>(48).fill(0);
const gaps = new Array<number>(16).fill(0), selected = new Array<number>(4).fill(0);
let faceSign = 1;

function clip(input: number[], count: number, output: number[], axis: number, sign: number, limit: number): number {
  if (count === 0) return 0;
  let result = 0;
  let px = input[(count - 1) * 3], py = input[(count - 1) * 3 + 1], pz = input[(count - 1) * 3 + 2];
  let previous = sign * input[(count - 1) * 3 + axis] - limit;
  for (let i = 0; i < count; i++) {
    const x = input[i * 3], y = input[i * 3 + 1], z = input[i * 3 + 2], distance = sign * input[i * 3 + axis] - limit;
    if (distance <= 0) {
      if (previous > 0) {
        const t = previous / (previous - distance);
        output[result * 3] = px + (x - px) * t; output[result * 3 + 1] = py + (y - py) * t; output[result * 3 + 2] = pz + (z - pz) * t;
        result++;
      }
      output[result * 3] = x; output[result * 3 + 1] = y; output[result * 3 + 2] = z;
      result++;
    } else if (previous <= 0) {
      const t = previous / (previous - distance);
      output[result * 3] = px + (x - px) * t; output[result * 3 + 1] = py + (y - py) * t; output[result * 3 + 2] = pz + (z - pz) * t;
      result++;
    }
    px = x; py = y; pz = z; previous = distance;
  }
  return result;
}

function face(ax: number, ay: number, az: number, x: number, y: number, z: number, r: number[], bx: number, by: number, bz: number, axis: number, margin: number): number {
  const sign = (axis === 0 ? x : axis === 1 ? y : z) >= 0 ? 1 : -1;
  faceSign = sign;
  const half = axis === 0 ? ax : axis === 1 ? ay : az, row = axis * 3;
  const rx = Math.abs(r[row]), ry = Math.abs(r[row + 1]), rz = Math.abs(r[row + 2]);
  let incident = 0;
  if (ry > rx && ry >= rz) incident = 1;
  else if (rz > rx && rz > ry) incident = 2;
  const direction = sign * r[row + incident] > 0 ? -1 : 1;
  const depth = incident === 0 ? bx : incident === 1 ? by : bz;
  const u = incident === 0 ? 1 : 0, v = incident === 2 ? 1 : 2;
  const hu = u === 0 ? bx : by, hv = v === 1 ? by : bz;
  const cx = x + direction * depth * r[incident], cy = y + direction * depth * r[3 + incident], cz = z + direction * depth * r[6 + incident];
  const ux = r[u] * hu, uy = r[3 + u] * hu, uz = r[6 + u] * hu;
  const vx = r[v] * hv, vy = r[3 + v] * hv, vz = r[6 + v] * hv;
  clipped[0] = cx + ux + vx; clipped[1] = cy + uy + vy; clipped[2] = cz + uz + vz;
  clipped[3] = cx - ux + vx; clipped[4] = cy - uy + vy; clipped[5] = cz - uz + vz;
  clipped[6] = cx - ux - vx; clipped[7] = cy - uy - vy; clipped[8] = cz - uz - vz;
  clipped[9] = cx + ux - vx; clipped[10] = cy + uy - vy; clipped[11] = cz + uz - vz;
  const a = axis === 0 ? 1 : 0, b = axis === 2 ? 1 : 2;
  const ha = a === 0 ? ax : ay, hb = b === 1 ? ay : az;
  let count = clip(clipped, 4, scratch, a, 1, ha);
  count = clip(scratch, count, clipped, a, -1, ha);
  count = clip(clipped, count, scratch, b, 1, hb);
  count = clip(scratch, count, clipped, b, -1, hb);
  let result = 0;
  for (let i = 0; i < count; i++) {
    const gap = sign * clipped[i * 3 + axis] - half;
    if (gap > margin) continue;
    candidates[result * 3] = clipped[i * 3]; candidates[result * 3 + 1] = clipped[i * 3 + 1]; candidates[result * 3 + 2] = clipped[i * 3 + 2];
    candidates[result * 3 + axis] -= sign * gap * 0.5;
    gaps[result++] = gap;
  }
  return result;
}

/** 保留最深点、最远点及对角线两侧的最大面积点，并保持裁剪顺序。 */
function reduce(count: number): number {
  if (count <= 4) return count;
  let first = 0;
  for (let i = 1; i < count; i++) if (gaps[i] < gaps[first]) first = i;
  const x = candidates[first * 3], y = candidates[first * 3 + 1], z = candidates[first * 3 + 2];
  let second = -1, maximum = -1;
  for (let i = 0; i < count; i++) {
    if (i === first) continue;
    const dx = candidates[i * 3] - x, dy = candidates[i * 3 + 1] - y, dz = candidates[i * 3 + 2] - z;
    const distance = dx * dx + dy * dy + dz * dz;
    if (distance > maximum) { maximum = distance; second = i; }
  }
  const dx = candidates[second * 3] - x, dy = candidates[second * 3 + 1] - y, dz = candidates[second * 3 + 2] - z;
  let third = -1, nx = 0, ny = 0, nz = 0;
  maximum = -1;
  for (let i = 0; i < count; i++) {
    if (i === first || i === second) continue;
    const px = candidates[i * 3] - x, py = candidates[i * 3 + 1] - y, pz = candidates[i * 3 + 2] - z;
    const cx = dy * pz - dz * py, cy = dz * px - dx * pz, cz = dx * py - dy * px;
    const area = cx * cx + cy * cy + cz * cz;
    if (area > maximum) { maximum = area; third = i; nx = cx; ny = cy; nz = cz; }
  }
  let fourth = -1;
  maximum = 0;
  for (let i = 0; i < count; i++) {
    if (i === first || i === second || i === third) continue;
    const px = candidates[i * 3] - x, py = candidates[i * 3 + 1] - y, pz = candidates[i * 3 + 2] - z;
    const cx = dy * pz - dz * py, cy = dz * px - dx * pz, cz = dx * py - dy * px;
    const area = -(cx * nx + cy * ny + cz * nz);
    if (area > maximum) { maximum = area; fourth = i; }
  }
  const retained = fourth >= 0 ? 4 : 3;
  selected[0] = first; selected[1] = second; selected[2] = third; selected[3] = fourth;
  for (let i = 1; i < retained; i++) {
    const value = selected[i];
    let j = i;
    while (j > 0 && selected[j - 1] > value) { selected[j] = selected[j - 1]; j--; }
    selected[j] = value;
  }
  for (let i = 0; i < retained; i++) {
    const j = selected[i];
    if (i === j) continue;
    candidates[i * 3] = candidates[j * 3]; candidates[i * 3 + 1] = candidates[j * 3 + 1]; candidates[i * 3 + 2] = candidates[j * 3 + 2]; gaps[i] = gaps[j];
  }
  return retained;
}

/** 在第一个盒子的局部坐标中执行 SAT 检测和面裁剪。r 为第二个盒子的相对旋转。 */
export function boxManifold(ax: number, ay: number, az: number, x: number, y: number, z: number, r: number[], bx: number, by: number, bz: number, margin: number): number {
  const r00 = r[0], r01 = r[1], r02 = r[2], r10 = r[3], r11 = r[4], r12 = r[5], r20 = r[6], r21 = r[7], r22 = r[8];
  const a00 = Math.abs(r00), a01 = Math.abs(r01), a02 = Math.abs(r02), a10 = Math.abs(r10), a11 = Math.abs(r11), a12 = Math.abs(r12), a20 = Math.abs(r20), a21 = Math.abs(r21), a22 = Math.abs(r22);
  let gap = Math.abs(x) - (ax + bx * a00 + by * a01 + bz * a02);
  if (gap > margin) return 0;
  let faceGap = gap, axis = 0;
  gap = Math.abs(y) - (ay + bx * a10 + by * a11 + bz * a12);
  if (gap > margin) return 0;
  if (gap > faceGap) { faceGap = gap; axis = 1; }
  gap = Math.abs(z) - (az + bx * a20 + by * a21 + bz * a22);
  if (gap > margin) return 0;
  if (gap > faceGap) { faceGap = gap; axis = 2; }
  const epsilon = 1e-12;
  const row0 = a00 > 1 - epsilon ? a01 < epsilon && a02 < epsilon ? 0 : -1 : a01 > 1 - epsilon ? a00 < epsilon && a02 < epsilon ? 1 : -1 : a02 > 1 - epsilon && a00 < epsilon && a01 < epsilon ? 2 : -1;
  let row1 = -1, row2 = -1;
  if (row0 >= 0) {
    row1 = a10 > 1 - epsilon ? a11 < epsilon && a12 < epsilon ? 0 : -1 : a11 > 1 - epsilon ? a10 < epsilon && a12 < epsilon ? 1 : -1 : a12 > 1 - epsilon && a10 < epsilon && a11 < epsilon ? 2 : -1;
    if (row1 >= 0 && row1 !== row0) row2 = a20 > 1 - epsilon ? a21 < epsilon && a22 < epsilon ? 0 : -1 : a21 > 1 - epsilon ? a20 < epsilon && a22 < epsilon ? 1 : -1 : a22 > 1 - epsilon && a20 < epsilon && a21 < epsilon ? 2 : -1;
  }
  const aligned = row2 >= 0 && row2 !== row0 && row2 !== row1;
  let px = 0, py = 0, pz = 0, edgeGap = -Infinity, edgeA = -1, edgeB = -1, nx = 0, ny = 0, nz = 0;
  if (!aligned) {
    px = x * r00 + y * r10 + z * r20; py = x * r01 + y * r11 + z * r21; pz = x * r02 + y * r12 + z * r22;
    gap = Math.abs(px) - (bx + ax * a00 + ay * a10 + az * a20);
    if (gap > margin) return 0;
    if (gap > faceGap) { faceGap = gap; axis = 3; }
    gap = Math.abs(py) - (by + ax * a01 + ay * a11 + az * a21);
    if (gap > margin) return 0;
    if (gap > faceGap) { faceGap = gap; axis = 4; }
    gap = Math.abs(pz) - (bz + ax * a02 + ay * a12 + az * a22);
    if (gap > margin) return 0;
    if (gap > faceGap) { faceGap = gap; axis = 5; }
    for (let i = 0; i < 3; i++) {
      const rx = Math.abs(r[i * 3]), ry = Math.abs(r[i * 3 + 1]), rz = Math.abs(r[i * 3 + 2]);
      for (let j = 0; j < 3; j++) {
        const ux = r[j], uy = r[3 + j], uz = r[6 + j];
        const cx = i === 0 ? 0 : i === 1 ? uz : -uy;
        const cy = i === 0 ? -uz : i === 1 ? 0 : ux;
        const cz = i === 0 ? uy : i === 1 ? -ux : 0;
        const lengthSq = cx * cx + cy * cy + cz * cz;
        if (lengthSq < 1e-6) continue;
        const projection = x * cx + y * cy + z * cz;
        const extentA = ax * Math.abs(cx) + ay * Math.abs(cy) + az * Math.abs(cz);
        const extentB = j === 0 ? by * rz + bz * ry : j === 1 ? bx * rz + bz * rx : bx * ry + by * rx;
        const inverse = 1 / Math.sqrt(lengthSq);
        gap = (Math.abs(projection) - extentA - extentB) * inverse;
        if (gap > margin) return 0;
        if (gap > edgeGap) {
          edgeGap = gap; edgeA = i; edgeB = j;
          const sign = projection >= 0 ? inverse : -inverse;
          nx = cx * sign; ny = cy * sign; nz = cz * sign;
        }
      }
    }
  }
  // 接近等深时保留面接触，避免细小转角切换到单点支撑。
  if (edgeA >= 0 && edgeGap > faceGap + 0.004 + 0.05 * Math.abs(faceGap)) {
    let sx = nx >= 0 ? ax : -ax, sy = ny >= 0 ? ay : -ay, sz = nz >= 0 ? az : -az;
    if (edgeA === 0) sx = 0; else if (edgeA === 1) sy = 0; else sz = 0;
    let tx = x, ty = y, tz = z;
    for (let j = 0; j < 3; j++) {
      if (j === edgeB) continue;
      const ux = r[j], uy = r[3 + j], uz = r[6 + j], half = j === 0 ? bx : j === 1 ? by : bz;
      const sign = nx * ux + ny * uy + nz * uz >= 0 ? -half : half;
      tx += ux * sign; ty += uy * sign; tz += uz * sign;
    }
    const ux = r[edgeB], uy = r[3 + edgeB], uz = r[6 + edgeB], dx = sx - tx, dy = sy - ty, dz = sz - tz;
    const dot = edgeA === 0 ? ux : edgeA === 1 ? uy : uz;
    const da = edgeA === 0 ? dx : edgeA === 1 ? dy : dz, db = ux * dx + uy * dy + uz * dz, divisor = 1 - dot * dot;
    let ta = 0, tb = 0;
    if (divisor > 1e-9) { ta = (dot * db - da) / divisor; tb = (db - dot * da) / divisor; }
    const ha = edgeA === 0 ? ax : edgeA === 1 ? ay : az, hb = edgeB === 0 ? bx : edgeB === 1 ? by : bz;
    ta = Math.max(-ha, Math.min(ha, ta)); tb = Math.max(-hb, Math.min(hb, tb));
    if (edgeA === 0) sx = ta; else if (edgeA === 1) sy = ta; else sz = ta;
    tx += ux * tb; ty += uy * tb; tz += uz * tb;
    manifoldNormal[0] = nx; manifoldNormal[1] = ny; manifoldNormal[2] = nz;
    manifoldPoints[0] = (sx + tx) * 0.5; manifoldPoints[1] = (sy + ty) * 0.5; manifoldPoints[2] = (sz + tz) * 0.5;
    manifoldSeparations[0] = edgeGap;
    return 1;
  }
  if (axis < 3) {
    const count = reduce(face(ax, ay, az, x, y, z, r, bx, by, bz, axis, margin));
    manifoldNormal[0] = 0; manifoldNormal[1] = 0; manifoldNormal[2] = 0; manifoldNormal[axis] = faceSign;
    for (let i = 0; i < count; i++) {
      manifoldPoints[i * 3] = candidates[i * 3]; manifoldPoints[i * 3 + 1] = candidates[i * 3 + 1]; manifoldPoints[i * 3 + 2] = candidates[i * 3 + 2]; manifoldSeparations[i] = gaps[i];
    }
    return count;
  }
  const column = axis - 3;
  transpose[0] = r00; transpose[1] = r10; transpose[2] = r20; transpose[3] = r01; transpose[4] = r11; transpose[5] = r21; transpose[6] = r02; transpose[7] = r12; transpose[8] = r22;
  const count = reduce(face(bx, by, bz, -px, -py, -pz, transpose, ax, ay, az, column, margin)), sign = -faceSign;
  manifoldNormal[0] = sign * r[column]; manifoldNormal[1] = sign * r[3 + column]; manifoldNormal[2] = sign * r[6 + column];
  for (let i = 0; i < count; i++) {
    const cx = candidates[i * 3], cy = candidates[i * 3 + 1], cz = candidates[i * 3 + 2];
    manifoldPoints[i * 3] = r00 * cx + r01 * cy + r02 * cz + x;
    manifoldPoints[i * 3 + 1] = r10 * cx + r11 * cy + r12 * cz + y;
    manifoldPoints[i * 3 + 2] = r20 * cx + r21 * cy + r22 * cz + z;
    manifoldSeparations[i] = gaps[i];
  }
  return count;
}
