// 悬浮材质提供重力补偿和随速度变化的阻力。
// 每种悬浮材质的方块合成一簇：簇的中心是方块的加权平均位置，协方差描述方块的分布（含每个方块自身的二阶矩 1/6）。
// 阻力按「慢速」「快速」两组系数，慢速部分随速度按高斯衰减（transitionSpeed 越小，越早过渡到快速阻力）。
// 坐标都在刚体局部坐标系里，以重心为原点。每个刚体每 tick 算一次，结果写进外力、外力矩，子步按常力积分。
import type { RigidBody } from "../core/dynamics/rigidbody/rigid-body.js";
import type { FloatingMaterial } from "./block-physics.js";

/** 一种悬浮材质的方块簇 */
export interface FloatingCluster {
  readonly material: FloatingMaterial;
  /** 簇的中心（刚体局部坐标，以重心为原点） */
  cx: number; cy: number; cz: number;
  /** 协方差，按行存的 3×3 对称矩阵 */
  readonly cov: Float64Array;
  /** 方块的悬浮强度之和 */
  scale: number;
}

/** 一个悬浮方块：位置是方块中心（刚体局部坐标，以重心为原点） */
export interface FloatingPoint {
  readonly x: number;
  readonly y: number;
  readonly z: number;
  readonly material: FloatingMaterial;
  readonly scale: number;
}

/** 按材质把悬浮方块分成簇 */
export function floatingClusters(points: readonly FloatingPoint[]): FloatingCluster[] {
  const groups = new Map<FloatingMaterial, FloatingCluster>();
  for (const p of points) {
    if (p.scale <= 0) continue;
    let c = groups.get(p.material);
    if (c === undefined) {
      c = { material: p.material, cx: 0, cy: 0, cz: 0, cov: new Float64Array(9), scale: 0 };
      groups.set(p.material, c);
    }
    c.scale += p.scale;
    c.cx += p.x * p.scale; c.cy += p.y * p.scale; c.cz += p.z * p.scale;
  }
  for (const c of groups.values()) { c.cx /= c.scale; c.cy /= c.scale; c.cz /= c.scale; }
  for (const p of points) {
    if (p.scale <= 0) continue;
    const c = groups.get(p.material)!;
    const o0 = p.x - c.cx, o1 = p.y - c.cy, o2 = p.z - c.cz, w = p.scale / c.scale;
    const m = c.cov;
    m[0] += o0 * o0 * w; m[1] += o0 * o1 * w; m[2] += o0 * o2 * w;
    m[3] += o1 * o0 * w; m[4] += o1 * o1 * w; m[5] += o1 * o2 * w;
    m[6] += o2 * o0 * w; m[7] += o2 * o1 * w; m[8] += o2 * o2 * w;
  }
  // 每个方块自身的二阶矩
  for (const c of groups.values()) { c.cov[0] += 1 / 6; c.cov[4] += 1 / 6; c.cov[8] += 1 / 6; }
  return [...groups.values()];
}

// 预分配的临时矩阵（3×3，按行存）
const inv = new Float64Array(9), shiftedCov = new Float64Array(9), slowDrag = new Float64Array(9), fastDrag = new Float64Array(9), torqueM = new Float64Array(9);

function det(m: Float64Array): number {
  return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
}

/** 3×3 求逆；奇异时写零矩阵 */
function invert(m: Float64Array, out: Float64Array): void {
  const a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
  const A = e * i - f * h, B = f * g - d * i, C = d * h - e * g;
  const D = det(m);
  if (D === 0) { out.fill(0); return; }
  const s = 1 / D;
  out[0] = A * s; out[1] = (c * h - b * i) * s; out[2] = (b * f - c * e) * s;
  out[3] = B * s; out[4] = (a * i - c * g) * s; out[5] = (c * d - a * f) * s;
  out[6] = C * s; out[7] = (b * g - a * h) * s; out[8] = (a * e - b * d) * s;
}

/** 阻力矩阵：沿重力方向 vertical、垂直方向 horizontal，再乘 scale（取负，阻力和速度反向） */
function dragMatrix(gx: number, gy: number, gz: number, vertical: number, horizontal: number, scale: number, out: Float64Array): void {
  const g2 = gx * gx + gy * gy + gz * gz;
  const k = g2 > 0 ? (horizontal - vertical) / g2 : 0;
  const g0 = gx, g1 = gy, g2v = gz;
  out[0] = scale * (k * g0 * g0 - horizontal); out[1] = scale * k * g0 * g1; out[2] = scale * k * g0 * g2v;
  out[3] = scale * k * g1 * g0; out[4] = scale * (k * g1 * g1 - horizontal); out[5] = scale * k * g1 * g2v;
  out[6] = scale * k * g2v * g0; out[7] = scale * k * g2v * g1; out[8] = scale * (k * g2v * g2v - horizontal);
}

/** 分布在 position（协方差）上的阻力 drag 对角速度产生的力矩矩阵，加到 out 上 */
function addDragTorque(x: Float64Array, y: Float64Array, out: Float64Array): void {
  const tx = x[0] + x[4] + x[8], ty = y[0] + y[4] + y[8];
  let txy = 0;
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) txy += x[r * 3 + c] * y[c * 3 + r];
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      let v = -tx * y[r * 3 + c] - ty * x[r * 3 + c];
      for (let k = 0; k < 3; k++) v += x[r * 3 + k] * y[k * 3 + c] + y[r * 3 + k] * x[k * 3 + c];
      out[r * 3 + c] += v + (r === c ? tx * ty - txy : 0);
    }
  }
}

/** 累加世界坐标下的悬浮力和力矩，gx..gz 为重力。休眠刚体不参与计算。 */
export function applyFloating(rb: RigidBody, clusters: readonly FloatingCluster[], gx: number, gy: number, gz: number): void {
  if (clusters.length === 0 || rb.sleeping) return;
  const t = rb.transform;
  const r00 = t.r00, r01 = t.r01, r02 = t.r02, r10 = t.r10, r11 = t.r11, r12 = t.r12, r20 = t.r20, r21 = t.r21, r22 = t.r22;
  // 世界 → 局部：乘旋转矩阵的转置
  const gs = rb.gravityScale;
  const lgx = (r00 * gx + r10 * gy + r20 * gz) * gs, lgy = (r01 * gx + r11 * gy + r21 * gz) * gs, lgz = (r02 * gx + r12 * gy + r22 * gz) * gs;
  const lvx = r00 * rb.vx + r10 * rb.vy + r20 * rb.vz, lvy = r01 * rb.vx + r11 * rb.vy + r21 * rb.vz, lvz = r02 * rb.vx + r12 * rb.vy + r22 * rb.vz;
  const wx = r00 * rb.wx + r10 * rb.wy + r20 * rb.wz, wy = r01 * rb.wx + r11 * rb.wy + r21 * rb.wz, wz = r02 * rb.wx + r12 * rb.wy + r22 * rb.wz;
  const w2 = wx * wx + wy * wy + wz * wz;
  const gLen = Math.sqrt(lgx * lgx + lgy * lgy + lgz * lgz);
  let fx = 0, fy = 0, fz = 0, tx = 0, ty = 0, tz = 0;
  let limited = 0, lcx = 0, lcy = 0, lcz = 0;
  for (let n = 0; n < clusters.length; n++) {
    const cl = clusters[n], m = cl.material;
    const cx = cl.cx, cy = cl.cy, cz = cl.cz;
    const lift = cl.scale * m.liftStrength;
    if (m.preventSelfLift) {
      limited += lift;
      lcx += cx * lift; lcy += cy * lift; lcz += cz * lift;
    } else {
      const px = -lgx * lift, py = -lgy * lift, pz = -lgz * lift;
      fx += px; fy += py; fz += pz;
      tx += cy * pz - cz * py; ty += cz * px - cx * pz; tz += cx * py - cy * px;
    }
    const speedScale = m.transitionSpeed === 0 ? 0 : 3 / (m.transitionSpeed * m.transitionSpeed);
    invert(cl.cov, inv);
    const ang = [wx, wy, wz];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) inv[r * 3 + c] += speedScale * ((r === c ? w2 : 0) - ang[r] * ang[c]);
    invert(inv, shiftedCov);
    // 簇中心的平均速度 = v + ω × c
    const mvx = wy * cz - wz * cy + lvx, mvy = wz * cx - wx * cz + lvy, mvz = wx * cy - wy * cx + lvz;
    // 偏移后的中心 = shiftedCov · (speedScale · ω × mv)
    const s0 = speedScale * (wy * mvz - wz * mvy), s1 = speedScale * (wz * mvx - wx * mvz), s2 = speedScale * (wx * mvy - wy * mvx);
    const shx = shiftedCov[0] * s0 + shiftedCov[1] * s1 + shiftedCov[2] * s2;
    const shy = shiftedCov[3] * s0 + shiftedCov[4] * s1 + shiftedCov[5] * s2;
    const shz = shiftedCov[6] * s0 + shiftedCov[7] * s1 + shiftedCov[8] * s2;
    const ix = inv[0] * shx + inv[1] * shy + inv[2] * shz, iy = inv[3] * shx + inv[4] * shy + inv[5] * shz, iz = inv[6] * shx + inv[7] * shy + inv[8] * shz;
    const slowScale = speedScale === 0 ? 0
      : Math.sqrt(det(shiftedCov) / det(cl.cov)) * Math.exp(-0.5 * (speedScale * (mvx * mvx + mvy * mvy + mvz * mvz) - (shx * ix + shy * iy + shz * iz)));
    const frictionScale = cl.scale * (m.scaleFrictionWithGravity ? gLen : 1);
    dragMatrix(lgx, lgy, lgz, m.slowVerticalFriction, m.slowHorizontalFriction, frictionScale * slowScale, slowDrag);
    dragMatrix(lgx, lgy, lgz, m.fastVerticalFriction, m.fastHorizontalFriction, frictionScale, fastDrag);
    // 慢速阻力作用在偏移后的中心上的速度 = mv + ω × shifted
    const svx = wy * shz - wz * shy + mvx, svy = wz * shx - wx * shz + mvy, svz = wx * shy - wy * shx + mvz;
    const sfx = slowDrag[0] * svx + slowDrag[1] * svy + slowDrag[2] * svz;
    const sfy = slowDrag[3] * svx + slowDrag[4] * svy + slowDrag[5] * svz;
    const sfz = slowDrag[6] * svx + slowDrag[7] * svy + slowDrag[8] * svz;
    const ffx = fastDrag[0] * mvx + fastDrag[1] * mvy + fastDrag[2] * mvz + sfx;
    const ffy = fastDrag[3] * mvx + fastDrag[4] * mvy + fastDrag[5] * mvz + sfy;
    const ffz = fastDrag[6] * mvx + fastDrag[7] * mvy + fastDrag[8] * mvz + sfz;
    let qx = cy * ffz - cz * ffy, qy = cz * ffx - cx * ffz, qz = cx * ffy - cy * ffx;
    qx += shy * sfz - shz * sfy; qy += shz * sfx - shx * sfz; qz += shx * sfy - shy * sfx;
    torqueM.fill(0);
    addDragTorque(cl.cov, fastDrag, torqueM);
    addDragTorque(shiftedCov, slowDrag, torqueM);
    qx += torqueM[0] * wx + torqueM[1] * wy + torqueM[2] * wz;
    qy += torqueM[3] * wx + torqueM[4] * wy + torqueM[5] * wz;
    qz += torqueM[6] * wx + torqueM[7] * wy + torqueM[8] * wz;
    fx += ffx; fy += ffy; fz += ffz;
    tx += qx; ty += qy; tz += qz;
  }
  if (limited > 0 && gLen > 0) {
    // 升力上限为重心沿重力方向的有效重量。
    const cx = lcx / limited, cy = lcy / limited, cz = lcz / limited;
    // 世界坐标里的力臂和重力方向
    const ax = r00 * cx + r01 * cy + r02 * cz, ay = r10 * cx + r11 * cy + r12 * cz, az = r20 * cx + r21 * cy + r22 * cz;
    const gwx = r00 * lgx + r01 * lgy + r02 * lgz, gwy = r10 * lgx + r11 * lgy + r12 * lgz, gwz = r20 * lgx + r21 * lgy + r22 * lgz;
    const ux = gwx / gLen, uy = gwy / gLen, uz = gwz / gLen;
    const kx = ay * uz - az * uy, ky = az * ux - ax * uz, kz = ax * uy - ay * ux;
    const ikx = rb.ii00 * kx + rb.ii01 * ky + rb.ii02 * kz, iky = rb.ii10 * kx + rb.ii11 * ky + rb.ii12 * kz, ikz = rb.ii20 * kx + rb.ii21 * ky + rb.ii22 * kz;
    const effective = 1 / (rb.invMass + kx * ikx + ky * iky + kz * ikz);
    const amount = Math.min(limited, effective);
    const px = -lgx * amount, py = -lgy * amount, pz = -lgz * amount;
    fx += px; fy += py; fz += pz;
    tx += cy * pz - cz * py; ty += cz * px - cx * pz; tz += cx * py - cy * px;
  }
  // 局部 → 世界
  rb.fx += r00 * fx + r01 * fy + r02 * fz; rb.fy += r10 * fx + r11 * fy + r12 * fz; rb.fz += r20 * fx + r21 * fy + r22 * fz;
  rb.tx += r00 * tx + r01 * ty + r02 * tz; rb.ty += r10 * tx + r11 * ty + r12 * tz; rb.tz += r20 * tx + r21 * ty + r22 * tz;
}
