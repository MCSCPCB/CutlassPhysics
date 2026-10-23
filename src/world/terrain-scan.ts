// 计算刚体当前、预读和休眠范围覆盖的 8³ 块。
// 当前范围包含平移和旋转扫掠，预读范围沿速度方向扩展。
// 休眠范围为形状包围盒外扩 margin。
// 小型形状集合的大范围扫描使用局部轴投影排除空角。
// 刚体的旋转中心是 transform 的原点，形状的局部包围盒都相对它。
import { Aabb } from "../core/collision/geometry/aabb.js";
import type { RigidBody } from "../core/dynamics/rigidbody/rigid-body.js";
import type { Shape } from "../core/dynamics/rigidbody/shape.js";
import { CHUNK_SIZE } from "./chunk-boxes.js";

/** 块坐标（方块坐标 / 8 向下取整）的范围：x、z 为 [-2^22, 2^22)，y 为 [-64, 64)。范围外当作没有方块 */
const XZ_LIMIT = 4194304;
const XZ_RANGE = 8388608;
const Y_LIMIT = 64;
const Y_RANGE = 128;
/** 当前范围每侧外扩的上限（格） */
const MAX_VELOCITY_MARGIN = 3;
/** 预读范围运动方向上额外外扩的上限（格） */
const MAX_PREFETCH_MARGIN = CHUNK_SIZE;
const SHADOW_MAX_SHAPES = 4;
const SHADOW_MIN_VOLUME = 256;

/** 块坐标转整数键（不超过 2^53）。超出范围返回 -1 */
export function chunkKey(cx: number, cy: number, cz: number): number {
  if (cx < -XZ_LIMIT || cx >= XZ_LIMIT || cz < -XZ_LIMIT || cz >= XZ_LIMIT || cy < -Y_LIMIT || cy >= Y_LIMIT) return -1;
  return ((cx + XZ_LIMIT) * XZ_RANGE + (cz + XZ_LIMIT)) * Y_RANGE + (cy + Y_LIMIT);
}

/** 键对应的块最小角（方块坐标），写进 out[0..2] */
export function chunkOrigin(key: number, out: number[]): void {
  const rest = Math.floor(key / Y_RANGE);
  out[0] = (Math.floor(rest / XZ_RANGE) - XZ_LIMIT) * CHUNK_SIZE;
  out[1] = ((key % Y_RANGE) - Y_LIMIT) * CHUNK_SIZE;
  out[2] = ((rest % XZ_RANGE) - XZ_LIMIT) * CHUNK_SIZE;
}

// 以下临时数据只在本文件的函数里同步使用
const localAabb = new Aabb();
/** 形状的局部包围盒：中心 xyz、半边长 xyz */
const lb = [0, 0, 0, 0, 0, 0];
const cur = [0, 0, 0, 0, 0, 0];
const pred = [0, 0, 0, 0, 0, 0];
const rot = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const prot = [1, 0, 0, 0, 1, 0, 0, 0, 1];
/** 每个形状的整数扫描范围（闭区间），6 个一组 */
const members: number[] = [];
/** 每个形状的投影区间（刚体局部坐标），6 个一组 */
const shadows: number[] = [];
let shadowX = 0, shadowY = 0, shadowZ = 0, halfX = 0, halfY = 0, halfZ = 0;

/** 形状的局部包围盒写进 lb，返回以刚体原点为中心、包住它的球的半径 */
function localBounds(shape: Shape): number {
  shape.geom.computeAabb(localAabb, shape.localTransform);
  const a = localAabb;
  lb[0] = (a.minX + a.maxX) / 2; lb[1] = (a.minY + a.maxY) / 2; lb[2] = (a.minZ + a.maxZ) / 2;
  lb[3] = (a.maxX - a.minX) / 2; lb[4] = (a.maxY - a.minY) / 2; lb[5] = (a.maxZ - a.minZ) / 2;
  return Math.hypot(Math.max(Math.abs(a.minX), Math.abs(a.maxX)), Math.max(Math.abs(a.minY), Math.abs(a.maxY)), Math.max(Math.abs(a.minZ), Math.abs(a.maxZ)));
}

/** lb 按旋转矩阵 r（行优先）和位置 p 变换后的世界包围盒，写进 out（min xyz、max xyz） */
function worldBounds(r: readonly number[], px: number, py: number, pz: number, out: number[]): void {
  const cx = lb[0], cy = lb[1], cz = lb[2], hx = lb[3], hy = lb[4], hz = lb[5];
  const wx = px + r[0] * cx + r[1] * cy + r[2] * cz, wy = py + r[3] * cx + r[4] * cy + r[5] * cz, wz = pz + r[6] * cx + r[7] * cy + r[8] * cz;
  const ex = Math.abs(r[0]) * hx + Math.abs(r[1]) * hy + Math.abs(r[2]) * hz;
  const ey = Math.abs(r[3]) * hx + Math.abs(r[4]) * hy + Math.abs(r[5]) * hz;
  const ez = Math.abs(r[6]) * hx + Math.abs(r[7]) * hy + Math.abs(r[8]) * hz;
  out[0] = wx - ex; out[1] = wy - ey; out[2] = wz - ez; out[3] = wx + ex; out[4] = wy + ey; out[5] = wz + ez;
}

function velocityMargin(v: number, t: number, margin: number, prefetch: boolean): number {
  if (prefetch) return margin + Math.min(MAX_PREFETCH_MARGIN, Math.max(0, v * t));
  return Math.min(MAX_VELOCITY_MARGIN, margin + Math.abs(v) * t);
}

function loadRotation(body: RigidBody): void {
  const tf = body.transform;
  rot[0] = tf.r00; rot[1] = tf.r01; rot[2] = tf.r02; rot[3] = tf.r10; rot[4] = tf.r11; rot[5] = tf.r12; rot[6] = tf.r20; rot[7] = tf.r21; rot[8] = tf.r22;
}
/**
 * 当前范围（prefetch 为 false，t 为一个 tick）或预读范围（prefetch 为 true，t 为预读的时长）覆盖的块，键写进 out（不重复，按 y、z、x 的顺序）。
 * margin 是基础外扩（格）。没有形状、状态不是有限数时 out 为空
 */
export function predictedChunks(body: RigidBody, t: number, margin: number, prefetch: boolean, out: number[]): void {
  out.length = 0;
  members.length = 0;
  const tf = body.transform;
  const px = tf.px, py = tf.py, pz = tf.pz, vx = body.vx, vy = body.vy, vz = body.vz, wx = body.wx, wy = body.wy, wz = body.wz;
  if (!Number.isFinite(px + py + pz + vx + vy + vz + wx + wy + wz) || body.shapeList === null) return;
  const mxP = velocityMargin(vx, t, margin, prefetch), myP = velocityMargin(vy, t, margin, prefetch), mzP = velocityMargin(vz, t, margin, prefetch);
  const mxN = velocityMargin(-vx, t, margin, prefetch), myN = velocityMargin(-vy, t, margin, prefetch), mzN = velocityMargin(-vz, t, margin, prefetch);
  const w = Math.sqrt(wx * wx + wy * wy + wz * wz), travel = w * t;
  loadRotation(body);
  const rotate = travel > 1e-12 && travel < Math.PI;
  if (rotate) {
    // 绕角速度方向转 travel（Rodrigues 公式），再左乘当前旋转：角速度是世界坐标的
    const ax = wx / w, ay = wy / w, az = wz / w, s = Math.sin(travel), c = Math.cos(travel), k = 1 - c;
    const d00 = c + ax * ax * k, d01 = ax * ay * k - az * s, d02 = ax * az * k + ay * s;
    const d10 = ay * ax * k + az * s, d11 = c + ay * ay * k, d12 = ay * az * k - ax * s;
    const d20 = az * ax * k - ay * s, d21 = az * ay * k + ax * s, d22 = c + az * az * k;
    for (let i = 0; i < 3; i++) {
      const r0 = rot[i], r1 = rot[3 + i], r2 = rot[6 + i];
      prot[i] = d00 * r0 + d01 * r1 + d02 * r2;
      prot[3 + i] = d10 * r0 + d11 * r1 + d12 * r2;
      prot[6 + i] = d20 * r0 + d21 * r1 + d22 * r2;
    }
  }
  let count = 0;
  for (let s: Shape | null = body.shapeList; s !== null; s = s.next) {
    const radius = localBounds(s);
    let x0: number, y0: number, z0: number, x1: number, y1: number, z1: number;
    if (travel >= Math.PI) {
      // 单 tick 转角超过半圈时，使用绕旋转中心的完整球面范围。
      x0 = px - radius; y0 = py - radius; z0 = pz - radius; x1 = px + radius; y1 = py + radius; z1 = pz + radius;
    } else {
      worldBounds(rot, px, py, pz, cur);
      if (!rotate) {
        x0 = cur[0]; y0 = cur[1]; z0 = cur[2]; x1 = cur[3]; y1 = cur[4]; z1 = cur[5];
      } else {
        // 两个姿态的包围盒取并，再外扩这段圆弧的拱高
        worldBounds(prot, px, py, pz, pred);
        const sag = radius * (1 - Math.cos(travel / 2));
        x0 = Math.min(cur[0], pred[0]) - sag; y0 = Math.min(cur[1], pred[1]) - sag; z0 = Math.min(cur[2], pred[2]) - sag;
        x1 = Math.max(cur[3], pred[3]) + sag; y1 = Math.max(cur[4], pred[4]) + sag; z1 = Math.max(cur[5], pred[5]) + sag;
      }
    }
    members.push(Math.floor(x0 - mxN), Math.floor(y0 - myN), Math.floor(z0 - mzN), Math.floor(x1 + mxP), Math.floor(y1 + myP), Math.floor(z1 + mzP));
    count++;
  }
  let shadow = false;
  if (count <= SHADOW_MAX_SHAPES) {
    let x0 = Infinity, y0 = Infinity, z0 = Infinity, x1 = -Infinity, y1 = -Infinity, z1 = -Infinity;
    for (let i = 0; i < members.length; i += 6) {
      x0 = Math.min(x0, members[i]); y0 = Math.min(y0, members[i + 1]); z0 = Math.min(z0, members[i + 2]);
      x1 = Math.max(x1, members[i + 3]); y1 = Math.max(y1, members[i + 4]); z1 = Math.max(z1, members[i + 5]);
    }
    shadow = (x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) >= SHADOW_MIN_VOLUME;
  }
  if (shadow) prepareShadows(body, t, margin);
  enumerate(shadow, out);
}

/**
 * 将形状包围盒外扩 margin，按 y、z、x 顺序写入不重复的块键。
 * 可选 bounds 接收合并后的闭整数范围；无形状时 out 为空，bounds 保持不变。
 */
export function sleepingChunks(body: RigidBody, margin: number, out: number[], bounds?: number[]): void {
  out.length = 0;
  members.length = 0;
  if (!bodyBounds(body, cur)) return;
  for (let s: Shape | null = body.shapeList; s !== null; s = s.next) {
    localBounds(s);
    worldBounds(rot, body.transform.px, body.transform.py, body.transform.pz, pred);
    members.push(Math.floor(pred[0] - margin), Math.floor(pred[1] - margin), Math.floor(pred[2] - margin), Math.floor(pred[3] + margin), Math.floor(pred[4] + margin), Math.floor(pred[5] + margin));
  }
  if (bounds !== undefined) {
    bounds[0] = Math.floor(cur[0] - margin); bounds[1] = Math.floor(cur[1] - margin); bounds[2] = Math.floor(cur[2] - margin);
    bounds[3] = Math.floor(cur[3] + margin); bounds[4] = Math.floor(cur[4] + margin); bounds[5] = Math.floor(cur[5] + margin);
  }
  enumerate(false, out);
}

/**
 * 流体范围：形状包围盒水平方向外扩 1 格、向下 1.5 格，up 为 true 时再向上 64 格（气腔查液面的上限）。
 * 整数范围（闭区间，min xyz、max xyz）写进 range。没有形状时返回 false
 */
export function fluidRange(body: RigidBody, up: boolean, range: number[]): boolean {
  if (!bodyBounds(body, cur)) return false;
  range[0] = Math.floor(cur[0] - 1); range[1] = Math.floor(cur[1] - 1.5); range[2] = Math.floor(cur[2] - 1);
  range[3] = Math.floor(cur[3] + 1); range[4] = Math.floor(cur[4] + (up ? 64 : 0)); range[5] = Math.floor(cur[5] + 1);
  return true;
}

/** 整数范围（见 fluidRange）覆盖的块，键写进 out（不重复） */
export function rangeChunks(range: readonly number[], out: number[]): void {
  out.length = 0;
  members.length = 0;
  members.push(range[0], range[1], range[2], range[3], range[4], range[5]);
  enumerate(false, out);
}

/** 所有形状的局部包围盒变换到世界后合起来的包围盒（不含预测接触的外扩），写进 out。没有形状或位置不是有限数时返回 false */
export function bodyBounds(body: RigidBody, out: number[]): boolean {
  const tf = body.transform;
  if (!Number.isFinite(tf.px + tf.py + tf.pz) || body.shapeList === null) return false;
  loadRotation(body);
  out[0] = Infinity; out[1] = Infinity; out[2] = Infinity; out[3] = -Infinity; out[4] = -Infinity; out[5] = -Infinity;
  for (let s: Shape | null = body.shapeList; s !== null; s = s.next) {
    localBounds(s);
    worldBounds(rot, tf.px, tf.py, tf.pz, pred);
    for (let i = 0; i < 3; i++) {
      if (pred[i] < out[i]) out[i] = pred[i];
      if (pred[i + 3] > out[i + 3]) out[i + 3] = pred[i + 3];
    }
  }
  return true;
}

/** 投影用的数据：刚体的位置、一个世界轴对齐单位立方体投影到刚体三根局部轴上的半宽、每个形状外扩后的局部区间 */
function prepareShadows(body: RigidBody, t: number, margin: number): void {
  const tf = body.transform;
  shadowX = tf.px; shadowY = tf.py; shadowZ = tf.pz;
  halfX = 0.5 * (Math.abs(rot[0]) + Math.abs(rot[3]) + Math.abs(rot[6]));
  halfY = 0.5 * (Math.abs(rot[1]) + Math.abs(rot[4]) + Math.abs(rot[7]));
  halfZ = 0.5 * (Math.abs(rot[2]) + Math.abs(rot[5]) + Math.abs(rot[8]));
  const linear = Math.hypot(body.vx, body.vy, body.vz) * t, angular = Math.hypot(body.wx, body.wy, body.wz) * t;
  shadows.length = 0;
  for (let s: Shape | null = body.shapeList; s !== null; s = s.next) {
    const radius = localBounds(s);
    const e = margin + linear + (angular >= Math.PI ? radius * 2 : radius * angular) + 1e-6;
    shadows.push(lb[0] - lb[3] - e, lb[1] - lb[4] - e, lb[2] - lb[5] - e, lb[0] + lb[3] + e, lb[1] + lb[4] + e, lb[2] + lb[5] + e);
  }
}

/** 最小角为 (ox, oy, oz) 的 8³ 块在刚体三根局部轴上都和某个形状的投影区间重叠 */
function retained(ox: number, oy: number, oz: number): boolean {
  const dx = ox + 4 - shadowX, dy = oy + 4 - shadowY, dz = oz + 4 - shadowZ;
  const lx = rot[0] * dx + rot[3] * dy + rot[6] * dz, ly = rot[1] * dx + rot[4] * dy + rot[7] * dz, lz = rot[2] * dx + rot[5] * dy + rot[8] * dz;
  const ex = halfX * 8, ey = halfY * 8, ez = halfZ * 8;
  for (let i = 0; i < shadows.length; i += 6) {
    if (lx + ex >= shadows[i] && lx - ex <= shadows[i + 3] && ly + ey >= shadows[i + 1] && ly - ey <= shadows[i + 4] &&
      lz + ez >= shadows[i + 2] && lz - ez <= shadows[i + 5]) return true;
  }
  return false;
}

const seen = new Set<number>();
/** members 里各个范围覆盖的块，按 y、z、x 的顺序去重，开了投影时去掉投影排除的，写进 out */
function enumerate(shadow: boolean, out: number[]): void {
  seen.clear();
  for (let i = 0; i < members.length; i += 6) {
    const cx0 = Math.max(-XZ_LIMIT, Math.floor(members[i] / CHUNK_SIZE)), cx1 = Math.min(XZ_LIMIT - 1, Math.floor(members[i + 3] / CHUNK_SIZE));
    const cy0 = Math.max(-Y_LIMIT, Math.floor(members[i + 1] / CHUNK_SIZE)), cy1 = Math.min(Y_LIMIT - 1, Math.floor(members[i + 4] / CHUNK_SIZE));
    const cz0 = Math.max(-XZ_LIMIT, Math.floor(members[i + 2] / CHUNK_SIZE)), cz1 = Math.min(XZ_LIMIT - 1, Math.floor(members[i + 5] / CHUNK_SIZE));
    for (let cy = cy0; cy <= cy1; cy++) for (let cz = cz0; cz <= cz1; cz++) for (let cx = cx0; cx <= cx1; cx++) {
      const key = chunkKey(cx, cy, cz);
      if (seen.has(key)) continue;
      seen.add(key);
      if (shadow && !retained(cx * CHUNK_SIZE, cy * CHUNK_SIZE, cz * CHUNK_SIZE)) continue;
      out.push(key);
    }
  }
}
