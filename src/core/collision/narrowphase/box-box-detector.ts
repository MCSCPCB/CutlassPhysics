// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/BoxBoxDetector.hx（MIT，见 Credits.md）
// SAT 使用 6 根面轴和 9 根棱轴，选择最小穿透方向。
// 棱接触生成 1 个点，面裁剪最多生成 4 个点。
// LINEAR_SLOP 偏置优先选择面轴和 box1，稳定接触法线。
// x/y/z 为单位轴，sx/sy/sz 为半轴，w/h/d 为半边长，c12 从 box1 指向 box2。
// margin 扩展预测接触距离，分离点深度为负；面裁剪保留阈值为 -(CONTACT_PERSISTENCE_THRESHOLD + margin)。
import type { Transform } from "../../common/transform.js";
import { CONTACT_PERSISTENCE_THRESHOLD, LINEAR_SLOP } from "../../settings.js";
import type { BoxGeometry } from "../geometry/box-geometry.js";
import type { Geometry } from "../geometry/geometry.js";
import type { DetectorResult } from "./detector-result.js";
import { Detector } from "./detector.js";
import { FaceClipper } from "./face-clipper.js";

export class BoxBoxDetector extends Detector {
  private readonly clipper = new FaceClipper();

  constructor() {
    super(false);
  }

  protected detectImpl(
    result: DetectorResult, geom1: Geometry, geom2: Geometry, tf1: Transform, tf2: Transform, _cachedData: unknown, margin: number,
  ): void {
    result.incremental = false;
    const b1 = geom1 as BoxGeometry;
    const b2 = geom2 as BoxGeometry;

    // 中心，以及从 box1 中心指向 box2 中心的向量
    let c1X = tf1.px, c1Y = tf1.py, c1Z = tf1.pz;
    const c2X = tf2.px, c2Y = tf2.py, c2Z = tf2.pz;
    let c12X = c2X - c1X, c12Y = c2Y - c1Y, c12Z = c2Z - c1Z;

    // 单位轴：旋转矩阵的三列
    let x1X = tf1.r00, x1Y = tf1.r10, x1Z = tf1.r20;
    let y1X = tf1.r01, y1Y = tf1.r11, y1Z = tf1.r21;
    let z1X = tf1.r02, z1Y = tf1.r12, z1Z = tf1.r22;
    let x2X = tf2.r00, x2Y = tf2.r10, x2Z = tf2.r20;
    let y2X = tf2.r01, y2Y = tf2.r11, y2Z = tf2.r21;
    let z2X = tf2.r02, z2Y = tf2.r12, z2Z = tf2.r22;

    // 半边长
    let w1 = b1.hx, h1 = b1.hy, d1 = b1.hz;
    const w2 = b2.hx, h2 = b2.hy, d2 = b2.hz;

    // 半轴 = 单位轴 × 半边长
    let sx1X = x1X * w1, sx1Y = x1Y * w1, sx1Z = x1Z * w1;
    let sy1X = y1X * h1, sy1Y = y1Y * h1, sy1Z = y1Z * h1;
    let sz1X = z1X * d1, sz1Y = z1Y * d1, sz1Z = z1Z * d1;
    let sx2X = x2X * w2, sx2Y = x2Y * w2, sx2Z = x2Z * w2;
    let sy2X = y2X * h2, sy2Y = y2Y * h2, sy2Z = y2Z * h2;
    let sz2X = z2X * d2, sz2Y = z2Y * d2, sz2Z = z2Z * d2;

    // SAT 过程中穿透最浅的轴：深度、编号（0~5 面轴，6~14 棱轴）、符号、方向
    let mDepth = Infinity;
    let mId = -1;
    let mSign = 0;
    let mAxisX = 0, mAxisY = 0, mAxisZ = 0;
    // 每根轴的临时量：两个盒子的投影半长、中心连线的投影、深度
    let proj1: number, proj2: number, projC12: number, sum: number, abs: number, depth: number;
    let neg: boolean;
    let dx: number, dy: number, dz: number;
    let eX: number, eY: number, eZ: number, l: number;

    // ---- 6 根面轴（不重叠就是分离轴，直接返回）----
    // x1
    dx = x1X * sx2X + x1Y * sx2Y + x1Z * sx2Z; if (dx < 0) dx = -dx;
    dy = x1X * sy2X + x1Y * sy2Y + x1Z * sy2Z; if (dy < 0) dy = -dy;
    dz = x1X * sz2X + x1Y * sz2Y + x1Z * sz2Z; if (dz < 0) dz = -dz;
    proj1 = w1;
    proj2 = dx + dy + dz;
    projC12 = x1X * c12X + x1Y * c12Y + x1Z * c12Z;
    sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) { mDepth = depth; mId = 0; mAxisX = x1X; mAxisY = x1Y; mAxisZ = x1Z; mSign = neg ? -1 : 1; }
    // y1
    dx = y1X * sx2X + y1Y * sx2Y + y1Z * sx2Z; if (dx < 0) dx = -dx;
    dy = y1X * sy2X + y1Y * sy2Y + y1Z * sy2Z; if (dy < 0) dy = -dy;
    dz = y1X * sz2X + y1Y * sz2Y + y1Z * sz2Z; if (dz < 0) dz = -dz;
    proj1 = h1;
    proj2 = dx + dy + dz;
    projC12 = y1X * c12X + y1Y * c12Y + y1Z * c12Z;
    sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) { mDepth = depth; mId = 1; mAxisX = y1X; mAxisY = y1Y; mAxisZ = y1Z; mSign = neg ? -1 : 1; }
    // z1
    dx = z1X * sx2X + z1Y * sx2Y + z1Z * sx2Z; if (dx < 0) dx = -dx;
    dy = z1X * sy2X + z1Y * sy2Y + z1Z * sy2Z; if (dy < 0) dy = -dy;
    dz = z1X * sz2X + z1Y * sz2Y + z1Z * sz2Z; if (dz < 0) dz = -dz;
    proj1 = d1;
    proj2 = dx + dy + dz;
    projC12 = z1X * c12X + z1Y * c12Y + z1Z * c12Z;
    sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) { mDepth = depth; mId = 2; mAxisX = z1X; mAxisY = z1Y; mAxisZ = z1Z; mSign = neg ? -1 : 1; }

    // 偏置：深度差不多时优先选 box1 的面
    if (mDepth > LINEAR_SLOP) mDepth -= LINEAR_SLOP; else if (mDepth < 0) mDepth -= LINEAR_SLOP; else mDepth = 0;

    // x2
    dx = x2X * sx1X + x2Y * sx1Y + x2Z * sx1Z; if (dx < 0) dx = -dx;
    dy = x2X * sy1X + x2Y * sy1Y + x2Z * sy1Z; if (dy < 0) dy = -dy;
    dz = x2X * sz1X + x2Y * sz1Y + x2Z * sz1Z; if (dz < 0) dz = -dz;
    proj1 = dx + dy + dz;
    proj2 = w2;
    projC12 = x2X * c12X + x2Y * c12Y + x2Z * c12Z;
    sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) { mDepth = depth; mId = 3; mAxisX = x2X; mAxisY = x2Y; mAxisZ = x2Z; mSign = neg ? -1 : 1; }
    // y2
    dx = y2X * sx1X + y2Y * sx1Y + y2Z * sx1Z; if (dx < 0) dx = -dx;
    dy = y2X * sy1X + y2Y * sy1Y + y2Z * sy1Z; if (dy < 0) dy = -dy;
    dz = y2X * sz1X + y2Y * sz1Y + y2Z * sz1Z; if (dz < 0) dz = -dz;
    proj1 = dx + dy + dz;
    proj2 = h2;
    projC12 = y2X * c12X + y2Y * c12Y + y2Z * c12Z;
    sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) { mDepth = depth; mId = 4; mAxisX = y2X; mAxisY = y2Y; mAxisZ = y2Z; mSign = neg ? -1 : 1; }
    // z2
    dx = z2X * sx1X + z2Y * sx1Y + z2Z * sx1Z; if (dx < 0) dx = -dx;
    dy = z2X * sy1X + z2Y * sy1Y + z2Z * sy1Z; if (dy < 0) dy = -dy;
    dz = z2X * sz1X + z2Y * sz1Y + z2Z * sz1Z; if (dz < 0) dz = -dz;
    proj1 = dx + dy + dz;
    proj2 = d2;
    projC12 = z2X * c12X + z2Y * c12Y + z2Z * c12Z;
    sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) { mDepth = depth; mId = 5; mAxisX = z2X; mAxisY = z2Y; mAxisZ = z2Z; mSign = neg ? -1 : 1; }

    // 偏置：深度差不多时优先选面轴
    if (mDepth > LINEAR_SLOP) mDepth -= LINEAR_SLOP; else if (mDepth < 0) mDepth -= LINEAR_SLOP; else mDepth = 0;

    // ---- 9 根棱轴：cross(box1 的轴, box2 的轴)，两根轴平行时跳过；投影只用垂直于棱的两根半轴 ----
    // cross(x1, x2)
    eX = x1Y * x2Z - x1Z * x2Y; eY = x1Z * x2X - x1X * x2Z; eZ = x1X * x2Y - x1Y * x2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sy1X + eY * sy1Y + eZ * sy1Z; if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sy2X + eY * sy2Y + eZ * sy2Z; if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 6; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }
    // cross(x1, y2)
    eX = x1Y * y2Z - x1Z * y2Y; eY = x1Z * y2X - x1X * y2Z; eZ = x1X * y2Y - x1Y * y2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sy1X + eY * sy1Y + eZ * sy1Z; if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z; if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 7; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }
    // cross(x1, z2)
    eX = x1Y * z2Z - x1Z * z2Y; eY = x1Z * z2X - x1X * z2Z; eZ = x1X * z2Y - x1Y * z2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sy1X + eY * sy1Y + eZ * sy1Z; if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z; if (dx < 0) dx = -dx;
      dy = eX * sy2X + eY * sy2Y + eZ * sy2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 8; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }

    // cross(y1, x2)
    eX = y1Y * x2Z - y1Z * x2Y; eY = y1Z * x2X - y1X * x2Z; eZ = y1X * x2Y - y1Y * x2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z; if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sy2X + eY * sy2Y + eZ * sy2Z; if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 9; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }
    // cross(y1, y2)
    eX = y1Y * y2Z - y1Z * y2Y; eY = y1Z * y2X - y1X * y2Z; eZ = y1X * y2Y - y1Y * y2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z; if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z; if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 10; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }
    // cross(y1, z2)
    eX = y1Y * z2Z - y1Z * z2Y; eY = y1Z * z2X - y1X * z2Z; eZ = y1X * z2Y - y1Y * z2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z; if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z; if (dx < 0) dx = -dx;
      dy = eX * sy2X + eY * sy2Y + eZ * sy2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 11; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }

    // cross(z1, x2)
    eX = z1Y * x2Z - z1Z * x2Y; eY = z1Z * x2X - z1X * x2Z; eZ = z1X * x2Y - z1Y * x2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z; if (dx < 0) dx = -dx;
      dy = eX * sy1X + eY * sy1Y + eZ * sy1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sy2X + eY * sy2Y + eZ * sy2Z; if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 12; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }
    // cross(z1, y2)
    eX = z1Y * y2Z - z1Z * y2Y; eY = z1Z * y2X - z1X * y2Z; eZ = z1X * y2Y - z1Y * y2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z; if (dx < 0) dx = -dx;
      dy = eX * sy1X + eY * sy1Y + eZ * sy1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z; if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 13; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }
    // cross(z1, z2)
    eX = z1Y * z2Z - z1Z * z2Y; eY = z1Z * z2X - z1X * z2Z; eZ = z1X * z2Y - z1Y * z2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ; if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l; eY *= l; eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z; if (dx < 0) dx = -dx;
      dy = eX * sy1X + eY * sy1Y + eZ * sy1Z; if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z; if (dx < 0) dx = -dx;
      dy = eX * sy2X + eY * sy2Y + eZ * sy2Z; if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2; neg = projC12 < 0; abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) { mDepth = depth; mId = 14; mAxisX = eX; mAxisY = eY; mAxisZ = eZ; mSign = neg ? -1 : 1; }
    }

    // ---- 棱对棱：两条棱的最近点作为唯一的接触点（编号固定为 4）----
    if (mId >= 6) {
      // 让轴从 box1 指向 box2
      mAxisX *= mSign; mAxisY *= mSign; mAxisZ *= mSign;
      const id1 = ((mId - 6) / 3) | 0;
      const id2 = mId - 6 - id1 * 3;
      // 每个盒子：棱方向 e，以及垂直于这条棱的两根半轴 a、b
      let e1X: number, e1Y: number, e1Z: number, a1X: number, a1Y: number, a1Z: number, b1X: number, b1Y: number, b1Z: number;
      if (id1 === 0) { e1X = x1X; e1Y = x1Y; e1Z = x1Z; a1X = sy1X; a1Y = sy1Y; a1Z = sy1Z; b1X = sz1X; b1Y = sz1Y; b1Z = sz1Z; }
      else if (id1 === 1) { e1X = y1X; e1Y = y1Y; e1Z = y1Z; a1X = sx1X; a1Y = sx1Y; a1Z = sx1Z; b1X = sz1X; b1Y = sz1Y; b1Z = sz1Z; }
      else { e1X = z1X; e1Y = z1Y; e1Z = z1Z; a1X = sx1X; a1Y = sx1Y; a1Z = sx1Z; b1X = sy1X; b1Y = sy1Y; b1Z = sy1Z; }
      let e2X: number, e2Y: number, e2Z: number, a2X: number, a2Y: number, a2Z: number, b2X: number, b2Y: number, b2Z: number;
      if (id2 === 0) { e2X = x2X; e2Y = x2Y; e2Z = x2Z; a2X = sy2X; a2Y = sy2Y; a2Z = sy2Z; b2X = sz2X; b2Y = sz2Y; b2Z = sz2Z; }
      else if (id2 === 1) { e2X = y2X; e2Y = y2Y; e2Z = y2Z; a2X = sx2X; a2Y = sx2Y; a2Z = sx2Z; b2X = sz2X; b2Y = sz2Y; b2Z = sz2Z; }
      else { e2X = z2X; e2Y = z2Y; e2Z = z2Z; a2X = sx2X; a2Y = sx2Y; a2Z = sx2Z; b2X = sy2X; b2Y = sy2Y; b2Z = sy2Z; }

      // 棱上一点：a、b 张成的矩形上沿轴最远的顶点。box1 用 c1 + 顶点，box2 用 c2 - 顶点
      let sa = a1X * mAxisX + a1Y * mAxisY + a1Z * mAxisZ > 0;
      let sb = b1X * mAxisX + b1Y * mAxisY + b1Z * mAxisZ > 0;
      let p1X: number, p1Y: number, p1Z: number;
      if (sa) {
        if (sb) { p1X = a1X + b1X; p1Y = a1Y + b1Y; p1Z = a1Z + b1Z; } else { p1X = a1X - b1X; p1Y = a1Y - b1Y; p1Z = a1Z - b1Z; }
      } else if (sb) { p1X = b1X - a1X; p1Y = b1Y - a1Y; p1Z = b1Z - a1Z; } else { p1X = -(a1X + b1X); p1Y = -(a1Y + b1Y); p1Z = -(a1Z + b1Z); }
      p1X = c1X + p1X; p1Y = c1Y + p1Y; p1Z = c1Z + p1Z;
      sa = a2X * mAxisX + a2Y * mAxisY + a2Z * mAxisZ > 0;
      sb = b2X * mAxisX + b2Y * mAxisY + b2Z * mAxisZ > 0;
      let p2X: number, p2Y: number, p2Z: number;
      if (sa) {
        if (sb) { p2X = a2X + b2X; p2Y = a2Y + b2Y; p2Z = a2Z + b2Z; } else { p2X = a2X - b2X; p2Y = a2Y - b2Y; p2Z = a2Z - b2Z; }
      } else if (sb) { p2X = b2X - a2X; p2Y = b2Y - a2Y; p2Z = b2Z - a2Z; } else { p2X = -(a2X + b2X); p2Y = -(a2Y + b2Y); p2Z = -(a2Z + b2Z); }
      p2X = c2X - p2X; p2Y = c2Y - p2Y; p2Z = c2Z - p2Z;

      // 两条直线 p1 + t1·e1、p2 + t2·e2 的最近点
      const rX = p1X - p2X, rY = p1Y - p2Y, rZ = p1Z - p2Z;
      const dot12 = e1X * e2X + e1Y * e2Y + e1Z * e2Z;
      const dot1r = e1X * rX + e1Y * rY + e1Z * rZ;
      const dot2r = e2X * rX + e2Y * rY + e2Z * rZ;
      const invDet = 1 / (1 - dot12 * dot12);
      const t1 = (dot12 * dot2r - dot1r) * invDet;
      const t2 = (dot2r - dot12 * dot1r) * invDet;
      this.setNormal(result, -mAxisX, -mAxisY, -mAxisZ);
      this.addPoint(result, p1X + e1X * t1, p1Y + e1Y * t1, p1Z + e1Z * t1, p2X + e2X * t2, p2Y + e2Y * t2, p2Z + e2Z * t2, mDepth, 4);
      return;
    }

    // ---- 面对面：把入射面裁剪进参考面，最多 4 个接触点 ----
    // 参考面位于 box2（mId 为 3～5）时交换盒子，使 box1 始终为参考盒。
    let t: number;
    const refOnBox2 = mId >= 3;
    if (refOnBox2) {
      mSign = -mSign;
      c12X = -c12X; c12Y = -c12Y; c12Z = -c12Z;
      // box2 的半边长和中心之后用不到，只需要换到 box1 这边
      w1 = w2; h1 = h2; d1 = d2;
      c1X = c2X; c1Y = c2Y; c1Z = c2Z;
      t = x1X; x1X = x2X; x2X = t; t = x1Y; x1Y = x2Y; x2Y = t; t = x1Z; x1Z = x2Z; x2Z = t;
      t = y1X; y1X = y2X; y2X = t; t = y1Y; y1Y = y2Y; y2Y = t; t = y1Z; y1Z = y2Z; y2Z = t;
      t = z1X; z1X = z2X; z2X = t; t = z1Y; z1Y = z2Y; z2Y = t; t = z1Z; z1Z = z2Z; z2Z = t;
      t = sx1X; sx1X = sx2X; sx2X = t; t = sx1Y; sx1Y = sx2Y; sx2Y = t; t = sx1Z; sx1Z = sx2Z; sx2Z = t;
      t = sy1X; sy1X = sy2X; sy2X = t; t = sy1Y; sy1Y = sy2Y; sy2Y = t; t = sy1Z; sy1Z = sy2Z; sy2Z = t;
      t = sz1X; sz1X = sz2X; sz2X = t; t = sz1Y; sz1Y = sz2Y; sz2Y = t; t = sz1Z; sz1Z = sz2Z; sz2Z = t;
      mId -= 3;
    }

    // 参考面：box1 的第 mId 个面（mSign < 0 时是反方向的面）。面中心 refC、法线 refN、面内两根轴 refX/refY、半宽半高
    let refCX: number, refCY: number, refCZ: number, refNX: number, refNY: number, refNZ: number;
    let refXX: number, refXY: number, refXZ: number, refYX: number, refYY: number, refYZ: number;
    let refW: number, refH: number;
    if (mId === 0) {
      refCX = sx1X; refCY = sx1Y; refCZ = sx1Z; refNX = x1X; refNY = x1Y; refNZ = x1Z;
      refXX = y1X; refXY = y1Y; refXZ = y1Z; refYX = z1X; refYY = z1Y; refYZ = z1Z; refW = h1; refH = d1;
    } else if (mId === 1) {
      refCX = sy1X; refCY = sy1Y; refCZ = sy1Z; refNX = y1X; refNY = y1Y; refNZ = y1Z;
      refXX = z1X; refXY = z1Y; refXZ = z1Z; refYX = x1X; refYY = x1Y; refYZ = x1Z; refW = d1; refH = w1;
    } else {
      refCX = sz1X; refCY = sz1Y; refCZ = sz1Z; refNX = z1X; refNY = z1Y; refNZ = z1Z;
      refXX = x1X; refXY = x1Y; refXZ = x1Z; refYX = y1X; refYY = y1Y; refYZ = y1Z; refW = w1; refH = h1;
    }
    if (mSign < 0) {
      // x-、y-、z- 面：面中心和法线取反，面内两根轴互换
      refCX = -refCX; refCY = -refCY; refCZ = -refCZ;
      refNX = -refNX; refNY = -refNY; refNZ = -refNZ;
      t = refXX; refXX = refYX; refYX = t; t = refXY; refXY = refYY; refYY = t; t = refXZ; refXZ = refYZ; refYZ = t;
      t = refW; refW = refH; refH = t;
    }
    // 参考面中心转成世界坐标
    refCX += c1X; refCY += c1Y; refCZ += c1Z;

    // 入射面：box2 上和参考面法线最“相对”的面（依次比较 x+、x-、y+、y-、z+、z-）
    let minIncDot = 1;
    let incId = 0;
    let incDot = refNX * x2X + refNY * x2Y + refNZ * x2Z;
    if (incDot < minIncDot) { minIncDot = incDot; incId = 0; }
    if (-incDot < minIncDot) { minIncDot = -incDot; incId = 1; }
    incDot = refNX * y2X + refNY * y2Y + refNZ * y2Z;
    if (incDot < minIncDot) { minIncDot = incDot; incId = 2; }
    if (-incDot < minIncDot) { minIncDot = -incDot; incId = 3; }
    incDot = refNX * z2X + refNY * z2Y + refNZ * z2Z;
    if (incDot < minIncDot) { minIncDot = incDot; incId = 4; }
    if (-incDot < minIncDot) incId = 5;

    // 入射面的 4 个顶点（相对 box2 中心），再平移到相对 box1 中心
    let v1X: number, v1Y: number, v1Z: number, v2X: number, v2Y: number, v2Z: number;
    let v3X: number, v3Y: number, v3Z: number, v4X: number, v4Y: number, v4Z: number;
    switch (incId) {
      case 0: // x+
        v1X = sx2X + sy2X + sz2X; v1Y = sx2Y + sy2Y + sz2Y; v1Z = sx2Z + sy2Z + sz2Z;
        v2X = sx2X - sy2X + sz2X; v2Y = sx2Y - sy2Y + sz2Y; v2Z = sx2Z - sy2Z + sz2Z;
        v3X = sx2X - sy2X - sz2X; v3Y = sx2Y - sy2Y - sz2Y; v3Z = sx2Z - sy2Z - sz2Z;
        v4X = sx2X + sy2X - sz2X; v4Y = sx2Y + sy2Y - sz2Y; v4Z = sx2Z + sy2Z - sz2Z;
        break;
      case 1: // x-
        v1X = sy2X - sx2X + sz2X; v1Y = sy2Y - sx2Y + sz2Y; v1Z = sy2Z - sx2Z + sz2Z;
        v2X = sy2X - sx2X - sz2X; v2Y = sy2Y - sx2Y - sz2Y; v2Z = sy2Z - sx2Z - sz2Z;
        v3X = -(sx2X + sy2X) - sz2X; v3Y = -(sx2Y + sy2Y) - sz2Y; v3Z = -(sx2Z + sy2Z) - sz2Z;
        v4X = -(sx2X + sy2X) + sz2X; v4Y = -(sx2Y + sy2Y) + sz2Y; v4Z = -(sx2Z + sy2Z) + sz2Z;
        break;
      case 2: // y+
        v1X = sx2X + sy2X + sz2X; v1Y = sx2Y + sy2Y + sz2Y; v1Z = sx2Z + sy2Z + sz2Z;
        v2X = sx2X + sy2X - sz2X; v2Y = sx2Y + sy2Y - sz2Y; v2Z = sx2Z + sy2Z - sz2Z;
        v3X = sy2X - sx2X - sz2X; v3Y = sy2Y - sx2Y - sz2Y; v3Z = sy2Z - sx2Z - sz2Z;
        v4X = sy2X - sx2X + sz2X; v4Y = sy2Y - sx2Y + sz2Y; v4Z = sy2Z - sx2Z + sz2Z;
        break;
      case 3: // y-
        v1X = sx2X - sy2X + sz2X; v1Y = sx2Y - sy2Y + sz2Y; v1Z = sx2Z - sy2Z + sz2Z;
        v2X = -(sx2X + sy2X) + sz2X; v2Y = -(sx2Y + sy2Y) + sz2Y; v2Z = -(sx2Z + sy2Z) + sz2Z;
        v3X = -(sx2X + sy2X) - sz2X; v3Y = -(sx2Y + sy2Y) - sz2Y; v3Z = -(sx2Z + sy2Z) - sz2Z;
        v4X = sx2X - sy2X - sz2X; v4Y = sx2Y - sy2Y - sz2Y; v4Z = sx2Z - sy2Z - sz2Z;
        break;
      case 4: // z+
        v1X = sx2X + sy2X + sz2X; v1Y = sx2Y + sy2Y + sz2Y; v1Z = sx2Z + sy2Z + sz2Z;
        v2X = sy2X - sx2X + sz2X; v2Y = sy2Y - sx2Y + sz2Y; v2Z = sy2Z - sx2Z + sz2Z;
        v3X = -(sx2X + sy2X) + sz2X; v3Y = -(sx2Y + sy2Y) + sz2Y; v3Z = -(sx2Z + sy2Z) + sz2Z;
        v4X = sx2X - sy2X + sz2X; v4Y = sx2Y - sy2Y + sz2Y; v4Z = sx2Z - sy2Z + sz2Z;
        break;
      default: // z-
        v1X = sx2X + sy2X - sz2X; v1Y = sx2Y + sy2Y - sz2Y; v1Z = sx2Z + sy2Z - sz2Z;
        v2X = sx2X - sy2X - sz2X; v2Y = sx2Y - sy2Y - sz2Y; v2Z = sx2Z - sy2Z - sz2Z;
        v3X = -(sx2X + sy2X) - sz2X; v3Y = -(sx2Y + sy2Y) - sz2Y; v3Z = -(sx2Z + sy2Z) - sz2Z;
        v4X = sy2X - sx2X - sz2X; v4Y = sy2Y - sx2Y - sz2Y; v4Z = sy2Z - sx2Z - sz2Z;
    }
    v1X += c12X; v1Y += c12Y; v1Z += c12Z;
    v2X += c12X; v2Y += c12Y; v2Z += c12Z;
    v3X += c12X; v3Y += c12Y; v3Z += c12Z;
    v4X += c12X; v4Y += c12Y; v4Z += c12Z;

    // 入射面投影到参考面坐标系后裁剪，再精简到最多 4 个点
    const clipper = this.clipper;
    clipper.init(refW, refH);
    clipper.addIncidentVertex(v1X * refXX + v1Y * refXY + v1Z * refXZ, v1X * refYX + v1Y * refYY + v1Z * refYZ, v1X, v1Y, v1Z);
    clipper.addIncidentVertex(v2X * refXX + v2Y * refXY + v2Z * refXZ, v2X * refYX + v2Y * refYY + v2Z * refYZ, v2X, v2Y, v2Z);
    clipper.addIncidentVertex(v3X * refXX + v3Y * refXY + v3Z * refXZ, v3X * refYX + v3Y * refYY + v3Z * refYZ, v3X, v3Y, v3Z);
    clipper.addIncidentVertex(v4X * refXX + v4Y * refXY + v4Z * refXZ, v4X * refYX + v4Y * refYY + v4Z * refYZ, v4X, v4Y, v4Z);
    clipper.clip();
    clipper.reduce();

    // 法线从 box2 指向 box1（交换过时参考面在原来的 box2 上，方向反过来）
    if (refOnBox2) this.setNormal(result, refNX, refNY, refNZ);
    else this.setNormal(result, -refNX, -refNY, -refNZ);

    for (let i = 0; i < clipper.numVertices; i++) {
      const v = clipper.vertices[i];
      const cX = v.wx + c1X, cY = v.wy + c1Y, cZ = v.wz + c1Z;
      // 顶点穿进参考面的深度，以及它在参考面上的投影点
      depth = (refCX - cX) * refNX + (refCY - cY) * refNY + (refCZ - cZ) * refNZ;
      const fX = cX + refNX * depth, fY = cY + refNY * depth, fZ = cZ + refNZ * depth;
      if (depth > -CONTACT_PERSISTENCE_THRESHOLD - margin) {
        if (refOnBox2) this.addPoint(result, cX, cY, cZ, fX, fY, fZ, depth, i);
        else this.addPoint(result, fX, fY, fZ, cX, cY, cZ, depth, i);
      }
    }
  }
}
