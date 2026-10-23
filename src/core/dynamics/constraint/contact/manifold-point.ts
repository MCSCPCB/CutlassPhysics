// 移植自 OimoPhysics src/oimo/dynamics/constraint/contact/ManifoldPoint.hx（MIT，见 Credits.md）
import type { DetectorResultPoint } from "../../../collision/narrowphase/detector-result.js";
import type { Transform } from "../../../common/transform.js";
import { ContactImpulse } from "./contact-impulse.js";

/** 接触流形里的一个接触点。位置相对的是刚体（不是形状） */
export class ManifoldPoint {
  /** 刚体局部坐标 */
  lp1X = 0; lp1Y = 0; lp1Z = 0;
  lp2X = 0; lp2Y = 0; lp2Z = 0;
  /** 相对刚体重心、世界方向的位置 */
  rp1X = 0; rp1Y = 0; rp1Z = 0;
  rp2X = 0; rp2Y = 0; rp2Z = 0;
  /** 世界坐标 */
  p1X = 0; p1Y = 0; p1Z = 0;
  p2X = 0; p2Y = 0; p2Z = 0;
  depth = 0;
  readonly impulse = new ContactImpulse();
  /** 冲量是从上一步接过来的 */
  warmStarted = false;
  /** 已分离接触点的停用标志。 */
  disabled = false;
  id = -1;

  clear(): void {
    this.lp1X = 0; this.lp1Y = 0; this.lp1Z = 0;
    this.lp2X = 0; this.lp2Y = 0; this.lp2Z = 0;
    this.rp1X = 0; this.rp1Y = 0; this.rp1Z = 0;
    this.rp2X = 0; this.rp2Y = 0; this.rp2Z = 0;
    this.p1X = 0; this.p1Y = 0; this.p1Z = 0;
    this.p2X = 0; this.p2Y = 0; this.p2Z = 0;
    this.depth = 0;
    this.impulse.clear();
    this.warmStarted = false;
    this.disabled = false;
    this.id = -1;
  }

  /** 用检测结果初始化 */
  initialize(r: DetectorResultPoint, tf1: Transform, tf2: Transform): void {
    this.p1X = r.p1x; this.p1Y = r.p1y; this.p1Z = r.p1z;
    this.p2X = r.p2x; this.p2Y = r.p2y; this.p2Z = r.p2z;
    this.rp1X = this.p1X - tf1.px; this.rp1Y = this.p1Y - tf1.py; this.rp1Z = this.p1Z - tf1.pz;
    this.rp2X = this.p2X - tf2.px; this.rp2Y = this.p2Y - tf2.py; this.rp2Z = this.p2Z - tf2.pz;
    // 世界方向 → 刚体局部：乘旋转矩阵的转置
    this.lp1X = tf1.r00 * this.rp1X + tf1.r10 * this.rp1Y + tf1.r20 * this.rp1Z;
    this.lp1Y = tf1.r01 * this.rp1X + tf1.r11 * this.rp1Y + tf1.r21 * this.rp1Z;
    this.lp1Z = tf1.r02 * this.rp1X + tf1.r12 * this.rp1Y + tf1.r22 * this.rp1Z;
    this.lp2X = tf2.r00 * this.rp2X + tf2.r10 * this.rp2Y + tf2.r20 * this.rp2Z;
    this.lp2Y = tf2.r01 * this.rp2X + tf2.r11 * this.rp2Y + tf2.r21 * this.rp2Z;
    this.lp2Z = tf2.r02 * this.rp2X + tf2.r12 * this.rp2Y + tf2.r22 * this.rp2Z;
    this.depth = r.depth;
    this.impulse.clear();
    this.id = r.id;
    this.warmStarted = false;
    this.disabled = false;
  }

  /** 用新的检测结果更新位置和深度，保留冲量、编号（增量更新用） */
  updateDepthAndPositions(r: DetectorResultPoint, tf1: Transform, tf2: Transform): void {
    this.p1X = r.p1x; this.p1Y = r.p1y; this.p1Z = r.p1z;
    this.p2X = r.p2x; this.p2Y = r.p2y; this.p2Z = r.p2z;
    this.rp1X = this.p1X - tf1.px; this.rp1Y = this.p1Y - tf1.py; this.rp1Z = this.p1Z - tf1.pz;
    this.rp2X = this.p2X - tf2.px; this.rp2Y = this.p2Y - tf2.py; this.rp2Z = this.p2Z - tf2.pz;
    this.lp1X = tf1.r00 * this.rp1X + tf1.r10 * this.rp1Y + tf1.r20 * this.rp1Z;
    this.lp1Y = tf1.r01 * this.rp1X + tf1.r11 * this.rp1Y + tf1.r21 * this.rp1Z;
    this.lp1Z = tf1.r02 * this.rp1X + tf1.r12 * this.rp1Y + tf1.r22 * this.rp1Z;
    this.lp2X = tf2.r00 * this.rp2X + tf2.r10 * this.rp2Y + tf2.r20 * this.rp2Z;
    this.lp2Y = tf2.r01 * this.rp2X + tf2.r11 * this.rp2Y + tf2.r21 * this.rp2Z;
    this.lp2Z = tf2.r02 * this.rp2X + tf2.r12 * this.rp2Y + tf2.r22 * this.rp2Z;
    this.depth = r.depth;
  }

  /** 复制另一个点。disabled 总是清掉 */
  copyFrom(cp: ManifoldPoint): void {
    this.lp1X = cp.lp1X; this.lp1Y = cp.lp1Y; this.lp1Z = cp.lp1Z;
    this.lp2X = cp.lp2X; this.lp2Y = cp.lp2Y; this.lp2Z = cp.lp2Z;
    this.rp1X = cp.rp1X; this.rp1Y = cp.rp1Y; this.rp1Z = cp.rp1Z;
    this.rp2X = cp.rp2X; this.rp2Y = cp.rp2Y; this.rp2Z = cp.rp2Z;
    this.p1X = cp.p1X; this.p1Y = cp.p1Y; this.p1Z = cp.p1Z;
    this.p2X = cp.p2X; this.p2Y = cp.p2Y; this.p2Z = cp.p2Z;
    this.depth = cp.depth;
    this.impulse.copyFrom(cp.impulse);
    this.id = cp.id;
    this.warmStarted = cp.warmStarted;
    this.disabled = false;
  }
}
