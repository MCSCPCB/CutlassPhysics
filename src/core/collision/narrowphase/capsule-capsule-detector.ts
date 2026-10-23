// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/CapsuleCapsuleDetector.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { Transform } from "../../common/transform.js";
import type { CapsuleGeometry } from "../geometry/capsule-geometry.js";
import type { Geometry } from "../geometry/geometry.js";
import type { DetectorResult } from "./detector-result.js";
import { Detector } from "./detector.js";

/**
 * 胶囊体对胶囊体：求两条中轴线段之间的最近点，再按球对球处理，最多 1 个接触点。
 * 线段 1 是 p1 + t1·d1，线段 2 是 p2 + t2·d2，t1、t2 在 [0, 1] 内。
 */
export class CapsuleCapsuleDetector extends Detector {
  constructor() {
    super(false);
  }

  protected detectImpl(
    result: DetectorResult, geom1: Geometry, geom2: Geometry, tf1: Transform, tf2: Transform, _cachedData: unknown, margin: number,
  ): void {
    result.incremental = false;
    const c1 = geom1 as CapsuleGeometry;
    const c2 = geom2 as CapsuleGeometry;
    const axis1X = tf1.r01, axis1Y = tf1.r11, axis1Z = tf1.r21;
    const axis2X = tf2.r01, axis2Y = tf2.r11, axis2Z = tf2.r21;
    const hh1 = c1.halfHeight;
    const hh2 = c2.halfHeight;
    const r1 = c1.radius;
    const r2 = c2.radius;
    const p1X = tf1.px + axis1X * -hh1;
    const p1Y = tf1.py + axis1Y * -hh1;
    const p1Z = tf1.pz + axis1Z * -hh1;
    const q1X = tf1.px + axis1X * hh1;
    const q1Y = tf1.py + axis1Y * hh1;
    const q1Z = tf1.pz + axis1Z * hh1;
    const p2X = tf2.px + axis2X * -hh2;
    const p2Y = tf2.py + axis2Y * -hh2;
    const p2Z = tf2.pz + axis2Z * -hh2;
    const q2X = tf2.px + axis2X * hh2;
    const q2Y = tf2.py + axis2Y * hh2;
    const q2Z = tf2.pz + axis2Z * hh2;
    const p12X = p1X - p2X, p12Y = p1Y - p2Y, p12Z = p1Z - p2Z;
    const d1X = q1X - p1X, d1Y = q1Y - p1Y, d1Z = q1Z - p1Z;
    const d2X = q2X - p2X, d2Y = q2Y - p2Y, d2Z = q2Z - p2Z;
    const p21d1 = -(p12X * d1X + p12Y * d1Y + p12Z * d1Z);
    const p12d2 = p12X * d2X + p12Y * d2Y + p12Z * d2Z;
    const d11 = hh1 * hh1 * 4;
    const d12 = d1X * d2X + d1Y * d2Y + d1Z * d2Z;
    const d22 = hh2 * hh2 * 4;

    // 两条线段的最近点参数。先不加限制求解，再把越界的一边夹到端点，重新求另一边
    let t1: number, t2: number;
    if (d11 === 0 && d22 === 0) {
      // 两个都退化成球
      t1 = 0;
      t2 = 0;
    } else if (d11 === 0) {
      t1 = 0;
      if (p12d2 < 0) t2 = 0;
      else if (p12d2 > d22) t2 = 1;
      else t2 = p12d2 / d22;
    } else if (d22 === 0) {
      t2 = 0;
      if (p21d1 < 0) t1 = 0;
      else if (p21d1 > d11) t1 = 1;
      else t1 = p21d1 / d11;
    } else {
      const det = d11 * d22 - d12 * d12;
      if (det === 0) {
        // 平行：线段 1 取起点
        t1 = 0;
      } else {
        t1 = d12 * p12d2 + d22 * p21d1;
        if (t1 < 0) t1 = 0;
        else if (t1 > det) t1 = 1;
        else t1 /= det;
      }
      t2 = t1 * d12 + p12d2;
      if (t2 < 0) {
        t2 = 0;
        if (p21d1 < 0) t1 = 0;
        else if (p21d1 > d11) t1 = 1;
        else t1 = p21d1 / d11;
      } else if (t2 > d22) {
        t2 = 1;
        t1 = d12 + p21d1;
        if (t1 < 0) t1 = 0;
        else if (t1 > d11) t1 = 1;
        else t1 /= d11;
      } else {
        t2 /= d22;
      }
    }

    const cp1X = p1X + d1X * t1, cp1Y = p1Y + d1Y * t1, cp1Z = p1Z + d1Z * t1;
    const cp2X = p2X + d2X * t2, cp2Y = p2Y + d2Y * t2, cp2Z = p2Z + d2Z * t2;
    const dX = cp1X - cp2X, dY = cp1Y - cp2Y, dZ = cp1Z - cp2Z;
    const len2 = dX * dX + dY * dY + dZ * dZ;
    // 检测距离包含预测接触余量 margin。
    const reach = r1 + r2 + margin;
    if (len2 >= reach * reach) return;
    const len = Math.sqrt(len2);
    let nX: number, nY: number, nZ: number;
    if (len > 0) {
      nX = dX * (1 / len);
      nY = dY * (1 / len);
      nZ = dZ * (1 / len);
    } else {
      nX = 1;
      nY = 0;
      nZ = 0;
    }
    this.setNormal(result, nX, nY, nZ);
    this.addPoint(
      result,
      cp1X + nX * -r1, cp1Y + nY * -r1, cp1Z + nZ * -r1,
      cp2X + nX * r2, cp2Y + nY * r2, cp2Z + nZ * r2,
      r1 + r2 - len,
      0,
    );
  }
}
