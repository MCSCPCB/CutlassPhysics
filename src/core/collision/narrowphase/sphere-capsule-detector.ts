// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/SphereCapsuleDetector.hx（MIT，见 Credits.md）
import { compat } from "../../compat.js";
import type { Transform } from "../../common/transform.js";
import type { CapsuleGeometry } from "../geometry/capsule-geometry.js";
import type { Geometry } from "../geometry/geometry.js";
import type { SphereGeometry } from "../geometry/sphere-geometry.js";
import type { DetectorResult } from "./detector-result.js";
import { Detector } from "./detector.js";

/**
 * 球对胶囊体：在胶囊体的中轴线段上找离球心最近的点，再按球对球处理，最多 1 个接触点。
 * 几何体 1 是球、几何体 2 是胶囊体；swapped 为 true 时处理 (胶囊体, 球)。
 */
export class SphereCapsuleDetector extends Detector {
  constructor(swapped: boolean) {
    super(swapped);
  }

  protected detectImpl(
    result: DetectorResult, geom1: Geometry, geom2: Geometry, tf1: Transform, tf2: Transform, _cachedData: unknown, margin: number,
  ): void {
    result.incremental = false;
    const c2 = geom2 as CapsuleGeometry;
    const hh2 = c2.halfHeight;
    const r1 = (geom1 as SphereGeometry).radius;
    const r2 = c2.radius;
    // 胶囊体中轴：局部 y 轴，两端是 p2、q2
    const axis2X = tf2.r01, axis2Y = tf2.r11, axis2Z = tf2.r21;
    const cp1X = tf1.px, cp1Y = tf1.py, cp1Z = tf1.pz;
    const p2X = tf2.px + axis2X * -hh2;
    const p2Y = tf2.py + axis2Y * -hh2;
    const p2Z = tf2.pz + axis2Z * -hh2;
    const q2X = tf2.px + axis2X * hh2;
    const q2Y = tf2.py + axis2Y * hh2;
    const q2Z = tf2.pz + axis2Z * hh2;
    const p12X = cp1X - p2X, p12Y = cp1Y - p2Y, p12Z = cp1Z - p2Z;
    const d2X = q2X - p2X, d2Y = q2Y - p2Y, d2Z = q2Z - p2Z;
    // 球心投影到线段上的参数，夹到 [0, 1]
    // 高度为 0 的胶囊退化为球，投影参数取 0。
    const d22 = hh2 * hh2 * 4;
    let t = p12X * d2X + p12Y * d2Y + p12Z * d2Z;
    if (t < 0) {
      t = 0;
    } else if (t > d22) {
      t = 1;
    } else if (d22 > 0 || compat.keepOimoBugs) {
      t /= d22;
    } else {
      t = 0;
    }
    const cp2X = p2X + d2X * t;
    const cp2Y = p2Y + d2Y * t;
    const cp2Z = p2Z + d2Z * t;
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
