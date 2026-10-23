// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/SphereSphereDetector.hx（MIT，见 Credits.md）
import type { Transform } from "../../common/transform.js";
import type { Geometry } from "../geometry/geometry.js";
import type { SphereGeometry } from "../geometry/sphere-geometry.js";
import type { DetectorResult } from "./detector-result.js";
import { Detector } from "./detector.js";

/** 球对球：沿两个球心的连线最多 1 个接触点 */
export class SphereSphereDetector extends Detector {
  constructor() {
    super(false);
  }

  protected detectImpl(
    result: DetectorResult, geom1: Geometry, geom2: Geometry, tf1: Transform, tf2: Transform, _cachedData: unknown, margin: number,
  ): void {
    result.incremental = false;
    const dX = tf1.px - tf2.px;
    const dY = tf1.py - tf2.py;
    const dZ = tf1.pz - tf2.pz;
    const r1 = (geom1 as SphereGeometry).radius;
    const r2 = (geom2 as SphereGeometry).radius;
    const len2 = dX * dX + dY * dY + dZ * dZ;
    // 检测距离包含预测接触余量 margin。
    const reach = r1 + r2 + margin;
    if (len2 >= reach * reach) return;
    const len = Math.sqrt(len2);
    // 法线从球 2 指向球 1；球心重合时取 +x
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
      tf1.px + nX * -r1, tf1.py + nY * -r1, tf1.pz + nZ * -r1,
      tf2.px + nX * r2, tf2.py + nY * r2, tf2.pz + nZ * r2,
      r1 + r2 - len,
      0,
    );
  }
}
