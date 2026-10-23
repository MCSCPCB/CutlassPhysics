import type { Transform } from "../../common/transform.js";
import type { BoxGeometry } from "../geometry/box-geometry.js";
import type { Geometry } from "../geometry/geometry.js";
import { boxManifold, manifoldNormal, manifoldPoints, manifoldSeparations } from "./compact-box-manifold.js";
import { Detector } from "./detector.js";
import type { DetectorResult } from "./detector-result.js";

/** 软接触盒子检测器：保留面支撑，并在两侧表面生成独立锚点。 */
export class SoftBoxBoxDetector extends Detector {
  private readonly relative = new Array<number>(9).fill(0);
  constructor() { super(false); }

  protected detectImpl(result: DetectorResult, ga: Geometry, gb: Geometry, ta: Transform, tb: Transform, _cache: unknown, margin: number): void {
    result.incremental = false;
    const a = ga as BoxGeometry, b = gb as BoxGeometry, r = this.relative;
    r[0] = ta.r00 * tb.r00 + ta.r10 * tb.r10 + ta.r20 * tb.r20;
    r[1] = ta.r00 * tb.r01 + ta.r10 * tb.r11 + ta.r20 * tb.r21;
    r[2] = ta.r00 * tb.r02 + ta.r10 * tb.r12 + ta.r20 * tb.r22;
    r[3] = ta.r01 * tb.r00 + ta.r11 * tb.r10 + ta.r21 * tb.r20;
    r[4] = ta.r01 * tb.r01 + ta.r11 * tb.r11 + ta.r21 * tb.r21;
    r[5] = ta.r01 * tb.r02 + ta.r11 * tb.r12 + ta.r21 * tb.r22;
    r[6] = ta.r02 * tb.r00 + ta.r12 * tb.r10 + ta.r22 * tb.r20;
    r[7] = ta.r02 * tb.r01 + ta.r12 * tb.r11 + ta.r22 * tb.r21;
    r[8] = ta.r02 * tb.r02 + ta.r12 * tb.r12 + ta.r22 * tb.r22;
    const dx = tb.px - ta.px, dy = tb.py - ta.py, dz = tb.pz - ta.pz;
    const count = boxManifold(a.hx, a.hy, a.hz,
      ta.r00 * dx + ta.r10 * dy + ta.r20 * dz, ta.r01 * dx + ta.r11 * dy + ta.r21 * dz, ta.r02 * dx + ta.r12 * dy + ta.r22 * dz,
      r, b.hx, b.hy, b.hz, margin);
    if (count === 0) return;
    const nx = ta.r00 * manifoldNormal[0] + ta.r01 * manifoldNormal[1] + ta.r02 * manifoldNormal[2];
    const ny = ta.r10 * manifoldNormal[0] + ta.r11 * manifoldNormal[1] + ta.r12 * manifoldNormal[2];
    const nz = ta.r20 * manifoldNormal[0] + ta.r21 * manifoldNormal[1] + ta.r22 * manifoldNormal[2];
    result.nx = -nx; result.ny = -ny; result.nz = -nz;
    result.numPoints = count;
    for (let i = 0; i < count; i++) {
      const p = result.points[i], x = manifoldPoints[i * 3], y = manifoldPoints[i * 3 + 1], z = manifoldPoints[i * 3 + 2];
      const wx = ta.px + ta.r00 * x + ta.r01 * y + ta.r02 * z;
      const wy = ta.py + ta.r10 * x + ta.r11 * y + ta.r12 * z;
      const wz = ta.pz + ta.r20 * x + ta.r21 * y + ta.r22 * z;
      const half = manifoldSeparations[i] * 0.5;
      p.p1x = wx - nx * half; p.p1y = wy - ny * half; p.p1z = wz - nz * half;
      p.p2x = wx + nx * half; p.p2y = wy + ny * half; p.p2z = wz + nz * half;
      p.depth = -manifoldSeparations[i]; p.id = i;
    }
  }
}
