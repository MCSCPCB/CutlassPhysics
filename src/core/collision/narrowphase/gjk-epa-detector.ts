// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/GjkEpaDetector.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { Transform } from "../../common/transform.js";
import { ENABLE_GJK_CACHING } from "../../settings.js";
import type { ConvexGeometry, Geometry } from "../geometry/geometry.js";
import type { CachedDetectorData } from "./cached-detector-data.js";
import type { DetectorResult } from "./detector-result.js";
import { Detector } from "./detector.js";
import { GjkEpa, GjkEpaResultState } from "./gjkepa/gjk-epa.js";

/**
 * 使用 GJK/EPA 计算凸体核心的最近点，并计入两侧外边距。
 * 每次生成 1 个接触点，由 ManifoldUpdater 增量合并。
 */
export class GjkEpaDetector extends Detector {
  constructor() {
    super(false);
  }

  protected detectImpl(
    result: DetectorResult, geom1: Geometry, geom2: Geometry, tf1: Transform, tf2: Transform, cachedData: CachedDetectorData | null,
    margin: number,
  ): void {
    const gjkEpa = GjkEpa.getInstance();
    const g1 = geom1 as ConvexGeometry, g2 = geom2 as ConvexGeometry;
    const status = gjkEpa.computeClosestPoints(g1, g2, tf1, tf2, ENABLE_GJK_CACHING ? cachedData : null);
    result.incremental = true;
    // 检测失败时本步无接触。
    if (status !== GjkEpaResultState.SUCCEEDED) return;
    // 核心距离超过外边距与 margin 之和时无接触；预测接触深度为负。
    if (gjkEpa.distance > g1.gjkMargin + g2.gjkMargin + margin) return;
    let pos1X = gjkEpa.closestPoint1.x, pos1Y = gjkEpa.closestPoint1.y, pos1Z = gjkEpa.closestPoint1.z;
    let pos2X = gjkEpa.closestPoint2.x, pos2Y = gjkEpa.closestPoint2.y, pos2Z = gjkEpa.closestPoint2.z;
    let nX = pos1X - pos2X, nY = pos1Y - pos2Y, nZ = pos1Z - pos2Z;
    // 两个核心恰好接触，没法确定法线
    if (nX * nX + nY * nY + nZ * nZ === 0) return;
    // 重叠时最近点的连线方向是反的
    if (gjkEpa.distance < 0) {
      nX = -nX;
      nY = -nY;
      nZ = -nZ;
    }
    let l = nX * nX + nY * nY + nZ * nZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    nX *= l;
    nY *= l;
    nZ *= l;
    this.setNormal(result, nX, nY, nZ);
    // 最近点从核心表面移到几何体表面
    pos1X += nX * -g1.gjkMargin;
    pos1Y += nY * -g1.gjkMargin;
    pos1Z += nZ * -g1.gjkMargin;
    pos2X += nX * g2.gjkMargin;
    pos2Y += nY * g2.gjkMargin;
    pos2Z += nZ * g2.gjkMargin;
    this.addPoint(result, pos1X, pos1Y, pos1Z, pos2X, pos2Y, pos2Z, g1.gjkMargin + g2.gjkMargin - gjkEpa.distance, 0);
  }
}
