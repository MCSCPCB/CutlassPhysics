// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/Detector.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { Transform } from "../../common/transform.js";
import type { Geometry } from "../geometry/geometry.js";
import type { CachedDetectorData } from "./cached-detector-data.js";
import type { DetectorResult } from "./detector-result.js";

/** 窄相检测器基类。swapped 启用时交换输入几何体，输出时还原法线和接触点顺序。 */
export abstract class Detector {
  protected readonly swapped: boolean;

  protected constructor(swapped: boolean) {
    this.swapped = swapped;
  }

  /**
   * 计算接触流形并写入 result，cachedData 保存 GJK/EPA 的跨步缓存。
   * margin 默认为 0；分离距离小于 margin 时生成负深度的预测接触点。
   */
  detect(
    result: DetectorResult, geom1: Geometry, geom2: Geometry, tf1: Transform, tf2: Transform, cachedData: CachedDetectorData | null,
    margin = 0,
  ): void {
    result.clear();
    if (this.swapped) {
      this.detectImpl(result, geom2, geom1, tf2, tf1, cachedData, margin);
    } else {
      this.detectImpl(result, geom1, geom2, tf1, tf2, cachedData, margin);
    }
  }

  protected abstract detectImpl(
    result: DetectorResult,
    geom1: Geometry,
    geom2: Geometry,
    tf1: Transform,
    tf2: Transform,
    cachedData: CachedDetectorData | null,
    margin: number,
  ): void;

  protected setNormal(result: DetectorResult, nx: number, ny: number, nz: number): void {
    if (this.swapped) {
      result.nx = -nx;
      result.ny = -ny;
      result.nz = -nz;
    } else {
      result.nx = nx;
      result.ny = ny;
      result.nz = nz;
    }
  }

  protected addPoint(
    result: DetectorResult,
    x1: number, y1: number, z1: number,
    x2: number, y2: number, z2: number,
    depth: number,
    id: number,
  ): void {
    const p = result.points[result.numPoints++];
    p.depth = depth;
    p.id = id;
    if (this.swapped) {
      p.p1x = x2;
      p.p1y = y2;
      p.p1z = z2;
      p.p2x = x1;
      p.p2y = y1;
      p.p2z = z1;
    } else {
      p.p1x = x1;
      p.p1y = y1;
      p.p1z = z1;
      p.p2x = x2;
      p.p2y = y2;
      p.p2z = z2;
    }
  }
}
