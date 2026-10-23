// 移植自 OimoPhysics src/oimo/collision/narrowphase/{DetectorResult,DetectorResultPoint}.hx（MIT，见 Credits.md）
import { MAX_MANIFOLD_POINTS } from "../../settings.js";

/** 一个接触点：两个几何体上各自的最近点（世界坐标）和重叠深度 */
export class DetectorResultPoint {
  /** 几何体 1 上的点 */
  p1x = 0;
  p1y = 0;
  p1z = 0;
  /** 几何体 2 上的点 */
  p2x = 0;
  p2y = 0;
  p2z = 0;
  /** 重叠深度，分离时为负 */
  depth = 0;
  /** 接触点编号，帧间据此认出同一个接触点 */
  id = 0;
}

/** 窄相检测结果。只有前 numPoints 个点有效 */
export class DetectorResult {
  numPoints = 0;
  readonly points: DetectorResultPoint[] = [];
  /** 接触法线（单位向量），从几何体 2 指向几何体 1 */
  nx = 0;
  ny = 0;
  nz = 0;
  /**
   * true：每次只给出 1 个接触点，要和流形里已有的点合并（GJK/EPA）。
   * false：每次给出完整的流形，直接替换。由检测器设置，clear() 不动它
   */
  incremental = false;

  constructor() {
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.points.push(new DetectorResultPoint());
  }

  /** 所有接触点里最大的深度，没有接触点时为 0 */
  getMaxDepth(): number {
    let max = 0;
    for (let i = 0; i < this.numPoints; i++) {
      if (this.points[i].depth > max) max = this.points[i].depth;
    }
    return max;
  }

  clear(): void {
    this.numPoints = 0;
    for (let i = 0; i < this.points.length; i++) {
      const p = this.points[i];
      p.p1x = 0;
      p.p1y = 0;
      p.p1z = 0;
      p.p2x = 0;
      p.p2y = 0;
      p.p2z = 0;
      p.depth = 0;
      p.id = 0;
    }
    this.nx = 0;
    this.ny = 0;
    this.nz = 0;
  }
}
