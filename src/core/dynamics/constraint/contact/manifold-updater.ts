// 移植自 OimoPhysics src/oimo/dynamics/constraint/contact/ManifoldUpdater.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { DetectorResult, DetectorResultPoint } from "../../../collision/narrowphase/detector-result.js";
import type { Transform } from "../../../common/transform.js";
import { CONTACT_PERSISTENCE_THRESHOLD, MAX_MANIFOLD_POINTS } from "../../../settings.js";
import type { Manifold } from "./manifold.js";
import { ContactImpulse } from "./contact-impulse.js";

/**
 * 四个点组成的四边形面积的平方（的 4 倍），不区分顶点顺序：
 * 三种对角线组合（12-34、13-24、14-23）的叉积取最大的
 */
function quadAreaFast(
  p1X: number, p1Y: number, p1Z: number, p2X: number, p2Y: number, p2Z: number,
  p3X: number, p3Y: number, p3Z: number, p4X: number, p4Y: number, p4Z: number,
): number {
  const v12X = p2X - p1X, v12Y = p2Y - p1Y, v12Z = p2Z - p1Z;
  const v34X = p4X - p3X, v34Y = p4Y - p3Y, v34Z = p4Z - p3Z;
  const v13X = p3X - p1X, v13Y = p3Y - p1Y, v13Z = p3Z - p1Z;
  const v24X = p4X - p2X, v24Y = p4Y - p2Y, v24Z = p4Z - p2Z;
  const v14X = p4X - p1X, v14Y = p4Y - p1Y, v14Z = p4Z - p1Z;
  const v23X = p3X - p2X, v23Y = p3Y - p2Y, v23Z = p3Z - p2Z;
  const c1X = v12Y * v34Z - v12Z * v34Y, c1Y = v12Z * v34X - v12X * v34Z, c1Z = v12X * v34Y - v12Y * v34X;
  const c2X = v13Y * v24Z - v13Z * v24Y, c2Y = v13Z * v24X - v13X * v24Z, c2Z = v13X * v24Y - v13Y * v24X;
  const c3X = v14Y * v23Z - v14Z * v23Y, c3Y = v14Z * v23X - v14X * v23Z, c3Z = v14X * v23Y - v14Y * v23X;
  const a1 = c1X * c1X + c1Y * c1Y + c1Z * c1Z;
  const a2 = c2X * c2X + c2Y * c2Y + c2Z * c2Z;
  const a3 = c3X * c3X + c3Y * c3Y + c3Z * c3Z;
  if (a1 > a2) return a1 > a3 ? a1 : a3;
  return a2 > a3 ? a2 : a3;
}

/** 用窄相检测结果更新接触流形，并把上一步同编号接触点的冲量接过来（热启动） */
export class ManifoldUpdater {
  private readonly manifold: Manifold;
  private readonly oldIds: number[] = [];
  private readonly oldImpulses: ContactImpulse[] = [];

  constructor(manifold: Manifold) {
    this.manifold = manifold;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) {
      this.oldIds.push(-1);
      this.oldImpulses.push(new ContactImpulse());
    }
  }

  /** 一次性更新：整个流形换成新的检测结果 */
  totalUpdate(result: DetectorResult, tf1: Transform, tf2: Transform): void {
    const m = this.manifold;
    const numOldPoints = m.numPoints;
    const num = result.numPoints;
    // 编号严格递增且顺序未变化时，直接复用对应槽位的冲量。
    let unchanged = num === numOldPoints;
    for (let i = 0; unchanged && i < num; i++) {
      const id = result.points[i].id;
      if (id !== m.points[i].id || (i > 0 && id <= result.points[i - 1].id)) unchanged = false;
    }
    if (unchanged) {
      for (let i = 0; i < num; i++) {
        const p = m.points[i];
        p.updateDepthAndPositions(result.points[i], tf1, tf2);
        // 位置修正冲量不参与热启动；initialize 原本会把它清零。
        p.impulse.impulseP = 0;
        p.id = result.points[i].id;
        p.warmStarted = true;
        p.disabled = false;
      }
      return;
    }
    // 热启动仅缓存旧编号和冲量，位置由新检测结果重建。
    for (let i = 0; i < numOldPoints; i++) {
      this.oldIds[i] = m.points[i].id;
      this.oldImpulses[i].copyFrom(m.points[i].impulse);
    }

    m.numPoints = num;
    for (let i = 0; i < num; i++) {
      const p = m.points[i];
      p.initialize(result.points[i], tf1, tf2);
      // 编号相同的旧接触点：接过它的冲量
      for (let k = 0; k < numOldPoints; k++) {
        if (p.id === this.oldIds[k]) {
          p.impulse.copyFrom(this.oldImpulses[k]);
          p.warmStarted = true;
          break;
        }
      }
    }
  }

  /**
   * 增量更新接触流形：相近点替换，新增点加入，分离或切向偏移过大的点移除。
   * 流形最多保留 4 个点，法向分离阈值包含预测接触余量 margin。
   */
  incrementalUpdate(result: DetectorResult, tf1: Transform, tf2: Transform, margin = 0): void {
    const m = this.manifold;
    m.updateDepthsAndPositions(tf1, tf2);
    for (let i = 0; i < m.numPoints; i++) m.points[i].warmStarted = true;
    const newPoint = result.points[0];
    const index = this.findNearestContactPointIndex(newPoint, tf1, tf2);
    if (index === -1) {
      this.addManifoldPoint(newPoint, tf1, tf2);
    } else {
      m.points[index].updateDepthAndPositions(newPoint, tf1, tf2);
    }
    this.removeOutdatedPoints(margin);
  }

  /** 删掉沿法线分开超过阈值、或者沿切向滑开超过阈值的点 */
  private removeOutdatedPoints(margin: number): void {
    const m = this.manifold;
    let index = m.numPoints;
    while (--index >= 0) {
      const p = m.points[index];
      let dX = p.p1X - p.p2X, dY = p.p1Y - p.p2Y, dZ = p.p1Z - p.p2Z;
      const dotN = m.nX * dX + m.nY * dY + m.nZ * dZ;
      if (dotN > CONTACT_PERSISTENCE_THRESHOLD + margin) {
        this.removeManifoldPoint(index);
        continue;
      }
      // 去掉法向分量，剩下的是切向滑动
      dX += m.nX * -dotN;
      dY += m.nY * -dotN;
      dZ += m.nZ * -dotN;
      if (dX * dX + dY * dY + dZ * dZ > CONTACT_PERSISTENCE_THRESHOLD * CONTACT_PERSISTENCE_THRESHOLD) {
        this.removeManifoldPoint(index);
        continue;
      }
    }
  }

  /** 把第 index 个点和最后一个点交换，再删掉最后一个 */
  private removeManifoldPoint(index: number): void {
    const m = this.manifold;
    const lastIndex = --m.numPoints;
    if (index !== lastIndex) {
      const tmp = m.points[index];
      m.points[index] = m.points[lastIndex];
      m.points[lastIndex] = tmp;
    }
    m.points[lastIndex].clear();
  }

  private addManifoldPoint(point: DetectorResultPoint, tf1: Transform, tf2: Transform): void {
    const m = this.manifold;
    const num = m.numPoints;
    if (num === MAX_MANIFOLD_POINTS) {
      // 满了：替换掉一个，让剩下的点围成的面积最大
      m.points[this.computeTargetIndex(point, tf1)].initialize(point, tf1, tf2);
      return;
    }
    m.points[num].initialize(point, tf1, tf2);
    m.numPoints++;
  }

  /** 流形满时保留最深点，并选择替换后四边形面积最大的组合。 */
  private computeTargetIndex(newPoint: DetectorResultPoint, tf1: Transform): number {
    const ps = this.manifold.points;
    const p1 = ps[0], p2 = ps[1], p3 = ps[2], p4 = ps[3];
    let maxDepth = p1.depth;
    let maxDepthIndex = 0;
    if (p2.depth > maxDepth) {
      maxDepth = p2.depth;
      maxDepthIndex = 1;
    }
    if (p3.depth > maxDepth) {
      maxDepth = p3.depth;
      maxDepthIndex = 2;
    }
    if (p4.depth > maxDepth) maxDepthIndex = 3;
    const rX = newPoint.p1x - tf1.px, rY = newPoint.p1y - tf1.py, rZ = newPoint.p1z - tf1.pz;
    const a1 = quadAreaFast(p2.rp1X, p2.rp1Y, p2.rp1Z, p3.rp1X, p3.rp1Y, p3.rp1Z, p4.rp1X, p4.rp1Y, p4.rp1Z, rX, rY, rZ);
    const a2 = quadAreaFast(p1.rp1X, p1.rp1Y, p1.rp1Z, p3.rp1X, p3.rp1Y, p3.rp1Z, p4.rp1X, p4.rp1Y, p4.rp1Z, rX, rY, rZ);
    const a3 = quadAreaFast(p1.rp1X, p1.rp1Y, p1.rp1Z, p2.rp1X, p2.rp1Y, p2.rp1Z, p4.rp1X, p4.rp1Y, p4.rp1Z, rX, rY, rZ);
    const a4 = quadAreaFast(p1.rp1X, p1.rp1Y, p1.rp1Z, p2.rp1X, p2.rp1Y, p2.rp1Z, p3.rp1X, p3.rp1Y, p3.rp1Z, rX, rY, rZ);
    let max = a1;
    let target = 0;
    if ((a2 > max && maxDepthIndex !== 1) || maxDepthIndex === 0) {
      max = a2;
      target = 1;
    }
    if (a3 > max && maxDepthIndex !== 2) {
      max = a3;
      target = 2;
    }
    if (a4 > max && maxDepthIndex !== 3) target = 3;
    return target;
  }

  /** 流形里离新点最近（两个刚体上的距离取较小的）且在保留阈值内的点，没有返回 -1 */
  private findNearestContactPointIndex(target: DetectorResultPoint, tf1: Transform, tf2: Transform): number {
    const m = this.manifold;
    let nearestSq = CONTACT_PERSISTENCE_THRESHOLD * CONTACT_PERSISTENCE_THRESHOLD;
    let idx = -1;
    for (let i = 0; i < m.numPoints; i++) {
      const mp = m.points[i];
      const rp1X = target.p1x - tf1.px, rp1Y = target.p1y - tf1.py, rp1Z = target.p1z - tf1.pz;
      const rp2X = target.p2x - tf2.px, rp2Y = target.p2y - tf2.py, rp2Z = target.p2z - tf2.pz;
      const d1X = mp.rp1X - rp1X, d1Y = mp.rp1Y - rp1Y, d1Z = mp.rp1Z - rp1Z;
      const d2X = mp.rp2X - rp2X, d2Y = mp.rp2Y - rp2Y, d2Z = mp.rp2Z - rp2Z;
      const sq1 = d1X * d1X + d1Y * d1Y + d1Z * d1Z;
      const sq2 = d2X * d2X + d2Y * d2Y + d2Z * d2Z;
      const d = sq1 < sq2 ? sq1 : sq2;
      if (d < nearestSq) {
        nearestSq = d;
        idx = i;
      }
    }
    return idx;
  }
}
