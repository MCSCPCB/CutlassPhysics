// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/SphereBoxDetector.hx（MIT，见 Credits.md）
import type { Transform } from "../../common/transform.js";
import type { BoxGeometry } from "../geometry/box-geometry.js";
import type { Geometry } from "../geometry/geometry.js";
import type { SphereGeometry } from "../geometry/sphere-geometry.js";
import type { DetectorResult } from "./detector-result.js";
import { Detector } from "./detector.js";

/**
 * 球对盒子，最多 1 个接触点。几何体 1 是球、几何体 2 是盒子；swapped 为 true 时处理 (盒子, 球)。
 * 球心在盒子外：取盒子上离球心最近的点。球心在盒子内：从最近的那个面推出去。
 */
export class SphereBoxDetector extends Detector {
  constructor(swapped: boolean) {
    super(swapped);
  }

  protected detectImpl(
    result: DetectorResult, geom1: Geometry, geom2: Geometry, tf1: Transform, tf2: Transform, _cachedData: unknown, margin: number,
  ): void {
    result.incremental = false;
    const b = geom2 as BoxGeometry;
    let halfExtX = b.hx, halfExtY = b.hy, halfExtZ = b.hz;
    let negHalfExtX = -halfExtX, negHalfExtY = -halfExtY, negHalfExtZ = -halfExtZ;
    const r = (geom1 as SphereGeometry).radius;
    // 球心转到盒子的局部坐标
    const wX = tf1.px - tf2.px, wY = tf1.py - tf2.py, wZ = tf1.pz - tf2.pz;
    const sX = tf2.r00 * wX + tf2.r10 * wY + tf2.r20 * wZ;
    const sY = tf2.r01 * wX + tf2.r11 * wY + tf2.r21 * wZ;
    const sZ = tf2.r02 * wX + tf2.r12 * wY + tf2.r22 * wZ;

    if (negHalfExtX < sX && halfExtX > sX && negHalfExtY < sY && halfExtY > sY && negHalfExtZ < sZ && halfExtZ > sZ) {
      // 球心在盒子内部：到三对面的距离里取最小的，沿那个面的法线推出去
      const distX = halfExtX - (sX < 0 ? -sX : sX);
      const distY = halfExtY - (sY < 0 ? -sY : sY);
      const distZ = halfExtZ - (sZ < 0 ? -sZ : sZ);
      let nbX: number, nbY: number, nbZ: number;
      let maskX: number, maskY: number, maskZ: number;
      let depth: number;
      if (distX < distY) {
        if (distX < distZ) {
          nbX = sX > 0 ? 1 : -1; nbY = 0; nbZ = 0;
          maskX = 0; maskY = 1; maskZ = 1;
          depth = distX;
        } else {
          nbX = 0; nbY = 0; nbZ = sZ > 0 ? 1 : -1;
          maskX = 1; maskY = 1; maskZ = 0;
          depth = distZ;
        }
      } else if (distY < distZ) {
        nbX = 0; nbY = sY > 0 ? 1 : -1; nbZ = 0;
        maskX = 1; maskY = 0; maskZ = 1;
        depth = distY;
      } else {
        nbX = 0; nbY = 0; nbZ = sZ > 0 ? 1 : -1;
        maskX = 1; maskY = 1; maskZ = 0;
        depth = distZ;
      }
      // 盒子上的接触点：球心投影到那个面上
      let cbX = nbX * halfExtX, cbY = nbY * halfExtY, cbZ = nbZ * halfExtZ;
      cbX += maskX * sX;
      cbY += maskY * sY;
      cbZ += maskZ * sZ;
      const cX = tf2.r00 * cbX + tf2.r01 * cbY + tf2.r02 * cbZ;
      const cY = tf2.r10 * cbX + tf2.r11 * cbY + tf2.r12 * cbZ;
      const cZ = tf2.r20 * cbX + tf2.r21 * cbY + tf2.r22 * cbZ;
      const nX = tf2.r00 * nbX + tf2.r01 * nbY + tf2.r02 * nbZ;
      const nY = tf2.r10 * nbX + tf2.r11 * nbY + tf2.r12 * nbZ;
      const nZ = tf2.r20 * nbX + tf2.r21 * nbY + tf2.r22 * nbZ;
      this.setNormal(result, nX, nY, nZ);
      this.addPoint(
        result,
        tf1.px + nX * -r, tf1.py + nY * -r, tf1.pz + nZ * -r,
        tf2.px + cX, tf2.py + cY, tf2.pz + cZ,
        depth,
        0,
      );
      return;
    }

    // 球心在盒子外（或正好在表面上）：把球心夹到稍微缩小一点的盒子里，得到最近点
    halfExtX -= 1e-9;
    halfExtY -= 1e-9;
    halfExtZ -= 1e-9;
    negHalfExtX += 1e-9;
    negHalfExtY += 1e-9;
    negHalfExtZ += 1e-9;
    let cbX = sX < halfExtX ? sX : halfExtX;
    let cbY = sY < halfExtY ? sY : halfExtY;
    let cbZ = sZ < halfExtZ ? sZ : halfExtZ;
    if (!(cbX > negHalfExtX)) cbX = negHalfExtX;
    if (!(cbY > negHalfExtY)) cbY = negHalfExtY;
    if (!(cbZ > negHalfExtZ)) cbZ = negHalfExtZ;
    const csX = sX - cbX, csY = sY - cbY, csZ = sZ - cbZ;
    let dist = csX * csX + csY * csY + csZ * csZ;
    // 检测距离包含预测接触余量 margin。
    const reach = r + margin;
    if (dist >= reach * reach) return;
    dist = Math.sqrt(dist);
    const cX = tf2.r00 * cbX + tf2.r01 * cbY + tf2.r02 * cbZ;
    const cY = tf2.r10 * cbX + tf2.r11 * cbY + tf2.r12 * cbZ;
    const cZ = tf2.r20 * cbX + tf2.r21 * cbY + tf2.r22 * cbZ;
    const wsX = tf2.r00 * csX + tf2.r01 * csY + tf2.r02 * csZ;
    const wsY = tf2.r10 * csX + tf2.r11 * csY + tf2.r12 * csZ;
    const wsZ = tf2.r20 * csX + tf2.r21 * csY + tf2.r22 * csZ;
    let l = wsX * wsX + wsY * wsY + wsZ * wsZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    const nX = wsX * l, nY = wsY * l, nZ = wsZ * l;
    this.setNormal(result, nX, nY, nZ);
    this.addPoint(
      result,
      tf1.px + nX * -r, tf1.py + nY * -r, tf1.pz + nZ * -r,
      tf2.px + cX, tf2.py + cY, tf2.pz + cZ,
      r - dist,
      0,
    );
  }
}
