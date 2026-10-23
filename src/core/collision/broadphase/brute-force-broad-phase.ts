// 移植自 OimoPhysics src/oimo/collision/broadphase/bruteforce/BruteForceBroadPhase.hx（MIT，见 Credits.md）
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import type { Aabb } from "../geometry/aabb.js";
import type { ConvexGeometry } from "../geometry/geometry.js";
import { BroadPhase, type BroadPhaseProxyCallback } from "./broad-phase.js";
import { Proxy } from "./proxy.js";

/** 暴力法宽相：每次测试所有代理两两之间的包围盒，O(n²)。包围盒不加厚 */
export class BruteForceBroadPhase extends BroadPhase {
  constructor() {
    super(false);
  }

  createProxy(userData: unknown, aabb: Aabb): Proxy {
    const p = new Proxy(userData, this.idCount++);
    this.addProxy(p);
    p.setAabb(aabb);
    return p;
  }

  destroyProxy(proxy: Proxy): void {
    this.removeProxy(proxy);
    proxy.userData = null;
  }

  moveProxy(proxy: Proxy, aabb: Aabb): void {
    proxy.setAabb(aabb);
  }

  rayCast(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    callback: BroadPhaseProxyCallback,
  ): void {
    // 线段包围盒和方向由所有代理查询共用。
    const sMinX = beginX < endX ? beginX : endX;
    const sMinY = beginY < endY ? beginY : endY;
    const sMinZ = beginZ < endZ ? beginZ : endZ;
    const sMaxX = beginX > endX ? beginX : endX;
    const sMaxY = beginY > endY ? beginY : endY;
    const sMaxZ = beginZ > endZ ? beginZ : endZ;
    const dX = endX - beginX;
    const dY = endY - beginY;
    const dZ = endZ - beginZ;
    const adX = dX < 0 ? -dX : dX;
    const adY = dY < 0 ? -dY : dY;
    const adZ = dZ < 0 ? -dZ : dZ;
    let p = this.proxyList;
    while (p !== null) {
      // 回调里可能删掉当前代理，先记下一个
      const next = p.next;
      if (!(p.minX > sMaxX || p.maxX < sMinX || p.minY > sMaxY || p.maxY < sMinY || p.minZ > sMaxZ || p.maxZ < sMinZ)) {
        // 分离轴测试：线段方向分别叉乘三根坐标轴
        const extX = (p.maxX - p.minX) * 0.5;
        const extY = (p.maxY - p.minY) * 0.5;
        const extZ = (p.maxZ - p.minZ) * 0.5;
        const cpX = beginX - (p.maxX + p.minX) * 0.5;
        const cpY = beginY - (p.maxY + p.minY) * 0.5;
        const cpZ = beginZ - (p.maxZ + p.minZ) * 0.5;
        let s = cpY * dZ - cpZ * dY;
        if (!((s < 0 ? -s : s) - (extY * adZ + extZ * adY) > 0)) {
          s = cpZ * dX - cpX * dZ;
          if (!((s < 0 ? -s : s) - (extZ * adX + extX * adZ) > 0)) {
            s = cpX * dY - cpY * dX;
            if (!((s < 0 ? -s : s) - (extX * adY + extY * adX) > 0)) callback.process(p);
          }
        }
      }
      p = next;
    }
  }

  aabbTest(aabb: Aabb, callback: BroadPhaseProxyCallback): void {
    let p = this.proxyList;
    while (p !== null) {
      const next = p.next;
      if (
        aabb.minX < p.maxX && aabb.maxX > p.minX &&
        aabb.minY < p.maxY && aabb.maxY > p.minY &&
        aabb.minZ < p.maxZ && aabb.maxZ > p.minZ
      ) {
        callback.process(p);
      }
      p = next;
    }
  }

  convexCast(convex: ConvexGeometry, begin: Transform, translation: Vec3, callback: BroadPhaseProxyCallback): void {
    let p = this.proxyList;
    while (p !== null) {
      const next = p.next;
      if (this.aabbConvexSweepTest(p.minX, p.minY, p.minZ, p.maxX, p.maxY, p.maxZ, convex, begin, translation)) callback.process(p);
      p = next;
    }
  }

  collectPairs(): void {
    this.poolProxyPairs();
    this.testCount = 0;
    for (let p1 = this.proxyList; p1 !== null; p1 = p1.next) {
      for (let p2 = p1.next; p2 !== null; p2 = p2.next) {
        this.testCount++;
        if (
          p1.minX < p2.maxX && p1.maxX > p2.minX &&
          p1.minY < p2.maxY && p1.maxY > p2.minY &&
          p1.minZ < p2.maxZ && p1.maxZ > p2.minZ
        ) {
          this.pickAndPushProxyPair(p1, p2);
        }
      }
    }
  }
}
