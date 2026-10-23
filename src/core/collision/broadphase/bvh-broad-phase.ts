// 移植自 OimoPhysics src/oimo/collision/broadphase/bvh/BvhBroadPhase.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import { compat } from "../../compat.js";
import { BVH_INCREMENTAL_COLLISION_THRESHOLD, BVH_PROXY_PADDING } from "../../settings.js";
import type { Aabb } from "../geometry/aabb.js";
import type { ConvexGeometry } from "../geometry/geometry.js";
import { BroadPhase, type BroadPhaseProxyCallback } from "./broad-phase.js";
import type { Proxy } from "./proxy.js";
import { type BvhNode, BvhProxy, BvhTree } from "./bvh-tree.js";

/**
 * 基于包围体层次树的增量宽相，平均复杂度为 O(n log n)。
 * 代理超出加厚包围盒时重新插入树，仅为这些代理查询新的重叠对。
 */
export class BvhBroadPhase extends BroadPhase {
  readonly tree = new BvhTree();
  private movedProxies: (BvhProxy | null)[] = [];
  private numMovedProxies = 0;

  constructor() {
    super(true);
  }

  createProxy(userData: unknown, aabb: Aabb): Proxy {
    const p = new BvhProxy(userData, this.idCount++);
    this.addProxy(p);
    // 加厚：四周加一圈余量（新建时没有位移）
    p.minX = aabb.minX - BVH_PROXY_PADDING;
    p.minY = aabb.minY - BVH_PROXY_PADDING;
    p.minZ = aabb.minZ - BVH_PROXY_PADDING;
    p.maxX = aabb.maxX + BVH_PROXY_PADDING;
    p.maxY = aabb.maxY + BVH_PROXY_PADDING;
    p.maxZ = aabb.maxZ + BVH_PROXY_PADDING;
    this.tree.insertProxy(p);
    this.addToMovedProxies(p);
    return p;
  }

  destroyProxy(proxy: Proxy): void {
    this.removeProxy(proxy);
    const p = proxy as BvhProxy;
    this.tree.deleteProxy(p);
    p.userData = null;
    p.next = null;
    p.prev = null;
    // 可能还留在 movedProxies 里，清掉标记，collectPairs() 时会跳过
    p.moved = false;
  }

  moveProxy(proxy: Proxy, aabb: Aabb, dx: number, dy: number, dz: number): void {
    const p = proxy as BvhProxy;
    // 形状仍在加厚包围盒内，保留树节点。
    if (
      p.minX <= aabb.minX && p.maxX >= aabb.maxX &&
      p.minY <= aabb.minY && p.maxY >= aabb.maxY &&
      p.minZ <= aabb.minZ && p.maxZ >= aabb.maxZ
    ) {
      return;
    }
    // 重新加厚：先加一圈余量，再沿位移方向延伸，预测下一步的运动
    p.minX = aabb.minX - BVH_PROXY_PADDING;
    p.minY = aabb.minY - BVH_PROXY_PADDING;
    p.minZ = aabb.minZ - BVH_PROXY_PADDING;
    p.maxX = aabb.maxX + BVH_PROXY_PADDING;
    p.maxY = aabb.maxY + BVH_PROXY_PADDING;
    p.maxZ = aabb.maxZ + BVH_PROXY_PADDING;
    p.minX += 0 < dx ? 0 : dx;
    p.minY += 0 < dy ? 0 : dy;
    p.minZ += 0 < dz ? 0 : dz;
    p.maxX += 0 > dx ? 0 : dx;
    p.maxY += 0 > dy ? 0 : dy;
    p.maxZ += 0 > dz ? 0 : dz;
    this.addToMovedProxies(p);
  }

  collectPairs(): void {
    this.poolProxyPairs();
    this.testCount = 0;
    if (this.numProxies < 2) {
      // 单代理仍需更新树中位置，供射线和范围查询使用。
      if (!compat.keepOimoBugs) this.reinsertMovedProxies();
      return;
    }
    const tree = this.tree;
    const incremental = this.numMovedProxies / this.numProxies < BVH_INCREMENTAL_COLLISION_THRESHOLD;
    for (let i = 0; i < this.numMovedProxies; i++) {
      const p = this.movedProxies[i]!;
      if (p.moved) {
        tree.deleteProxy(p);
        tree.insertProxy(p);
        if (incremental) this.collide(tree.root!, p.leaf!);
        p.moved = false;
      }
      this.movedProxies[i] = null;
    }
    if (!incremental) this.collide(tree.root!, tree.root!);
    this.numMovedProxies = 0;
  }

  /** 只有一个代理时用：把移动过的代理在树里更新位置，不找重叠对 */
  private reinsertMovedProxies(): void {
    const tree = this.tree;
    for (let i = 0; i < this.numMovedProxies; i++) {
      const p = this.movedProxies[i]!;
      if (p.moved) {
        tree.deleteProxy(p);
        tree.insertProxy(p);
        p.moved = false;
      }
      this.movedProxies[i] = null;
    }
    this.numMovedProxies = 0;
  }

  rayCast(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    callback: BroadPhaseProxyCallback,
  ): void {
    if (this.tree.root === null) return;
    this.rayCastRecursive(this.tree.root, beginX, beginY, beginZ, endX, endY, endZ, callback);
  }

  aabbTest(aabb: Aabb, callback: BroadPhaseProxyCallback): void {
    if (this.tree.root === null) return;
    this.aabbTestRecursive(this.tree.root, aabb, callback);
  }

  convexCast(convex: ConvexGeometry, begin: Transform, translation: Vec3, callback: BroadPhaseProxyCallback): void {
    if (this.tree.root === null) return;
    this.convexCastRecursive(this.tree.root, convex, begin, translation, callback);
  }

  private convexCastRecursive(node: BvhNode, convex: ConvexGeometry, begin: Transform, translation: Vec3, callback: BroadPhaseProxyCallback): void {
    if (!this.aabbConvexSweepTest(node.minX, node.minY, node.minZ, node.maxX, node.maxY, node.maxZ, convex, begin, translation)) return;
    if (node.height === 0) {
      callback.process(node.proxy!);
      return;
    }
    this.convexCastRecursive(node.child0!, convex, begin, translation, callback);
    this.convexCastRecursive(node.child1!, convex, begin, translation, callback);
  }

  private rayCastRecursive(
    node: BvhNode,
    x1: number, y1: number, z1: number,
    x2: number, y2: number, z2: number,
    callback: BroadPhaseProxyCallback,
  ): void {
    // 先比线段自己的包围盒，再做分离轴测试（线段方向分别叉乘三根坐标轴）
    if (
      node.minX > (x1 > x2 ? x1 : x2) || node.maxX < (x1 < x2 ? x1 : x2) ||
      node.minY > (y1 > y2 ? y1 : y2) || node.maxY < (y1 < y2 ? y1 : y2) ||
      node.minZ > (z1 > z2 ? z1 : z2) || node.maxZ < (z1 < z2 ? z1 : z2)
    ) {
      return;
    }
    const dX = x2 - x1;
    const dY = y2 - y1;
    const dZ = z2 - z1;
    const adX = dX < 0 ? -dX : dX;
    const adY = dY < 0 ? -dY : dY;
    const adZ = dZ < 0 ? -dZ : dZ;
    const extX = (node.maxX - node.minX) * 0.5;
    const extY = (node.maxY - node.minY) * 0.5;
    const extZ = (node.maxZ - node.minZ) * 0.5;
    const cpX = x1 - (node.maxX + node.minX) * 0.5;
    const cpY = y1 - (node.maxY + node.minY) * 0.5;
    const cpZ = z1 - (node.maxZ + node.minZ) * 0.5;
    let s = cpY * dZ - cpZ * dY;
    if ((s < 0 ? -s : s) - (extY * adZ + extZ * adY) > 0) return;
    s = cpZ * dX - cpX * dZ;
    if ((s < 0 ? -s : s) - (extZ * adX + extX * adZ) > 0) return;
    s = cpX * dY - cpY * dX;
    if ((s < 0 ? -s : s) - (extX * adY + extY * adX) > 0) return;
    if (node.height === 0) {
      callback.process(node.proxy!);
      return;
    }
    this.rayCastRecursive(node.child0!, x1, y1, z1, x2, y2, z2, callback);
    this.rayCastRecursive(node.child1!, x1, y1, z1, x2, y2, z2, callback);
  }

  private aabbTestRecursive(node: BvhNode, aabb: Aabb, callback: BroadPhaseProxyCallback): void {
    if (
      !(
        node.minX < aabb.maxX && node.maxX > aabb.minX &&
        node.minY < aabb.maxY && node.maxY > aabb.minY &&
        node.minZ < aabb.maxZ && node.maxZ > aabb.minZ
      )
    ) {
      return;
    }
    if (node.height === 0) {
      callback.process(node.proxy!);
      return;
    }
    this.aabbTestRecursive(node.child0!, aabb, callback);
    this.aabbTestRecursive(node.child1!, aabb, callback);
  }

  private addToMovedProxies(p: BvhProxy): void {
    if (p.moved) return;
    p.moved = true;
    this.movedProxies[this.numMovedProxies++] = p;
  }

  /** 递归找出 n1、n2 两棵子树之间包围盒重叠的叶子对 */
  private collide(n1: BvhNode, n2: BvhNode): void {
    this.testCount++;
    const l1 = n1.height === 0;
    const l2 = n2.height === 0;
    if (n1 === n2) {
      if (l1) return;
      this.collide(n1.child0!, n2);
      this.collide(n1.child1!, n2);
      return;
    }
    if (
      !(
        n1.minX < n2.maxX && n1.maxX > n2.minX &&
        n1.minY < n2.maxY && n1.maxY > n2.minY &&
        n1.minZ < n2.maxZ && n1.maxZ > n2.minZ
      )
    ) {
      return;
    }
    if (l1 && l2) {
      this.pickAndPushProxyPair(n1.proxy!, n2.proxy!);
      return;
    }
    if (l2 || n1.height > n2.height) {
      // 先往 n1 的子节点走
      this.collide(n1.child0!, n2);
      this.collide(n1.child1!, n2);
    } else {
      this.collide(n2.child0!, n1);
      this.collide(n2.child1!, n1);
    }
  }
}
