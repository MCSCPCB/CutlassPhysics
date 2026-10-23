// 移植自 OimoPhysics src/oimo/collision/broadphase/{Proxy,ProxyPair}.hx（MIT，见 Credits.md）
import type { Aabb } from "../geometry/aabb.js";

/** 宽相里的一个包围盒。物理部分用 userData 连接到对应的形状 */
export class Proxy {
  prev: Proxy | null = null;
  next: Proxy | null = null;

  // 宽相实际使用的包围盒。BVH 会在形状包围盒外加一圈余量，并沿位移方向延伸（“加厚”）
  minX = 0;
  minY = 0;
  minZ = 0;
  maxX = 0;
  maxY = 0;
  maxZ = 0;

  readonly id: number;
  userData: unknown;

  constructor(userData: unknown, id: number) {
    this.userData = userData;
    this.id = id;
  }

  setAabb(aabb: Aabb): void {
    this.minX = aabb.minX;
    this.minY = aabb.minY;
    this.minZ = aabb.minZ;
    this.maxX = aabb.maxX;
    this.maxY = aabb.maxY;
    this.maxZ = aabb.maxZ;
  }
}

/** 一对包围盒重叠的代理。宽相把它们串成单向链表 */
export class ProxyPair {
  next: ProxyPair | null = null;
  p1: Proxy | null = null;
  p2: Proxy | null = null;
}
