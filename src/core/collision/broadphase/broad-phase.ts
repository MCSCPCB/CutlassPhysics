// 移植自 OimoPhysics src/oimo/collision/broadphase/{BroadPhase,BroadPhaseProxyCallback}.hx（MIT，见 Credits.md）
import { Transform } from "../../common/transform.js";
import type { Vec3 } from "../../common/vec3.js";
import type { Aabb } from "../geometry/aabb.js";
import type { ConvexGeometry } from "../geometry/geometry.js";
import { GjkEpa, GjkEpaResultState } from "../narrowphase/gjkepa/gjk-epa.js";
import { AabbGeometry, ConvexSweepGeometry } from "./convex-sweep.js";
import { type Proxy, ProxyPair } from "./proxy.js";

/** 宽相查询的回调：每找到一个代理调用一次 */
export interface BroadPhaseProxyCallback {
  process(proxy: Proxy): void;
}

/** 宽相碰撞检测基类：管理代理链表，收集包围盒重叠的代理对 */
export abstract class BroadPhase {
  numProxies = 0;
  proxyList: Proxy | null = null;
  proxyListLast: Proxy | null = null;

  /** collectPairs() 收集到的代理对 */
  proxyPairList: ProxyPair | null = null;
  /**
   * true：每次只收集新出现的重叠对（BVH），没收集到的对不一定已经分开。
   * false：每次收集全部重叠的对（暴力法），没收集到的对一定已经分开。
   */
  readonly incremental: boolean;
  /** 上次 collectPairs() 做了多少次包围盒测试 */
  testCount = 0;

  protected idCount = 0;
  private proxyPairPool: ProxyPair | null = null;

  protected constructor(incremental: boolean) {
    this.incremental = incremental;
  }

  abstract createProxy(userData: unknown, aabb: Aabb): Proxy;
  abstract destroyProxy(proxy: Proxy): void;
  /** 把代理移到新的包围盒。(dx, dy, dz) 是包围盒中心这一步的位移，BVH 用它预测运动方向 */
  abstract moveProxy(proxy: Proxy, aabb: Aabb, dx: number, dy: number, dz: number): void;
  /** 收集包围盒重叠的代理对，结果放在 proxyPairList */
  abstract collectPairs(): void;
  /**
   * 找出（加厚后的）包围盒和线段 begin→end 相交的代理，每个调用一次 callback。
   * 只是粗筛，精确的求交由调用方用几何体做。
   */
  abstract rayCast(
    beginX: number, beginY: number, beginZ: number,
    endX: number, endY: number, endZ: number,
    callback: BroadPhaseProxyCallback,
  ): void;
  /** 找出（加厚后的）包围盒和 aabb 重叠的代理，每个调用一次 callback */
  abstract aabbTest(aabb: Aabb, callback: BroadPhaseProxyCallback): void;
  /**
   * 找出（加厚后的）包围盒和凸体扫过的范围相交的代理，每个调用一次 callback。
   * 凸体从 begin 出发平移 translation。只是粗筛，精确的扫掠由调用方做
   */
  abstract convexCast(convex: ConvexGeometry, begin: Transform, translation: Vec3, callback: BroadPhaseProxyCallback): void;

  private readonly convexSweep = new ConvexSweepGeometry();
  private readonly aabbGeometry = new AabbGeometry();
  private readonly identity = new Transform();

  /**
   * 使用 GJK 判断包围盒与凸体扫掠范围是否相交，不包含 GJK 外边距。
   * 未加厚的代理可能过滤仅在外边距内命中的形状。
   */
  protected aabbConvexSweepTest(
    minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number,
    convex: ConvexGeometry, begin: Transform, translation: Vec3,
  ): boolean {
    this.aabbGeometry.min.init(minX, minY, minZ);
    this.aabbGeometry.max.init(maxX, maxY, maxZ);
    this.convexSweep.init(convex, begin, translation);
    const gjkEpa = GjkEpa.getInstance();
    if (gjkEpa.computeDistance(this.convexSweep, this.aabbGeometry, begin, this.identity, null) === GjkEpaResultState.SUCCEEDED) {
      return gjkEpa.distance <= 0;
    }
    return false;
  }

  /** 两个代理（加厚后的）包围盒是否重叠 */
  isOverlapping(p1: Proxy, p2: Proxy): boolean {
    return (
      p1.minX < p2.maxX && p1.maxX > p2.minX &&
      p1.minY < p2.maxY && p1.maxY > p2.minY &&
      p1.minZ < p2.maxZ && p1.maxZ > p2.minZ
    );
  }

  protected addProxy(p: Proxy): void {
    this.numProxies++;
    if (this.proxyList === null) {
      this.proxyList = p;
      this.proxyListLast = p;
    } else {
      this.proxyListLast!.next = p;
      p.prev = this.proxyListLast;
      this.proxyListLast = p;
    }
  }

  protected removeProxy(p: Proxy): void {
    this.numProxies--;
    const prev = p.prev, next = p.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (p === this.proxyList) this.proxyList = next;
    if (p === this.proxyListLast) this.proxyListLast = prev;
    p.next = null;
    p.prev = null;
  }

  /** 从对象池取一个代理对，插到链表头部 */
  protected pickAndPushProxyPair(p1: Proxy, p2: Proxy): void {
    let pp = this.proxyPairPool;
    if (pp !== null) {
      this.proxyPairPool = pp.next;
      pp.next = null;
    } else {
      pp = new ProxyPair();
    }
    if (this.proxyPairList !== null) pp.next = this.proxyPairList;
    this.proxyPairList = pp;
    pp.p1 = p1;
    pp.p2 = p2;
  }

  /** 将全部代理对归还对象池，保留链表顺序。 */
  protected poolProxyPairs(): void {
    let p = this.proxyPairList;
    if (p === null) return;
    let last = p;
    while (p !== null) {
      p.p1 = null;
      p.p2 = null;
      last = p;
      p = p.next;
    }
    last.next = this.proxyPairPool;
    this.proxyPairPool = this.proxyPairList;
    this.proxyPairList = null;
  }
}
