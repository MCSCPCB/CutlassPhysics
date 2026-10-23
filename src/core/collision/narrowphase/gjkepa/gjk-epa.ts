// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/gjkepa/{GjkEpa,GjkEpaResultState}.hx（MIT，见 Credits.md）
//
// GJK：在两个凸几何体的闵可夫斯基差上迭代一个单纯形，求离原点最近的点，得到两者的距离和最近点。
// 原点落进单纯形（两者重叠）时，把单纯形补成四面体交给 EPA，求穿透深度和最近点。
// 几何体都按“核心 + GJK 外边距”处理：这里算的是两个核心之间的距离，外边距由检测器再减掉。
//
// 凸体扫掠（convexCastImpl）也是 GJK：沿平移方向一步步推进射线（保守推进），直到单纯形碰到原点。
//
// 向量对象预分配。
import { Transform } from "../../../common/transform.js";
import { Vec3 } from "../../../common/vec3.js";
import type { ConvexGeometry } from "../../geometry/geometry.js";
import type { RayCastHit } from "../../geometry/ray-cast-hit.js";
import type { CachedDetectorData } from "../cached-detector-data.js";
import { EpaPolyhedron } from "./epa-polyhedron.js";
import { GjkCache } from "./gjk-cache.js";
import { projectOrigin2, projectOrigin3, projectOrigin4 } from "./simplex-util.js";

/** GJK/EPA 的结果状态 */
export const GjkEpaResultState = {
  SUCCEEDED: 0x000,
  GJK_FAILED_TO_MAKE_TETRAHEDRON: 0x001,
  GJK_DID_NOT_CONVERGE: 0x002,
  EPA_FAILED_TO_INIT: 0x101,
  EPA_FAILED_TO_ADD_VERTEX: 0x102,
  EPA_DID_NOT_CONVERGE: 0x103,
} as const;

let instance: GjkEpa | null = null;

export class GjkEpa {
  /** 共用的实例。第一次用到时才创建，import 本库没有副作用 */
  static getInstance(): GjkEpa {
    if (instance === null) instance = new GjkEpa();
    return instance;
  }

  /** 几何体 1、2 上的最近点（世界坐标） */
  readonly closestPoint1 = new Vec3();
  readonly closestPoint2 = new Vec3();
  /** 两个核心之间的距离，重叠时是负的穿透深度 */
  distance = 0;

  private c1: ConvexGeometry | null = null;
  private c2: ConvexGeometry | null = null;
  private tf1: Transform | null = null;
  private tf2: Transform | null = null;
  /** 单纯形顶点（闵可夫斯基差上的点），以及它们在两个几何体上对应的点 */
  private readonly s: Vec3[] = [new Vec3(), new Vec3(), new Vec3(), new Vec3()];
  private readonly w1: Vec3[] = [new Vec3(), new Vec3(), new Vec3(), new Vec3()];
  private readonly w2: Vec3[] = [new Vec3(), new Vec3(), new Vec3(), new Vec3()];
  private simplexSize = 0;
  /** 求支撑点的方向 */
  private readonly dir = new Vec3();
  /** 单纯形上离原点最近的点 */
  private readonly closest = new Vec3();
  /** 把单纯形补成四面体时试探的方向 */
  private readonly baseDirs: Vec3[] = [new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1)];
  private depth = 0;
  private readonly polyhedron = new EpaPolyhedron();
  /** 凸体扫掠用：两个几何体的平移、射线的起点和相对平移；射线检测时代表起点的变换 */
  private readonly tl1 = new Vec3();
  private readonly tl2 = new Vec3();
  private readonly rayX = new Vec3();
  private readonly rayR = new Vec3();
  private readonly tempTransform = new Transform();

  /**
   * 求两个凸几何体核心之间的最近点和距离（重叠时用 EPA 求穿透深度），返回 GjkEpaResultState。
   * cache 不为 null 时从上次的结果开始迭代，并把这次的结果存进去。
   */
  computeClosestPoints(c1: ConvexGeometry, c2: ConvexGeometry, tf1: Transform, tf2: Transform, cache: CachedDetectorData | null): number {
    return this.computeClosestPointsImpl(c1, c2, tf1, tf2, cache, true);
  }

  /** 同上，但重叠时不跑 EPA：distance 直接是 0，最近点不更新 */
  computeDistance(c1: ConvexGeometry, c2: ConvexGeometry, tf1: Transform, tf2: Transform, cache: CachedDetectorData | null): number {
    return this.computeClosestPointsImpl(c1, c2, tf1, tf2, cache, false);
  }

  /**
   * 凸体扫掠：c1、c2 分别从 tf1、tf2 出发平移 tl1、tl2，求第一次碰到的时刻。
   * 碰到时返回 true，hit 里是比例（0~1）、碰撞点（世界坐标）、法线；一开始就重叠、或者扫完都没碰到，返回 false
   */
  convexCast(c1: ConvexGeometry, c2: ConvexGeometry, tf1: Transform, tf2: Transform, tl1: Vec3, tl2: Vec3, hit: RayCastHit): boolean {
    return this.convexCastImpl(c1, c2, tf1, tf2, tl1, tl2, hit);
  }

  /** 用 GJK 做射线检测：把线段当作从 begin 平移到 end 的一个点，和放在 tf 处的 c 做凸体扫掠 */
  rayCast(
    c: ConvexGeometry, tf: Transform,
    beginX: number, beginY: number, beginZ: number, endX: number, endY: number, endZ: number,
    hit: RayCastHit,
  ): boolean {
    const tf1 = this.tempTransform;
    tf1.px = beginX;
    tf1.py = beginY;
    tf1.pz = beginZ;
    this.tl1.init(endX - beginX, endY - beginY, endZ - beginZ);
    this.tl2.zero();
    return this.convexCastImpl(null, c, tf1, tf, this.tl1, this.tl2, hit);
  }

  private computeClosestPointsImpl(
    c1: ConvexGeometry, c2: ConvexGeometry, tf1: Transform, tf2: Transform, cache: CachedDetectorData | null, useEpa: boolean,
  ): number {
    this.c1 = c1;
    this.c2 = c2;
    this.tf1 = tf1;
    this.tf2 = tf2;
    const s = this.s, closest = this.closest, dir = this.dir;

    if (cache !== null) {
      if (cache.gjkCache === null) cache.gjkCache = new GjkCache();
      dir.copyFrom(cache.gjkCache.prevClosestDir);
    } else {
      dir.zero();
    }
    if (dir.lengthSq() === 0) {
      // 第一个顶点：从 1 指向 2 的方向
      dir.init(tf2.px - tf1.px, tf2.py - tf1.py, tf2.pz - tf1.pz);
      if (dir.lengthSq() < 1e-6) dir.init(1, 0, 0);
    }

    this.simplexSize = 0;
    this.computeSupportingVertex();
    this.simplexSize = 1;

    const eps2 = 1e-8;
    let count = 0;
    while (count < 40) {
      // 原点投影到单纯形上，得到最近点和它所在的 Voronoi 区域
      let v = 0;
      switch (this.simplexSize) {
        case 1:
          closest.copyFrom(s[0]);
          v = 1;
          break;
        case 2:
          v = projectOrigin2(s[0], s[1], closest);
          break;
        case 3:
          v = projectOrigin3(s[0], s[1], s[2], closest);
          break;
        case 4:
          v = projectOrigin4(s[0], s[1], s[2], s[3], closest);
          break;
      }

      // 原点碰到或落进单纯形：两者重叠
      if (closest.lengthSq() < eps2) {
        if (!useEpa) {
          this.distance = 0;
          return GjkEpaResultState.SUCCEEDED;
        }
        // EPA 需要一个四面体
        switch (this.simplexSize) {
          case 1:
            this.pointToTetrahedron();
            break;
          case 2:
            this.lineToTetrahedron();
            break;
          case 3:
            this.triangleToTetrahedron();
            break;
        }
        if (this.simplexSize === 4) {
          const epaState = this.computeDepth();
          if (epaState !== GjkEpaResultState.SUCCEEDED) {
            this.distance = 0;
            return epaState;
          }
          this.distance = -this.depth;
          return GjkEpaResultState.SUCCEEDED;
        }
        this.distance = 0;
        return GjkEpaResultState.GJK_FAILED_TO_MAKE_TETRAHEDRON;
      }

      // 只保留最近点用到的顶点，然后朝原点方向找下一个顶点
      this.shrinkSimplex(v);
      dir.copyFrom(closest).negateEq();
      this.computeSupportingVertex();
      if (dir.lengthSq() < eps2) throw new Error("GJK 内部错误：搜索方向为零");

      // 新顶点没有让单纯形更靠近原点：收敛
      const d1 = closest.dot(dir);
      const d2 = s[this.simplexSize].dot(dir);
      if (d2 - d1 < eps2) {
        this.interpolateClosestPoints();
        this.distance = closest.length();
        if (cache !== null && cache.gjkCache !== null) cache.gjkCache.prevClosestDir.copyFrom(closest).negateEq();
        return GjkEpaResultState.SUCCEEDED;
      }
      this.simplexSize++;
      count++;
    }
    return GjkEpaResultState.GJK_DID_NOT_CONVERGE;
  }

  /** c1 可以是 null：这时几何体 1 是 tf1 位置上的一个点（射线检测） */
  private convexCastImpl(
    c1: ConvexGeometry | null, c2: ConvexGeometry, tf1: Transform, tf2: Transform, tl1: Vec3, tl2: Vec3, hit: RayCastHit,
  ): boolean {
    this.c1 = c1;
    this.c2 = c2;
    this.tf1 = tf1;
    this.tf2 = tf2;
    const s = this.s, closest = this.closest, dir = this.dir;

    dir.init(tf2.px - tf1.px, tf2.py - tf1.py, tf2.pz - tf1.pz);
    if (dir.lengthSq() < 1e-6) dir.init(1, 0, 0);
    this.simplexSize = 0;
    this.computeConvexCastSupportingVertex();
    this.simplexSize = 1;

    // 射线：原点沿 rayR（两者的相对平移）推进，推进到 rayX，比例是 lambda
    let lambda = 0.0;
    const rayX = this.rayX, rayR = this.rayR;
    rayX.zero();
    rayR.copyFrom(tl2).subEq(tl1);

    const eps2 = 1e-8;
    let count = 0;
    while (count < 40) {
      let v = 0;
      switch (this.simplexSize) {
        case 1:
          closest.copyFrom(s[0]);
          v = 1;
          break;
        case 2:
          v = projectOrigin2(s[0], s[1], closest);
          break;
        case 3:
          v = projectOrigin3(s[0], s[1], s[2], closest);
          break;
        case 4:
          v = projectOrigin4(s[0], s[1], s[2], s[3], closest);
          break;
      }
      this.shrinkSimplex(v);

      // 单纯形碰到原点：在比例 lambda 处接触
      if (closest.lengthSq() < eps2) {
        if (lambda === 0 || this.simplexSize === 4) {
          // 一开始就重叠
          hit.fraction = lambda;
          return false;
        }
        this.interpolateClosestPoints();
        hit.fraction = lambda;
        // 法线是上一次的搜索方向
        let nX = dir.x, nY = dir.y, nZ = dir.z;
        let invLen = Math.sqrt(nX * nX + nY * nY + nZ * nZ);
        if (invLen > 0) invLen = 1 / invLen;
        nX *= invLen;
        nY *= invLen;
        nZ *= invLen;
        hit.normalX = nX;
        hit.normalY = nY;
        hit.normalZ = nZ;
        let pX = this.closestPoint1.x, pY = this.closestPoint1.y, pZ = this.closestPoint1.z;
        pX += tl1.x * lambda;
        pY += tl1.y * lambda;
        pZ += tl1.z * lambda;
        hit.positionX = pX;
        hit.positionY = pY;
        hit.positionZ = pZ;
        return true;
      }

      // 新的支撑点（平移到当前射线位置）
      dir.copyFrom(closest).negateEq();
      this.computeConvexCastSupportingVertex();
      s[this.simplexSize].subEq(rayX);
      if (dir.lengthSq() < eps2) throw new Error("GJK 内部错误：搜索方向为零");

      // p 是新顶点，n（= dir）是那里的法线：p·n < 0 时射线可以安全地推进一段
      const p = s[this.simplexSize], n = dir;
      const pn = p.dot(n);
      if (pn < 0) {
        // 射线背离这个面：整条射线都碰不到
        if (rayR.dot(n) >= 0) return false;
        const dLambda = pn / rayR.dot(n);
        lambda += dLambda;
        if (lambda >= 1) return false;
        rayX.addScaledEq(rayR, dLambda);
        // 单纯形跟着平移
        for (let i = 0; i < this.simplexSize + 1; i++) s[i].addScaledEq(rayR, -dLambda);
      }

      // 和单纯形里已有的顶点重合就不加
      let duplicate = false;
      for (let i = 0; i < this.simplexSize; i++) {
        const dx = s[i].x - s[this.simplexSize].x;
        const dy = s[i].y - s[this.simplexSize].y;
        const dz = s[i].z - s[this.simplexSize].z;
        if (dx * dx + dy * dy + dz * dz < eps2) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) this.simplexSize++;
      count++;
    }
    return false;
  }

  /** 凸体扫掠的支撑点：两边都把外边距加回来（扫的是整个几何体）；c1 为 null 时几何体 1 是一个点 */
  private computeConvexCastSupportingVertex(): void {
    const i = this.simplexSize;
    if (this.c1 !== null) {
      this.computeWitnessPoint1(true);
    } else {
      const tf = this.tf1!;
      this.w1[i].init(tf.px, tf.py, tf.pz);
    }
    this.computeWitnessPoint2(true);
    this.s[i].copyFrom(this.w1[i]).subEq(this.w2[i]);
  }

  /** 沿 dir 求闵可夫斯基差上的支撑点，写到单纯形的第 simplexSize 个顶点 */
  private computeSupportingVertex(): void {
    const i = this.simplexSize;
    this.computeWitnessPoint1(false);
    this.computeWitnessPoint2(false);
    this.s[i].copyFrom(this.w1[i]).subEq(this.w2[i]);
  }

  /** 几何体 1 沿 dir 的支撑点（世界坐标）。dir 临时改成局部方向，算完恢复 */
  private computeWitnessPoint1(addMargin: boolean): void {
    const tf = this.tf1!, dir = this.dir, w = this.w1[this.simplexSize];
    const idirX = dir.x, idirY = dir.y, idirZ = dir.z;
    dir.init(
      tf.r00 * idirX + tf.r10 * idirY + tf.r20 * idirZ,
      tf.r01 * idirX + tf.r11 * idirY + tf.r21 * idirZ,
      tf.r02 * idirX + tf.r12 * idirY + tf.r22 * idirZ,
    );
    this.c1!.computeLocalSupportingVertex(dir, w);
    if (addMargin) {
      dir.normalize();
      w.addScaledEq(dir, this.c1!.gjkMargin);
    }
    const tX = w.x, tY = w.y, tZ = w.z;
    let wX = tf.r00 * tX + tf.r01 * tY + tf.r02 * tZ;
    let wY = tf.r10 * tX + tf.r11 * tY + tf.r12 * tZ;
    let wZ = tf.r20 * tX + tf.r21 * tY + tf.r22 * tZ;
    wX += tf.px;
    wY += tf.py;
    wZ += tf.pz;
    w.init(wX, wY, wZ);
    dir.init(idirX, idirY, idirZ);
  }

  /** 几何体 2 沿 -dir 的支撑点（世界坐标） */
  private computeWitnessPoint2(addMargin: boolean): void {
    const tf = this.tf2!, dir = this.dir, w = this.w2[this.simplexSize];
    const idirX = dir.x, idirY = dir.y, idirZ = dir.z;
    let lX = tf.r00 * idirX + tf.r10 * idirY + tf.r20 * idirZ;
    let lY = tf.r01 * idirX + tf.r11 * idirY + tf.r21 * idirZ;
    let lZ = tf.r02 * idirX + tf.r12 * idirY + tf.r22 * idirZ;
    lX = -lX;
    lY = -lY;
    lZ = -lZ;
    dir.init(lX, lY, lZ);
    this.c2!.computeLocalSupportingVertex(dir, w);
    if (addMargin) {
      dir.normalize();
      w.addScaledEq(dir, this.c2!.gjkMargin);
    }
    const tX = w.x, tY = w.y, tZ = w.z;
    let wX = tf.r00 * tX + tf.r01 * tY + tf.r02 * tZ;
    let wY = tf.r10 * tX + tf.r11 * tY + tf.r12 * tZ;
    let wZ = tf.r20 * tX + tf.r21 * tY + tf.r22 * tZ;
    wX += tf.px;
    wY += tf.py;
    wZ += tf.pz;
    w.init(wX, wY, wZ);
    dir.init(idirX, idirY, idirZ);
  }

  /** 用单纯形上最近点的重心坐标，插值出两个几何体上的最近点 */
  private interpolateClosestPoints(): void {
    const s = this.s, w1 = this.w1, w2 = this.w2, c = this.closest;
    switch (this.simplexSize) {
      case 1:
        this.closestPoint1.copyFrom(w1[0]);
        this.closestPoint2.copyFrom(w2[0]);
        break;
      case 2: {
        const s01X = s[1].x - s[0].x, s01Y = s[1].y - s[0].y, s01Z = s[1].z - s[0].z;
        let invDet = s01X * s01X + s01Y * s01Y + s01Z * s01Z;
        if (invDet !== 0) invDet = 1 / invDet;
        const s0cX = c.x - s[0].x, s0cY = c.y - s[0].y, s0cZ = c.z - s[0].z;
        const t = (s0cX * s01X + s0cY * s01Y + s0cZ * s01Z) * invDet;
        let dX = w1[1].x - w1[0].x, dY = w1[1].y - w1[0].y, dZ = w1[1].z - w1[0].z;
        this.closestPoint1.init(w1[0].x + dX * t, w1[0].y + dY * t, w1[0].z + dZ * t);
        dX = w2[1].x - w2[0].x;
        dY = w2[1].y - w2[0].y;
        dZ = w2[1].z - w2[0].z;
        this.closestPoint2.init(w2[0].x + dX * t, w2[0].y + dY * t, w2[0].z + dZ * t);
        break;
      }
      case 3:
        this.interpolateOnTriangle(s[0], w1[0], w2[0], s[1], w1[1], w2[1], s[2], w1[2], w2[2]);
        break;
      default:
        throw new Error("GJK 内部错误：单纯形顶点数不对");
    }
  }

  /** 按三角形重心坐标插值两个几何体上的最近点，写入 closestPoint1 和 closestPoint2。 */
  private interpolateOnTriangle(
    s0: Vec3, w10: Vec3, w20: Vec3, s1: Vec3, w11: Vec3, w21: Vec3, s2: Vec3, w12: Vec3, w22: Vec3,
  ): void {
    const c = this.closest;
    const s01X = s1.x - s0.x, s01Y = s1.y - s0.y, s01Z = s1.z - s0.z;
    const s02X = s2.x - s0.x, s02Y = s2.y - s0.y, s02Z = s2.z - s0.z;
    const s0cX = c.x - s0.x, s0cY = c.y - s0.y, s0cZ = c.z - s0.z;
    const d11 = s01X * s01X + s01Y * s01Y + s01Z * s01Z;
    const d12 = s01X * s02X + s01Y * s02Y + s01Z * s02Z;
    const d22 = s02X * s02X + s02Y * s02Y + s02Z * s02Z;
    const d1c = s01X * s0cX + s01Y * s0cY + s01Z * s0cZ;
    const d2c = s02X * s0cX + s02Y * s0cY + s02Z * s0cZ;
    let invDet = d11 * d22 - d12 * d12;
    if (invDet !== 0) invDet = 1 / invDet;
    const u = (d1c * d22 - d2c * d12) * invDet;
    const t = (-d1c * d12 + d2c * d11) * invDet;
    let dX = w11.x - w10.x, dY = w11.y - w10.y, dZ = w11.z - w10.z;
    let pX = w10.x + dX * u, pY = w10.y + dY * u, pZ = w10.z + dZ * u;
    dX = w12.x - w10.x;
    dY = w12.y - w10.y;
    dZ = w12.z - w10.z;
    pX += dX * t;
    pY += dY * t;
    pZ += dZ * t;
    this.closestPoint1.init(pX, pY, pZ);
    dX = w21.x - w20.x;
    dY = w21.y - w20.y;
    dZ = w21.z - w20.z;
    pX = w20.x + dX * u;
    pY = w20.y + dY * u;
    pZ = w20.z + dZ * u;
    dX = w22.x - w20.x;
    dY = w22.y - w20.y;
    dZ = w22.z - w20.z;
    pX += dX * t;
    pY += dY * t;
    pZ += dZ * t;
    this.closestPoint2.init(pX, pY, pZ);
  }

  /** 按 Voronoi 区域的位掩码只保留用到的顶点，并将其移到数组前部 */
  private shrinkSimplex(vertexBits: number): void {
    // 数一下有几位是 1
    let n = vertexBits;
    n = (n & 5) + ((n >> 1) & 5);
    n = (n & 3) + ((n >> 2) & 3);
    this.simplexSize = n;
    switch (vertexBits) {
      case 2: this.moveVertex(1, 0); break;
      case 4: this.moveVertex(2, 0); break;
      case 5: this.moveVertex(2, 1); break;
      case 6: this.moveVertex(2, 0); break;
      case 8: this.moveVertex(3, 0); break;
      case 9: this.moveVertex(3, 1); break;
      case 10: this.moveVertex(3, 0); break;
      case 11: this.moveVertex(3, 2); break;
      case 12: this.moveVertex(2, 0); this.moveVertex(3, 1); break;
      case 13: this.moveVertex(3, 1); break;
      case 14: this.moveVertex(3, 0); break;
      // 掩码为 0、1、3、7、15 时，保留的顶点已连续排列。
    }
  }

  private moveVertex(from: number, to: number): void {
    this.s[to].copyFrom(this.s[from]);
    this.w1[to].copyFrom(this.w1[from]);
    this.w2[to].copyFrom(this.w2[from]);
  }

  // 单纯形退化（点、线段、三角形）时，沿几个方向找支撑点，把它补成体积不为 0 的四面体。
  // 找不到时 simplexSize 不到 4，调用方返回 GJK_FAILED_TO_MAKE_TETRAHEDRON。

  private pointToTetrahedron(): void {
    for (let i = 0; i < 3; i++) {
      this.dir.copyFrom(this.baseDirs[i]);
      this.computeSupportingVertex();
      this.simplexSize++;
      this.lineToTetrahedron();
      if (this.simplexSize === 4) break;
      this.simplexSize--;

      this.dir.negateEq();
      this.computeSupportingVertex();
      this.simplexSize++;
      this.lineToTetrahedron();
      if (this.simplexSize === 4) break;
      this.simplexSize--;
    }
  }

  private lineToTetrahedron(): void {
    const dir = this.dir, s = this.s;
    const oldX = dir.x, oldY = dir.y, oldZ = dir.z;
    const lX = s[0].x - s[1].x, lY = s[0].y - s[1].y, lZ = s[0].z - s[1].z;
    for (let i = 0; i < 3; i++) {
      // 和线段垂直的方向：线段方向叉乘坐标轴
      const b = this.baseDirs[i];
      dir.init(lY * b.z - lZ * b.y, lZ * b.x - lX * b.z, lX * b.y - lY * b.x);
      this.computeSupportingVertex();
      this.simplexSize++;
      this.triangleToTetrahedron();
      if (this.simplexSize === 4) break;
      this.simplexSize--;

      dir.negateEq();
      this.computeSupportingVertex();
      this.simplexSize++;
      this.triangleToTetrahedron();
      if (this.simplexSize === 4) break;
      this.simplexSize--;
    }
    dir.init(oldX, oldY, oldZ);
  }

  private triangleToTetrahedron(): void {
    const dir = this.dir, s = this.s;
    const oldX = dir.x, oldY = dir.y, oldZ = dir.z;
    // 沿三角形法线的两个方向各试一次
    const s01X = s[1].x - s[0].x, s01Y = s[1].y - s[0].y, s01Z = s[1].z - s[0].z;
    const s02X = s[2].x - s[0].x, s02Y = s[2].y - s[0].y, s02Z = s[2].z - s[0].z;
    dir.init(s01Y * s02Z - s01Z * s02Y, s01Z * s02X - s01X * s02Z, s01X * s02Y - s01Y * s02X);
    this.computeSupportingVertex();
    this.simplexSize++;
    if (!this.isValidTetrahedron()) {
      this.simplexSize--;
      dir.negateEq();
      this.computeSupportingVertex();
      this.simplexSize++;
      if (!this.isValidTetrahedron()) this.simplexSize--;
    }
    dir.init(oldX, oldY, oldZ);
  }

  /** 四面体的体积（行列式）不接近 0 */
  private isValidTetrahedron(): boolean {
    const s = this.s;
    const e00 = s[1].x - s[0].x, e01 = s[1].y - s[0].y, e02 = s[1].z - s[0].z;
    const e10 = s[2].x - s[0].x, e11 = s[2].y - s[0].y, e12 = s[2].z - s[0].z;
    const e20 = s[3].x - s[0].x, e21 = s[3].y - s[0].y, e22 = s[3].z - s[0].z;
    const det = e00 * (e11 * e22 - e12 * e21) - e01 * (e10 * e22 - e12 * e20) + e02 * (e10 * e21 - e11 * e20);
    return det > 1e-12 || det < -1e-12;
  }

  /** EPA：从单纯形（四面体）开始扩张多面体，求穿透深度和最近点 */
  private computeDepth(): number {
    const poly = this.polyhedron, s = this.s, w1 = this.w1, w2 = this.w2, dir = this.dir;
    poly.clear();
    const ok = poly.init(
      poly.pickVertex().init(s[0], w1[0], w2[0]),
      poly.pickVertex().init(s[1], w1[1], w2[1]),
      poly.pickVertex().init(s[2], w1[2], w2[2]),
      poly.pickVertex().init(s[3], w1[3], w2[3]),
    );
    if (!ok) return GjkEpaResultState.EPA_FAILED_TO_INIT;

    // 之后新的支撑点都算到单纯形的第 0 个位置
    this.simplexSize = 0;
    const supportingVertex = s[0], witness1 = w1[0], witness2 = w2[0];
    const maxIterations = 40;
    for (let count = 0; count < maxIterations; count++) {
      // 沿离原点最近的面的法线找新的支撑点
      const face = poly.getBestTriangle();
      dir.copyFrom(face.normal).normalize();
      this.computeSupportingVertex();
      const v0 = face.vertices[0]!, v1 = face.vertices[1]!, v2 = face.vertices[2]!;
      const dot1 = v0.v.dot(dir);
      const dot2 = supportingVertex.dot(dir);
      if (dot2 - dot1 < 1e-6 || count === maxIterations - 1) {
        // 没法再往外扩了：原点在这个面上的投影就是最近点
        this.closest.copyFrom(dir).scaleEq(dir.dot(v0.v) / dir.lengthSq());
        this.interpolateOnTriangle(v0.v, v0.w1, v0.w2, v1.v, v1.w1, v1.w2, v2.v, v2.w1, v2.w2);
        this.depth = this.closest.length();
        return GjkEpaResultState.SUCCEEDED;
      }
      const epaVertex = poly.pickVertex().init(supportingVertex, witness1, witness2);
      if (!poly.addVertex(epaVertex, face)) return GjkEpaResultState.EPA_FAILED_TO_ADD_VERTEX;
    }
    return GjkEpaResultState.EPA_DID_NOT_CONVERGE;
  }
}
