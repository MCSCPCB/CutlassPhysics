// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/gjkepa/{EpaPolyhedron,EpaPolyhedronState}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { Vec3 } from "../../../common/vec3.js";
import { EpaTriangle, NEXT_INDEX } from "./epa-triangle.js";
import { EpaVertex } from "./epa-vertex.js";

/** 多面体的状态，出错时记下原因 */
export const EpaPolyhedronState = {
  OK: 0,
  INVALID_TRIANGLE: 1,
  NO_ADJACENT_PAIR_INDEX: 2,
  NO_ADJACENT_TRIANGLE: 3,
  EDGE_LOOP_BROKEN: 4,
  NO_OUTER_TRIANGLE: 5,
  TRIANGLE_INVISIBLE: 6,
} as const;

/**
 * EPA 用的凸多面体：从 GJK 得到的包含原点的四面体开始，每次往离原点最近的面外加一个顶点，
 * 删掉从新顶点能看到的面，再用新顶点把洞补上。三角形和顶点都放在对象池里复用。
 */
export class EpaPolyhedron {
  readonly vertices: EpaVertex[] = [];
  numVertices = 0;
  triangleList: EpaTriangle | null = null;
  triangleListLast: EpaTriangle | null = null;
  numTriangles = 0;
  status = 0;
  /** 初始四面体的中心，判断新三角形法线朝向用 */
  private readonly center = new Vec3();
  private trianglePool: EpaTriangle | null = null;
  private vertexPool: EpaVertex | null = null;

  pickVertex(): EpaVertex {
    const v = this.vertexPool;
    if (v === null) return new EpaVertex();
    this.vertexPool = v.next;
    v.next = null;
    return v;
  }

  private poolVertex(v: EpaVertex): void {
    v.removeReferences();
    v.next = this.vertexPool;
    this.vertexPool = v;
  }

  private pickTriangle(): EpaTriangle {
    const t = this.trianglePool;
    if (t === null) return new EpaTriangle();
    this.trianglePool = t.next;
    t.next = null;
    return t;
  }

  private poolTriangle(t: EpaTriangle): void {
    t.removeReferences();
    t.next = this.trianglePool;
    this.trianglePool = t;
  }

  private setAdjacentTriangle(t1: EpaTriangle, t2: EpaTriangle): void {
    if (!t1.setAdjacentTriangle(t2)) this.status = EpaPolyhedronState.INVALID_TRIANGLE;
  }

  private initTriangle(t: EpaTriangle, v1: EpaVertex, v2: EpaVertex, v3: EpaVertex, center: Vec3, autoCheck = false): void {
    if (!t.init(v1, v2, v3, center, autoCheck)) this.status = EpaPolyhedronState.INVALID_TRIANGLE;
  }

  private addTriangle(t: EpaTriangle): void {
    this.numTriangles++;
    if (this.triangleList === null) {
      this.triangleList = t;
      this.triangleListLast = t;
    } else {
      this.triangleListLast!.next = t;
      t.prev = this.triangleListLast;
      this.triangleListLast = t;
    }
  }

  private removeTriangle(t: EpaTriangle): void {
    this.numTriangles--;
    const prev = t.prev, next = t.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (t === this.triangleList) this.triangleList = this.triangleList.next;
    if (t === this.triangleListLast) this.triangleListLast = this.triangleListLast.prev;
    t.next = null;
    t.prev = null;
    this.poolTriangle(t);
  }

  /** 检查每个三角形的三条边是否都有相邻的三角形，同时清除补洞用的临时标记 */
  private validate(): boolean {
    for (let t = this.triangleList; t !== null; t = t.next) {
      for (let i = 0; i < 3; i++) {
        t.vertices[i]!.tmpEdgeLoopOuterTriangle = null;
        t.vertices[i]!.tmpEdgeLoopNext = null;
        if (t.adjacentPairIndex[i] === -1) {
          this.status = EpaPolyhedronState.NO_ADJACENT_PAIR_INDEX;
          return false;
        }
        if (t.adjacentTriangles[i] === null) {
          this.status = EpaPolyhedronState.NO_ADJACENT_TRIANGLE;
          return false;
        }
      }
    }
    return true;
  }

  /**
   * 从 base 开始深度优先搜索从 from 能看到的三角形，把它们删掉（挖出一个洞），
   * 并在洞边界的顶点上记下沿边界的下一个顶点和边界外侧的三角形。
   */
  private findEdgeLoop(id: number, base: EpaTriangle, from: Vec3): void {
    if (base.tmpDfsId === id) return;
    base.tmpDfsId = id;
    if (!base.checkVisible(from)) {
      this.status = EpaPolyhedronState.TRIANGLE_INVISIBLE;
      return;
    }
    for (let i = 0; i < 3; i++) {
      // 递归可能修改 base 的邻接表，每次迭代重新读取。
      const t = base.adjacentTriangles[i];
      if (t === null) continue;
      if (t.checkVisible(from)) {
        this.findEdgeLoop(id, t, from);
      } else {
        // base 能看到、t 看不到：这条边在洞的边界上
        const v1 = base.vertices[i]!;
        const v2 = base.vertices[NEXT_INDEX[i]]!;
        v1.tmpEdgeLoopNext = v2;
        v1.tmpEdgeLoopOuterTriangle = t;
      }
    }
    base.removeAdjacentTriangles();
    this.removeTriangle(base);
  }

  /** 删掉所有三角形，顶点放回对象池 */
  clear(): void {
    while (this.numTriangles > 0) this.removeTriangle(this.triangleList!);
    while (this.numVertices > 0) this.poolVertex(this.vertices[--this.numVertices]);
  }

  /** 用四个顶点建立初始四面体 */
  init(v1: EpaVertex, v2: EpaVertex, v3: EpaVertex, v4: EpaVertex): boolean {
    this.status = EpaPolyhedronState.OK;
    this.numVertices = 4;
    this.vertices[0] = v1;
    this.vertices[1] = v2;
    this.vertices[2] = v3;
    this.vertices[3] = v4;
    this.center.copyFrom(v1.v).addEq(v2.v).addEq(v3.v).addEq(v4.v).scaleEq(0.25);
    const t1 = this.pickTriangle();
    const t2 = this.pickTriangle();
    const t3 = this.pickTriangle();
    const t4 = this.pickTriangle();
    this.initTriangle(t1, v1, v2, v3, this.center, true);
    this.initTriangle(t2, v1, v2, v4, this.center, true);
    this.initTriangle(t3, v1, v3, v4, this.center, true);
    this.initTriangle(t4, v2, v3, v4, this.center, true);
    this.setAdjacentTriangle(t1, t2);
    this.setAdjacentTriangle(t1, t3);
    this.setAdjacentTriangle(t1, t4);
    this.setAdjacentTriangle(t2, t3);
    this.setAdjacentTriangle(t2, t4);
    this.setAdjacentTriangle(t3, t4);
    this.addTriangle(t1);
    this.addTriangle(t2);
    this.addTriangle(t3);
    this.addTriangle(t4);
    return this.status === EpaPolyhedronState.OK;
  }

  /** 离原点最近的面；距离相同时取链表里靠前的 */
  getBestTriangle(): EpaTriangle {
    let mind = Infinity;
    let minf: EpaTriangle | null = null;
    for (let f = this.triangleList; f !== null; f = f.next) {
      if (f.distanceSq < mind) {
        mind = f.distanceSq;
        minf = f;
      }
    }
    return minf!;
  }

  /** 在面 base 外加一个顶点：挖洞，再用新顶点和洞的边界补上 */
  addVertex(vertex: EpaVertex, base: EpaTriangle): boolean {
    this.vertices[this.numVertices++] = vertex;
    // base 会在挖洞时被删掉，先记下它的一个顶点作为洞边界的起点
    const v1 = base.vertices[0]!;
    this.findEdgeLoop(this.numVertices, base, vertex.v);
    if (this.status !== EpaPolyhedronState.OK) return false;
    let v = v1;
    const firstV = v1;
    let prevT: EpaTriangle | null = null;
    let firstT: EpaTriangle | null = null;
    do {
      if (v.tmpEdgeLoopNext === null) {
        this.status = EpaPolyhedronState.EDGE_LOOP_BROKEN;
        return false;
      }
      if (v.tmpEdgeLoopOuterTriangle === null) {
        this.status = EpaPolyhedronState.NO_OUTER_TRIANGLE;
        return false;
      }
      const t = this.pickTriangle();
      if (firstT === null) firstT = t;
      this.initTriangle(t, v, v.tmpEdgeLoopNext, vertex, this.center);
      if (this.status !== EpaPolyhedronState.OK) return false;
      this.addTriangle(t);
      this.setAdjacentTriangle(t, v.tmpEdgeLoopOuterTriangle);
      if (prevT !== null) this.setAdjacentTriangle(t, prevT);
      prevT = t;
      v = v.tmpEdgeLoopNext;
    } while (v !== firstV);
    this.setAdjacentTriangle(prevT!, firstT!);
    return this.status === EpaPolyhedronState.OK && this.validate();
  }
}
