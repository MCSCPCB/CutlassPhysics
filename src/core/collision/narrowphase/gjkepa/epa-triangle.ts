// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/gjkepa/EpaTriangle.hx（MIT，见 Credits.md）
import { Vec3 } from "../../../common/vec3.js";
import type { EpaVertex } from "./epa-vertex.js";
import { projectOrigin3 } from "./simplex-util.js";

/** 边 i 的另一个端点：(0, 1, 2) -> (1, 2, 0) */
export const NEXT_INDEX: readonly number[] = [1, 2, 0];

/** EPA 多面体的一个面。顶点逆时针排列（从外面看），法线朝外 */
export class EpaTriangle {
  next: EpaTriangle | null = null;
  prev: EpaTriangle | null = null;
  readonly vertices: (EpaVertex | null)[] = [null, null, null];
  /** 第 i 条边（顶点 i 到顶点 i+1）对面的三角形，以及这条边在对面三角形里的编号 */
  readonly adjacentTriangles: (EpaTriangle | null)[] = [null, null, null];
  readonly adjacentPairIndex: number[] = [0, 0, 0];
  /** 法线（没有归一化） */
  readonly normal = new Vec3();
  /** 原点到这个面的最近距离的平方 */
  distanceSq = 0;
  /** 找洞的深度优先搜索里用的访问标记 */
  tmpDfsId = 0;
  tmpDfsVisible = false;
  private readonly tmp = new Vec3();

  /** 从点 from 能不能看到这个面（from 在面的外侧） */
  checkVisible(from: Vec3): boolean {
    this.tmp.copyFrom(from).subEq(this.vertices[0]!.v);
    this.tmpDfsVisible = this.tmp.dot(this.normal) > 0;
    return this.tmpDfsVisible;
  }

  /**
   * 用三个顶点初始化。center 是多面体内部的一个点，用来判断法线朝向：
   * 法线朝里时，autoCheck 为 true 就交换两个顶点把它翻过来，否则返回 false。
   */
  init(vertex1: EpaVertex, vertex2: EpaVertex, vertex3: EpaVertex, center: Vec3, autoCheck = false): boolean {
    const v1X = vertex1.v.x, v1Y = vertex1.v.y, v1Z = vertex1.v.z;
    const v2X = vertex2.v.x, v2Y = vertex2.v.y, v2Z = vertex2.v.z;
    const v3X = vertex3.v.x, v3Y = vertex3.v.y, v3Z = vertex3.v.z;
    const v12X = v2X - v1X, v12Y = v2Y - v1Y, v12Z = v2Z - v1Z;
    const v13X = v3X - v1X, v13Y = v3Y - v1Y, v13Z = v3Z - v1Z;
    const vc1X = v1X - center.x, vc1Y = v1Y - center.y, vc1Z = v1Z - center.z;
    let nX = v12Y * v13Z - v12Z * v13Y;
    let nY = v12Z * v13X - v12X * v13Z;
    let nZ = v12X * v13Y - v12Y * v13X;
    let inverted = false;
    const d = vc1X * nX + vc1Y * nY + vc1Z * nZ;
    if (d < 0) {
      if (autoCheck) {
        // 交换顶点 2、3，让顶点逆时针排列
        const tmp = vertex2;
        vertex2 = vertex3;
        vertex3 = tmp;
        nX = nX * -1;
        nY = nY * -1;
        nZ = nZ * -1;
      } else {
        inverted = true;
      }
    }
    this.vertices[0] = vertex1;
    this.vertices[1] = vertex2;
    this.vertices[2] = vertex3;
    this.normal.init(nX, nY, nZ);
    projectOrigin3(vertex1.v, vertex2.v, vertex3.v, this.tmp);
    this.distanceSq = this.tmp.lengthSq();
    this.adjacentTriangles[0] = null;
    this.adjacentTriangles[1] = null;
    this.adjacentTriangles[2] = null;
    this.adjacentPairIndex[0] = -1;
    this.adjacentPairIndex[1] = -1;
    this.adjacentPairIndex[2] = -1;
    return !inverted;
  }

  /** 和共用一条边的三角形互相登记。恰好共用一条边才算成功 */
  setAdjacentTriangle(triangle: EpaTriangle): boolean {
    let count = 0;
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        const i2 = NEXT_INDEX[i];
        const j2 = NEXT_INDEX[j];
        if (this.vertices[i] === triangle.vertices[j2] && this.vertices[i2] === triangle.vertices[j]) {
          this.adjacentTriangles[i] = triangle;
          this.adjacentPairIndex[i] = j;
          triangle.adjacentTriangles[j] = this;
          triangle.adjacentPairIndex[j] = i;
          count++;
        }
      }
    }
    return count === 1;
  }

  removeAdjacentTriangles(): void {
    for (let i = 0; i < 3; i++) {
      const triangle = this.adjacentTriangles[i];
      if (triangle !== null) {
        const pairIndex = this.adjacentPairIndex[i];
        triangle.adjacentTriangles[pairIndex] = null;
        triangle.adjacentPairIndex[pairIndex] = -1;
        this.adjacentTriangles[i] = null;
        this.adjacentPairIndex[i] = -1;
      }
    }
  }

  removeReferences(): void {
    this.next = null;
    this.prev = null;
    this.tmpDfsId = 0;
    this.tmpDfsVisible = false;
    this.distanceSq = 0;
    this.vertices[0] = null;
    this.vertices[1] = null;
    this.vertices[2] = null;
    this.adjacentTriangles[0] = null;
    this.adjacentTriangles[1] = null;
    this.adjacentTriangles[2] = null;
    this.adjacentPairIndex[0] = 0;
    this.adjacentPairIndex[1] = 0;
    this.adjacentPairIndex[2] = 0;
  }
}
