// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/gjkepa/EpaVertex.hx（MIT，见 Credits.md）
import { Vec3 } from "../../../common/vec3.js";
import type { EpaTriangle } from "./epa-triangle.js";

/** EPA 多面体的顶点：闵可夫斯基差上的点 v，以及它在两个几何体上对应的点 w1、w2（v = w1 - w2） */
export class EpaVertex {
  /** 对象池链表 */
  next: EpaVertex | null = null;
  readonly v = new Vec3();
  readonly w1 = new Vec3();
  readonly w2 = new Vec3();
  /** 补洞时用：洞的边界上，沿边走到的下一个顶点，以及这条边外侧（没被删掉）的三角形 */
  tmpEdgeLoopNext: EpaVertex | null = null;
  tmpEdgeLoopOuterTriangle: EpaTriangle | null = null;

  init(v: Vec3, w1: Vec3, w2: Vec3): this {
    this.v.copyFrom(v);
    this.w1.copyFrom(w1);
    this.w2.copyFrom(w2);
    this.next = null;
    this.tmpEdgeLoopNext = null;
    this.tmpEdgeLoopOuterTriangle = null;
    return this;
  }

  removeReferences(): void {
    this.next = null;
    this.tmpEdgeLoopNext = null;
    this.tmpEdgeLoopOuterTriangle = null;
  }
}
