// 移植自 OimoPhysics src/oimo/collision/broadphase/BroadPhase.hx 里的私有类 ConvexSweepGeometry、AabbGeometry（MIT，见 Credits.md）
// 宽相做凸体扫掠时用：把“凸体沿平移扫过的范围”和“代理的包围盒”都包装成凸几何体，交给 GJK 判断是否相交。
// 复用支撑点向量。
import type { Transform } from "../../common/transform.js";
import { Vec3 } from "../../common/vec3.js";
import { ConvexGeometry } from "../geometry/geometry.js";

/** 凸体 c 从 transform 平移 translation 扫过的范围（两端位置的凸包） */
export class ConvexSweepGeometry extends ConvexGeometry {
  private c: ConvexGeometry | null = null;
  /** 平移量转到 c 的局部坐标 */
  private readonly localTranslation = new Vec3();

  constructor() {
    super(-1);
  }

  init(c: ConvexGeometry, transform: Transform, translation: Vec3): void {
    this.c = c;
    const tX = translation.x, tY = translation.y, tZ = translation.z;
    this.localTranslation.init(
      transform.r00 * tX + transform.r10 * tY + transform.r20 * tZ,
      transform.r01 * tX + transform.r11 * tY + transform.r21 * tZ,
      transform.r02 * tX + transform.r12 * tY + transform.r22 * tZ,
    );
    this.gjkMargin = c.gjkMargin;
  }

  computeLocalSupportingVertex(dir: Vec3, out: Vec3): void {
    this.c!.computeLocalSupportingVertex(dir, out);
    if (dir.dot(this.localTranslation) > 0) out.addEq(this.localTranslation);
  }

  // 仅供 GJK 支撑映射使用，其余几何接口为空实现。
  updateMass(): void {}
  computeAabb(): void {}
  rayCastLocal(): boolean {
    return false;
  }
}

/** 轴对齐包围盒 [min, max]，放在单位变换处 */
export class AabbGeometry extends ConvexGeometry {
  readonly min = new Vec3();
  readonly max = new Vec3();

  constructor() {
    super(-1);
  }

  computeLocalSupportingVertex(dir: Vec3, out: Vec3): void {
    out.x = dir.x > 0 ? this.max.x : this.min.x;
    out.y = dir.y > 0 ? this.max.y : this.min.y;
    out.z = dir.z > 0 ? this.max.z : this.min.z;
  }

  updateMass(): void {}
  computeAabb(): void {}
  rayCastLocal(): boolean {
    return false;
  }
}
