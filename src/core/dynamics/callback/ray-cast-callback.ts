// 移植自 OimoPhysics src/oimo/dynamics/callback/RayCastCallback.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { RayCastHit } from "../../collision/geometry/ray-cast-hit.js";
import type { Shape } from "../rigidbody/shape.js";

/** World.rayCast 的回调 */
export interface RayCastCallback {
  /** 射线每命中一个形状调用一次。hit 为复用对象，仅在当前回调中有效。 */
  process(shape: Shape, hit: RayCastHit): void;
}
