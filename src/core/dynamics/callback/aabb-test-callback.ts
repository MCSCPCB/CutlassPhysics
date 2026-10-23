// 移植自 OimoPhysics src/oimo/dynamics/callback/AabbTestCallback.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { Shape } from "../rigidbody/shape.js";

/** World.aabbTest 的回调 */
export interface AabbTestCallback {
  /** 每找到一个包围盒和查询范围重叠的形状调用一次 */
  process(shape: Shape): void;
}
