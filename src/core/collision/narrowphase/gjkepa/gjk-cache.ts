// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/gjkepa/GjkCache.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { Vec3 } from "../../../common/vec3.js";

/** 上次 GJK 搜索方向，供后续迭代热启动。 */
export class GjkCache {
  readonly prevClosestDir = new Vec3();

  clear(): void {
    this.prevClosestDir.zero();
  }
}
