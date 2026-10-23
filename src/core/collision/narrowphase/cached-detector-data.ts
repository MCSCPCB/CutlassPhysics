// 移植自 OimoPhysics src/oimo/collision/narrowphase/detector/CachedDetectorData.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { GjkCache } from "./gjkepa/gjk-cache.js";

/** 一对形状在窄相里要跨步保存的数据。目前只有 GJK 的缓存，第一次用到时才创建 */
export class CachedDetectorData {
  gjkCache: GjkCache | null = null;

  clear(): void {
    if (this.gjkCache !== null) this.gjkCache.clear();
  }
}
