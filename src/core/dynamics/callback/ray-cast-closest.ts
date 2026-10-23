// 移植自 OimoPhysics src/oimo/dynamics/callback/RayCastClosest.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { RayCastHit } from "../../collision/geometry/ray-cast-hit.js";
import type { Shape } from "../rigidbody/shape.js";
import type { RayCastCallback } from "./ray-cast-callback.js";

/** 保留最近的射线命中。clear 重置结果，供后续查询复用。 */
export class RayCastClosest implements RayCastCallback {
  /** 命中的形状，没命中时为 null */
  shape: Shape | null = null;
  positionX = 0;
  positionY = 0;
  positionZ = 0;
  normalX = 0;
  normalY = 0;
  normalZ = 0;
  fraction = 1;
  /** 是否命中了任何形状 */
  hit = false;

  clear(): void {
    this.shape = null;
    this.fraction = 1;
    this.positionX = 0;
    this.positionY = 0;
    this.positionZ = 0;
    this.normalX = 0;
    this.normalY = 0;
    this.normalZ = 0;
    this.hit = false;
  }

  process(shape: Shape, hit: RayCastHit): void {
    if (hit.fraction < this.fraction) {
      this.shape = shape;
      this.hit = true;
      this.fraction = hit.fraction;
      this.positionX = hit.positionX;
      this.positionY = hit.positionY;
      this.positionZ = hit.positionZ;
      this.normalX = hit.normalX;
      this.normalY = hit.normalY;
      this.normalZ = hit.normalZ;
    }
  }
}
