// 移植自 OimoPhysics src/oimo/collision/geometry/RayCastHit.hx（MIT，见 THIRD-PARTY-NOTICES.txt）

/** 一次射线命中的结果，世界坐标 */
export class RayCastHit {
  /** 命中点 */
  positionX = 0;
  positionY = 0;
  positionZ = 0;
  /** 命中面的法线（朝外，单位向量） */
  normalX = 0;
  normalY = 0;
  normalZ = 0;
  /** 命中点在线段上的比例：0 是起点，1 是终点 */
  fraction = 0;
}
