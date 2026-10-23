// 移植自 OimoPhysics src/oimo/collision/geometry/Aabb.hx（MIT，见 Credits.md）
// 轴对齐包围盒。
export class Aabb {
  minX = 0;
  minY = 0;
  minZ = 0;
  maxX = 0;
  maxY = 0;
  maxZ = 0;

  copyFrom(aabb: Aabb): void {
    this.minX = aabb.minX;
    this.minY = aabb.minY;
    this.minZ = aabb.minZ;
    this.maxX = aabb.maxX;
    this.maxY = aabb.maxY;
    this.maxZ = aabb.maxZ;
  }
}
