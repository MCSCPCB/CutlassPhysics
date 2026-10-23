// 移植自 OimoPhysics src/oimo/common/Transform.hx（MIT，见 Credits.md）
// 以标量字段保存平移和旋转矩阵。
// rIJ 是旋转矩阵第 I 行第 J 列；第 J 列是局部坐标系第 J 根轴在世界坐标里的方向。
export class Transform {
  px = 0;
  py = 0;
  pz = 0;

  r00 = 1;
  r01 = 0;
  r02 = 0;

  r10 = 0;
  r11 = 1;
  r12 = 0;

  r20 = 0;
  r21 = 0;
  r22 = 1;

  copyFrom(t: Transform): void {
    this.px = t.px; this.py = t.py; this.pz = t.pz;
    this.r00 = t.r00; this.r01 = t.r01; this.r02 = t.r02;
    this.r10 = t.r10; this.r11 = t.r11; this.r12 = t.r12;
    this.r20 = t.r20; this.r21 = t.r21; this.r22 = t.r22;
  }
}

/**
 * 变换组合：先做 local，再做 parent。
 * 旋转 = R_parent · R_local，位置 = R_parent · p_local + p_parent。dst 不能和输入是同一个对象。
 */
export function transformMul(dst: Transform, local: Transform, parent: Transform): void {
  const a = parent, b = local;
  dst.r00 = a.r00 * b.r00 + a.r01 * b.r10 + a.r02 * b.r20;
  dst.r01 = a.r00 * b.r01 + a.r01 * b.r11 + a.r02 * b.r21;
  dst.r02 = a.r00 * b.r02 + a.r01 * b.r12 + a.r02 * b.r22;
  dst.r10 = a.r10 * b.r00 + a.r11 * b.r10 + a.r12 * b.r20;
  dst.r11 = a.r10 * b.r01 + a.r11 * b.r11 + a.r12 * b.r21;
  dst.r12 = a.r10 * b.r02 + a.r11 * b.r12 + a.r12 * b.r22;
  dst.r20 = a.r20 * b.r00 + a.r21 * b.r10 + a.r22 * b.r20;
  dst.r21 = a.r20 * b.r01 + a.r21 * b.r11 + a.r22 * b.r21;
  dst.r22 = a.r20 * b.r02 + a.r21 * b.r12 + a.r22 * b.r22;
  dst.px = a.r00 * b.px + a.r01 * b.py + a.r02 * b.pz;
  dst.py = a.r10 * b.px + a.r11 * b.py + a.r12 * b.pz;
  dst.pz = a.r20 * b.px + a.r21 * b.py + a.r22 * b.pz;
  dst.px += a.px;
  dst.py += a.py;
  dst.pz += a.pz;
}
