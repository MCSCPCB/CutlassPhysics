// 移植自 OimoPhysics src/oimo/common/Mat3.hx（MIT，见 Credits.md）
// 关节局部坐标系的矩阵存储。

/** 3×3 矩阵，行优先：eRC 是第 R 行第 C 列 */
export class Mat3 {
  e00: number; e01: number; e02: number;
  e10: number; e11: number; e12: number;
  e20: number; e21: number; e22: number;

  /** 默认是单位矩阵 */
  constructor(e00 = 1, e01 = 0, e02 = 0, e10 = 0, e11 = 1, e12 = 0, e20 = 0, e21 = 0, e22 = 1) {
    this.e00 = e00; this.e01 = e01; this.e02 = e02;
    this.e10 = e10; this.e11 = e11; this.e12 = e12;
    this.e20 = e20; this.e21 = e21; this.e22 = e22;
  }

  init(e00: number, e01: number, e02: number, e10: number, e11: number, e12: number, e20: number, e21: number, e22: number): this {
    this.e00 = e00; this.e01 = e01; this.e02 = e02;
    this.e10 = e10; this.e11 = e11; this.e12 = e12;
    this.e20 = e20; this.e21 = e21; this.e22 = e22;
    return this;
  }
}
