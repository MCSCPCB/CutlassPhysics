// 移植自 OimoPhysics src/oimo/dynamics/constraint/solver/direct/MassMatrix.hx（MIT，见 Credits.md）
// 子矩阵采用通用循环求逆。
import type { JointSolverInfo, JointSolverMassDataRow } from "../joint/joint-solver-info.js";

/** 关节约束的逆质量矩阵 J·M⁻¹·Jᵀ（直接求解器用），以及按行子集缓存的子矩阵的逆 */
export class MassMatrix {
  readonly size: number;
  /** 逆质量矩阵：含 CFM、不含 CFM */
  readonly invMass: number[][] = [];
  readonly invMassWithoutCfm: number[][] = [];
  /** 按行子集（二进制位表示选了哪些行）缓存的子矩阵的逆，以及缓存是否有效 */
  readonly cachedSubmatrices: number[][][] = [];
  readonly cacheComputed: boolean[] = [];
  readonly maxSubmatrixId: number;
  /** 求逆时用的临时矩阵 */
  private readonly tmpMatrix: number[][] = [];

  constructor(size: number) {
    this.size = size;
    for (let i = 0; i < size; i++) {
      this.tmpMatrix.push(new Array<number>(size).fill(0));
      this.invMass.push(new Array<number>(size).fill(0));
      this.invMassWithoutCfm.push(new Array<number>(size).fill(0));
    }
    this.maxSubmatrixId = 1 << size;
    for (let id = 0; id < this.maxSubmatrixId; id++) {
      // 子矩阵的阶数 = id 里 1 的个数
      let n = 0;
      for (let t = id; t !== 0; t >>= 1) n += t & 1;
      const sub: number[][] = [];
      for (let j = 0; j < n; j++) sub.push(new Array<number>(n).fill(0));
      this.cacheComputed.push(false);
      this.cachedSubmatrices.push(sub);
    }
  }

  /** 取出 indices 前 size 个行列组成的子矩阵，求逆后放进第 id 个缓存（高斯-约当消元，不选主元） */
  private computeSubmatrix(id: number, indices: number[], size: number): void {
    const src = this.tmpMatrix, dst = this.cachedSubmatrices[id];
    for (let i = 0; i < size; i++) {
      const ii = indices[i];
      for (let j = 0; j < size; j++) src[i][j] = this.invMass[ii][indices[j]];
    }
    for (let i = 0; i < size; i++) {
      const srci = src[i], dsti = dst[i];
      const diag = 1 / srci[i];
      dsti[i] = diag;
      for (let j = 0; j < i; j++) dsti[j] = dsti[j] * diag;
      for (let j = i + 1; j < size; j++) srci[j] = srci[j] * diag;
      for (let j = 0; j < i; j++) {
        const srcj = src[j], dstj = dst[j];
        for (let k = 0; k < j + 1; k++) dstj[k] = dstj[k] - dsti[k] * srcj[i];
        for (let k = i + 1; k < size; k++) srcj[k] = srcj[k] - srci[k] * srcj[i];
      }
      for (let j = i + 1; j < size; j++) {
        const srcj = src[j], dstj = dst[j];
        for (let k = 0; k < i; k++) dstj[k] = dstj[k] - dsti[k] * srcj[i];
        dstj[i] = -diag * srcj[i];
        for (let k = i + 1; k < size; k++) srcj[k] = srcj[k] - srci[k] * srcj[i];
      }
    }
    // 结果是对称的：上三角从下三角抄过去
    for (let i = 1; i < size; i++) {
      const dsti = dst[i];
      for (let j = 0; j < i; j++) dst[j][i] = dsti[j];
    }
  }

  /** indices 前 n 个行列组成的子矩阵的逆（缓存里没有时现算） */
  getSubmatrix(indices: number[], n: number): number[][] {
    let id = 0;
    for (let i = 0; i < n; i++) id |= 1 << indices[i];
    if (!this.cacheComputed[id]) {
      this.computeSubmatrix(id, indices, n);
      this.cacheComputed[id] = true;
    }
    return this.cachedSubmatrices[id];
  }

  /** 由约束行算出逆质量矩阵，同时把每一行的 M⁻¹·Jᵀ 和有效质量写进 massData；清空子矩阵缓存 */
  computeInvMass(info: JointSolverInfo, massData: JointSolverMassDataRow[]): void {
    const invMass = this.invMass, invMassWithoutCfm = this.invMassWithoutCfm;
    const numRows = info.numRows, b1 = info.b1!, b2 = info.b2!;
    const invM1 = b1.invMass, invM2 = b2.invMass;
    for (let i = 0; i < numRows; i++) {
      const j = info.rows[i].jacobian, md = massData[i];
      j.updateSparsity();
      if (j.isLinearSet()) {
        md.invMLin1X = j.lin1X * invM1; md.invMLin1Y = j.lin1Y * invM1; md.invMLin1Z = j.lin1Z * invM1;
        md.invMLin2X = j.lin2X * invM2; md.invMLin2Y = j.lin2Y * invM2; md.invMLin2Z = j.lin2Z * invM2;
      } else {
        md.invMLin1X = 0; md.invMLin1Y = 0; md.invMLin1Z = 0;
        md.invMLin2X = 0; md.invMLin2Y = 0; md.invMLin2Z = 0;
      }
      if (j.isAngularSet()) {
        md.invMAng1X = b1.ii00 * j.ang1X + b1.ii01 * j.ang1Y + b1.ii02 * j.ang1Z;
        md.invMAng1Y = b1.ii10 * j.ang1X + b1.ii11 * j.ang1Y + b1.ii12 * j.ang1Z;
        md.invMAng1Z = b1.ii20 * j.ang1X + b1.ii21 * j.ang1Y + b1.ii22 * j.ang1Z;
        md.invMAng2X = b2.ii00 * j.ang2X + b2.ii01 * j.ang2Y + b2.ii02 * j.ang2Z;
        md.invMAng2Y = b2.ii10 * j.ang2X + b2.ii11 * j.ang2Y + b2.ii12 * j.ang2Z;
        md.invMAng2Z = b2.ii20 * j.ang2X + b2.ii21 * j.ang2Y + b2.ii22 * j.ang2Z;
      } else {
        md.invMAng1X = 0; md.invMAng1Y = 0; md.invMAng1Z = 0;
        md.invMAng2X = 0; md.invMAng2Y = 0; md.invMAng2Z = 0;
      }
    }
    // J·(M⁻¹·Jᵀ)：只算上三角，下三角对称。运算顺序：lin1 + (ang1) + (lin2) + (ang2)
    for (let i = 0; i < numRows; i++) {
      const j1 = info.rows[i].jacobian;
      for (let j = i; j < numRows; j++) {
        const md2 = massData[j];
        const val =
          j1.lin1X * md2.invMLin1X + j1.lin1Y * md2.invMLin1Y + j1.lin1Z * md2.invMLin1Z +
          (j1.ang1X * md2.invMAng1X + j1.ang1Y * md2.invMAng1Y + j1.ang1Z * md2.invMAng1Z) +
          (j1.lin2X * md2.invMLin2X + j1.lin2Y * md2.invMLin2Y + j1.lin2Z * md2.invMLin2Z) +
          (j1.ang2X * md2.invMAng2X + j1.ang2Y * md2.invMAng2Y + j1.ang2Z * md2.invMAng2Z);
        if (i === j) {
          invMass[i][j] = val + info.rows[i].cfm;
          invMassWithoutCfm[i][j] = val;
          md2.mass = val + info.rows[i].cfm;
          md2.massWithoutCfm = val;
          if (md2.mass !== 0) md2.mass = 1 / md2.mass;
          if (md2.massWithoutCfm !== 0) md2.massWithoutCfm = 1 / md2.massWithoutCfm;
        } else {
          invMass[i][j] = val;
          invMass[j][i] = val;
          invMassWithoutCfm[i][j] = val;
          invMassWithoutCfm[j][i] = val;
        }
      }
    }
    for (let id = 0; id < this.maxSubmatrixId; id++) this.cacheComputed[id] = false;
  }
}
