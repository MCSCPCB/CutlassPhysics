// 移植自 OimoPhysics src/oimo/dynamics/constraint/solver/direct/{Boundary,BoundaryBuildInfo,BoundaryBuilder,BoundarySelector}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import { DIRECT_MLCP_SOLVER_EPS } from "../../../settings.js";
import type { JointSolverInfo } from "../joint/joint-solver-info.js";
import type { MassMatrix } from "./mass-matrix.js";

/**
 * 混合线性互补问题（MLCP）的一种情况：哪些行的冲量顶在上限或下限（bounded），哪些行在上下限之间（unbounded）。
 * 假设成立时，unbounded 的行可以直接解线性方程组算出来
 */
export class Boundary {
  /** 顶在上下限的行：下标、方向（-1 下限，1 上限，0 这一行不起作用、冲量是 0） */
  numBounded = 0;
  readonly iBounded: number[];
  readonly signs: number[];
  /** 在上下限之间的行的下标 */
  numUnbounded = 0;
  readonly iUnbounded: number[];
  /** 解方程用的右端项：冲量增量 = 子矩阵的逆 × b */
  private readonly b: number[];

  constructor(maxRows: number) {
    this.iBounded = new Array<number>(maxRows).fill(0);
    this.signs = new Array<number>(maxRows).fill(0);
    this.iUnbounded = new Array<number>(maxRows).fill(0);
    this.b = new Array<number>(maxRows).fill(0);
  }

  init(buildInfo: BoundaryBuildInfo): void {
    this.numBounded = buildInfo.numBounded;
    for (let i = 0; i < this.numBounded; i++) {
      this.iBounded[i] = buildInfo.iBounded[i];
      this.signs[i] = buildInfo.signs[i];
    }
    this.numUnbounded = buildInfo.numUnbounded;
    for (let i = 0; i < this.numUnbounded; i++) this.iUnbounded[i] = buildInfo.iUnbounded[i];
  }

  /**
   * 按这种情况算出每一行的冲量增量，写进 dImpulses。返回这种情况是不是答案：
   * unbounded 的冲量没有超出上下限，bounded 的行满足互补条件（容差 DIRECT_MLCP_SOLVER_EPS）
   */
  computeImpulses(
    info: JointSolverInfo, mass: MassMatrix, relVels: number[], impulses: number[], dImpulses: number[], impulseFactor: number,
  ): boolean {
    const b = this.b;
    // b = rhs - 相对速度 - cfm × 累积冲量
    for (let i = 0; i < this.numUnbounded; i++) {
      const idx = this.iUnbounded[i], row = info.rows[idx];
      b[idx] = row.rhs * impulseFactor - relVels[idx] - row.cfm * impulses[idx];
    }
    // 顶在上下限的行：冲量直接取上限或下限，它对 unbounded 那些行的相对速度的影响从 b 里减掉
    const invMassWithoutCfm = mass.invMassWithoutCfm;
    for (let i = 0; i < this.numBounded; i++) {
      const idx = this.iBounded[i], sign = this.signs[i], row = info.rows[idx];
      const dImpulse = (sign < 0 ? row.minImpulse : sign > 0 ? row.maxImpulse : 0) - impulses[idx];
      dImpulses[idx] = dImpulse;
      if (dImpulse !== 0) {
        for (let j = 0; j < this.numUnbounded; j++) {
          const idx2 = this.iUnbounded[j];
          b[idx2] -= invMassWithoutCfm[idx][idx2] * dImpulse;
        }
      }
    }
    // 在上下限之间的行：冲量增量 = 子矩阵的逆 × b，算出来超出上下限就说明这种情况不对
    const massMatrix = mass.getSubmatrix(this.iUnbounded, this.numUnbounded);
    let ok = true;
    for (let i = 0; i < this.numUnbounded; i++) {
      const idx = this.iUnbounded[i], row = info.rows[idx];
      const oldImpulse = impulses[idx];
      let impulse = oldImpulse;
      for (let j = 0; j < this.numUnbounded; j++) impulse += b[this.iUnbounded[j]] * massMatrix[i][j];
      if (impulse < row.minImpulse - DIRECT_MLCP_SOLVER_EPS || impulse > row.maxImpulse + DIRECT_MLCP_SOLVER_EPS) {
        ok = false;
        break;
      }
      dImpulses[idx] = impulse - oldImpulse;
    }
    if (!ok) return false;
    // 互补条件：顶在下限的行加完冲量后误差不能是正的，顶在上限的不能是负的（冲量加多了）
    for (let i = 0; i < this.numBounded; i++) {
      const idx = this.iBounded[i], row = info.rows[idx], sign = this.signs[i];
      const newImpulse = impulses[idx] + dImpulses[idx];
      // relVels 是加冲量之前的相对速度，补上所有行的冲量增量的影响
      let relVel = relVels[idx];
      for (let j = 0; j < info.numRows; j++) relVel += invMassWithoutCfm[idx][j] * dImpulses[j];
      const error = row.rhs * impulseFactor - relVel - row.cfm * newImpulse;
      if (sign < 0 && error > DIRECT_MLCP_SOLVER_EPS || sign > 0 && error < -DIRECT_MLCP_SOLVER_EPS) {
        ok = false;
        break;
      }
    }
    return ok;
  }
}

/** 枚举情况时用的栈：当前已经定下的 bounded、unbounded 的行 */
export class BoundaryBuildInfo {
  numBounded = 0;
  readonly iBounded: number[];
  readonly signs: number[];
  numUnbounded = 0;
  readonly iUnbounded: number[];

  constructor(size: number) {
    this.iBounded = new Array<number>(size).fill(0);
    this.signs = new Array<number>(size).fill(0);
    this.iUnbounded = new Array<number>(size).fill(0);
  }
}

/**
 * 枚举 MLCP 的所有情况：不起作用的行（上下限都是 0）只有一种；其余的行可以在上下限之间，
 * 有下限时还可以顶在下限，有上限时还可以顶在上限（一行最多一个有限的上下限，所以最多 2^行数 种）
 */
export class BoundaryBuilder {
  numBoundaries = 0;
  /** 用到时才创建，创建过的留着下一步复用 */
  readonly boundaries: Boundary[] = [];
  readonly maxBoundaries: number;
  private readonly maxRows: number;
  private readonly bbInfo: BoundaryBuildInfo;

  constructor(maxRows: number) {
    this.maxRows = maxRows;
    this.maxBoundaries = 1 << maxRows;
    this.bbInfo = new BoundaryBuildInfo(maxRows);
  }

  buildBoundaries(info: JointSolverInfo): void {
    this.numBoundaries = 0;
    this.bbInfo.numBounded = 0;
    this.bbInfo.numUnbounded = 0;
    this.buildBoundariesRecursive(info, 0);
  }

  private buildBoundariesRecursive(info: JointSolverInfo, i: number): void {
    const bb = this.bbInfo;
    if (i === info.numRows) {
      if (this.boundaries.length === this.numBoundaries) this.boundaries.push(new Boundary(this.maxRows));
      this.boundaries[this.numBoundaries++].init(bb);
      return;
    }
    const row = info.rows[i];
    const lowerLimitEnabled = row.minImpulse > -Infinity;
    const upperLimitEnabled = row.maxImpulse < Infinity;
    if (row.minImpulse === 0 && row.maxImpulse === 0) {
      // 不起作用的行
      bb.iBounded[bb.numBounded] = i; bb.signs[bb.numBounded] = 0; bb.numBounded++;
      this.buildBoundariesRecursive(info, i + 1);
      bb.numBounded--;
      return;
    }
    // 在上下限之间
    bb.iUnbounded[bb.numUnbounded] = i; bb.numUnbounded++;
    this.buildBoundariesRecursive(info, i + 1);
    bb.numUnbounded--;
    if (lowerLimitEnabled) {
      bb.iBounded[bb.numBounded] = i; bb.signs[bb.numBounded] = -1; bb.numBounded++;
      this.buildBoundariesRecursive(info, i + 1);
      bb.numBounded--;
    }
    if (upperLimitEnabled) {
      bb.iBounded[bb.numBounded] = i; bb.signs[bb.numBounded] = 1; bb.numBounded++;
      this.buildBoundariesRecursive(info, i + 1);
      bb.numBounded--;
    }
  }
}

/** 尝试各种情况的顺序：上一次成功的情况排到最前面，下一次先试它 */
export class BoundarySelector {
  private readonly n: number;
  private indices: number[];
  private tmpIndices: number[];

  constructor(n: number) {
    this.n = n;
    this.indices = [];
    for (let i = 0; i < n; i++) this.indices.push(i);
    this.tmpIndices = new Array<number>(n).fill(0);
  }

  getIndex(i: number): number {
    return this.indices[i];
  }

  /** 把 index 移到最前面，其余的顺序不变 */
  select(index: number): void {
    let i = 0;
    while (this.indices[i] !== index) i++;
    while (i > 0) {
      const tmp = this.indices[i];
      this.indices[i] = this.indices[i - 1];
      this.indices[i - 1] = tmp;
      i--;
    }
  }

  /** 让前 size 个正好是 0 ~ size-1 的一个排列（保持原来的相对顺序） */
  setSize(size: number): void {
    let numSmaller = 0, numGreater = 0;
    for (let i = 0; i < this.n; i++) {
      const idx = this.indices[i];
      if (idx < size) {
        this.tmpIndices[numSmaller] = idx;
        numSmaller++;
      } else {
        this.tmpIndices[size + numGreater] = idx;
        numGreater++;
      }
    }
    const tmp = this.indices;
    this.indices = this.tmpIndices;
    this.tmpIndices = tmp;
  }
}
