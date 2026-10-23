// 移植自 OimoPhysics src/oimo/dynamics/constraint/contact/ContactConstraint.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { Transform } from "../../../common/transform.js";
import {
  CONTACT_ENABLE_BOUNCE_THRESHOLD,
  DEFAULT_CONTACT_POSITION_CORRECTION_ALGORITHM,
  LINEAR_SLOP,
  VELOCITY_BAUMGARTE,
} from "../../../settings.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { Shape } from "../../rigidbody/shape.js";
import type { TimeStep } from "../../time-step.js";
import { PositionCorrectionAlgorithm } from "../position-correction-algorithm.js";
import { PgsContactConstraintSolver } from "../solver/pgs-contact-constraint-solver.js";
import { SoftContactSolver } from "../solver/soft-contact-solver.js";
import type { ContactSolverInfo, JacobianRow } from "./contact-solver-info.js";
import type { Manifold } from "./manifold.js";
import type { ManifoldPoint } from "./manifold-point.js";

/** 一行雅可比：线性部分是方向 d，角度部分是 相对位置 × d */
function setJacobian(j: JacobianRow, dX: number, dY: number, dZ: number, p: ManifoldPoint): void {
  j.lin1X = dX; j.lin1Y = dY; j.lin1Z = dZ;
  j.lin2X = dX; j.lin2Y = dY; j.lin2Z = dZ;
  j.ang1X = p.rp1Y * dZ - p.rp1Z * dY;
  j.ang1Y = p.rp1Z * dX - p.rp1X * dZ;
  j.ang1Z = p.rp1X * dY - p.rp1Y * dX;
  j.ang2X = p.rp2Y * dZ - p.rp2Z * dY;
  j.ang2Y = p.rp2Z * dX - p.rp2X * dZ;
  j.ang2Z = p.rp2X * dY - p.rp2Y * dX;
}

/** 接触约束：把接触流形整理成求解器需要的雅可比、目标速度等数据 */
export class ContactConstraint {
  /** 这一步用的位置修正算法（Contact 按穿透深度选择） */
  positionCorrectionAlgorithm: number = DEFAULT_CONTACT_POSITION_CORRECTION_ALGORITHM;
  /** true 时恢复系数取两个形状的较大值，false 时取几何平均（原版）。由 ContactManager 在创建接触时设置 */
  maxRestitution = false;
  readonly manifold: Manifold;
  s1: Shape | null = null;
  s2: Shape | null = null;
  b1: RigidBody | null = null;
  b2: RigidBody | null = null;
  tf1: Transform | null = null;
  tf2: Transform | null = null;
  readonly solver: PgsContactConstraintSolver;
  /** 软接触求解器，在 World.softContacts 启用时使用。 */
  readonly softSolver: SoftContactSolver;
  /** 最近参与求解的步进编号，-1 表示尚未参与。预测接触是否生效由 pushed 在求解后确定。 */
  solvedStamp = -1;
  /** 最近一次求解是否施加过正的法向冲量。 */
  pushed = false;
  /** 最近一次求解开始时的最大法向靠近速度，单位为米/秒，用于撞击判定。 */
  approachSpeed = 0;
  /** 上次求解的子步时长，用于缩放热启动冲量。 */
  substepDt = 0;

  constructor(manifold: Manifold) {
    this.solver = new PgsContactConstraintSolver(this);
    this.manifold = manifold;
    this.softSolver = new SoftContactSolver(this);
  }

  attach(s1: Shape, s2: Shape): void {
    this.s1 = s1;
    this.s2 = s2;
    this.b1 = s1.rigidBody;
    this.b2 = s2.rigidBody;
    this.tf1 = this.b1!.transform;
    this.tf2 = this.b2!.transform;
    this.solvedStamp = -1;
    this.pushed = false;
    this.approachSpeed = 0;
    this.substepDt = 0;
  }

  detach(): void {
    this.s1 = null;
    this.s2 = null;
    this.b1 = null;
    this.b2 = null;
    this.tf1 = null;
    this.tf2 = null;
  }

  /** 速度求解的数据（原版 _getVelocitySolverInfo）：每个没分开的接触点一行 */
  getVelocitySolverInfo(timeStep: TimeStep, info: ContactSolverInfo): void {
    const b1 = this.b1!, b2 = this.b2!, m = this.manifold;
    info.b1 = b1;
    info.b2 = b2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ;
    const tX = m.tX, tY = m.tY, tZ = m.tZ;
    const bX = m.bX, bY = m.bY, bZ = m.bZ;
    // 摩擦系数取两侧的几何平均；恢复系数在 maxRestitution 启用时取较大值，否则取几何平均。
    const friction = Math.sqrt(this.s1!.friction * this.s2!.friction);
    const restitution = this.maxRestitution
      ? Math.max(this.s1!.restitution, this.s2!.restitution)
      : Math.sqrt(this.s1!.restitution * this.s2!.restitution);
    info.numRows = 0;
    for (let i = 0; i < m.numPoints; i++) {
      const p = m.points[i];
      if (p.depth < 0) {
        // 已经分开的点：停用，清掉累积冲量
        p.disabled = true;
        p.impulse.clear();
        continue;
      }
      p.disabled = false;
      const row = info.rows[info.numRows++];
      row.friction = friction;
      setJacobian(row.jacobianN, nX, nY, nZ, p);
      setJacobian(row.jacobianT, tX, tY, tZ, p);
      setJacobian(row.jacobianB, bX, bY, bZ, p);
      // 法向相对速度
      const j = row.jacobianN;
      const rvn =
        j.lin1X * b1.vx + j.lin1Y * b1.vy + j.lin1Z * b1.vz + (j.ang1X * b1.wx + j.ang1Y * b1.wy + j.ang1Z * b1.wz) -
        (j.lin2X * b2.vx + j.lin2Y * b2.vy + j.lin2Z * b2.vz + (j.ang2X * b2.wx + j.ang2Y * b2.wy + j.ang2Z * b2.wz));
      // 撞得够快才反弹；热启动过的（持续接触的）点不反弹
      if (rvn < -CONTACT_ENABLE_BOUNCE_THRESHOLD && !p.warmStarted) {
        row.rhs = -rvn * restitution;
      } else {
        row.rhs = 0;
      }
      // Baumgarte：把超出 LINEAR_SLOP 的穿透折算成最小推开速度
      if (this.positionCorrectionAlgorithm === PositionCorrectionAlgorithm.BAUMGARTE && p.depth > LINEAR_SLOP) {
        const minRhs = (p.depth - LINEAR_SLOP) * VELOCITY_BAUMGARTE * timeStep.invDt;
        if (row.rhs < minRhs) row.rhs = minRhs;
      }
      if (!p.warmStarted) p.impulse.clear();
      row.impulse = p.impulse;
    }
  }

  /** 位置修正（split impulse）的数据（原版 _getPositionSolverInfo）：每个点只有法向一行 */
  getPositionSolverInfo(info: ContactSolverInfo): void {
    const m = this.manifold;
    info.b1 = this.b1;
    info.b2 = this.b2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ;
    info.numRows = 0;
    for (let i = 0; i < m.numPoints; i++) {
      const p = m.points[i];
      if (p.disabled) continue;
      const row = info.rows[info.numRows++];
      setJacobian(row.jacobianN, nX, nY, nZ, p);
      row.rhs = p.depth - LINEAR_SLOP;
      if (row.rhs < 0) row.rhs = 0;
      row.impulse = p.impulse;
    }
  }

  /** 刚体移动后同步接触点，供后续求解使用。 */
  syncManifold(): void {
    this.manifold.updateDepthsAndPositions(this.tf1!, this.tf2!);
  }

  /** 是否还有接触点在接触 */
  isTouching(): boolean {
    for (let i = 0; i < this.manifold.numPoints; i++) {
      if (this.manifold.points[i].depth >= 0) return true;
    }
    return false;
  }
}
