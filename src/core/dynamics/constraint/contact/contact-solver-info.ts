// 移植自 OimoPhysics src/oimo/dynamics/constraint/info/{JacobianRow,contact/ContactSolverInfo,contact/ContactSolverInfoRow}.hx
// （MIT，见 Credits.md）。
import { MAX_MANIFOLD_POINTS } from "../../../settings.js";
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { ContactImpulse } from "./contact-impulse.js";

const BIT_LINEAR_SET = 1;
const BIT_ANGULAR_SET = 2;

/** 约束的一行雅可比矩阵：作用在两个刚体上的线性部分和角度部分。接触和关节共用 */
export class JacobianRow {
  lin1X = 0; lin1Y = 0; lin1Z = 0;
  lin2X = 0; lin2Y = 0; lin2Z = 0;
  ang1X = 0; ang1Y = 0; ang1Z = 0;
  ang2X = 0; ang2Y = 0; ang2Z = 0;
  /** 稀疏标记：线性部分、角度部分是否非零（关节求解器用来跳过全零的部分） */
  private flag = 0;

  clear(): void {
    this.lin1X = 0; this.lin1Y = 0; this.lin1Z = 0;
    this.lin2X = 0; this.lin2Y = 0; this.lin2Z = 0;
    this.ang1X = 0; this.ang1Y = 0; this.ang1Z = 0;
    this.ang2X = 0; this.ang2Y = 0; this.ang2Z = 0;
  }

  updateSparsity(): void {
    this.flag = 0;
    if (!(this.lin1X === 0 && this.lin1Y === 0 && this.lin1Z === 0) || !(this.lin2X === 0 && this.lin2Y === 0 && this.lin2Z === 0)) {
      this.flag |= BIT_LINEAR_SET;
    }
    if (!(this.ang1X === 0 && this.ang1Y === 0 && this.ang1Z === 0) || !(this.ang2X === 0 && this.ang2Y === 0 && this.ang2Z === 0)) {
      this.flag |= BIT_ANGULAR_SET;
    }
  }

  isLinearSet(): boolean {
    return (this.flag & BIT_LINEAR_SET) !== 0;
  }

  isAngularSet(): boolean {
    return (this.flag & BIT_ANGULAR_SET) !== 0;
  }
}

/** 一个接触点交给求解器的数据：法向、两个切向的雅可比，目标速度，摩擦系数，累积冲量 */
export class ContactSolverInfoRow {
  readonly jacobianN = new JacobianRow();
  readonly jacobianT = new JacobianRow();
  readonly jacobianB = new JacobianRow();
  /** 法向的目标相对速度（反弹、Baumgarte 位置修正） */
  rhs = 0;
  friction = 0;
  impulse: ContactImpulse | null = null;
}

export class ContactSolverInfo {
  b1: RigidBody | null = null;
  b2: RigidBody | null = null;
  numRows = 0;
  readonly rows: ContactSolverInfoRow[] = [];

  constructor() {
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.rows.push(new ContactSolverInfoRow());
  }
}
