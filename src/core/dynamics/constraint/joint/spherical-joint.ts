// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/{SphericalJoint,SphericalJointConfig}.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { TimeStep } from "../../time-step.js";
import { JointConfig } from "./joint-config.js";
import { computeSoftConstraintParameters, Joint, JointType, SOFT } from "./joint.js";
import type { JointSolverInfo } from "./joint-solver-info.js";
import { SpringDamper } from "./spring-damper.js";

export class SphericalJointConfig extends JointConfig {
  /** 弹簧阻尼：frequency 大于 0 时锚点之间像弹簧一样连着 */
  readonly springDamper = new SpringDamper();

  /** 两个刚体在世界坐标的 anchor 处连起来 */
  init(rigidBody1: RigidBody, rigidBody2: RigidBody, anchorX: number, anchorY: number, anchorZ: number): this {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    return this;
  }
}

/** 球关节：两个锚点重合，三个方向都能自由转动（3 行平移约束） */
export class SphericalJoint extends Joint {
  readonly sd: SpringDamper;

  constructor(config: SphericalJointConfig) {
    super(config, JointType.SPHERICAL);
    this.sd = config.springDamper.clone();
  }

  private getInfo(info: JointSolverInfo, timeStep: TimeStep | null, isPositionPart: boolean): void {
    const sd = this.sd;
    // 软约束只在速度阶段处理
    if (sd.frequency > 0 && isPositionPart) return;
    const errorX = this.a2X - this.a1X, errorY = this.a2Y - this.a1Y, errorZ = this.a2Z - this.a1Z;
    let cfm: number, erp: number;
    if (sd.frequency > 0) {
      computeSoftConstraintParameters(sd.frequency, sd.dampingRatio, timeStep!.dt, sd.useSymplecticEuler);
      cfm = SOFT.cfmFactor;
      erp = SOFT.erp;
      cfm *= this.b1.invMass + this.b2.invMass;
    } else {
      cfm = 0;
      erp = this.getErp(timeStep, isPositionPart);
    }
    const linRhsX = errorX * erp, linRhsY = errorY * erp, linRhsZ = errorZ * erp;
    // 角度部分是锚点相对位置的叉乘矩阵取负的各行：-[r]× = [[0, r.z, -r.y], [-r.z, 0, r.x], [r.y, -r.x, 0]]
    // 对角线是 -0（原版先构造叉乘矩阵再整体取负）
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;

    let row = info.addRow(this.impulses[0]);
    row.equalLimit(linRhsX, cfm);
    let j = row.jacobian;
    j.lin1X = 1; j.lin1Y = 0; j.lin1Z = 0;
    j.lin2X = 1; j.lin2Y = 0; j.lin2Z = 0;
    j.ang1X = -0; j.ang1Y = r1Z; j.ang1Z = -r1Y;
    j.ang2X = -0; j.ang2Y = r2Z; j.ang2Z = -r2Y;

    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, cfm);
    j = row.jacobian;
    j.lin1X = 0; j.lin1Y = 1; j.lin1Z = 0;
    j.lin2X = 0; j.lin2Y = 1; j.lin2Z = 0;
    j.ang1X = -r1Z; j.ang1Y = -0; j.ang1Z = r1X;
    j.ang2X = -r2Z; j.ang2Y = -0; j.ang2Z = r2X;

    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, cfm);
    j = row.jacobian;
    j.lin1X = 0; j.lin1Y = 0; j.lin1Z = 1;
    j.lin2X = 0; j.lin2Y = 0; j.lin2Z = 1;
    j.ang1X = r1Y; j.ang1Y = -r1X; j.ang1Z = -0;
    j.ang2X = r2Y; j.ang2Y = -r2X; j.ang2Z = -0;
  }

  override getVelocitySolverInfo(timeStep: TimeStep, info: JointSolverInfo): void {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }

  override getPositionSolverInfo(info: JointSolverInfo): void {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
}
