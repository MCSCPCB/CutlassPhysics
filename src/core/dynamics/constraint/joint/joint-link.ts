// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/JointLink.hx（MIT，见 THIRD-PARTY-NOTICES.txt）
import type { RigidBody } from "../../rigidbody/rigid-body.js";
import type { Joint } from "./joint.js";

/** 刚体上的关节链表节点：通过它找到关节和另一个刚体 */
export class JointLink {
  prev: JointLink | null = null;
  next: JointLink | null = null;
  readonly joint: Joint;
  other: RigidBody | null = null;

  constructor(joint: Joint) {
    this.joint = joint;
  }
}
