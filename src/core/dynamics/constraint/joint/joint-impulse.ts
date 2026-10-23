// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/JointImpulse.hx（MIT，见 Credits.md）

/** 关节一行约束的累积冲量，跨步保存用来热启动 */
export class JointImpulse {
  /** 约束冲量 */
  impulse = 0;
  /** 马达冲量 */
  impulseM = 0;
  /** 位置修正冲量 */
  impulseP = 0;

  clear(): void {
    this.impulse = 0;
    this.impulseM = 0;
    this.impulseP = 0;
  }
}
