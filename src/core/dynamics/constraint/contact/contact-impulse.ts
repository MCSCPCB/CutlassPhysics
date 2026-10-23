// 移植自 OimoPhysics src/oimo/dynamics/constraint/contact/ContactImpulse.hx（MIT，见 Credits.md）

/** 一个接触点上累积的冲量，下一步用来热启动 */
export class ContactImpulse {
  /** 法向冲量 */
  impulseN = 0;
  /** 两个切向（tangent、binormal）的摩擦冲量 */
  impulseT = 0;
  impulseB = 0;
  /** 位置修正（split impulse）的冲量 */
  impulseP = 0;
  /** 摩擦冲量在世界坐标里的向量。切向每步都会重建，热启动时把它投影到新的切向上 */
  impulseLX = 0;
  impulseLY = 0;
  impulseLZ = 0;

  clear(): void {
    this.impulseN = 0;
    this.impulseT = 0;
    this.impulseB = 0;
    this.impulseP = 0;
    this.impulseLX = 0;
    this.impulseLY = 0;
    this.impulseLZ = 0;
  }

  /** 复制另一个冲量。不复制 impulseP */
  copyFrom(imp: ContactImpulse): void {
    this.impulseN = imp.impulseN;
    this.impulseT = imp.impulseT;
    this.impulseB = imp.impulseB;
    this.impulseLX = imp.impulseLX;
    this.impulseLY = imp.impulseLY;
    this.impulseLZ = imp.impulseLZ;
  }
}
