// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/TranslationalLimitMotor.hx（MIT，见 THIRD-PARTY-NOTICES.txt）

/** 关节平移方向的限制和马达 */
export class TranslationalLimitMotor {
  /** 位移下限（米）。lowerLimit > upperLimit 时不限制，相等时锁死在这个位置 */
  lowerLimit = 1;
  /** 位移上限（米） */
  upperLimit = 0;
  /** 马达目标速度，单位为米/秒，默认为 0。 */
  motorSpeed = 0;
  /** 马达的最大推力（N），不大于 0 时马达关闭 */
  motorForce = 0;

  setLimits(lower: number, upper: number): this {
    this.lowerLimit = lower;
    this.upperLimit = upper;
    return this;
  }

  setMotor(speed: number, force: number): this {
    this.motorSpeed = speed;
    this.motorForce = force;
    return this;
  }

  clone(): TranslationalLimitMotor {
    const lm = new TranslationalLimitMotor();
    lm.lowerLimit = this.lowerLimit;
    lm.upperLimit = this.upperLimit;
    lm.motorSpeed = this.motorSpeed;
    lm.motorForce = this.motorForce;
    return lm;
  }
}
