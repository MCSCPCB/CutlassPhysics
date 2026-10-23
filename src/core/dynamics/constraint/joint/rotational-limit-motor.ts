// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/RotationalLimitMotor.hx（MIT，见 THIRD-PARTY-NOTICES.txt）

/** 关节转动方向的限制和马达 */
export class RotationalLimitMotor {
  /** 转角下限（弧度）。lowerLimit > upperLimit 时不限制，相等时锁死在这个角度 */
  lowerLimit = 1;
  /** 转角上限（弧度） */
  upperLimit = 0;
  /** 马达目标角速度，单位为弧度/秒，默认为 0。 */
  motorSpeed = 0;
  /** 马达的最大力矩（N·m），不大于 0 时马达关闭 */
  motorTorque = 0;

  setLimits(lower: number, upper: number): this {
    this.lowerLimit = lower;
    this.upperLimit = upper;
    return this;
  }

  setMotor(speed: number, torque: number): this {
    this.motorSpeed = speed;
    this.motorTorque = torque;
    return this;
  }

  clone(): RotationalLimitMotor {
    const lm = new RotationalLimitMotor();
    lm.lowerLimit = this.lowerLimit;
    lm.upperLimit = this.upperLimit;
    lm.motorSpeed = this.motorSpeed;
    lm.motorTorque = this.motorTorque;
    return lm;
  }
}
