// 移植自 OimoPhysics src/oimo/dynamics/constraint/joint/SpringDamper.hx（MIT，见 Credits.md）

/** 弹簧阻尼：frequency 为 0 时约束是硬的，大于 0 时约束变软（像弹簧） */
export class SpringDamper {
  /** 弹簧的频率（Hz） */
  frequency = 0;
  /** 阻尼比：1 是临界阻尼 */
  dampingRatio = 0;
  /** true：辛欧拉（能量守恒好，但高频时不稳定）；false：隐式欧拉（会损失能量，但一定稳定） */
  useSymplecticEuler = false;

  setSpring(frequency: number, dampingRatio: number): this {
    this.frequency = frequency;
    this.dampingRatio = dampingRatio;
    return this;
  }

  setSymplecticEuler(useSymplecticEuler: boolean): this {
    this.useSymplecticEuler = useSymplecticEuler;
    return this;
  }

  clone(): SpringDamper {
    const sd = new SpringDamper();
    sd.frequency = this.frequency;
    sd.dampingRatio = this.dampingRatio;
    sd.useSymplecticEuler = this.useSymplecticEuler;
    return sd;
  }
}
