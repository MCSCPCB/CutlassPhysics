// 移植自 OimoPhysics src/oimo/common/Vec3.hx（MIT，见 Credits.md）
// GJK/EPA 使用预分配向量对象，接触求解使用标量字段。

/** 三维向量 */
export class Vec3 {
  x: number;
  y: number;
  z: number;

  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  init(x: number, y: number, z: number): this {
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }

  zero(): this {
    this.x = 0;
    this.y = 0;
    this.z = 0;
    return this;
  }

  copyFrom(v: Vec3): this {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
    return this;
  }

  // xxxEq 方法原地更新向量。

  addEq(v: Vec3): this {
    return this.init(this.x + v.x, this.y + v.y, this.z + v.z);
  }

  subEq(v: Vec3): this {
    return this.init(this.x - v.x, this.y - v.y, this.z - v.z);
  }

  scaleEq(s: number): this {
    return this.init(this.x * s, this.y * s, this.z * s);
  }

  /** this += v * s */
  addScaledEq(v: Vec3, s: number): this {
    return this.init(this.x + v.x * s, this.y + v.y * s, this.z + v.z * s);
  }

  negateEq(): this {
    return this.init(-this.x, -this.y, -this.z);
  }

  dot(v: Vec3): number {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }

  lengthSq(): number {
    return this.x * this.x + this.y * this.y + this.z * this.z;
  }

  length(): number {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }

  /** 归一化写回自己；零向量保持为零 */
  normalize(): this {
    let invLen = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    if (invLen > 0) invLen = 1 / invLen;
    return this.init(this.x * invLen, this.y * invLen, this.z * invLen);
  }
}
