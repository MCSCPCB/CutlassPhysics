// 刚体。
import { RigidBodyType } from "../core/dynamics/rigidbody/rigid-body-config.js";
import type { RigidBody } from "../core/dynamics/rigidbody/rigid-body.js";
import { matrixToQuaternion, quaternionToMatrix } from "./convert.js";
import type { Quaternion, Vector3 } from "./math.js";
import type { ShapeOptions } from "./shapes.js";
import type { PhysicsWorld } from "./world.js";

function finite3(name: string, v: Vector3): void {
  if (!Number.isFinite(v.x) || !Number.isFinite(v.y) || !Number.isFinite(v.z)) {
    throw new Error(`${name} 必须是有限的数，收到 (${v.x}, ${v.y}, ${v.z})`);
  }
}

function nonNegative(name: string, v: number): number {
  if (!(v >= 0) || !Number.isFinite(v)) throw new Error(`${name} 必须是不小于零的有限数，收到 ${v}`);
  return v;
}

/**
 * 刚体的运动类型。
 *
 * @remarks
 * `"dynamic"` 表示受重力、力和碰撞影响。`"static"` 表示不运动，质量视为无穷大。`"kinematic"` 表示按设定的速度运动，不受力和碰撞影响，但会推动动态刚体。
 */
export type BodyType = "dynamic" | "static" | "kinematic";

/**
 * 创建刚体时使用的选项。
 */
export interface BodyOptions {
  /**
   * 刚体的运动类型。
   *
   * @remarks
   * 默认值为 `"dynamic"`。
   */
  type?: BodyType;
  /**
   * 刚体的碰撞形状，至少包含一个。
   *
   * @remarks
   * 多个形状组成复合形状，可表示凹形物体。动态刚体的质量和转动惯量由形状体积与密度计算。
   *
   * @example compound-shape.ts
   * {@includeCode ../../examples/concepts/compound-shape.ts}
   */
  shapes: ShapeOptions[];
  /**
   * 刚体的初始位置，单位为格，指刚体原点的位置。
   *
   * @remarks
   * 默认值为 `{ x: 0, y: 0, z: 0 }`。
   */
  location?: Vector3;
  /**
   * 刚体的初始朝向。
   *
   * @remarks
   * 默认值为没有旋转。
   */
  rotation?: Quaternion;
  /**
   * 刚体的初始线速度，单位为格/秒。
   *
   * @remarks
   * 默认值为 `{ x: 0, y: 0, z: 0 }`。
   */
  linearVelocity?: Vector3;
  /**
   * 刚体的初始角速度，单位为弧度/秒。
   *
   * @remarks
   * 默认值为 `{ x: 0, y: 0, z: 0 }`。
   */
  angularVelocity?: Vector3;
  /**
   * 线速度每秒衰减的比例，相当于空气阻力。
   *
   * @remarks
   * 默认值为 `0.09`。必须是不小于 `0` 的有限数，`0` 表示没有空气阻力。
   */
  linearDamping?: number;
  /**
   * 角速度每秒衰减的比例。
   *
   * @remarks
   * 默认值为 `0.09`。必须是不小于 `0` 的有限数。
   */
  angularDamping?: number;
  /**
   * 重力倍数。`0` 表示不受重力影响。
   *
   * @remarks
   * 默认值为 `1`。
   */
  gravityScale?: number;
  /**
   * 刚体静止一段时间后是否自动休眠。
   *
   * @remarks
   * 默认值为 `true`。休眠的刚体不参与计算，被其他物体碰到时会被唤醒。
   */
  allowSleep?: boolean;
  /**
   * 与刚体关联的自定义数据。
   *
   * @remarks
   * 该值不会被库读取，可通过 {@link Body.userData} 取回。
   *
   * @example sync-entity.ts
   * {@includeCode ../../examples/display-sync/sync-entity.ts}
   */
  userData?: unknown;
}

/**
 * 表示物理世界中的刚体。
 *
 * @remarks
 * 由 {@link PhysicsWorld.createBody} 创建。位置、朝向和速度为最近一步结束后的值，读取时返回新对象。
 */
export class Body {
  /** 内部使用：内核里的刚体
   * @internal */
  readonly core: RigidBody;
  private readonly owner: PhysicsWorld;
  private valid = true;
  /** 上一次被世界推进之前的位置和朝向（用于插值），由世界在每一步开始时记录 */
  private prevPos: Vector3 = { x: 0, y: 0, z: 0 };
  private prevRot: Quaternion = { x: 0, y: 0, z: 0, w: 1 };
  private userDataValue: unknown;

  /** 由 PhysicsWorld.createBody 创建。@internal */
  constructor(owner: PhysicsWorld, core: RigidBody, userData: unknown) {
    this.owner = owner;
    this.core = core;
    this.userDataValue = userData;
    core.userData = this;
  }

  /** 内部使用：世界销毁或刚体移除时调用
   * @internal */
  invalidate(): void {
    this.valid = false;
  }

  private check(): void {
    if (!this.valid) throw new Error("刚体已经失效（已被移除，或所在的世界已销毁）");
  }

  /**
   * 刚体所在的世界。
   */
  get world(): PhysicsWorld {
    return this.owner;
  }

  /**
   * 刚体是否有效。
   *
   * @remarks
   * 刚体被移除或所在的世界被销毁后为 `false`，此后访问该刚体会抛出错误。
   */
  get isValid(): boolean {
    return this.valid;
  }

  /**
   * 与刚体关联的自定义数据。
   */
  get userData(): unknown {
    return this.userDataValue;
  }

  set userData(value: unknown) {
    this.userDataValue = value;
    // encodeUserData 可能把它存进存档
    if (this.valid) this.owner.markChanged();
  }

  /**
   * 刚体的运动类型。
   */
  get type(): BodyType {
    this.check();
    const t = this.core.type;
    return t === RigidBodyType.STATIC ? "static" : t === RigidBodyType.KINEMATIC ? "kinematic" : "dynamic";
  }

  set type(value: BodyType) {
    this.check();
    this.owner.markChanged();
    const t = value === "static" ? RigidBodyType.STATIC : value === "kinematic" ? RigidBodyType.KINEMATIC : value === "dynamic" ? RigidBodyType.DYNAMIC : -1;
    if (t < 0) throw new Error(`未知的刚体类型：${String(value)}`);
    if (t === this.core.type) return;
    this.core.setType(t); // 内部重算质量时会唤醒刚体
  }


  /**
   * 刚体的位置，单位为格。
   *
   * @remarks
   * 设置时保留速度、唤醒刚体，并同步更新 {@link Body.previousLocation}。
   */
  get location(): Vector3 {
    this.check();
    const t = this.core.transform;
    return { x: t.px, y: t.py, z: t.pz };
  }

  set location(value: Vector3) {
    this.check();
    this.owner.markChanged();
    finite3("location", value);
    const c = this.core;
    c.transform.px = value.x; c.transform.py = value.y; c.transform.pz = value.z;
    // 同步前一步位置，避免扫掠范围跨越瞬移路径。
    c.updateTransformExt();
    this.prevPos = { x: value.x, y: value.y, z: value.z };
  }

  /**
   * 刚体的朝向。
   *
   * @remarks
   * 设置时唤醒刚体，并同步更新 {@link Body.previousRotation}。
   */
  get rotation(): Quaternion {
    this.check();
    const t = this.core.transform;
    return matrixToQuaternion(t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22);
  }

  set rotation(value: Quaternion) {
    this.check();
    this.owner.markChanged();
    const m = quaternionToMatrix(value);
    const c = this.core, t = c.transform;
    t.r00 = m[0]; t.r01 = m[1]; t.r02 = m[2]; t.r10 = m[3]; t.r11 = m[4]; t.r12 = m[5]; t.r20 = m[6]; t.r21 = m[7]; t.r22 = m[8];
    c.updateInvInertia();
    c.updateTransformExt();
    this.prevRot = this.rotation;
  }

  /**
   * 刚体的线速度，单位为格/秒。
   */
  get linearVelocity(): Vector3 {
    this.check();
    return { x: this.core.vx, y: this.core.vy, z: this.core.vz };
  }

  set linearVelocity(value: Vector3) {
    this.check();
    this.owner.markChanged();
    finite3("linearVelocity", value);
    this.core.vx = value.x; this.core.vy = value.y; this.core.vz = value.z;
    this.core.wakeUp();
    // 标记预测接触距离失效，下一步按新速度重算。
    this.core.speculativeStale = true;
  }

  /**
   * 刚体的角速度，单位为弧度/秒。
   *
   * @remarks
   * 方向为转轴方向，长度为转速。
   */
  get angularVelocity(): Vector3 {
    this.check();
    return { x: this.core.wx, y: this.core.wy, z: this.core.wz };
  }

  set angularVelocity(value: Vector3) {
    this.check();
    this.owner.markChanged();
    finite3("angularVelocity", value);
    this.core.wx = value.x; this.core.wy = value.y; this.core.wz = value.z;
    this.core.wakeUp();
    this.core.speculativeStale = true;
  }

  /**
   * 线速度的阻尼。
   */
  get linearDamping(): number {
    this.check();
    return this.core.linearDamping;
  }

  set linearDamping(value: number) {
    this.check();
    this.owner.markChanged();
    this.core.linearDamping = nonNegative("linearDamping", value);
  }

  /**
   * 角速度的阻尼。
   */
  get angularDamping(): number {
    this.check();
    return this.core.angularDamping;
  }

  set angularDamping(value: number) {
    this.check();
    this.owner.markChanged();
    this.core.angularDamping = nonNegative("angularDamping", value);
  }

  /**
   * 重力倍数。
   */
  get gravityScale(): number {
    this.check();
    return this.core.gravityScale;
  }

  set gravityScale(value: number) {
    this.check();
    this.owner.markChanged();
    if (!Number.isFinite(value)) throw new Error(`gravityScale 必须是有限的数，收到 ${value}`);
    this.core.gravityScale = value;
    this.core.wakeUp();
  }

  /**
   * 刚体的质量，单位为千克。
   *
   * @remarks
   * 静态刚体和运动学刚体的质量为 `0`。
   */
  get mass(): number {
    this.check();
    return this.core.type === RigidBodyType.DYNAMIC ? this.core.mass : 0;
  }

  /**
   * 刚体浸在水或岩浆里的体积，单位为格³。
   *
   * @remarks
   * 每个 tick 推进前更新；包括空心船体里托住的空气。没有地形、关闭了 `fluids`、或刚体不是动态刚体时为 `0`。
   * 刚体休眠时保留休眠前的值。
   */
  get submerged(): number {
    this.check();
    return this.owner.submergedVolume(this);
  }

  /**
   * 刚体当前是否处于休眠状态。
   */
  get isSleeping(): boolean {
    this.check();
    // 等待地形的冻结状态不计为休眠。
    return this.core.sleeping && !this.owner.isWaiting(this);
  }

  /**
   * 刚体上一步的位置，单位为格。
   *
   * @remarks
   * 与 {@link Body.location} 组成位置插值的两个端点。
   *
   * @example interpolate.ts
   * {@includeCode ../../examples/display-sync/interpolate.ts}
   */
  get previousLocation(): Vector3 {
    this.check();
    return { x: this.prevPos.x, y: this.prevPos.y, z: this.prevPos.z };
  }

  /**
   * 刚体上一步的朝向。
   */
  get previousRotation(): Quaternion {
    this.check();
    return { x: this.prevRot.x, y: this.prevRot.y, z: this.prevRot.z, w: this.prevRot.w };
  }

  /**
   * 记录步进前的位置和朝向，用于插值。
   * @internal
   */
  recordPrevious(): void {
    const t = this.core.transform;
    this.prevPos.x = t.px; this.prevPos.y = t.py; this.prevPos.z = t.pz;
    this.prevRot = matrixToQuaternion(t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22);
  }

  /**
   * 对刚体施加冲量，立即改变其速度。
   *
   * @remarks
   * 速度变化量等于冲量除以质量。仅对动态刚体有效，并会唤醒刚体。
   *
   * @param impulse
   * 冲量，单位为千克·格/秒，世界坐标。
   * @param point
   * 作用点，世界坐标。省略时作用于重心；作用点偏离重心时，同时产生旋转。
   *
   * @example apply-impulse.ts
   * {@includeCode ../../examples/body/apply-impulse.ts}
   */
  applyImpulse(impulse: Vector3, point?: Vector3): void {
    this.check();
    this.owner.markChanged();
    finite3("impulse", impulse);
    if (this.core.type !== RigidBodyType.DYNAMIC) return;
    if (point === undefined) {
      this.core.applyLinearImpulse(impulse.x, impulse.y, impulse.z);
    } else {
      finite3("point", point);
      this.core.applyImpulseAt(impulse.x, impulse.y, impulse.z, point.x, point.y, point.z);
    }
  }

  /**
   * 对刚体施加力。
   *
   * @remarks
   * 力在下一步生效，步进结束后清零。仅对动态刚体有效，并会唤醒刚体。
   *
   * @param force
   * 力，单位为牛顿，世界坐标。
   * @param point
   * 作用点，世界坐标。省略时作用于重心。
   *
   * @example apply-force.ts
   * {@includeCode ../../examples/body/apply-force.ts}
   */
  applyForce(force: Vector3, point?: Vector3): void {
    this.check();
    this.owner.markChanged();
    finite3("force", force);
    if (this.core.type !== RigidBodyType.DYNAMIC) return;
    if (point === undefined) {
      this.core.applyForceToCenter(force.x, force.y, force.z);
    } else {
      finite3("point", point);
      this.core.applyForceAt(force.x, force.y, force.z, point.x, point.y, point.z);
    }
  }

  /**
   * 对刚体施加角冲量，立即改变其角速度。
   *
   * @remarks
   * 仅对动态刚体有效，并会唤醒刚体。
   *
   * @param impulse
   * 角冲量，世界坐标。
   */
  applyAngularImpulse(impulse: Vector3): void {
    this.check();
    this.owner.markChanged();
    finite3("impulse", impulse);
    if (this.core.type !== RigidBodyType.DYNAMIC) return;
    this.core.applyAngularImpulse(impulse.x, impulse.y, impulse.z);
  }

  /**
   * 对刚体施加力矩。
   *
   * @remarks
   * 力矩仅在下一步生效，每步结束后清零。仅对动态刚体有效，并会唤醒刚体。
   *
   * @param torque
   * 力矩，单位为牛顿·格，世界坐标。
   */
  applyTorque(torque: Vector3): void {
    this.check();
    this.owner.markChanged();
    finite3("torque", torque);
    if (this.core.type !== RigidBodyType.DYNAMIC) return;
    this.core.applyTorque(torque.x, torque.y, torque.z);
  }

  /**
   * 唤醒刚体。
   */
  wakeUp(): void {
    this.check();
    this.owner.markChanged();
    this.core.wakeUp();
  }

  /**
   * 使刚体立即进入休眠状态。
   *
   * @remarks
   * 刚体的速度会被保留，被唤醒后继续运动。
   */
  sleep(): void {
    this.check();
    this.owner.markChanged();
    this.core.sleep();
  }

  /**
   * 将世界坐标转换为刚体的局部坐标。
   *
   * @param point
   * 世界坐标。
   * @returns
   * 刚体的局部坐标。
   */
  worldToLocal(point: Vector3): Vector3 {
    this.check();
    finite3("point", point);
    const t = this.core.transform;
    const dx = point.x - t.px, dy = point.y - t.py, dz = point.z - t.pz;
    return {
      x: t.r00 * dx + t.r10 * dy + t.r20 * dz,
      y: t.r01 * dx + t.r11 * dy + t.r21 * dz,
      z: t.r02 * dx + t.r12 * dy + t.r22 * dz,
    };
  }

  /**
   * 将刚体的局部坐标转换为世界坐标。
   *
   * @param point
   * 刚体的局部坐标。
   * @returns
   * 世界坐标。
   */
  localToWorld(point: Vector3): Vector3 {
    this.check();
    finite3("point", point);
    const t = this.core.transform;
    return {
      x: t.r00 * point.x + t.r01 * point.y + t.r02 * point.z + t.px,
      y: t.r10 * point.x + t.r11 * point.y + t.r12 * point.z + t.py,
      z: t.r20 * point.x + t.r21 * point.y + t.r22 * point.z + t.pz,
    };
  }

  /**
   * 返回刚体在某一点的速度。
   *
   * @remarks
   * 该速度等于线速度加上旋转产生的速度。
   *
   * @param point
   * 世界坐标。
   * @returns
   * 该点的速度，单位为格/秒，世界坐标。
   */
  getVelocityAtPoint(point: Vector3): Vector3 {
    this.check();
    finite3("point", point);
    const c = this.core, t = c.transform;
    const rx = point.x - t.px, ry = point.y - t.py, rz = point.z - t.pz;
    return {
      x: c.vx + (c.wy * rz - c.wz * ry),
      y: c.vy + (c.wz * rx - c.wx * rz),
      z: c.vz + (c.wx * ry - c.wy * rx),
    };
  }

  /**
   * 将刚体从世界中移除。
   *
   * @remarks
   * 与该刚体相连的关节同时被移除。重复调用没有额外效果。
   *
   * @example remove.ts
   * {@includeCode ../../examples/body/remove.ts}
   */
  remove(): void {
    if (!this.valid) return;
    this.owner.removeBody(this);
  }
}
