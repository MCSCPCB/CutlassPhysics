// 关节。
import type { Joint as CoreJoint } from "../core/dynamics/constraint/joint/joint.js";
import type { Body } from "./body.js";
import type { Vector3 } from "./math.js";
import type { PhysicsWorld } from "./world.js";

/**
 * 描述关节约束的弹簧阻尼。
 *
 * @remarks
 * `frequency` 为 `0` 时，约束为硬约束。
 */
export interface SpringOptions {
  /**
   * 弹簧的振动频率，单位为赫兹。
   *
   * @remarks
   * 频率越高，约束越硬。
   */
  frequency: number;
  /**
   * 阻尼比。`0` 表示无阻尼，`1` 表示刚好不发生振荡。
   *
   * @remarks
   * 默认值为 `1`。
   */
  dampingRatio?: number;
  /**
   * 是否使用辛欧拉积分。
   *
   * @remarks
   * 默认值为 `false`。辛欧拉积分的能量守恒更好，但频率较高时会发散。隐式欧拉积分会损失少量能量，但始终稳定。
   */
  useSymplecticEuler?: boolean;
}

/**
 * 描述绕轴转动自由度的限制和马达。角度单位为度。
 *
 * @remarks
 * `lower`、`upper` 都省略时不限制；只给一侧时，另一侧为 `-180` 或 `180`。
 * 数值可以是 `Infinity`（等同于不限制），不能是 `NaN`。
 */
export interface AngularLimitOptions {
  /**
   * 角度下限。
   *
   * @remarks
   * 大于 `upper` 时不限制，等于 `upper` 时锁定在该角度。
   */
  lower?: number;
  /**
   * 角度上限。
   */
  upper?: number;
  /**
   * 用于替代硬限制的弹簧。
   */
  spring?: SpringOptions;
  /**
   * 马达的目标转速，单位为度/秒。
   */
  motorSpeed?: number;
  /**
   * 马达的最大扭矩，单位为牛顿·格。
   *
   * @remarks
   * 默认值为 `0`，表示不启用马达。`Infinity` 表示不限制扭矩。
   */
  motorMaxTorque?: number;
}

/**
 * 描述沿轴滑动自由度的限制和马达。长度单位为格。
 *
 * @remarks
 * `lower`、`upper` 都省略时不限制；只给一侧时，另一侧为 `-1e9` 或 `1e9` 格。
 * 数值可以是 `Infinity`（等同于不限制），不能是 `NaN`。
 */
export interface LinearLimitOptions {
  /**
   * 位移下限。
   *
   * @remarks
   * 大于 `upper` 时不限制，等于 `upper` 时锁定在该位移。
   */
  lower?: number;
  /**
   * 位移上限。
   */
  upper?: number;
  /**
   * 用于替代硬限制的弹簧。
   */
  spring?: SpringOptions;
  /**
   * 马达的目标速度，单位为格/秒。
   */
  motorSpeed?: number;
  /**
   * 马达的最大推力，单位为牛顿。
   *
   * @remarks
   * 默认值为 `0`，表示不启用马达。`Infinity` 表示不限制推力。
   */
  motorMaxForce?: number;
}

/**
 * 所有关节共有的选项。
 */
export interface JointBaseOptions {
  /**
   * 关节连接的第一个刚体。
   */
  bodyA: Body;
  /**
   * 关节连接的第二个刚体。
   *
   * @remarks
   * 两个刚体必须位于同一个世界，且至少有一个是动态刚体。
   */
  bodyB: Body;
  /**
   * 关节的连接点，世界坐标。
   *
   * @remarks
   * 创建时两个刚体的当前位置决定关节的初始状态。此后各刚体分别记录连接点在自身坐标系中的位置。
   */
  anchor: Vector3;
  /**
   * 两个刚体之间是否仍然发生碰撞。
   *
   * @remarks
   * 默认值为 `false`。
   */
  allowCollision?: boolean;
  /**
   * 关节断开的受力阈值，单位为牛顿。
   *
   * @remarks
   * 默认值为 `0`，表示不会断开。`Infinity` 和负数也表示不会断开。
   *
   * @example break-force.ts
   * {@includeCode ../../examples/joints/break-force.ts}
   */
  breakForce?: number;
  /**
   * 关节断开的力矩阈值，单位为牛顿·格。
   *
   * @remarks
   * 默认值为 `0`，表示不会断开。`Infinity` 和负数也表示不会断开。
   */
  breakTorque?: number;
  /**
   * 关节的求解方式。
   *
   * @remarks
   * 默认为 iterative。direct 使用直接求解，通常精度和计算开销更高。
   */
  solver?: "iterative" | "direct";
}

/**
 * 描述球关节的选项。球关节使两个刚体共用一个点，并允许任意转动。
 *
 * @example ball-chain.ts
 * {@includeCode ../../examples/joints/ball-chain.ts}
 */
export interface BallJointOptions extends JointBaseOptions {
  /** 关节类型：球关节 */
  type: "ball";
  /**
   * 用于替代硬连接的弹簧。
   */
  spring?: SpringOptions;
}

/**
 * 描述铰链关节的选项。铰链关节允许两个刚体绕一根轴相对转动。
 *
 * @example hinge-door.ts
 * {@includeCode ../../examples/joints/hinge-door.ts}
 */
export interface HingeJointOptions extends JointBaseOptions {
  /** 关节类型：铰链关节 */
  type: "hinge";
  /**
   * 转轴方向，世界坐标。无需归一化。
   */
  axis: Vector3;
  /**
   * 转角的限制和马达。
   */
  limit?: AngularLimitOptions;
}

/**
 * 描述滑动关节的选项。滑动关节允许两个刚体沿一根轴相对滑动，不允许转动。
 */
export interface SliderJointOptions extends JointBaseOptions {
  /** 关节类型：滑动关节 */
  type: "slider";
  /**
   * 滑动方向，世界坐标。无需归一化。
   */
  axis: Vector3;
  /**
   * 位移的限制和马达。
   */
  limit?: LinearLimitOptions;
}

/**
 * 描述圆柱关节的选项。圆柱关节允许两个刚体沿同一根轴相对滑动并绕该轴转动。
 */
export interface CylinderJointOptions extends JointBaseOptions {
  /** 关节类型：圆柱关节 */
  type: "cylinder";
  /**
   * 轴方向，世界坐标。无需归一化。
   */
  axis: Vector3;
  /**
   * 位移的限制和马达。
   */
  linearLimit?: LinearLimitOptions;
  /**
   * 转角的限制和马达。
   */
  angularLimit?: AngularLimitOptions;
}

/**
 * 描述万向节的选项。万向节具有两根互相垂直的转轴。
 */
export interface UniversalJointOptions extends JointBaseOptions {
  /** 关节类型：万向节 */
  type: "universal";
  /**
   * 刚体 A 上的转轴，世界坐标。
   */
  axisA: Vector3;
  /**
   * 刚体 B 上的转轴，世界坐标。需要与 `axisA` 垂直。
   */
  axisB: Vector3;
  /**
   * 绕 `axisA` 转动的限制和马达。
   */
  limitA?: AngularLimitOptions;
  /**
   * 绕 `axisB` 转动的限制和马达。
   */
  limitB?: AngularLimitOptions;
}

/**
 * 描述布娃娃关节的选项。布娃娃关节由扭转和锥形摆动组成。
 */
export interface RagdollJointOptions extends JointBaseOptions {
  /** 关节类型：布娃娃关节 */
  type: "ragdoll";
  /**
   * 扭转轴，世界坐标。
   */
  twistAxis: Vector3;
  /**
   * 摆动轴，世界坐标。创建时需要与扭转轴垂直。
   */
  swingAxis: Vector3;
  /**
   * 绕 X 方向的最大摆动角，单位为度。
   */
  maxSwingAngleX?: number;
  /**
   * 绕 Z 方向的最大摆动角，单位为度。
   */
  maxSwingAngleZ?: number;
  /**
   * 扭转角的限制和马达。
   */
  twistLimit?: AngularLimitOptions;
  /**
   * 摆动使用的弹簧。
   */
  swingSpring?: SpringOptions;
}

/**
 * 描述通用六自由度关节的选项。三个平移自由度和三个转动自由度可分别设置。
 */
export interface GenericJointOptions extends JointBaseOptions {
  /** 关节类型：通用六自由度关节 */
  type: "generic";
  /**
   * X、Y、Z 三个方向的平移限制。长度必须为 `3`。
   */
  linearLimits?: [LinearLimitOptions, LinearLimitOptions, LinearLimitOptions];
  /**
   * 绕 X、Y、Z 三个轴的转动限制。长度必须为 `3`。
   */
  angularLimits?: [AngularLimitOptions, AngularLimitOptions, AngularLimitOptions];
}

/**
 * 描述关节的选项，通过 `type` 区分种类。
 */
export type JointOptions =
  | BallJointOptions
  | HingeJointOptions
  | SliderJointOptions
  | CylinderJointOptions
  | UniversalJointOptions
  | RagdollJointOptions
  | GenericJointOptions;

/**
 * 表示连接两个刚体的关节。
 *
 * @remarks
 * 使用 {@link PhysicsWorld.createJoint} 创建。
 */
export class Joint {
  /** 内部使用：内核里的关节
   * @internal */
  readonly core: CoreJoint;
  private readonly owner: PhysicsWorld;
  private readonly a: Body;
  private readonly b: Body;
  private readonly kind: JointOptions["type"];
  private valid = true;

  /**
   * 由 PhysicsWorld.createJoint 创建。
   * @internal
   */
  constructor(owner: PhysicsWorld, core: CoreJoint, a: Body, b: Body, kind: JointOptions["type"]) {
    this.owner = owner;
    this.core = core;
    this.a = a;
    this.b = b;
    this.kind = kind;
  }

  /** 内部使用：关节被移除、断开或所在的世界销毁时调用
   * @internal */
  invalidate(): void {
    this.valid = false;
  }

  private check(): void {
    if (!this.valid) throw new Error("关节已经失效（已被移除或断开，或所在的世界已销毁）");
  }

  /**
   * 关节所在的世界。
   */
  get world(): PhysicsWorld {
    return this.owner;
  }

  /**
   * 关节连接的第一个刚体。
   */
  get bodyA(): Body {
    return this.a;
  }

  /**
   * 关节连接的第二个刚体。
   */
  get bodyB(): Body {
    return this.b;
  }

  /**
   * 关节的种类，与创建时的 `type` 一致。
   */
  get type(): JointOptions["type"] {
    return this.kind;
  }

  /**
   * 关节是否有效。
   *
   * @remarks
   * 关节被移除或断开，或任一刚体被移除后为 `false`。
   */
  get isValid(): boolean {
    return this.valid;
  }

  /**
   * 连接点在刚体 A 上的世界坐标。
   */
  get anchorA(): Vector3 {
    this.check();
    const j = this.core;
    j.syncAnchors();
    return { x: j.a1X, y: j.a1Y, z: j.a1Z };
  }

  /**
   * 连接点在刚体 B 上的世界坐标。
   */
  get anchorB(): Vector3 {
    this.check();
    const j = this.core;
    j.syncAnchors();
    return { x: j.a2X, y: j.a2Y, z: j.a2Z };
  }

  /**
   * 上一步关节承受的力，单位为牛顿。
   *
   * @remarks
   * 可用于判断关节是否接近断开。
   */
  get appliedForce(): Vector3 {
    this.check();
    return { x: this.core.appliedForceX, y: this.core.appliedForceY, z: this.core.appliedForceZ };
  }

  /**
   * 上一步关节承受的力矩，单位为牛顿·格。
   */
  get appliedTorque(): Vector3 {
    this.check();
    return { x: this.core.appliedTorqueX, y: this.core.appliedTorqueY, z: this.core.appliedTorqueZ };
  }

  /**
   * 移除关节。
   *
   * @remarks
   * 两个刚体恢复自由。重复调用没有额外效果。
   */
  remove(): void {
    if (!this.valid) return;
    this.owner.removeJoint(this);
  }
}
