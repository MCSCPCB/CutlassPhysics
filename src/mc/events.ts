// 公开事件类型，使用 subscribe 和 unsubscribe 管理订阅。
import type { Body } from "./body.js";
import type { Joint } from "./joints.js";
import type { Vector3 } from "./math.js";
// 仅在 {@link} 注释中引用，用于让 TypeDoc 解析链接
// eslint-disable-next-line @typescript-eslint/no-unused-vars
import type { PhysicsWorld, PhysicsWorldOptions } from "./world.js";
// eslint-disable-next-line @typescript-eslint/no-unused-vars
import type { blockShapes } from "./block-body.js";

/**
 * 可订阅和取消订阅的事件。
 *
 * @remarks
 *
 * - 事件在 {@link PhysicsWorld.step} 结束后依次触发，`stepEnd` 最后触发。
 * - 回调中的刚体和关节变更在下一步生效。
 * - 回调错误不阻断其余事件；派发结束后抛出首个错误，自动推进时记录到 ContentLog。
 */
export interface Signal<T> {
  /**
   * 订阅事件。
   *
   * @param callback
   * 事件触发时调用的函数。
   * @returns
   * 传入的回调函数，可用于 {@link Signal.unsubscribe}。
   */
  subscribe(callback: (event: T) => void): (event: T) => void;

  /**
   * 取消订阅事件。
   *
   * @param callback
   * 订阅时传入的回调函数。
   */
  unsubscribe(callback: (event: T) => void): void;
}

/**
 * 包含两个刚体开始接触时的信息。
 *
 * @remarks
 * 仅在刚开始接触的那一步触发一次。
 *
 * @example collision-event.ts
 * {@includeCode ../../examples/concepts/collision-event.ts}
 */
export interface BodyCollisionStartEvent {
  /**
   * 接触的第一个刚体。
   */
  bodyA: Body;
  /**
   * 接触的第二个刚体。
   */
  bodyB: Body;
  /**
   * 接触点的平均位置，世界坐标。
   */
  point: Vector3;
  /**
   * 接触面的法线，世界坐标，从刚体 A 指向刚体 B。
   */
  normal: Vector3;
  /** 沿法线方向的相对速度大小，单位为格/秒。 */
  impactSpeed: number;
}

/**
 * 包含两个刚体分开时的信息。
 */
export interface BodyCollisionEndEvent {
  /**
   * 分开的第一个刚体。
   */
  bodyA: Body;
  /**
   * 分开的第二个刚体。
   */
  bodyB: Body;
}

/**
 * 包含刚体开始碰到方块时的信息。
 *
 * @example block-collision-event.ts
 * {@includeCode ../../examples/terrain/block-collision-event.ts}
 */
export interface BlockCollisionStartEvent {
  /**
   * 碰到方块的刚体。
   */
  body: Body;
  /**
   * 被碰到的方块坐标。
   */
  blockLocation: Vector3;
  /**
   * 接触点，世界坐标。
   */
  point: Vector3;
  /**
   * 接触面的法线，世界坐标，从方块指向刚体。
   */
  normal: Vector3;
  /**
   * 沿法线方向的相对速度大小，单位为格/秒。
   */
  impactSpeed: number;
}

/**
 * 包含刚体进入或离开休眠状态时的信息。
 */
export interface BodySleepEvent {
  /**
   * 状态发生变化的刚体。
   */
  body: Body;
}

/**
 * 一次 {@link PhysicsWorld.step} 结束时的信息。
 *
 * @remarks
 * 事件触发时，刚体状态已完成更新。
 *
 * @example print-location.ts
 * {@includeCode ../../examples/display-sync/print-location.ts}
 */
export interface StepEndEvent {
  /**
   * 本次推进的时间，单位为秒，为所有子步之和。
   *
   * @remarks
   * 等于传给 {@link PhysicsWorld.step} 的时间；超出目标耗时只在安全范围内调整子步数，不缩短推进的时间。
   */
  deltaTime: number;
  /**
   * 世界创建以来推进的子步数之和。
   *
   * @remarks
   * 相邻两次的差就是这次推进的子步数。负载太高时子步会减少，见 {@link PhysicsWorldOptions.substeps}。
   */
  stepCount: number;
  /**
   * 本次推进的耗时，单位为毫秒。
   *
   * @remarks
   * 运行环境不提供计时时为 `0`。
   */
  elapsedMs: number;
}

/**
 * 包含关节断开时的信息。
 *
 * @remarks
 * 关节因受力超过 `breakForce` 或 `breakTorque` 而断开。事件触发时，该关节已失效。
 */
export interface JointBreakEvent {
  /**
   * 断开的关节。
   */
  joint: Joint;
}

/**
 * 刚体进入水或岩浆时的信息。
 *
 * @remarks
 *
 * - 由未浸没变为浸没时触发。
 * - 溅水点位于平均接触点上方 0.02 格。
 * - 岩浆事件仅在离开超过 4 tick 后可再次触发。
 * - 无订阅者时不收集接触点。
 */
export interface FluidEntryEvent {
  /**
   * 进入流体的刚体。
   */
  body: Body;
  /**
   * 接触点里向下速度最快的那个，单位为格/秒。向上时为负。
   */
  fastestContactVelocityY: number;
  /**
   * 溅水点，世界坐标。
   */
  point: Vector3;
  /**
   * 浸没部分中心点的最小 x 坐标，世界坐标。
   *
   * @remarks
   * maxX - minX 为接触范围的近似宽度；单点接触时为 0。
   */
  minX: number;
  /**
   * 浸没部分在 x 方向的最大坐标，世界坐标，单位为格。
   */
  maxX: number;
  /**
   * 浸没部分在 z 方向的最小坐标，世界坐标，单位为格。
   */
  minZ: number;
  /**
   * 浸没部分在 z 方向的最大坐标，世界坐标，单位为格。
   */
  maxZ: number;
  /**
   * 刚体包围盒在 x 方向的尺寸，单位为格。
   */
  bodySizeX: number;
  /**
   * 刚体包围盒在 z 方向的尺寸，单位为格。
   */
  bodySizeZ: number;
}

/**
 * 易碎方块被撞碎时的信息。
 *
 * @remarks
 *
 * - 撞击速度达到方块的 `fragileImpactSpeed` 时触发。
 * - 世界方块通过 `blockLocation` 标识；启用 `breakFragileBlocks` 时在下一 tick 破坏并掉落物品。
 * - 刚体方块通过 `shapeIndex` 标识，事件不移除对应形状。
 */
export interface BlockBreakEvent {
  /**
   * 撞上方块的刚体，或者方块所在的刚体。
   */
  body: Body;
  /**
   * 方块的位置。
   */
  blockLocation: Vector3;
  /**
   * 方块的类型标识符。
   */
  typeId: string;
  /**
   * 刚体上的方块在 `shapes` 里的下标；世界里的方块没有这一项。
   */
  shapeIndex?: number;
  /**
   * 撞击速度，单位为格/秒。
   */
  impactSpeed: number;
}

/**
 * 物理世界的事件集合。
 *
 * @remarks
 * 通过 {@link PhysicsWorld.afterEvents} 访问。所有事件均在发生后通知，不可取消。
 */
export interface PhysicsAfterEvents {
  /**
   * 两个刚体开始接触时触发。
   */
  readonly bodyCollisionStart: Signal<BodyCollisionStartEvent>;
  /**
   * 两个刚体分开时触发。
   */
  readonly bodyCollisionEnd: Signal<BodyCollisionEndEvent>;
  /**
   * 刚体开始碰到方块时触发。
   */
  readonly blockCollisionStart: Signal<BlockCollisionStartEvent>;
  /**
   * 刚体进入休眠状态时触发。
   */
  readonly bodySleep: Signal<BodySleepEvent>;
  /**
   * 刚体从休眠状态唤醒时触发。
   */
  readonly bodyWake: Signal<BodySleepEvent>;
  /**
   * 一次推进结束时触发。
   */
  readonly stepEnd: Signal<StepEndEvent>;
  /**
   * 关节断开时触发。
   */
  readonly jointBreak: Signal<JointBreakEvent>;
  /**
   * 刚体进入水时触发。
   */
  readonly waterEntry: Signal<FluidEntryEvent>;
  /**
   * 刚体进入岩浆时触发。
   */
  readonly lavaEntry: Signal<FluidEntryEvent>;
  /**
   * 易碎方块被撞碎时触发。
   */
  readonly blockBreak: Signal<BlockBreakEvent>;
}
