import { DragConstraint } from "../core/dynamics/constraint/drag-constraint.js";
import type { Body } from "./body.js";
import type { Quaternion, Vector3 } from "./math.js";

/** 抓取约束的参数。 */
export interface DragOptions {
  /** 抓取点的世界坐标。 */
  anchor: Vector3;
  /** 位置弹簧频率，单位为 Hz，默认 4。 */
  frequency?: number;
  /** 位置弹簧阻尼比，默认 1。 */
  dampingRatio?: number;
  /** 最大线加速度，单位为格/秒²，默认 60。 */
  maxAcceleration?: number;
  /** 是否保持目标姿态，默认 true。 */
  holdRotation?: boolean;
  /** 姿态弹簧频率，单位为 Hz，默认 3。 */
  angularFrequency?: number;
  /** 姿态弹簧阻尼比，默认 1。 */
  angularDampingRatio?: number;
  /** 最大角加速度，单位为弧度/秒²，默认 40。 */
  maxAngularAcceleration?: number;
}

function finite(point: Vector3): void {
  if (!Number.isFinite(point.x) || !Number.isFinite(point.y) || !Number.isFinite(point.z)) throw new Error("抓取坐标必须是有限数");
}

/**
 * 将刚体上的抓取点牵引至目标，同时保持指定姿态。
 * 同一刚体只能有一个抓取约束；该约束不写入快照。
 */
export class Drag {
  private readonly constraint: DragConstraint;

  /** @internal */
  constructor(readonly body: Body, options: DragOptions) {
    if (body.type !== "dynamic") throw new Error("只能抓取动态刚体");
    finite(options.anchor);
    const d = new DragConstraint(body.core), local = body.worldToLocal(options.anchor);
    this.constraint = d;
    d.lax = local.x; d.lay = local.y; d.laz = local.z;
    d.x = d.px = options.anchor.x; d.y = d.py = options.anchor.y; d.z = d.pz = options.anchor.z;
    const q = body.rotation;
    d.qx = q.x; d.qy = q.y; d.qz = q.z; d.qw = q.w;
    for (const [key, value] of [
      ["hertz", options.frequency], ["dampingRatio", options.dampingRatio], ["maxAcceleration", options.maxAcceleration],
      ["angularHertz", options.angularFrequency], ["angularDampingRatio", options.angularDampingRatio], ["maxAngularAcceleration", options.maxAngularAcceleration],
    ] as const) {
      if (value === undefined) continue;
      if (!Number.isFinite(value) || value <= 0) throw new Error(`${key} 必须是大于零的有限数`);
      d[key] = value;
    }
    if (options.holdRotation !== undefined) d.holdRotation = options.holdRotation;
    body.core.drag = d;
    body.wakeUp();
  }

  /** 刚体移除、类型改变、约束替换或释放后为 false。 */
  get isValid(): boolean {
    return this.body.isValid && this.body.core.drag === this.constraint;
  }

  /** 更新下一次步进的目标位置和姿态。 */
  setTarget(location: Vector3, rotation?: Quaternion): void {
    if (!this.isValid) throw new Error("抓取约束已经失效");
    finite(location);
    const d = this.constraint;
    if (rotation !== undefined) {
      finite(rotation);
      const length = Math.hypot(rotation.x, rotation.y, rotation.z, rotation.w);
      if (!(length > 0) || !Number.isFinite(length)) throw new Error("抓取姿态必须是非零有限四元数");
      d.qx = rotation.x / length; d.qy = rotation.y / length; d.qz = rotation.z / length; d.qw = rotation.w / length;
    }
    d.px = d.x; d.py = d.y; d.pz = d.z;
    d.x = location.x; d.y = location.y; d.z = location.z;
    this.body.wakeUp();
  }

  /** 释放约束，保留刚体当前速度。重复调用无作用。 */
  remove(): void {
    if (this.isValid) { this.body.core.drag = null; this.body.wakeUp(); }
  }
}
