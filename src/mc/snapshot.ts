// 可持久化的物理状态。只保存重建所需的数据，接触缓存和地形碰撞体在恢复后重新建立。
import { BoxGeometry } from "../core/collision/geometry/box-geometry.js";
import { CapsuleGeometry } from "../core/collision/geometry/capsule-geometry.js";
import { ConeGeometry } from "../core/collision/geometry/cone-geometry.js";
import { ConvexHullGeometry } from "../core/collision/geometry/convex-hull-geometry.js";
import { CylinderGeometry } from "../core/collision/geometry/cylinder-geometry.js";
import { SphereGeometry } from "../core/collision/geometry/sphere-geometry.js";
import { CylindricalJoint } from "../core/dynamics/constraint/joint/cylindrical-joint.js";
import { GenericJoint } from "../core/dynamics/constraint/joint/generic-joint.js";
import type { Joint as CoreJoint } from "../core/dynamics/constraint/joint/joint.js";
import { PrismaticJoint } from "../core/dynamics/constraint/joint/prismatic-joint.js";
import { RagdollJoint } from "../core/dynamics/constraint/joint/ragdoll-joint.js";
import { RevoluteJoint } from "../core/dynamics/constraint/joint/revolute-joint.js";
import type { RotationalLimitMotor } from "../core/dynamics/constraint/joint/rotational-limit-motor.js";
import { SphericalJoint } from "../core/dynamics/constraint/joint/spherical-joint.js";
import type { SpringDamper } from "../core/dynamics/constraint/joint/spring-damper.js";
import type { TranslationalLimitMotor } from "../core/dynamics/constraint/joint/translational-limit-motor.js";
import { UniversalJoint } from "../core/dynamics/constraint/joint/universal-joint.js";
import { DirectJointConstraintSolver } from "../core/dynamics/constraint/solver/direct-joint-constraint-solver.js";
import type { Shape } from "../core/dynamics/rigidbody/shape.js";
import { Transform } from "../core/common/transform.js";
import type { Body, BodyOptions } from "./body.js";
import { matrixToQuaternion } from "./convert.js";
import type { Joint, JointOptions } from "./joints.js";
import type { Quaternion, Vector3 } from "./math.js";
import { BlockRegistry, type BlockDefinitions } from "./block-registry.js";
import type { BodyBlock, ShapeBaseOptions, ShapeOptions } from "./shapes.js";
import type { TerrainOptions } from "./terrain.js";
import type { FluidOptions, PhysicsWorld, PhysicsWorldOptions, StepMode, VoidSeaOptions } from "./world.js";

/** 自定义存档数据，只接受 JSON 能无损表达的值。 */
export type JsonValue = null | boolean | number | string | JsonValue[] | { [key: string]: JsonValue };

/** 自定义业务数据的保存方式。默认不保存 Body.userData（其中可能含有实体引用）。 */
export interface SnapshotOptions {
  /** 把刚体的 userData 转成可保存的 JSON 值，结果存进 {@link BodySnapshot.data} */
  encodeUserData?: (body: Body) => JsonValue;
}

/** 恢复时的运行配置。快照不包含自定义碰撞函数和事件订阅。 */
export interface RestoreOptions extends PhysicsWorldOptions {
  /**
   * 将 {@link BodySnapshot.data} 还原成刚体的 `userData`。
   *
   * @remarks
   *
   * - `index` 是刚体在快照中的下标。
   * - 省略时直接使用存档中的值。
   */
  decodeUserData?: (data: JsonValue, index: number) => unknown;
}

/** 单个刚体的存档记录。data 由 encodeUserData 提供。 */
export interface BodySnapshot {
  /** 重新创建刚体用的选项（与 {@link PhysicsWorld.createBody} 相同，全部写明） */
  options: Required<Omit<BodyOptions, "userData">>;
  /** 保存时是否休眠（冻结的刚体记为不休眠） */
  sleeping: boolean;
  /** 等待下一步施加的外力。 */
  force: Vector3;
  /** 等待下一步施加的外力矩。 */
  torque: Vector3;
  /** encodeUserData 的结果；没有给 encodeUserData 时省略 */
  data?: JsonValue;
}

/** 快照里一个弹簧的参数 */
export interface SpringState {
  /** 频率（Hz），0 表示没有弹簧 */
  frequency: number;
  /** 阻尼比 */
  dampingRatio: number;
  /** 是否用辛欧拉积分 */
  useSymplecticEuler: boolean;
}
/** 快照里一个限制和马达的参数 */
export interface LimitState {
  /** 下限 */
  lower: number;
  /** 上限 */
  upper: number;
  /** 马达速度 */
  speed: number;
  /** 马达的最大力（平移）或最大力矩（转动） */
  maximum: number;
}

/** 关节以局部锚点、完整局部坐标系保存；角度和角速度在存档中使用弧度。 */
export interface JointSnapshot {
  /** 关节类型，与 {@link JointOptions} 的 `type` 相同 */
  type: JointOptions["type"];
  /** 刚体 A 在 {@link PhysicsSnapshot.bodies} 里的下标 */
  bodyA: number;
  /** 刚体 B 在 {@link PhysicsSnapshot.bodies} 里的下标 */
  bodyB: number;
  /** 关节点在刚体 A 上的局部坐标 */
  anchorA: Vector3;
  /** 关节点在刚体 B 上的局部坐标 */
  anchorB: Vector3;
  /** 关节坐标系在刚体 A 上的局部朝向，3×3 矩阵按行展开 */
  basisA: number[];
  /** 关节坐标系在刚体 B 上的局部朝向，3×3 矩阵按行展开 */
  basisB: number[];
  /** 两个刚体之间是否碰撞 */
  allowCollision: boolean;
  /** 断开所需的力，0 表示不会断开 */
  breakForce: number;
  /** 断开所需的力矩，0 表示不会断开 */
  breakTorque: number;
  /** 求解方式 */
  solver: "iterative" | "direct";
  /** 各个方向的弹簧 */
  springs: SpringState[];
  /** 各个平移方向的限制和马达 */
  linear: LimitState[];
  /** 各个转动方向的限制和马达 */
  angular: LimitState[];
  /** 布娃娃关节的两个最大摆动角，其他关节为空 */
  swing: number[];
}

/**
 * 可用 JSON 保存的快照。
 *
 * @remarks
 *
 * - `format` 是兼容性标识，`schema` 是存档格式版本。
 * - 不包含地形缓存、接触热启动缓存、回调或原版实体，不能逐位恢复求解器状态。
 */
export interface PhysicsSnapshot {
  /** 固定为 `"cutlass-physics"` */
  format: "cutlass-physics";
  /** 存档格式版本 */
  schema: 1;
  /** 保存时所在的维度 */
  dimensionId: string;
  /** 世界设置，对应 {@link PhysicsWorldOptions} 的同名选项 */
  settings: {
    /** 重力 */
    gravity: Vector3;
    /** 虚空海的参数，false 表示关闭。旧存档没有这一项时按默认值 */
    voidSea?: Required<VoidSeaOptions> | false;
    /** 流体参数，false 表示关闭。旧存档没有这一项时按默认值（开启） */
    fluids?: FluidOptions | false;
    /** 原版爆炸是否推动刚体。旧存档没有这一项时按默认值（开启） */
    explosions?: boolean;
    /** 是否破坏被撞碎的易碎方块。旧存档没有这一项时按默认值（开启） */
    breakFragileBlocks?: boolean;
    /** 每个 tick 的子步数 */
    substeps: number;
    /** null 表示 Infinity。 */
    timeBudgetMs: number | null;
    /** 推进方式 */
    stepMode: StepMode;
    /** 是否暂停 */
    paused: boolean;
    /** 创建世界时显式给出的地形选项（不含 collision 函数）；没有地形时为 null */
    terrain: Omit<TerrainOptions, "collision"> | null;
    /** 是否用了自定义方块解析：恢复时必须重新提供 terrain.collision */
    customCollision: boolean;
    /** 方块登记表（键都带命名空间）。没有给 blocks 时省略 */
    blocks?: BlockDefinitions;
  };
  /** 所有刚体 */
  bodies: BodySnapshot[];
  /** 所有关节 */
  joints: JointSnapshot[];
}

/** 拒绝循环、非有限数和实体等非普通对象，避免 JSON.stringify 静默丢失数据。
 * @internal */
export function copyJson(value: unknown, depth = 0, seen = new Set<object>()): JsonValue {
  if (depth > 64) throw new Error("存档数据嵌套超过 64 层");
  if (value === null || typeof value === "boolean" || typeof value === "string") return value;
  if (typeof value === "number") {
    if (Number.isFinite(value)) return value;
    throw new Error(`存档数据不能包含 ${value}（JSON 无法表示 NaN 和无穷大）`);
  }
  if (typeof value !== "object" || value === null || seen.has(value)) throw new Error("存档数据必须是无循环的 JSON 值");
  const proto = Object.getPrototypeOf(value);
  if (!Array.isArray(value) && proto !== Object.prototype && proto !== null) throw new Error("存档不接受实体或类实例，请转换为普通数据");
  seen.add(value);
  let result: JsonValue;
  if (Array.isArray(value)) result = Array.from(value, (v) => copyJson(v, depth + 1, seen));
  else result = Object.fromEntries(Object.entries(value).map(([k, v]) => [k, copyJson(v, depth + 1, seen)]));
  seen.delete(value);
  return result;
}

function shapeSnapshot(shape: Shape): ShapeOptions {
  const t = shape.localTransform, g = shape.geom;
  const common: ShapeBaseOptions = {
    offset: { x: t.px, y: t.py, z: t.pz },
    rotation: matrixToQuaternion(t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22),
    friction: shape.friction, restitution: shape.restitution, density: shape.density,
    collisionGroup: shape.collisionGroup, collisionMask: shape.collisionMask,
  };
  // 方块形状：方块不可变（创建刚体时复制过），原样存
  const block = (shape.userData as { block?: BodyBlock } | null)?.block;
  if (block !== undefined) common.block = block.states === undefined
    ? { location: { ...block.location }, typeId: block.typeId }
    : { location: { ...block.location }, typeId: block.typeId, states: { ...block.states } };
  if (g instanceof BoxGeometry) return { ...common, type: "box", size: { x: g.hx * 2, y: g.hy * 2, z: g.hz * 2 } };
  if (g instanceof SphereGeometry) return { ...common, type: "sphere", radius: g.radius };
  if (g instanceof CapsuleGeometry) return { ...common, type: "capsule", radius: g.radius, height: 2 * (g.halfHeight + g.radius) };
  if (g instanceof CylinderGeometry) return { ...common, type: "cylinder", radius: g.radius, height: 2 * g.halfHeight };
  if (g instanceof ConeGeometry) return { ...common, type: "cone", radius: g.radius, height: 2 * g.halfHeight };
  if (g instanceof ConvexHullGeometry) return { ...common, type: "convexHull", points: g.vertices.map((p) => ({ x: p.x, y: p.y, z: p.z })) };
  throw new Error("无法保存未知的形状类型");
}

/** @internal 存储器专用：未变的形状描述和编码可复用；公共快照不共享这些对象。 */
export class SnapshotShapeCache {
  private readonly entries = new WeakMap<Shape, { transform: Transform; value: ShapeOptions }>();
  private readonly texts = new WeakMap<ShapeOptions, string>();

  capture(shape: Shape): ShapeOptions {
    const entry = this.entries.get(shape), t = shape.localTransform, v = entry?.value, p = entry?.transform;
    if (v && p && p.px === t.px && p.py === t.py && p.pz === t.pz &&
        p.r00 === t.r00 && p.r01 === t.r01 && p.r02 === t.r02 && p.r10 === t.r10 && p.r11 === t.r11 && p.r12 === t.r12 && p.r20 === t.r20 && p.r21 === t.r21 && p.r22 === t.r22 &&
        v.friction === shape.friction && v.restitution === shape.restitution && v.density === shape.density && v.collisionGroup === shape.collisionGroup && v.collisionMask === shape.collisionMask) {
      // 凸包顶点按约定不可变，仍检查逐项数值，避免内部调用误改后保存旧结构。
      const g = shape.geom;
      if (!(g instanceof ConvexHullGeometry) || (v.type === "convexHull" && v.points.length === g.vertices.length && g.vertices.every((a, i) => a.x === v.points[i].x && a.y === v.points[i].y && a.z === v.points[i].z))) return v;
    }
    const value = shapeSnapshot(shape), transform = new Transform();
    shapeOptions(value);
    transform.copyFrom(t);
    this.entries.set(shape, { transform, value });
    this.texts.set(value, JSON.stringify(value));
    return value;
  }

  text(value: ShapeOptions): string { return this.texts.get(value) ?? JSON.stringify(value); }
}

/** @internal */
export function captureBody(body: Body, options: SnapshotOptions, cache?: SnapshotShapeCache): BodySnapshot {
  const c = body.core;
  const shapes: ShapeOptions[] = [];
  if (body.snapshotShapes) {
    for (const s of body.snapshotShapes) shapes.push(cache ? cache.capture(s) : shapeSnapshot(s));
  } else {
    for (let s = c.shapeList; s !== null; s = s.next) shapes.push(cache ? cache.capture(s) : shapeSnapshot(s));
  }
  const out: BodySnapshot = {
    options: {
      type: body.type, shapes, location: body.location, rotation: body.rotation,
      linearVelocity: body.linearVelocity, angularVelocity: body.angularVelocity,
      linearDamping: body.linearDamping, angularDamping: body.angularDamping, gravityScale: body.gravityScale, allowSleep: c.autoSleep,
    },
    // isSleeping 不算等地形：等地形的刚体恢复后照常运动（速度一直留着）
    sleeping: body.isSleeping,
    force: { x: c.fx, y: c.fy, z: c.fz }, torque: { x: c.tx, y: c.ty, z: c.tz },
  };
  if (options.encodeUserData) out.data = copyJson(options.encodeUserData(body));
  return out;
}

function parts(c: CoreJoint): { springs: SpringDamper[]; linear: TranslationalLimitMotor[]; angular: RotationalLimitMotor[] } {
  if (c instanceof SphericalJoint) return { springs: [c.sd], linear: [], angular: [] };
  if (c instanceof RevoluteJoint) return { springs: [c.sd], linear: [], angular: [c.lm] };
  if (c instanceof PrismaticJoint) return { springs: [c.sd], linear: [c.lm], angular: [] };
  if (c instanceof CylindricalJoint) return { springs: [c.translSd, c.rotSd], linear: [c.translLm], angular: [c.rotLm] };
  if (c instanceof UniversalJoint) return { springs: [c.sd1, c.sd2], linear: [], angular: [c.lm1, c.lm2] };
  if (c instanceof RagdollJoint) return { springs: [c.twistSd, c.swingSd], linear: [], angular: [c.twistLm] };
  if (c instanceof GenericJoint) return { springs: [...c.translSds, ...c.rotSds], linear: c.translLms, angular: c.rotLms };
  throw new Error("无法保存未知的关节类型");
}

/** @internal */
export function captureJoint(j: Joint, ids: Map<Body, number>): JointSnapshot {
  const c = j.core, p = parts(c);
  return {
    type: j.type, bodyA: ids.get(j.bodyA)!, bodyB: ids.get(j.bodyB)!,
    anchorA: { x: c.la1X, y: c.la1Y, z: c.la1Z }, anchorB: { x: c.la2X, y: c.la2Y, z: c.la2Z },
    basisA: [c.lbX1X, c.lbX1Y, c.lbX1Z, c.lbY1X, c.lbY1Y, c.lbY1Z, c.lbZ1X, c.lbZ1Y, c.lbZ1Z],
    basisB: [c.lbX2X, c.lbX2Y, c.lbX2Z, c.lbY2X, c.lbY2Y, c.lbY2Z, c.lbZ2X, c.lbZ2Y, c.lbZ2Z],
    allowCollision: c.allowCollision, breakForce: c.breakForce, breakTorque: c.breakTorque,
    solver: c.solver instanceof DirectJointConstraintSolver ? "direct" : "iterative",
    springs: p.springs.map((s) => ({ frequency: s.frequency, dampingRatio: s.dampingRatio, useSymplecticEuler: s.useSymplecticEuler })),
    linear: p.linear.map((l) => ({ lower: l.lowerLimit, upper: l.upperLimit, speed: l.motorSpeed, maximum: l.motorForce })),
    angular: p.angular.map((l) => ({ lower: l.lowerLimit, upper: l.upperLimit, speed: l.motorSpeed, maximum: l.motorTorque })),
    swing: c instanceof RagdollJoint ? [c.maxSwingAngle1, c.maxSwingAngle2] : [],
  };
}

/** 已经过格式校验的记录 → 新世界内的对象。调用方负责出错时销毁新世界。
 * @internal */
export function restoreContents(w: PhysicsWorld, s: PhysicsSnapshot, o: RestoreOptions, hasTerrain: boolean): void {
  const bodies = s.bodies.map((b, i) => w.createBody({ ...b.options, userData: b.data === undefined ? undefined : o.decodeUserData ? o.decodeUserData(b.data, i) : b.data }));
  const x = { x: 1, y: 0, z: 0 }, y = { x: 0, y: 1, z: 0 };
  for (const j of s.joints) {
    const common = { bodyA: bodies[j.bodyA], bodyB: bodies[j.bodyB], anchor: { x: 0, y: 0, z: 0 }, allowCollision: j.allowCollision, breakForce: j.breakForce, breakTorque: j.breakTorque, solver: j.solver };
    let options: JointOptions;
    switch (j.type) {
      case "ball": case "generic": options = { ...common, type: j.type }; break;
      case "hinge": case "slider": case "cylinder": options = { ...common, type: j.type, axis: x }; break;
      case "universal": options = { ...common, type: j.type, axisA: x, axisB: y }; break;
      case "ragdoll": options = { ...common, type: j.type, twistAxis: x, swingAxis: y }; break;
    }
    // 不走 createJoint：存档里可能有两端都已改成静态的关节（运行时合法）
    const c = w.addJoint(options).core, p = parts(c);
    c.la1X = j.anchorA.x; c.la1Y = j.anchorA.y; c.la1Z = j.anchorA.z;
    c.la2X = j.anchorB.x; c.la2Y = j.anchorB.y; c.la2Z = j.anchorB.z;
    [c.lbX1X, c.lbX1Y, c.lbX1Z, c.lbY1X, c.lbY1Y, c.lbY1Z, c.lbZ1X, c.lbZ1Y, c.lbZ1Z] = j.basisA;
    [c.lbX2X, c.lbX2Y, c.lbX2Z, c.lbY2X, c.lbY2Y, c.lbY2Z, c.lbZ2X, c.lbZ2Y, c.lbZ2Z] = j.basisB;
    p.springs.forEach((v, i) => {
      const spring = j.springs[i];
      v.setSpring(spring.frequency, spring.dampingRatio).setSymplecticEuler(spring.useSymplecticEuler);
    });
    p.linear.forEach((v, i) => { const l = j.linear[i]; v.setLimits(l.lower, l.upper).setMotor(l.speed, l.maximum); });
    p.angular.forEach((v, i) => { const l = j.angular[i]; v.setLimits(l.lower, l.upper).setMotor(l.speed, l.maximum); });
    if (c instanceof RagdollJoint) [c.maxSwingAngle1, c.maxSwingAngle2] = j.swing;
    c.syncAnchors();
  }
  bodies.forEach((b, i) => {
    const state = s.bodies[i], c = b.core;
    c.fx = state.force.x; c.fy = state.force.y; c.fz = state.force.z;
    c.tx = state.torque.x; c.ty = state.torque.y; c.tz = state.torque.z;
    // 重新读取地形后必须重新确认支撑：离线期间脚下的方块可能已被移除。
    if (state.sleeping && !hasTerrain) b.sleep();
  });
}

function requireValue(ok: boolean, name: string): asserts ok {
  if (!ok) throw new Error(`物理存档的 ${name} 不合法`);
}
function number(v: unknown, name: string, min = -Infinity, integer = false): asserts v is number {
  requireValue(typeof v === "number" && Number.isFinite(v) && v >= min && (!integer || Number.isSafeInteger(v)), name);
}
function boolean(v: unknown, name: string): void { requireValue(typeof v === "boolean", name); }
function vector(v: Vector3, name: string): void {
  requireValue(v !== null && typeof v === "object", name);
  number(v.x, name); number(v.y, name); number(v.z, name);
}
function quaternion(v: Quaternion): void {
  vector(v, "rotation"); number(v.w, "rotation.w");
  const length = Math.hypot(v.x, v.y, v.z, v.w);
  requireValue(length > 0 && Number.isFinite(length), "rotation");
}
function positive(v: unknown, name: string): void {
  number(v, name, 0); requireValue(v > 0, name);
}
/** 校验存档中的几何描述，不为校验而分配 Shape、Transform 和 Aabb。 */
function shapeOptions(shape: ShapeOptions): void {
  requireValue(shape !== null && typeof shape === "object", "shape");
  if (shape.offset !== undefined) vector(shape.offset, "offset");
  if (shape.rotation !== undefined) quaternion(shape.rotation);
  for (const k of ["friction", "restitution"] as const) if (shape[k] !== undefined) number(shape[k], k, 0);
  if (shape.density !== undefined) positive(shape.density, "density");
  for (const k of ["collisionGroup", "collisionMask"] as const) if (shape[k] !== undefined) { number(shape[k], k, -2147483648, true); requireValue(shape[k]! <= 4294967295, k); }
  if (shape.block !== undefined) {
    const b = shape.block;
    requireValue(b !== null && typeof b === "object" && typeof b.typeId === "string", "block");
    vector(b.location, "block.location");
    requireValue(Number.isInteger(b.location.x) && Number.isInteger(b.location.y) && Number.isInteger(b.location.z), "block.location");
    if (b.states !== undefined) {
      requireValue(b.states !== null && typeof b.states === "object" && !Array.isArray(b.states), "block.states");
      for (const v of Object.values(b.states)) requireValue(typeof v === "boolean" || typeof v === "string" || (typeof v === "number" && Number.isFinite(v)), "block.states");
    }
  }
  switch (shape.type) {
    case "box": vector(shape.size, "size"); positive(shape.size.x, "size.x"); positive(shape.size.y, "size.y"); positive(shape.size.z, "size.z"); break;
    case "sphere": positive(shape.radius, "radius"); break;
    case "capsule": positive(shape.radius, "radius"); positive(shape.height, "height"); requireValue(shape.height >= shape.radius * 2, "capsule.height"); break;
    case "cylinder": case "cone": positive(shape.radius, "radius"); positive(shape.height, "height"); break;
    case "convexHull": array(shape.points, "points"); requireValue(shape.points.length >= 4, "points"); shape.points.forEach((p) => vector(p, "point")); break;
    default: throw new Error("物理存档包含未知形状");
  }
}
function array(v: unknown, name: string, length?: number): asserts v is unknown[] {
  requireValue(Array.isArray(v) && (length === undefined || v.length === length), name);
}
function basis(v: number[], name: string, ball: boolean): void {
  array(v, name, 9);
  v.forEach((n) => number(n, name));
  if (ball) return;
  for (let a = 0; a < 3; a++) for (let b = a; b < 3; b++) {
    const dot = v[a * 3] * v[b * 3] + v[a * 3 + 1] * v[b * 3 + 1] + v[a * 3 + 2] * v[b * 3 + 2];
    requireValue(Math.abs(dot - (a === b ? 1 : 0)) < 1e-5, name);
  }
  const det = v[0] * (v[4] * v[8] - v[5] * v[7]) - v[1] * (v[3] * v[8] - v[5] * v[6]) + v[2] * (v[3] * v[7] - v[4] * v[6]);
  requireValue(det > 0, name);
}

const FLUID_FIELDS = ["waterDensity", "lavaDensity", "drag", "linearDrag", "lavaViscosity", "waveDamping", "airPockets", "floodRate", "currentSpeed"];

/** 校验并复制存档。未知格式或版本会报错，不能当作空场景覆盖。 */
export function readSnapshot(input: unknown): PhysicsSnapshot {
  return validateSnapshot(copyJson(input) as unknown as PhysicsSnapshot);
}

/** 完整校验外部来的存档（读档、write 传入的快照）。本库刚捕获的快照用 checkCapturedBodies */
function validateSnapshot(s: PhysicsSnapshot): PhysicsSnapshot {
  requireValue(s !== null && s.format === "cutlass-physics" && s.schema === 1, "格式或版本");
  requireValue(typeof s.dimensionId === "string" && s.dimensionId.length > 0, "维度");
  const t = s.settings;
  requireValue(t !== null && typeof t === "object", "settings");
  vector(t.gravity, "gravity"); number(t.substeps, "substeps", 1, true);
  if (t.voidSea !== undefined && t.voidSea !== false) {
    requireValue(t.voidSea !== null && typeof t.voidSea === "object", "voidSea");
    number(t.voidSea.startY, "voidSea.startY"); number(t.voidSea.depthGradient, "voidSea.depthGradient"); number(t.voidSea.drag, "voidSea.drag", 0);
  }
  if (t.fluids !== undefined && t.fluids !== false) {
    requireValue(t.fluids !== null && typeof t.fluids === "object", "fluids");
    const f = t.fluids;
    for (const [k, v] of Object.entries(f)) {
      if (!FLUID_FIELDS.includes(k)) throw new Error(`物理存档包含未知流体选项 ${k}`);
      if (k === "airPockets") boolean(v, "fluids.airPockets");
      else number(v, `fluids.${k}`, 0);
    }
  }
  if (t.explosions !== undefined) boolean(t.explosions, "explosions");
  if (t.breakFragileBlocks !== undefined) boolean(t.breakFragileBlocks, "breakFragileBlocks");
  if (t.timeBudgetMs !== null) { number(t.timeBudgetMs, "timeBudgetMs", 0); requireValue(t.timeBudgetMs > 0, "timeBudgetMs"); }
  requireValue(t.stepMode === "auto" || t.stepMode === "manual", "stepMode");
  boolean(t.paused, "paused"); boolean(t.customCollision, "customCollision");
  if (t.terrain !== null) {
    requireValue(typeof t.terrain === "object", "terrain");
    for (const [k, v] of Object.entries(t.terrain)) {
      const check = Object.prototype.hasOwnProperty.call(TERRAIN_FIELDS, k) ? TERRAIN_FIELDS[k as keyof typeof TERRAIN_FIELDS] : undefined;
      if (check === undefined) throw new Error(`物理存档包含未知地形选项 ${k}`);
      check(v, k);
    }
  } else requireValue(!t.customCollision, "customCollision");
  if (t.blocks !== undefined) {
    // 登记表的校验和创建世界时相同；报错时说明是存档的问题
    try {
      new BlockRegistry(t.blocks);
    } catch (error) {
      const wrapped = new Error(`物理存档的 blocks 不合法：${(error as Error).message}`) as Error & { cause?: unknown };
      wrapped.cause = error;
      throw wrapped;
    }
  }
  array(s.bodies, "bodies"); array(s.joints, "joints");
  for (const b of s.bodies) {
    requireValue(b !== null && typeof b === "object" && b.options !== null && typeof b.options === "object", "body");
    const o = b.options;
    requireValue(o.type === "dynamic" || o.type === "static" || o.type === "kinematic", "body.type");
    vector(o.location, "location"); quaternion(o.rotation);
    vector(o.linearVelocity, "linearVelocity"); vector(o.angularVelocity, "angularVelocity");
    number(o.linearDamping, "linearDamping", 0); number(o.angularDamping, "angularDamping", 0); number(o.gravityScale, "gravityScale");
    boolean(o.allowSleep, "allowSleep"); boolean(b.sleeping, "sleeping"); vector(b.force, "force"); vector(b.torque, "torque");
    array(o.shapes, "shapes"); requireValue(o.shapes.length > 0, "shapes");
    for (const shape of o.shapes) shapeOptions(shape);
  }
  const counts = { ball: [1, 0, 0, 0], hinge: [1, 0, 1, 0], slider: [1, 1, 0, 0], cylinder: [2, 1, 1, 0], universal: [2, 0, 2, 0], ragdoll: [2, 0, 1, 2], generic: [6, 3, 3, 0] };
  for (const j of s.joints) {
    requireValue(j !== null && typeof j === "object" && Object.prototype.hasOwnProperty.call(counts, j.type), "joint.type");
    number(j.bodyA, "bodyA", 0, true); number(j.bodyB, "bodyB", 0, true);
    requireValue(j.bodyA < s.bodies.length && j.bodyB < s.bodies.length && j.bodyA !== j.bodyB, "关节刚体引用");
    // 运行时允许关节两端均改为非动态刚体，存档保留该状态。
    vector(j.anchorA, "anchorA"); vector(j.anchorB, "anchorB"); basis(j.basisA, "basisA", j.type === "ball"); basis(j.basisB, "basisB", j.type === "ball");
    // 与运行时一致，仅拒绝非有限数；负断开阈值和马达力按 0 处理。
    boolean(j.allowCollision, "allowCollision"); number(j.breakForce, "breakForce"); number(j.breakTorque, "breakTorque");
    requireValue(j.solver === "iterative" || j.solver === "direct", "solver");
    const n = counts[j.type]; array(j.springs, "springs", n[0]); array(j.linear, "linear", n[1]); array(j.angular, "angular", n[2]); array(j.swing, "swing", n[3]);
    for (const p of j.springs) { requireValue(p !== null && typeof p === "object", "spring"); number(p.frequency, "frequency"); number(p.dampingRatio, "dampingRatio"); boolean(p.useSymplecticEuler, "useSymplecticEuler"); }
    for (const l of [...j.linear, ...j.angular]) { requireValue(l !== null && typeof l === "object", "limit"); number(l.lower, "lower"); number(l.upper, "upper"); number(l.speed, "speed"); number(l.maximum, "maximum"); }
    j.swing.forEach((v) => number(v, "swing"));
  }
  return s;
}

/** 地形可序列化字段的校验规则，类型约束覆盖全部可存字段。 */
const TERRAIN_FIELDS: { readonly [K in keyof Omit<TerrainOptions, "collision">]-?: (v: unknown, name: string) => void } = {
  margin: (v, k) => number(v, k, 0),
  maxChunksPerTick: (v, k) => number(v, k, 1, true),
  unloadDelayTicks: (v, k) => number(v, k, 0, true),
  revalidateBlocksPerTick: (v, k) => number(v, k, 0, true),
  preloadTicks: (v, k) => number(v, k, 0, true),
  watchBlockEvents: (v, k) => boolean(v, k),
  unknownBlock: (v, k) => requireValue(v === "fullCube" || v === "none", k),
  unloadedChunkPolicy: (v, k) => requireValue(v === "freeze" || v === "fall", k),
  material: (v, k) => {
    requireValue(v !== null && typeof v === "object", k);
    const m = v as { friction: unknown; restitution: unknown };
    number(m.friction, k, 0); number(m.restitution, k, 0);
  },
};

/**
 * 校验捕获状态中的非有限数。形状、关节和世界配置已在构造或缓存时校验。
 * @internal
 */
export function checkCapturedBodies(s: PhysicsSnapshot): PhysicsSnapshot {
  for (const b of s.bodies) {
    const o = b.options, l = o.location, r = o.rotation, v = o.linearVelocity, w = o.angularVelocity, f = b.force, t = b.torque;
    if (!(Number.isFinite(l.x + l.y + l.z) && Number.isFinite(r.x + r.y + r.z + r.w) && Number.isFinite(v.x + v.y + v.z) &&
          Number.isFinite(w.x + w.y + w.z) && Number.isFinite(f.x + f.y + f.z) && Number.isFinite(t.x + t.y + t.z))) {
      throw new Error("物理模拟出现了 NaN 或无穷大（模拟发散），拒绝保存，已有的存档保持不变");
    }
  }
  return s;
}
