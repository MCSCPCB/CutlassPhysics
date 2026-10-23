// 将公开的形状和关节选项转换为内核配置。
import { BoxGeometry } from "../core/collision/geometry/box-geometry.js";
import { CapsuleGeometry } from "../core/collision/geometry/capsule-geometry.js";
import { ConeGeometry } from "../core/collision/geometry/cone-geometry.js";
import { ConvexHullGeometry } from "../core/collision/geometry/convex-hull-geometry.js";
import { CylinderGeometry } from "../core/collision/geometry/cylinder-geometry.js";
import { ConvexGeometry, type Geometry } from "../core/collision/geometry/geometry.js";
import { SphereGeometry } from "../core/collision/geometry/sphere-geometry.js";
import { Mat3 } from "../core/common/mat3.js";
import { Vec3 } from "../core/common/vec3.js";
import { ConstraintSolverType } from "../core/dynamics/constraint/joint/joint-config.js";
import type { JointConfig } from "../core/dynamics/constraint/joint/joint-config.js";
import type { Joint as CoreJoint } from "../core/dynamics/constraint/joint/joint.js";
import { CylindricalJoint, CylindricalJointConfig } from "../core/dynamics/constraint/joint/cylindrical-joint.js";
import { GenericJoint, GenericJointConfig } from "../core/dynamics/constraint/joint/generic-joint.js";
import { PrismaticJoint, PrismaticJointConfig } from "../core/dynamics/constraint/joint/prismatic-joint.js";
import { RagdollJoint, RagdollJointConfig } from "../core/dynamics/constraint/joint/ragdoll-joint.js";
import { RevoluteJoint, RevoluteJointConfig } from "../core/dynamics/constraint/joint/revolute-joint.js";
import type { RotationalLimitMotor } from "../core/dynamics/constraint/joint/rotational-limit-motor.js";
import type { SpringDamper } from "../core/dynamics/constraint/joint/spring-damper.js";
import { SphericalJoint, SphericalJointConfig } from "../core/dynamics/constraint/joint/spherical-joint.js";
import type { TranslationalLimitMotor } from "../core/dynamics/constraint/joint/translational-limit-motor.js";
import { UniversalJoint, UniversalJointConfig } from "../core/dynamics/constraint/joint/universal-joint.js";
import { ShapeConfig } from "../core/dynamics/rigidbody/shape-config.js";
import { Shape } from "../core/dynamics/rigidbody/shape.js";
import type { RigidBody } from "../core/dynamics/rigidbody/rigid-body.js";
import type { AngularLimitOptions, JointOptions, LinearLimitOptions, SpringOptions } from "./joints.js";
import type { Quaternion, Vector3 } from "./math.js";
import type { ShapeOptions } from "./shapes.js";

const DEG = Math.PI / 180;

/** 四元数 → 旋转矩阵（按行：r00 r01 r02 r10 ...）。输入会先归一化 */
export function quaternionToMatrix(q: Quaternion, out: Float64Array | number[] = new Array<number>(9)): Float64Array | number[] {
  const len = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(len > 0)) throw new Error("四元数的所有分量均为零");
  const x = q.x / len, y = q.y / len, z = q.z / len, w = q.w / len;
  const x2 = 2 * x, y2 = 2 * y, z2 = 2 * z;
  const xx = x * x2, yy = y * y2, zz = z * z2, xy = x * y2, yz = y * z2, xz = x * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  out[0] = 1 - yy - zz; out[1] = xy - wz; out[2] = xz + wy;
  out[3] = xy + wz; out[4] = 1 - xx - zz; out[5] = yz - wx;
  out[6] = xz - wy; out[7] = yz + wx; out[8] = 1 - xx - yy;
  return out;
}

/** 旋转矩阵 → 四元数（已归一化），与 core 里 RigidBody.applyRotation 的转换一致 */
export function matrixToQuaternion(
  r00: number, r01: number, r02: number, r10: number, r11: number, r12: number, r20: number, r21: number, r22: number,
): Quaternion {
  const t = r00 + r11 + r22;
  let x: number, y: number, z: number, w: number, s: number;
  if (t > 0) {
    s = Math.sqrt(t + 1);
    w = 0.5 * s;
    s = 0.5 / s;
    x = (r21 - r12) * s; y = (r02 - r20) * s; z = (r10 - r01) * s;
  } else if (r00 > r11) {
    if (r00 > r22) {
      s = Math.sqrt(r00 - r11 - r22 + 1);
      x = 0.5 * s;
      s = 0.5 / s;
      y = (r01 + r10) * s; z = (r02 + r20) * s; w = (r21 - r12) * s;
    } else {
      s = Math.sqrt(r22 - r00 - r11 + 1);
      z = 0.5 * s;
      s = 0.5 / s;
      x = (r02 + r20) * s; y = (r12 + r21) * s; w = (r10 - r01) * s;
    }
  } else if (r11 > r22) {
    s = Math.sqrt(r11 - r22 - r00 + 1);
    y = 0.5 * s;
    s = 0.5 / s;
    x = (r01 + r10) * s; z = (r12 + r21) * s; w = (r02 - r20) * s;
  } else {
    s = Math.sqrt(r22 - r00 - r11 + 1);
    z = 0.5 * s;
    s = 0.5 / s;
    x = (r02 + r20) * s; y = (r12 + r21) * s; w = (r10 - r01) * s;
  }
  const l = Math.sqrt(x * x + y * y + z * z + w * w);
  return { x: x / l + 0, y: y / l + 0, z: z / l + 0, w: w / l + 0 };
}

function positive(name: string, v: number): number {
  if (!(v > 0) || !Number.isFinite(v)) throw new Error(`${name} 必须是大于零的有限数，收到 ${v}`);
  return v;
}

function finite(name: string, v: Vector3): void {
  if (!Number.isFinite(v.x) || !Number.isFinite(v.y) || !Number.isFinite(v.z)) {
    throw new Error(`${name} 必须是有限的数，收到 (${v.x}, ${v.y}, ${v.z})`);
  }
}

/** 形状选项 → 内核几何体 */
export function makeGeometry(o: ShapeOptions): Geometry {
  switch (o.type) {
    case "box": {
      finite("box 的 size", o.size);
      positive("box 的 size.x", o.size.x); positive("box 的 size.y", o.size.y); positive("box 的 size.z", o.size.z);
      return new BoxGeometry(o.size.x / 2, o.size.y / 2, o.size.z / 2);
    }
    case "sphere":
      return new SphereGeometry(positive("sphere 的 radius", o.radius));
    case "capsule": {
      positive("capsule 的 radius", o.radius);
      if (!(o.height >= o.radius * 2)) throw new Error(`capsule 的 height 不得小于 radius * 2，收到 height = ${o.height}，radius = ${o.radius}`);
      return new CapsuleGeometry(o.radius, o.height / 2 - o.radius);
    }
    case "cylinder":
      return new CylinderGeometry(positive("cylinder 的 radius", o.radius), positive("cylinder 的 height", o.height) / 2);
    case "cone":
      return new ConeGeometry(positive("cone 的 radius", o.radius), positive("cone 的 height", o.height) / 2);
    case "convexHull": {
      if (o.points.length < 4) throw new Error(`convexHull 至少需要 4 个点，收到 ${o.points.length} 个`);
      const vertices: Vec3[] = [];
      for (const p of o.points) {
        finite("convexHull 的点", p);
        vertices.push(new Vec3(p.x, p.y, p.z));
      }
      return new ConvexHullGeometry(vertices);
    }
    default:
      throw new Error(`未知的形状类型：${(o as { type: string }).type}`);
  }
}

/** 形状选项的默认值（见 shapes.ts） */
const SHAPE_DEFAULTS = { friction: 0.5, restitution: 0, density: 1, collisionGroup: 1, collisionMask: 1 };

/** 形状选项 → 内核形状 */
export function makeShape(o: ShapeOptions): Shape {
  const sc = new ShapeConfig(makeGeometry(o));
  const offset = o.offset;
  if (offset !== undefined) {
    finite("形状的 offset", offset);
    sc.transform.px = offset.x; sc.transform.py = offset.y; sc.transform.pz = offset.z;
  }
  if (o.rotation !== undefined) {
    const m = quaternionToMatrix(o.rotation);
    const t = sc.transform;
    t.r00 = m[0]; t.r01 = m[1]; t.r02 = m[2]; t.r10 = m[3]; t.r11 = m[4]; t.r12 = m[5]; t.r20 = m[6]; t.r21 = m[7]; t.r22 = m[8];
  }
  sc.friction = o.friction ?? SHAPE_DEFAULTS.friction;
  sc.restitution = o.restitution ?? SHAPE_DEFAULTS.restitution;
  sc.density = positive("density", o.density ?? SHAPE_DEFAULTS.density);
  sc.collisionGroup = o.collisionGroup ?? SHAPE_DEFAULTS.collisionGroup;
  sc.collisionMask = o.collisionMask ?? SHAPE_DEFAULTS.collisionMask;
  return new Shape(sc);
}

/** 是不是凸几何体（扫掠用） */
export function isConvex(g: Geometry): g is ConvexGeometry {
  return g instanceof ConvexGeometry;
}

// ---------------------------------------------------------------------------------------------------------------------
// 关节

/** 规范化关节参数：NaN 抛错，±Infinity 替换为可序列化的 ±bound。 */
function jointNumber(name: string, v: number, bound: number): number {
  if (Number.isNaN(v)) throw new Error(`${name} 不能是 NaN`);
  return v === Infinity ? bound : v === -Infinity ? -bound : v;
}

/** 断开阈值：0 表示不会断开，Infinity 和负数都按 0 处理 */
function breakLimit(name: string, v: number | undefined): number {
  if (v === undefined) return 0;
  if (Number.isNaN(v)) throw new Error(`${name} 不能是 NaN`);
  return v > 0 && v !== Infinity ? v : 0;
}

/** 无穷大马达力或扭矩的有限替代值。 */
const UNLIMITED_MOTOR = 1e12;

function applySpring(sd: SpringDamper, s: SpringOptions | undefined): void {
  if (s === undefined) return;
  if (!Number.isFinite(s.frequency)) throw new Error(`spring 的 frequency 必须是有限的数，收到 ${s.frequency}`);
  const damping = s.dampingRatio ?? 1;
  if (!Number.isFinite(damping)) throw new Error(`spring 的 dampingRatio 必须是有限的数，收到 ${damping}`);
  sd.setSpring(s.frequency, damping);
  sd.setSymplecticEuler(s.useSymplecticEuler ?? false);
}

/** 单侧角度限制的另一侧默认为 ±180 度，与内核转角范围一致。 */
function applyAngular(lm: RotationalLimitMotor, sd: SpringDamper, o: AngularLimitOptions | undefined): void {
  if (o === undefined) return;
  // 对外用度，内核用弧度。两侧都没给就不限制（内核的默认值 lower > upper）
  if (o.lower !== undefined || o.upper !== undefined) {
    lm.setLimits(jointNumber("limit 的 lower", o.lower ?? -180, 180) * DEG, jointNumber("limit 的 upper", o.upper ?? 180, 180) * DEG);
  }
  if (o.motorSpeed !== undefined || o.motorMaxTorque !== undefined) {
    if (!Number.isFinite(o.motorSpeed ?? 0)) throw new Error(`motorSpeed 必须是有限的数，收到 ${o.motorSpeed}`);
    lm.setMotor((o.motorSpeed ?? 0) * DEG, jointNumber("motorMaxTorque", o.motorMaxTorque ?? 0, UNLIMITED_MOTOR));
  }
  applySpring(sd, o.spring);
}

/** 单侧位移限制的另一侧默认为 ±1e9 格。 */
function applyLinear(lm: TranslationalLimitMotor, sd: SpringDamper, o: LinearLimitOptions | undefined): void {
  if (o === undefined) return;
  if (o.lower !== undefined || o.upper !== undefined) {
    lm.setLimits(jointNumber("limit 的 lower", o.lower ?? -1e9, 1e9), jointNumber("limit 的 upper", o.upper ?? 1e9, 1e9));
  }
  if (o.motorSpeed !== undefined || o.motorMaxForce !== undefined) {
    if (!Number.isFinite(o.motorSpeed ?? 0)) throw new Error(`motorSpeed 必须是有限的数，收到 ${o.motorSpeed}`);
    lm.setMotor(o.motorSpeed ?? 0, jointNumber("motorMaxForce", o.motorMaxForce ?? 0, UNLIMITED_MOTOR));
  }
  applySpring(sd, o.spring);
}

function axisOf(name: string, v: Vector3): Vector3 {
  finite(name, v);
  if (v.x === 0 && v.y === 0 && v.z === 0) throw new Error(`${name} 不能是零向量`);
  return v;
}

/**
 * 关节选项 → 内核关节配置。bodyA、bodyB 是内核刚体。
 * 配置里的锚点和轴会按两个刚体当前的变换换算成各自的局部坐标。
 */
export function makeJointConfig(o: JointOptions, a: RigidBody, b: RigidBody): JointConfig {
  finite("关节的 anchor", o.anchor);
  const { x, y, z } = o.anchor;
  let config: JointConfig;
  switch (o.type) {
    case "ball": {
      const c = new SphericalJointConfig().init(a, b, x, y, z);
      applySpring(c.springDamper, o.spring);
      config = c;
      break;
    }
    case "hinge": {
      const ax = axisOf("hinge 的 axis", o.axis);
      const c = new RevoluteJointConfig().init(a, b, x, y, z, ax.x, ax.y, ax.z);
      applyAngular(c.limitMotor, c.springDamper, o.limit);
      config = c;
      break;
    }
    case "slider": {
      const ax = axisOf("slider 的 axis", o.axis);
      const c = new PrismaticJointConfig().init(a, b, x, y, z, ax.x, ax.y, ax.z);
      applyLinear(c.limitMotor, c.springDamper, o.limit);
      config = c;
      break;
    }
    case "cylinder": {
      const ax = axisOf("cylinder 的 axis", o.axis);
      const c = new CylindricalJointConfig().init(a, b, x, y, z, ax.x, ax.y, ax.z);
      applyLinear(c.translationalLimitMotor, c.translationalSpringDamper, o.linearLimit);
      applyAngular(c.rotationalLimitMotor, c.rotationalSpringDamper, o.angularLimit);
      config = c;
      break;
    }
    case "universal": {
      const a1 = axisOf("universal 的 axisA", o.axisA), a2 = axisOf("universal 的 axisB", o.axisB);
      const c = new UniversalJointConfig().init(a, b, x, y, z, a1.x, a1.y, a1.z, a2.x, a2.y, a2.z);
      applyAngular(c.limitMotor1, c.springDamper1, o.limitA);
      applyAngular(c.limitMotor2, c.springDamper2, o.limitB);
      config = c;
      break;
    }
    case "ragdoll": {
      const t = axisOf("ragdoll 的 twistAxis", o.twistAxis), s = axisOf("ragdoll 的 swingAxis", o.swingAxis);
      const c = new RagdollJointConfig().init(a, b, x, y, z, t.x, t.y, t.z, s.x, s.y, s.z);
      if (o.maxSwingAngleX !== undefined) c.maxSwingAngle1 = jointNumber("maxSwingAngleX", o.maxSwingAngleX, 180) * DEG;
      if (o.maxSwingAngleZ !== undefined) c.maxSwingAngle2 = jointNumber("maxSwingAngleZ", o.maxSwingAngleZ, 180) * DEG;
      applyAngular(c.twistLimitMotor, c.twistSpringDamper, o.twistLimit);
      applySpring(c.swingSpringDamper, o.swingSpring);
      config = c;
      break;
    }
    case "generic": {
      const c = new GenericJointConfig().init(a, b, x, y, z, new Mat3(), new Mat3());
      if (o.linearLimits !== undefined) {
        if (o.linearLimits.length !== 3) throw new Error("generic 的 linearLimits 长度必须为 3");
        for (let i = 0; i < 3; i++) applyLinear(c.translationalLimitMotors[i], c.translationalSpringDampers[i], o.linearLimits[i]);
      }
      if (o.angularLimits !== undefined) {
        if (o.angularLimits.length !== 3) throw new Error("generic 的 angularLimits 长度必须为 3");
        for (let i = 0; i < 3; i++) applyAngular(c.rotationalLimitMotors[i], c.rotationalSpringDampers[i], o.angularLimits[i]);
      }
      config = c;
      break;
    }
    default:
      throw new Error(`未知的关节类型：${(o as { type: string }).type}`);
  }
  config.allowCollision = o.allowCollision ?? false;
  config.breakForce = breakLimit("breakForce", o.breakForce);
  config.breakTorque = breakLimit("breakTorque", o.breakTorque);
  config.solverType = o.solver === "direct" ? ConstraintSolverType.DIRECT : ConstraintSolverType.ITERATIVE;
  return config;
}

/** 将公开关节选项转换为尚未加入世界的内核关节。 */
export function makeJoint(o: JointOptions, a: RigidBody, b: RigidBody): CoreJoint {
  const config = makeJointConfig(o, a, b);
  if (config instanceof SphericalJointConfig) return new SphericalJoint(config);
  if (config instanceof RevoluteJointConfig) return new RevoluteJoint(config);
  if (config instanceof PrismaticJointConfig) return new PrismaticJoint(config);
  if (config instanceof CylindricalJointConfig) return new CylindricalJoint(config);
  if (config instanceof UniversalJointConfig) return new UniversalJoint(config);
  if (config instanceof RagdollJointConfig) return new RagdollJoint(config);
  if (config instanceof GenericJointConfig) return new GenericJoint(config);
  throw new Error("未知的关节配置");
}
