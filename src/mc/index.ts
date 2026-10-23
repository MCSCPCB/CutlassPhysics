// Minecraft 绑定层与公开 API 导出。
// 通过 @minecraft/server 接入游戏运行时。
export { eulerToQuaternion, quaternionToEuler } from "./math.js";
export type { EulerDegrees, Quaternion, Vector3 } from "./math.js";
export type {
  BodyBlock, BoxShapeOptions, CapsuleShapeOptions, ConeShapeOptions, ConvexHullShapeOptions, CylinderShapeOptions,
  ShapeBaseOptions, ShapeOptions, SphereShapeOptions,
} from "./shapes.js";
export { blockShapes } from "./block-body.js";
export type { BlockShapes } from "./block-body.js";
export { getBlockPhysics } from "./block-physics.js";
export type { BlockPhysics, BlockPhysicsStates, FloatingMaterial } from "./block-physics.js";
export type { BlockDefinition, BlockDefinitions, BlockProperties, BlockStateValue, BlockVariant } from "./block-registry.js";
export type { BodyOptions, BodyType } from "./body.js";
export { Body } from "./body.js";
export { Drag } from "./drag.js";
export type { DragOptions } from "./drag.js";
export type {
  AngularLimitOptions, BallJointOptions, CylinderJointOptions, GenericJointOptions, HingeJointOptions,
  JointBaseOptions, JointOptions, LinearLimitOptions, RagdollJointOptions, SliderJointOptions, SpringOptions,
  UniversalJointOptions,
} from "./joints.js";
export { Joint } from "./joints.js";
export type {
  BlockBreakEvent, BlockCollisionStartEvent, BodyCollisionEndEvent, BodyCollisionStartEvent, BodySleepEvent, FluidEntryEvent, JointBreakEvent,
  PhysicsAfterEvents, Signal, StepEndEvent,
} from "./events.js";
export type {
  BlockBox, BlockCollision, BlockCollisionResolver, SurfaceMaterial, TerrainOptions, UnknownBlockPolicy,
} from "./terrain.js";
export type { FluidOptions, PhysicsWorldOptions, QueryOptions, RayCastHit, RayCastOptions, StepMode, VoidSeaOptions } from "./world.js";
export { PhysicsWorld } from "./world.js";
export type {
  BodySnapshot, JointSnapshot, JsonValue, LimitState, PhysicsSnapshot, RestoreOptions, SnapshotOptions, SpringState,
} from "./snapshot.js";
export { PersistentPhysicsWorld, PhysicsWorldStore } from "./persistence.js";
export type { PersistenceOptions, PersistenceSaveResult, PhysicsStorageTarget } from "./persistence.js";
