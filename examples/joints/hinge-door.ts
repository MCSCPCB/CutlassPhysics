import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

// 门绕竖直轴转动，转角范围为 0 到 90 度
const frame = physics.createBody({
  type: "static",
  shapes: [{ type: "box", size: { x: 0.2, y: 2, z: 0.2 } }],
  location: { x: 0, y: 81, z: 0 },
});
const door = physics.createBody({
  shapes: [{ type: "box", size: { x: 1, y: 2, z: 0.1 } }],
  location: { x: 0.6, y: 81, z: 0 },
});

physics.createJoint({
  type: "hinge",
  bodyA: frame,
  bodyB: door,
  anchor: { x: 0, y: 81, z: 0 },
  axis: { x: 0, y: 1, z: 0 },
  limit: { lower: 0, upper: 90 },
});
