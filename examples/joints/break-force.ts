import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

const anchor = physics.createBody({ type: "static", shapes: [{ type: "sphere", radius: 0.1 }], location: { x: 0, y: 90, z: 0 } });
const weight = physics.createBody({ shapes: [{ type: "box", size: { x: 1, y: 1, z: 1 } }], location: { x: 0, y: 88, z: 0 } });

// 受力超过 500 牛顿时关节断开
physics.createJoint({ type: "ball", bodyA: anchor, bodyB: weight, anchor: { x: 0, y: 90, z: 0 }, breakForce: 500 });

physics.afterEvents.jointBreak.subscribe(() => {
  console.warn("关节断了");
});
