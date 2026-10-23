import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));
const crate = physics.createBody({
  shapes: [{ type: "box", size: { x: 1, y: 1, z: 1 } }],
  location: { x: 0, y: 80, z: 0 },
});

// 速度变化量等于冲量除以质量，此处使向上的速度增加 5 格/秒
crate.applyImpulse({ x: 0, y: 5 * crate.mass, z: 0 });
