import { system, world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));
const crate = physics.createBody({
  shapes: [{ type: "box", size: { x: 1, y: 1, z: 1 } }],
  location: { x: 0, y: 80, z: 0 },
});

// 5 秒后移除刚体，此后 isValid 为 false
system.runTimeout(() => {
  crate.remove();
  console.warn(`isValid：${crate.isValid}`);
}, 100);
