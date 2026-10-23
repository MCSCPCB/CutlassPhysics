import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

const crate = physics.createBody({
  shapes: [{ type: "box", size: { x: 1, y: 1, z: 1 } }],
  location: { x: 0, y: 80, z: 0 },
});

console.warn(`刚体质量：${crate.mass} kg`);
