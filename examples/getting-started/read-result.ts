import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));
const crate = physics.createBody({
  shapes: [{ type: "box", size: { x: 1, y: 1, z: 1 } }],
  location: { x: 0, y: 80, z: 0 },
});

physics.afterEvents.stepEnd.subscribe(() => {
  const { x, y, z } = crate.location;
  console.warn(`位置：${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)}`);
});
