import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));
const ball = physics.createBody({
  shapes: [{ type: "sphere", radius: 0.5 }],
  location: { x: 0, y: 80, z: 0 },
});

physics.afterEvents.stepEnd.subscribe(() => {
  console.warn(JSON.stringify(ball.location));
});
