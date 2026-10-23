import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

physics.afterEvents.stepEnd.subscribe(() => {
  physics.forEachBody((body) => {
    if (body.isSleeping) return;
    console.warn(JSON.stringify(body.location));
  });
});
