import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

physics.afterEvents.bodyCollisionStart.subscribe((event) => {
  if (event.impactSpeed > 5) {
    console.warn(`重重地撞了一下，速度 ${event.impactSpeed.toFixed(1)} 格/秒`);
  }
});
