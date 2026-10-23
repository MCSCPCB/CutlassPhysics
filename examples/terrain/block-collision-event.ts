import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"), { terrain: {} });

physics.afterEvents.blockCollisionStart.subscribe(({ blockLocation, impactSpeed }) => {
  const { x, y, z } = blockLocation;
  console.warn(`撞上了方块 (${x}, ${y}, ${z})，速度 ${impactSpeed.toFixed(1)} 格/秒`);
});
