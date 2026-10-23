import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"), { terrain: {} });

// 沿玩家的视线方向检测
const player = world.getAllPlayers()[0];
const hit = physics.rayCast(player.getHeadLocation(), player.getViewDirection(), { maxDistance: 20 });

if (hit?.body) {
  hit.body.applyImpulse({ x: 0, y: 3 * hit.body.mass, z: 0 });
} else if (hit?.blockLocation) {
  console.warn(`看着方块 ${JSON.stringify(hit.blockLocation)}，距离 ${hit.distance.toFixed(1)}`);
}
