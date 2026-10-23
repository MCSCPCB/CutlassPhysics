import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

console.warn(`物理世界已创建，当前有 ${physics.bodyCount} 个刚体`);
