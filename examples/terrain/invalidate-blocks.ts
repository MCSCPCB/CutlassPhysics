import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"), { terrain: {} });

// 使用命令填充方块后，通知世界重新读取该范围
physics.invalidateBlocks({ x: 0, y: 60, z: 0 }, { x: 15, y: 70, z: 15 });
