import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

try {
  physics.createBody({ shapes: [{ type: "box", size: { x: 0, y: 1, z: 1 } }] });
} catch (error) {
  console.warn(`创建失败：${(error as Error).message}`);
}
