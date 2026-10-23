import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

// 由一个竖直的盒子和一个水平的盒子组成 L 形
physics.createBody({
  location: { x: 0, y: 80, z: 0 },
  shapes: [
    { type: "box", size: { x: 1, y: 3, z: 1 } },
    { type: "box", size: { x: 2, y: 1, z: 1 }, offset: { x: 1.5, y: -1, z: 0 } },
  ],
});
