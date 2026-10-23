import { system, world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));
const balloon = physics.createBody({
  shapes: [{ type: "sphere", radius: 0.5 }],
  location: { x: 0, y: 80, z: 0 },
});

// 每 tick 施加与重力等大、反向的力；力在下一步结束后清零。
system.runInterval(() => {
  balloon.applyForce({ x: 0, y: -balloon.mass * physics.gravity.y, z: 0 });
});
