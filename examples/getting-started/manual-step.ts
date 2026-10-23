import { system, world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"), { stepMode: "manual" });

physics.createBody({
  shapes: [{ type: "sphere", radius: 0.5 }],
  location: { x: 0, y: 80, z: 0 },
});

// 每个 tick 推进一步
system.runInterval(() => {
  physics.step();
});
