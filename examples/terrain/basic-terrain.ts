import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"), { terrain: {} });

physics.createBody({
  shapes: [{ type: "sphere", radius: 0.25 }],
  location: { x: 0, y: 100, z: 0 },
});
