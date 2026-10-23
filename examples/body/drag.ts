import { system, world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));
const body = physics.createBody({
  shapes: [{ type: "box", size: { x: 1, y: 1, z: 1 } }],
  location: { x: 0, y: 80, z: 0 },
});

const drag = body.createDrag({ anchor: body.location });
drag.setTarget({ x: 2, y: 83, z: 0 });

// 释放后保留当前速度，恢复自由运动。
system.runTimeout(() => drag.remove(), 60);
