import { world } from "@minecraft/server";
import { PhysicsWorld, type Body } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

// 用球关节依次连接五个小球，第一个小球固定在空中
let previous: Body = physics.createBody({
  type: "static",
  shapes: [{ type: "sphere", radius: 0.2 }],
  location: { x: 0, y: 90, z: 0 },
});

for (let i = 1; i <= 5; i++) {
  const link = physics.createBody({
    shapes: [{ type: "sphere", radius: 0.2 }],
    location: { x: i * 0.5, y: 90, z: 0 },
  });
  physics.createJoint({ type: "ball", bodyA: previous, bodyB: link, anchor: { x: i * 0.5 - 0.25, y: 90, z: 0 } });
  previous = link;
}
