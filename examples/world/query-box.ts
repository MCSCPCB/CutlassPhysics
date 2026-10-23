import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

// 查询以 (0, 80, 0) 为中心、边长为 10 的范围内的刚体，并使其远离中心
const center = { x: 0, y: 80, z: 0 };
const bodies = physics.queryBox(
  { x: center.x - 5, y: center.y - 5, z: center.z - 5 },
  { x: center.x + 5, y: center.y + 5, z: center.z + 5 },
);

for (const body of bodies) {
  const away = { x: body.location.x - center.x, y: body.location.y - center.y + 1, z: body.location.z - center.z };
  body.applyImpulse({ x: away.x * body.mass, y: away.y * body.mass, z: away.z * body.mass });
}
