import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const dimension = world.getDimension("overworld");
const physics = new PhysicsWorld(dimension);

// userData 保存刚体关联的实体。
const entity = dimension.spawnEntity("minecraft:armor_stand", { x: 0, y: 80, z: 0 });
const body = physics.createBody({
  shapes: [{ type: "box", size: { x: 1, y: 1, z: 1 } }],
  location: { x: 0, y: 80, z: 0 },
  userData: entity,
});

// 仅同步未休眠刚体的实体位置。
// teleport 不提供客户端插值；位置插值见 interpolate.ts。
physics.afterEvents.stepEnd.subscribe(() => {
  if (!entity.isValid || body.isSleeping) return;
  const from = body.previousLocation, to = body.location;
  if (from.x === to.x && from.y === to.y && from.z === to.z) return;
  entity.teleport(to);
});
