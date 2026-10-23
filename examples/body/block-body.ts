import { world } from "@minecraft/server";
import { blockShapes, PhysicsWorld } from "cutlass-physics";

const dimension = world.getDimension("overworld");
const physics = new PhysicsWorld(dimension);

// 将 3×2×3 范围内的方块转换为刚体，保留各方块的物理属性。
const blocks = [];
for (let x = 0; x < 3; x++) for (let y = 0; y < 2; y++) for (let z = 0; z < 3; z++) {
  const location = { x: 10 + x, y: 70 + y, z: 10 + z };
  const block = dimension.getBlock(location);
  if (block === undefined || block.isAir) continue;
  blocks.push({ location, typeId: block.typeId, states: block.permutation.getAllStates() });
}
const { shapes, centerOfMass } = blockShapes(blocks);
for (const b of blocks) dimension.setBlockType(b.location, "minecraft:air");
// 刚体原点设为质心，方块保持原位置。
const body = physics.createBody({ shapes, location: centerOfMass });

// 易碎方块的撞击事件包含对应的形状下标。
physics.afterEvents.blockBreak.subscribe((e) => {
  if (e.body === body && e.shapeIndex !== undefined) console.warn(`${e.typeId} 撞碎了`);
});
