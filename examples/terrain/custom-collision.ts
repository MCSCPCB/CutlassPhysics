import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

// 按类型名称匹配具有相同碰撞形状的方块。
// 此处将所有 example:*_pillar 方块解析为细柱。
const physics = new PhysicsWorld(world.getDimension("overworld"), {
  terrain: {
    collision: ({ typeId, states }) => {
      if (typeId.startsWith("example:") && typeId.endsWith("_pillar")) {
        // states 和 block.permutation.getAllStates() 相同，键是方块 JSON 里声明的状态名
        const axis = states["minecraft:pillar_axis"] ?? states["pillar_axis"];
        if (axis === "x") return [{ min: { x: 0, y: 0.25, z: 0.25 }, max: { x: 1, y: 0.75, z: 0.75 } }];
        if (axis === "z") return [{ min: { x: 0.25, y: 0.25, z: 0 }, max: { x: 0.75, y: 0.75, z: 1 } }];
        return [{ min: { x: 0.25, y: 0, z: 0.25 }, max: { x: 0.75, y: 1, z: 0.75 } }];
      }
      // 返回 undefined 时使用内置碰撞数据。
      return undefined;
    },
  },
});

physics.createBody({
  shapes: [{ type: "sphere", radius: 0.25 }],
  location: { x: 0, y: 100, z: 0 },
});
