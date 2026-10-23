import { world } from "@minecraft/server";
import { blockShapes, PhysicsWorld, type BlockDefinitions } from "cutlass-physics";

// 坐标相对方块的最小角，单位为格：x 向东、y 向上、z 向南
const blocks: BlockDefinitions = {
  // 半格高的碰撞盒，质量大于默认值。
  "example:half_crate": {
    collision: [{ min: { x: 0, y: 0, z: 0 }, max: { x: 1, y: 0.5, z: 1 } }],
    mass: 2,
  },
  // 三级碰撞盒近似斜坡，默认高侧朝南。
  // example:facing 为 "north" 时，高侧朝北。
  "example:ramp": {
    collision: [
      { min: { x: 0, y: 0, z: 0 }, max: { x: 1, y: 1 / 3, z: 1 } },
      { min: { x: 0, y: 1 / 3, z: 1 / 3 }, max: { x: 1, y: 2 / 3, z: 1 } },
      { min: { x: 0, y: 2 / 3, z: 2 / 3 }, max: { x: 1, y: 1, z: 1 } },
    ],
    variants: [
      {
        when: { "example:facing": "north" },
        collision: [
          { min: { x: 0, y: 0, z: 0 }, max: { x: 1, y: 1 / 3, z: 1 } },
          { min: { x: 0, y: 1 / 3, z: 0 }, max: { x: 1, y: 2 / 3, z: 2 / 3 } },
          { min: { x: 0, y: 2 / 3, z: 0 }, max: { x: 1, y: 1, z: 1 / 3 } },
        ],
      },
    ],
  },
  // 打开时无碰撞，摩擦倍率为 0。
  "example:gate": {
    collision: [{ min: { x: 0, y: 0, z: 0.375 }, max: { x: 1, y: 1.5, z: 0.625 } }],
    friction: 0,
    variants: [{ when: { "example:open": true }, collision: [] }],
  },
  // 覆盖原版玻璃的易碎速度。
  "minecraft:glass": { fragileImpactSpeed: 6 },
};

// 登记表可由 JSON 模块导入：
// import blocks from "./blocks.json" with { type: "json" };
// 地形、刚体方块和易碎判定共享此登记表。
const physics = new PhysicsWorld(world.getDimension("overworld"), { terrain: {}, blocks });

// blockShapes 使用相同登记表解析形状和质量。
const { shapes, centerOfMass } = blockShapes([{ location: { x: 0, y: 80, z: 0 }, typeId: "example:half_crate" }], blocks);
physics.createBody({ shapes, location: centerOfMass });
