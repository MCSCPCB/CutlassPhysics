import { getBlockPhysics } from "cutlass-physics";

// 状态无关的属性可仅按方块类型查询。
console.warn(getBlockPhysics("minecraft:iron_block").mass); // 4
console.warn(getBlockPhysics("minecraft:packed_ice").friction); // 0
// 双层台阶的质量是两倍
console.warn(getBlockPhysics("minecraft:stone_block_slab", { "minecraft:vertical_half": "bottom" }).mass);
// 自然生成的树叶具有易碎属性，玩家放置的树叶没有该属性。
console.warn(getBlockPhysics("minecraft:oak_leaves", { persistent_bit: true }).fragileImpactSpeed); // undefined
console.warn(getBlockPhysics("minecraft:oak_leaves", { persistent_bit: false }).fragileImpactSpeed); // 4
