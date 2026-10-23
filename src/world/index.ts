// 体素世界适配层，将方块数据转换为静态碰撞体。
// 通过 BlockSource 读取方块，不依赖 Minecraft 运行时。
export { BlockShapeTable, SHAPE_FULL_CUBE, SHAPE_NONE, SHAPE_UNLOADED } from "./block-shapes.js";
export type { BlockSource, LocalBox } from "./block-shapes.js";
export { FLUID_LAVA, FLUID_NONE, FLUID_WATER, fluidHeight, fluidKind } from "./block-shapes.js";
export { FLUID_GRID_MAX_CELLS, FluidGrid } from "./fluid-grid.js";
export { FluidForces } from "./fluid-forces.js";
export type { FluidBody, FluidOptions, FluidTerrain } from "./fluid-forces.js";
export { VoxelCollider } from "./voxel-collider.js";
export type { VoxelColliderOptions } from "./voxel-collider.js";
