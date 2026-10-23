// 对外 API：碰撞形状的选项。转换成内核形状在 convert.ts。
import type { Quaternion, Vector3 } from "./math.js";
// 仅在 {@link} 注释中引用，用于让 TypeDoc 解析链接
// eslint-disable-next-line @typescript-eslint/no-unused-vars
import type { blockShapes } from "./block-body.js";
// eslint-disable-next-line @typescript-eslint/no-unused-vars
import type { getBlockPhysics } from "./block-physics.js";
// eslint-disable-next-line @typescript-eslint/no-unused-vars
import type { PhysicsWorldOptions } from "./world.js";

/**
 * 所有碰撞形状共有的选项。
 *
 * @remarks
 * 尺寸均为完整尺寸，`size: { x: 1, y: 1, z: 1 }` 的大小与一个方块相同。两个形状仅当“A 的碰撞分组与 B 的碰撞掩码”和“B 的碰撞分组与 A 的碰撞掩码”的按位与结果均不为零时才会发生碰撞。两个形状接触时，摩擦系数取二者的几何平均值。
 */
export interface ShapeBaseOptions {
  /**
   * 形状中心相对于刚体原点的偏移。
   *
   * @remarks
   * 默认值为 `{ x: 0, y: 0, z: 0 }`。
   */
  offset?: Vector3;
  /**
   * 形状相对于刚体的朝向。
   *
   * @remarks
   * 默认值为没有旋转。
   */
  rotation?: Quaternion;
  /**
   * 形状的摩擦系数。
   *
   * @remarks
   * 默认值为 `0.525`。两个形状接触时取两侧摩擦的几何平均：两个默认形状之间是 `0.525`，默认形状和普通方块之间是 `0.5125`。
   */
  friction?: number;
  /**
   * 形状的弹性。0 表示不反弹，1 表示完全弹性。
   *
   * @remarks
   * 默认为 0。接触时取两侧较大的值；非零弹性可能使堆叠物体持续弹跳。
   */
  restitution?: number;
  /**
   * 形状的密度，单位为 kg/格³。
   *
   * @remarks
   * 默认值为 `1`。
   */
  density?: number;
  /**
   * 形状所属的碰撞分组，为位掩码中的一位。
   *
   * @remarks
   * 默认值为 `1`。
   */
  collisionGroup?: number;
  /**
   * 形状参与碰撞的分组掩码。
   *
   * @remarks
   * 默认值为 `1`。
   */
  collisionMask?: number;
  /**
   * 形状对应的方块信息。
   *
   * @remarks
   * 由 {@link blockShapes} 生成，随存档保存。悬浮和易碎属性按所属世界的方块登记表解析。
   * 质量、摩擦和弹性仍取形状的 density、friction、restitution。
   */
  block?: BodyBlock;
}

/**
 * 拼成刚体的一个方块。
 */
export interface BodyBlock {
  /**
   * 方块的位置：方块最小角的整数坐标，例如方块在世界里的坐标。
   */
  location: Vector3;
  /**
   * 方块的类型标识符，例如 `"minecraft:oak_planks"`。
   */
  typeId: string;
  /**
   * 方块的状态，例如 `block.permutation.getAllStates()`。省略时按没有状态处理。
   */
  states?: Record<string, boolean | number | string>;
}

/**
 * 描述长方体形状的选项。
 */
export interface BoxShapeOptions extends ShapeBaseOptions {
  /** 形状类型：长方体 */
  type: "box";
  /**
   * 长方体在三个方向上的边长，单位为格。每个分量必须大于零。
   */
  size: Vector3;
}

/**
 * 描述球形状的选项。
 */
export interface SphereShapeOptions extends ShapeBaseOptions {
  /** 形状类型：球 */
  type: "sphere";
  /**
   * 球的半径，单位为格。必须大于零。
   */
  radius: number;
}

/**
 * 描述胶囊体形状的选项。胶囊体沿 Y 轴放置。
 */
export interface CapsuleShapeOptions extends ShapeBaseOptions {
  /** 形状类型：胶囊体 */
  type: "capsule";
  /**
   * 胶囊体的半径，单位为格。必须大于零。
   */
  radius: number;
  /**
   * 胶囊体的总高度，单位为格，包含两端的半球。不得小于 `radius * 2`。
   */
  height: number;
}

/**
 * 描述圆柱形状的选项。圆柱沿 Y 轴放置。
 */
export interface CylinderShapeOptions extends ShapeBaseOptions {
  /** 形状类型：圆柱 */
  type: "cylinder";
  /**
   * 圆柱的半径，单位为格。必须大于零。
   */
  radius: number;
  /**
   * 圆柱的总高度，单位为格。必须大于零。
   */
  height: number;
}

/**
 * 描述圆锥形状的选项。圆锥沿 Y 轴放置，尖端朝向 +Y。
 */
export interface ConeShapeOptions extends ShapeBaseOptions {
  /** 形状类型：圆锥 */
  type: "cone";
  /**
   * 圆锥的底面半径，单位为格。必须大于零。
   */
  radius: number;
  /**
   * 圆锥的总高度，单位为格。必须大于零。
   */
  height: number;
}

/**
 * 由一组点的最小凸多面体定义的形状。
 *
 * @remarks
 * 计算开销随顶点数增加。单个凸包不表示凹形结构。
 */
export interface ConvexHullShapeOptions extends ShapeBaseOptions {
  /** 形状类型：凸包 */
  type: "convexHull";
  /**
   * 构成凸包的点，坐标相对于形状原点。至少需要 4 个不共面的点。
   */
  points: Vector3[];
}

/**
 * 描述碰撞形状的选项，通过 `type` 区分种类。
 */
export type ShapeOptions =
  | BoxShapeOptions
  | SphereShapeOptions
  | CapsuleShapeOptions
  | CylinderShapeOptions
  | ConeShapeOptions
  | ConvexHullShapeOptions;
