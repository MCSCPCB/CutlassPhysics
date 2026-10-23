// 地形（方块）碰撞的配置和解析接口，由 PhysicsWorld 接到体素碰撞体。
import type { Vector3 } from "./math.js";
// 仅在 {@link} 注释中引用，用于让 TypeDoc 解析链接
// eslint-disable-next-line @typescript-eslint/no-unused-vars
import type { PhysicsWorld } from "./world.js";

/**
 * 方块的轴对齐碰撞盒。
 *
 * @remarks
 * 坐标相对于方块最小角，单位为格；x 向东、y 向上、z 向南。
 * 每个方向均须满足 max > min。范围可超出方块边界，例如栅栏高度为 1.5。
 */
export interface BlockBox {
  /**
   * 盒子的最小角。
   */
  min: Vector3;
  /**
   * 盒子的最大角。
   */
  max: Vector3;
}

/**
 * 表示方块的碰撞形状，由若干个碰撞盒组成。
 *
 * @remarks
 * 空数组表示方块没有碰撞。一个 1×1×1 的盒子表示完整方块。
 */
export type BlockCollision = BlockBox[];

/**
 * 按方块类型和状态返回碰撞形状的函数。
 *
 * @remarks
 * 已登记的碰撞盒优先；其余方块调用此函数。返回 undefined 时回退到内置数据，再按 unknownBlock 处理。
 * 返回的盒子不参与邻居连接修正。结果按类型和状态缓存，每种组合仅解析一次。
 * 启用此函数后，未登记碰撞盒的方块需额外读取状态。坐标约定见 {@link BlockBox}。
 *
 * @example custom-collision.ts
 * {@includeCode ../../examples/terrain/custom-collision.ts}
 */
export type BlockCollisionResolver = (block: {
  /**
   * 方块的类型标识符，带命名空间，例如 `"minecraft:oak_stairs"`、`"example:pillar"`。
   */
  typeId: string;
  /**
   * 方块的状态，和 `block.permutation.getAllStates()` 相同。自定义方块的键是方块 JSON 里声明的状态名，例如 `"example:open"`；
   * 原版方块的键有的带 `minecraft:` 前缀（`"minecraft:cardinal_direction"`），有的不带（`"upside_down_bit"`）。
   */
  states: Record<string, string | number | boolean>;
}) => BlockCollision | undefined;

/**
 * 无法识别方块时的处理方式。
 *
 * @remarks
 * `"fullCube"` 表示按完整方块处理，`"none"` 表示按没有碰撞处理。
 */
export type UnknownBlockPolicy = "fullCube" | "none";

/**
 * 描述方块表面的材质。
 */
export interface SurfaceMaterial {
  /**
   * 摩擦系数。
   */
  friction: number;
  /**
   * 弹性。
   */
  restitution: number;
}

/**
 * 地形碰撞的选项。
 *
 * @remarks
 * 地形以 8×8×8 块缓存，同材质相邻完整方块合并为碰撞盒。
 * 刚体按运动范围请求地形，当前范围未就绪时按 unloadedChunkPolicy 处理。
 * 方块通知和定期复查使缓存失效，受变化影响的休眠刚体会被唤醒。
 *
 * @example basic-terrain.ts
 * {@includeCode ../../examples/terrain/basic-terrain.ts}
 */
export interface TerrainOptions {
  /**
   * 按规则返回方块碰撞形状的函数。
   *
   * @remarks
   * 省略时使用登记表和内置数据。解析顺序见 {@link BlockCollisionResolver}。
   * 函数不写入存档，恢复时需重新提供。
   */
  collision?: BlockCollisionResolver;
  /**
   * 无法识别方块时的处理方式。
   *
   * @remarks
   * 默认值为 `"fullCube"`：没有登记的自定义方块按完整方块处理。登记表、自定义函数和内置数据都无法识别时生效。
   */
  unknownBlock?: UnknownBlockPolicy;
  /**
   * 所有方块统一使用的表面材质。
   *
   * @remarks
   * 省略时按方块类型取材质。指定后覆盖所有方块的摩擦和弹性。
   */
  material?: SurfaceMaterial;
  /**
   * 刚体周围的地形读取余量，单位为格。
   *
   * @remarks
   * 默认为 1。当前范围每侧外扩 min(3, margin + 速度 × tick 时长)，休眠范围外扩 margin。
   */
  margin?: number;
  /**
   * 沿速度方向预读地形的 tick 数，默认为 3，0 表示关闭。须为非负整数。
   * 预读距离为 margin + 速度 × 时长，上限为 8 格；使用预读预算，不影响地形等待状态。
   */
  preloadTicks?: number;
  /**
   * 每 tick 当前地形请求的读取预算，单位为 8×8×8 块。
   *
   * @remarks
   * 默认为 4，须为正整数。预读和休眠请求共用额外的 2 块预算及当前请求的剩余预算。
   * 区块未加载时读取失败，下一 tick 重试。
   */
  maxChunksPerTick?: number;
  /**
   * 地形块失去全部引用后的缓存保留时长，单位为 tick。
   *
   * @remarks
   * 默认为 400，须为非负整数。保留期间可直接复用缓存，方块通知仅使受影响部分失效。
   * 未产生通知的变化在重新引用后由定期复查更新。
   */
  unloadDelayTicks?: number;
  /**
   * 是否监听原版方块事件。
   *
   * @remarks
   * 默认为 true。未产生事件的变化由定期复查或 {@link PhysicsWorld.invalidateBlocks} 更新。
   */
  watchBlockEvents?: boolean;
  /**
   * 每 tick 的方块复查预算，默认为 12，0 表示关闭。
   *
   * @remarks
   * 每个动态刚体轮询包围盒外扩 1 格的范围，预算为 max(2, floor(总预算 / 刚体数))。
   * 包含休眠刚体，不包含等待地形的刚体。显式失效通知见 {@link PhysicsWorld.invalidateBlocks}。
   */
  revalidateBlocksPerTick?: number;
  /**
   * 附近地形未就绪时的处理方式。
   *
   * @remarks
   * 默认为 freeze，保留速度并冻结关节相连的整组刚体，地形就绪后恢复。
   * fall 将未加载或尚未读取的地形视为空，刚体可能穿过该区域。
   */
  unloadedChunkPolicy?: "freeze" | "fall";
}
