// 将方块转换为刚体形状，按方块属性设置质量、摩擦和弹性。
// ShapeOptions.block 记录对应方块，供悬浮和易碎判定使用。
// 形状以质心为原点，与内核的重心约定一致。
import type { LocalBox } from "../world/block-shapes.js";
import { SHAPE_FULL_CUBE, SHAPE_NONE } from "../world/block-shapes.js";
import { stateBoxes } from "./block-neighbors.js";
import { getBlockPhysics, registeredPhysics, type BlockPhysics } from "./block-physics.js";
import { registryFor, type BlockDefinitions, type BlockRegistry } from "./block-registry.js";
import { isStateShaped, resolveShapeId, shapeBoxes, SHAPE_UNKNOWN } from "./block-shapes.js";
import type { Vector3 } from "./math.js";
import { BASE_FRICTION } from "./materials.js";
import type { BodyBlock, BoxShapeOptions } from "./shapes.js";

/**
 * {@link blockShapes} 的结果。
 */
export interface BlockShapes {
  /** 以质心为原点的形状列表。 */
  shapes: BoxShapeOptions[];
  /** 与输入方块位置处于同一坐标系的质心。 */
  centerOfMass: Vector3;
  /**
   * 总质量。
   */
  mass: number;
}

const FULL: readonly LocalBox[] = [[0, 0, 0, 1, 1, 1]];

/** 方块碰撞盒，登记表优先。无碰撞方块返回空数组，未知方块按完整方块处理。 */
function boxesOf(typeId: string, states: Readonly<Record<string, boolean | number | string>>, registry: BlockRegistry | undefined): readonly LocalBox[] {
  const registered = registry?.get(typeId)?.match(states).boxes;
  if (registered !== undefined) return registered;
  if (isStateShaped(typeId)) {
    const boxes = stateBoxes(typeId, states);
    if (boxes !== undefined) return boxes;
  }
  const id = resolveShapeId(typeId, states);
  if (id === SHAPE_NONE) return [];
  if (id === SHAPE_FULL_CUBE || id === SHAPE_UNKNOWN) return FULL;
  return shapeBoxes(id);
}

/**
 * 将一组方块转换为刚体形状。
 *
 * @remarks
 *
 * - 无碰撞或无质量的方块不生成形状。
 * - 质量按方块属性计算，与碰撞盒体积无关。
 * - 登记表覆盖内置数据；与所属世界使用相同登记表时，方块属性一致。
 * - 形状以质心为原点，刚体位置设为 `centerOfMass` 时保留输入位置。
 * - 栅栏、玻璃板、楼梯读取连接状态；门和活塞仅按自身状态解析。
 * - 栅栏、栅栏门和墙按 1 格显示高度生成形状。
 *
 * @param blocks 方块列表，位置须唯一。
 * @param definitions 方块登记表，省略时使用内置数据。
 * @returns 形状列表、质心和总质量。
 * @throws 方块位置非整数、无可生成的形状或登记表无效时抛出错误。
 *
 * @example block-body.ts
 * {@includeCode ../../examples/body/block-body.ts}
 */
export function blockShapes(blocks: readonly BodyBlock[], definitions?: BlockDefinitions): BlockShapes {
  const registry = definitions === undefined ? undefined : registryFor(definitions);
  const parts: { block: BodyBlock; physics: BlockPhysics; boxes: readonly LocalBox[]; volume: number }[] = [];
  let mass = 0, mx = 0, my = 0, mz = 0;
  for (const block of blocks) {
    const l = block.location;
    if (!Number.isInteger(l.x) || !Number.isInteger(l.y) || !Number.isInteger(l.z)) {
      throw new Error(`方块的位置必须是整数，收到 (${l.x}, ${l.y}, ${l.z})`);
    }
    const states = block.states ?? {};
    const boxes = boxesOf(block.typeId, states, registry);
    if (boxes.length === 0) continue;
    const physics = registry === undefined ? getBlockPhysics(block.typeId, states) : registeredPhysics(registry, block.typeId, states);
    if (!(physics.mass > 0)) continue;
    let volume = 0, cx = 0, cy = 0, cz = 0;
    for (const b of boxes) {
      const v = (b[3] - b[0]) * (b[4] - b[1]) * (b[5] - b[2]);
      volume += v;
      cx += v * (b[0] + b[3]) / 2; cy += v * (b[1] + b[4]) / 2; cz += v * (b[2] + b[5]) / 2;
    }
    parts.push({ block, physics, boxes, volume });
    mass += physics.mass;
    mx += physics.mass * (l.x + cx / volume); my += physics.mass * (l.y + cy / volume); mz += physics.mass * (l.z + cz / volume);
  }
  if (parts.length === 0) throw new Error("没有可以生成形状的方块（全部没有碰撞或没有质量）");
  const com = { x: mx / mass, y: my / mass, z: mz / mass };
  const shapes: BoxShapeOptions[] = [];
  for (const { block, physics, boxes, volume } of parts) {
    const l = block.location;
    // 质量按方块计：几个盒子按体积分这个方块的质量
    const density = physics.mass / volume;
    // 单侧摩擦系数为 0.525 × 倍率²；接触时取两侧的几何平均。
    const friction = BASE_FRICTION * physics.friction * physics.friction;
    const copy: BodyBlock = block.states === undefined
      ? { location: { x: l.x, y: l.y, z: l.z }, typeId: block.typeId }
      : { location: { x: l.x, y: l.y, z: l.z }, typeId: block.typeId, states: { ...block.states } };
    for (const b of boxes) {
      shapes.push({
        type: "box",
        size: { x: b[3] - b[0], y: b[4] - b[1], z: b[5] - b[2] },
        offset: { x: l.x + (b[0] + b[3]) / 2 - com.x, y: l.y + (b[1] + b[4]) / 2 - com.y, z: l.z + (b[2] + b[5]) / 2 - com.z },
        density, friction, restitution: physics.restitution,
        block: copy,
      });
    }
  }
  return { shapes, centerOfMass: com, mass };
}
