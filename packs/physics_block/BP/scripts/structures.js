import { ItemTypes } from "@minecraft/server";
import { blockShapes } from "./physics.js";
import { heldItemTypeId } from "./block-visuals.js";
import { connectedComponents } from "./topology.js";

export const MAX_BLOCKS = 256;
export const MAX_EDGE = 24;
let serial = 0;
const sessionId = `${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`;
const nextId = () => `${sessionId}-${serial++}`;

export function cellPoint(record, cell) {
  const c = record.data.center;
  return record.body.localToWorld({
    x: cell.location.x + 0.5 - c.x,
    y: cell.location.y + 0.5 - c.y,
    z: cell.location.z + 0.5 - c.z,
  });
}

export class Structures {
  constructor(dimension, physics, renderer) {
    this.dimension = dimension;
    this.physics = physics;
    this.renderer = renderer;
    this.records = new Map();
    this.pending = new Set();
  }

  restore() {
    for (const body of this.physics.getBodies()) {
      const data = body.userData;
      // 牵引锚点随玩家操作重建。
      if (data?.kind === "grab") { body.remove(); continue; }
      if (data?.kind !== "blocks") throw new Error("方块结构存档缺少显示数据。");
      const record = { body, data };
      this.records.set(body, record);
      this.renderer.add(record);
    }
    this.renderer.restore();
  }

  assemble(first, second) {
    const min = {}, max = {};
    for (const axis of ["x", "y", "z"]) {
      min[axis] = Math.min(first[axis], second[axis]);
      max[axis] = Math.max(first[axis], second[axis]);
      if (max[axis] - min[axis] + 1 > MAX_EDGE) throw new Error(`选区每边最多 ${MAX_EDGE} 格。`);
    }
    const captured = [];
    for (let y = min.y; y <= max.y; y++) for (let z = min.z; z <= max.z; z++) for (let x = min.x; x <= max.x; x++) {
      const block = this.dimension.getBlock({ x, y, z });
      if (!block) throw new Error("选区包含未加载的区块。");
      if (block.isAir || block.isLiquid) continue;
      const inventory = block.getComponent("minecraft:inventory")?.container;
      if (inventory && inventory.emptySlotsCount !== inventory.size) throw new Error("含物品的容器暂不支持物理化。");
      const states = block.permutation.getAllStates();
      const itemTypeId = heldItemTypeId(block.typeId, states);
      if (!ItemTypes.get(itemTypeId)) throw new Error(`此方块没有可用的手持物品：${block.typeId}`);
      captured.push({ block, permutation: block.permutation, waterlogged: block.isWaterlogged, cell: {
        id: nextId(), location: { x: x - min.x, y: y - min.y, z: z - min.z }, typeId: block.typeId, states,
      } });
      if (captured.length > MAX_BLOCKS) throw new Error(`每次最多物理化 ${MAX_BLOCKS} 个方块。`);
    }
    if (!captured.length) throw new Error("选区内没有可物理化的方块。");

    const added = [], cleared = [];
    try {
      for (const cells of connectedComponents(captured.map(entry => entry.cell))) {
        const geometry = blockShapes(cells);
        const record = this.create(cells, geometry, {
          location: { x: min.x + geometry.centerOfMass.x, y: min.y + geometry.centerOfMass.y, z: min.z + geometry.centerOfMass.z },
        });
        added.push(record);
        this.renderer.add(record);
        this.renderer.updateRecord(record, true);
      }
      for (const entry of captured) {
        cleared.push(entry);
        entry.block.setType(entry.waterlogged ? "minecraft:water" : "minecraft:air");
      }
      this.physics.invalidateBlocks(min, max);
      return captured.length;
    } catch (error) {
      for (const record of added) {
        this.renderer.remove(record);
        this.records.delete(record.body);
        record.body.remove();
      }
      for (const entry of cleared) {
        entry.block.setPermutation(entry.permutation);
        if (entry.waterlogged) entry.block.setWaterlogged(true);
      }
      this.physics.invalidateBlocks(min, max);
      throw error;
    }
  }

  create(cells, geometry, options) {
    const data = { kind: "blocks", id: nextId(), cells, center: geometry.centerOfMass };
    const body = this.physics.createBody({ ...options, shapes: geometry.shapes, userData: data });
    const record = { body, data };
    this.records.set(body, record);
    return record;
  }

  breakCell(id) {
    if (this.renderer.entries.has(id)) this.pending.add(id);
  }

  flushBreaks() {
    if (!this.pending.size) return false;
    const affected = new Map();
    for (const id of this.pending) {
      const entry = this.renderer.entries.get(id);
      if (!entry) continue;
      if (!affected.has(entry.record)) affected.set(entry.record, new Set());
      affected.get(entry.record).add(id);
    }
    for (const [record, ids] of affected) {
      this.split(record, ids);
      for (const id of ids) this.pending.delete(id);
    }
    return affected.size > 0;
  }

  split(record, removed) {
    const body = record.body;
    const remaining = record.data.cells.filter(cell => !removed.has(cell.id));
    const added = [];
    try {
      for (const cells of connectedComponents(remaining)) {
        let geometry;
        try { geometry = blockShapes(cells); }
        catch (error) {
          // 没有独立碰撞体的装饰物随断开的连接移除。
          if (String(error).includes("全部没有碰撞或没有质量")) {
            for (const cell of cells) removed.add(cell.id);
            continue;
          }
          throw error;
        }
        const c = geometry.centerOfMass, old = record.data.center;
        const location = body.localToWorld({ x: c.x - old.x, y: c.y - old.y, z: c.z - old.z });
        added.push(this.create(cells, geometry, {
          location, rotation: body.rotation,
          linearVelocity: body.getVelocityAtPoint(location), angularVelocity: body.angularVelocity,
        }));
      }
    } catch (error) {
      for (const fragment of added) { this.records.delete(fragment.body); fragment.body.remove(); }
      throw error;
    }
    for (const fragment of added) this.renderer.add(fragment);
    for (const id of removed) this.renderer.removeCell(id);
    this.records.delete(body);
    body.remove();
  }
}
