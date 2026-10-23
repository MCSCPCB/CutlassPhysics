import { system } from "@minecraft/server";
import { heldItemTypeId, resolveSubLevelBlockRotation, resolveSubLevelBlockVisualOffset, resolveSubLevelBlockVisualYOffset } from "./block-visuals.js";
import { cellPoint } from "./structures.js";

export const ENTITY_TYPE = "cutlass:physics_block";
export const CELL_PROPERTY = "cutlass:cell";

export class Renderer {
  constructor(dimension) {
    this.dimension = dimension;
    this.entries = new Map();
    this.byEntity = new Map();
    this.serial = 0;
  }

  add(record) {
    for (const cell of record.data.cells) {
      const existing = this.entries.get(cell.id);
      const entry = { ...existing, record, cell, refreshed: -100, dirty: true };
      this.entries.set(cell.id, entry);
      if (entry.entity) this.byEntity.set(entry.entity.id, entry);
    }
  }

  restore() {
    for (const entity of this.dimension.getEntities({ type: ENTITY_TYPE })) this.attach(entity);
  }

  attach(entity) {
    const entry = this.entries.get(entity.getDynamicProperty(CELL_PROPERTY));
    if (!entry || (entry.entity?.isValid && entry.entity.id !== entity.id)) {
      entity.remove();
      return;
    }
    entry.entity = entity;
    this.byEntity.set(entity.id, entry);
    entry.dirty = true;
    this.configure(entry);
  }

  configure(entry) {
    const { cell, entity } = entry;
    const r = resolveSubLevelBlockRotation(cell.typeId, cell.states) ?? { x: 0, y: 90, z: 0 };
    const offset = resolveSubLevelBlockVisualOffset(cell.typeId, cell.states) ?? { x: 0, y: 0, z: 0 };
    for (const [name, value] of Object.entries({
      local_pitch: r.x, local_yaw: r.y, local_roll: r.z,
      local_x: offset.x, local_y: offset.y + resolveSubLevelBlockVisualYOffset(cell.typeId, cell.states), local_z: offset.z,
    })) entity.setProperty(`cutlass:${name}`, value);
    entity.runCommand(`replaceitem entity @s slot.weapon.mainhand 0 ${heldItemTypeId(cell.typeId, cell.states)}`);
    entity.setProperty("cutlass:scale", 1);
  }

  updateRecord(record, required = false) {
    if (!record.body.isValid) return;
    const rotation = record.body.rotation;
    const sleeping = record.body.isSleeping;
    for (let i = 0; i < record.data.cells.length; i++) {
      const entry = this.entries.get(record.data.cells[i].id);
      if (!entry) continue;
      if (!entry.dirty && sleeping && system.currentTick - entry.refreshed < 40 && entry.entity?.isValid) continue;
      const point = cellPoint(record, entry.cell);
      const location = { x: point.x, y: point.y - 0.5, z: point.z };
      if (!entry.entity?.isValid) {
        if (!required && !this.dimension.isChunkLoaded(location)) continue;
        const entity = this.dimension.spawnEntity(ENTITY_TYPE, location);
        if (entry.entity) this.byEntity.delete(entry.entity.id);
        entry.entity = entity;
        this.byEntity.set(entity.id, entry);
        try {
          entity.setDynamicProperty(CELL_PROPERTY, entry.cell.id);
          this.configure(entry);
        } catch (error) { this.byEntity.delete(entity.id); entity.remove(); entry.entity = undefined; throw error; }
      }
      const entity = entry.entity;
      entity.teleport(location, { rotation: { x: 0, y: 0 } });
      entity.clearVelocity();
      // 姿态输入使用独立控制器，静止实体定期刷新以覆盖后进入的客户端。
      const q = rotation;
      entity.playAnimation("animation.cutlass.fancy.input", {
        controller: "cutlass_pose",
        stopExpression: `v.pose_moving=${sleeping ? 0 : 1};v.pose_serial=${this.serial++ % 1000000};v.pose_dt=1;v.pose_cn=${system.currentTick % 1000000};v.pose_cx=0;v.pose_cy=0.5;v.pose_cz=0;v.pose_cqx=${q.x};v.pose_cqy=${q.y};v.pose_cqz=${q.z};v.pose_cqw=${q.w};return 0;`,
      });
      entry.refreshed = system.currentTick;
      entry.dirty = false;
    }
  }

  removeCell(id) {
    const entry = this.entries.get(id);
    this.entries.delete(id);
    if (entry?.entity) this.byEntity.delete(entry.entity.id);
    if (entry?.entity?.isValid) entry.entity.remove();
  }

  remove(record) {
    for (const cell of record.data.cells) this.removeCell(cell.id);
  }
}
