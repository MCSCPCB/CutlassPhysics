import { system } from "@minecraft/server";
import { heldItemTypeId, resolveSubLevelBlockRotation, resolveSubLevelBlockVisualOffset, resolveSubLevelBlockVisualYOffset } from "./block-visuals.js";
import { RenderPose } from "./render-pose.js";

export const ENTITY_TYPE = "cutlass:physics_block";
export const CELL_PROPERTY = "cutlass:cell";
let itemBatch = 0;

export class Renderer {
  constructor(dimension) {
    this.dimension = dimension;
    this.entries = new Map();
    this.byEntity = new Map();
    this.poses = new WeakMap();
    this.records = new WeakMap();
  }

  add(record) {
    const state = { entries: [], dirty: true, generation: -1, cursor: 0, retryAt: 0 };
    this.records.set(record, state);
    for (const cell of record.data.cells) {
      const existing = this.entries.get(cell.id);
      const entry = { ...existing, record, cell, dirty: true };
      this.entries.set(cell.id, entry);
      state.entries.push(entry);
      if (entry.entity) this.byEntity.set(entry.entity.id, entry);
    }
  }

  restore() {
    for (const entity of this.dimension.getEntities({ type: ENTITY_TYPE })) this.attach(entity);
  }

  /** 爆炸后恢复显示锚点，清除实体的原生击退速度。 */
  resetAnchors() {
    for (const entry of this.entries.values()) {
      if (!entry.entity?.isValid || !entry.pose) continue;
      entry.entity.clearVelocity();
      entry.entity.teleport(entry.pose.worldAnchor, { rotation: { x: 0, y: 0 } });
    }
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
    this.records.get(entry.record).dirty = true;
    this.records.get(entry.record).retryAt = 0;
  }

  poseFor(record) {
    let pose = this.poses.get(record);
    if (pose) return pose;
    let selected = record.data.cells[0], distance = Infinity;
    for (const cell of record.data.cells) {
      const p = cell.location;
      const d = p.x * p.x + (p.y - 1) ** 2 + p.z * p.z;
      if (d < distance) { selected = cell; distance = d; }
    }
    const c = record.data.center, p = selected.location;
    const anchor = { x: p.x + 0.5 - c.x, y: p.y + 0.5 - c.y, z: p.z + 0.5 - c.z };
    const body = record.body;
    pose = new RenderPose({
      localPointToWorld: point => body.localToWorld(point),
      getRenderQuaternion: () => body.rotation,
      get isSleeping() { return body.isSleeping; },
      get velocity() { return body.linearVelocity; },
    }, anchor, this.dimension);
    this.poses.set(record, pose);
    return pose;
  }

  configure(entry, pose) {
    const { cell, entity, record } = entry;
    const r = resolveSubLevelBlockRotation(cell.typeId, cell.states) ?? { x: 0, y: 90, z: 0 };
    const offset = resolveSubLevelBlockVisualOffset(cell.typeId, cell.states) ?? { x: 0, y: 0, z: 0 };
    const c = record.data.center, a = pose.anchor, p = cell.location;
    for (const [name, value] of Object.entries({
      local_pitch: r.x, local_yaw: r.y, local_roll: r.z,
      local_x: p.x + 0.5 - c.x - a.x + offset.x,
      local_y: p.y + 0.5 - c.y - a.y + offset.y + resolveSubLevelBlockVisualYOffset(cell.typeId, cell.states),
      local_z: p.z + 0.5 - c.z - a.z + offset.z,
    })) entity.setProperty(`cutlass:${name}`, value);
  }

  equip(added) {
    const batches = new Map();
    for (const { entry, entity } of added) {
      const item = heldItemTypeId(entry.cell.typeId, entry.cell.states);
      let batch = batches.get(item);
      if (!batch) { batch = []; batches.set(item, batch); }
      batch.push(entity);
    }
    for (const [item, entities] of batches) {
      const tag = `cutlass_item_${itemBatch++}`;
      try {
        for (const entity of entities) entity.addTag(tag);
        const result = this.dimension.runCommand(`replaceitem entity @e[type=${ENTITY_TYPE},tag=${tag}] slot.weapon.mainhand 0 ${item} 1`);
        if (!(result.successCount > 0)) throw new Error("手持物品设置未完成。");
      } finally {
        for (const entity of entities) if (entity.isValid) entity.removeTag(tag);
      }
    }
  }

  updateRecord(record, required = false) {
    const state = this.records.get(record);
    if (!record.body.isValid || (!required && system.currentTick < state.retryAt)) return;
    if (!this.poses.has(record) && !required && !this.dimension.isChunkLoaded(record.body.location)) { state.retryAt = system.currentTick + 20; return; }
    const pose = this.poseFor(record);
    pose.updateWorldAnchor();
    pose.sampleForRender();
    const replaceAll = state.generation !== pose.anchorRevision;
    // 缺失实体分批复查；结构编辑和锚点迁移立即处理整组。
    let entries;
    if (required || state.dirty || replaceAll) entries = state.entries;
    else {
      entries = [];
      const count = Math.ceil(state.entries.length / 20);
      for (let i = 0; i < count; i++) {
        const entry = state.entries[state.cursor++ % state.entries.length];
        if (!entry.entity?.isValid) entries.push(entry);
      }
    }
    if (entries.length && !required && !this.dimension.isChunkLoaded(record.body.location)) { state.retryAt = system.currentTick + 20; return; }
    const added = [];
    try {
      for (const entry of entries) {
        const cell = entry.cell;
        if (this.entries.get(cell.id) !== entry) continue;
        const replace = entry.pose && (entry.pose !== pose || entry.generation !== pose.anchorRevision);
        if (!entry.dirty && !replace && entry.entity?.isValid) continue;
        const old = entry.entity;
        if (replace || !old?.isValid) {
          const entity = this.dimension.spawnEntity(ENTITY_TYPE, pose.worldAnchor);
          try {
            entity.setDynamicProperty(CELL_PROPERTY, cell.id);
            entry.entity = entity;
            this.configure(entry, pose);
          } catch (error) { entity.remove(); entry.entity = old; throw error; }
          if (old) {
            this.byEntity.delete(old.id);
            entry.pose?.removeModel(old);
            if (old.isValid) old.remove();
          }
        } else {
          // 存档恢复时清除实体上一次会话的插值状态。
          old.setProperty("cutlass:scale", 0);
          old.teleport(pose.worldAnchor, { rotation: { x: 0, y: 0 } });
          old.clearVelocity();
          this.configure(entry, pose);
          old.playAnimation("animation.cutlass.fancy.input", { controller: "cutlass_pose_reset", stopExpression: "v.pose_initialized=0;return 1;" });
        }
        entry.entity.setProperty("cutlass:scale", 0);
        entry.pose = pose;
        entry.generation = pose.anchorRevision;
        entry.dirty = false;
        this.byEntity.set(entry.entity.id, entry);
        pose.addModel(entry.entity);
        added.push({ entry, entity: entry.entity });
      }
      this.equip(added);
      state.dirty = false;
      state.generation = pose.anchorRevision;
    } catch (error) {
      state.dirty = true;
      for (const { entry } of added) entry.dirty = true;
      throw error;
    } finally {
      if (added.length) pose.deferAnchorCommit(() => {
        if (!record.body.isValid) return;
        if (state.dirty) return false;
        pose.sampleForRender(true);
        pose.publish(true);
        if (!pose.isPublished) return false;
        for (const { entry, entity } of added) {
          if (entry.entity === entity && !entry.dirty && entity.isValid) entity.setProperty("cutlass:scale", 1);
        }
      });
    }
    if (added.length) pose.refreshAfterSpawn(() => record.body.isValid);
    pose.publish();
  }

  removeCell(id) {
    const entry = this.entries.get(id);
    this.entries.delete(id);
    if (entry?.entity) {
      entry.pose?.removeModel(entry.entity);
      this.byEntity.delete(entry.entity.id);
      if (entry.entity.isValid) entry.entity.remove();
    }
  }

  remove(record) {
    for (const cell of record.data.cells) this.removeCell(cell.id);
  }
}
