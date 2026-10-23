import { pickStructure } from "./picking.js";
const BREAK_DISTANCE = 14;

export class DragController {
  constructor() {
    this.players = new Map();
  }

  release(playerId) {
    const drag = this.players.get(playerId);
    if (!drag) return;
    this.players.delete(playerId);
    drag.constraint.remove();
  }

  toggle(player, session) {
    if (this.players.has(player.id)) { this.release(player.id); return; }
    const hit = pickStructure(player, session);
    if (!hit) return;
    for (const [id, drag] of this.players) if (drag.body === hit.body) this.release(id);
    const constraint = hit.body.createDrag({ anchor: hit.point });
    this.players.set(player.id, {
      player, body: hit.body, constraint, dimensionId: player.dimension.id,
      localPoint: hit.body.worldToLocal(hit.point), distance: Math.max(2, Math.min(8, hit.distance)),
      yaw: player.getRotation().y, rotation: hit.body.rotation,
    });
  }

  update() {
    for (const [id, drag] of this.players) {
      const { player, body, constraint } = drag;
      if (!player.isValid || player.dimension.id !== drag.dimensionId || !constraint.isValid || heldItem(player) !== "minecraft:slime_ball") {
        this.release(id);
        continue;
      }
      const head = player.getHeadLocation(), direction = player.getViewDirection();
      const point = body.localToWorld(drag.localPoint);
      if (Math.hypot(point.x - head.x, point.y - head.y, point.z - head.z) > BREAK_DISTANCE) {
        this.release(id);
        continue;
      }
      const halfYaw = -(player.getRotation().y - drag.yaw) * Math.PI / 360;
      const s = Math.sin(halfYaw), c = Math.cos(halfYaw), q = drag.rotation;
      constraint.setTarget(
        { x: head.x + direction.x * drag.distance, y: head.y + direction.y * drag.distance, z: head.z + direction.z * drag.distance },
        { x: c * q.x + s * q.z, y: c * q.y + s * q.w, z: c * q.z - s * q.x, w: c * q.w - s * q.y },
      );
    }
  }
}

export function heldItem(player) {
  return player.getComponent("minecraft:inventory")?.container?.getItem(player.selectedSlotIndex)?.typeId;
}
