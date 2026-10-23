const REACH = 6;
const BREAK_DISTANCE = 14;

export class DragController {
  constructor() {
    this.players = new Map();
  }

  release(playerId) {
    const drag = this.players.get(playerId);
    if (!drag) return;
    this.players.delete(playerId);
    if (drag.anchor.isValid) drag.anchor.remove();
  }

  toggle(player, session) {
    if (this.players.has(player.id)) { this.release(player.id); return; }
    const origin = player.getHeadLocation(), direction = player.getViewDirection();
    const hit = session.physics.rayCast(origin, direction, { maxDistance: REACH, includeBlocks: false, collisionMask: 1 });
    if (!hit?.body || !session.structures.records.has(hit.body)) return;
    const terrain = player.dimension.getBlockFromRay(origin, direction, { maxDistance: REACH, includeLiquidBlocks: false, includePassableBlocks: false });
    if (terrain) {
      const location = terrain.block.location, face = terrain.faceLocation;
      const distance = Math.hypot(location.x + face.x - origin.x, location.y + face.y - origin.y, location.z + face.z - origin.z);
      if (distance + 0.01 < hit.distance) return;
    }
    const anchor = session.physics.createBody({
      type: "kinematic", location: hit.point, gravityScale: 0,
      shapes: [{ type: "sphere", radius: 0.01, collisionGroup: 0, collisionMask: 0 }],
      userData: { kind: "grab" },
    });
    try {
      const joint = session.physics.createJoint({
        type: "ball", bodyA: hit.body, bodyB: anchor, anchor: hit.point,
        spring: { frequency: 4, dampingRatio: 1 },
      });
      this.players.set(player.id, {
        player, body: hit.body, anchor, joint, dimensionId: player.dimension.id,
        localPoint: hit.body.worldToLocal(hit.point), distance: Math.max(2, Math.min(8, hit.distance)),
      });
      hit.body.wakeUp();
    } catch (error) { this.players.delete(player.id); anchor.remove(); throw error; }
  }

  update() {
    for (const [id, drag] of this.players) {
      const { player, body, anchor, joint } = drag;
      if (!player.isValid || player.dimension.id !== drag.dimensionId || !body.isValid || !anchor.isValid || !joint.isValid || heldItem(player) !== "minecraft:slime_ball") {
        this.release(id);
        continue;
      }
      const head = player.getHeadLocation(), direction = player.getViewDirection();
      const point = body.localToWorld(drag.localPoint);
      if (Math.hypot(point.x - head.x, point.y - head.y, point.z - head.z) > BREAK_DISTANCE) {
        this.release(id);
        continue;
      }
      anchor.location = { x: head.x + direction.x * drag.distance, y: head.y + direction.y * drag.distance, z: head.z + direction.z * drag.distance };
      body.wakeUp();
    }
  }
}

export function heldItem(player) {
  return player.getComponent("minecraft:inventory")?.container?.getItem(player.selectedSlotIndex)?.typeId;
}
