import { blockShapes } from "./physics.js";

export function pickStructure(player, session) {
  const origin = player.getHeadLocation(), direction = player.getViewDirection();
  const hit = session.physics.rayCast(origin, direction, { maxDistance: 6, includeBlocks: false, collisionMask: 1 });
  const record = hit?.body && session.structures.records.get(hit.body);
  if (!record) return;
  const terrain = player.dimension.getBlockFromRay(origin, direction, { maxDistance: 6, includeLiquidBlocks: false, includePassableBlocks: false });
  if (terrain) {
    const p = terrain.block.location, f = terrain.faceLocation;
    if (Math.hypot(p.x + f.x - origin.x, p.y + f.y - origin.y, p.z + f.z - origin.z) + 0.01 < hit.distance) return;
  }
  return { ...hit, record };
}

export function hitCell(hit) {
  const { record } = hit;
  const point = hit.body.worldToLocal({
    x: hit.point.x - hit.normal.x * 1e-5,
    y: hit.point.y - hit.normal.y * 1e-5,
    z: hit.point.z - hit.normal.z * 1e-5,
  });
  record.pickShapes ??= blockShapes(record.data.cells).shapes;
  for (const shape of record.pickShapes) {
    const o = shape.offset, h = shape.size;
    if (Math.abs(point.x - o.x) > h.x / 2 + 1e-6 || Math.abs(point.y - o.y) > h.y / 2 + 1e-6 || Math.abs(point.z - o.z) > h.z / 2 + 1e-6) continue;
    const p = shape.block.location;
    return record.data.cells.find(cell => cell.location.x === p.x && cell.location.y === p.y && cell.location.z === p.z);
  }
}
