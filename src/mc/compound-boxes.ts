import { BoxGeometry } from "../core/collision/geometry/box-geometry.js";
import { ShapeConfig } from "../core/dynamics/rigidbody/shape-config.js";
import { Shape } from "../core/dynamics/rigidbody/shape.js";
import type { BlockPhysics } from "./block-physics.js";
import type { BodyBlock } from "./shapes.js";

interface Cell { x: number; y: number; z: number; shape: Shape }
const key = (x: number, y: number, z: number) => `${x},${y},${z}`;

/** 连续同材质完整方块按 x、z、y 合并；易碎和悬浮方块保留独立形状。 */
export function mergeBlockShapes(shapes: Shape[]): Shape[] {
  if (shapes.length < 2) return shapes;
  const groups = new Map<string, Map<string, Cell>>();
  for (const shape of shapes) {
    const info = shape.userData as { block: BodyBlock; physics: BlockPhysics } | null;
    const g = shape.geom, t = shape.localTransform;
    if (!info || info.physics.fragileImpactSpeed !== undefined || info.physics.floatingMaterial !== undefined ||
        !(g instanceof BoxGeometry) || g.hx !== 0.5 || g.hy !== 0.5 || g.hz !== 0.5 ||
        t.r00 !== 1 || t.r11 !== 1 || t.r22 !== 1 || t.r01 !== 0 || t.r02 !== 0 || t.r10 !== 0 || t.r12 !== 0 || t.r20 !== 0 || t.r21 !== 0) continue;
    const { x, y, z } = info.block.location;
    if (!Number.isSafeInteger(x) || !Number.isSafeInteger(y) || !Number.isSafeInteger(z)) continue;
    const groupKey = `${shape.density},${shape.friction},${shape.restitution},${shape.collisionGroup},${shape.collisionMask}:${(t.px - x).toFixed(9)},${(t.py - y).toFixed(9)},${(t.pz - z).toFixed(9)}`;
    let group = groups.get(groupKey);
    if (!group) { group = new Map(); groups.set(groupKey, group); }
    const cellKey = key(x, y, z);
    // 重叠形状仍各自计入质量与接触。
    if (group.has(cellKey)) continue;
    group.set(cellKey, { x, y, z, shape });
  }
  const replaced = new Map<Shape, Shape | null>();
  for (const group of groups.values()) {
    if (group.size < 2) continue;
    const cells = [...group.values()].sort((a, b) => a.y - b.y || a.z - b.z || a.x - b.x);
    for (const cell of cells) {
      const { x, y, z, shape } = cell;
      if (!group.has(key(x, y, z))) continue;
      let nx = 1, nz = 1, ny = 1;
      while (group.has(key(x + nx, y, z))) nx++;
      zScan: for (;;) {
        for (let dx = 0; dx < nx; dx++) if (!group.has(key(x + dx, y, z + nz))) break zScan;
        nz++;
      }
      yScan: for (;;) {
        for (let dz = 0; dz < nz; dz++) for (let dx = 0; dx < nx; dx++) if (!group.has(key(x + dx, y + ny, z + dz))) break yScan;
        ny++;
      }
      if (nx * ny * nz === 1) { group.delete(key(x, y, z)); continue; }
      const config = new ShapeConfig(new BoxGeometry(nx / 2, ny / 2, nz / 2));
      config.transform.copyFrom(shape.localTransform);
      config.transform.px += (nx - 1) / 2; config.transform.py += (ny - 1) / 2; config.transform.pz += (nz - 1) / 2;
      config.density = shape.density; config.friction = shape.friction; config.restitution = shape.restitution;
      config.collisionGroup = shape.collisionGroup; config.collisionMask = shape.collisionMask;
      for (let dy = 0; dy < ny; dy++) for (let dz = 0; dz < nz; dz++) for (let dx = 0; dx < nx; dx++) {
        const k = key(x + dx, y + dy, z + dz);
        replaced.set(group.get(k)!.shape, null);
        group.delete(k);
      }
      replaced.set(shape, new Shape(config));
    }
  }
  if (replaced.size === 0) return shapes;
  const result: Shape[] = [];
  for (const shape of shapes) {
    const replacement = replaced.get(shape);
    if (replacement === null) continue;
    result.push(replacement ?? shape);
  }
  return result;
}
