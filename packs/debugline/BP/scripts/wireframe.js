export const CIRCLE_SEGMENTS = 12;

const IDENTITY = { x: 0, y: 0, z: 0, w: 1 };

export function rotate(q, v) {
  const cx = q.y * v.z - q.z * v.y, cy = q.z * v.x - q.x * v.z, cz = q.x * v.y - q.y * v.x;
  return {
    x: v.x + 2 * (q.w * cx + q.y * cz - q.z * cy),
    y: v.y + 2 * (q.w * cy + q.z * cx - q.x * cz),
    z: v.z + 2 * (q.w * cz + q.x * cy - q.y * cx),
  };
}

function makeToWorld(location, rotation) {
  return (x, y, z) => {
    const r = rotate(rotation, { x, y, z });
    return { x: r.x + location.x, y: r.y + location.y, z: r.z + location.z };
  };
}

function segment(out, a, b) {
  out.push(a.x, a.y, a.z, b.x, b.y, b.z);
}

function ring(out, toWorld, r, y) {
  for (let i = 0; i < CIRCLE_SEGMENTS; i++) {
    const a0 = (i / CIRCLE_SEGMENTS) * Math.PI * 2, a1 = ((i + 1) / CIRCLE_SEGMENTS) * Math.PI * 2;
    segment(out, toWorld(Math.cos(a0) * r, y, Math.sin(a0) * r), toWorld(Math.cos(a1) * r, y, Math.sin(a1) * r));
  }
}

function arc(out, toWorld, r, cy, sign, alongX) {
  const n = CIRCLE_SEGMENTS / 2;
  for (let i = 0; i < n; i++) {
    const a0 = (i / n) * Math.PI, a1 = ((i + 1) / n) * Math.PI;
    const p = (a) => {
      const s = Math.cos(a) * r, h = Math.sin(a) * r * sign;
      return alongX ? toWorld(s, cy + h, 0) : toWorld(0, cy + h, s);
    };
    segment(out, p(a0), p(a1));
  }
}

function boxLines(out, toWorld, hx, hy, hz) {
  const c = (sx, sy, sz) => toWorld(sx * hx, sy * hy, sz * hz);
  for (const sy of [-1, 1]) {
    segment(out, c(-1, sy, -1), c(1, sy, -1));
    segment(out, c(1, sy, -1), c(1, sy, 1));
    segment(out, c(1, sy, 1), c(-1, sy, 1));
    segment(out, c(-1, sy, 1), c(-1, sy, -1));
  }
  for (const sx of [-1, 1])
    for (const sz of [-1, 1])
      segment(out, c(sx, -1, sz), c(sx, 1, sz));
}

function sphereLines(out, toWorld, r, cy) {
  for (let i = 0; i < CIRCLE_SEGMENTS; i++) {
    const a0 = (i / CIRCLE_SEGMENTS) * Math.PI * 2, a1 = ((i + 1) / CIRCLE_SEGMENTS) * Math.PI * 2;
    const c0 = Math.cos(a0) * r, s0 = Math.sin(a0) * r, c1 = Math.cos(a1) * r, s1 = Math.sin(a1) * r;
    segment(out, toWorld(c0, cy, s0), toWorld(c1, cy, s1));
    segment(out, toWorld(c0, cy + s0, 0), toWorld(c1, cy + s1, 0));
    segment(out, toWorld(0, cy + c0, s0), toWorld(0, cy + c1, s1));
  }
}

function capsuleLines(out, toWorld, r, height) {
  const hh = Math.max(0, height / 2 - r);
  ring(out, toWorld, r, hh);
  ring(out, toWorld, r, -hh);
  arc(out, toWorld, r, hh, 1, true);
  arc(out, toWorld, r, hh, 1, false);
  arc(out, toWorld, r, -hh, -1, true);
  arc(out, toWorld, r, -hh, -1, false);
  for (const [x, z] of [[r, 0], [-r, 0], [0, r], [0, -r]])
    segment(out, toWorld(x, hh, z), toWorld(x, -hh, z));
}

function cylinderLines(out, toWorld, r, height) {
  const hh = height / 2;
  ring(out, toWorld, r, hh);
  ring(out, toWorld, r, -hh);
  for (const [x, z] of [[r, 0], [-r, 0], [0, r], [0, -r]])
    segment(out, toWorld(x, hh, z), toWorld(x, -hh, z));
}

function coneLines(out, toWorld, r, height) {
  const hh = height / 2;
  ring(out, toWorld, r, -hh);
  const tip = toWorld(0, hh, 0);
  for (const [x, z] of [[r, 0], [-r, 0], [0, r], [0, -r]])
    segment(out, toWorld(x, -hh, z), tip);
}

// 凸包以局部包围盒显示。
function hullLines(out, toWorld, points) {
  if (points.length === 0)
    return;
  let minX = Infinity, minY = Infinity, minZ = Infinity, maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (const p of points) {
    if (p.x < minX)
      minX = p.x;
    if (p.y < minY)
      minY = p.y;
    if (p.z < minZ)
      minZ = p.z;
    if (p.x > maxX)
      maxX = p.x;
    if (p.y > maxY)
      maxY = p.y;
    if (p.z > maxZ)
      maxZ = p.z;
  }
  const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2, cz = (minZ + maxZ) / 2;
  boxLines(out, (x, y, z) => toWorld(x + cx, y + cy, z + cz), (maxX - minX) / 2, (maxY - minY) / 2, (maxZ - minZ) / 2);
}

export function bodyLines(shapes, location, rotation, out = []) {
  for (const shape of shapes) {
    const offset = shape.offset ?? { x: 0, y: 0, z: 0 };
    const local = shape.rotation ?? IDENTITY;
    const world = makeToWorld(location, rotation);
    const toWorld = (x, y, z) => {
      const r = rotate(local, { x, y, z });
      return world(r.x + offset.x, r.y + offset.y, r.z + offset.z);
    };
    switch (shape.type) {
      case "box":
        boxLines(out, toWorld, shape.size.x / 2, shape.size.y / 2, shape.size.z / 2);
        break;
      case "sphere":
        sphereLines(out, toWorld, shape.radius, 0);
        break;
      case "capsule":
        capsuleLines(out, toWorld, shape.radius, shape.height);
        break;
      case "cylinder":
        cylinderLines(out, toWorld, shape.radius, shape.height);
        break;
      case "cone":
        coneLines(out, toWorld, shape.radius, shape.height);
        break;
      case "convexHull":
        hullLines(out, toWorld, shape.points);
        break;
    }
  }
  return out;
}
