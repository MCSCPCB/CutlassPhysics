const OFFSETS = [
  [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1],
  [1, 1, 0], [-1, -1, 0], [1, -1, 0], [-1, 1, 0],
  [1, 0, 1], [-1, 0, -1], [1, 0, -1], [-1, 0, 1],
  [0, 1, 1], [0, -1, -1], [0, 1, -1], [0, -1, 1],
];

export function cellKey(location) {
  return `${location.x},${location.y},${location.z}`;
}

// 共面或共边的方块相连，仅共顶点的方块不相连。
export function connectedComponents(cells) {
  const remaining = new Map(cells.map(cell => [cellKey(cell.location), cell]));
  const groups = [];
  for (const [key, cell] of remaining) {
    const group = [cell];
    remaining.delete(key);
    for (let index = 0; index < group.length; index++) {
      const { x, y, z } = group[index].location;
      for (const [dx, dy, dz] of OFFSETS) {
        const nextKey = `${x + dx},${y + dy},${z + dz}`;
        const next = remaining.get(nextKey);
        if (!next) continue;
        remaining.delete(nextKey);
        group.push(next);
      }
    }
    groups.push(group);
  }
  return groups;
}
