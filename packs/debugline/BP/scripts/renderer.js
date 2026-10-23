import { DebugLine, debugDrawer } from "@minecraft/debug-utilities";
import { bodyLines } from "./wireframe.js";

const COLORS = {
  dynamic: { red: 0.2, green: 0.9, blue: 1, alpha: 1 },
  static: { red: 0.8, green: 0.8, blue: 0.8, alpha: 1 },
  kinematic: { red: 1, green: 0.8, blue: 0.2, alpha: 1 },
  joint: { red: 0.7, green: 0.4, blue: 1, alpha: 1 },
  special: { red: 1, green: 0.4, blue: 0.2, alpha: 1 },
};

export class Renderer {
  constructor(dimension) {
    this.dimension = dimension;
    this.items = new Map();
    this.cursor = 0;
  }

  add(body, shapes, color) {
    this.items.set(body, { shapes, color: COLORS[color] ?? COLORS[body.type], lines: [] });
  }

  update() {
    for (const [body, item] of this.items) {
      if (body.isValid) continue;
      for (const line of item.lines) line.remove();
      this.items.delete(body);
    }
    const entries = [...this.items];
    if (!entries.length) return;
    // 超出每 tick 的线段预算时，下次从未处理的刚体继续。
    let budget = 1500;
    let visited = 0;
    const start = this.cursor % entries.length;
    for (; visited < entries.length && budget > 0; visited++) {
      const [body, item] = entries[(start + visited) % entries.length];
      const location = body.location;
      const rotation = body.rotation;
      const pose = [location.x, location.y, location.z, rotation.x, rotation.y, rotation.z, rotation.w];
      if (item.pose?.every((value, i) => value === pose[i])) continue;
      const segments = bodyLines(item.shapes, location, rotation);
      for (let i = 0; i < segments.length; i += 6) {
        const from = { dimension: this.dimension, x: segments[i], y: segments[i + 1], z: segments[i + 2] };
        const to = { x: segments[i + 3], y: segments[i + 4], z: segments[i + 5] };
        let line = item.lines[i / 6];
        if (!line) {
          line = new DebugLine(from, to);
          item.lines.push(line);
          line.color = item.color;
          debugDrawer.addShape(line);
        } else {
          line.setLocation(from);
          line.endLocation = to;
        }
      }
      item.pose = pose;
      budget -= segments.length / 6;
    }
    this.cursor = (start + visited) % entries.length;
  }

  clear() {
    for (const item of this.items.values()) {
      for (const line of item.lines) line.remove();
    }
    this.items.clear();
  }
}
