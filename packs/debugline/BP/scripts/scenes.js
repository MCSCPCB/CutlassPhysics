import { blockShapes } from "./physics.js";

const BOX = (x, y, z) => ({ type: "box", size: { x, y, z } });

// 场景坐标随玩家的水平方向旋转。
function place(ctx, right, up, ahead) {
  const f = ctx.forward;
  const rx = -f.z, rz = f.x;
  return {
    x: ctx.origin.x + rx * right + f.x * ahead,
    y: ctx.origin.y + up,
    z: ctx.origin.z + rz * right + f.z * ahead,
  };
}

const drop = (ctx) => {
  ctx.add([BOX(1, 1, 1)], { location: place(ctx, -2, 8, 4) });
  ctx.add([{ type: "sphere", radius: 0.5 }], { location: place(ctx, 0, 8, 4) });
  ctx.add([{ type: "capsule", radius: 0.3, height: 1.6 }], { location: place(ctx, 2, 8, 4) });
};

const stack = (ctx) => {
  for (let i = 0; i < 10; i++) {
    ctx.add([BOX(1, 1, 1)], { location: place(ctx, 0, 0.5 + i * 1.01, 4) });
  }
};

const wall = (ctx) => {
  for (let row = 0; row < 5; row++) {
    for (let col = 0; col < 5; col++) {
      const shift = row % 2 === 0 ? 0 : 0.5;
      ctx.add([BOX(1, 0.5, 0.5)], { location: place(ctx, (col - 2) * 1.02 + shift, 0.25 + row * 0.51, 6) });
    }
  }
  const f = ctx.forward;
  ctx.add([{ type: "sphere", radius: 0.6, density: 4 }], {
    location: place(ctx, 0, 1.5, 0), linearVelocity: { x: f.x * 16, y: 2, z: f.z * 16 }, color: "special",
  });
};

const bounce = (ctx) => {
  [0, 0.3, 0.6, 0.9].forEach((restitution, i) => {
    ctx.add([{ type: "sphere", radius: 0.4, restitution }], { location: place(ctx, (i - 1.5) * 1.5, 6, 4) });
  });
};

const chain = (ctx) => {
  const top = 7, links = 6, length = 0.8;
  let prev = ctx.add([BOX(0.4, 0.4, 0.4)], { location: place(ctx, 0, top, 4), type: "static" });
  for (let i = 1; i <= links; i++) {
    const link = ctx.add([BOX(length, 0.3, 0.3)], { location: place(ctx, (i - 0.5) * length, top, 4), color: "joint" });
    ctx.world.createJoint({ type: "ball", bodyA: prev, bodyB: link, anchor: place(ctx, (i - 1) * length, top, 4) });
    prev = link;
  }
};

const hinge = (ctx) => {
  const post = ctx.add([BOX(0.3, 3, 0.3)], { location: place(ctx, 0, 1.5, 4), type: "static" });
  const door = ctx.add([BOX(2, 2.6, 0.2)], { location: place(ctx, 1.15, 1.5, 4), color: "joint" });
  ctx.world.createJoint({
    type: "hinge", bodyA: post, bodyB: door, anchor: place(ctx, 0.15, 1.5, 4), axis: { x: 0, y: 1, z: 0 },
    limit: { lower: -100, upper: 100, motorSpeed: 90, motorMaxTorque: 400 },
  });
};

const slope = (ctx) => {
  const f = ctx.forward;
  const yaw = Math.atan2(f.x, f.z), tilt = 25 * Math.PI / 180;
  const sy = Math.sin(yaw / 2), cy = Math.cos(yaw / 2), sz = Math.sin(tilt / 2), cz = Math.cos(tilt / 2);
  const rotation = { x: sy * sz, y: sy * cz, z: cy * sz, w: cy * cz };
  const ramp = ctx.add([BOX(8, 0.3, 4)], { location: place(ctx, 0, 3, 5), type: "static" });
  ramp.rotation = rotation;
  [0.05, 0.2, 0.5, 1].forEach((friction, i) => {
    const box = ctx.add([{ type: "box", size: { x: 0.8, y: 0.8, z: 0.8 }, friction }], {
      location: ramp.localToWorld({ x: 0, y: 0.15 + 0.4 + 0.02, z: (i - 1.5) * 0.9 }),
    });
    box.rotation = rotation;
  });
};

const shapes = (ctx) => {
  ctx.add([{ type: "cylinder", radius: 0.5, height: 1 }], { location: place(ctx, -3, 7, 4) });
  ctx.add([{ type: "cone", radius: 0.5, height: 1.2 }], { location: place(ctx, -1.5, 7, 4) });
  ctx.add([{
    type: "convexHull",
    points: [
      { x: 0, y: 0.6, z: 0 }, { x: 0.6, y: -0.4, z: 0.6 }, { x: -0.6, y: -0.4, z: 0.6 },
      { x: -0.6, y: -0.4, z: -0.6 }, { x: 0.6, y: -0.4, z: -0.6 },
    ],
  }], { location: place(ctx, 0, 7, 4) });
  ctx.add([BOX(1.6, 0.4, 0.4), { type: "box", size: { x: 0.4, y: 1.2, z: 0.4 }, offset: { x: 0, y: -0.8, z: 0 } }], {
    location: place(ctx, 1.8, 7, 4),
  });
  ctx.add([{ type: "box", size: { x: 0.5, y: 0.5, z: 0.5 }, offset: { x: 0.4, y: 0, z: 0 } }, { type: "sphere", radius: 0.35, offset: { x: -0.4, y: 0, z: 0 } }], {
    location: place(ctx, 3.3, 7, 4),
  });
};

const platform = (ctx) => {
  const f = ctx.forward;
  ctx.add([BOX(4, 0.4, 4)], {
    location: place(ctx, 0, 0.6, 5), type: "kinematic", linearVelocity: { x: -f.z * 1.5, y: 0, z: f.x * 1.5 },
  });
  ctx.add([BOX(1, 1, 1)], { location: place(ctx, 0, 1.5, 5) });
};

const swarm = (ctx) => {
  for (let i = 0; i < 60; i++) {
    const x = (i % 5 - 2) * 1.1, z = (Math.floor(i / 5) % 4 - 1.5) * 1.1, y = 4 + Math.floor(i / 20) * 1.2;
    const shape = i % 2 === 0 ? { type: "sphere", radius: 0.4 } : BOX(0.7, 0.7, 0.7);
    ctx.add([shape], { location: place(ctx, x, y, 5 + z) });
  }
};

const beam = (ctx) => {
  const b = ctx.add([BOX(24, 0.8, 0.8)], {
    location: place(ctx, 0, 4, 8), angularVelocity: { x: 0, y: 0.2, z: 0 },
  });
  b.rotation = { x: 0, y: Math.sin(Math.PI / 8), z: 0, w: Math.cos(Math.PI / 8) };
};

const blocks = (ctx) => {
  const cube = (typeId, n) => {
    const out = [];
    for (let x = 0; x < n; x++)
      for (let y = 0; y < n; y++)
        for (let z = 0; z < n; z++)
          out.push({ location: { x, y, z }, typeId });
    return out;
  };
  const drop = (list, right, up, color) => {
    // 方块形状以质心为原点。
    const { shapes, centerOfMass } = blockShapes(list);
    const at = place(ctx, right, up, 5);
    return ctx.add(shapes, { location: { x: at.x + centerOfMass.x, y: at.y + centerOfMass.y, z: at.z + centerOfMass.z }, color });
  };
  drop(cube("minecraft:iron_block", 2), -3, 6);
  drop(cube("minecraft:white_wool", 2), 0, 6);
  const mixed = cube("minecraft:stone", 2).map((b) => b.location.x === 1 ? { ...b, typeId: "minecraft:end_stone" } : b);
  drop(mixed, 3, 6, "special");
};

const fragile = (ctx) => {
  const f = ctx.forward;
  const speed = { x: f.x * 12, y: 0, z: f.z * 12 };
  ctx.add([{ type: "box", size: { x: 0.6, y: 0.6, z: 0.6 }, density: 4 }], { location: place(ctx, -1.5, 1, 0), linearVelocity: speed, color: "special" });
  ctx.add([{ type: "box", size: { x: 0.6, y: 0.6, z: 0.6 }, density: 4 }], { location: place(ctx, 1.5, 1, 0), linearVelocity: speed, color: "special" });
};

const water = (ctx) => {
  [0.3, 0.9, 3].forEach((density, i) => {
    ctx.add([{ type: "box", size: { x: 0.8, y: 0.8, z: 0.8 }, density }], { location: place(ctx, (i - 1) * 2, 6, 4) });
  });
};

export const SCENES = {
  drop, stack, wall, bounce, chain, hinge, slope, shapes, platform, swarm, beam, blocks, fragile, water,
};

export const SCENE_NAMES = Object.keys(SCENES);
