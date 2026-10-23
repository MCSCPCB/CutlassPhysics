/*! cutlass-physics | Portions derived from OimoPhysics (c) 2017 saharan | MIT License | See Credits.md */

// src/version.ts
var VERSION = "0.0.0";

// src/mc/math.ts
function eulerToQuaternion(euler) {
  const hy = -euler.y * Math.PI / 360, hx = euler.x * Math.PI / 360, hz = euler.z * Math.PI / 360;
  const sy = Math.sin(hy), cy = Math.cos(hy), sx = Math.sin(hx), cx = Math.cos(hx), sz = Math.sin(hz), cz = Math.cos(hz);
  const w1 = cy * cx, x1 = cy * sx, y1 = sy * cx, z1 = -sy * sx;
  return {
    x: x1 * cz + y1 * sz + 0,
    y: y1 * cz - x1 * sz + 0,
    z: w1 * sz + z1 * cz + 0,
    w: w1 * cz - z1 * sz
  };
}
function quaternionToEuler(rotation) {
  const len = Math.sqrt(rotation.x * rotation.x + rotation.y * rotation.y + rotation.z * rotation.z + rotation.w * rotation.w);
  if (!(len > 0)) throw new Error("\u56DB\u5143\u6570\u7684\u6240\u6709\u5206\u91CF\u5747\u4E3A\u96F6\uFF0C\u65E0\u6CD5\u8F6C\u6362");
  const x = rotation.x / len, y = rotation.y / len, z = rotation.z / len, w = rotation.w / len;
  const r00 = 1 - 2 * (y * y + z * z), r02 = 2 * (x * z + w * y), r10 = 2 * (x * y + w * z);
  const r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x), r20 = 2 * (x * z - w * y), r22 = 1 - 2 * (x * x + y * y);
  const RAD = 180 / Math.PI;
  const sinPitch = Math.max(-1, Math.min(1, -r12));
  const pitch = Math.asin(sinPitch) * RAD + 0;
  if (Math.abs(sinPitch) > 0.9999999) {
    return { x: pitch, y: -Math.atan2(-r20, r00) * RAD + 0, z: 0 };
  }
  return { x: pitch, y: -Math.atan2(r02, r22) * RAD + 0, z: Math.atan2(r10, r11) * RAD + 0 };
}

// src/world/block-shapes.ts
var SHAPE_NONE = 0;
var SHAPE_FULL_CUBE = 1;
var SHAPE_UNLOADED = -1;
var MATERIAL_SHIFT = 16;
var SHAPE_MASK = (1 << MATERIAL_SHIFT) - 1;
var MAX_MATERIALS = 255;
function packBlock(shapeId, materialId) {
  return shapeId | materialId << MATERIAL_SHIFT;
}
function shapeOf(blockId) {
  return blockId & SHAPE_MASK;
}
function materialOf(blockId) {
  return blockId >> MATERIAL_SHIFT;
}
var TYPE_AIR = 0;
var TYPE_RESOLVE = -1;
var FLUID_NONE = 0;
var FLUID_WATER = 1;
var FLUID_LAVA = 2;
function fluidKind(code) {
  return code >> 5;
}
function fluidHeight(code) {
  return (code & 31) / 16;
}
var BlockShapeTable = class {
  constructor() {
    this.boxes = /* @__PURE__ */ new Map();
  }
  register(id, boxes) {
    if (!Number.isInteger(id) || id <= SHAPE_FULL_CUBE || id > SHAPE_MASK) {
      throw new Error(`\u5F62\u72B6\u7F16\u53F7\u5FC5\u987B\u662F\u5927\u4E8E ${SHAPE_FULL_CUBE}\u3001\u4E0D\u8D85\u8FC7 ${SHAPE_MASK} \u7684\u6574\u6570\uFF0C\u6536\u5230 ${id}`);
    }
    if (this.boxes.has(id)) throw new Error(`\u5F62\u72B6\u7F16\u53F7 ${id} \u5DF2\u7ECF\u767B\u8BB0\u8FC7`);
    if (boxes.length === 0) throw new Error(`\u5F62\u72B6 ${id} \u81F3\u5C11\u8981\u6709\u4E00\u4E2A\u76D2\u5B50`);
    for (const b of boxes) {
      if (!(b[3] > b[0] && b[4] > b[1] && b[5] > b[2])) throw new Error(`\u5F62\u72B6 ${id} \u7684\u76D2\u5B50\u5FC5\u987B\u6709\u4F53\u79EF\uFF1A[${b.join(", ")}]`);
    }
    this.boxes.set(id, boxes.map((b) => [...b]));
  }
  /** 返回已登记的盒子；编号未登记时返回 undefined */
  get(id) {
    return this.boxes.get(id);
  }
};

// src/mc/data/block-collision.ts
var SHAPES = [
  [],
  [0, 0, 0, 1, 1, 1],
  [0.1, 0, 0.1, 0.9, 0.8, 0.9],
  [0.19166666, 0, 0.25833333, 0.44166666, 1, 0.5083333],
  [0.19166666, 0.75, 0.25833333, 0.44166666, 1, 0.5083333],
  [0.19166666, 0.625, 0.25833333, 0.44166666, 1, 0.5083333],
  [0.19166666, 0.4375, 0.25833333, 0.44166666, 1, 0.5083333],
  [0.19166666, 0.1875, 0.25833333, 0.44166666, 1, 0.5083333],
  [0, 0, 0, 1, 0.5625, 1],
  [0, 0, 0, 1, 0.0625, 1],
  [0, 0, 0, 1, 0.5, 1],
  [-0.083333336, -0.040000007, -0.016666658, 0.71666664, 0.76, 0.7833333],
  [0.1, 0, 0.1, 0.9, 1, 0.9],
  [0, 0, 0, 1, 0.8, 1],
  [-0.058333337, -0.040000007, 833334e-8, 0.69166666, 0.585, 0.7583333],
  [-0.12083334, -0.040000007, -0.05416666, 0.75416666, 0.96, 0.8208333],
  [-0.03333333, 0, 0.033333346, 0.51666665, 0.6, 0.5833333],
  [0.3, 0, 0.3, 0.7, 0.4, 0.7],
  [0, 0, 0, 1, 1, 0.3125],
  [0.6875, 0, 0, 1, 1, 1],
  [0, 0, 0.6875, 1, 1, 1],
  [0, 0, 0, 0.3125, 1, 1],
  [0.4, 0, 0.4, 0.6, 0.6, 0.6],
  [0, 0.2, 0.35, 0.3, 0.8, 0.65],
  [0.7, 0.2, 0.35, 1, 0.8, 0.65],
  [0.35, 0.2, 0, 0.65, 0.8, 0.3],
  [0.35, 0.2, 0.7, 0.65, 0.8, 1],
  [0, 0, 0, 1, 0.5, 1, 0.5, 0.5, 0, 1, 1, 1],
  [0, 0, 0, 1, 0.5, 1, 0, 0.5, 0, 0.5, 1, 1],
  [0, 0, 0, 1, 0.5, 1, 0, 0.5, 0.5, 1, 1, 1],
  [0, 0, 0, 1, 0.5, 1, 0, 0.5, 0, 1, 1, 0.5],
  [0, 0.5, 0, 1, 1, 1, 0.5, 0, 0, 1, 0.5, 1],
  [0, 0.5, 0, 1, 1, 1, 0, 0, 0, 0.5, 0.5, 1],
  [0, 0.5, 0, 1, 1, 1, 0, 0, 0.5, 1, 0.5, 1],
  [0, 0.5, 0, 1, 1, 1, 0, 0, 0, 1, 0.5, 0.5],
  [0.025, 0, 0.025, 0.975, 0.95000005, 0.975],
  [0, 0, 0, 1, 0.14285715, 1],
  [0, 0, 0, 1, 0.2857143, 1],
  [0, 0, 0, 1, 0.42857143, 1],
  [0, 0, 0, 1, 0.5714286, 1],
  [0, 0, 0, 1, 0.71428573, 1],
  [0, 0, 0, 1, 0.85714287, 1],
  [0, 0, 0, 1, 1.1428572, 1],
  [0, 0, 0, 1, 0.9375, 1],
  [0.25, 0, 0.25, 0.75, 1, 0.75],
  [0, 0, 0, 0.1875, 1, 1],
  [0, 0, 0.8125, 1, 1, 1],
  [0, 0, 0, 1, 1, 0.1875],
  [0.8125, 0, 0, 1, 1, 1],
  [0, 0, 0, 0.1875, 1, 1],
  [0, 0.28125, 0.875, 1, 0.78125, 1],
  [0, 0.28125, 0, 1, 0.78125, 0.125],
  [0.875, 0.28125, 0, 1, 0.78125, 1],
  [0, 0.28125, 0, 0.125, 0.78125, 1],
  [0.375, 0.875, 0, 0.625, 1, 1],
  [0, 0, 0.375, 1, 1, 0.625],
  [0, 0.875, 0.375, 1, 1, 0.625],
  [0.375, 0, 0, 0.625, 1, 1],
  [0.25, 0.4, 0.25, 0.75, 1, 0.75],
  [0, 0.25, 0.3125, 0.375, 0.75, 0.6875],
  [0.625, 0.25, 0.3125, 1, 0.75, 0.6875],
  [0.3125, 0.25, 0, 0.6875, 0.75, 0.375],
  [0.3125, 0.25, 0.625, 0.6875, 0.75, 1],
  [0.25, 0, 0.25, 0.75, 0.6, 0.75],
  [0.0625, 0, 0.0625, 0.9375, 0.0625, 0.9375],
  [0.0625, 0, 0.0625, 0.9375, 0.03125, 0.9375],
  [0.3125, 0.875, 0.375, 0.6875, 1, 0.625],
  [0.3125, 0, 0.375, 0.6875, 0.125, 0.625],
  [0.3125, 0.375, 0.875, 0.6875, 0.625, 1],
  [0.3125, 0.375, 0, 0.6875, 0.625, 0.125],
  [0.875, 0.375, 0.3125, 1, 0.625, 0.6875],
  [0, 0.375, 0.3125, 0.125, 0.625, 0.6875],
  [0.3125, 0.9375, 0.375, 0.6875, 1, 0.625],
  [0.3125, 0, 0.375, 0.6875, 0.0625, 0.625],
  [0.3125, 0.375, 0.9375, 0.6875, 0.625, 1],
  [0.3125, 0.375, 0, 0.6875, 0.625, 0.0625],
  [0.9375, 0.375, 0.3125, 1, 0.625, 0.6875],
  [0, 0.375, 0.3125, 0.0625, 0.625, 0.6875],
  [0, 0, 0, 1, 0.125, 1],
  [0, 0, 0, 1, 0.25, 1],
  [0, 0, 0, 1, 0.375, 1],
  [0, 0, 0, 1, 0.625, 1],
  [0, 0, 0, 1, 0.75, 1],
  [0, 0, 0, 1, 0.875, 1],
  [0.0625, 0, 0.0625, 0.9375, 1, 0.9375],
  [0.0625, 0, 0.0625, 0.9375, 0.875, 0.9375],
  [0.125, 0, 0.125, 0.875, 1, 0.875],
  [0.375, 0, 0.375, 0.625, 1.5, 0.625],
  [0.375, 0, 0.375, 0.625, 1, 0.625],
  [0.0625, 0, 0.0625, 0.9375, 0.5, 0.9375],
  [0.1875, 0, 0.0625, 0.9375, 0.5, 0.9375],
  [0.3125, 0, 0.0625, 0.9375, 0.5, 0.9375],
  [0.4375, 0, 0.0625, 0.9375, 0.5, 0.9375],
  [0.5625, 0, 0.0625, 0.9375, 0.5, 0.9375],
  [0.6875, 0, 0.0625, 0.9375, 0.5, 0.9375],
  [0.8125, 0, 0.0625, 0.9375, 0.5, 0.9375],
  [0, 0, 0, 1, 0.1875, 1],
  [5e-4, 5e-4, 5e-4, 0.9995, 0.1875, 0.9995],
  [0, 0.8125, 0, 1, 1, 1],
  [5e-4, 0.8125, 5e-4, 0.9995, 0.9995, 0.9995],
  [5e-4, 5e-4, 5e-4, 0.1875, 0.9995, 0.9995],
  [0.8125, 0, 0, 1, 1, 1],
  [0.8125, 5e-4, 5e-4, 0.9995, 0.9995, 0.9995],
  [0, 0, 0, 1, 1, 0.1875],
  [5e-4, 5e-4, 5e-4, 0.9995, 0.9995, 0.1875],
  [0, 0, 0.8125, 1, 1, 1],
  [5e-4, 5e-4, 0.8125, 0.9995, 0.9995, 0.9995],
  [0.4375, 0, 0.4375, 0.5625, 1, 0.5625],
  [0.40625, 0, 0.40625, 0.59375, 1, 0.59375],
  [0, 0.40625, 0.40625, 1, 0.59375, 0.59375],
  [0.40625, 0.40625, 0, 0.59375, 0.59375, 1],
  [0.375, 0, 0.375, 0.625, 0.125, 0.625],
  [0.375, 0, 0.375, 0.625, 0.25, 0.625],
  [0.375, 0, 0.375, 0.625, 0.375, 0.625],
  [0.375, 0, 0.375, 0.625, 0.5, 0.625],
  [0.375, 0, 0.375, 0.625, 0.625, 0.625],
  [0.375, 0, 0.375, 0.625, 0.75, 0.625],
  [0.375, 0, 0.375, 0.625, 0.875, 0.625],
  [0.375, 0, 0.375, 0.625, 0.5625, 0.625],
  [0.375, 0, 0, 0.625, 0.5625, 0.625],
  [0.375, 0, 0.375, 0.625, 0.5625, 1],
  [0, 0, 0.375, 0.625, 0.5625, 0.625],
  [0.375, 0, 0.375, 1, 0.5625, 0.625],
  [0, 0, 0.9375, 1, 1, 1],
  [0, 0, 0, 0.0625, 1, 1],
  [0, 0, 0, 1, 1, 0.0625],
  [0.9375, 0, 0, 1, 1, 1],
  [0, 0.9375, 0, 1, 1, 1],
  [0, 0, 0.375, 1, 1.5, 0.625],
  [0.375, 0, 0, 0.625, 1.5, 1],
  [0.0625, 0, 0.0625, 0.9375, 0.09375, 0.9375],
  [0, 0.5, 0, 1, 1, 1],
  [0.25, 0, 0.25, 0.75, 1.5, 0.75],
  [0.25, 0, 0.25, 0.75, 0.8125, 0.75],
  [0.25, 0, 0.25, 0.75, 0.9999, 0.75],
  [0.25, 0, 0, 0.75, 1.5, 0.75],
  [0.25, 0, 0, 0.75, 0.8125, 0.75],
  [0.25, 0, 0, 0.75, 0.9999, 0.75],
  [0.25, 0, 0.25, 1, 1.5, 0.75],
  [0.25, 0, 0.25, 1, 0.8125, 0.75],
  [0.25, 0, 0.25, 1, 0.9999, 0.75],
  [0.25, 0, 0, 1, 1.5, 0.75],
  [0.25, 0, 0, 1, 0.8125, 0.75],
  [0.25, 0, 0, 1, 0.9999, 0.75],
  [0.25, 0, 0.25, 0.75, 1.5, 1],
  [0.25, 0, 0.25, 0.75, 0.8125, 1],
  [0.25, 0, 0.25, 0.75, 0.9999, 1],
  [0.3125, 0, 0, 0.6875, 1.5, 1],
  [0.3125, 0, 0, 0.6875, 0.8125, 1],
  [0.25, 0, 0, 0.75, 1.5, 1],
  [0.25, 0, 0, 0.75, 0.9999, 1],
  [0.3125, 0, 0, 0.6875, 0.9999, 1],
  [0.25, 0, 0.25, 1, 1.5, 1],
  [0.25, 0, 0.25, 1, 0.8125, 1],
  [0.25, 0, 0.25, 1, 0.9999, 1],
  [0.25, 0, 0, 1, 1.5, 1],
  [0.25, 0, 0, 1, 0.8125, 1],
  [0.25, 0, 0, 1, 0.9999, 1],
  [0, 0, 0.25, 0.75, 1.5, 0.75],
  [0, 0, 0.25, 0.75, 0.8125, 0.75],
  [0, 0, 0.25, 0.75, 0.9999, 0.75],
  [0, 0, 0, 0.75, 1.5, 0.75],
  [0, 0, 0, 0.75, 0.8125, 0.75],
  [0, 0, 0, 0.75, 0.9999, 0.75],
  [0, 0, 0.3125, 1, 1.5, 0.6875],
  [0, 0, 0.3125, 1, 0.8125, 0.6875],
  [0, 0, 0.25, 1, 1.5, 0.75],
  [0, 0, 0.25, 1, 0.9999, 0.75],
  [0, 0, 0, 1, 1.5, 0.75],
  [0, 0, 0, 1, 0.8125, 0.75],
  [0, 0, 0, 1, 0.9999, 0.75],
  [0, 0, 0.3125, 1, 0.9999, 0.6875],
  [0, 0, 0.25, 0.75, 1.5, 1],
  [0, 0, 0.25, 0.75, 0.8125, 1],
  [0, 0, 0.25, 0.75, 0.9999, 1],
  [0, 0, 0, 0.75, 1.5, 1],
  [0, 0, 0, 0.75, 0.8125, 1],
  [0, 0, 0, 0.75, 0.9999, 1],
  [0, 0, 0.25, 1, 1.5, 1],
  [0, 0, 0.25, 1, 0.8125, 1],
  [0, 0, 0.25, 1, 0.9999, 1],
  [0, 0, 0, 1, 1.5, 1],
  [0, 0, 0, 1, 0.8125, 1],
  [0, 0, 0, 1, 0.9999, 1],
  [0.4375, 0, 0.4375, 0.5625, 0.875, 0.5625, 0, 0, 0, 1, 0.125, 1],
  [0.125, 0, 0.125, 0.875, 0.875, 0.875],
  [0, 0, 0, 1, 0.3125, 1, 0, 0, 0, 0.125, 1, 1, 0, 0, 0, 1, 1, 0.125, 0.875, 0, 0, 1, 1, 1, 0, 0, 0.875, 1, 1, 1],
  [0, 0, 0, 1, 0.8125, 1, 0.3125, 0.8125, 0.3125, 0.6875, 1, 0.6875],
  [0.375, 0.4375, 0.6875, 0.625, 0.75, 0.9375],
  [0.0625, 0.4375, 0.375, 0.3125, 0.75, 0.625],
  [0.375, 0.4375, 0.0625, 0.625, 0.75, 0.3125],
  [0.6875, 0.4375, 0.375, 0.9375, 0.75, 0.625],
  [0.3125, 0.3125, 0.5625, 0.6875, 0.75, 0.9375],
  [0.0625, 0.3125, 0.3125, 0.4375, 0.75, 0.6875],
  [0.3125, 0.3125, 0.0625, 0.6875, 0.75, 0.4375],
  [0.5625, 0.3125, 0.3125, 0.9375, 0.75, 0.6875],
  [0.25, 0.1875, 0.4375, 0.75, 0.75, 0.9375],
  [0.0625, 0.1875, 0.25, 0.5625, 0.75, 0.75],
  [0.25, 0.1875, 0.0625, 0.75, 0.75, 0.5625],
  [0.4375, 0.1875, 0.25, 0.9375, 0.75, 0.75],
  [0.375, 0.0625, 0, 0.625, 0.5625, 0.375],
  [0.625, 0.0625, 0.375, 1, 0.5625, 0.625],
  [0.375, 0.0625, 0.625, 0.625, 0.5625, 1],
  [0, 0.0625, 0.375, 0.375, 0.5625, 0.625],
  [0, 0, 0, 1, 0.09375, 1],
  [0, 0.0625, 0, 1, 0.15625, 1],
  [0, 0, 0, 1, 0.1, 1],
  [0, 0, 0, 1, 0.2, 1],
  [0, 0, 0, 1, 0.3, 1],
  [0, 0, 0, 1, 0.4, 1],
  [0, 0, 0, 1, 0.6, 1],
  [0, 0, 0, 1, 0.7, 1],
  [0, 0, 0, 1, 0.085714296, 1],
  [0, 0, 0, 1, 0.17142859, 1],
  [0, 0, 0, 1, 0.2571429, 1],
  [0, 0, 0, 1, 0.34285718, 1],
  [0, 0, 0, 1, 0.42857146, 1],
  [0, 0, 0, 1, 0.5142858, 1],
  [0, 0, 0, 1, 0.68571436, 1],
  [0.25, 0, 0.25, 0.75, 0.5, 0.75],
  [0.25, 0.25, 0.5, 0.75, 0.75, 1],
  [0.25, 0.25, 0, 0.75, 0.75, 0.5],
  [0.5, 0.25, 0.25, 1, 0.75, 0.75],
  [0, 0.25, 0.25, 0.5, 0.75, 0.75],
  [0.125, 0, 0, 0.875, 1, 1],
  [0, 0, 0.125, 1, 1, 0.875],
  [0, 0.625, 0, 1, 0.6875, 1, 0, 0.6875, 0, 0.125, 1, 1, 0.875, 0.6875, 0, 1, 1, 1, 0.125, 0.6875, 0, 0.875, 1, 0.125, 0.125, 0.6875, 0.875, 0.875, 1, 1, 0.25, 0.25, 0.25, 0.75, 0.625, 0.75, 0.375, 0, 0.375, 0.625, 0.25, 0.625],
  [0, 0.625, 0, 1, 0.6875, 1, 0, 0.6875, 0, 0.125, 1, 1, 0.875, 0.6875, 0, 1, 1, 1, 0.125, 0.6875, 0, 0.875, 1, 0.125, 0.125, 0.6875, 0.875, 0.875, 1, 1, 0.25, 0.25, 0.25, 0.75, 0.625, 0.75, 0.375, 0.25, 0, 0.625, 0.5, 0.25],
  [0, 0.625, 0, 1, 0.6875, 1, 0, 0.6875, 0, 0.125, 1, 1, 0.875, 0.6875, 0, 1, 1, 1, 0.125, 0.6875, 0, 0.875, 1, 0.125, 0.125, 0.6875, 0.875, 0.875, 1, 1, 0.25, 0.25, 0.25, 0.75, 0.625, 0.75, 0.375, 0.25, 0.75, 0.625, 0.5, 1],
  [0, 0.625, 0, 1, 0.6875, 1, 0, 0.6875, 0, 0.125, 1, 1, 0.875, 0.6875, 0, 1, 1, 1, 0.125, 0.6875, 0, 0.875, 1, 0.125, 0.125, 0.6875, 0.875, 0.875, 1, 1, 0.25, 0.25, 0.25, 0.75, 0.625, 0.75, 0, 0.25, 0.375, 0.25, 0.5, 0.625],
  [0, 0.625, 0, 1, 0.6875, 1, 0, 0.6875, 0, 0.125, 1, 1, 0.875, 0.6875, 0, 1, 1, 1, 0.125, 0.6875, 0, 0.875, 1, 0.125, 0.125, 0.6875, 0.875, 0.875, 1, 1, 0.25, 0.25, 0.25, 0.75, 0.625, 0.75, 0.75, 0.25, 0.375, 1, 0.5, 0.625],
  [0.116666675, 0, 0.18333335, 0.51666665, 1, 0.5833333],
  [0.116666675, 0, 0.18333335, 0.51666665, 0.6, 0.5833333],
  [0, 0, 0.875, 1, 0.78125, 1],
  [0, 0, 0, 1, 0.78125, 0.125],
  [0.875, 0, 0, 1, 0.78125, 1],
  [0, 0, 0, 0.125, 0.78125, 1],
  [0.375, 0.375, 0, 0.625, 0.625, 1],
  [0, 0.375, 0.375, 1, 0.625, 0.625],
  [0.125, 0.125, 0.125, 0.875, 0.875, 0.875],
  [0.15, 0, 0.15, 0.7, 0.6, 0.7],
  [0.3125, -0.0625, 0.3125, 0.6875, 0.1875, 0.6875],
  [0.1875, 0, 0.1875, 0.8125, 0.9375, 0.8125],
  [0.1875, -0.0625, 0.1875, 0.8125, 0.3125, 0.8125],
  [0.1875, -0.0625, 0.1875, 0.8125, 0.875, 0.8125],
  [0.1875, -0.0625, 0.1875, 0.8125, 1, 0.8125],
  [0.1875, 0, 0.1875, 0.8125, 0.6875, 0.8125],
  [0.19999999, 0, 0.19999999, 0.8, 0.45000002, 0.8],
  [0.0625, 0, 0.125, 0.9375, 1, 0.875],
  [0.1875, 0, 0.1875, 0.8125, 0.625, 0.8125],
  [-0.08333334, 0, -0.016666666, 0.71666664, 0.8, 0.7833333],
  [0.01666665, 0, 0.08333333, 0.6166667, 0.4, 0.68333334],
  [0.39999998, 0.3, 0.19999999, 1, 0.7, 0.8],
  [-14901161e-15, 0.3, 0.19999999, 0.6, 0.7, 0.8],
  [0.19999999, 0.3, 0.39999998, 0.8, 0.7, 1],
  [0.19999999, 0.3, -14901161e-15, 0.8, 0.7, 0.6],
  [-0.083333336, 0, -0.016666658, 0.71666664, 0.8, 0.7833333],
  [0.31666666, 0, 0.38333333, 0.44166666, 1, 0.5083333],
  [0.31666666, 0, 0.38333333, 0.50416666, 1, 0.5708333],
  [0.125, 0.125, 0, 0.875, 0.875, 1],
  [0, 0.125, 0.125, 1, 0.875, 0.875],
  [0, 0, 0, 1, 0.9, 1],
  [0, 0, 0.25, 1, 0.8125, 0.75],
  [0.25, 0, 0, 0.75, 0.8125, 1],
  [0.25, 0.25, 0.25, 0.75, 1, 0.75],
  [0.25, 0.25, 0, 0.75, 0.9375, 0.8125],
  [0.1875, 0.25, 0.25, 1, 0.9375, 0.75],
  [0.25, 0.25, 0.1875, 0.75, 0.9375, 1],
  [0, 0.25, 0.25, 0.8125, 0.9375, 0.75],
  [0.25, 0.25, 0, 0.75, 1, 1],
  [0, 0.25, 0.25, 1, 1, 0.75],
  [0.3125, 0, 0.3125, 0.6875, 0.5, 0.6875],
  [0.3125, 0.125, 0.3125, 0.6875, 0.625, 0.6875],
  [0, 0, 0, 1, 0.4375, 1],
  [0.3, 0, 0.3, 0.7, 0.5, 0.7],
  [0.099999994, 0, 0.099999994, 0.9, 1, 0.9],
  [0.26999998, 0, 0.26999998, 0.73, 0.58, 0.73],
  [-0.03333333, 0, 0.033333346, 0.6666667, 0.3, 0.73333335],
  [0.26999998, 0, 0.26999998, 0.73, 0.75, 0.73],
  [0.25, 0.5625, 0.25, 0.75, 1, 0.75],
  [0.25, 0, 0.25, 0.75, 0.9375, 0.75],
  [0, 0, 0, 1, 0.125, 1, 0, 0, 0, 0.125, 1, 1, 0, 0, 0, 1, 1, 0.125, 0.875, 0, 0, 1, 1, 1, 0, 0, 0.875, 1, 1, 1],
  [0, 0, 0, 1, 0.1875, 1, 0, 0, 0, 0.125, 1, 1, 0, 0, 0, 1, 1, 0.125, 0.875, 0, 0, 1, 1, 1, 0, 0, 0.875, 1, 1, 1],
  [0, 0, 0, 1, 0.4375, 1, 0, 0, 0, 0.125, 1, 1, 0, 0, 0, 1, 1, 0.125, 0.875, 0, 0, 1, 1, 1, 0, 0, 0.875, 1, 1, 1],
  [0, 0, 0, 1, 0.5625, 1, 0, 0, 0, 0.125, 1, 1, 0, 0, 0, 1, 1, 0.125, 0.875, 0, 0, 1, 1, 1, 0, 0, 0.875, 1, 1, 1],
  [0, 0, 0, 1, 0.6875, 1, 0, 0, 0, 0.125, 1, 1, 0, 0, 0, 1, 1, 0.125, 0.875, 0, 0, 1, 1, 1, 0, 0, 0.875, 1, 1, 1],
  [0, 0, 0, 1, 0.8125, 1, 0, 0, 0, 0.125, 1, 1, 0, 0, 0, 1, 1, 0.125, 0.875, 0, 0, 1, 1, 1, 0, 0, 0.875, 1, 1, 1],
  [0, 0, 0, 1, 0.9375, 1, 0, 0, 0, 0.125, 1, 1, 0, 0, 0, 1, 1, 0.125, 0.875, 0, 0, 1, 1, 1, 0, 0, 0.875, 1, 1, 1],
  [0.4375, 0, 0.4375, 0.5625, 0.375, 0.5625],
  [0.3125, 0, 0.4375, 0.6875, 0.375, 0.625],
  [0.3125, 0, 0.375, 0.625, 0.375, 0.6875],
  [0.3125, 0, 0.3125, 0.6875, 0.375, 0.625],
  [0.1875, 0.5625, 0.1875, 0.8125, 1, 0.8125],
  [0.1875, 0, 0.1875, 0.8125, 0.4375, 0.8125],
  [0.1875, 0.1875, 0.5625, 0.8125, 0.8125, 1],
  [0.1875, 0.1875, 0, 0.8125, 0.8125, 0.4375],
  [0.5625, 0.1875, 0.1875, 1, 0.8125, 0.8125],
  [0, 0.1875, 0.1875, 0.4375, 0.8125, 0.8125],
  [0.1875, 0.6875, 0.1875, 0.8125, 1, 0.8125],
  [0.1875, 0, 0.1875, 0.8125, 0.3125, 0.8125],
  [0.1875, 0.1875, 0.6875, 0.8125, 0.8125, 1],
  [0.1875, 0.1875, 0, 0.8125, 0.8125, 0.3125],
  [0.6875, 0.1875, 0.1875, 1, 0.8125, 0.8125],
  [0, 0.1875, 0.1875, 0.3125, 0.8125, 0.8125],
  [0.1875, 0.75, 0.1875, 0.8125, 1, 0.8125],
  [0.1875, 0, 0.1875, 0.8125, 0.25, 0.8125],
  [0.1875, 0.1875, 0.75, 0.8125, 0.8125, 1],
  [0.1875, 0.1875, 0, 0.8125, 0.8125, 0.25],
  [0.75, 0.1875, 0.1875, 1, 0.8125, 0.8125],
  [0, 0.1875, 0.1875, 0.25, 0.8125, 0.8125],
  [0.25, 0.8125, 0.25, 0.75, 1, 0.75],
  [0.25, 0, 0.25, 0.75, 0.1875, 0.75],
  [0.25, 0.25, 0.8125, 0.75, 0.75, 1],
  [0.25, 0.25, 0, 0.75, 0.75, 0.1875],
  [0.8125, 0.25, 0.25, 1, 0.75, 0.75],
  [0, 0.25, 0.25, 0.1875, 0.75, 0.75],
  [0.1875, 0, 0.1875, 0.8125, 0.875, 0.8125],
  [1e-4, 1e-4, 1e-4, 0.9999, 0.9999, 0.9999],
  [0.22083333, 0, 0.25416666, 0.5958333, 0.6875, 0.62916666],
  [0.15833333, 0, 0.19166666, 0.6583333, 1, 0.69166666],
  [0.09583333, 0, 0.12916666, 0.7208333, 1, 0.75416666],
  [0.03333333, 0, 0.06666667, 0.7833333, 1, 0.81666666],
  [0.22083333, 0, 0.25416666, 0.5958333, 1, 0.62916666],
  [0.22083333, 0.3125, 0.25416666, 0.5958333, 1, 0.62916666],
  [0.15, 0.75, 0.15, 0.85, 1, 0.85],
  [0.3125, 0, 0.125, 0.6875, 1, 0.5],
  [0, 0.6875, 0, 1, 0.9375, 1],
  [0, 0.6875, 0, 1, 0.8125, 1],
  [0.5, 0, 0.3125, 0.875, 1, 0.6875],
  [0.3125, 0, 0.5, 0.6875, 1, 0.875],
  [0.125, 0, 0.3125, 0.5, 1, 0.6875],
  [0, 0.125, 0, 1, 1, 1],
  [0, 0, 0, 1, 0.1875, 1],
  [0, 0, 0, 1, 0.015625, 1],
  [0.125, 0.9375, 0.125, 0.875, 1, 0.875],
  [0.125, 0, 0.125, 0.875, 0.0625, 0.875],
  [0.125, 0.125, 0.9375, 0.875, 0.875, 1],
  [0.125, 0.125, 0, 0.875, 0.875, 0.0625],
  [0.9375, 0.125, 0.125, 1, 0.875, 0.875],
  [0, 0.125, 0.125, 0.0625, 0.875, 0.875],
  [0.3125, 0, 0.3125, 0.6875, 0.375, 0.6875],
  [-0.03333333, 0.7, 0.033333346, 0.6666667, 1, 0.73333335]
];
var PROFILES = [
  { keys: ["minecraft:cardinal_direction"], values: [["east", "north", "south", "west"]], shapes: [21, 20, 18, 19] },
  { keys: ["upside_down_bit", "weirdo_direction"], values: [[0, 1], [0, 1, 2, 3]], shapes: [27, 28, 29, 30, 31, 32, 33, 34] },
  { keys: ["facing_direction"], values: [[0, 1, 2, 3, 4, 5]], shapes: [0, 0, 46, 47, 48, 49] },
  { keys: ["facing_direction", "hanging"], values: [[0, 1, 2, 3, 4, 5], [0, 1]], shapes: [54, 0, 54, 0, 56, 0, 56, 0, 54, 0, 54, 0] },
  { keys: ["height"], values: [[0, 1, 2, 3, 4, 5, 6, 7]], shapes: [0, 78, 79, 80, 10, 81, 82, 83] },
  { keys: ["bite_counter"], values: [[0, 1, 2, 3, 4, 5, 6]], shapes: [89, 90, 91, 92, 93, 94, 95] },
  { keys: ["direction", "open_bit", "upside_down_bit"], values: [[0, 1, 2, 3], [0, 1], [0, 1]], shapes: [96, 98, 45, 45, 96, 98, 101, 101, 96, 98, 103, 103, 96, 98, 105, 105] },
  { keys: ["pillar_axis"], values: [["x", "y", "z"]], shapes: [109, 108, 110] },
  { keys: ["minecraft:cardinal_direction", "open_bit"], values: [["east", "north", "south", "west"], [0, 1]], shapes: [129, 0, 128, 0, 128, 0, 129, 0] },
  { keys: ["minecraft:vertical_half"], values: [["bottom", "top"]], shapes: [10, 131] },
  { keys: ["wall_connection_type_east", "wall_connection_type_north", "wall_connection_type_south", "wall_connection_type_west", "wall_post_bit"], values: [["none", "short", "tall"], ["none", "short", "tall"], ["none", "short", "tall"], ["none", "short", "tall"], [0, 1]], shapes: [132, 132, 158, 158, 158, 158, 144, 144, 172, 172, 172, 172, 144, 144, 172, 172, 172, 172, 135, 135, 161, 161, 161, 161, 147, 149, 175, 175, 175, 175, 147, 149, 175, 175, 175, 175, 135, 135, 161, 161, 161, 161, 147, 149, 175, 175, 175, 175, 147, 149, 175, 175, 175, 175, 138, 138, 164, 166, 164, 166, 152, 152, 178, 178, 178, 178, 152, 152, 178, 178, 178, 178, 141, 141, 168, 168, 168, 168, 155, 155, 181, 181, 181, 181, 155, 155, 181, 181, 181, 181, 141, 141, 168, 168, 168, 168, 155, 155, 181, 181, 181, 181, 155, 155, 181, 181, 181, 181, 138, 138, 164, 166, 164, 166, 152, 152, 178, 178, 178, 178, 152, 152, 178, 178, 178, 178, 141, 141, 168, 168, 168, 168, 155, 155, 181, 181, 181, 181, 155, 155, 181, 181, 181, 181, 141, 141, 168, 168, 168, 168, 155, 155, 181, 181, 181, 181, 155, 155, 181, 181, 181, 181] },
  { keys: ["end_portal_eye_bit"], values: [[0, 1]], shapes: [182, 187] },
  { keys: ["age", "direction"], values: [[0, 1, 2], [0, 1, 2, 3]], shapes: [188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199] },
  { keys: ["facing_direction"], values: [[0, 1, 2, 3, 4, 5]], shapes: [219, 219, 220, 221, 222, 223] },
  { keys: ["minecraft:cardinal_direction"], values: [["east", "north", "south", "west"]], shapes: [225, 224, 224, 225] },
  { keys: ["facing_direction"], values: [[0, 1, 2, 3, 4, 5]], shapes: [226, 226, 227, 228, 229, 230] },
  { keys: ["facing_direction"], values: [[0, 1, 2, 3, 4, 5]], shapes: [88, 88, 237, 237, 238, 238] },
  { keys: ["growth", "upper_block_bit"], values: [[0, 1, 2, 3, 4, 5, 6, 7], [0, 1]], shapes: [241, 0, 243, 0, 243, 0, 243, 0, 243, 0, 243, 0, 243, 0, 243, 0] },
  { keys: ["bamboo_stalk_thickness"], values: [["thick", "thin"]], shapes: [258, 257] },
  { keys: ["attachment", "direction"], values: [["hanging", "multiple", "side", "standing"], [0, 1, 2, 3]], shapes: [86, 86, 86, 86, 86, 86, 86, 86, 259, 260, 259, 260, 86, 86, 86, 86] },
  { keys: ["attachment", "direction"], values: [["hanging", "multiple", "side", "standing"], [0, 1, 2, 3]], shapes: [264, 264, 264, 264, 269, 270, 269, 270, 265, 266, 267, 268, 262, 263, 262, 263] },
  { keys: ["hanging"], values: [[0, 1]], shapes: [271, 272] },
  { keys: ["composter_fill_level"], values: [[0, 1, 2, 3, 4, 5, 6, 7, 8]], shapes: [281, 282, 186, 283, 284, 285, 286, 287, 287] },
  { keys: ["candles"], values: [[0, 1, 2, 3]], shapes: [288, 289, 290, 291] },
  { keys: ["minecraft:block_face"], values: [["down", "east", "north", "south", "up", "west"]], shapes: [292, 297, 294, 295, 293, 296] },
  { keys: ["minecraft:block_face"], values: [["down", "east", "north", "south", "up", "west"]], shapes: [298, 303, 300, 301, 299, 302] },
  { keys: ["minecraft:block_face"], values: [["down", "east", "north", "south", "up", "west"]], shapes: [304, 309, 306, 307, 305, 308] },
  { keys: ["minecraft:block_face"], values: [["down", "east", "north", "south", "up", "west"]], shapes: [310, 315, 312, 313, 311, 314] },
  { keys: ["dripstone_thickness", "hanging"], values: [["base", "frustum", "merge", "middle", "tip"], [0, 1]], shapes: [321, 321, 319, 319, 322, 322, 320, 320, 318, 323] },
  { keys: ["big_dripleaf_head", "big_dripleaf_tilt"], values: [[0, 1], ["full_tilt", "none", "partial_tilt", "unstable"]], shapes: [0, 0, 0, 0, 0, 326, 327, 326] },
  { keys: ["upper_block_bit"], values: [[0, 1]], shapes: [9, 0] }
];
var BLOCKS = {
  air: 0,
  stone: 1,
  granite: 1,
  polished_granite: 1,
  diorite: 1,
  polished_diorite: 1,
  andesite: 1,
  polished_andesite: 1,
  grass_block: 1,
  dirt: 1,
  coarse_dirt: 1,
  podzol: 1,
  cobblestone: 1,
  oak_planks: 1,
  spruce_planks: 1,
  birch_planks: 1,
  jungle_planks: 1,
  acacia_planks: 1,
  cherry_planks: 1,
  dark_oak_planks: 1,
  pale_oak_wood: 1,
  pale_oak_planks: 1,
  mangrove_planks: 1,
  bamboo_planks: 1,
  bamboo_mosaic: 1,
  oak_sapling: 0,
  spruce_sapling: 0,
  birch_sapling: 0,
  jungle_sapling: 0,
  acacia_sapling: 0,
  cherry_sapling: 0,
  dark_oak_sapling: 0,
  pale_oak_sapling: 0,
  mangrove_propagule: 0,
  bedrock: 1,
  flowing_water: 0,
  water: 0,
  lava: 0,
  flowing_lava: 0,
  sand: 1,
  suspicious_sand: 1,
  red_sand: 1,
  gravel: 1,
  suspicious_gravel: 1,
  gold_ore: 1,
  deepslate_gold_ore: 1,
  iron_ore: 1,
  deepslate_iron_ore: 1,
  coal_ore: 1,
  deepslate_coal_ore: 1,
  nether_gold_ore: 1,
  oak_log: 1,
  spruce_log: 1,
  birch_log: 1,
  jungle_log: 1,
  acacia_log: 1,
  cherry_log: 1,
  dark_oak_log: 1,
  pale_oak_log: 1,
  mangrove_log: 1,
  mangrove_roots: 1,
  muddy_mangrove_roots: 1,
  bamboo_block: 1,
  stripped_spruce_log: 1,
  stripped_birch_log: 1,
  stripped_jungle_log: 1,
  stripped_acacia_log: 1,
  stripped_cherry_log: 1,
  stripped_dark_oak_log: 1,
  stripped_pale_oak_log: 1,
  stripped_oak_log: 1,
  stripped_mangrove_log: 1,
  stripped_bamboo_block: 1,
  oak_wood: 1,
  spruce_wood: 1,
  birch_wood: 1,
  jungle_wood: 1,
  acacia_wood: 1,
  cherry_wood: 1,
  dark_oak_wood: 1,
  mangrove_wood: 1,
  stripped_oak_wood: 1,
  stripped_spruce_wood: 1,
  stripped_birch_wood: 1,
  stripped_jungle_wood: 1,
  stripped_acacia_wood: 1,
  stripped_cherry_wood: 1,
  stripped_dark_oak_wood: 1,
  stripped_pale_oak_wood: 1,
  stripped_mangrove_wood: 1,
  oak_leaves: 1,
  spruce_leaves: 1,
  birch_leaves: 1,
  jungle_leaves: 1,
  acacia_leaves: 1,
  cherry_leaves: 1,
  dark_oak_leaves: 1,
  pale_oak_leaves: 1,
  mangrove_leaves: 1,
  azalea_leaves: 1,
  azalea_leaves_flowered: 1,
  sponge: 1,
  wet_sponge: 1,
  glass: 1,
  lapis_ore: 1,
  deepslate_lapis_ore: 1,
  lapis_block: 1,
  dispenser: 1,
  sandstone: 1,
  chiseled_sandstone: 1,
  cut_sandstone: 1,
  noteblock: 1,
  bed: 8,
  golden_rail: 0,
  detector_rail: 0,
  sticky_piston: 1,
  web: 0,
  short_grass: 0,
  fern: 0,
  deadbush: 0,
  bush: 0,
  short_dry_grass: 0,
  tall_dry_grass: 0,
  seagrass: 0,
  piston: 1,
  piston_arm_collision: 0,
  sticky_piston_arm_collision: 0,
  white_wool: 1,
  orange_wool: 1,
  magenta_wool: 1,
  light_blue_wool: 1,
  yellow_wool: 1,
  lime_wool: 1,
  pink_wool: 1,
  gray_wool: 1,
  light_gray_wool: 1,
  cyan_wool: 1,
  purple_wool: 1,
  blue_wool: 1,
  brown_wool: 1,
  green_wool: 1,
  red_wool: 1,
  black_wool: 1,
  moving_block: 0,
  dandelion: 0,
  golden_dandelion: 0,
  torchflower: 0,
  poppy: 0,
  blue_orchid: 0,
  allium: 0,
  azure_bluet: 0,
  red_tulip: 0,
  orange_tulip: 0,
  white_tulip: 0,
  pink_tulip: 0,
  oxeye_daisy: 0,
  cornflower: 0,
  wither_rose: 0,
  lily_of_the_valley: 0,
  brown_mushroom: 0,
  red_mushroom: 0,
  gold_block: 1,
  iron_block: 1,
  brick_block: 1,
  tnt: 1,
  bookshelf: 1,
  chiseled_bookshelf: 1,
  acacia_shelf: [0],
  bamboo_shelf: [0],
  birch_shelf: [0],
  cherry_shelf: [0],
  crimson_shelf: [0],
  dark_oak_shelf: [0],
  jungle_shelf: [0],
  mangrove_shelf: [0],
  oak_shelf: [0],
  pale_oak_shelf: [0],
  spruce_shelf: [0],
  warped_shelf: [0],
  mossy_cobblestone: 1,
  obsidian: 1,
  torch: 0,
  fire: 0,
  soul_fire: 0,
  mob_spawner: 1,
  creaking_heart: 1,
  oak_stairs: [1],
  chest: 35,
  redstone_wire: 0,
  diamond_ore: 1,
  deepslate_diamond_ore: 1,
  diamond_block: 1,
  crafting_table: 1,
  wheat: 0,
  farmland: 43,
  lit_furnace: 1,
  furnace: 1,
  standing_sign: 0,
  spruce_standing_sign: 0,
  birch_standing_sign: 0,
  acacia_standing_sign: 0,
  cherry_standing_sign: 0,
  jungle_standing_sign: 0,
  darkoak_standing_sign: 0,
  pale_oak_standing_sign: 0,
  mangrove_standing_sign: 0,
  bamboo_standing_sign: 0,
  wooden_door: 45,
  ladder: [2],
  rail: 0,
  stone_stairs: [1],
  wall_sign: 0,
  spruce_wall_sign: 0,
  birch_wall_sign: 0,
  acacia_wall_sign: 0,
  cherry_wall_sign: 0,
  jungle_wall_sign: 0,
  darkoak_wall_sign: 0,
  pale_oak_wall_sign: 0,
  mangrove_wall_sign: 0,
  bamboo_wall_sign: 0,
  oak_hanging_sign: [3],
  spruce_hanging_sign: [3],
  birch_hanging_sign: [3],
  acacia_hanging_sign: [3],
  cherry_hanging_sign: [3],
  jungle_hanging_sign: [3],
  dark_oak_hanging_sign: [3],
  pale_oak_hanging_sign: [3],
  crimson_hanging_sign: [3],
  warped_hanging_sign: [3],
  mangrove_hanging_sign: [3],
  bamboo_hanging_sign: [3],
  lever: 0,
  stone_pressure_plate: 0,
  iron_door: 45,
  wooden_pressure_plate: 0,
  spruce_pressure_plate: 0,
  birch_pressure_plate: 0,
  jungle_pressure_plate: 0,
  acacia_pressure_plate: 0,
  cherry_pressure_plate: 0,
  dark_oak_pressure_plate: 0,
  pale_oak_pressure_plate: 0,
  mangrove_pressure_plate: 0,
  bamboo_pressure_plate: 0,
  redstone_ore: 1,
  lit_redstone_ore: 1,
  deepslate_redstone_ore: 1,
  lit_deepslate_redstone_ore: 1,
  redstone_torch: 0,
  unlit_redstone_torch: 0,
  stone_button: 0,
  snow_layer: [4],
  ice: 1,
  snow: 1,
  cactus: 84,
  cactus_flower: 0,
  clay: 1,
  reeds: 0,
  jukebox: 1,
  oak_fence: 87,
  netherrack: 1,
  soul_sand: 83,
  soul_soil: 1,
  basalt: 1,
  polished_basalt: 1,
  soul_torch: 0,
  copper_torch: 0,
  glowstone: 1,
  portal: 0,
  carved_pumpkin: 1,
  lit_pumpkin: 1,
  cake: [5],
  powered_repeater: 78,
  unpowered_repeater: 78,
  white_stained_glass: 1,
  orange_stained_glass: 1,
  magenta_stained_glass: 1,
  light_blue_stained_glass: 1,
  yellow_stained_glass: 1,
  lime_stained_glass: 1,
  pink_stained_glass: 1,
  gray_stained_glass: 1,
  light_gray_stained_glass: 1,
  cyan_stained_glass: 1,
  purple_stained_glass: 1,
  blue_stained_glass: 1,
  brown_stained_glass: 1,
  green_stained_glass: 1,
  red_stained_glass: 1,
  black_stained_glass: 1,
  trapdoor: [6],
  spruce_trapdoor: [6],
  birch_trapdoor: [6],
  jungle_trapdoor: [6],
  acacia_trapdoor: [6],
  cherry_trapdoor: [6],
  dark_oak_trapdoor: [6],
  pale_oak_trapdoor: [6],
  mangrove_trapdoor: [6],
  bamboo_trapdoor: [6],
  stone_bricks: 1,
  mossy_stone_bricks: 1,
  cracked_stone_bricks: 1,
  chiseled_stone_bricks: 1,
  packed_mud: 1,
  mud_bricks: 1,
  infested_stone: 1,
  infested_cobblestone: 1,
  infested_stone_bricks: 1,
  infested_mossy_stone_bricks: 1,
  infested_cracked_stone_bricks: 1,
  infested_chiseled_stone_bricks: 1,
  brown_mushroom_block: 1,
  red_mushroom_block: 1,
  mushroom_stem: 1,
  iron_bars: 107,
  copper_bars: 107,
  exposed_copper_bars: 107,
  weathered_copper_bars: 107,
  oxidized_copper_bars: 107,
  waxed_copper_bars: 107,
  waxed_exposed_copper_bars: 107,
  waxed_weathered_copper_bars: 107,
  waxed_oxidized_copper_bars: 107,
  iron_chain: [7],
  copper_chain: [7],
  exposed_copper_chain: [7],
  weathered_copper_chain: [7],
  oxidized_copper_chain: [7],
  waxed_copper_chain: [7],
  waxed_exposed_copper_chain: [7],
  waxed_weathered_copper_chain: [7],
  waxed_oxidized_copper_chain: [7],
  glass_pane: 107,
  pumpkin: 1,
  melon_block: 1,
  pumpkin_stem: 0,
  melon_stem: 0,
  vine: 0,
  glow_lichen: 0,
  resin_clump: 0,
  fence_gate: [8],
  brick_stairs: [1],
  stone_brick_stairs: [1],
  mud_brick_stairs: [1],
  mycelium: 1,
  waterlily: 130,
  resin_block: 1,
  resin_bricks: 1,
  resin_brick_stairs: [1],
  resin_brick_double_slab: 1,
  resin_brick_slab: [9],
  resin_brick_wall: [10],
  chiseled_resin_bricks: 1,
  nether_brick: 1,
  nether_brick_fence: 87,
  nether_brick_stairs: [1],
  nether_wart: 0,
  enchanting_table: 82,
  brewing_stand: 184,
  cauldron: 186,
  end_portal: 79,
  end_portal_frame: [11],
  end_stone: 1,
  dragon_egg: 1,
  redstone_lamp: 1,
  lit_redstone_lamp: 1,
  cocoa: [12],
  sandstone_stairs: [1],
  emerald_ore: 1,
  deepslate_emerald_ore: 1,
  ender_chest: 35,
  tripwire_hook: 0,
  trip_wire: 0,
  emerald_block: 1,
  spruce_stairs: [1],
  birch_stairs: [1],
  jungle_stairs: [1],
  command_block: 1,
  beacon: 1,
  cobblestone_wall: [10],
  mossy_cobblestone_wall: [10],
  carrots: 0,
  potatoes: 0,
  wooden_button: 0,
  spruce_button: 0,
  birch_button: 0,
  jungle_button: 0,
  acacia_button: 0,
  cherry_button: 0,
  dark_oak_button: 0,
  pale_oak_button: 0,
  mangrove_button: 0,
  bamboo_button: 0,
  skeleton_skull: [13],
  wither_skeleton_skull: [13],
  zombie_head: [13],
  player_head: [13],
  creeper_head: [13],
  dragon_head: [13],
  piglin_head: [13],
  anvil: [14],
  chipped_anvil: [14],
  damaged_anvil: [14],
  trapped_chest: 35,
  light_weighted_pressure_plate: 0,
  heavy_weighted_pressure_plate: 0,
  powered_comparator: 78,
  unpowered_comparator: 78,
  daylight_detector: 80,
  daylight_detector_inverted: 80,
  redstone_block: 1,
  quartz_ore: 1,
  hopper: [15],
  quartz_block: 1,
  chiseled_quartz_block: 1,
  quartz_pillar: 1,
  quartz_stairs: [1],
  activator_rail: 0,
  dropper: 1,
  white_terracotta: 1,
  orange_terracotta: 1,
  magenta_terracotta: 1,
  light_blue_terracotta: 1,
  yellow_terracotta: 1,
  lime_terracotta: 1,
  pink_terracotta: 1,
  gray_terracotta: 1,
  light_gray_terracotta: 1,
  cyan_terracotta: 1,
  purple_terracotta: 1,
  blue_terracotta: 1,
  brown_terracotta: 1,
  green_terracotta: 1,
  red_terracotta: 1,
  black_terracotta: 1,
  white_stained_glass_pane: 107,
  orange_stained_glass_pane: 107,
  magenta_stained_glass_pane: 107,
  light_blue_stained_glass_pane: 107,
  yellow_stained_glass_pane: 107,
  lime_stained_glass_pane: 107,
  pink_stained_glass_pane: 107,
  gray_stained_glass_pane: 107,
  light_gray_stained_glass_pane: 107,
  cyan_stained_glass_pane: 107,
  purple_stained_glass_pane: 107,
  blue_stained_glass_pane: 107,
  brown_stained_glass_pane: 107,
  green_stained_glass_pane: 107,
  red_stained_glass_pane: 107,
  black_stained_glass_pane: 107,
  acacia_stairs: [1],
  cherry_stairs: [1],
  dark_oak_stairs: [1],
  pale_oak_stairs: [1],
  mangrove_stairs: [1],
  bamboo_stairs: [1],
  bamboo_mosaic_stairs: [1],
  slime: 1,
  barrier: 1,
  light_block_9: 0,
  light_block_8: 0,
  light_block_7: 0,
  light_block_6: 0,
  light_block_5: 0,
  light_block_4: 0,
  light_block_3: 0,
  light_block_2: 0,
  light_block_1: 0,
  light_block_0: 0,
  light_block_12: 0,
  light_block_13: 0,
  light_block_10: 0,
  light_block_11: 0,
  light_block_14: 0,
  light_block_15: 0,
  iron_trapdoor: [6],
  prismarine: 1,
  prismarine_bricks: 1,
  dark_prismarine: 1,
  prismarine_stairs: [1],
  prismarine_bricks_stairs: [1],
  dark_prismarine_stairs: [1],
  prismarine_slab: [9],
  prismarine_double_slab: 1,
  prismarine_brick_slab: [9],
  prismarine_brick_double_slab: 1,
  dark_prismarine_double_slab: 1,
  dark_prismarine_slab: [9],
  sea_lantern: 1,
  hay_block: 1,
  white_carpet: 9,
  orange_carpet: 9,
  magenta_carpet: 9,
  light_blue_carpet: 9,
  yellow_carpet: 9,
  lime_carpet: 9,
  pink_carpet: 9,
  gray_carpet: 9,
  light_gray_carpet: 9,
  cyan_carpet: 9,
  purple_carpet: 9,
  blue_carpet: 9,
  brown_carpet: 9,
  green_carpet: 9,
  red_carpet: 9,
  black_carpet: 9,
  hardened_clay: 1,
  coal_block: 1,
  packed_ice: 1,
  sunflower: 0,
  lilac: 0,
  rose_bush: 0,
  peony: 0,
  tall_grass: 0,
  large_fern: 0,
  standing_banner: 0,
  wall_banner: 0,
  red_sandstone: 1,
  chiseled_red_sandstone: 1,
  cut_red_sandstone: 1,
  red_sandstone_stairs: [1],
  oak_double_slab: 1,
  oak_slab: [9],
  spruce_double_slab: 1,
  spruce_slab: [9],
  birch_slab: [9],
  birch_double_slab: 1,
  jungle_slab: [9],
  jungle_double_slab: 1,
  acacia_double_slab: 1,
  acacia_slab: [9],
  cherry_double_slab: 1,
  cherry_slab: [9],
  dark_oak_slab: [9],
  dark_oak_double_slab: 1,
  pale_oak_slab: [9],
  pale_oak_double_slab: 1,
  mangrove_double_slab: 1,
  mangrove_slab: [9],
  bamboo_double_slab: 1,
  bamboo_slab: [9],
  bamboo_mosaic_slab: [9],
  bamboo_mosaic_double_slab: 1,
  normal_stone_slab: [9],
  normal_stone_double_slab: 1,
  smooth_stone_double_slab: 1,
  smooth_stone_slab: [9],
  sandstone_slab: [9],
  sandstone_double_slab: 1,
  cut_sandstone_double_slab: 1,
  cut_sandstone_slab: [9],
  petrified_oak_double_slab: 1,
  petrified_oak_slab: [9],
  cobblestone_slab: [9],
  cobblestone_double_slab: 1,
  brick_double_slab: 1,
  brick_slab: [9],
  stone_brick_double_slab: 1,
  stone_brick_slab: [9],
  mud_brick_double_slab: 1,
  mud_brick_slab: [9],
  nether_brick_slab: [9],
  nether_brick_double_slab: 1,
  quartz_double_slab: 1,
  quartz_slab: [9],
  red_sandstone_double_slab: 1,
  red_sandstone_slab: [9],
  cut_red_sandstone_double_slab: 1,
  cut_red_sandstone_slab: [9],
  purpur_slab: [9],
  purpur_double_slab: 1,
  smooth_stone: 1,
  smooth_sandstone: 1,
  smooth_quartz: 1,
  smooth_red_sandstone: 1,
  spruce_fence_gate: [8],
  birch_fence_gate: [8],
  jungle_fence_gate: [8],
  acacia_fence_gate: [8],
  cherry_fence_gate: [8],
  dark_oak_fence_gate: [8],
  pale_oak_fence_gate: [8],
  mangrove_fence_gate: [8],
  bamboo_fence_gate: [8],
  spruce_fence: 87,
  birch_fence: 87,
  jungle_fence: 87,
  acacia_fence: 87,
  cherry_fence: 87,
  dark_oak_fence: 87,
  pale_oak_fence: 87,
  mangrove_fence: 87,
  bamboo_fence: 87,
  spruce_door: 45,
  birch_door: 45,
  jungle_door: 45,
  acacia_door: 45,
  cherry_door: 45,
  dark_oak_door: 45,
  pale_oak_door: 45,
  mangrove_door: 45,
  bamboo_door: 45,
  end_rod: [16],
  chorus_plant: 239,
  chorus_flower: 1,
  purpur_block: 1,
  purpur_pillar: 1,
  purpur_stairs: [1],
  end_bricks: 1,
  torchflower_crop: 0,
  pitcher_crop: [17],
  pitcher_plant: 0,
  beetroot: 0,
  grass_path: 43,
  end_gateway: 0,
  repeating_command_block: 1,
  chain_command_block: 1,
  frosted_ice: 1,
  magma: 1,
  nether_wart_block: 1,
  red_nether_brick: 1,
  bone_block: 1,
  structure_void: 0,
  observer: 1,
  undyed_shulker_box: 1,
  white_shulker_box: 1,
  orange_shulker_box: 1,
  magenta_shulker_box: 1,
  light_blue_shulker_box: 1,
  yellow_shulker_box: 1,
  lime_shulker_box: 1,
  pink_shulker_box: 1,
  gray_shulker_box: 1,
  light_gray_shulker_box: 1,
  cyan_shulker_box: 1,
  purple_shulker_box: 1,
  blue_shulker_box: 1,
  brown_shulker_box: 1,
  green_shulker_box: 1,
  red_shulker_box: 1,
  black_shulker_box: 1,
  white_glazed_terracotta: 1,
  orange_glazed_terracotta: 1,
  magenta_glazed_terracotta: 1,
  light_blue_glazed_terracotta: 1,
  yellow_glazed_terracotta: 1,
  lime_glazed_terracotta: 1,
  pink_glazed_terracotta: 1,
  gray_glazed_terracotta: 1,
  silver_glazed_terracotta: 1,
  cyan_glazed_terracotta: 1,
  purple_glazed_terracotta: 1,
  blue_glazed_terracotta: 1,
  brown_glazed_terracotta: 1,
  green_glazed_terracotta: 1,
  red_glazed_terracotta: 1,
  black_glazed_terracotta: 1,
  white_concrete: 1,
  orange_concrete: 1,
  magenta_concrete: 1,
  light_blue_concrete: 1,
  yellow_concrete: 1,
  lime_concrete: 1,
  pink_concrete: 1,
  gray_concrete: 1,
  light_gray_concrete: 1,
  cyan_concrete: 1,
  purple_concrete: 1,
  blue_concrete: 1,
  brown_concrete: 1,
  green_concrete: 1,
  red_concrete: 1,
  black_concrete: 1,
  white_concrete_powder: 1,
  orange_concrete_powder: 1,
  magenta_concrete_powder: 1,
  light_blue_concrete_powder: 1,
  yellow_concrete_powder: 1,
  lime_concrete_powder: 1,
  pink_concrete_powder: 1,
  gray_concrete_powder: 1,
  light_gray_concrete_powder: 1,
  cyan_concrete_powder: 1,
  purple_concrete_powder: 1,
  blue_concrete_powder: 1,
  brown_concrete_powder: 1,
  green_concrete_powder: 1,
  red_concrete_powder: 1,
  black_concrete_powder: 1,
  kelp: 0,
  dried_kelp_block: 1,
  turtle_egg: 247,
  sniffer_egg: 248,
  dried_ghast: 249,
  dead_tube_coral_block: 1,
  dead_brain_coral_block: 1,
  dead_bubble_coral_block: 1,
  dead_fire_coral_block: 1,
  dead_horn_coral_block: 1,
  tube_coral_block: 1,
  brain_coral_block: 1,
  bubble_coral_block: 1,
  fire_coral_block: 1,
  horn_coral_block: 1,
  dead_tube_coral: 0,
  dead_brain_coral: 0,
  dead_bubble_coral: 0,
  dead_fire_coral: 0,
  dead_horn_coral: 0,
  tube_coral: 0,
  brain_coral: 0,
  bubble_coral: 0,
  fire_coral: 0,
  horn_coral: 0,
  dead_tube_coral_fan: 0,
  dead_brain_coral_fan: 0,
  dead_bubble_coral_fan: 0,
  dead_fire_coral_fan: 0,
  dead_horn_coral_fan: 0,
  tube_coral_fan: 0,
  brain_coral_fan: 0,
  bubble_coral_fan: 0,
  fire_coral_fan: 0,
  horn_coral_fan: 0,
  dead_tube_coral_wall_fan: 0,
  dead_brain_coral_wall_fan: 0,
  dead_bubble_coral_wall_fan: 0,
  dead_fire_coral_wall_fan: 0,
  dead_horn_coral_wall_fan: 0,
  tube_coral_wall_fan: 0,
  brain_coral_wall_fan: 0,
  bubble_coral_wall_fan: 0,
  fire_coral_wall_fan: 0,
  horn_coral_wall_fan: 0,
  sea_pickle: 0,
  blue_ice: 1,
  conduit: 219,
  bamboo_sapling: 0,
  bamboo: [18],
  bubble_column: 0,
  polished_granite_stairs: [1],
  smooth_red_sandstone_stairs: [1],
  mossy_stone_brick_stairs: [1],
  polished_diorite_stairs: [1],
  mossy_cobblestone_stairs: [1],
  end_brick_stairs: [1],
  normal_stone_stairs: [1],
  smooth_sandstone_stairs: [1],
  smooth_quartz_stairs: [1],
  granite_stairs: [1],
  andesite_stairs: [1],
  red_nether_brick_stairs: [1],
  polished_andesite_stairs: [1],
  diorite_stairs: [1],
  polished_granite_slab: [9],
  polished_granite_double_slab: 1,
  smooth_red_sandstone_double_slab: 1,
  smooth_red_sandstone_slab: [9],
  mossy_stone_brick_slab: [9],
  mossy_stone_brick_double_slab: 1,
  polished_diorite_slab: [9],
  polished_diorite_double_slab: 1,
  mossy_cobblestone_slab: [9],
  mossy_cobblestone_double_slab: 1,
  end_stone_brick_slab: [9],
  end_stone_brick_double_slab: 1,
  smooth_sandstone_slab: [9],
  smooth_sandstone_double_slab: 1,
  smooth_quartz_double_slab: 1,
  smooth_quartz_slab: [9],
  granite_slab: [9],
  granite_double_slab: 1,
  andesite_double_slab: 1,
  andesite_slab: [9],
  red_nether_brick_double_slab: 1,
  red_nether_brick_slab: [9],
  polished_andesite_double_slab: 1,
  polished_andesite_slab: [9],
  diorite_slab: [9],
  diorite_double_slab: 1,
  brick_wall: [10],
  prismarine_wall: [10],
  red_sandstone_wall: [10],
  mossy_stone_brick_wall: [10],
  granite_wall: [10],
  stone_brick_wall: [10],
  mud_brick_wall: [10],
  nether_brick_wall: [10],
  andesite_wall: [10],
  red_nether_brick_wall: [10],
  sandstone_wall: [10],
  end_stone_brick_wall: [10],
  diorite_wall: [10],
  scaffolding: 0,
  loom: 1,
  barrel: 1,
  smoker: 1,
  lit_smoker: 1,
  lit_blast_furnace: 1,
  blast_furnace: 1,
  cartography_table: 1,
  fletching_table: 1,
  grindstone: [19],
  lectern: 261,
  smithing_table: 1,
  stonecutter_block: 8,
  bell: [20],
  lantern: [21],
  soul_lantern: [21],
  copper_lantern: [21],
  exposed_copper_lantern: [21],
  weathered_copper_lantern: [21],
  oxidized_copper_lantern: [21],
  waxed_copper_lantern: [21],
  waxed_exposed_copper_lantern: [21],
  waxed_weathered_copper_lantern: [21],
  waxed_oxidized_copper_lantern: [21],
  campfire: 273,
  soul_campfire: 273,
  sweet_berry_bush: 0,
  warped_stem: 1,
  stripped_warped_stem: 1,
  warped_hyphae: 1,
  stripped_warped_hyphae: 1,
  warped_nylium: 1,
  warped_fungus: 0,
  warped_wart_block: 1,
  warped_roots: 0,
  nether_sprouts: 0,
  crimson_stem: 1,
  stripped_crimson_stem: 1,
  crimson_hyphae: 1,
  stripped_crimson_hyphae: 1,
  crimson_nylium: 1,
  crimson_fungus: 0,
  shroomlight: 1,
  weeping_vines: 0,
  twisting_vines: 0,
  crimson_roots: 0,
  crimson_planks: 1,
  warped_planks: 1,
  crimson_double_slab: 1,
  crimson_slab: [9],
  warped_double_slab: 1,
  warped_slab: [9],
  crimson_pressure_plate: 0,
  warped_pressure_plate: 0,
  crimson_fence: 87,
  warped_fence: 87,
  crimson_trapdoor: [6],
  warped_trapdoor: [6],
  crimson_fence_gate: [8],
  warped_fence_gate: [8],
  crimson_stairs: [1],
  warped_stairs: [1],
  crimson_button: 0,
  warped_button: 0,
  crimson_door: 45,
  warped_door: 45,
  crimson_standing_sign: 0,
  warped_standing_sign: 0,
  crimson_wall_sign: 0,
  warped_wall_sign: 0,
  structure_block: 1,
  jigsaw: 1,
  unknown: 1,
  composter: [22],
  target: 1,
  bee_nest: 1,
  beehive: 1,
  honey_block: 84,
  honeycomb_block: 1,
  netherite_block: 1,
  ancient_debris: 1,
  crying_obsidian: 1,
  respawn_anchor: 1,
  lodestone: 1,
  blackstone: 1,
  blackstone_stairs: [1],
  blackstone_wall: [10],
  blackstone_double_slab: 1,
  blackstone_slab: [9],
  polished_blackstone: 1,
  polished_blackstone_bricks: 1,
  cracked_polished_blackstone_bricks: 1,
  chiseled_polished_blackstone: 1,
  polished_blackstone_brick_double_slab: 1,
  polished_blackstone_brick_slab: [9],
  polished_blackstone_brick_stairs: [1],
  polished_blackstone_brick_wall: [10],
  gilded_blackstone: 1,
  polished_blackstone_stairs: [1],
  polished_blackstone_double_slab: 1,
  polished_blackstone_slab: [9],
  polished_blackstone_pressure_plate: 0,
  polished_blackstone_button: 0,
  polished_blackstone_wall: [10],
  chiseled_nether_bricks: 1,
  cracked_nether_bricks: 1,
  quartz_bricks: 1,
  candle: [23],
  white_candle: [23],
  orange_candle: [23],
  magenta_candle: [23],
  light_blue_candle: [23],
  yellow_candle: [23],
  lime_candle: [23],
  pink_candle: [23],
  gray_candle: [23],
  light_gray_candle: [23],
  cyan_candle: [23],
  purple_candle: [23],
  blue_candle: [23],
  brown_candle: [23],
  green_candle: [23],
  red_candle: [23],
  black_candle: [23],
  candle_cake: 89,
  white_candle_cake: 89,
  orange_candle_cake: 89,
  magenta_candle_cake: 89,
  light_blue_candle_cake: 89,
  yellow_candle_cake: 89,
  lime_candle_cake: 89,
  pink_candle_cake: 89,
  gray_candle_cake: 89,
  light_gray_candle_cake: 89,
  cyan_candle_cake: 89,
  purple_candle_cake: 89,
  blue_candle_cake: 89,
  brown_candle_cake: 89,
  green_candle_cake: 89,
  red_candle_cake: 89,
  black_candle_cake: 89,
  amethyst_block: 1,
  budding_amethyst: 1,
  amethyst_cluster: [24],
  large_amethyst_bud: [25],
  medium_amethyst_bud: [26],
  small_amethyst_bud: [27],
  tuff: 1,
  tuff_slab: [9],
  tuff_double_slab: 1,
  tuff_stairs: [1],
  tuff_wall: [10],
  polished_tuff: 1,
  polished_tuff_slab: [9],
  polished_tuff_double_slab: 1,
  polished_tuff_stairs: [1],
  polished_tuff_wall: [10],
  chiseled_tuff: 1,
  tuff_bricks: 1,
  tuff_brick_slab: [9],
  tuff_brick_double_slab: 1,
  tuff_brick_stairs: [1],
  tuff_brick_wall: [10],
  chiseled_tuff_bricks: 1,
  sulfur: 1,
  potent_sulfur: 1,
  cinnabar_slab: [9],
  raw_iron_block: 1,
  hard_blue_stained_glass_pane: 107,
  colored_torch_red: 0,
  polished_sulfur_wall: [10],
  weathered_copper: 1,
  oxidized_copper_door: 45,
  material_reducer: 1,
  waxed_oxidized_copper_grate: 1,
  waxed_copper_golem_statue: 316,
  waxed_weathered_lightning_rod: [16],
  cobbled_deepslate_wall: [10],
  hard_light_gray_stained_glass: 1,
  waxed_cut_copper_slab: [9],
  polished_deepslate_wall: [10],
  info_update2: 1,
  cinnabar_brick_wall: [10],
  waxed_chiseled_copper: 1,
  colored_torch_purple: 0,
  waxed_copper: 1,
  waxed_oxidized_double_cut_copper_slab: 1,
  waxed_copper_door: 45,
  polished_deepslate: 1,
  crafter: 1,
  trial_spawner: 1,
  element_84: 1,
  element_85: 1,
  element_86: 1,
  element_87: 1,
  element_80: 1,
  element_81: 1,
  element_82: 1,
  element_83: 1,
  element_88: 1,
  element_89: 1,
  element_97: 1,
  element_96: 1,
  element_95: 1,
  element_94: 1,
  element_93: 1,
  element_92: 1,
  element_91: 1,
  element_90: 1,
  element_99: 1,
  element_98: 1,
  element_68: 1,
  element_69: 1,
  element_62: 1,
  element_63: 1,
  element_60: 1,
  element_61: 1,
  element_66: 1,
  element_67: 1,
  element_64: 1,
  element_65: 1,
  element_79: 1,
  element_78: 1,
  element_75: 1,
  element_74: 1,
  element_77: 1,
  element_76: 1,
  element_71: 1,
  element_70: 1,
  element_73: 1,
  element_72: 1,
  element_40: 1,
  element_41: 1,
  element_42: 1,
  element_43: 1,
  element_44: 1,
  element_45: 1,
  element_46: 1,
  element_47: 1,
  element_48: 1,
  element_49: 1,
  element_53: 1,
  element_52: 1,
  element_51: 1,
  element_50: 1,
  element_57: 1,
  element_56: 1,
  element_55: 1,
  element_54: 1,
  element_59: 1,
  element_58: 1,
  element_26: 1,
  element_27: 1,
  element_24: 1,
  element_25: 1,
  element_22: 1,
  element_23: 1,
  element_20: 1,
  element_21: 1,
  element_28: 1,
  element_29: 1,
  element_39: 1,
  element_38: 1,
  element_31: 1,
  element_30: 1,
  element_33: 1,
  element_32: 1,
  element_35: 1,
  element_34: 1,
  element_37: 1,
  element_36: 1,
  element_19: 1,
  element_18: 1,
  element_17: 1,
  element_16: 1,
  element_15: 1,
  element_14: 1,
  element_13: 1,
  element_12: 1,
  element_11: 1,
  element_10: 1,
  netherreactor: 1,
  waxed_cut_copper: 1,
  weathered_double_cut_copper_slab: 1,
  pointed_dripstone: [28],
  spore_blossom: 0,
  cobbled_deepslate_slab: [9],
  deepslate_brick_stairs: [1],
  heavy_core: 219,
  camera: 1,
  element_8: 1,
  element_9: 1,
  element_6: 1,
  element_7: 1,
  element_4: 1,
  element_5: 1,
  element_2: 1,
  element_3: 1,
  element_0: 1,
  element_1: 1,
  polished_sulfur_slab: [9],
  sulfur_brick_slab: [9],
  hard_cyan_stained_glass_pane: 107,
  hard_white_stained_glass_pane: 107,
  polished_cinnabar: 1,
  polished_cinnabar_stairs: [1],
  sculk_vein: 0,
  waxed_oxidized_copper: 1,
  weathered_cut_copper: 1,
  weathered_copper_bulb: 1,
  hard_pink_stained_glass_pane: 107,
  cave_vines_head_with_berries: 0,
  chemical_heat: 1,
  deprecated_purpur_block_1: 1,
  deprecated_purpur_block_2: 1,
  allow: 1,
  cinnabar: 1,
  cinnabar_brick_slab: [9],
  waxed_weathered_copper_door: 45,
  hard_purple_stained_glass_pane: 107,
  sculk: 1,
  waxed_oxidized_lightning_rod: [16],
  hard_red_stained_glass_pane: 107,
  hard_red_stained_glass: 1,
  sulfur_brick_wall: [10],
  exposed_chiseled_copper: 1,
  exposed_cut_copper_slab: [9],
  waxed_weathered_cut_copper_slab: [9],
  weathered_copper_trapdoor: [6],
  oxidized_copper_grate: 1,
  moss_block: 1,
  mud: 83,
  cobbled_deepslate: 1,
  colored_torch_green: 0,
  hard_green_stained_glass: 1,
  cinnabar_stairs: [1],
  hard_gray_stained_glass_pane: 107,
  hard_lime_stained_glass: 1,
  chiseled_cinnabar: 1,
  waxed_exposed_double_cut_copper_slab: 1,
  weathered_copper_door: 45,
  hard_magenta_stained_glass_pane: 107,
  waxed_oxidized_copper_bulb: 1,
  oxidized_copper_chest: 35,
  azalea: 1,
  verdant_froglight: 1,
  pearlescent_froglight: 1,
  decorated_pot: 84,
  closed_eyeblossom: 0,
  hard_orange_stained_glass: 1,
  cinnabar_bricks: 1,
  cinnabar_brick_stairs: [1],
  tinted_glass: 1,
  calibrated_sculk_sensor: 10,
  exposed_cut_copper: 1,
  waxed_weathered_cut_copper_stairs: [1],
  waxed_oxidized_cut_copper_stairs: [1],
  weathered_cut_copper_slab: [9],
  copper_trapdoor: [6],
  waxed_exposed_copper_trapdoor: [6],
  waxed_weathered_copper_bulb: 1,
  big_dripleaf: [29],
  hard_black_stained_glass: 1,
  copper_door: 45,
  waxed_oxidized_copper_door: 45,
  exposed_copper_trapdoor: [6],
  exposed_copper_bulb: 1,
  weathered_copper_golem_statue: 316,
  cave_vines_body_with_berries: 0,
  cobbled_deepslate_double_slab: 1,
  reinforced_deepslate: 1,
  vault: 1,
  pale_moss_carpet: [30],
  pale_hanging_moss: 0,
  deny: 1,
  chiseled_copper: 1,
  weathered_copper_chest: 35,
  exposed_copper_golem_statue: 316,
  wildflowers: 0,
  pale_moss_block: 1,
  element_118: 1,
  element_114: 1,
  element_115: 1,
  element_116: 1,
  element_117: 1,
  element_110: 1,
  element_111: 1,
  element_112: 1,
  element_113: 1,
  element_109: 1,
  element_108: 1,
  element_107: 1,
  element_106: 1,
  element_105: 1,
  element_104: 1,
  element_103: 1,
  element_102: 1,
  element_101: 1,
  element_100: 1,
  sulfur_double_slab: 1,
  polished_sulfur_double_slab: 1,
  oxidized_cut_copper: 1,
  waxed_double_cut_copper_slab: 1,
  waxed_exposed_copper_door: 45,
  flowering_azalea: 1,
  deepslate_bricks: 1,
  hard_light_blue_stained_glass_pane: 107,
  exposed_copper_grate: 1,
  waxed_copper_grate: 1,
  exposed_copper_chest: 35,
  dirt_with_roots: 1,
  cracked_deepslate_bricks: 1,
  deprecated_anvil: [14],
  border_block: 181,
  chiseled_sulfur: 1,
  polished_cinnabar_slab: [9],
  oxidized_chiseled_copper: 1,
  cut_copper_slab: [9],
  oxidized_cut_copper_slab: [9],
  waxed_oxidized_copper_trapdoor: [6],
  waxed_oxidized_copper_golem_statue: 316,
  weathered_lightning_rod: [16],
  deepslate_tile_wall: [10],
  chiseled_deepslate: 1,
  hard_glass: 1,
  lab_table: 1,
  cut_copper_stairs: [1],
  reserved6: 1,
  exposed_cut_copper_stairs: [1],
  hard_yellow_stained_glass: 1,
  oxidized_copper_trapdoor: [6],
  hard_light_gray_stained_glass_pane: 107,
  deepslate_tiles: 1,
  element_constructor: 1,
  hard_lime_stained_glass_pane: 107,
  sculk_sensor: 10,
  copper_bulb: 1,
  hard_purple_stained_glass: 1,
  hard_blue_stained_glass: 1,
  copper_golem_statue: 316,
  hard_cyan_stained_glass: 1,
  polished_sulfur_stairs: [1],
  waxed_weathered_copper_grate: 1,
  frog_spawn: 0,
  colored_torch_blue: 0,
  client_request_placeholder_block: 1,
  cinnabar_double_slab: 1,
  polished_cinnabar_wall: [10],
  weathered_cut_copper_stairs: [1],
  waxed_copper_chest: 35,
  small_dripleaf_block: 0,
  deepslate_tile_slab: [9],
  frame: 0,
  hard_yellow_stained_glass_pane: 107,
  sulfur_stairs: [1],
  waxed_exposed_copper_bulb: 1,
  waxed_oxidized_copper_chest: 35,
  waxed_exposed_copper_golem_statue: 316,
  hard_magenta_stained_glass: 1,
  waxed_exposed_lightning_rod: [16],
  cracked_deepslate_tiles: 1,
  hard_white_stained_glass: 1,
  compound_creator: 1,
  sulfur_slab: [9],
  sulfur_wall: [10],
  sulfur_brick_stairs: [1],
  polished_cinnabar_double_slab: 1,
  cinnabar_brick_double_slab: 1,
  copper_block: 1,
  cut_copper: 1,
  waxed_exposed_cut_copper: 1,
  waxed_weathered_cut_copper: 1,
  waxed_exposed_cut_copper_stairs: [1],
  waxed_weathered_double_cut_copper_slab: 1,
  exposed_copper_door: 45,
  waxed_weathered_copper_trapdoor: [6],
  weathered_copper_grate: 1,
  copper_chest: 35,
  waxed_weathered_copper_golem_statue: 316,
  cave_vines: 0,
  pink_petals: 0,
  deepslate_tile_stairs: [1],
  deepslate_brick_slab: [9],
  infested_deepslate: 1,
  raw_copper_block: 1,
  chalkboard: 0,
  sulfur_brick_double_slab: 1,
  sculk_catalyst: 1,
  oxidized_copper: 1,
  copper_ore: 1,
  weathered_chiseled_copper: 1,
  waxed_weathered_chiseled_copper: 1,
  oxidized_cut_copper_stairs: [1],
  waxed_cut_copper_stairs: [1],
  double_cut_copper_slab: 1,
  oxidized_double_cut_copper_slab: 1,
  waxed_exposed_cut_copper_slab: [9],
  waxed_copper_trapdoor: [6],
  copper_grate: 1,
  waxed_exposed_copper_grate: 1,
  exposed_lightning_rod: [16],
  oxidized_lightning_rod: [16],
  waxed_lightning_rod: [16],
  glowingobsidian: 1,
  moss_carpet: 9,
  polished_deepslate_stairs: [1],
  deepslate_tile_double_slab: 1,
  deepslate_brick_double_slab: 1,
  underwater_torch: 0,
  lightning_rod: [16],
  hard_light_blue_stained_glass: 1,
  deepslate_brick_wall: [10],
  smooth_basalt: 1,
  hard_brown_stained_glass_pane: 107,
  oxidized_copper_bulb: 1,
  raw_gold_block: 1,
  ochre_froglight: 1,
  invisible_bedrock: 1,
  stonecutter: 1,
  underwater_tnt: 1,
  sulfur_bricks: 1,
  waxed_exposed_copper: 1,
  waxed_weathered_copper: 1,
  waxed_oxidized_cut_copper_slab: [9],
  dripstone_block: 1,
  hard_glass_pane: 107,
  exposed_copper: 1,
  exposed_double_cut_copper_slab: 1,
  waxed_copper_bulb: 1,
  oxidized_copper_golem_statue: 316,
  polished_deepslate_double_slab: 1,
  open_eyeblossom: 0,
  hard_gray_stained_glass: 1,
  cinnabar_wall: [10],
  waxed_exposed_chiseled_copper: 1,
  sulfur_spike: [28],
  deepslate: 1,
  polished_deepslate_slab: [9],
  flower_pot: 340,
  firefly_bush: 0,
  info_update: 1,
  hard_black_stained_glass_pane: 107,
  hard_brown_stained_glass: 1,
  hard_orange_stained_glass_pane: 107,
  calcite: 1,
  sculk_shrieker: 10,
  waxed_oxidized_cut_copper: 1,
  glow_frame: 0,
  polished_sulfur: 1,
  waxed_exposed_copper_chest: 35,
  hanging_roots: 0,
  cobbled_deepslate_stairs: [1],
  hard_green_stained_glass_pane: 107,
  powder_snow: 0,
  deepslate_copper_ore: 1,
  waxed_oxidized_chiseled_copper: 1,
  waxed_weathered_copper_chest: 35,
  leaf_litter: 0,
  hard_pink_stained_glass: 1
};

// src/mc/block-shapes.ts
var SHAPE_UNKNOWN = -2;
var WEST = [0, 0, 0, 0.1875, 1, 1];
var EAST = [0.8125, 0, 0, 1, 1, 1];
var NORTH = [0, 0, 0, 1, 1, 0.1875];
var SOUTH = [0, 0, 0.8125, 1, 1, 1];
var DOOR_SHAPE_WEST = SHAPES.length;
var DOOR_SHAPE_EAST = SHAPES.length + 1;
var DOOR_SHAPE_NORTH = SHAPES.length + 2;
var DOOR_SHAPE_SOUTH = SHAPES.length + 3;
var SHAPE_COUNT = SHAPES.length + 4;
function shapeBoxes(id) {
  if (id === DOOR_SHAPE_WEST) return [WEST];
  if (id === DOOR_SHAPE_EAST) return [EAST];
  if (id === DOOR_SHAPE_NORTH) return [NORTH];
  if (id === DOOR_SHAPE_SOUTH) return [SOUTH];
  const flat = SHAPES[id];
  if (flat === void 0) throw new Error(`\u5F62\u72B6\u7F16\u53F7 ${id} \u4E0D\u5B58\u5728`);
  const out = [];
  for (let i = 0; i < flat.length; i += 6) {
    out.push([flat[i], flat[i + 1], flat[i + 2], flat[i + 3], flat[i + 4], flat[i + 5]]);
  }
  return out;
}
function createBlockShapeTable() {
  const table = new BlockShapeTable();
  for (let id = 2; id < SHAPE_COUNT; id++) table.register(id, shapeBoxes(id));
  return table;
}
function normalize(v) {
  return typeof v === "boolean" ? v ? 1 : 0 : v;
}
function readState(states, key2) {
  let v = states[key2];
  if (v === void 0) v = key2.startsWith("minecraft:") ? states[key2.slice(10)] : states[`minecraft:${key2}`];
  return normalize(v);
}
function isDoor(name) {
  return name === "wooden_door" || name === "iron_door" || name.endsWith("_door");
}
function resolveDoorShape(states) {
  const dir = readState(states, "minecraft:cardinal_direction");
  const open = readState(states, "open_bit");
  const hinge = readState(states, "door_hinge_bit");
  if (dir === void 0 || open === void 0 || hinge === void 0) return SHAPE_UNKNOWN;
  const closed = [DOOR_SHAPE_WEST, DOOR_SHAPE_NORTH, DOOR_SHAPE_EAST, DOOR_SHAPE_SOUTH];
  const openLeft = [DOOR_SHAPE_NORTH, DOOR_SHAPE_EAST, DOOR_SHAPE_SOUTH, DOOR_SHAPE_WEST];
  const openRight = [DOOR_SHAPE_SOUTH, DOOR_SHAPE_WEST, DOOR_SHAPE_NORTH, DOOR_SHAPE_EAST];
  const i = dir === "south" ? 0 : dir === "west" ? 1 : dir === "north" ? 2 : dir === "east" ? 3 : -1;
  if (i < 0) return SHAPE_UNKNOWN;
  if (open === 0) return closed[i];
  return hinge === 0 ? openLeft[i] : openRight[i];
}
var hasOwn = Object.prototype.hasOwnProperty;
function isStateIndependent(typeId) {
  const name = typeId.startsWith("minecraft:") ? typeId.slice(10) : typeId;
  return typeof BLOCKS[name] === "number" && !needsBlockContext(typeId) && !isStateShaped(typeId);
}
function isStateShaped(typeId) {
  if (typeId.includes(":") && !typeId.startsWith("minecraft:")) return false;
  const n = typeId.replace(/^minecraft:/, "");
  return n === "fence" || n.endsWith("_fence") || n === "fence_gate" || n.endsWith("_fence_gate") || n.endsWith("_wall") || n.endsWith("_stairs") || n === "glass_pane" || n.endsWith("_stained_glass_pane") || n.endsWith("_bars");
}
function needsBlockContext(typeId) {
  if (typeId.includes(":") && !typeId.startsWith("minecraft:")) return false;
  const n = typeId.replace(/^minecraft:/, "");
  return n === "chorus_plant" || isDoor(n) || n === "piston" || n === "sticky_piston" || n === "piston_arm_collision" || n === "sticky_piston_arm_collision";
}
function resolveShapeId(typeId, states) {
  const name = typeId.startsWith("minecraft:") ? typeId.slice(10) : typeId;
  if (!hasOwn.call(BLOCKS, name)) return SHAPE_UNKNOWN;
  const entry = BLOCKS[name];
  if (typeof entry === "number") {
    return entry === SHAPE_NONE || !isDoor(name) ? entry : resolveDoorShape(states);
  }
  const profile = PROFILES[entry[0]];
  let at = 0;
  for (let j = 0; j < profile.keys.length; j++) {
    const v = readState(states, profile.keys[j]);
    const index = v === void 0 ? -1 : profile.values[j].indexOf(v);
    if (index < 0) return SHAPE_UNKNOWN;
    at = at * profile.values[j].length + index;
  }
  return profile.shapes[at];
}

// src/mc/block-neighbors.ts
var clockwise = [2, 3, 1, 0];
var counterClockwise = [3, 2, 0, 1];
var connectionKeys = ["minecraft:connection_east", "minecraft:connection_west", "minecraft:connection_south", "minecraft:connection_north"];
var full = [0, 0, 0, 1, 1, 1];
var nameOf = (b) => b.typeId.replace(/^minecraft:/, "");
var fence = (n) => n === "fence" || n.endsWith("_fence");
var pane = (n) => n.endsWith("_bars") || n === "glass_pane" || n.endsWith("_stained_glass_pane");
function stairBoxes(states) {
  const up = readState(states, "upside_down_bit"), d = readState(states, "weirdo_direction");
  if (typeof d !== "number" || !Number.isInteger(d) || d < 0 || d > 3) return void 0;
  const shape = readState(states, "minecraft:corner");
  const outer = shape === "outer_left" || shape === "outer_right";
  const corner = shape === "outer_left" || shape === "inner_left" ? counterClockwise[d] : shape === "outer_right" || shape === "inner_right" ? clockwise[d] : -1;
  const y0 = up === 1 ? 0 : 0.5, y1 = up === 1 ? 0.5 : 1;
  const boxes = [up === 1 ? [0, 0.5, 0, 1, 1, 1] : [0, 0, 0, 1, 0.5, 1]];
  const occupied = [false, false, false, false];
  for (let z = 0; z < 2; z++) for (let x = 0; x < 2; x++) {
    const high = d === 0 ? x === 1 : d === 1 ? x === 0 : d === 2 ? z === 1 : z === 0;
    const side = corner === 0 ? x === 1 : corner === 1 ? x === 0 : corner === 2 ? z === 1 : z === 0;
    occupied[z * 2 + x] = corner < 0 ? high : outer ? high && side : high || side;
  }
  for (let z = 0; z < 2; z++) for (let x = 0; x < 2; x++) {
    if (!occupied[z * 2 + x]) continue;
    const width = x === 0 && occupied[z * 2 + 1] ? 2 : 1;
    const depth = z === 0 && occupied[2 + x] && (width === 1 || occupied[3]) ? 2 : 1;
    boxes.push([x / 2, y0, z / 2, (x + width) / 2, y1, (z + depth) / 2]);
    for (let zz = z; zz < z + depth; zz++) for (let xx = x; xx < x + width; xx++) occupied[zz * 2 + xx] = false;
  }
  return boxes;
}
function stateBoxes(typeId, states) {
  const n = typeId.replace(/^minecraft:/, "");
  if (n.endsWith("_stairs")) return stairBoxes(states);
  if (n.endsWith("_fence_gate") || n === "fence_gate") {
    if (readState(states, "open_bit") === 1) return [];
    const d = readState(states, "direction");
    const cardinal = readState(states, "minecraft:cardinal_direction");
    const alongZ = typeof d === "number" ? (d & 1) === 1 : cardinal === "east" || cardinal === "west";
    return alongZ ? [[0.375, 0, 0, 0.625, 1, 1]] : [[0, 0, 0.375, 1, 1, 0.625]];
  }
  if (n.endsWith("_wall")) {
    const values = ["east", "west", "south", "north"].map((d) => readState(states, `wall_connection_type_${d}`));
    const c2 = values.map((v) => v === "short" || v === "low" || v === "tall");
    const post = readState(states, "wall_post_bit") === 1 || values.some((v) => v === void 0);
    const lo2 = 0.3125, hi2 = 0.6875;
    const boxes2 = [];
    if (post) {
      boxes2.push([0.25, 0, 0.25, 0.75, 1, 0.75]);
      if (c2[0]) boxes2.push([0.75, 0, lo2, 1, 1, hi2]);
      if (c2[1]) boxes2.push([0, 0, lo2, 0.25, 1, hi2]);
      if (c2[2]) boxes2.push([lo2, 0, 0.75, hi2, 1, 1]);
      if (c2[3]) boxes2.push([lo2, 0, 0, hi2, 1, 0.25]);
    } else if (c2.some(Boolean)) {
      boxes2.push([c2[1] ? 0 : lo2, 0, lo2, c2[0] ? 1 : hi2, 1, hi2]);
      if (c2[2]) boxes2.push([lo2, 0, hi2, hi2, 1, 1]);
      if (c2[3]) boxes2.push([lo2, 0, 0, hi2, 1, lo2]);
    }
    return boxes2;
  }
  if (!fence(n) && !pane(n)) return void 0;
  const isFence = fence(n), lo = isFence ? 0.375 : 0.4375, hi = 1 - lo, height = 1;
  const c = connectionKeys.map((k) => readState(states, k) === 1);
  const boxes = [];
  if (c[0] || c[1]) boxes.push([c[1] ? 0 : lo, 0, lo, c[0] ? 1 : hi, height, hi]);
  else boxes.push([lo, 0, lo, hi, height, hi]);
  if (c[2]) boxes.push([lo, 0, hi, hi, height, 1]);
  if (c[3]) boxes.push([lo, 0, 0, hi, height, lo]);
  return boxes;
}
function contextualBoxes(b, read) {
  const n = nameOf(b);
  if (isDoor(n)) {
    const upper = readState(b.states, "upper_block_bit") === 1;
    const other = read(0, upper ? -1 : 1, 0);
    if (other?.typeId !== b.typeId) return void 0;
    const lower = upper ? other.states : b.states, top = upper ? b.states : other.states;
    const id = resolveShapeId(b.typeId, { ...lower, door_hinge_bit: readState(top, "door_hinge_bit") ?? readState(lower, "door_hinge_bit") ?? false });
    return id === SHAPE_UNKNOWN ? void 0 : shapeBoxes(id);
  }
  if (n === "piston" || n === "sticky_piston" || n.endsWith("piston_arm_collision")) {
    const facing = readState(b.states, "facing_direction");
    if (typeof facing !== "number" || !Number.isInteger(facing) || facing < 0 || facing > 5) return void 0;
    const axis = facing < 2 ? 1 : facing < 4 ? 2 : 0, positive3 = facing === 1 || facing === 2 || facing === 4;
    if (n.endsWith("piston_arm_collision")) {
      const plate = [...full], rod = [0.375, 0.375, 0.375, 0.625, 0.625, 0.625];
      plate[axis + (positive3 ? 0 : 3)] = positive3 ? 0.75 : 0.25;
      rod[axis] = positive3 ? -0.25 : 0.25;
      rod[axis + 3] = positive3 ? 0.75 : 1.25;
      return [plate, rod];
    }
    if (!b.pistonExtended) return [full];
    const base = [...full];
    base[axis + (positive3 ? 3 : 0)] = positive3 ? 0.75 : 0.25;
    return [base];
  }
  if (n === "chorus_plant") {
    const center = [0.1875, 0.1875, 0.1875, 0.8125, 0.8125, 0.8125];
    const boxes = [center];
    for (let axis = 0; axis < 3; axis++) for (const sign of [-1, 1]) {
      const other = read(axis === 0 ? sign : 0, axis === 1 ? sign : 0, axis === 2 ? sign : 0);
      const type = other && nameOf(other);
      if (type === "chorus_plant" || type === "chorus_flower" || axis === 1 && sign === -1 && type === "end_stone") {
        const arm = [...center];
        arm[axis] = sign === 1 ? 0.8125 : 0;
        arm[axis + 3] = sign === 1 ? 1 : 0.1875;
        boxes.push(arm);
      }
    }
    return boxes;
  }
  return void 0;
}

// src/mc/materials.ts
var MATERIAL_DEFAULT = 0;
var MATERIAL_ICE = 1;
var MATERIAL_SLIME = 2;
var MATERIAL_STICKY = 3;
var MATERIAL_MUD = 4;
var MATERIAL_GRINDSTONE = 5;
var DEFAULT_MATERIALS = [
  { frictionScale: 1, restitution: 0 },
  { frictionScale: 0, restitution: 0 },
  { frictionScale: 1, restitution: 0.5 },
  { frictionScale: 1.65, restitution: 0 },
  { frictionScale: 0.25, restitution: 0 },
  { frictionScale: 0.05, restitution: 0 }
];
var BASE_FRICTION = 0.525;
var TERRAIN_FRICTION = 0.5125;
function materialFriction(m) {
  return BASE_FRICTION * m.frictionScale * m.frictionScale;
}
function terrainFriction(m) {
  return TERRAIN_FRICTION * TERRAIN_FRICTION / BASE_FRICTION * m.frictionScale * m.frictionScale;
}
var BY_NAME = /* @__PURE__ */ new Map([
  ["ice", MATERIAL_ICE],
  ["packed_ice", MATERIAL_ICE],
  ["blue_ice", MATERIAL_ICE],
  ["frosted_ice", MATERIAL_ICE],
  ["slime", MATERIAL_SLIME],
  ["slime_block", MATERIAL_SLIME],
  ["honey_block", MATERIAL_STICKY],
  ["soul_sand", MATERIAL_STICKY],
  ["soul_soil", MATERIAL_STICKY],
  ["cactus", MATERIAL_STICKY],
  ["mud", MATERIAL_MUD],
  ["grindstone", MATERIAL_GRINDSTONE]
]);
function resolveMaterialId(typeId) {
  const name = typeId.startsWith("minecraft:") ? typeId.slice(10) : typeId;
  return BY_NAME.get(name) ?? MATERIAL_DEFAULT;
}

// src/mc/block-registry.ts
var BOX_MIN = -1;
var BOX_MAX = 2;
var BlockEntry = class {
  constructor(typeId, base, variants, terrainStateful) {
    this.typeId = typeId;
    this.base = base;
    this.variants = variants;
    this.terrainStateful = terrainStateful;
  }
  /** 默认项是否登记了碰撞盒 */
  get hasCollision() {
    return this.base.boxes !== void 0;
  }
  /** 只按类型就能确定地形形状和材质（可以按类型缓存）：登记了碰撞盒、没有变体改碰撞或材质；或者什么都没改 */
  get terrainByType() {
    return !this.terrainStateful;
  }
  /** 状态符合的第一个变体，没有符合的返回默认项 */
  match(states) {
    const variants = this.variants;
    for (let i = 0; i < variants.length; i++) {
      const conditions = variants[i].conditions;
      let ok = true;
      for (let j = 0; j < conditions.length; j++) {
        if (!matches(readState2(states, conditions[j].key), conditions[j].values)) {
          ok = false;
          break;
        }
      }
      if (ok) return variants[i].match;
    }
    return this.base;
  }
};
function readState2(states, key2) {
  const v = states[key2];
  if (v !== void 0) return v;
  if (key2.startsWith("minecraft:")) return states[key2.slice(10)];
  return key2.includes(":") ? void 0 : states[`minecraft:${key2}`];
}
function matches(value, wanted) {
  if (value === void 0) return false;
  for (let i = 0; i < wanted.length; i++) if (wanted[i] === value) return true;
  return false;
}
function fail(typeId, path, message) {
  throw new Error(`\u65B9\u5757\u767B\u8BB0\u8868 ${typeId} \u7684 ${path} ${message}`);
}
function finiteNumber(v) {
  return typeof v === "number" && Number.isFinite(v);
}
function checkBox(typeId, path, box) {
  if (box === null || typeof box !== "object" || box.min === null || typeof box.min !== "object" || box.max === null || typeof box.max !== "object") {
    fail(typeId, path, "\u5FC5\u987B\u662F { min, max }");
  }
  const lo = [box.min.x, box.min.y, box.min.z], hi = [box.max.x, box.max.y, box.max.z];
  for (let i = 0; i < 3; i++) {
    const axis = "xyz"[i];
    if (!finiteNumber(lo[i]) || !finiteNumber(hi[i])) fail(typeId, path, `\u7684 ${axis} \u5FC5\u987B\u662F\u6709\u9650\u6570`);
    if (lo[i] < BOX_MIN || hi[i] > BOX_MAX) {
      fail(typeId, path, `\u7684 ${axis} \u5FC5\u987B\u5728 ${BOX_MIN} \u5230 ${BOX_MAX} \u4E4B\u95F4\uFF08\u6700\u591A\u8D8A\u51FA\u65B9\u5757 1 \u683C\uFF09\uFF0C\u6536\u5230 ${lo[i]} ~ ${hi[i]}\u3002\u5750\u6807\u7684\u5355\u4F4D\u662F\u683C\uFF081 \u662F\u4E00\u4E2A\u65B9\u5757\u7684\u8FB9\u957F\uFF09\uFF0C\u4E0D\u662F\u50CF\u7D20`);
    }
    if (!(hi[i] > lo[i])) fail(typeId, path, `\u5728 ${axis} \u65B9\u5411\u5FC5\u987B\u6709\u4F53\u79EF\uFF08max \u5927\u4E8E min\uFF09\uFF0C\u6536\u5230 ${lo[i]} ~ ${hi[i]}`);
  }
  return {
    local: [lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]],
    copy: { min: { x: lo[0], y: lo[1], z: lo[2] }, max: { x: hi[0], y: hi[1], z: hi[2] } }
  };
}
function checkProperties(typeId, path, p) {
  if (p === null || typeof p !== "object") fail(typeId, path, "\u5FC5\u987B\u662F\u5BF9\u8C61");
  const copy = {};
  let boxes;
  if (p.collision !== void 0) {
    if (!Array.isArray(p.collision)) fail(typeId, `${path}.collision`, "\u5FC5\u987B\u662F\u76D2\u5B50\u6570\u7EC4");
    boxes = [];
    copy.collision = [];
    p.collision.forEach((b, i) => {
      const checked = checkBox(typeId, `${path}.collision[${i}]`, b);
      boxes.push(checked.local);
      copy.collision.push(checked.copy);
    });
  }
  const numbers = [
    ["mass", 0, true],
    ["friction", 0, true],
    ["restitution", 0, true],
    ["fragileImpactSpeed", 0, false]
  ];
  for (const [key2, min, inclusive] of numbers) {
    const v = p[key2];
    if (v === void 0) continue;
    if (!finiteNumber(v) || (inclusive ? v < min : v <= min)) fail(typeId, `${path}.${key2}`, `\u5FC5\u987B\u662F${inclusive ? "\u4E0D\u5C0F\u4E8E" : "\u5927\u4E8E"} ${min} \u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${String(v)}`);
    copy[key2] = v;
  }
  return { copy, boxes };
}
function checkCondition(typeId, path, when) {
  if (when === null || typeof when !== "object" || Array.isArray(when)) fail(typeId, path, "\u5FC5\u987B\u662F\u5BF9\u8C61\uFF1A\u72B6\u6001\u540D \u2192 \u8981\u6C42\u7684\u503C");
  const keys = Object.keys(when);
  if (keys.length === 0) fail(typeId, path, "\u81F3\u5C11\u8981\u5199\u4E00\u9879\u72B6\u6001");
  const conditions = [], copy = {};
  for (const key2 of keys) {
    const raw = when[key2];
    const list = Array.isArray(raw) ? raw : [raw];
    if (list.length === 0) fail(typeId, `${path}.${key2}`, "\u4E0D\u80FD\u662F\u7A7A\u6570\u7EC4");
    for (const v of list) {
      if (typeof v !== "boolean" && typeof v !== "string" && !finiteNumber(v)) fail(typeId, `${path}.${key2}`, `\u7684\u503C\u5FC5\u987B\u662F\u5E03\u5C14\u3001\u6709\u9650\u6570\u6216\u5B57\u7B26\u4E32\uFF0C\u6536\u5230 ${String(v)}`);
    }
    conditions.push({ key: key2, values: list });
    copy[key2] = Array.isArray(raw) ? [...raw] : raw;
  }
  return { conditions, copy };
}
var BlockRegistry = class {
  constructor(definitions) {
    this.entries = /* @__PURE__ */ new Map();
    this.json = {};
    if (definitions === null || typeof definitions !== "object" || Array.isArray(definitions)) throw new Error("\u65B9\u5757\u767B\u8BB0\u8868\u5FC5\u987B\u662F\u5BF9\u8C61\uFF1A\u65B9\u5757\u7C7B\u578B \u2192 \u5B9A\u4E49");
    const materials = [...DEFAULT_MATERIALS];
    const materialOf2 = (typeId, friction, restitution) => {
      const builtin = DEFAULT_MATERIALS[resolveMaterialId(typeId)];
      const frictionScale = friction ?? builtin.frictionScale, r = restitution ?? builtin.restitution;
      let id = materials.findIndex((m) => m.frictionScale === frictionScale && m.restitution === r);
      if (id < 0) {
        id = materials.length;
        materials.push({ frictionScale, restitution: r });
      }
      return id;
    };
    let minFragile = Infinity;
    for (const key2 of Object.keys(definitions)) {
      if (key2.length === 0 || key2.startsWith(":") || key2.endsWith(":")) throw new Error(`\u65B9\u5757\u767B\u8BB0\u8868\u7684\u65B9\u5757\u7C7B\u578B\u4E0D\u5408\u6CD5\uFF1A${JSON.stringify(key2)}`);
      const typeId = qualified(key2);
      if (typeId === "minecraft:air") throw new Error("\u65B9\u5757\u767B\u8BB0\u8868\u4E0D\u80FD\u767B\u8BB0\u7A7A\u6C14");
      if (this.entries.has(typeId)) throw new Error(`\u65B9\u5757\u767B\u8BB0\u8868\u91CC ${typeId} \u767B\u8BB0\u4E86\u4E24\u6B21`);
      const definition = definitions[key2];
      const base = checkProperties(typeId, "\u9ED8\u8BA4\u9879", definition);
      const merge = (over, under) => {
        const friction = over.copy.friction ?? under?.friction, restitution = over.copy.restitution ?? under?.restitution;
        return {
          boxes: over.boxes ?? under?.boxes,
          material: materialOf2(typeId, friction, restitution),
          mass: over.copy.mass ?? under?.mass,
          friction,
          restitution,
          fragileImpactSpeed: over.copy.fragileImpactSpeed ?? under?.fragileImpactSpeed
        };
      };
      const baseMatch = merge(base);
      const json = { ...base.copy };
      const variants = [];
      let terrainStateful = false;
      if (definition.variants !== void 0) {
        if (!Array.isArray(definition.variants)) fail(typeId, "variants", "\u5FC5\u987B\u662F\u6570\u7EC4");
        const jsonVariants = [];
        definition.variants.forEach((v, i) => {
          const path = `variants[${i}]`;
          const props = checkProperties(typeId, path, v);
          const cond = checkCondition(typeId, `${path}.when`, v.when);
          variants.push({ conditions: cond.conditions, match: merge(props, baseMatch) });
          jsonVariants.push({ ...props.copy, when: cond.copy });
          if (props.copy.collision !== void 0 || props.copy.friction !== void 0 || props.copy.restitution !== void 0) terrainStateful = true;
        });
        json.variants = jsonVariants;
      }
      this.entries.set(typeId, new BlockEntry(typeId, baseMatch, variants, terrainStateful));
      this.json[typeId] = json;
      for (const m of [baseMatch, ...variants.map((v) => v.match)]) {
        if (m.fragileImpactSpeed !== void 0 && m.fragileImpactSpeed < minFragile) minFragile = m.fragileImpactSpeed;
      }
    }
    if (materials.length > 255) throw new Error(`\u65B9\u5757\u767B\u8BB0\u8868\u91CC\u4E0D\u540C\u7684\u6469\u64E6\u3001\u5F39\u6027\u7EC4\u5408\u6709 ${materials.length - DEFAULT_MATERIALS.length} \u79CD\uFF0C\u8D85\u8FC7\u4E86\u4E0A\u9650 ${255 - DEFAULT_MATERIALS.length}`);
    this.materials = materials;
    this.minFragileSpeed = minFragile;
  }
  /** 方块类型的定义（带不带命名空间都行），没有登记返回 undefined */
  get(typeId) {
    return this.entries.get(typeId.includes(":") ? typeId : `minecraft:${typeId}`);
  }
  /** 登记表的副本，键都带命名空间。可以直接存成 JSON */
  toJSON() {
    return JSON.parse(JSON.stringify(this.json));
  }
};
var cache = /* @__PURE__ */ new WeakMap();
function registryFor(definitions) {
  if (definitions === null || typeof definitions !== "object") return new BlockRegistry(definitions);
  let registry = cache.get(definitions);
  if (registry === void 0) {
    registry = new BlockRegistry(definitions);
    cache.set(definitions, registry);
  }
  return registry;
}
function qualified(typeId) {
  return typeId.includes(":") ? typeId : `minecraft:${typeId}`;
}

// src/mc/block-physics.ts
var END_STONE = {
  preventSelfLift: true,
  scaleFrictionWithGravity: true,
  liftStrength: 2,
  transitionSpeed: 2,
  slowVerticalFriction: 0.4,
  fastVerticalFriction: 0.02,
  slowHorizontalFriction: 0.3,
  fastHorizontalFriction: 0.01
};
var MATERIALS = {
  timber: { mass: 0.5, buoyancyVolume: 1 },
  leaves: { mass: 0.25, buoyancyVolume: 1 },
  light: { mass: 0.5 },
  superLight: { mass: 0.25 },
  stone: { mass: 2 },
  heavyStorage: { mass: 4 },
  bedrock: { mass: 1e3 },
  ice: { friction: 0 },
  sticky: { friction: 1.65 },
  mud: { friction: 0.25 },
  grindstone: { friction: 0.05 },
  slime: { restitution: 0.5 },
  endStone: { floatingMaterial: END_STONE, floatingScale: 1 }
};
var DEFAULT = { mass: 1, buoyancyVolume: 1, friction: 1, restitution: 0 };
function state(states, name) {
  const v = states[name];
  if (v !== void 0) return v;
  const i = name.indexOf(":");
  return i >= 0 ? states[name.slice(i + 1)] : states[`minecraft:${name}`];
}
var WOODS = "oak|spruce|birch|jungle|acacia|dark_oak|mangrove|cherry|pale_oak|crimson|warped|bamboo";
var COLORS = "white|orange|magenta|light_blue|yellow|lime|pink|gray|light_gray|cyan|purple|blue|brown|green|red|black";
var re = (source) => new RegExp(`^(?:${source})$`);
var doubled = { massMultiplier: 2, buoyancyVolumeMultiplier: 2, floatingScaleMultiplier: 2 };
var RULES = [
  {
    pattern: re(`minecraft:(?:stripped_)?(?:${WOODS})_(?:planks|log|wood|stem|hyphae|block)`),
    blocks: ["minecraft:planks", "minecraft:log", "minecraft:log2", "minecraft:wood", "minecraft:barrel", "minecraft:chest", "minecraft:trapped_chest"],
    definition: MATERIALS.timber
  },
  {
    blocks: ["minecraft:piston_arm_collision", "minecraft:sticky_piston_arm_collision", "minecraft:piston_head", "minecraft:amethyst_cluster"],
    definition: MATERIALS.light
  },
  { pattern: re("minecraft:.*_(?:slab|stairs)"), definition: { mass: 0.5, buoyancyVolume: 0.5 } },
  {
    pattern: re(`minecraft:(?:${WOODS}|bamboo_mosaic)_(?:double_)?(?:slab|stairs)`),
    blocks: ["minecraft:wooden_slab", "minecraft:double_wooden_slab"],
    definition: { mass: 0.25 }
  },
  {
    pattern: re("minecraft:(?:stone|normal_stone|cobblestone|mossy_cobblestone|granite|diorite|andesite|cobbled_deepslate|tuff|blackstone)_(?:double_)?slab"),
    definition: { mass: 1 }
  },
  {
    pattern: re(`minecraft:(?:(?:.*_)?(?:fence|fence_gate|trapdoor|door|candle)|(?:${COLORS})_carpet|carpet|glass_pane|.*_stained_glass_pane)`),
    blocks: [
      "minecraft:ladder",
      "minecraft:iron_bars",
      "minecraft:cocoa",
      "minecraft:moss_carpet",
      "minecraft:bamboo",
      "minecraft:sea_pickle",
      "minecraft:chain",
      "minecraft:iron_chain",
      "minecraft:lightning_rod",
      "minecraft:flower_pot",
      "minecraft:end_rod",
      "minecraft:lantern",
      "minecraft:soul_lantern",
      "minecraft:unpowered_repeater",
      "minecraft:powered_repeater",
      "minecraft:unpowered_comparator",
      "minecraft:powered_comparator",
      "minecraft:repeater",
      "minecraft:comparator"
    ],
    definition: { mass: 0.25, buoyancyVolume: 0.25 }
  },
  {
    pattern: re("minecraft:(?:(?:oak|spruce|birch|jungle|acacia|dark_oak|mangrove|cherry|pale_oak)_sapling|(?:.*_)?wool)"),
    blocks: [
      "minecraft:sapling",
      "minecraft:azalea",
      "minecraft:flowering_azalea",
      "minecraft:lily_pad",
      "minecraft:waterlily",
      "minecraft:big_dripleaf",
      "minecraft:turtle_egg",
      "minecraft:snow_layer",
      "minecraft:mangrove_propagule"
    ],
    definition: MATERIALS.superLight
  },
  {
    pattern: re("minecraft:(?:.*_)?leaves"),
    blocks: ["minecraft:leaves2", "minecraft:azalea_leaves_flowered"],
    definition: MATERIALS.leaves,
    // persistent 为 true 的树叶无易碎属性。
    variants: [{ when: (s) => !state(s, "persistent_bit") && !state(s, "persistent"), definition: { fragileImpactSpeed: 4 } }]
  },
  {
    blocks: [
      "minecraft:stone",
      "minecraft:andesite",
      "minecraft:diorite",
      "minecraft:granite",
      "minecraft:tuff",
      "minecraft:deepslate",
      "minecraft:cobblestone",
      "minecraft:mossy_cobblestone",
      "minecraft:cobbled_deepslate",
      "minecraft:infested_cobblestone",
      "minecraft:basalt",
      "minecraft:polished_basalt",
      "minecraft:smooth_basalt",
      "minecraft:blackstone",
      "minecraft:obsidian",
      "minecraft:crying_obsidian"
    ],
    definition: MATERIALS.stone
  },
  {
    blocks: [
      "minecraft:coal_block",
      "minecraft:diamond_block",
      "minecraft:dried_kelp_block",
      "minecraft:emerald_block",
      "minecraft:gold_block",
      "minecraft:iron_block",
      "minecraft:lapis_block",
      "minecraft:netherite_block",
      "minecraft:raw_copper_block",
      "minecraft:raw_gold_block",
      "minecraft:raw_iron_block",
      "minecraft:redstone_block",
      "minecraft:hay_block",
      "minecraft:anvil",
      "minecraft:chipped_anvil",
      "minecraft:damaged_anvil",
      "minecraft:heavy_core"
    ],
    definition: MATERIALS.heavyStorage
  },
  { blocks: ["minecraft:bedrock"], definition: MATERIALS.bedrock },
  { blocks: ["minecraft:ice", "minecraft:frosted_ice", "minecraft:packed_ice", "minecraft:blue_ice"], definition: MATERIALS.ice },
  { blocks: ["minecraft:honey_block", "minecraft:soul_sand", "minecraft:soul_soil", "minecraft:cactus"], definition: MATERIALS.sticky },
  { blocks: ["minecraft:mud"], definition: MATERIALS.mud },
  { blocks: ["minecraft:grindstone"], definition: MATERIALS.grindstone },
  { blocks: ["minecraft:slime", "minecraft:slime_block"], definition: MATERIALS.slime },
  {
    blocks: [
      "minecraft:end_stone",
      "minecraft:end_bricks",
      "minecraft:end_stone_bricks",
      "minecraft:end_brick_stairs",
      "minecraft:end_stone_brick_stairs",
      "minecraft:end_stone_brick_wall",
      "minecraft:end_stone_brick_slab",
      "minecraft:end_stone_brick_double_slab"
    ],
    definition: MATERIALS.endStone
  },
  { pattern: re("minecraft:end_stone_brick_(?:double_)?slab"), definition: { floatingScale: 0.5 } },
  // 双层台阶：Java 版的台阶用 type=double 状态，基岩版有的用单独的 double_slab 方块
  { pattern: re("minecraft:.*_slab"), definition: {}, variants: [{ when: (s) => state(s, "type") === "double", definition: doubled }] },
  { pattern: re("minecraft:.*double_.*slab"), definition: {}, variants: [{ when: (s) => state(s, "type") !== "double", definition: doubled }] },
  {
    pattern: re("minecraft:(?:sticky_)?piston"),
    definition: {},
    variants: [{ when: (s) => state(s, "extended") === true || state(s, "extended_bit") === true, definition: { mass: 0.5 } }]
  },
  {
    blocks: [
      "minecraft:bamboo",
      "minecraft:cactus",
      "minecraft:frosted_ice",
      "minecraft:ice",
      "minecraft:lily_pad",
      "minecraft:waterlily",
      "minecraft:melon",
      "minecraft:melon_block",
      "minecraft:pumpkin"
    ],
    definition: { fragileImpactSpeed: 4 }
  }
];
var ruleSets = RULES.map((r) => ({ rule: r, blocks: new Set(r.blocks) }));
var rulesByType = /* @__PURE__ */ new Map();
var staticByType = /* @__PURE__ */ new Map();
var NO_STATES = {};
function rulesOf(typeId) {
  let rules = rulesByType.get(typeId);
  if (rules === void 0) {
    rules = ruleSets.filter((r) => r.blocks.has(typeId) || r.rule.pattern?.test(typeId) === true).map((r) => r.rule);
    rulesByType.set(typeId, rules);
  }
  return rules;
}
function apply(p, d) {
  if (d.mass !== void 0) p.mass = d.mass;
  if (d.buoyancyVolume !== void 0) p.buoyancyVolume = d.buoyancyVolume;
  if (d.friction !== void 0) p.friction = d.friction;
  if (d.restitution !== void 0) p.restitution = d.restitution;
  if (d.fragileImpactSpeed !== void 0) p.fragileImpactSpeed = d.fragileImpactSpeed;
  if (d.floatingMaterial !== void 0) p.floatingMaterial = d.floatingMaterial;
  if (d.floatingScale !== void 0) p.floatingScale = d.floatingScale;
  if (d.massMultiplier !== void 0) p.mass *= d.massMultiplier;
  if (d.buoyancyVolumeMultiplier !== void 0) p.buoyancyVolume *= d.buoyancyVolumeMultiplier;
  if (d.floatingScaleMultiplier !== void 0 && p.floatingMaterial !== void 0) p.floatingScale = (p.floatingScale ?? 1) * d.floatingScaleMultiplier;
}
function getBlockPhysics(typeId, states = NO_STATES, blocks) {
  const builtin = builtinPhysics(typeId, states);
  if (blocks === void 0) return builtin;
  return registeredPhysics(registryFor(blocks), typeId, states, builtin);
}
function registeredPhysics(registry, typeId, states, builtin = builtinPhysics(typeId, states)) {
  const entry = registry.get(typeId);
  if (entry === void 0) return builtin;
  const m = entry.match(states);
  if (m.mass === void 0 && m.friction === void 0 && m.restitution === void 0 && m.fragileImpactSpeed === void 0) return builtin;
  let byBuiltin = merged.get(m);
  if (byBuiltin === void 0) {
    byBuiltin = /* @__PURE__ */ new WeakMap();
    merged.set(m, byBuiltin);
  }
  const known = byBuiltin.get(builtin);
  if (known !== void 0) return known;
  const p = { ...builtin };
  if (m.mass !== void 0) p.mass = m.mass;
  if (m.friction !== void 0) p.friction = m.friction;
  if (m.restitution !== void 0) p.restitution = m.restitution;
  if (m.fragileImpactSpeed !== void 0) p.fragileImpactSpeed = m.fragileImpactSpeed;
  byBuiltin.set(builtin, p);
  return p;
}
var merged = /* @__PURE__ */ new WeakMap();
function builtinPhysics(typeId, states) {
  const id = typeId.includes(":") ? typeId : `minecraft:${typeId}`;
  const cached = staticByType.get(id);
  if (cached !== void 0) return cached;
  const rules = rulesOf(id);
  let stateful = false;
  for (let i = 0; i < rules.length; i++) if (rules[i].variants !== void 0) stateful = true;
  const p = { ...DEFAULT };
  for (let i = 0; i < rules.length; i++) {
    const r = rules[i];
    apply(p, r.definition);
    const v = r.variants?.find((x) => x.when(states));
    if (v !== void 0) apply(p, v.definition);
  }
  if (!stateful) staticByType.set(id, p);
  return p;
}

// src/mc/block-body.ts
var FULL = [[0, 0, 0, 1, 1, 1]];
function boxesOf(typeId, states, registry) {
  const registered = registry?.get(typeId)?.match(states).boxes;
  if (registered !== void 0) return registered;
  if (isStateShaped(typeId)) {
    const boxes = stateBoxes(typeId, states);
    if (boxes !== void 0) return boxes;
  }
  const id = resolveShapeId(typeId, states);
  if (id === SHAPE_NONE) return [];
  if (id === SHAPE_FULL_CUBE || id === SHAPE_UNKNOWN) return FULL;
  return shapeBoxes(id);
}
function blockShapes(blocks, definitions) {
  const registry = definitions === void 0 ? void 0 : registryFor(definitions);
  const parts2 = [];
  let mass = 0, mx = 0, my = 0, mz = 0;
  for (const block of blocks) {
    const l = block.location;
    if (!Number.isInteger(l.x) || !Number.isInteger(l.y) || !Number.isInteger(l.z)) {
      throw new Error(`\u65B9\u5757\u7684\u4F4D\u7F6E\u5FC5\u987B\u662F\u6574\u6570\uFF0C\u6536\u5230 (${l.x}, ${l.y}, ${l.z})`);
    }
    const states = block.states ?? {};
    const boxes = boxesOf(block.typeId, states, registry);
    if (boxes.length === 0) continue;
    const physics = registry === void 0 ? getBlockPhysics(block.typeId, states) : registeredPhysics(registry, block.typeId, states);
    if (!(physics.mass > 0)) continue;
    let volume = 0, cx = 0, cy = 0, cz = 0;
    for (const b of boxes) {
      const v = (b[3] - b[0]) * (b[4] - b[1]) * (b[5] - b[2]);
      volume += v;
      cx += v * (b[0] + b[3]) / 2;
      cy += v * (b[1] + b[4]) / 2;
      cz += v * (b[2] + b[5]) / 2;
    }
    parts2.push({ block, physics, boxes, volume });
    mass += physics.mass;
    mx += physics.mass * (l.x + cx / volume);
    my += physics.mass * (l.y + cy / volume);
    mz += physics.mass * (l.z + cz / volume);
  }
  if (parts2.length === 0) throw new Error("\u6CA1\u6709\u53EF\u4EE5\u751F\u6210\u5F62\u72B6\u7684\u65B9\u5757\uFF08\u5168\u90E8\u6CA1\u6709\u78B0\u649E\u6216\u6CA1\u6709\u8D28\u91CF\uFF09");
  const com = { x: mx / mass, y: my / mass, z: mz / mass };
  const shapes = [];
  for (const { block, physics, boxes, volume } of parts2) {
    const l = block.location;
    const density = physics.mass / volume;
    const friction = BASE_FRICTION * physics.friction * physics.friction;
    const copy = block.states === void 0 ? { location: { x: l.x, y: l.y, z: l.z }, typeId: block.typeId } : { location: { x: l.x, y: l.y, z: l.z }, typeId: block.typeId, states: { ...block.states } };
    for (const b of boxes) {
      shapes.push({
        type: "box",
        size: { x: b[3] - b[0], y: b[4] - b[1], z: b[5] - b[2] },
        offset: { x: l.x + (b[0] + b[3]) / 2 - com.x, y: l.y + (b[1] + b[4]) / 2 - com.y, z: l.z + (b[2] + b[5]) / 2 - com.z },
        density,
        friction,
        restitution: physics.restitution,
        block: copy
      });
    }
  }
  return { shapes, centerOfMass: com, mass };
}

// src/core/common/transform.ts
var Transform = class {
  constructor() {
    this.px = 0;
    this.py = 0;
    this.pz = 0;
    this.r00 = 1;
    this.r01 = 0;
    this.r02 = 0;
    this.r10 = 0;
    this.r11 = 1;
    this.r12 = 0;
    this.r20 = 0;
    this.r21 = 0;
    this.r22 = 1;
  }
  copyFrom(t) {
    this.px = t.px;
    this.py = t.py;
    this.pz = t.pz;
    this.r00 = t.r00;
    this.r01 = t.r01;
    this.r02 = t.r02;
    this.r10 = t.r10;
    this.r11 = t.r11;
    this.r12 = t.r12;
    this.r20 = t.r20;
    this.r21 = t.r21;
    this.r22 = t.r22;
  }
};
function transformMul(dst, local, parent) {
  const a = parent, b = local;
  dst.r00 = a.r00 * b.r00 + a.r01 * b.r10 + a.r02 * b.r20;
  dst.r01 = a.r00 * b.r01 + a.r01 * b.r11 + a.r02 * b.r21;
  dst.r02 = a.r00 * b.r02 + a.r01 * b.r12 + a.r02 * b.r22;
  dst.r10 = a.r10 * b.r00 + a.r11 * b.r10 + a.r12 * b.r20;
  dst.r11 = a.r10 * b.r01 + a.r11 * b.r11 + a.r12 * b.r21;
  dst.r12 = a.r10 * b.r02 + a.r11 * b.r12 + a.r12 * b.r22;
  dst.r20 = a.r20 * b.r00 + a.r21 * b.r10 + a.r22 * b.r20;
  dst.r21 = a.r20 * b.r01 + a.r21 * b.r11 + a.r22 * b.r21;
  dst.r22 = a.r20 * b.r02 + a.r21 * b.r12 + a.r22 * b.r22;
  dst.px = a.r00 * b.px + a.r01 * b.py + a.r02 * b.pz;
  dst.py = a.r10 * b.px + a.r11 * b.py + a.r12 * b.pz;
  dst.pz = a.r20 * b.px + a.r21 * b.py + a.r22 * b.pz;
  dst.px += a.px;
  dst.py += a.py;
  dst.pz += a.pz;
}

// src/core/dynamics/constraint/position-correction-algorithm.ts
var PositionCorrectionAlgorithm = {
  /** 将穿透转换为目标分离速度，可能引入额外动能。 */
  BAUMGARTE: 0,
  /** 速度求解之后单独解一遍“伪速度”把物体推开，不影响真实速度 */
  SPLIT_IMPULSE: 1,
  /** 非线性高斯-赛德尔，直接修正位置和朝向。 */
  NGS: 2
};

// src/core/settings.ts
var MAX_MANIFOLD_POINTS = 4;
var LINEAR_SLOP = 5e-3;
var CONTACT_PERSISTENCE_THRESHOLD = 0.05;
var DEFAULT_GJK_MARGIN = 0.05;
var ENABLE_GJK_CACHING = true;
var DEFAULT_FRICTION = 0.2;
var DEFAULT_RESTITUTION = 0.2;
var DEFAULT_DENSITY = 1;
var DEFAULT_COLLISION_GROUP = 1;
var DEFAULT_COLLISION_MASK = 1;
var MAX_TRANSLATION_PER_STEP = 20;
var MAX_ROTATION_PER_STEP = 3.14159265358979;
var SLEEPING_VELOCITY_THRESHOLD = 0.2;
var SLEEPING_ANGULAR_VELOCITY_THRESHOLD = 0.5;
var SLEEPING_TIME_THRESHOLD = 1;
var BVH_PROXY_PADDING = 0.1;
var BVH_INCREMENTAL_COLLISION_THRESHOLD = 0.45;
var CONTACT_ENABLE_BOUNCE_THRESHOLD = 0.5;
var VELOCITY_BAUMGARTE = 0.2;
var POSITION_SPLIT_IMPULSE_BAUMGARTE = 0.4;
var ALTERNATIVE_POSITION_CORRECTION_DEPTH_THRESHOLD = 0.05;
var DEFAULT_CONTACT_POSITION_CORRECTION_ALGORITHM = PositionCorrectionAlgorithm.BAUMGARTE;
var ALTERNATIVE_CONTACT_POSITION_CORRECTION_ALGORITHM = PositionCorrectionAlgorithm.SPLIT_IMPULSE;
var DEFAULT_GRAVITY_Y = -9.80665;
var POSITION_NGS_BAUMGARTE = 1;
var DEFAULT_JOINT_POSITION_CORRECTION_ALGORITHM = PositionCorrectionAlgorithm.BAUMGARTE;
var JOINT_WARM_STARTING_FACTOR_FOR_BAUMGARTE = 0.8;
var JOINT_WARM_STARTING_FACTOR = 0.95;
var MIN_SPRING_DAMPER_DAMPING_RATIO = 1e-6;
var MAX_JACOBIAN_ROWS = 6;
var ANGULAR_SLOP = 0.017453292519943278;
var MIN_RAGDOLL_MAX_SWING_ANGLE = 1e-6;
var DIRECT_MLCP_SOLVER_EPS = 1e-9;

// src/core/dynamics/rigidbody/rigid-body-config.ts
var RigidBodyType = {
  /** 受力和碰撞影响，正常运动 */
  DYNAMIC: 0,
  /** 不动，质量视为无穷大 */
  STATIC: 1,
  /** 按设定的速度运动，不受力和碰撞影响，但会推开动态刚体 */
  KINEMATIC: 2
};
var RigidBodyConfig = class {
  constructor() {
    /** 重心的世界位置和旋转 */
    this.transform = new Transform();
    this.vx = 0;
    this.vy = 0;
    this.vz = 0;
    /** 角速度（rad/s） */
    this.wx = 0;
    this.wy = 0;
    this.wz = 0;
    this.type = RigidBodyType.DYNAMIC;
    this.autoSleep = true;
    this.linearDamping = 0;
    this.angularDamping = 0;
    this.sleepingVelocityThreshold = SLEEPING_VELOCITY_THRESHOLD;
    this.sleepingAngularVelocityThreshold = SLEEPING_ANGULAR_VELOCITY_THRESHOLD;
    this.sleepingTimeThreshold = SLEEPING_TIME_THRESHOLD;
  }
};

// src/core/common/vec3.ts
var Vec3 = class {
  constructor(x = 0, y = 0, z = 0) {
    this.x = x;
    this.y = y;
    this.z = z;
  }
  init(x, y, z) {
    this.x = x;
    this.y = y;
    this.z = z;
    return this;
  }
  zero() {
    this.x = 0;
    this.y = 0;
    this.z = 0;
    return this;
  }
  copyFrom(v) {
    this.x = v.x;
    this.y = v.y;
    this.z = v.z;
    return this;
  }
  // xxxEq 方法原地更新向量。
  addEq(v) {
    return this.init(this.x + v.x, this.y + v.y, this.z + v.z);
  }
  subEq(v) {
    return this.init(this.x - v.x, this.y - v.y, this.z - v.z);
  }
  scaleEq(s) {
    return this.init(this.x * s, this.y * s, this.z * s);
  }
  /** this += v * s */
  addScaledEq(v, s) {
    return this.init(this.x + v.x * s, this.y + v.y * s, this.z + v.z * s);
  }
  negateEq() {
    return this.init(-this.x, -this.y, -this.z);
  }
  dot(v) {
    return this.x * v.x + this.y * v.y + this.z * v.z;
  }
  lengthSq() {
    return this.x * this.x + this.y * this.y + this.z * this.z;
  }
  length() {
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
  }
  /** 归一化写回自己；零向量保持为零 */
  normalize() {
    let invLen = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    if (invLen > 0) invLen = 1 / invLen;
    return this.init(this.x * invLen, this.y * invLen, this.z * invLen);
  }
};

// src/core/collision/narrowphase/gjkepa/simplex-util.ts
function projectOrigin2(vec1, vec2, out) {
  const v1X = vec1.x, v1Y = vec1.y, v1Z = vec1.z;
  const v2X = vec2.x, v2Y = vec2.y, v2Z = vec2.z;
  const v12X = v2X - v1X, v12Y = v2Y - v1Y, v12Z = v2Z - v1Z;
  const d = v12X * v12X + v12Y * v12Y + v12Z * v12Z;
  let t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
  t = -t / d;
  if (t < 0) {
    out.init(v1X, v1Y, v1Z);
    return 1;
  }
  if (t > 1) {
    out.init(v2X, v2Y, v2Z);
    return 2;
  }
  out.init(v1X + v12X * t, v1Y + v12Y * t, v1Z + v12Z * t);
  return 3;
}
function projectOrigin3(vec1, vec2, vec3, out) {
  const v1X = vec1.x, v1Y = vec1.y, v1Z = vec1.z;
  const v2X = vec2.x, v2Y = vec2.y, v2Z = vec2.z;
  const v3X = vec3.x, v3Y = vec3.y, v3Z = vec3.z;
  const v12X = v2X - v1X, v12Y = v2Y - v1Y, v12Z = v2Z - v1Z;
  const v23X = v3X - v2X, v23Y = v3Y - v2Y, v23Z = v3Z - v2Z;
  const v31X = v1X - v3X, v31Y = v1Y - v3Y, v31Z = v1Z - v3Z;
  let nX = v12Y * v23Z - v12Z * v23Y;
  let nY = v12Z * v23X - v12X * v23Z;
  let nZ = v12X * v23Y - v12Y * v23X;
  const n12X = v12Y * nZ - v12Z * nY, n12Y = v12Z * nX - v12X * nZ, n12Z = v12X * nY - v12Y * nX;
  const n23X = v23Y * nZ - v23Z * nY, n23Y = v23Z * nX - v23X * nZ, n23Z = v23X * nY - v23Y * nX;
  const n31X = v31Y * nZ - v31Z * nY, n31Y = v31Z * nX - v31X * nZ, n31Z = v31X * nY - v31Y * nX;
  const d12 = v1X * n12X + v1Y * n12Y + v1Z * n12Z;
  const d23 = v2X * n23X + v2Y * n23Y + v2Z * n23Z;
  const d31 = v3X * n31X + v3Y * n31Y + v3Z * n31Z;
  let mind = -1;
  let minvX = 0, minvY = 0, minvZ = 0;
  let mini = 0;
  if (d12 < 0) {
    const b = projectOrigin2(vec1, vec2, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    mini = b;
    mind = d;
    minvX = out.x;
    minvY = out.y;
    minvZ = out.z;
  }
  if (d23 < 0) {
    const b = projectOrigin2(vec2, vec3, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = b << 1;
      mind = d;
      minvX = out.x;
      minvY = out.y;
      minvZ = out.z;
    }
  }
  if (d31 < 0) {
    const b = projectOrigin2(vec1, vec3, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = b & 1 | (b & 2) << 1;
      mind = d;
      minvX = out.x;
      minvY = out.y;
      minvZ = out.z;
    }
  }
  if (mind > 0) {
    out.init(minvX, minvY, minvZ);
    return mini;
  }
  let l = nX * nX + nY * nY + nZ * nZ;
  if (l > 0) l = 1 / Math.sqrt(l);
  nX = nX * l;
  nY = nY * l;
  nZ = nZ * l;
  const dn = v1X * nX + v1Y * nY + v1Z * nZ;
  let l2 = nX * nX + nY * nY + nZ * nZ;
  l2 = dn / l2;
  out.init(nX * l2, nY * l2, nZ * l2);
  return 7;
}
function projectOrigin4(vec1, vec2, vec3, vec4, out) {
  const v1X = vec1.x, v1Y = vec1.y, v1Z = vec1.z;
  const v2X = vec2.x, v2Y = vec2.y, v2Z = vec2.z;
  const v3X = vec3.x, v3Y = vec3.y, v3Z = vec3.z;
  const v4X = vec4.x, v4Y = vec4.y, v4Z = vec4.z;
  const v12X = v2X - v1X, v12Y = v2Y - v1Y, v12Z = v2Z - v1Z;
  const v13X = v3X - v1X, v13Y = v3Y - v1Y, v13Z = v3Z - v1Z;
  const v14X = v4X - v1X, v14Y = v4Y - v1Y, v14Z = v4Z - v1Z;
  const v23X = v3X - v2X, v23Y = v3Y - v2Y, v23Z = v3Z - v2Z;
  const v24X = v4X - v2X, v24Y = v4Y - v2Y, v24Z = v4Z - v2Z;
  const n123X = v12Y * v13Z - v12Z * v13Y, n123Y = v12Z * v13X - v12X * v13Z, n123Z = v12X * v13Y - v12Y * v13X;
  const n134X = v13Y * v14Z - v13Z * v14Y, n134Y = v13Z * v14X - v13X * v14Z, n134Z = v13X * v14Y - v13Y * v14X;
  const n142X = v14Y * v12Z - v14Z * v12Y, n142Y = v14Z * v12X - v14X * v12Z, n142Z = v14X * v12Y - v14Y * v12X;
  const n243X = v24Y * v23Z - v24Z * v23Y, n243Y = v24Z * v23X - v24X * v23Z, n243Z = v24X * v23Y - v24Y * v23X;
  const sign = v12X * n243X + v12Y * n243Y + v12Z * n243Z > 0 ? 1 : -1;
  const d123 = v1X * n123X + v1Y * n123Y + v1Z * n123Z;
  const d134 = v1X * n134X + v1Y * n134Y + v1Z * n134Z;
  const d142 = v1X * n142X + v1Y * n142Y + v1Z * n142Z;
  const d243 = v2X * n243X + v2Y * n243Y + v2Z * n243Z;
  let mind = -1;
  let minvX = 0, minvY = 0, minvZ = 0;
  let mini = 0;
  if (d123 * sign < 0) {
    const b = projectOrigin3(vec1, vec2, vec3, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    mini = b;
    mind = d;
    minvX = out.x;
    minvY = out.y;
    minvZ = out.z;
  }
  if (d134 * sign < 0) {
    const b = projectOrigin3(vec1, vec3, vec4, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = b & 1 | (b & 6) << 1;
      mind = d;
      minvX = out.x;
      minvY = out.y;
      minvZ = out.z;
    }
  }
  if (d142 * sign < 0) {
    const b = projectOrigin3(vec1, vec2, vec4, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = b & 3 | (b & 4) << 1;
      mind = d;
      minvX = out.x;
      minvY = out.y;
      minvZ = out.z;
    }
  }
  if (d243 * sign < 0) {
    const b = projectOrigin3(vec2, vec3, vec4, out);
    const d = out.x * out.x + out.y * out.y + out.z * out.z;
    if (mind < 0 || d < mind) {
      mini = b << 1;
      mind = d;
      minvX = out.x;
      minvY = out.y;
      minvZ = out.z;
    }
  }
  if (mind > 0) {
    out.init(minvX, minvY, minvZ);
    return mini;
  }
  out.zero();
  return 15;
}

// src/core/collision/narrowphase/gjkepa/epa-triangle.ts
var NEXT_INDEX = [1, 2, 0];
var EpaTriangle = class {
  constructor() {
    this.next = null;
    this.prev = null;
    this.vertices = [null, null, null];
    /** 第 i 条边（顶点 i 到顶点 i+1）对面的三角形，以及这条边在对面三角形里的编号 */
    this.adjacentTriangles = [null, null, null];
    this.adjacentPairIndex = [0, 0, 0];
    /** 法线（没有归一化） */
    this.normal = new Vec3();
    /** 原点到这个面的最近距离的平方 */
    this.distanceSq = 0;
    /** 找洞的深度优先搜索里用的访问标记 */
    this.tmpDfsId = 0;
    this.tmpDfsVisible = false;
    this.tmp = new Vec3();
  }
  /** 从点 from 能不能看到这个面（from 在面的外侧） */
  checkVisible(from) {
    this.tmp.copyFrom(from).subEq(this.vertices[0].v);
    this.tmpDfsVisible = this.tmp.dot(this.normal) > 0;
    return this.tmpDfsVisible;
  }
  /**
   * 用三个顶点初始化。center 是多面体内部的一个点，用来判断法线朝向：
   * 法线朝里时，autoCheck 为 true 就交换两个顶点把它翻过来，否则返回 false。
   */
  init(vertex1, vertex2, vertex3, center, autoCheck = false) {
    const v1X = vertex1.v.x, v1Y = vertex1.v.y, v1Z = vertex1.v.z;
    const v2X = vertex2.v.x, v2Y = vertex2.v.y, v2Z = vertex2.v.z;
    const v3X = vertex3.v.x, v3Y = vertex3.v.y, v3Z = vertex3.v.z;
    const v12X = v2X - v1X, v12Y = v2Y - v1Y, v12Z = v2Z - v1Z;
    const v13X = v3X - v1X, v13Y = v3Y - v1Y, v13Z = v3Z - v1Z;
    const vc1X = v1X - center.x, vc1Y = v1Y - center.y, vc1Z = v1Z - center.z;
    let nX = v12Y * v13Z - v12Z * v13Y;
    let nY = v12Z * v13X - v12X * v13Z;
    let nZ = v12X * v13Y - v12Y * v13X;
    let inverted = false;
    const d = vc1X * nX + vc1Y * nY + vc1Z * nZ;
    if (d < 0) {
      if (autoCheck) {
        const tmp = vertex2;
        vertex2 = vertex3;
        vertex3 = tmp;
        nX = nX * -1;
        nY = nY * -1;
        nZ = nZ * -1;
      } else {
        inverted = true;
      }
    }
    this.vertices[0] = vertex1;
    this.vertices[1] = vertex2;
    this.vertices[2] = vertex3;
    this.normal.init(nX, nY, nZ);
    projectOrigin3(vertex1.v, vertex2.v, vertex3.v, this.tmp);
    this.distanceSq = this.tmp.lengthSq();
    this.adjacentTriangles[0] = null;
    this.adjacentTriangles[1] = null;
    this.adjacentTriangles[2] = null;
    this.adjacentPairIndex[0] = -1;
    this.adjacentPairIndex[1] = -1;
    this.adjacentPairIndex[2] = -1;
    return !inverted;
  }
  /** 和共用一条边的三角形互相登记。恰好共用一条边才算成功 */
  setAdjacentTriangle(triangle) {
    let count = 0;
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        const i2 = NEXT_INDEX[i];
        const j2 = NEXT_INDEX[j];
        if (this.vertices[i] === triangle.vertices[j2] && this.vertices[i2] === triangle.vertices[j]) {
          this.adjacentTriangles[i] = triangle;
          this.adjacentPairIndex[i] = j;
          triangle.adjacentTriangles[j] = this;
          triangle.adjacentPairIndex[j] = i;
          count++;
        }
      }
    }
    return count === 1;
  }
  removeAdjacentTriangles() {
    for (let i = 0; i < 3; i++) {
      const triangle = this.adjacentTriangles[i];
      if (triangle !== null) {
        const pairIndex = this.adjacentPairIndex[i];
        triangle.adjacentTriangles[pairIndex] = null;
        triangle.adjacentPairIndex[pairIndex] = -1;
        this.adjacentTriangles[i] = null;
        this.adjacentPairIndex[i] = -1;
      }
    }
  }
  removeReferences() {
    this.next = null;
    this.prev = null;
    this.tmpDfsId = 0;
    this.tmpDfsVisible = false;
    this.distanceSq = 0;
    this.vertices[0] = null;
    this.vertices[1] = null;
    this.vertices[2] = null;
    this.adjacentTriangles[0] = null;
    this.adjacentTriangles[1] = null;
    this.adjacentTriangles[2] = null;
    this.adjacentPairIndex[0] = 0;
    this.adjacentPairIndex[1] = 0;
    this.adjacentPairIndex[2] = 0;
  }
};

// src/core/collision/narrowphase/gjkepa/epa-vertex.ts
var EpaVertex = class {
  constructor() {
    /** 对象池链表 */
    this.next = null;
    this.v = new Vec3();
    this.w1 = new Vec3();
    this.w2 = new Vec3();
    /** 补洞时用：洞的边界上，沿边走到的下一个顶点，以及这条边外侧（没被删掉）的三角形 */
    this.tmpEdgeLoopNext = null;
    this.tmpEdgeLoopOuterTriangle = null;
  }
  init(v, w1, w2) {
    this.v.copyFrom(v);
    this.w1.copyFrom(w1);
    this.w2.copyFrom(w2);
    this.next = null;
    this.tmpEdgeLoopNext = null;
    this.tmpEdgeLoopOuterTriangle = null;
    return this;
  }
  removeReferences() {
    this.next = null;
    this.tmpEdgeLoopNext = null;
    this.tmpEdgeLoopOuterTriangle = null;
  }
};

// src/core/collision/narrowphase/gjkepa/epa-polyhedron.ts
var EpaPolyhedronState = {
  OK: 0,
  INVALID_TRIANGLE: 1,
  NO_ADJACENT_PAIR_INDEX: 2,
  NO_ADJACENT_TRIANGLE: 3,
  EDGE_LOOP_BROKEN: 4,
  NO_OUTER_TRIANGLE: 5,
  TRIANGLE_INVISIBLE: 6
};
var EpaPolyhedron = class {
  constructor() {
    this.vertices = [];
    this.numVertices = 0;
    this.triangleList = null;
    this.triangleListLast = null;
    this.numTriangles = 0;
    this.status = 0;
    /** 初始四面体的中心，判断新三角形法线朝向用 */
    this.center = new Vec3();
    this.trianglePool = null;
    this.vertexPool = null;
  }
  pickVertex() {
    const v = this.vertexPool;
    if (v === null) return new EpaVertex();
    this.vertexPool = v.next;
    v.next = null;
    return v;
  }
  poolVertex(v) {
    v.removeReferences();
    v.next = this.vertexPool;
    this.vertexPool = v;
  }
  pickTriangle() {
    const t = this.trianglePool;
    if (t === null) return new EpaTriangle();
    this.trianglePool = t.next;
    t.next = null;
    return t;
  }
  poolTriangle(t) {
    t.removeReferences();
    t.next = this.trianglePool;
    this.trianglePool = t;
  }
  setAdjacentTriangle(t1, t2) {
    if (!t1.setAdjacentTriangle(t2)) this.status = EpaPolyhedronState.INVALID_TRIANGLE;
  }
  initTriangle(t, v1, v2, v3, center, autoCheck = false) {
    if (!t.init(v1, v2, v3, center, autoCheck)) this.status = EpaPolyhedronState.INVALID_TRIANGLE;
  }
  addTriangle(t) {
    this.numTriangles++;
    if (this.triangleList === null) {
      this.triangleList = t;
      this.triangleListLast = t;
    } else {
      this.triangleListLast.next = t;
      t.prev = this.triangleListLast;
      this.triangleListLast = t;
    }
  }
  removeTriangle(t) {
    this.numTriangles--;
    const prev = t.prev, next = t.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (t === this.triangleList) this.triangleList = this.triangleList.next;
    if (t === this.triangleListLast) this.triangleListLast = this.triangleListLast.prev;
    t.next = null;
    t.prev = null;
    this.poolTriangle(t);
  }
  /** 检查每个三角形的三条边是否都有相邻的三角形，同时清除补洞用的临时标记 */
  validate() {
    for (let t = this.triangleList; t !== null; t = t.next) {
      for (let i = 0; i < 3; i++) {
        t.vertices[i].tmpEdgeLoopOuterTriangle = null;
        t.vertices[i].tmpEdgeLoopNext = null;
        if (t.adjacentPairIndex[i] === -1) {
          this.status = EpaPolyhedronState.NO_ADJACENT_PAIR_INDEX;
          return false;
        }
        if (t.adjacentTriangles[i] === null) {
          this.status = EpaPolyhedronState.NO_ADJACENT_TRIANGLE;
          return false;
        }
      }
    }
    return true;
  }
  /**
   * 从 base 开始深度优先搜索从 from 能看到的三角形，把它们删掉（挖出一个洞），
   * 并在洞边界的顶点上记下沿边界的下一个顶点和边界外侧的三角形。
   */
  findEdgeLoop(id, base, from) {
    if (base.tmpDfsId === id) return;
    base.tmpDfsId = id;
    if (!base.checkVisible(from)) {
      this.status = EpaPolyhedronState.TRIANGLE_INVISIBLE;
      return;
    }
    for (let i = 0; i < 3; i++) {
      const t = base.adjacentTriangles[i];
      if (t === null) continue;
      if (t.checkVisible(from)) {
        this.findEdgeLoop(id, t, from);
      } else {
        const v1 = base.vertices[i];
        const v2 = base.vertices[NEXT_INDEX[i]];
        v1.tmpEdgeLoopNext = v2;
        v1.tmpEdgeLoopOuterTriangle = t;
      }
    }
    base.removeAdjacentTriangles();
    this.removeTriangle(base);
  }
  /** 删掉所有三角形，顶点放回对象池 */
  clear() {
    while (this.numTriangles > 0) this.removeTriangle(this.triangleList);
    while (this.numVertices > 0) this.poolVertex(this.vertices[--this.numVertices]);
  }
  /** 用四个顶点建立初始四面体 */
  init(v1, v2, v3, v4) {
    this.status = EpaPolyhedronState.OK;
    this.numVertices = 4;
    this.vertices[0] = v1;
    this.vertices[1] = v2;
    this.vertices[2] = v3;
    this.vertices[3] = v4;
    this.center.copyFrom(v1.v).addEq(v2.v).addEq(v3.v).addEq(v4.v).scaleEq(0.25);
    const t1 = this.pickTriangle();
    const t2 = this.pickTriangle();
    const t3 = this.pickTriangle();
    const t4 = this.pickTriangle();
    this.initTriangle(t1, v1, v2, v3, this.center, true);
    this.initTriangle(t2, v1, v2, v4, this.center, true);
    this.initTriangle(t3, v1, v3, v4, this.center, true);
    this.initTriangle(t4, v2, v3, v4, this.center, true);
    this.setAdjacentTriangle(t1, t2);
    this.setAdjacentTriangle(t1, t3);
    this.setAdjacentTriangle(t1, t4);
    this.setAdjacentTriangle(t2, t3);
    this.setAdjacentTriangle(t2, t4);
    this.setAdjacentTriangle(t3, t4);
    this.addTriangle(t1);
    this.addTriangle(t2);
    this.addTriangle(t3);
    this.addTriangle(t4);
    return this.status === EpaPolyhedronState.OK;
  }
  /** 离原点最近的面；距离相同时取链表里靠前的 */
  getBestTriangle() {
    let mind = Infinity;
    let minf = null;
    for (let f = this.triangleList; f !== null; f = f.next) {
      if (f.distanceSq < mind) {
        mind = f.distanceSq;
        minf = f;
      }
    }
    return minf;
  }
  /** 在面 base 外加一个顶点：挖洞，再用新顶点和洞的边界补上 */
  addVertex(vertex, base) {
    this.vertices[this.numVertices++] = vertex;
    const v1 = base.vertices[0];
    this.findEdgeLoop(this.numVertices, base, vertex.v);
    if (this.status !== EpaPolyhedronState.OK) return false;
    let v = v1;
    const firstV = v1;
    let prevT = null;
    let firstT = null;
    do {
      if (v.tmpEdgeLoopNext === null) {
        this.status = EpaPolyhedronState.EDGE_LOOP_BROKEN;
        return false;
      }
      if (v.tmpEdgeLoopOuterTriangle === null) {
        this.status = EpaPolyhedronState.NO_OUTER_TRIANGLE;
        return false;
      }
      const t = this.pickTriangle();
      if (firstT === null) firstT = t;
      this.initTriangle(t, v, v.tmpEdgeLoopNext, vertex, this.center);
      if (this.status !== EpaPolyhedronState.OK) return false;
      this.addTriangle(t);
      this.setAdjacentTriangle(t, v.tmpEdgeLoopOuterTriangle);
      if (prevT !== null) this.setAdjacentTriangle(t, prevT);
      prevT = t;
      v = v.tmpEdgeLoopNext;
    } while (v !== firstV);
    this.setAdjacentTriangle(prevT, firstT);
    return this.status === EpaPolyhedronState.OK && this.validate();
  }
};

// src/core/collision/narrowphase/gjkepa/gjk-cache.ts
var GjkCache = class {
  constructor() {
    this.prevClosestDir = new Vec3();
  }
  clear() {
    this.prevClosestDir.zero();
  }
};

// src/core/collision/narrowphase/gjkepa/gjk-epa.ts
var GjkEpaResultState = {
  SUCCEEDED: 0,
  GJK_FAILED_TO_MAKE_TETRAHEDRON: 1,
  GJK_DID_NOT_CONVERGE: 2,
  EPA_FAILED_TO_INIT: 257,
  EPA_FAILED_TO_ADD_VERTEX: 258,
  EPA_DID_NOT_CONVERGE: 259
};
var instance = null;
var GjkEpa = class _GjkEpa {
  constructor() {
    /** 几何体 1、2 上的最近点（世界坐标） */
    this.closestPoint1 = new Vec3();
    this.closestPoint2 = new Vec3();
    /** 两个核心之间的距离，重叠时是负的穿透深度 */
    this.distance = 0;
    this.c1 = null;
    this.c2 = null;
    this.tf1 = null;
    this.tf2 = null;
    /** 单纯形顶点（闵可夫斯基差上的点），以及它们在两个几何体上对应的点 */
    this.s = [new Vec3(), new Vec3(), new Vec3(), new Vec3()];
    this.w1 = [new Vec3(), new Vec3(), new Vec3(), new Vec3()];
    this.w2 = [new Vec3(), new Vec3(), new Vec3(), new Vec3()];
    this.simplexSize = 0;
    /** 求支撑点的方向 */
    this.dir = new Vec3();
    /** 单纯形上离原点最近的点 */
    this.closest = new Vec3();
    /** 把单纯形补成四面体时试探的方向 */
    this.baseDirs = [new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1)];
    this.depth = 0;
    this.polyhedron = new EpaPolyhedron();
    /** 凸体扫掠用：两个几何体的平移、射线的起点和相对平移；射线检测时代表起点的变换 */
    this.tl1 = new Vec3();
    this.tl2 = new Vec3();
    this.rayX = new Vec3();
    this.rayR = new Vec3();
    this.tempTransform = new Transform();
  }
  /** 共用的实例。第一次用到时才创建，import 本库没有副作用 */
  static getInstance() {
    if (instance === null) instance = new _GjkEpa();
    return instance;
  }
  /**
   * 求两个凸几何体核心之间的最近点和距离（重叠时用 EPA 求穿透深度），返回 GjkEpaResultState。
   * cache 不为 null 时从上次的结果开始迭代，并把这次的结果存进去。
   */
  computeClosestPoints(c1, c2, tf1, tf2, cache2) {
    return this.computeClosestPointsImpl(c1, c2, tf1, tf2, cache2, true);
  }
  /** 同上，但重叠时不跑 EPA：distance 直接是 0，最近点不更新 */
  computeDistance(c1, c2, tf1, tf2, cache2) {
    return this.computeClosestPointsImpl(c1, c2, tf1, tf2, cache2, false);
  }
  /**
   * 凸体扫掠：c1、c2 分别从 tf1、tf2 出发平移 tl1、tl2，求第一次碰到的时刻。
   * 碰到时返回 true，hit 里是比例（0~1）、碰撞点（世界坐标）、法线；一开始就重叠、或者扫完都没碰到，返回 false
   */
  convexCast(c1, c2, tf1, tf2, tl1, tl2, hit) {
    return this.convexCastImpl(c1, c2, tf1, tf2, tl1, tl2, hit);
  }
  /** 用 GJK 做射线检测：把线段当作从 begin 平移到 end 的一个点，和放在 tf 处的 c 做凸体扫掠 */
  rayCast(c, tf, beginX, beginY, beginZ, endX, endY, endZ, hit) {
    const tf1 = this.tempTransform;
    tf1.px = beginX;
    tf1.py = beginY;
    tf1.pz = beginZ;
    this.tl1.init(endX - beginX, endY - beginY, endZ - beginZ);
    this.tl2.zero();
    return this.convexCastImpl(null, c, tf1, tf, this.tl1, this.tl2, hit);
  }
  computeClosestPointsImpl(c1, c2, tf1, tf2, cache2, useEpa) {
    this.c1 = c1;
    this.c2 = c2;
    this.tf1 = tf1;
    this.tf2 = tf2;
    const s = this.s, closest = this.closest, dir = this.dir;
    if (cache2 !== null) {
      if (cache2.gjkCache === null) cache2.gjkCache = new GjkCache();
      dir.copyFrom(cache2.gjkCache.prevClosestDir);
    } else {
      dir.zero();
    }
    if (dir.lengthSq() === 0) {
      dir.init(tf2.px - tf1.px, tf2.py - tf1.py, tf2.pz - tf1.pz);
      if (dir.lengthSq() < 1e-6) dir.init(1, 0, 0);
    }
    this.simplexSize = 0;
    this.computeSupportingVertex();
    this.simplexSize = 1;
    const eps2 = 1e-8;
    let count = 0;
    while (count < 40) {
      let v = 0;
      switch (this.simplexSize) {
        case 1:
          closest.copyFrom(s[0]);
          v = 1;
          break;
        case 2:
          v = projectOrigin2(s[0], s[1], closest);
          break;
        case 3:
          v = projectOrigin3(s[0], s[1], s[2], closest);
          break;
        case 4:
          v = projectOrigin4(s[0], s[1], s[2], s[3], closest);
          break;
      }
      if (closest.lengthSq() < eps2) {
        if (!useEpa) {
          this.distance = 0;
          return GjkEpaResultState.SUCCEEDED;
        }
        switch (this.simplexSize) {
          case 1:
            this.pointToTetrahedron();
            break;
          case 2:
            this.lineToTetrahedron();
            break;
          case 3:
            this.triangleToTetrahedron();
            break;
        }
        if (this.simplexSize === 4) {
          const epaState = this.computeDepth();
          if (epaState !== GjkEpaResultState.SUCCEEDED) {
            this.distance = 0;
            return epaState;
          }
          this.distance = -this.depth;
          return GjkEpaResultState.SUCCEEDED;
        }
        this.distance = 0;
        return GjkEpaResultState.GJK_FAILED_TO_MAKE_TETRAHEDRON;
      }
      this.shrinkSimplex(v);
      dir.copyFrom(closest).negateEq();
      this.computeSupportingVertex();
      if (dir.lengthSq() < eps2) throw new Error("GJK \u5185\u90E8\u9519\u8BEF\uFF1A\u641C\u7D22\u65B9\u5411\u4E3A\u96F6");
      const d1 = closest.dot(dir);
      const d2 = s[this.simplexSize].dot(dir);
      if (d2 - d1 < eps2) {
        this.interpolateClosestPoints();
        this.distance = closest.length();
        if (cache2 !== null && cache2.gjkCache !== null) cache2.gjkCache.prevClosestDir.copyFrom(closest).negateEq();
        return GjkEpaResultState.SUCCEEDED;
      }
      this.simplexSize++;
      count++;
    }
    return GjkEpaResultState.GJK_DID_NOT_CONVERGE;
  }
  /** c1 可以是 null：这时几何体 1 是 tf1 位置上的一个点（射线检测） */
  convexCastImpl(c1, c2, tf1, tf2, tl1, tl2, hit) {
    this.c1 = c1;
    this.c2 = c2;
    this.tf1 = tf1;
    this.tf2 = tf2;
    const s = this.s, closest = this.closest, dir = this.dir;
    dir.init(tf2.px - tf1.px, tf2.py - tf1.py, tf2.pz - tf1.pz);
    if (dir.lengthSq() < 1e-6) dir.init(1, 0, 0);
    this.simplexSize = 0;
    this.computeConvexCastSupportingVertex();
    this.simplexSize = 1;
    let lambda = 0;
    const rayX = this.rayX, rayR = this.rayR;
    rayX.zero();
    rayR.copyFrom(tl2).subEq(tl1);
    const eps2 = 1e-8;
    let count = 0;
    while (count < 40) {
      let v = 0;
      switch (this.simplexSize) {
        case 1:
          closest.copyFrom(s[0]);
          v = 1;
          break;
        case 2:
          v = projectOrigin2(s[0], s[1], closest);
          break;
        case 3:
          v = projectOrigin3(s[0], s[1], s[2], closest);
          break;
        case 4:
          v = projectOrigin4(s[0], s[1], s[2], s[3], closest);
          break;
      }
      this.shrinkSimplex(v);
      if (closest.lengthSq() < eps2) {
        if (lambda === 0 || this.simplexSize === 4) {
          hit.fraction = lambda;
          return false;
        }
        this.interpolateClosestPoints();
        hit.fraction = lambda;
        let nX = dir.x, nY = dir.y, nZ = dir.z;
        let invLen = Math.sqrt(nX * nX + nY * nY + nZ * nZ);
        if (invLen > 0) invLen = 1 / invLen;
        nX *= invLen;
        nY *= invLen;
        nZ *= invLen;
        hit.normalX = nX;
        hit.normalY = nY;
        hit.normalZ = nZ;
        let pX = this.closestPoint1.x, pY = this.closestPoint1.y, pZ = this.closestPoint1.z;
        pX += tl1.x * lambda;
        pY += tl1.y * lambda;
        pZ += tl1.z * lambda;
        hit.positionX = pX;
        hit.positionY = pY;
        hit.positionZ = pZ;
        return true;
      }
      dir.copyFrom(closest).negateEq();
      this.computeConvexCastSupportingVertex();
      s[this.simplexSize].subEq(rayX);
      if (dir.lengthSq() < eps2) throw new Error("GJK \u5185\u90E8\u9519\u8BEF\uFF1A\u641C\u7D22\u65B9\u5411\u4E3A\u96F6");
      const p = s[this.simplexSize], n = dir;
      const pn = p.dot(n);
      if (pn < 0) {
        if (rayR.dot(n) >= 0) return false;
        const dLambda = pn / rayR.dot(n);
        lambda += dLambda;
        if (lambda >= 1) return false;
        rayX.addScaledEq(rayR, dLambda);
        for (let i = 0; i < this.simplexSize + 1; i++) s[i].addScaledEq(rayR, -dLambda);
      }
      let duplicate = false;
      for (let i = 0; i < this.simplexSize; i++) {
        const dx = s[i].x - s[this.simplexSize].x;
        const dy = s[i].y - s[this.simplexSize].y;
        const dz = s[i].z - s[this.simplexSize].z;
        if (dx * dx + dy * dy + dz * dz < eps2) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) this.simplexSize++;
      count++;
    }
    return false;
  }
  /** 凸体扫掠的支撑点：两边都把外边距加回来（扫的是整个几何体）；c1 为 null 时几何体 1 是一个点 */
  computeConvexCastSupportingVertex() {
    const i = this.simplexSize;
    if (this.c1 !== null) {
      this.computeWitnessPoint1(true);
    } else {
      const tf = this.tf1;
      this.w1[i].init(tf.px, tf.py, tf.pz);
    }
    this.computeWitnessPoint2(true);
    this.s[i].copyFrom(this.w1[i]).subEq(this.w2[i]);
  }
  /** 沿 dir 求闵可夫斯基差上的支撑点，写到单纯形的第 simplexSize 个顶点 */
  computeSupportingVertex() {
    const i = this.simplexSize;
    this.computeWitnessPoint1(false);
    this.computeWitnessPoint2(false);
    this.s[i].copyFrom(this.w1[i]).subEq(this.w2[i]);
  }
  /** 几何体 1 沿 dir 的支撑点（世界坐标）。dir 临时改成局部方向，算完恢复 */
  computeWitnessPoint1(addMargin) {
    const tf = this.tf1, dir = this.dir, w = this.w1[this.simplexSize];
    const idirX = dir.x, idirY = dir.y, idirZ = dir.z;
    dir.init(
      tf.r00 * idirX + tf.r10 * idirY + tf.r20 * idirZ,
      tf.r01 * idirX + tf.r11 * idirY + tf.r21 * idirZ,
      tf.r02 * idirX + tf.r12 * idirY + tf.r22 * idirZ
    );
    this.c1.computeLocalSupportingVertex(dir, w);
    if (addMargin) {
      dir.normalize();
      w.addScaledEq(dir, this.c1.gjkMargin);
    }
    const tX = w.x, tY = w.y, tZ = w.z;
    let wX = tf.r00 * tX + tf.r01 * tY + tf.r02 * tZ;
    let wY = tf.r10 * tX + tf.r11 * tY + tf.r12 * tZ;
    let wZ = tf.r20 * tX + tf.r21 * tY + tf.r22 * tZ;
    wX += tf.px;
    wY += tf.py;
    wZ += tf.pz;
    w.init(wX, wY, wZ);
    dir.init(idirX, idirY, idirZ);
  }
  /** 几何体 2 沿 -dir 的支撑点（世界坐标） */
  computeWitnessPoint2(addMargin) {
    const tf = this.tf2, dir = this.dir, w = this.w2[this.simplexSize];
    const idirX = dir.x, idirY = dir.y, idirZ = dir.z;
    let lX = tf.r00 * idirX + tf.r10 * idirY + tf.r20 * idirZ;
    let lY = tf.r01 * idirX + tf.r11 * idirY + tf.r21 * idirZ;
    let lZ = tf.r02 * idirX + tf.r12 * idirY + tf.r22 * idirZ;
    lX = -lX;
    lY = -lY;
    lZ = -lZ;
    dir.init(lX, lY, lZ);
    this.c2.computeLocalSupportingVertex(dir, w);
    if (addMargin) {
      dir.normalize();
      w.addScaledEq(dir, this.c2.gjkMargin);
    }
    const tX = w.x, tY = w.y, tZ = w.z;
    let wX = tf.r00 * tX + tf.r01 * tY + tf.r02 * tZ;
    let wY = tf.r10 * tX + tf.r11 * tY + tf.r12 * tZ;
    let wZ = tf.r20 * tX + tf.r21 * tY + tf.r22 * tZ;
    wX += tf.px;
    wY += tf.py;
    wZ += tf.pz;
    w.init(wX, wY, wZ);
    dir.init(idirX, idirY, idirZ);
  }
  /** 用单纯形上最近点的重心坐标，插值出两个几何体上的最近点 */
  interpolateClosestPoints() {
    const s = this.s, w1 = this.w1, w2 = this.w2, c = this.closest;
    switch (this.simplexSize) {
      case 1:
        this.closestPoint1.copyFrom(w1[0]);
        this.closestPoint2.copyFrom(w2[0]);
        break;
      case 2: {
        const s01X = s[1].x - s[0].x, s01Y = s[1].y - s[0].y, s01Z = s[1].z - s[0].z;
        let invDet = s01X * s01X + s01Y * s01Y + s01Z * s01Z;
        if (invDet !== 0) invDet = 1 / invDet;
        const s0cX = c.x - s[0].x, s0cY = c.y - s[0].y, s0cZ = c.z - s[0].z;
        const t = (s0cX * s01X + s0cY * s01Y + s0cZ * s01Z) * invDet;
        let dX = w1[1].x - w1[0].x, dY = w1[1].y - w1[0].y, dZ = w1[1].z - w1[0].z;
        this.closestPoint1.init(w1[0].x + dX * t, w1[0].y + dY * t, w1[0].z + dZ * t);
        dX = w2[1].x - w2[0].x;
        dY = w2[1].y - w2[0].y;
        dZ = w2[1].z - w2[0].z;
        this.closestPoint2.init(w2[0].x + dX * t, w2[0].y + dY * t, w2[0].z + dZ * t);
        break;
      }
      case 3:
        this.interpolateOnTriangle(s[0], w1[0], w2[0], s[1], w1[1], w2[1], s[2], w1[2], w2[2]);
        break;
      default:
        throw new Error("GJK \u5185\u90E8\u9519\u8BEF\uFF1A\u5355\u7EAF\u5F62\u9876\u70B9\u6570\u4E0D\u5BF9");
    }
  }
  /** 按三角形重心坐标插值两个几何体上的最近点，写入 closestPoint1 和 closestPoint2。 */
  interpolateOnTriangle(s0, w10, w20, s1, w11, w21, s2, w12, w22) {
    const c = this.closest;
    const s01X = s1.x - s0.x, s01Y = s1.y - s0.y, s01Z = s1.z - s0.z;
    const s02X = s2.x - s0.x, s02Y = s2.y - s0.y, s02Z = s2.z - s0.z;
    const s0cX = c.x - s0.x, s0cY = c.y - s0.y, s0cZ = c.z - s0.z;
    const d11 = s01X * s01X + s01Y * s01Y + s01Z * s01Z;
    const d12 = s01X * s02X + s01Y * s02Y + s01Z * s02Z;
    const d22 = s02X * s02X + s02Y * s02Y + s02Z * s02Z;
    const d1c = s01X * s0cX + s01Y * s0cY + s01Z * s0cZ;
    const d2c = s02X * s0cX + s02Y * s0cY + s02Z * s0cZ;
    let invDet = d11 * d22 - d12 * d12;
    if (invDet !== 0) invDet = 1 / invDet;
    const u = (d1c * d22 - d2c * d12) * invDet;
    const t = (-d1c * d12 + d2c * d11) * invDet;
    let dX = w11.x - w10.x, dY = w11.y - w10.y, dZ = w11.z - w10.z;
    let pX = w10.x + dX * u, pY = w10.y + dY * u, pZ = w10.z + dZ * u;
    dX = w12.x - w10.x;
    dY = w12.y - w10.y;
    dZ = w12.z - w10.z;
    pX += dX * t;
    pY += dY * t;
    pZ += dZ * t;
    this.closestPoint1.init(pX, pY, pZ);
    dX = w21.x - w20.x;
    dY = w21.y - w20.y;
    dZ = w21.z - w20.z;
    pX = w20.x + dX * u;
    pY = w20.y + dY * u;
    pZ = w20.z + dZ * u;
    dX = w22.x - w20.x;
    dY = w22.y - w20.y;
    dZ = w22.z - w20.z;
    pX += dX * t;
    pY += dY * t;
    pZ += dZ * t;
    this.closestPoint2.init(pX, pY, pZ);
  }
  /** 按 Voronoi 区域的位掩码只保留用到的顶点，并将其移到数组前部 */
  shrinkSimplex(vertexBits) {
    let n = vertexBits;
    n = (n & 5) + (n >> 1 & 5);
    n = (n & 3) + (n >> 2 & 3);
    this.simplexSize = n;
    switch (vertexBits) {
      case 2:
        this.moveVertex(1, 0);
        break;
      case 4:
        this.moveVertex(2, 0);
        break;
      case 5:
        this.moveVertex(2, 1);
        break;
      case 6:
        this.moveVertex(2, 0);
        break;
      case 8:
        this.moveVertex(3, 0);
        break;
      case 9:
        this.moveVertex(3, 1);
        break;
      case 10:
        this.moveVertex(3, 0);
        break;
      case 11:
        this.moveVertex(3, 2);
        break;
      case 12:
        this.moveVertex(2, 0);
        this.moveVertex(3, 1);
        break;
      case 13:
        this.moveVertex(3, 1);
        break;
      case 14:
        this.moveVertex(3, 0);
        break;
    }
  }
  moveVertex(from, to) {
    this.s[to].copyFrom(this.s[from]);
    this.w1[to].copyFrom(this.w1[from]);
    this.w2[to].copyFrom(this.w2[from]);
  }
  // 单纯形退化（点、线段、三角形）时，沿几个方向找支撑点，把它补成体积不为 0 的四面体。
  // 找不到时 simplexSize 不到 4，调用方返回 GJK_FAILED_TO_MAKE_TETRAHEDRON。
  pointToTetrahedron() {
    for (let i = 0; i < 3; i++) {
      this.dir.copyFrom(this.baseDirs[i]);
      this.computeSupportingVertex();
      this.simplexSize++;
      this.lineToTetrahedron();
      if (this.simplexSize === 4) break;
      this.simplexSize--;
      this.dir.negateEq();
      this.computeSupportingVertex();
      this.simplexSize++;
      this.lineToTetrahedron();
      if (this.simplexSize === 4) break;
      this.simplexSize--;
    }
  }
  lineToTetrahedron() {
    const dir = this.dir, s = this.s;
    const oldX = dir.x, oldY = dir.y, oldZ = dir.z;
    const lX = s[0].x - s[1].x, lY = s[0].y - s[1].y, lZ = s[0].z - s[1].z;
    for (let i = 0; i < 3; i++) {
      const b = this.baseDirs[i];
      dir.init(lY * b.z - lZ * b.y, lZ * b.x - lX * b.z, lX * b.y - lY * b.x);
      this.computeSupportingVertex();
      this.simplexSize++;
      this.triangleToTetrahedron();
      if (this.simplexSize === 4) break;
      this.simplexSize--;
      dir.negateEq();
      this.computeSupportingVertex();
      this.simplexSize++;
      this.triangleToTetrahedron();
      if (this.simplexSize === 4) break;
      this.simplexSize--;
    }
    dir.init(oldX, oldY, oldZ);
  }
  triangleToTetrahedron() {
    const dir = this.dir, s = this.s;
    const oldX = dir.x, oldY = dir.y, oldZ = dir.z;
    const s01X = s[1].x - s[0].x, s01Y = s[1].y - s[0].y, s01Z = s[1].z - s[0].z;
    const s02X = s[2].x - s[0].x, s02Y = s[2].y - s[0].y, s02Z = s[2].z - s[0].z;
    dir.init(s01Y * s02Z - s01Z * s02Y, s01Z * s02X - s01X * s02Z, s01X * s02Y - s01Y * s02X);
    this.computeSupportingVertex();
    this.simplexSize++;
    if (!this.isValidTetrahedron()) {
      this.simplexSize--;
      dir.negateEq();
      this.computeSupportingVertex();
      this.simplexSize++;
      if (!this.isValidTetrahedron()) this.simplexSize--;
    }
    dir.init(oldX, oldY, oldZ);
  }
  /** 四面体的体积（行列式）不接近 0 */
  isValidTetrahedron() {
    const s = this.s;
    const e00 = s[1].x - s[0].x, e01 = s[1].y - s[0].y, e02 = s[1].z - s[0].z;
    const e10 = s[2].x - s[0].x, e11 = s[2].y - s[0].y, e12 = s[2].z - s[0].z;
    const e20 = s[3].x - s[0].x, e21 = s[3].y - s[0].y, e22 = s[3].z - s[0].z;
    const det2 = e00 * (e11 * e22 - e12 * e21) - e01 * (e10 * e22 - e12 * e20) + e02 * (e10 * e21 - e11 * e20);
    return det2 > 1e-12 || det2 < -1e-12;
  }
  /** EPA：从单纯形（四面体）开始扩张多面体，求穿透深度和最近点 */
  computeDepth() {
    const poly = this.polyhedron, s = this.s, w1 = this.w1, w2 = this.w2, dir = this.dir;
    poly.clear();
    const ok = poly.init(
      poly.pickVertex().init(s[0], w1[0], w2[0]),
      poly.pickVertex().init(s[1], w1[1], w2[1]),
      poly.pickVertex().init(s[2], w1[2], w2[2]),
      poly.pickVertex().init(s[3], w1[3], w2[3])
    );
    if (!ok) return GjkEpaResultState.EPA_FAILED_TO_INIT;
    this.simplexSize = 0;
    const supportingVertex = s[0], witness1 = w1[0], witness2 = w2[0];
    const maxIterations = 40;
    for (let count = 0; count < maxIterations; count++) {
      const face2 = poly.getBestTriangle();
      dir.copyFrom(face2.normal).normalize();
      this.computeSupportingVertex();
      const v0 = face2.vertices[0], v1 = face2.vertices[1], v2 = face2.vertices[2];
      const dot1 = v0.v.dot(dir);
      const dot2 = supportingVertex.dot(dir);
      if (dot2 - dot1 < 1e-6 || count === maxIterations - 1) {
        this.closest.copyFrom(dir).scaleEq(dir.dot(v0.v) / dir.lengthSq());
        this.interpolateOnTriangle(v0.v, v0.w1, v0.w2, v1.v, v1.w1, v1.w2, v2.v, v2.w1, v2.w2);
        this.depth = this.closest.length();
        return GjkEpaResultState.SUCCEEDED;
      }
      const epaVertex = poly.pickVertex().init(supportingVertex, witness1, witness2);
      if (!poly.addVertex(epaVertex, face2)) return GjkEpaResultState.EPA_FAILED_TO_ADD_VERTEX;
    }
    return GjkEpaResultState.EPA_DID_NOT_CONVERGE;
  }
};

// src/core/collision/geometry/geometry.ts
var GeometryType = {
  SPHERE: 0,
  BOX: 1,
  CYLINDER: 2,
  CONE: 3,
  CAPSULE: 4,
  CONVEX_HULL: 5
};
var Geometry = class {
  constructor(type) {
    this.volume = 0;
    /** 惯性张量除以质量（I / m），局部坐标。icIJ 是第 I 行第 J 列 */
    this.ic00 = 0;
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 0;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = 0;
    this.type = type;
  }
  /**
   * 世界坐标下的射线检测：几何体放在变换 tf 处，线段从 begin 到 end。
   * 命中时把世界坐标的结果写入 hit 并返回 true；没命中时返回 false，hit 不变。
   */
  rayCast(beginX, beginY, beginZ, endX, endY, endZ, tf, hit) {
    beginX -= tf.px;
    beginY -= tf.py;
    beginZ -= tf.pz;
    endX -= tf.px;
    endY -= tf.py;
    endZ -= tf.pz;
    const blX = tf.r00 * beginX + tf.r10 * beginY + tf.r20 * beginZ;
    const blY = tf.r01 * beginX + tf.r11 * beginY + tf.r21 * beginZ;
    const blZ = tf.r02 * beginX + tf.r12 * beginY + tf.r22 * beginZ;
    const elX = tf.r00 * endX + tf.r10 * endY + tf.r20 * endZ;
    const elY = tf.r01 * endX + tf.r11 * endY + tf.r21 * endZ;
    const elZ = tf.r02 * endX + tf.r12 * endY + tf.r22 * endZ;
    if (!this.rayCastLocal(blX, blY, blZ, elX, elY, elZ, hit)) return false;
    const lpX = hit.positionX, lpY = hit.positionY, lpZ = hit.positionZ;
    const lnX = hit.normalX, lnY = hit.normalY, lnZ = hit.normalZ;
    hit.positionX = tf.r00 * lpX + tf.r01 * lpY + tf.r02 * lpZ + tf.px;
    hit.positionY = tf.r10 * lpX + tf.r11 * lpY + tf.r12 * lpZ + tf.py;
    hit.positionZ = tf.r20 * lpX + tf.r21 * lpY + tf.r22 * lpZ + tf.pz;
    hit.normalX = tf.r00 * lnX + tf.r01 * lnY + tf.r02 * lnZ;
    hit.normalY = tf.r10 * lnX + tf.r11 * lnY + tf.r12 * lnZ;
    hit.normalZ = tf.r20 * lnX + tf.r21 * lnY + tf.r22 * lnZ;
    return true;
  }
};
var ConvexGeometry = class extends Geometry {
  constructor() {
    super(...arguments);
    /** GJK 外边距：几何体“核心”外面包的一层 */
    this.gjkMargin = DEFAULT_GJK_MARGIN;
    /** 射线检测用 GJK 做，而不是 rayCastLocal 的解析解（凸包没有解析解） */
    this.useGjkRayCast = false;
  }
  rayCast(beginX, beginY, beginZ, endX, endY, endZ, tf, hit) {
    if (this.useGjkRayCast) return GjkEpa.getInstance().rayCast(this, tf, beginX, beginY, beginZ, endX, endY, endZ, hit);
    return super.rayCast(beginX, beginY, beginZ, endX, endY, endZ, tf, hit);
  }
};

// src/core/collision/geometry/box-geometry.ts
var BoxGeometry = class extends ConvexGeometry {
  constructor(hx, hy, hz) {
    super(GeometryType.BOX);
    this.hx = hx;
    this.hy = hy;
    this.hz = hz;
    this.updateMass();
    const minHalfExtent = hx < hy ? hz < hx ? hz : hx : hz < hy ? hz : hy;
    if (this.gjkMargin > minHalfExtent * 0.2) this.gjkMargin = minHalfExtent * 0.2;
  }
  updateMass() {
    this.volume = 8 * (this.hx * this.hy * this.hz);
    const sqx = this.hx * this.hx;
    const sqy = this.hy * this.hy;
    const sqz = this.hz * this.hz;
    this.ic00 = 1 / 3 * (sqy + sqz);
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 1 / 3 * (sqz + sqx);
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = 1 / 3 * (sqx + sqy);
  }
  computeAabb(aabb, tf) {
    let a = tf.r00 * this.hx, b = tf.r01 * this.hy, c = tf.r02 * this.hz;
    const ex = (a < 0 ? -a : a) + (b < 0 ? -b : b) + (c < 0 ? -c : c);
    a = tf.r10 * this.hx;
    b = tf.r11 * this.hy;
    c = tf.r12 * this.hz;
    const ey = (a < 0 ? -a : a) + (b < 0 ? -b : b) + (c < 0 ? -c : c);
    a = tf.r20 * this.hx;
    b = tf.r21 * this.hy;
    c = tf.r22 * this.hz;
    const ez = (a < 0 ? -a : a) + (b < 0 ? -b : b) + (c < 0 ? -c : c);
    aabb.minX = tf.px - ex;
    aabb.minY = tf.py - ey;
    aabb.minZ = tf.pz - ez;
    aabb.maxX = tf.px + ex;
    aabb.maxY = tf.py + ey;
    aabb.maxZ = tf.pz + ez;
  }
  computeLocalSupportingVertex(dir, out) {
    let mx = this.gjkMargin, my = this.gjkMargin, mz = this.gjkMargin;
    if (!(mx < this.hx)) mx = this.hx;
    if (!(my < this.hy)) my = this.hy;
    if (!(mz < this.hz)) mz = this.hz;
    const cx = this.hx - mx, cy = this.hy - my, cz = this.hz - mz;
    out.x = dir.x > 0 ? cx : -cx;
    out.y = dir.y > 0 ? cy : -cy;
    out.z = dir.z > 0 ? cz : -cz;
  }
  rayCastLocal(beginX, beginY, beginZ, endX, endY, endZ, hit) {
    const hx = this.hx, hy = this.hy, hz = this.hz;
    const dx = endX - beginX;
    const dy = endY - beginY;
    const dz = endZ - beginZ;
    let tminX = 0, tminY = 0, tminZ = 0;
    let tmaxX = 1, tmaxY = 1, tmaxZ = 1;
    if (dx > -1e-6 && dx < 1e-6) {
      if (beginX <= -hx || beginX >= hx) return false;
    } else {
      const inv2 = 1 / dx;
      let t1 = (-hx - beginX) * inv2;
      let t2 = (hx - beginX) * inv2;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminX = t1;
      if (t2 < 1) tmaxX = t2;
    }
    if (dy > -1e-6 && dy < 1e-6) {
      if (beginY <= -hy || beginY >= hy) return false;
    } else {
      const inv2 = 1 / dy;
      let t1 = (-hy - beginY) * inv2;
      let t2 = (hy - beginY) * inv2;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminY = t1;
      if (t2 < 1) tmaxY = t2;
    }
    if (dz > -1e-6 && dz < 1e-6) {
      if (beginZ <= -hz || beginZ >= hz) return false;
    } else {
      const inv2 = 1 / dz;
      let t1 = (-hz - beginZ) * inv2;
      let t2 = (hz - beginZ) * inv2;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminZ = t1;
      if (t2 < 1) tmaxZ = t2;
    }
    if (tminX >= 1 || tminY >= 1 || tminZ >= 1 || tmaxX <= 0 || tmaxY <= 0 || tmaxZ <= 0) return false;
    let min = tminX;
    let max = tmaxX;
    let hitAxis = 0;
    if (tminY > min) {
      min = tminY;
      hitAxis = 1;
    }
    if (tminZ > min) {
      min = tminZ;
      hitAxis = 2;
    }
    if (tmaxY < max) max = tmaxY;
    if (tmaxZ < max) max = tmaxZ;
    if (min > max) return false;
    if (min === 0) return false;
    if (hitAxis === 0) {
      hit.normalX = dx > 0 ? -1 : 1;
      hit.normalY = 0;
      hit.normalZ = 0;
    } else if (hitAxis === 1) {
      hit.normalX = 0;
      hit.normalY = dy > 0 ? -1 : 1;
      hit.normalZ = 0;
    } else {
      hit.normalX = 0;
      hit.normalY = 0;
      hit.normalZ = dz > 0 ? -1 : 1;
    }
    hit.positionX = beginX + min * dx;
    hit.positionY = beginY + min * dy;
    hit.positionZ = beginZ + min * dz;
    hit.fraction = min;
    return true;
  }
};

// src/core/compat.ts
var compat = {
  /** 是否启用数值兼容行为，包括已知缺陷。 */
  keepOimoBugs: false
};

// src/core/collision/geometry/capsule-geometry.ts
var CapsuleGeometry = class extends ConvexGeometry {
  constructor(radius, halfHeight) {
    super(GeometryType.CAPSULE);
    this.radius = radius;
    this.halfHeight = halfHeight;
    this.gjkMargin = radius;
    this.updateMass();
  }
  updateMass() {
    const r2 = this.radius * this.radius;
    const hh2 = this.halfHeight * this.halfHeight;
    const cylinderVolume = 6.28318530717958 * r2 * this.halfHeight;
    const sphereVolume = 3.14159265358979 * r2 * this.radius * 4 / 3;
    this.volume = cylinderVolume + sphereVolume;
    const invVolume = this.volume === 0 ? 0 : 1 / this.volume;
    const inertiaXZ = invVolume * (cylinderVolume * (r2 * 0.25 + hh2 / 3) + sphereVolume * (r2 * 0.4 + this.halfHeight * this.radius * 0.75 + hh2));
    this.ic00 = inertiaXZ;
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = invVolume * (cylinderVolume * r2 * 0.5 + sphereVolume * r2 * 0.4);
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = inertiaXZ;
  }
  computeAabb(aabb, tf) {
    let ax = tf.r01, ay = tf.r11, az = tf.r21;
    if (ax < 0) ax = -ax;
    if (ay < 0) ay = -ay;
    if (az < 0) az = -az;
    ax *= this.halfHeight;
    ay *= this.halfHeight;
    az *= this.halfHeight;
    let ex = this.radius, ey = this.radius, ez = this.radius;
    ex += ax;
    ey += ay;
    ez += az;
    aabb.minX = tf.px - ex;
    aabb.minY = tf.py - ey;
    aabb.minZ = tf.pz - ez;
    aabb.maxX = tf.px + ex;
    aabb.maxY = tf.py + ey;
    aabb.maxZ = tf.pz + ez;
  }
  computeLocalSupportingVertex(dir, out) {
    if (dir.y > 0) out.init(0, this.halfHeight, 0);
    else out.init(0, -this.halfHeight, 0);
  }
  rayCastLocal(beginX, beginY, beginZ, endX, endY, endZ, hit) {
    const halfH = this.halfHeight;
    const dx = endX - beginX;
    const dz = endZ - beginZ;
    let tminxz;
    let a = dx * dx + dz * dz;
    let b = beginX * dx + beginZ * dz;
    let c = beginX * beginX + beginZ * beginZ - this.radius * this.radius;
    let D = b * b - a * c;
    if (D < 0) return false;
    if (a > 0) {
      const sqrtD = Math.sqrt(D);
      tminxz = (-b - sqrtD) / a;
      const tmaxxz = (-b + sqrtD) / a;
      if (tminxz >= 1 || tmaxxz <= 0) return false;
    } else {
      if (c >= 0) return false;
      tminxz = 0;
    }
    const crossY = beginY + (endY - beginY) * tminxz;
    if (crossY > -halfH && crossY < halfH) {
      if (tminxz > 0) {
        const min = tminxz;
        const nx = beginX + dx * min, nz = beginZ + dz * min;
        let invLen = Math.sqrt(nx * nx + 0 * 0 + nz * nz);
        if (invLen > 0) invLen = 1 / invLen;
        hit.normalX = nx * invLen;
        hit.normalY = 0 * invLen;
        hit.normalZ = nz * invLen;
        hit.positionX = beginX + min * dx;
        hit.positionY = crossY;
        hit.positionZ = beginZ + min * dz;
        hit.fraction = min;
        return true;
      }
      return false;
    }
    const sphereY = crossY < 0 ? -halfH : halfH;
    const sbX = beginX - 0;
    const sbY = beginY - sphereY;
    const sbZ = beginZ - 0;
    const dX = endX - beginX;
    const dY = endY - beginY;
    const dZ = endZ - beginZ;
    a = dX * dX + dY * dY + dZ * dZ;
    b = sbX * dX + sbY * dY + sbZ * dZ;
    c = sbX * sbX + sbY * sbY + sbZ * sbZ - this.radius * this.radius;
    D = b * b - a * c;
    if (D < 0) return false;
    if (a === 0 && !compat.keepOimoBugs) return false;
    const t = (-b - Math.sqrt(D)) / a;
    if (t < 0 || t > 1) return false;
    let pX = sbX + dX * t;
    let pY = sbY + dY * t;
    let pZ = sbZ + dZ * t;
    let l = pX * pX + pY * pY + pZ * pZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    hit.normalX = pX * l;
    hit.normalY = pY * l;
    hit.normalZ = pZ * l;
    pX += 0;
    pY += sphereY;
    pZ += 0;
    hit.positionX = pX;
    hit.positionY = pY;
    hit.positionZ = pZ;
    hit.fraction = t;
    return true;
  }
};

// src/core/collision/geometry/cone-geometry.ts
var ConeGeometry = class extends ConvexGeometry {
  constructor(radius, halfHeight) {
    super(GeometryType.CONE);
    this.radius = radius;
    this.halfHeight = halfHeight;
    this.sinTheta = radius / Math.sqrt(radius * radius + 4 * halfHeight * halfHeight);
    this.cosTheta = 2 * halfHeight / Math.sqrt(radius * radius + 4 * halfHeight * halfHeight);
    this.updateMass();
  }
  updateMass() {
    const r2 = this.radius * this.radius;
    const h2 = this.halfHeight * this.halfHeight * 4;
    this.volume = 3.14159265358979 * r2 * this.halfHeight * 2 / 3;
    this.ic00 = 0.05 * (3 * r2 + 2 * h2);
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 0.3 * r2;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = 0.05 * (3 * r2 + 2 * h2);
  }
  computeAabb(aabb, tf) {
    const ax = tf.r01, ay = tf.r11, az = tf.r21;
    const ax2 = ax * ax, ay2 = ay * ay, az2 = az * az;
    let erX = Math.sqrt(1 - ax2), erY = Math.sqrt(1 - ay2), erZ = Math.sqrt(1 - az2);
    erX *= this.radius;
    erY *= this.radius;
    erZ *= this.radius;
    const ehX = ax * this.halfHeight, ehY = ay * this.halfHeight, ehZ = az * this.halfHeight;
    let rminX = -ehX, rminY = -ehY, rminZ = -ehZ;
    rminX -= erX;
    rminY -= erY;
    rminZ -= erZ;
    let rmaxX = -ehX, rmaxY = -ehY, rmaxZ = -ehZ;
    rmaxX += erX;
    rmaxY += erY;
    rmaxZ += erZ;
    let maxX = rminX > rmaxX ? rminX : rmaxX;
    let maxY = rminY > rmaxY ? rminY : rmaxY;
    let maxZ = rminZ > rmaxZ ? rminZ : rmaxZ;
    if (!(maxX > ehX)) maxX = ehX;
    if (!(maxY > ehY)) maxY = ehY;
    if (!(maxZ > ehZ)) maxZ = ehZ;
    let minX = rminX < rmaxX ? rminX : rmaxX;
    let minY = rminY < rmaxY ? rminY : rmaxY;
    let minZ = rminZ < rmaxZ ? rminZ : rmaxZ;
    if (!(minX < ehX)) minX = ehX;
    if (!(minY < ehY)) minY = ehY;
    if (!(minZ < ehZ)) minZ = ehZ;
    aabb.minX = tf.px + minX;
    aabb.minY = tf.py + minY;
    aabb.minZ = tf.pz + minZ;
    aabb.maxX = tf.px + maxX;
    aabb.maxY = tf.py + maxY;
    aabb.maxZ = tf.pz + maxZ;
  }
  computeLocalSupportingVertex(dir, out) {
    const dx = dir.x, dy = dir.y, dz = dir.z;
    if (dy > 0 && dy * dy > this.sinTheta * this.sinTheta * (dx * dx + dy * dy + dz * dz)) {
      out.init(0, this.halfHeight - this.gjkMargin / this.sinTheta, 0);
      if (out.y < 0) out.y = 0;
      return;
    }
    const rx = dir.x, rz = dir.z;
    const len = rx * rx + rz * rz;
    const height = 2 * this.halfHeight;
    let coreRadius = (height - this.gjkMargin) / height * this.radius - this.gjkMargin / this.cosTheta;
    if (coreRadius < 0) coreRadius = 0;
    const invLen = len > 0 ? coreRadius / Math.sqrt(len) : 0;
    let coreHalfHeight = this.halfHeight - this.gjkMargin;
    if (coreHalfHeight < 0) coreHalfHeight = 0;
    out.x = rx * invLen;
    out.y = -coreHalfHeight;
    out.z = rz * invLen;
  }
  rayCastLocal(beginX, beginY, beginZ, endX, endY, endZ, hit) {
    const halfH = this.halfHeight;
    const dx = endX - beginX;
    const dy = endY - beginY;
    const dz = endZ - beginZ;
    let tminy = 0;
    let tmaxy = 1;
    if (dy > -1e-6 && dy < 1e-6) {
      if (beginY <= -halfH || beginY >= halfH) return false;
    } else {
      const inv2 = 1 / dy;
      let t1 = (-halfH - beginY) * inv2;
      let t2 = (halfH - beginY) * inv2;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminy = t1;
      if (t2 < 1) tmaxy = t2;
    }
    if (tminy >= 1 || tmaxy <= 0) return false;
    let tminxz;
    let tmaxxz;
    let p1y = beginY - halfH;
    const cos2 = this.cosTheta * this.cosTheta;
    const a = cos2 * (dx * dx + dy * dy + dz * dz) - dy * dy;
    const b = cos2 * (beginX * dx + p1y * dy + beginZ * dz) - p1y * dy;
    const c = cos2 * (beginX * beginX + p1y * p1y + beginZ * beginZ) - p1y * p1y;
    const D = b * b - a * c;
    if (a !== 0) {
      if (D < 0) return false;
      const sqrtD = Math.sqrt(D);
      if (a < 0) {
        if (dy > 0) {
          tminxz = 0;
          tmaxxz = (-b + sqrtD) / a;
          if (tmaxxz <= 0) return false;
        } else {
          tminxz = (-b - sqrtD) / a;
          tmaxxz = 1;
          if (tminxz >= 1) return false;
        }
      } else {
        tminxz = (-b - sqrtD) / a;
        tmaxxz = (-b + sqrtD) / a;
        if (tminxz >= 1 || tmaxxz <= 0) return false;
      }
    } else {
      const t = -c / (2 * b);
      if (b > 0) {
        tminxz = 0;
        tmaxxz = t;
        if (t <= 0) return false;
      } else {
        tminxz = t;
        tmaxxz = 1;
        if (t >= 1) return false;
      }
    }
    p1y += halfH;
    if (tmaxxz <= tminy || tmaxy <= tminxz) return false;
    let min;
    if (tminxz < tminy) {
      min = tminy;
      if (min === 0) return false;
      hit.normalX = 0;
      hit.normalY = dy > 0 ? -1 : 1;
      hit.normalZ = 0;
    } else {
      min = tminxz;
      if (min === 0) return false;
      let nx = beginX + dx * min, ny = 0, nz = beginZ + dz * min;
      let invLen = Math.sqrt(nx * nx + ny * ny + nz * nz);
      if (invLen > 0) invLen = 1 / invLen;
      nx *= invLen;
      ny *= invLen;
      nz *= invLen;
      const s = this.cosTheta;
      nx *= s;
      ny *= s;
      nz *= s;
      ny += this.sinTheta;
      hit.normalX = nx;
      hit.normalY = ny;
      hit.normalZ = nz;
    }
    hit.positionX = beginX + min * dx;
    hit.positionY = p1y + min * dy;
    hit.positionZ = beginZ + min * dz;
    hit.fraction = min;
    return true;
  }
};

// src/core/collision/geometry/convex-hull-geometry.ts
var ConvexHullGeometry = class extends ConvexGeometry {
  constructor(vertices) {
    super(GeometryType.CONVEX_HULL);
    this.vertices = vertices;
    this.useGjkRayCast = true;
    this.updateMass();
  }
  updateMass() {
    const v = this.vertices;
    let minx = v[0].x, miny = v[0].y, minz = v[0].z;
    let maxx = v[0].x, maxy = v[0].y, maxz = v[0].z;
    for (let i = 1; i < v.length; i++) {
      const vx = v[i].x, vy = v[i].y, vz = v[i].z;
      if (vx < minx) minx = vx;
      else if (vx > maxx) maxx = vx;
      if (vy < miny) miny = vy;
      else if (vy > maxy) maxy = vy;
      if (vz < minz) minz = vz;
      else if (vz > maxz) maxz = vz;
    }
    let sizex = maxx - minx, sizey = maxy - miny, sizez = maxz - minz;
    this.volume = sizex * sizey * sizez;
    const diffCog = ((minx + maxx) * (minx + maxx) + (miny + maxy) * (miny + maxy) + (minz + maxz) * (minz + maxz)) * 0.25;
    sizex = sizex * sizex * 0.25;
    sizey = sizey * sizey * 0.25;
    sizez = sizez * sizez * 0.25;
    this.ic00 = 1 / 3 * (sizey + sizez) + diffCog;
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 1 / 3 * (sizez + sizex) + diffCog;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = 1 / 3 * (sizex + sizey) + diffCog;
  }
  computeAabb(aabb, tf) {
    const v = this.vertices, m = this.gjkMargin;
    let lx = v[0].x, ly = v[0].y, lz = v[0].z;
    let wx = tf.r00 * lx + tf.r01 * ly + tf.r02 * lz;
    let wy = tf.r10 * lx + tf.r11 * ly + tf.r12 * lz;
    let wz = tf.r20 * lx + tf.r21 * ly + tf.r22 * lz;
    wx += tf.px;
    wy += tf.py;
    wz += tf.pz;
    let minX = wx, minY = wy, minZ = wz, maxX = wx, maxY = wy, maxZ = wz;
    for (let i = 1; i < v.length; i++) {
      lx = v[i].x;
      ly = v[i].y;
      lz = v[i].z;
      wx = tf.r00 * lx + tf.r01 * ly + tf.r02 * lz;
      wy = tf.r10 * lx + tf.r11 * ly + tf.r12 * lz;
      wz = tf.r20 * lx + tf.r21 * ly + tf.r22 * lz;
      wx += tf.px;
      wy += tf.py;
      wz += tf.pz;
      if (!(minX < wx)) minX = wx;
      if (!(minY < wy)) minY = wy;
      if (!(minZ < wz)) minZ = wz;
      if (!(maxX > wx)) maxX = wx;
      if (!(maxY > wy)) maxY = wy;
      if (!(maxZ > wz)) maxZ = wz;
    }
    aabb.minX = minX - m;
    aabb.minY = minY - m;
    aabb.minZ = minZ - m;
    aabb.maxX = maxX + m;
    aabb.maxY = maxY + m;
    aabb.maxZ = maxZ + m;
  }
  computeLocalSupportingVertex(dir, out) {
    const v = this.vertices;
    let maxDot = v[0].x * dir.x + v[0].y * dir.y + v[0].z * dir.z;
    let maxIndex = 0;
    for (let i = 1; i < v.length; i++) {
      const dot = v[i].x * dir.x + v[i].y * dir.y + v[i].z * dir.z;
      if (dot > maxDot) {
        maxDot = dot;
        maxIndex = i;
      }
    }
    out.copyFrom(v[maxIndex]);
  }
  /** 射线检测由 GJK 处理，此解析接口返回 false。 */
  rayCastLocal() {
    return false;
  }
};

// src/core/collision/geometry/cylinder-geometry.ts
var CylinderGeometry = class extends ConvexGeometry {
  constructor(radius, halfHeight) {
    super(GeometryType.CYLINDER);
    this.radius = radius;
    this.halfHeight = halfHeight;
    this.updateMass();
  }
  updateMass() {
    const r2 = this.radius * this.radius;
    const h2 = this.halfHeight * this.halfHeight * 4;
    this.volume = 3.14159265358979 * r2 * this.halfHeight * 2;
    this.ic00 = 1 / 12 * (3 * r2 + h2);
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 0.5 * r2;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = 1 / 12 * (3 * r2 + h2);
  }
  computeAabb(aabb, tf) {
    let ax = tf.r01, ay = tf.r11, az = tf.r21;
    if (ax < 0) ax = -ax;
    if (ay < 0) ay = -ay;
    if (az < 0) az = -az;
    const ax2 = ax * ax, ay2 = ay * ay, az2 = az * az;
    let erX = Math.sqrt(1 - ax2), erY = Math.sqrt(1 - ay2), erZ = Math.sqrt(1 - az2);
    erX *= this.radius;
    erY *= this.radius;
    erZ *= this.radius;
    const ehX = ax * this.halfHeight, ehY = ay * this.halfHeight, ehZ = az * this.halfHeight;
    const maxX = erX + ehX, maxY = erY + ehY, maxZ = erZ + ehZ;
    aabb.minX = tf.px - maxX;
    aabb.minY = tf.py - maxY;
    aabb.minZ = tf.pz - maxZ;
    aabb.maxX = tf.px + maxX;
    aabb.maxY = tf.py + maxY;
    aabb.maxZ = tf.pz + maxZ;
  }
  computeLocalSupportingVertex(dir, out) {
    const rx = dir.x, rz = dir.z;
    const len = rx * rx + rz * rz;
    let coreRadius = this.radius - this.gjkMargin;
    if (coreRadius < 0) coreRadius = 0;
    const invLen = len > 0 ? coreRadius / Math.sqrt(len) : 0;
    let coreHeight = this.halfHeight - this.gjkMargin;
    if (coreHeight < 0) coreHeight = 0;
    out.x = rx * invLen;
    out.y = dir.y > 0 ? coreHeight : -coreHeight;
    out.z = rz * invLen;
  }
  rayCastLocal(beginX, beginY, beginZ, endX, endY, endZ, hit) {
    const halfH = this.halfHeight;
    const dx = endX - beginX;
    const dy = endY - beginY;
    const dz = endZ - beginZ;
    let tminy = 0;
    let tmaxy = 1;
    if (dy > -1e-6 && dy < 1e-6) {
      if (beginY <= -halfH || beginY >= halfH) return false;
    } else {
      const inv2 = 1 / dy;
      let t1 = (-halfH - beginY) * inv2;
      let t2 = (halfH - beginY) * inv2;
      if (t1 > t2) {
        const tmp = t1;
        t1 = t2;
        t2 = tmp;
      }
      if (t1 > 0) tminy = t1;
      if (t2 < 1) tmaxy = t2;
    }
    if (tminy >= 1 || tmaxy <= 0) return false;
    let tminxz;
    let tmaxxz;
    const a = dx * dx + dz * dz;
    const b = beginX * dx + beginZ * dz;
    const c = beginX * beginX + beginZ * beginZ - this.radius * this.radius;
    const D = b * b - a * c;
    if (D < 0) return false;
    if (a > 0) {
      const sqrtD = Math.sqrt(D);
      tminxz = (-b - sqrtD) / a;
      tmaxxz = (-b + sqrtD) / a;
      if (tminxz >= 1 || tmaxxz <= 0) return false;
    } else {
      if (c >= 0) return false;
      tminxz = 0;
      tmaxxz = 1;
    }
    if (tmaxxz <= tminy || tmaxy <= tminxz) return false;
    let min;
    if (tminxz < tminy) {
      min = tminy;
      if (min === 0) return false;
      hit.normalX = 0;
      hit.normalY = dy > 0 ? -1 : 1;
      hit.normalZ = 0;
    } else {
      min = tminxz;
      if (min === 0) return false;
      const nx = beginX + dx * min, nz = beginZ + dz * min;
      let invLen = Math.sqrt(nx * nx + 0 * 0 + nz * nz);
      if (invLen > 0) invLen = 1 / invLen;
      hit.normalX = nx * invLen;
      hit.normalY = 0 * invLen;
      hit.normalZ = nz * invLen;
    }
    hit.positionX = beginX + min * dx;
    hit.positionY = beginY + min * dy;
    hit.positionZ = beginZ + min * dz;
    hit.fraction = min;
    return true;
  }
};

// src/core/collision/geometry/sphere-geometry.ts
var SphereGeometry = class extends ConvexGeometry {
  constructor(radius) {
    super(GeometryType.SPHERE);
    this.radius = radius;
    this.gjkMargin = radius;
    this.updateMass();
  }
  updateMass() {
    this.volume = 4.188790204786386 * this.radius * this.radius * this.radius;
    this.ic00 = 0.4 * this.radius * this.radius;
    this.ic01 = 0;
    this.ic02 = 0;
    this.ic10 = 0;
    this.ic11 = 0.4 * this.radius * this.radius;
    this.ic12 = 0;
    this.ic20 = 0;
    this.ic21 = 0;
    this.ic22 = 0.4 * this.radius * this.radius;
  }
  computeAabb(aabb, tf) {
    const r = this.radius;
    aabb.minX = tf.px - r;
    aabb.minY = tf.py - r;
    aabb.minZ = tf.pz - r;
    aabb.maxX = tf.px + r;
    aabb.maxY = tf.py + r;
    aabb.maxZ = tf.pz + r;
  }
  computeLocalSupportingVertex(_dir, out) {
    out.zero();
  }
  rayCastLocal(beginX, beginY, beginZ, endX, endY, endZ, hit) {
    const dX = endX - beginX;
    const dY = endY - beginY;
    const dZ = endZ - beginZ;
    const a = dX * dX + dY * dY + dZ * dZ;
    const b = beginX * dX + beginY * dY + beginZ * dZ;
    const D = b * b - a * (beginX * beginX + beginY * beginY + beginZ * beginZ - this.radius * this.radius);
    if (D < 0) return false;
    if (a === 0 && !compat.keepOimoBugs) return false;
    const t = (-b - Math.sqrt(D)) / a;
    if (t < 0 || t > 1) return false;
    const pX = beginX + dX * t;
    const pY = beginY + dY * t;
    const pZ = beginZ + dZ * t;
    let l = pX * pX + pY * pY + pZ * pZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    hit.positionX = pX;
    hit.positionY = pY;
    hit.positionZ = pZ;
    hit.normalX = pX * l;
    hit.normalY = pY * l;
    hit.normalZ = pZ * l;
    hit.fraction = t;
    return true;
  }
};

// src/core/common/mat3.ts
var Mat3 = class {
  /** 默认是单位矩阵 */
  constructor(e00 = 1, e01 = 0, e02 = 0, e10 = 0, e11 = 1, e12 = 0, e20 = 0, e21 = 0, e22 = 1) {
    this.e00 = e00;
    this.e01 = e01;
    this.e02 = e02;
    this.e10 = e10;
    this.e11 = e11;
    this.e12 = e12;
    this.e20 = e20;
    this.e21 = e21;
    this.e22 = e22;
  }
  init(e00, e01, e02, e10, e11, e12, e20, e21, e22) {
    this.e00 = e00;
    this.e01 = e01;
    this.e02 = e02;
    this.e10 = e10;
    this.e11 = e11;
    this.e12 = e12;
    this.e20 = e20;
    this.e21 = e21;
    this.e22 = e22;
    return this;
  }
};

// src/core/dynamics/constraint/joint/joint-config.ts
var ConstraintSolverType = {
  /** 迭代求解（PGS） */
  ITERATIVE: 0,
  /** 直接求解（更精确、更慢） */
  DIRECT: 1
};
var JointConfig = class {
  constructor() {
    this.rigidBody1 = null;
    this.rigidBody2 = null;
    /** 锚点在两个刚体上的局部坐标 */
    this.localAnchor1 = new Vec3();
    this.localAnchor2 = new Vec3();
    /** 是否允许两个刚体互相碰撞 */
    this.allowCollision = false;
    this.solverType = ConstraintSolverType.ITERATIVE;
    /** 约束力超过它时关节断开，0 表示不会断 */
    this.breakForce = 0;
    /** 约束力矩超过它时关节断开，0 表示不会断 */
    this.breakTorque = 0;
  }
  /** 用世界坐标的锚点设置两个刚体和局部锚点 */
  initBase(rb1, rb2, anchorX, anchorY, anchorZ) {
    this.rigidBody1 = rb1;
    this.rigidBody2 = rb2;
    toLocalPoint(rb1, anchorX, anchorY, anchorZ, this.localAnchor1);
    toLocalPoint(rb2, anchorX, anchorY, anchorZ, this.localAnchor2);
  }
};
function toLocalVector(rb, x, y, z, out) {
  const tf = rb.transform;
  out.init(
    tf.r00 * x + tf.r10 * y + tf.r20 * z,
    tf.r01 * x + tf.r11 * y + tf.r21 * z,
    tf.r02 * x + tf.r12 * y + tf.r22 * z
  );
}
function toLocalPoint(rb, x, y, z, out) {
  const tf = rb.transform;
  const dX = x - tf.px, dY = y - tf.py, dZ = z - tf.pz;
  out.init(
    tf.r00 * dX + tf.r10 * dY + tf.r20 * dZ,
    tf.r01 * dX + tf.r11 * dY + tf.r21 * dZ,
    tf.r02 * dX + tf.r12 * dY + tf.r22 * dZ
  );
}

// src/core/dynamics/constraint/joint/basis-tracker.ts
var BasisTracker = class {
  constructor(joint) {
    this.xX = 0;
    this.xY = 0;
    this.xZ = 0;
    this.yX = 0;
    this.yY = 0;
    this.yZ = 0;
    this.zX = 0;
    this.zY = 0;
    this.zZ = 0;
    this.joint = joint;
  }
  /** 以两个刚体的 X 轴为准更新坐标系 */
  trackByX() {
    const j = this.joint;
    const invM1 = j.b1.invMass, invM2 = j.b2.invMass;
    const x1X = j.bX1X, x1Y = j.bX1Y, x1Z = j.bX1Z;
    const x2X = j.bX2X, x2Y = j.bX2Y, x2Z = j.bX2Z;
    let qX, qY, qZ, qW;
    let d = x1X * x2X + x1Y * x2Y + x1Z * x2Z;
    if (d < -0.999999999) {
      const x22 = x1X * x1X, y22 = x1Y * x1Y, z22 = x1Z * x1Z;
      let e;
      if (x22 < y22) {
        if (x22 < z22) {
          e = 1 / Math.sqrt(y22 + z22);
          qX = 0;
          qY = x1Z * e;
          qZ = -x1Y * e;
        } else {
          e = 1 / Math.sqrt(x22 + y22);
          qX = x1Y * e;
          qY = -x1X * e;
          qZ = 0;
        }
      } else if (y22 < z22) {
        e = 1 / Math.sqrt(z22 + x22);
        qX = -x1Z * e;
        qY = 0;
        qZ = x1X * e;
      } else {
        e = 1 / Math.sqrt(x22 + y22);
        qX = x1Y * e;
        qY = -x1X * e;
        qZ = 0;
      }
      qW = 0;
    } else {
      let cX = x1Y * x2Z - x1Z * x2Y, cY = x1Z * x2X - x1X * x2Z, cZ = x1X * x2Y - x1Y * x2X;
      const w = Math.sqrt((1 + d) * 0.5);
      d = 0.5 / w;
      cX *= d;
      cY *= d;
      cZ *= d;
      qX = cX;
      qY = cY;
      qZ = cZ;
      qW = w;
    }
    const ratio = invM1 + invM2 === 0 && !compat.keepOimoBugs ? 0.5 : invM1 / (invM1 + invM2);
    let q1X = 0, q1Y = 0, q1Z = 0, q1W = 1;
    let q2X = qX, q2Y = qY, q2Z = qZ, q2W = qW;
    let d1 = q1X * q2X + q1Y * q2Y + q1Z * q2Z + q1W * q2W;
    if (d1 < 0) {
      d1 = -d1;
      q2X = -q2X;
      q2Y = -q2Y;
      q2Z = -q2Z;
      q2W = -q2W;
    }
    let sX, sY, sZ, sW;
    if (d1 > 0.999999) {
      const dqX = q2X - q1X, dqY = q2Y - q1Y, dqZ = q2Z - q1Z, dqW = q2W - q1W;
      q2X = q1X + dqX * ratio;
      q2Y = q1Y + dqY * ratio;
      q2Z = q1Z + dqZ * ratio;
      q2W = q1W + dqW * ratio;
      let l = q2X * q2X + q2Y * q2Y + q2Z * q2Z + q2W * q2W;
      if (l > 1e-32) l = 1 / Math.sqrt(l);
      sX = q2X * l;
      sY = q2Y * l;
      sZ = q2Z * l;
      sW = q2W * l;
    } else {
      const theta = ratio * Math.acos(d1);
      q2X += q1X * -d1;
      q2Y += q1Y * -d1;
      q2Z += q1Z * -d1;
      q2W += q1W * -d1;
      let l = q2X * q2X + q2Y * q2Y + q2Z * q2Z + q2W * q2W;
      if (l > 1e-32) l = 1 / Math.sqrt(l);
      q2X *= l;
      q2Y *= l;
      q2Z *= l;
      q2W *= l;
      const sin = Math.sin(theta), cos = Math.cos(theta);
      q1X *= cos;
      q1Y *= cos;
      q1Z *= cos;
      q1W *= cos;
      sX = q1X + q2X * sin;
      sY = q1Y + q2Y * sin;
      sZ = q1Z + q2Z * sin;
      sW = q1W + q2W * sin;
    }
    let x2 = 2 * sX, y2 = 2 * sY, z2 = 2 * sZ;
    let xx = sX * x2, yy = sY * y2, zz = sZ * z2;
    let xy = sX * y2, yz = sY * z2, xz = sX * z2;
    let wx = sW * x2, wy = sW * y2, wz = sW * z2;
    let m00 = 1 - yy - zz, m01 = xy - wz, m02 = xz + wy;
    let m10 = xy + wz, m11 = 1 - xx - zz, m12 = yz - wx;
    let m20 = xz - wy, m21 = yz + wx, m22 = 1 - xx - yy;
    const newXX = m00 * x1X + m01 * x1Y + m02 * x1Z;
    const newXY = m10 * x1X + m11 * x1Y + m12 * x1Z;
    const newXZ = m20 * x1X + m21 * x1Y + m22 * x1Z;
    const prevXX = this.xX, prevXY = this.xY, prevXZ = this.xZ;
    const prevYX = this.yX, prevYY = this.yY, prevYZ = this.yZ;
    let d2 = prevXX * newXX + prevXY * newXY + prevXZ * newXZ;
    if (d2 < -0.999999999) {
      const px2 = prevXX * prevXX, py2 = prevXY * prevXY, pz2 = prevXZ * prevXZ;
      let e;
      if (px2 < py2) {
        if (px2 < pz2) {
          e = 1 / Math.sqrt(py2 + pz2);
          sX = 0;
          sY = prevXZ * e;
          sZ = -prevXY * e;
        } else {
          e = 1 / Math.sqrt(px2 + py2);
          sX = prevXY * e;
          sY = -prevXX * e;
          sZ = 0;
        }
      } else if (py2 < pz2) {
        e = 1 / Math.sqrt(pz2 + px2);
        sX = -prevXZ * e;
        sY = 0;
        sZ = prevXX * e;
      } else {
        e = 1 / Math.sqrt(px2 + py2);
        sX = prevXY * e;
        sY = -prevXX * e;
        sZ = 0;
      }
      sW = 0;
    } else {
      let cX = prevXY * newXZ - prevXZ * newXY, cY = prevXZ * newXX - prevXX * newXZ, cZ = prevXX * newXY - prevXY * newXX;
      const w = Math.sqrt((1 + d2) * 0.5);
      d2 = 0.5 / w;
      cX *= d2;
      cY *= d2;
      cZ *= d2;
      sX = cX;
      sY = cY;
      sZ = cZ;
      sW = w;
    }
    x2 = 2 * sX;
    y2 = 2 * sY;
    z2 = 2 * sZ;
    xx = sX * x2;
    yy = sY * y2;
    zz = sZ * z2;
    xy = sX * y2;
    yz = sY * z2;
    xz = sX * z2;
    wx = sW * x2;
    wy = sW * y2;
    wz = sW * z2;
    m00 = 1 - yy - zz;
    m01 = xy - wz;
    m02 = xz + wy;
    m10 = xy + wz;
    m11 = 1 - xx - zz;
    m12 = yz - wx;
    m20 = xz - wy;
    m21 = yz + wx;
    m22 = 1 - xx - yy;
    let newYX = m00 * prevYX + m01 * prevYY + m02 * prevYZ;
    let newYY = m10 * prevYX + m11 * prevYY + m12 * prevYZ;
    let newYZ = m20 * prevYX + m21 * prevYY + m22 * prevYZ;
    let newZX = newXY * newYZ - newXZ * newYY;
    let newZY = newXZ * newYX - newXX * newYZ;
    let newZZ = newXX * newYY - newXY * newYX;
    if (newZX * newZX + newZY * newZY + newZZ * newZZ > 1e-6) {
      let l = newZX * newZX + newZY * newZY + newZZ * newZZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      newZX *= l;
      newZY *= l;
      newZZ *= l;
    } else {
      const nx2 = newXX * newXX, ny2 = newXY * newXY, nz2 = newXZ * newXZ;
      let e;
      if (nx2 < ny2) {
        if (nx2 < nz2) {
          e = 1 / Math.sqrt(ny2 + nz2);
          newZX = 0;
          newZY = newXZ * e;
          newZZ = -newXY * e;
        } else {
          e = 1 / Math.sqrt(nx2 + ny2);
          newZX = newXY * e;
          newZY = -newXX * e;
          newZZ = 0;
        }
      } else if (ny2 < nz2) {
        e = 1 / Math.sqrt(nz2 + nx2);
        newZX = -newXZ * e;
        newZY = 0;
        newZZ = newXX * e;
      } else {
        e = 1 / Math.sqrt(nx2 + ny2);
        newZX = newXY * e;
        newZY = -newXX * e;
        newZZ = 0;
      }
    }
    newYX = newZY * newXZ - newZZ * newXY;
    newYY = newZZ * newXX - newZX * newXZ;
    newYZ = newZX * newXY - newZY * newXX;
    this.xX = newXX;
    this.xY = newXY;
    this.xZ = newXZ;
    this.yX = newYX;
    this.yY = newYY;
    this.yZ = newYZ;
    this.zX = newZX;
    this.zY = newZY;
    this.zZ = newZZ;
  }
};

// src/core/dynamics/constraint/constraint-solver.ts
var ConstraintSolver = class {
  constructor() {
    this.b1 = null;
    this.b2 = null;
    this.addedToIsland = false;
    /** 最近一次软接触子步的步长，用于关节热启动。 */
    this.softSubstepDt = 0;
  }
};

// src/core/dynamics/constraint/contact/contact-solver-info.ts
var BIT_LINEAR_SET = 1;
var BIT_ANGULAR_SET = 2;
var JacobianRow = class {
  constructor() {
    this.lin1X = 0;
    this.lin1Y = 0;
    this.lin1Z = 0;
    this.lin2X = 0;
    this.lin2Y = 0;
    this.lin2Z = 0;
    this.ang1X = 0;
    this.ang1Y = 0;
    this.ang1Z = 0;
    this.ang2X = 0;
    this.ang2Y = 0;
    this.ang2Z = 0;
    /** 稀疏标记：线性部分、角度部分是否非零（关节求解器用来跳过全零的部分） */
    this.flag = 0;
  }
  clear() {
    this.lin1X = 0;
    this.lin1Y = 0;
    this.lin1Z = 0;
    this.lin2X = 0;
    this.lin2Y = 0;
    this.lin2Z = 0;
    this.ang1X = 0;
    this.ang1Y = 0;
    this.ang1Z = 0;
    this.ang2X = 0;
    this.ang2Y = 0;
    this.ang2Z = 0;
  }
  updateSparsity() {
    this.flag = 0;
    if (!(this.lin1X === 0 && this.lin1Y === 0 && this.lin1Z === 0) || !(this.lin2X === 0 && this.lin2Y === 0 && this.lin2Z === 0)) {
      this.flag |= BIT_LINEAR_SET;
    }
    if (!(this.ang1X === 0 && this.ang1Y === 0 && this.ang1Z === 0) || !(this.ang2X === 0 && this.ang2Y === 0 && this.ang2Z === 0)) {
      this.flag |= BIT_ANGULAR_SET;
    }
  }
  isLinearSet() {
    return (this.flag & BIT_LINEAR_SET) !== 0;
  }
  isAngularSet() {
    return (this.flag & BIT_ANGULAR_SET) !== 0;
  }
};
var ContactSolverInfoRow = class {
  constructor() {
    this.jacobianN = new JacobianRow();
    this.jacobianT = new JacobianRow();
    this.jacobianB = new JacobianRow();
    /** 法向的目标相对速度（反弹、Baumgarte 位置修正） */
    this.rhs = 0;
    this.friction = 0;
    this.impulse = null;
  }
};
var ContactSolverInfo = class {
  constructor() {
    this.b1 = null;
    this.b2 = null;
    this.numRows = 0;
    this.rows = [];
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.rows.push(new ContactSolverInfoRow());
  }
};

// src/core/dynamics/constraint/joint/joint-solver-info.ts
var JointSolverInfoRow = class {
  constructor() {
    this.jacobian = new JacobianRow();
    /** 目标相对速度（位置修正） */
    this.rhs = 0;
    /** 约束力混合（软约束） */
    this.cfm = 0;
    /** 冲量的范围：等式约束是 (-∞, ∞)，限制是单边 */
    this.minImpulse = 0;
    this.maxImpulse = 0;
    /** 马达的目标速度和每步最大冲量 */
    this.motorSpeed = 0;
    this.motorMaxImpulse = 0;
    this.impulse = null;
  }
  clear() {
    this.jacobian.clear();
    this.rhs = 0;
    this.cfm = 0;
    this.minImpulse = 0;
    this.maxImpulse = 0;
    this.motorSpeed = 0;
    this.motorMaxImpulse = 0;
    this.impulse = null;
  }
  /** 等式约束（两边都不限制冲量） */
  equalLimit(rhs, cfm) {
    this.rhs = rhs;
    this.cfm = cfm;
    this.minImpulse = -Infinity;
    this.maxImpulse = Infinity;
  }
  motor(speed, maxImpulse) {
    this.motorSpeed = speed;
    this.motorMaxImpulse = maxImpulse;
  }
};
var JointSolverInfo = class {
  constructor() {
    this.b1 = null;
    this.b2 = null;
    this.numRows = 0;
    this.rows = [];
    for (let i = 0; i < MAX_JACOBIAN_ROWS; i++) this.rows.push(new JointSolverInfoRow());
  }
  addRow(impulse) {
    const row = this.rows[this.numRows++];
    row.clear();
    row.impulse = impulse;
    return row;
  }
};
var JointSolverMassDataRow = class {
  constructor() {
    this.invMLin1X = 0;
    this.invMLin1Y = 0;
    this.invMLin1Z = 0;
    this.invMLin2X = 0;
    this.invMLin2Y = 0;
    this.invMLin2Z = 0;
    this.invMAng1X = 0;
    this.invMAng1Y = 0;
    this.invMAng1Z = 0;
    this.invMAng2X = 0;
    this.invMAng2Y = 0;
    this.invMAng2Z = 0;
    this.mass = 0;
    this.massWithoutCfm = 0;
  }
};

// src/core/dynamics/constraint/solver/boundary.ts
var Boundary = class {
  constructor(maxRows) {
    /** 顶在上下限的行：下标、方向（-1 下限，1 上限，0 这一行不起作用、冲量是 0） */
    this.numBounded = 0;
    /** 在上下限之间的行的下标 */
    this.numUnbounded = 0;
    this.iBounded = new Array(maxRows).fill(0);
    this.signs = new Array(maxRows).fill(0);
    this.iUnbounded = new Array(maxRows).fill(0);
    this.b = new Array(maxRows).fill(0);
  }
  init(buildInfo) {
    this.numBounded = buildInfo.numBounded;
    for (let i = 0; i < this.numBounded; i++) {
      this.iBounded[i] = buildInfo.iBounded[i];
      this.signs[i] = buildInfo.signs[i];
    }
    this.numUnbounded = buildInfo.numUnbounded;
    for (let i = 0; i < this.numUnbounded; i++) this.iUnbounded[i] = buildInfo.iUnbounded[i];
  }
  /**
   * 按这种情况算出每一行的冲量增量，写进 dImpulses。返回这种情况是不是答案：
   * unbounded 的冲量没有超出上下限，bounded 的行满足互补条件（容差 DIRECT_MLCP_SOLVER_EPS）
   */
  computeImpulses(info, mass, relVels, impulses, dImpulses, impulseFactor) {
    const b = this.b;
    for (let i = 0; i < this.numUnbounded; i++) {
      const idx = this.iUnbounded[i], row = info.rows[idx];
      b[idx] = row.rhs * impulseFactor - relVels[idx] - row.cfm * impulses[idx];
    }
    const invMassWithoutCfm = mass.invMassWithoutCfm;
    for (let i = 0; i < this.numBounded; i++) {
      const idx = this.iBounded[i], sign = this.signs[i], row = info.rows[idx];
      const dImpulse = (sign < 0 ? row.minImpulse : sign > 0 ? row.maxImpulse : 0) - impulses[idx];
      dImpulses[idx] = dImpulse;
      if (dImpulse !== 0) {
        for (let j = 0; j < this.numUnbounded; j++) {
          const idx2 = this.iUnbounded[j];
          b[idx2] -= invMassWithoutCfm[idx][idx2] * dImpulse;
        }
      }
    }
    const massMatrix = mass.getSubmatrix(this.iUnbounded, this.numUnbounded);
    let ok = true;
    for (let i = 0; i < this.numUnbounded; i++) {
      const idx = this.iUnbounded[i], row = info.rows[idx];
      const oldImpulse = impulses[idx];
      let impulse = oldImpulse;
      for (let j = 0; j < this.numUnbounded; j++) impulse += b[this.iUnbounded[j]] * massMatrix[i][j];
      if (impulse < row.minImpulse - DIRECT_MLCP_SOLVER_EPS || impulse > row.maxImpulse + DIRECT_MLCP_SOLVER_EPS) {
        ok = false;
        break;
      }
      dImpulses[idx] = impulse - oldImpulse;
    }
    if (!ok) return false;
    for (let i = 0; i < this.numBounded; i++) {
      const idx = this.iBounded[i], row = info.rows[idx], sign = this.signs[i];
      const newImpulse = impulses[idx] + dImpulses[idx];
      let relVel = relVels[idx];
      for (let j = 0; j < info.numRows; j++) relVel += invMassWithoutCfm[idx][j] * dImpulses[j];
      const error = row.rhs * impulseFactor - relVel - row.cfm * newImpulse;
      if (sign < 0 && error > DIRECT_MLCP_SOLVER_EPS || sign > 0 && error < -DIRECT_MLCP_SOLVER_EPS) {
        ok = false;
        break;
      }
    }
    return ok;
  }
};
var BoundaryBuildInfo = class {
  constructor(size) {
    this.numBounded = 0;
    this.numUnbounded = 0;
    this.iBounded = new Array(size).fill(0);
    this.signs = new Array(size).fill(0);
    this.iUnbounded = new Array(size).fill(0);
  }
};
var BoundaryBuilder = class {
  constructor(maxRows) {
    this.numBoundaries = 0;
    /** 用到时才创建，创建过的留着下一步复用 */
    this.boundaries = [];
    this.maxRows = maxRows;
    this.maxBoundaries = 1 << maxRows;
    this.bbInfo = new BoundaryBuildInfo(maxRows);
  }
  buildBoundaries(info) {
    this.numBoundaries = 0;
    this.bbInfo.numBounded = 0;
    this.bbInfo.numUnbounded = 0;
    this.buildBoundariesRecursive(info, 0);
  }
  buildBoundariesRecursive(info, i) {
    const bb = this.bbInfo;
    if (i === info.numRows) {
      if (this.boundaries.length === this.numBoundaries) this.boundaries.push(new Boundary(this.maxRows));
      this.boundaries[this.numBoundaries++].init(bb);
      return;
    }
    const row = info.rows[i];
    const lowerLimitEnabled = row.minImpulse > -Infinity;
    const upperLimitEnabled = row.maxImpulse < Infinity;
    if (row.minImpulse === 0 && row.maxImpulse === 0) {
      bb.iBounded[bb.numBounded] = i;
      bb.signs[bb.numBounded] = 0;
      bb.numBounded++;
      this.buildBoundariesRecursive(info, i + 1);
      bb.numBounded--;
      return;
    }
    bb.iUnbounded[bb.numUnbounded] = i;
    bb.numUnbounded++;
    this.buildBoundariesRecursive(info, i + 1);
    bb.numUnbounded--;
    if (lowerLimitEnabled) {
      bb.iBounded[bb.numBounded] = i;
      bb.signs[bb.numBounded] = -1;
      bb.numBounded++;
      this.buildBoundariesRecursive(info, i + 1);
      bb.numBounded--;
    }
    if (upperLimitEnabled) {
      bb.iBounded[bb.numBounded] = i;
      bb.signs[bb.numBounded] = 1;
      bb.numBounded++;
      this.buildBoundariesRecursive(info, i + 1);
      bb.numBounded--;
    }
  }
};
var BoundarySelector = class {
  constructor(n) {
    this.n = n;
    this.indices = [];
    for (let i = 0; i < n; i++) this.indices.push(i);
    this.tmpIndices = new Array(n).fill(0);
  }
  getIndex(i) {
    return this.indices[i];
  }
  /** 把 index 移到最前面，其余的顺序不变 */
  select(index) {
    let i = 0;
    while (this.indices[i] !== index) i++;
    while (i > 0) {
      const tmp = this.indices[i];
      this.indices[i] = this.indices[i - 1];
      this.indices[i - 1] = tmp;
      i--;
    }
  }
  /** 让前 size 个正好是 0 ~ size-1 的一个排列（保持原来的相对顺序） */
  setSize(size) {
    let numSmaller = 0, numGreater = 0;
    for (let i = 0; i < this.n; i++) {
      const idx = this.indices[i];
      if (idx < size) {
        this.tmpIndices[numSmaller] = idx;
        numSmaller++;
      } else {
        this.tmpIndices[size + numGreater] = idx;
        numGreater++;
      }
    }
    const tmp = this.indices;
    this.indices = this.tmpIndices;
    this.tmpIndices = tmp;
  }
};

// src/core/dynamics/constraint/solver/mass-matrix.ts
var MassMatrix = class {
  constructor(size) {
    /** 逆质量矩阵：含 CFM、不含 CFM */
    this.invMass = [];
    this.invMassWithoutCfm = [];
    /** 按行子集（二进制位表示选了哪些行）缓存的子矩阵的逆，以及缓存是否有效 */
    this.cachedSubmatrices = [];
    this.cacheComputed = [];
    /** 求逆时用的临时矩阵 */
    this.tmpMatrix = [];
    this.size = size;
    for (let i = 0; i < size; i++) {
      this.tmpMatrix.push(new Array(size).fill(0));
      this.invMass.push(new Array(size).fill(0));
      this.invMassWithoutCfm.push(new Array(size).fill(0));
    }
    this.maxSubmatrixId = 1 << size;
    for (let id = 0; id < this.maxSubmatrixId; id++) {
      let n = 0;
      for (let t = id; t !== 0; t >>= 1) n += t & 1;
      const sub = [];
      for (let j = 0; j < n; j++) sub.push(new Array(n).fill(0));
      this.cacheComputed.push(false);
      this.cachedSubmatrices.push(sub);
    }
  }
  /** 取出 indices 前 size 个行列组成的子矩阵，求逆后放进第 id 个缓存（高斯-约当消元，不选主元） */
  computeSubmatrix(id, indices, size) {
    const src = this.tmpMatrix, dst = this.cachedSubmatrices[id];
    for (let i = 0; i < size; i++) {
      const ii = indices[i];
      for (let j = 0; j < size; j++) src[i][j] = this.invMass[ii][indices[j]];
    }
    for (let i = 0; i < size; i++) {
      const srci = src[i], dsti = dst[i];
      const diag = 1 / srci[i];
      dsti[i] = diag;
      for (let j = 0; j < i; j++) dsti[j] = dsti[j] * diag;
      for (let j = i + 1; j < size; j++) srci[j] = srci[j] * diag;
      for (let j = 0; j < i; j++) {
        const srcj = src[j], dstj = dst[j];
        for (let k = 0; k < j + 1; k++) dstj[k] = dstj[k] - dsti[k] * srcj[i];
        for (let k = i + 1; k < size; k++) srcj[k] = srcj[k] - srci[k] * srcj[i];
      }
      for (let j = i + 1; j < size; j++) {
        const srcj = src[j], dstj = dst[j];
        for (let k = 0; k < i; k++) dstj[k] = dstj[k] - dsti[k] * srcj[i];
        dstj[i] = -diag * srcj[i];
        for (let k = i + 1; k < size; k++) srcj[k] = srcj[k] - srci[k] * srcj[i];
      }
    }
    for (let i = 1; i < size; i++) {
      const dsti = dst[i];
      for (let j = 0; j < i; j++) dst[j][i] = dsti[j];
    }
  }
  /** indices 前 n 个行列组成的子矩阵的逆（缓存里没有时现算） */
  getSubmatrix(indices, n) {
    let id = 0;
    for (let i = 0; i < n; i++) id |= 1 << indices[i];
    if (!this.cacheComputed[id]) {
      this.computeSubmatrix(id, indices, n);
      this.cacheComputed[id] = true;
    }
    return this.cachedSubmatrices[id];
  }
  /** 由约束行算出逆质量矩阵，同时把每一行的 M⁻¹·Jᵀ 和有效质量写进 massData；清空子矩阵缓存 */
  computeInvMass(info, massData) {
    const invMass = this.invMass, invMassWithoutCfm = this.invMassWithoutCfm;
    const numRows = info.numRows, b1 = info.b1, b2 = info.b2;
    const invM1 = b1.invMass, invM2 = b2.invMass;
    for (let i = 0; i < numRows; i++) {
      const j = info.rows[i].jacobian, md = massData[i];
      j.updateSparsity();
      if (j.isLinearSet()) {
        md.invMLin1X = j.lin1X * invM1;
        md.invMLin1Y = j.lin1Y * invM1;
        md.invMLin1Z = j.lin1Z * invM1;
        md.invMLin2X = j.lin2X * invM2;
        md.invMLin2Y = j.lin2Y * invM2;
        md.invMLin2Z = j.lin2Z * invM2;
      } else {
        md.invMLin1X = 0;
        md.invMLin1Y = 0;
        md.invMLin1Z = 0;
        md.invMLin2X = 0;
        md.invMLin2Y = 0;
        md.invMLin2Z = 0;
      }
      if (j.isAngularSet()) {
        md.invMAng1X = b1.ii00 * j.ang1X + b1.ii01 * j.ang1Y + b1.ii02 * j.ang1Z;
        md.invMAng1Y = b1.ii10 * j.ang1X + b1.ii11 * j.ang1Y + b1.ii12 * j.ang1Z;
        md.invMAng1Z = b1.ii20 * j.ang1X + b1.ii21 * j.ang1Y + b1.ii22 * j.ang1Z;
        md.invMAng2X = b2.ii00 * j.ang2X + b2.ii01 * j.ang2Y + b2.ii02 * j.ang2Z;
        md.invMAng2Y = b2.ii10 * j.ang2X + b2.ii11 * j.ang2Y + b2.ii12 * j.ang2Z;
        md.invMAng2Z = b2.ii20 * j.ang2X + b2.ii21 * j.ang2Y + b2.ii22 * j.ang2Z;
      } else {
        md.invMAng1X = 0;
        md.invMAng1Y = 0;
        md.invMAng1Z = 0;
        md.invMAng2X = 0;
        md.invMAng2Y = 0;
        md.invMAng2Z = 0;
      }
    }
    for (let i = 0; i < numRows; i++) {
      const j1 = info.rows[i].jacobian;
      for (let j = i; j < numRows; j++) {
        const md2 = massData[j];
        const val = j1.lin1X * md2.invMLin1X + j1.lin1Y * md2.invMLin1Y + j1.lin1Z * md2.invMLin1Z + (j1.ang1X * md2.invMAng1X + j1.ang1Y * md2.invMAng1Y + j1.ang1Z * md2.invMAng1Z) + (j1.lin2X * md2.invMLin2X + j1.lin2Y * md2.invMLin2Y + j1.lin2Z * md2.invMLin2Z) + (j1.ang2X * md2.invMAng2X + j1.ang2Y * md2.invMAng2Y + j1.ang2Z * md2.invMAng2Z);
        if (i === j) {
          invMass[i][j] = val + info.rows[i].cfm;
          invMassWithoutCfm[i][j] = val;
          md2.mass = val + info.rows[i].cfm;
          md2.massWithoutCfm = val;
          if (md2.mass !== 0) md2.mass = 1 / md2.mass;
          if (md2.massWithoutCfm !== 0) md2.massWithoutCfm = 1 / md2.massWithoutCfm;
        } else {
          invMass[i][j] = val;
          invMass[j][i] = val;
          invMassWithoutCfm[i][j] = val;
          invMassWithoutCfm[j][i] = val;
        }
      }
    }
    for (let id = 0; id < this.maxSubmatrixId; id++) this.cacheComputed[id] = false;
  }
};

// src/core/dynamics/constraint/solver/direct-joint-constraint-solver.ts
var DirectJointConstraintSolver = class extends ConstraintSolver {
  constructor(joint) {
    super();
    this.info = new JointSolverInfo();
    this.massData = [];
    this.relVels = [];
    this.impulses = [];
    this.dImpulses = [];
    this.dTotalImpulses = [];
    this.massMatrix = new MassMatrix(MAX_JACOBIAN_ROWS);
    this.boundaryBuilder = new BoundaryBuilder(MAX_JACOBIAN_ROWS);
    this.velBoundarySelector = new BoundarySelector(this.boundaryBuilder.maxBoundaries);
    this.posBoundarySelector = new BoundarySelector(this.boundaryBuilder.maxBoundaries);
    /** 冲量换算成两个刚体的速度（或位移、转动）变化时用的临时变量 */
    this.l1X = 0;
    this.l1Y = 0;
    this.l1Z = 0;
    this.l2X = 0;
    this.l2Y = 0;
    this.l2Z = 0;
    this.a1X = 0;
    this.a1Y = 0;
    this.a1Z = 0;
    this.a2X = 0;
    this.a2Y = 0;
    this.a2Z = 0;
    this.linearSet = false;
    this.angularSet = false;
    this.joint = joint;
    for (let i = 0; i < MAX_JACOBIAN_ROWS; i++) {
      this.massData.push(new JointSolverMassDataRow());
      this.relVels.push(0);
      this.impulses.push(0);
      this.dImpulses.push(0);
      this.dTotalImpulses.push(0);
    }
  }
  /** 每一行的冲量换算成速度变化，加到 l1..a2 上 */
  accumulate(impulses) {
    const info = this.info;
    this.linearSet = false;
    this.angularSet = false;
    for (let i = 0; i < info.numRows; i++) {
      const j = info.rows[i].jacobian, md = this.massData[i], imp = impulses[i];
      if (j.isLinearSet()) {
        this.l1X += md.invMLin1X * imp;
        this.l1Y += md.invMLin1Y * imp;
        this.l1Z += md.invMLin1Z * imp;
        this.l2X += md.invMLin2X * -imp;
        this.l2Y += md.invMLin2Y * -imp;
        this.l2Z += md.invMLin2Z * -imp;
        this.linearSet = true;
      }
      if (j.isAngularSet()) {
        this.a1X += md.invMAng1X * imp;
        this.a1Y += md.invMAng1Y * imp;
        this.a1Z += md.invMAng1Z * imp;
        this.a2X += md.invMAng2X * -imp;
        this.a2Y += md.invMAng2Y * -imp;
        this.a2Z += md.invMAng2Z * -imp;
        this.angularSet = true;
      }
    }
  }
  /** 冲量加到两个刚体的速度上 */
  applyImpulses(impulses) {
    const b1 = this.b1, b2 = this.b2;
    this.l1X = b1.vx;
    this.l1Y = b1.vy;
    this.l1Z = b1.vz;
    this.l2X = b2.vx;
    this.l2Y = b2.vy;
    this.l2Z = b2.vz;
    this.a1X = b1.wx;
    this.a1Y = b1.wy;
    this.a1Z = b1.wz;
    this.a2X = b2.wx;
    this.a2Y = b2.wy;
    this.a2Z = b2.wz;
    this.accumulate(impulses);
    if (this.linearSet) {
      b1.vx = this.l1X;
      b1.vy = this.l1Y;
      b1.vz = this.l1Z;
      b2.vx = this.l2X;
      b2.vy = this.l2Y;
      b2.vz = this.l2Z;
    }
    if (this.angularSet) {
      b1.wx = this.a1X;
      b1.wy = this.a1Y;
      b1.wz = this.a1Z;
      b2.wx = this.a2X;
      b2.wy = this.a2Y;
      b2.wz = this.a2Z;
    }
  }
  /** 位置修正（分离冲量）的冲量加到两个刚体的伪速度上 */
  applySplitImpulses(impulses) {
    const b1 = this.b1, b2 = this.b2;
    this.l1X = b1.pvx;
    this.l1Y = b1.pvy;
    this.l1Z = b1.pvz;
    this.l2X = b2.pvx;
    this.l2Y = b2.pvy;
    this.l2Z = b2.pvz;
    this.a1X = b1.pwx;
    this.a1Y = b1.pwy;
    this.a1Z = b1.pwz;
    this.a2X = b2.pwx;
    this.a2Y = b2.pwy;
    this.a2Z = b2.pwz;
    this.accumulate(impulses);
    if (this.linearSet) {
      b1.pvx = this.l1X;
      b1.pvy = this.l1Y;
      b1.pvz = this.l1Z;
      b2.pvx = this.l2X;
      b2.pvy = this.l2Y;
      b2.pvz = this.l2Z;
    }
    if (this.angularSet) {
      b1.pwx = this.a1X;
      b1.pwy = this.a1Y;
      b1.pwz = this.a1Z;
      b2.pwx = this.a2X;
      b2.pwy = this.a2Y;
      b2.pwz = this.a2Z;
    }
  }
  /** 位置修正（NGS）的冲量直接换算成两个刚体的位移和转动 */
  applyPositionImpulses(impulses) {
    const b1 = this.b1, b2 = this.b2;
    this.l1X = 0;
    this.l1Y = 0;
    this.l1Z = 0;
    this.l2X = 0;
    this.l2Y = 0;
    this.l2Z = 0;
    this.a1X = 0;
    this.a1Y = 0;
    this.a1Z = 0;
    this.a2X = 0;
    this.a2Y = 0;
    this.a2Z = 0;
    this.accumulate(impulses);
    if (this.linearSet) {
      b1.applyTranslation(this.l1X, this.l1Y, this.l1Z);
      b2.applyTranslation(this.l2X, this.l2Y, this.l2Z);
    }
    if (this.angularSet) {
      b1.applyRotation(this.a1X, this.a1Y, this.a1Z);
      b2.applyRotation(this.a2X, this.a2Y, this.a2Z);
    }
  }
  preSolveVelocity(timeStep) {
    this.joint.syncAnchors();
    this.joint.getVelocitySolverInfo(timeStep, this.info);
    this.b1 = this.info.b1;
    this.b2 = this.info.b2;
    this.massMatrix.computeInvMass(this.info, this.massData);
    this.boundaryBuilder.buildBoundaries(this.info);
    this.velBoundarySelector.setSize(this.boundaryBuilder.numBoundaries);
  }
  warmStart(timeStep) {
    const info = this.info;
    let factor = this.joint.getWarmStartingFactor();
    factor *= timeStep.dtRatio;
    if (factor <= 0) {
      for (let i = 0; i < info.numRows; i++) info.rows[i].impulse.clear();
      return;
    }
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse;
      let impulse = imp.impulse * factor;
      if (impulse < row.minImpulse) impulse = row.minImpulse;
      else if (impulse > row.maxImpulse) impulse = row.maxImpulse;
      imp.impulse = impulse;
      if (row.motorMaxImpulse > 0) {
        let impulseM = imp.impulseM * factor;
        const max = row.motorMaxImpulse;
        if (impulseM < -max) impulseM = -max;
        else if (impulseM > max) impulseM = max;
        imp.impulseM = impulseM;
      } else {
        imp.impulseM = 0;
      }
      this.dImpulses[i] = imp.impulse + imp.impulseM;
    }
    this.applyImpulses(this.dImpulses);
  }
  solveVelocity() {
    const info = this.info, numRows = info.numRows, b1 = this.b1, b2 = this.b2;
    const relVels = this.relVels, impulses = this.impulses, dImpulses = this.dImpulses, dTotalImpulses = this.dTotalImpulses;
    const lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz;
    const lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    const av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz;
    const av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    for (let i = 0; i < numRows; i++) {
      const row = info.rows[i], j = row.jacobian;
      let relVel = 0;
      relVel += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      relVel -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      relVel += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      relVel -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      relVels[i] = relVel;
      impulses[i] = row.impulse.impulse;
      dTotalImpulses[i] = 0;
    }
    const invMass = this.massMatrix.invMassWithoutCfm;
    for (let i = 0; i < numRows; i++) {
      const row = info.rows[i], imp = row.impulse;
      if (row.motorMaxImpulse > 0) {
        const oldImpulseM = imp.impulseM;
        let impulseM = oldImpulseM + this.massData[i].massWithoutCfm * (-row.motorSpeed - relVels[i]);
        const maxImpulseM = row.motorMaxImpulse;
        if (impulseM < -maxImpulseM) impulseM = -maxImpulseM;
        else if (impulseM > maxImpulseM) impulseM = maxImpulseM;
        imp.impulseM = impulseM;
        const dImpulseM = impulseM - oldImpulseM;
        dTotalImpulses[i] = dImpulseM;
        for (let k = 0; k < numRows; k++) relVels[k] += dImpulseM * invMass[i][k];
      }
    }
    const builder = this.boundaryBuilder;
    for (let i = 0; i < builder.numBoundaries; i++) {
      const idx = this.velBoundarySelector.getIndex(i);
      if (builder.boundaries[idx].computeImpulses(info, this.massMatrix, relVels, impulses, dImpulses, 1)) {
        for (let k = 0; k < numRows; k++) {
          const dimp = dImpulses[k];
          info.rows[k].impulse.impulse += dimp;
          dTotalImpulses[k] += dimp;
        }
        this.applyImpulses(dTotalImpulses);
        this.velBoundarySelector.select(idx);
        return;
      }
    }
  }
  /** 由累积冲量算出这一步的约束力、力矩（断开判断用） */
  postSolveVelocity(timeStep) {
    const info = this.info;
    let linX = 0, linY = 0, linZ = 0, angX = 0, angY = 0, angZ = 0;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse, j = row.jacobian;
      if (j.isLinearSet()) {
        linX += j.lin1X * imp.impulse;
        linY += j.lin1Y * imp.impulse;
        linZ += j.lin1Z * imp.impulse;
      } else if (j.isAngularSet()) {
        angX += j.ang1X * imp.impulse;
        angY += j.ang1Y * imp.impulse;
        angZ += j.ang1Z * imp.impulse;
      }
    }
    const joint = this.joint, invDt = timeStep.invDt;
    joint.appliedForceX = linX * invDt;
    joint.appliedForceY = linY * invDt;
    joint.appliedForceZ = linZ * invDt;
    joint.appliedTorqueX = angX * invDt;
    joint.appliedTorqueY = angY * invDt;
    joint.appliedTorqueZ = angZ * invDt;
  }
  /** 位置修正阶段的约束行、逆质量矩阵和情况列表 */
  updatePositionData() {
    this.joint.syncAnchors();
    this.joint.getPositionSolverInfo(this.info);
    this.b1 = this.info.b1;
    this.b2 = this.info.b2;
    this.massMatrix.computeInvMass(this.info, this.massData);
    this.boundaryBuilder.buildBoundaries(this.info);
    this.posBoundarySelector.setSize(this.boundaryBuilder.numBoundaries);
  }
  preSolvePosition() {
    this.updatePositionData();
    for (let i = 0; i < this.info.numRows; i++) this.info.rows[i].impulse.impulseP = 0;
  }
  solvePositionSplitImpulse() {
    const info = this.info, numRows = info.numRows, b1 = this.b1, b2 = this.b2;
    const relVels = this.relVels, impulses = this.impulses, dImpulses = this.dImpulses;
    const lv1X = b1.pvx, lv1Y = b1.pvy, lv1Z = b1.pvz;
    const lv2X = b2.pvx, lv2Y = b2.pvy, lv2Z = b2.pvz;
    const av1X = b1.pwx, av1Y = b1.pwy, av1Z = b1.pwz;
    const av2X = b2.pwx, av2Y = b2.pwy, av2Z = b2.pwz;
    for (let i = 0; i < numRows; i++) {
      const row = info.rows[i], j = row.jacobian;
      let relVel = 0;
      relVel += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      relVel -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      relVel += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      relVel -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      relVels[i] = relVel;
      impulses[i] = row.impulse.impulseP;
    }
    const builder = this.boundaryBuilder;
    for (let i = 0; i < builder.numBoundaries; i++) {
      const idx = this.posBoundarySelector.getIndex(i);
      if (builder.boundaries[idx].computeImpulses(info, this.massMatrix, relVels, impulses, dImpulses, POSITION_SPLIT_IMPULSE_BAUMGARTE)) {
        for (let k = 0; k < numRows; k++) info.rows[k].impulse.impulseP += dImpulses[k];
        this.applySplitImpulses(dImpulses);
        this.posBoundarySelector.select(idx);
        return;
      }
    }
  }
  /** 非线性高斯-赛德尔：每次迭代重新算约束行，直接修改刚体的位置和朝向 */
  solvePositionNgs() {
    this.updatePositionData();
    const info = this.info, numRows = info.numRows;
    const relVels = this.relVels, impulses = this.impulses, dImpulses = this.dImpulses;
    for (let i = 0; i < numRows; i++) {
      relVels[i] = 0;
      impulses[i] = info.rows[i].impulse.impulseP;
    }
    const builder = this.boundaryBuilder;
    for (let i = 0; i < builder.numBoundaries; i++) {
      const idx = this.posBoundarySelector.getIndex(i);
      if (builder.boundaries[idx].computeImpulses(info, this.massMatrix, relVels, impulses, dImpulses, POSITION_NGS_BAUMGARTE)) {
        for (let k = 0; k < numRows; k++) info.rows[k].impulse.impulseP += dImpulses[k];
        this.applyPositionImpulses(dImpulses);
        this.posBoundarySelector.select(idx);
        return;
      }
    }
  }
  postSolve() {
    this.joint.syncAnchors();
    this.joint.checkDestruction();
  }
};

// src/core/dynamics/constraint/solver/pgs-joint-constraint-solver.ts
var PgsJointConstraintSolver = class extends ConstraintSolver {
  constructor(joint) {
    super();
    this.info = new JointSolverInfo();
    this.massData = [];
    this.joint = joint;
    for (let i = 0; i < MAX_JACOBIAN_ROWS; i++) this.massData.push(new JointSolverMassDataRow());
  }
  preSolveVelocity(timeStep) {
    this.joint.syncAnchors();
    this.joint.getVelocitySolverInfo(timeStep, this.info);
    const info = this.info;
    const b1 = this.b1 = info.b1, b2 = this.b2 = info.b2;
    const invM1 = b1.invMass, invM2 = b2.invMass;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], j = row.jacobian;
      j.updateSparsity();
      if (j.isLinearSet()) {
        md.invMLin1X = j.lin1X * invM1;
        md.invMLin1Y = j.lin1Y * invM1;
        md.invMLin1Z = j.lin1Z * invM1;
        md.invMLin2X = j.lin2X * invM2;
        md.invMLin2Y = j.lin2Y * invM2;
        md.invMLin2Z = j.lin2Z * invM2;
      } else {
        md.invMLin1X = 0;
        md.invMLin1Y = 0;
        md.invMLin1Z = 0;
        md.invMLin2X = 0;
        md.invMLin2Y = 0;
        md.invMLin2Z = 0;
      }
      if (j.isAngularSet()) {
        md.invMAng1X = b1.ii00 * j.ang1X + b1.ii01 * j.ang1Y + b1.ii02 * j.ang1Z;
        md.invMAng1Y = b1.ii10 * j.ang1X + b1.ii11 * j.ang1Y + b1.ii12 * j.ang1Z;
        md.invMAng1Z = b1.ii20 * j.ang1X + b1.ii21 * j.ang1Y + b1.ii22 * j.ang1Z;
        md.invMAng2X = b2.ii00 * j.ang2X + b2.ii01 * j.ang2Y + b2.ii02 * j.ang2Z;
        md.invMAng2Y = b2.ii10 * j.ang2X + b2.ii11 * j.ang2Y + b2.ii12 * j.ang2Z;
        md.invMAng2Z = b2.ii20 * j.ang2X + b2.ii21 * j.ang2Y + b2.ii22 * j.ang2Z;
      } else {
        md.invMAng1X = 0;
        md.invMAng1Y = 0;
        md.invMAng1Z = 0;
        md.invMAng2X = 0;
        md.invMAng2Y = 0;
        md.invMAng2Z = 0;
      }
      md.massWithoutCfm = md.invMLin1X * j.lin1X + md.invMLin1Y * j.lin1Y + md.invMLin1Z * j.lin1Z + (md.invMLin2X * j.lin2X + md.invMLin2Y * j.lin2Y + md.invMLin2Z * j.lin2Z) + (md.invMAng1X * j.ang1X + md.invMAng1Y * j.ang1Y + md.invMAng1Z * j.ang1Z) + (md.invMAng2X * j.ang2X + md.invMAng2Y * j.ang2Y + md.invMAng2Z * j.ang2Z);
      md.mass = md.massWithoutCfm + row.cfm;
      if (md.massWithoutCfm !== 0) md.massWithoutCfm = 1 / md.massWithoutCfm;
      if (md.mass !== 0) md.mass = 1 / md.mass;
    }
  }
  warmStart(timeStep) {
    const info = this.info;
    let factor = this.joint.getWarmStartingFactor();
    factor *= timeStep.dtRatio;
    if (factor <= 0) {
      for (let i = 0; i < info.numRows; i++) info.rows[i].impulse.clear();
      return;
    }
    const b1 = this.b1, b2 = this.b2;
    let lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz;
    let lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    let av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz;
    let av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    for (let i = 0; i < info.numRows; i++) {
      const md = this.massData[i], imp = info.rows[i].impulse;
      imp.impulse *= factor;
      imp.impulseM *= factor;
      const impulse = imp.impulse + imp.impulseM;
      lv1X += md.invMLin1X * impulse;
      lv1Y += md.invMLin1Y * impulse;
      lv1Z += md.invMLin1Z * impulse;
      lv2X += md.invMLin2X * -impulse;
      lv2Y += md.invMLin2Y * -impulse;
      lv2Z += md.invMLin2Z * -impulse;
      av1X += md.invMAng1X * impulse;
      av1Y += md.invMAng1Y * impulse;
      av1Z += md.invMAng1Z * impulse;
      av2X += md.invMAng2X * -impulse;
      av2Y += md.invMAng2Y * -impulse;
      av2Z += md.invMAng2Z * -impulse;
    }
    b1.vx = lv1X;
    b1.vy = lv1Y;
    b1.vz = lv1Z;
    b2.vx = lv2X;
    b2.vy = lv2Y;
    b2.vz = lv2Z;
    b1.wx = av1X;
    b1.wy = av1Y;
    b1.wz = av1Z;
    b2.wx = av2X;
    b2.wy = av2Y;
    b2.wz = av2Z;
  }
  solveVelocity() {
    const info = this.info, b1 = this.b1, b2 = this.b2;
    let lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz;
    let lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    let av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz;
    let av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse, j = row.jacobian;
      if (row.motorMaxImpulse === 0) continue;
      let rv = 0;
      rv += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rv -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rv += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rv -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseM = (-row.motorSpeed - rv) * md.massWithoutCfm;
      const oldImpulseM = imp.impulseM;
      imp.impulseM += impulseM;
      if (imp.impulseM < -row.motorMaxImpulse) imp.impulseM = -row.motorMaxImpulse;
      else if (imp.impulseM > row.motorMaxImpulse) imp.impulseM = row.motorMaxImpulse;
      impulseM = imp.impulseM - oldImpulseM;
      if (j.isLinearSet()) {
        lv1X += md.invMLin1X * impulseM;
        lv1Y += md.invMLin1Y * impulseM;
        lv1Z += md.invMLin1Z * impulseM;
        lv2X += md.invMLin2X * -impulseM;
        lv2Y += md.invMLin2Y * -impulseM;
        lv2Z += md.invMLin2Z * -impulseM;
      }
      if (j.isAngularSet()) {
        av1X += md.invMAng1X * impulseM;
        av1Y += md.invMAng1Y * impulseM;
        av1Z += md.invMAng1Z * impulseM;
        av2X += md.invMAng2X * -impulseM;
        av2Y += md.invMAng2Y * -impulseM;
        av2Z += md.invMAng2Z * -impulseM;
      }
    }
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse, j = row.jacobian;
      let rv = 0;
      rv += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rv -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rv += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rv -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulse = (row.rhs - rv - imp.impulse * row.cfm) * md.mass;
      const oldImpulse = imp.impulse;
      imp.impulse += impulse;
      if (imp.impulse < row.minImpulse) imp.impulse = row.minImpulse;
      else if (imp.impulse > row.maxImpulse) imp.impulse = row.maxImpulse;
      impulse = imp.impulse - oldImpulse;
      if (j.isLinearSet()) {
        lv1X += md.invMLin1X * impulse;
        lv1Y += md.invMLin1Y * impulse;
        lv1Z += md.invMLin1Z * impulse;
        lv2X += md.invMLin2X * -impulse;
        lv2Y += md.invMLin2Y * -impulse;
        lv2Z += md.invMLin2Z * -impulse;
      }
      if (j.isAngularSet()) {
        av1X += md.invMAng1X * impulse;
        av1Y += md.invMAng1Y * impulse;
        av1Z += md.invMAng1Z * impulse;
        av2X += md.invMAng2X * -impulse;
        av2Y += md.invMAng2Y * -impulse;
        av2Z += md.invMAng2Z * -impulse;
      }
    }
    b1.vx = lv1X;
    b1.vy = lv1Y;
    b1.vz = lv1Z;
    b2.vx = lv2X;
    b2.vy = lv2Y;
    b2.vz = lv2Z;
    b1.wx = av1X;
    b1.wy = av1Y;
    b1.wz = av1Z;
    b2.wx = av2X;
    b2.wy = av2Y;
    b2.wz = av2Z;
  }
  /** 由累积冲量算出这一步的约束力、力矩（断开判断用） */
  postSolveVelocity(timeStep) {
    const info = this.info;
    let linX = 0, linY = 0, linZ = 0, angX = 0, angY = 0, angZ = 0;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse, j = row.jacobian;
      if (j.isLinearSet()) {
        linX += j.lin1X * imp.impulse;
        linY += j.lin1Y * imp.impulse;
        linZ += j.lin1Z * imp.impulse;
      } else if (j.isAngularSet()) {
        angX += j.ang1X * imp.impulse;
        angY += j.ang1Y * imp.impulse;
        angZ += j.ang1Z * imp.impulse;
      }
    }
    const joint = this.joint, invDt = timeStep.invDt;
    joint.appliedForceX = linX * invDt;
    joint.appliedForceY = linY * invDt;
    joint.appliedForceZ = linZ * invDt;
    joint.appliedTorqueX = angX * invDt;
    joint.appliedTorqueY = angY * invDt;
    joint.appliedTorqueZ = angZ * invDt;
  }
  /** 位置修正阶段的约束行和有效质量（不含 CFM） */
  updatePositionData() {
    this.joint.syncAnchors();
    this.joint.getPositionSolverInfo(this.info);
    const info = this.info;
    const b1 = this.b1 = info.b1, b2 = this.b2 = info.b2;
    const invM1 = b1.invMass, invM2 = b2.invMass;
    for (let i = 0; i < info.numRows; i++) {
      const md = this.massData[i], j = info.rows[i].jacobian;
      md.invMLin1X = j.lin1X * invM1;
      md.invMLin1Y = j.lin1Y * invM1;
      md.invMLin1Z = j.lin1Z * invM1;
      md.invMLin2X = j.lin2X * invM2;
      md.invMLin2Y = j.lin2Y * invM2;
      md.invMLin2Z = j.lin2Z * invM2;
      md.invMAng1X = b1.ii00 * j.ang1X + b1.ii01 * j.ang1Y + b1.ii02 * j.ang1Z;
      md.invMAng1Y = b1.ii10 * j.ang1X + b1.ii11 * j.ang1Y + b1.ii12 * j.ang1Z;
      md.invMAng1Z = b1.ii20 * j.ang1X + b1.ii21 * j.ang1Y + b1.ii22 * j.ang1Z;
      md.invMAng2X = b2.ii00 * j.ang2X + b2.ii01 * j.ang2Y + b2.ii02 * j.ang2Z;
      md.invMAng2Y = b2.ii10 * j.ang2X + b2.ii11 * j.ang2Y + b2.ii12 * j.ang2Z;
      md.invMAng2Z = b2.ii20 * j.ang2X + b2.ii21 * j.ang2Y + b2.ii22 * j.ang2Z;
      md.mass = md.invMLin1X * j.lin1X + md.invMLin1Y * j.lin1Y + md.invMLin1Z * j.lin1Z + (md.invMLin2X * j.lin2X + md.invMLin2Y * j.lin2Y + md.invMLin2Z * j.lin2Z) + (md.invMAng1X * j.ang1X + md.invMAng1Y * j.ang1Y + md.invMAng1Z * j.ang1Z) + (md.invMAng2X * j.ang2X + md.invMAng2Y * j.ang2Y + md.invMAng2Z * j.ang2Z);
      if (md.mass !== 0) md.mass = 1 / md.mass;
    }
  }
  preSolvePosition() {
    this.updatePositionData();
    for (let i = 0; i < this.info.numRows; i++) this.info.rows[i].impulse.impulseP = 0;
  }
  solvePositionSplitImpulse() {
    const info = this.info, b1 = this.b1, b2 = this.b2;
    let lv1X = b1.pvx, lv1Y = b1.pvy, lv1Z = b1.pvz;
    let lv2X = b2.pvx, lv2Y = b2.pvy, lv2Z = b2.pvz;
    let av1X = b1.pwx, av1Y = b1.pwy, av1Z = b1.pwz;
    let av2X = b2.pwx, av2Y = b2.pwy, av2Z = b2.pwz;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse, j = row.jacobian;
      let rv = 0;
      rv += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rv -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rv += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rv -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseP = (row.rhs * POSITION_SPLIT_IMPULSE_BAUMGARTE - rv) * md.mass;
      const oldImpulseP = imp.impulseP;
      imp.impulseP += impulseP;
      if (imp.impulseP < row.minImpulse) imp.impulseP = row.minImpulse;
      else if (imp.impulseP > row.maxImpulse) imp.impulseP = row.maxImpulse;
      impulseP = imp.impulseP - oldImpulseP;
      lv1X += md.invMLin1X * impulseP;
      lv1Y += md.invMLin1Y * impulseP;
      lv1Z += md.invMLin1Z * impulseP;
      lv2X += md.invMLin2X * -impulseP;
      lv2Y += md.invMLin2Y * -impulseP;
      lv2Z += md.invMLin2Z * -impulseP;
      av1X += md.invMAng1X * impulseP;
      av1Y += md.invMAng1Y * impulseP;
      av1Z += md.invMAng1Z * impulseP;
      av2X += md.invMAng2X * -impulseP;
      av2Y += md.invMAng2Y * -impulseP;
      av2Z += md.invMAng2Z * -impulseP;
    }
    b1.pvx = lv1X;
    b1.pvy = lv1Y;
    b1.pvz = lv1Z;
    b2.pvx = lv2X;
    b2.pvy = lv2Y;
    b2.pvz = lv2Z;
    b1.pwx = av1X;
    b1.pwy = av1Y;
    b1.pwz = av1Z;
    b2.pwx = av2X;
    b2.pwy = av2Y;
    b2.pwz = av2Z;
  }
  /** 非线性高斯-赛德尔：每次迭代重新算约束行，直接修改刚体的位置和朝向 */
  solvePositionNgs() {
    this.updatePositionData();
    const info = this.info, b1 = this.b1, b2 = this.b2;
    let lv1X = 0, lv1Y = 0, lv1Z = 0;
    let lv2X = 0, lv2Y = 0, lv2Z = 0;
    let av1X = 0, av1Y = 0, av1Z = 0;
    let av2X = 0, av2Y = 0, av2Z = 0;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse, j = row.jacobian;
      let rv = 0;
      rv += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rv -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rv += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rv -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseP = (row.rhs * POSITION_NGS_BAUMGARTE - rv) * md.mass;
      const oldImpulseP = imp.impulseP;
      imp.impulseP += impulseP;
      if (imp.impulseP < row.minImpulse) imp.impulseP = row.minImpulse;
      else if (imp.impulseP > row.maxImpulse) imp.impulseP = row.maxImpulse;
      impulseP = imp.impulseP - oldImpulseP;
      lv1X += md.invMLin1X * impulseP;
      lv1Y += md.invMLin1Y * impulseP;
      lv1Z += md.invMLin1Z * impulseP;
      lv2X += md.invMLin2X * -impulseP;
      lv2Y += md.invMLin2Y * -impulseP;
      lv2Z += md.invMLin2Z * -impulseP;
      av1X += md.invMAng1X * impulseP;
      av1Y += md.invMAng1Y * impulseP;
      av1Z += md.invMAng1Z * impulseP;
      av2X += md.invMAng2X * -impulseP;
      av2Y += md.invMAng2Y * -impulseP;
      av2Z += md.invMAng2Z * -impulseP;
    }
    b1.applyTranslation(lv1X, lv1Y, lv1Z);
    b2.applyTranslation(lv2X, lv2Y, lv2Z);
    b1.applyRotation(av1X, av1Y, av1Z);
    b2.applyRotation(av2X, av2Y, av2Z);
  }
  postSolve() {
    this.joint.syncAnchors();
    this.joint.checkDestruction();
  }
};

// src/core/dynamics/constraint/joint/joint-impulse.ts
var JointImpulse = class {
  constructor() {
    /** 约束冲量 */
    this.impulse = 0;
    /** 马达冲量 */
    this.impulseM = 0;
    /** 位置修正冲量 */
    this.impulseP = 0;
  }
  clear() {
    this.impulse = 0;
    this.impulseM = 0;
    this.impulseP = 0;
  }
};

// src/core/dynamics/constraint/joint/joint-link.ts
var JointLink = class {
  constructor(joint) {
    this.prev = null;
    this.next = null;
    this.other = null;
    this.joint = joint;
  }
};

// src/core/dynamics/constraint/joint/joint-math.ts
var PERP = { x: 0, y: 0, z: 0 };
function perp(x1, y1, z1) {
  const x2 = x1 * x1, y2 = y1 * y1, z2 = z1 * z1;
  let d;
  if (x2 < y2) {
    if (x2 < z2) {
      d = 1 / Math.sqrt(y2 + z2);
      PERP.x = 0;
      PERP.y = z1 * d;
      PERP.z = -y1 * d;
    } else {
      d = 1 / Math.sqrt(x2 + y2);
      PERP.x = y1 * d;
      PERP.y = -x1 * d;
      PERP.z = 0;
    }
  } else if (y2 < z2) {
    d = 1 / Math.sqrt(z2 + x2);
    PERP.x = -z1 * d;
    PERP.y = 0;
    PERP.z = x1 * d;
  } else {
    d = 1 / Math.sqrt(x2 + y2);
    PERP.x = y1 * d;
    PERP.y = -x1 * d;
    PERP.z = 0;
  }
}
var ARC = { x: 0, y: 0, z: 0, w: 0 };
function arc(ux, uy, uz, vx, vy, vz) {
  let d = ux * vx + uy * vy + uz * vz;
  if (d < -0.999999999) {
    perp(ux, uy, uz);
    ARC.x = PERP.x;
    ARC.y = PERP.y;
    ARC.z = PERP.z;
    ARC.w = 0;
  } else {
    let cX = uy * vz - uz * vy, cY = uz * vx - ux * vz, cZ = ux * vy - uy * vx;
    const w = Math.sqrt((1 + d) * 0.5);
    d = 0.5 / w;
    cX *= d;
    cY *= d;
    cZ *= d;
    ARC.x = cX;
    ARC.y = cY;
    ARC.z = cZ;
    ARC.w = w;
  }
}
var ROT = { m00: 0, m01: 0, m02: 0, m10: 0, m11: 0, m12: 0, m20: 0, m21: 0, m22: 0 };
function mat3FromQuat(x, y, z, w) {
  const x2 = 2 * x, y2 = 2 * y, z2 = 2 * z;
  const xx = x * x2, yy = y * y2, zz = z * z2;
  const xy = x * y2, yz = y * z2, xz = x * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  ROT.m00 = 1 - yy - zz;
  ROT.m01 = xy - wz;
  ROT.m02 = xz + wy;
  ROT.m10 = xy + wz;
  ROT.m11 = 1 - xx - zz;
  ROT.m12 = yz - wx;
  ROT.m20 = xz - wy;
  ROT.m21 = yz + wx;
  ROT.m22 = 1 - xx - yy;
}

// src/core/dynamics/constraint/joint/joint.ts
var JointType = {
  SPHERICAL: 0,
  REVOLUTE: 1,
  CYLINDRICAL: 2,
  PRISMATIC: 3,
  UNIVERSAL: 4,
  RAGDOLL: 5,
  GENERIC: 6
};
var SOFT = { cfmFactor: 0, erp: 0 };
function computeSoftConstraintParameters(frequency, dampingRatio, dt, useSymplecticEuler) {
  const omega = 2 * 3.14159265358979 * frequency;
  let zeta = dampingRatio;
  if (zeta < MIN_SPRING_DAMPER_DAMPING_RATIO) zeta = MIN_SPRING_DAMPER_DAMPING_RATIO;
  const h = dt;
  const c = 2 * zeta * omega;
  const k = omega * omega;
  if (useSymplecticEuler) {
    SOFT.cfmFactor = 1 / (h * c);
    SOFT.erp = k / c;
  } else {
    SOFT.cfmFactor = 1 / (h * (h * k + c));
    SOFT.erp = k / (h * k + c);
  }
}
var Joint = class {
  constructor(config, type) {
    this.positionCorrectionAlgorithm = DEFAULT_JOINT_POSITION_CORRECTION_ALGORITHM;
    this.prev = null;
    this.next = null;
    this.world = null;
    this.ra1X = 0;
    this.ra1Y = 0;
    this.ra1Z = 0;
    this.ra2X = 0;
    this.ra2Y = 0;
    this.ra2Z = 0;
    this.a1X = 0;
    this.a1Y = 0;
    this.a1Z = 0;
    this.a2X = 0;
    this.a2Y = 0;
    this.a2Z = 0;
    /** 关节坐标系：两个刚体上的局部坐标、世界坐标 */
    this.lbX1X = 0;
    this.lbX1Y = 0;
    this.lbX1Z = 0;
    this.lbY1X = 0;
    this.lbY1Y = 0;
    this.lbY1Z = 0;
    this.lbZ1X = 0;
    this.lbZ1Y = 0;
    this.lbZ1Z = 0;
    this.lbX2X = 0;
    this.lbX2Y = 0;
    this.lbX2Z = 0;
    this.lbY2X = 0;
    this.lbY2Y = 0;
    this.lbY2Z = 0;
    this.lbZ2X = 0;
    this.lbZ2Y = 0;
    this.lbZ2Z = 0;
    this.bX1X = 0;
    this.bX1Y = 0;
    this.bX1Z = 0;
    this.bY1X = 0;
    this.bY1Y = 0;
    this.bY1Z = 0;
    this.bZ1X = 0;
    this.bZ1Y = 0;
    this.bZ1Z = 0;
    this.bX2X = 0;
    this.bX2Y = 0;
    this.bX2Z = 0;
    this.bY2X = 0;
    this.bY2Y = 0;
    this.bY2Z = 0;
    this.bZ2X = 0;
    this.bZ2Y = 0;
    this.bZ2Z = 0;
    this.impulses = [];
    /** 上一步作用在刚体 1 上的约束力、力矩（求解器算出来） */
    this.appliedForceX = 0;
    this.appliedForceY = 0;
    this.appliedForceZ = 0;
    this.appliedTorqueX = 0;
    this.appliedTorqueY = 0;
    this.appliedTorqueZ = 0;
    this.userData = null;
    this.link1 = new JointLink(this);
    this.link2 = new JointLink(this);
    this.type = type;
    if (config.rigidBody1 === null || config.rigidBody2 === null) throw new Error("\u5173\u8282\u914D\u7F6E\u7F3A\u5C11\u521A\u4F53");
    this.b1 = config.rigidBody1;
    this.b2 = config.rigidBody2;
    this.allowCollision = config.allowCollision;
    this.breakForce = config.breakForce;
    this.breakTorque = config.breakTorque;
    this.solver = config.solverType === ConstraintSolverType.DIRECT ? new DirectJointConstraintSolver(this) : new PgsJointConstraintSolver(this);
    this.la1X = config.localAnchor1.x;
    this.la1Y = config.localAnchor1.y;
    this.la1Z = config.localAnchor1.z;
    this.la2X = config.localAnchor2.x;
    this.la2Y = config.localAnchor2.y;
    this.la2Z = config.localAnchor2.z;
    for (let i = 0; i < MAX_JACOBIAN_ROWS; i++) this.impulses.push(new JointImpulse());
  }
  /**
   * 由两个刚体上的局部 X 轴构造两组局部坐标系：
   * 刚体 1 的 Y、Z 取和 X1 垂直的任意方向，刚体 2 的坐标系是刚体 1 的绕最短弧从 X1 转到 X2
   */
  buildLocalBasesFromX() {
    if (this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z === 0) {
      this.lbX1X = 1;
      this.lbX1Y = 0;
      this.lbX1Z = 0;
    } else {
      let l = this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbX1X *= l;
      this.lbX1Y *= l;
      this.lbX1Z *= l;
    }
    if (this.lbX2X * this.lbX2X + this.lbX2Y * this.lbX2Y + this.lbX2Z * this.lbX2Z === 0) {
      this.lbX2X = 1;
      this.lbX2Y = 0;
      this.lbX2Z = 0;
    } else {
      let l = this.lbX2X * this.lbX2X + this.lbX2Y * this.lbX2Y + this.lbX2Z * this.lbX2Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbX2X *= l;
      this.lbX2Y *= l;
      this.lbX2Z *= l;
    }
    arc(this.lbX1X, this.lbX1Y, this.lbX1Z, this.lbX2X, this.lbX2Y, this.lbX2Z);
    mat3FromQuat(ARC.x, ARC.y, ARC.z, ARC.w);
    perp(this.lbX1X, this.lbX1Y, this.lbX1Z);
    this.lbY1X = PERP.x;
    this.lbY1Y = PERP.y;
    this.lbY1Z = PERP.z;
    this.lbZ1X = this.lbX1Y * this.lbY1Z - this.lbX1Z * this.lbY1Y;
    this.lbZ1Y = this.lbX1Z * this.lbY1X - this.lbX1X * this.lbY1Z;
    this.lbZ1Z = this.lbX1X * this.lbY1Y - this.lbX1Y * this.lbY1X;
    const m = ROT;
    this.lbX2X = m.m00 * this.lbX1X + m.m01 * this.lbX1Y + m.m02 * this.lbX1Z;
    this.lbX2Y = m.m10 * this.lbX1X + m.m11 * this.lbX1Y + m.m12 * this.lbX1Z;
    this.lbX2Z = m.m20 * this.lbX1X + m.m21 * this.lbX1Y + m.m22 * this.lbX1Z;
    this.lbY2X = m.m00 * this.lbY1X + m.m01 * this.lbY1Y + m.m02 * this.lbY1Z;
    this.lbY2Y = m.m10 * this.lbY1X + m.m11 * this.lbY1Y + m.m12 * this.lbY1Z;
    this.lbY2Z = m.m20 * this.lbY1X + m.m21 * this.lbY1Y + m.m22 * this.lbY1Z;
    this.lbZ2X = m.m00 * this.lbZ1X + m.m01 * this.lbZ1Y + m.m02 * this.lbZ1Z;
    this.lbZ2Y = m.m10 * this.lbZ1X + m.m11 * this.lbZ1Y + m.m12 * this.lbZ1Z;
    this.lbZ2Z = m.m20 * this.lbZ1X + m.m21 * this.lbZ1Y + m.m22 * this.lbZ1Z;
  }
  /**
   * 由刚体 1 的局部 X 轴、刚体 2 的局部 Z 轴构造两组局部坐标系（万向节用）：
   * 按刚体当前的朝向算出世界坐标的 Y = Z2 × X1（平行时取和 X1 垂直的方向），再补齐两组坐标系
   */
  buildLocalBasesFromX1Z2() {
    if (this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z === 0) {
      this.lbX1X = 1;
      this.lbX1Y = 0;
      this.lbX1Z = 0;
    } else {
      let l = this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbX1X *= l;
      this.lbX1Y *= l;
      this.lbX1Z *= l;
    }
    if (this.lbZ2X * this.lbZ2X + this.lbZ2Y * this.lbZ2Y + this.lbZ2Z * this.lbZ2Z === 0) {
      this.lbZ2X = 0;
      this.lbZ2Y = 0;
      this.lbZ2Z = 1;
    } else {
      let l = this.lbZ2X * this.lbZ2X + this.lbZ2Y * this.lbZ2Y + this.lbZ2Z * this.lbZ2Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbZ2X *= l;
      this.lbZ2Y *= l;
      this.lbZ2Z *= l;
    }
    const tf1 = this.b1.transform, tf2 = this.b2.transform;
    const x1X = tf1.r00 * this.lbX1X + tf1.r01 * this.lbX1Y + tf1.r02 * this.lbX1Z;
    const x1Y = tf1.r10 * this.lbX1X + tf1.r11 * this.lbX1Y + tf1.r12 * this.lbX1Z;
    const x1Z = tf1.r20 * this.lbX1X + tf1.r21 * this.lbX1Y + tf1.r22 * this.lbX1Z;
    const z2X = tf2.r00 * this.lbZ2X + tf2.r01 * this.lbZ2Y + tf2.r02 * this.lbZ2Z;
    const z2Y = tf2.r10 * this.lbZ2X + tf2.r11 * this.lbZ2Y + tf2.r12 * this.lbZ2Z;
    const z2Z = tf2.r20 * this.lbZ2X + tf2.r21 * this.lbZ2Y + tf2.r22 * this.lbZ2Z;
    let yX = z2Y * x1Z - z2Z * x1Y, yY = z2Z * x1X - z2X * x1Z, yZ = z2X * x1Y - z2Y * x1X;
    if (yX * yX + yY * yY + yZ * yZ === 0) {
      perp(x1X, x1Y, x1Z);
      yX = PERP.x;
      yY = PERP.y;
      yZ = PERP.z;
    }
    const z1X = x1Y * yZ - x1Z * yY, z1Y = x1Z * yX - x1X * yZ, z1Z = x1X * yY - x1Y * yX;
    const x2X = yY * z2Z - yZ * z2Y, x2Y = yZ * z2X - yX * z2Z, x2Z = yX * z2Y - yY * z2X;
    this.lbX1X = tf1.r00 * x1X + tf1.r10 * x1Y + tf1.r20 * x1Z;
    this.lbX1Y = tf1.r01 * x1X + tf1.r11 * x1Y + tf1.r21 * x1Z;
    this.lbX1Z = tf1.r02 * x1X + tf1.r12 * x1Y + tf1.r22 * x1Z;
    this.lbY1X = tf1.r00 * yX + tf1.r10 * yY + tf1.r20 * yZ;
    this.lbY1Y = tf1.r01 * yX + tf1.r11 * yY + tf1.r21 * yZ;
    this.lbY1Z = tf1.r02 * yX + tf1.r12 * yY + tf1.r22 * yZ;
    this.lbZ1X = tf1.r00 * z1X + tf1.r10 * z1Y + tf1.r20 * z1Z;
    this.lbZ1Y = tf1.r01 * z1X + tf1.r11 * z1Y + tf1.r21 * z1Z;
    this.lbZ1Z = tf1.r02 * z1X + tf1.r12 * z1Y + tf1.r22 * z1Z;
    this.lbX2X = tf2.r00 * x2X + tf2.r10 * x2Y + tf2.r20 * x2Z;
    this.lbX2Y = tf2.r01 * x2X + tf2.r11 * x2Y + tf2.r21 * x2Z;
    this.lbX2Z = tf2.r02 * x2X + tf2.r12 * x2Y + tf2.r22 * x2Z;
    this.lbY2X = tf2.r00 * yX + tf2.r10 * yY + tf2.r20 * yZ;
    this.lbY2Y = tf2.r01 * yX + tf2.r11 * yY + tf2.r21 * yZ;
    this.lbY2Z = tf2.r02 * yX + tf2.r12 * yY + tf2.r22 * yZ;
    this.lbZ2X = tf2.r00 * z2X + tf2.r10 * z2Y + tf2.r20 * z2Z;
    this.lbZ2Y = tf2.r01 * z2X + tf2.r11 * z2Y + tf2.r21 * z2Z;
    this.lbZ2Z = tf2.r02 * z2X + tf2.r12 * z2Y + tf2.r22 * z2Z;
  }
  /**
   * 由局部 X1、Y1 和 X2 构造关节坐标系。
   * Y1 与 X1 正交化，两组坐标系通过最短弧关联；X2 归一化，零向量取 (1, 0, 0)。
   */
  buildLocalBasesFromXY1X2() {
    if (this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z === 0) {
      this.lbX1X = 1;
      this.lbX1Y = 0;
      this.lbX1Z = 0;
    } else {
      let l = this.lbX1X * this.lbX1X + this.lbX1Y * this.lbX1Y + this.lbX1Z * this.lbX1Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbX1X *= l;
      this.lbX1Y *= l;
      this.lbX1Z *= l;
    }
    this.lbZ1X = this.lbX1Y * this.lbY1Z - this.lbX1Z * this.lbY1Y;
    this.lbZ1Y = this.lbX1Z * this.lbY1X - this.lbX1X * this.lbY1Z;
    this.lbZ1Z = this.lbX1X * this.lbY1Y - this.lbX1Y * this.lbY1X;
    if (this.lbZ1X * this.lbZ1X + this.lbZ1Y * this.lbZ1Y + this.lbZ1Z * this.lbZ1Z === 0) {
      perp(this.lbX1X, this.lbX1Y, this.lbX1Z);
      this.lbY1X = PERP.x;
      this.lbY1Y = PERP.y;
      this.lbY1Z = PERP.z;
      this.lbZ1X = this.lbX1Y * this.lbY1Z - this.lbX1Z * this.lbY1Y;
      this.lbZ1Y = this.lbX1Z * this.lbY1X - this.lbX1X * this.lbY1Z;
      this.lbZ1Z = this.lbX1X * this.lbY1Y - this.lbX1Y * this.lbY1X;
    } else {
      let l = this.lbZ1X * this.lbZ1X + this.lbZ1Y * this.lbZ1Y + this.lbZ1Z * this.lbZ1Z;
      if (l > 0) l = 1 / Math.sqrt(l);
      this.lbZ1X *= l;
      this.lbZ1Y *= l;
      this.lbZ1Z *= l;
      this.lbY1X = this.lbZ1Y * this.lbX1Z - this.lbZ1Z * this.lbX1Y;
      this.lbY1Y = this.lbZ1Z * this.lbX1X - this.lbZ1X * this.lbX1Z;
      this.lbY1Z = this.lbZ1X * this.lbX1Y - this.lbZ1Y * this.lbX1X;
    }
    if (!compat.keepOimoBugs) {
      if (this.lbX2X * this.lbX2X + this.lbX2Y * this.lbX2Y + this.lbX2Z * this.lbX2Z === 0) {
        this.lbX2X = 1;
        this.lbX2Y = 0;
        this.lbX2Z = 0;
      } else {
        let l = this.lbX2X * this.lbX2X + this.lbX2Y * this.lbX2Y + this.lbX2Z * this.lbX2Z;
        if (l > 0) l = 1 / Math.sqrt(l);
        this.lbX2X *= l;
        this.lbX2Y *= l;
        this.lbX2Z *= l;
      }
    }
    arc(this.lbX1X, this.lbX1Y, this.lbX1Z, this.lbX2X, this.lbX2Y, this.lbX2Z);
    mat3FromQuat(ARC.x, ARC.y, ARC.z, ARC.w);
    const m = ROT;
    this.lbX2X = m.m00 * this.lbX1X + m.m01 * this.lbX1Y + m.m02 * this.lbX1Z;
    this.lbX2Y = m.m10 * this.lbX1X + m.m11 * this.lbX1Y + m.m12 * this.lbX1Z;
    this.lbX2Z = m.m20 * this.lbX1X + m.m21 * this.lbX1Y + m.m22 * this.lbX1Z;
    this.lbY2X = m.m00 * this.lbY1X + m.m01 * this.lbY1Y + m.m02 * this.lbY1Z;
    this.lbY2Y = m.m10 * this.lbY1X + m.m11 * this.lbY1Y + m.m12 * this.lbY1Z;
    this.lbY2Z = m.m20 * this.lbY1X + m.m21 * this.lbY1Y + m.m22 * this.lbY1Z;
    this.lbZ2X = m.m00 * this.lbZ1X + m.m01 * this.lbZ1Y + m.m02 * this.lbZ1Z;
    this.lbZ2Y = m.m10 * this.lbZ1X + m.m11 * this.lbZ1Y + m.m12 * this.lbZ1Z;
    this.lbZ2Z = m.m20 * this.lbZ1X + m.m21 * this.lbZ1Y + m.m22 * this.lbZ1Z;
  }
  /**
   * 填一行平移方向的约束：限制、马达、弹簧。
   * diff 是当前位移；mass 是沿这个方向的有效质量（马达和弹簧用）
   */
  setSolverInfoRowLinear(row, diff, lm, mass, sd, timeStep, isPositionPart) {
    let cfmFactor, erp;
    let slop = LINEAR_SLOP;
    if (isPositionPart) {
      cfmFactor = 0;
      erp = 1;
    } else {
      if (sd.frequency > 0) {
        slop = 0;
        computeSoftConstraintParameters(sd.frequency, sd.dampingRatio, timeStep.dt, sd.useSymplecticEuler);
        cfmFactor = SOFT.cfmFactor;
        erp = SOFT.erp;
      } else {
        cfmFactor = 0;
        erp = this.getErp(timeStep, false);
      }
      if (lm.motorForce > 0) row.motor(lm.motorSpeed, lm.motorForce * timeStep.dt);
      else row.motor(0, 0);
    }
    const lower = lm.lowerLimit, upper = lm.upperLimit;
    let minImp, maxImp, error;
    if (lower > upper) {
      minImp = 0;
      maxImp = 0;
      error = 0;
    } else if (lower === upper) {
      minImp = -Infinity;
      maxImp = Infinity;
      error = diff - lower;
    } else if (diff < lower) {
      minImp = -Infinity;
      maxImp = 0;
      error = diff - lower + slop;
      if (error > 0) error = 0;
    } else if (diff > upper) {
      minImp = 0;
      maxImp = Infinity;
      error = diff - upper - slop;
      if (error < 0) error = 0;
    } else {
      minImp = 0;
      maxImp = 0;
      error = 0;
    }
    row.minImpulse = minImp;
    row.maxImpulse = maxImp;
    row.cfm = cfmFactor * (mass === 0 ? 0 : 1 / mass);
    row.rhs = error * erp;
  }
  /**
   * 填一行转动方向的约束：限制、马达、弹簧。
   * diff 是当前转角，先换算到 [限制中点 - π, 限制中点 + π]；mass 是绕这个轴的有效转动惯量（马达和弹簧用）
   */
  setSolverInfoRowAngular(row, diff, lm, mass, sd, timeStep, isPositionPart) {
    let cfmFactor, erp;
    let slop = ANGULAR_SLOP;
    if (isPositionPart) {
      cfmFactor = 0;
      erp = 1;
    } else {
      if (sd.frequency > 0) {
        slop = 0;
        computeSoftConstraintParameters(sd.frequency, sd.dampingRatio, timeStep.dt, sd.useSymplecticEuler);
        cfmFactor = SOFT.cfmFactor;
        erp = SOFT.erp;
      } else {
        cfmFactor = 0;
        erp = this.getErp(timeStep, false);
      }
      if (lm.motorTorque > 0) row.motor(lm.motorSpeed, lm.motorTorque * timeStep.dt);
      else row.motor(0, 0);
    }
    const lower = lm.lowerLimit, upper = lm.upperLimit;
    const mid = (lower + upper) * 0.5;
    diff -= mid;
    diff = ((diff + 3.14159265358979) % 6.28318530717958 + 6.28318530717958) % 6.28318530717958 - 3.14159265358979;
    diff += mid;
    let minImp, maxImp, error;
    if (lower > upper) {
      minImp = 0;
      maxImp = 0;
      error = 0;
    } else if (lower === upper) {
      minImp = -Infinity;
      maxImp = Infinity;
      error = diff - lower;
    } else if (diff < lower) {
      minImp = -Infinity;
      maxImp = 0;
      error = diff - lower + slop;
      if (error > 0) error = 0;
    } else if (diff > upper) {
      minImp = 0;
      maxImp = Infinity;
      error = diff - upper - slop;
      if (error < 0) error = 0;
    } else {
      minImp = 0;
      maxImp = 0;
      error = 0;
    }
    row.minImpulse = minImp;
    row.maxImpulse = maxImp;
    row.cfm = cfmFactor * (mass === 0 ? 0 : 1 / mass);
    row.rhs = error * erp;
  }
  /** 位置修正的 ERP：Baumgarte 时是 VELOCITY_BAUMGARTE / dt，否则 0（位置修正阶段总是 1） */
  getErp(timeStep, isPositionPart) {
    if (isPositionPart) return 1;
    if (this.positionCorrectionAlgorithm === PositionCorrectionAlgorithm.BAUMGARTE) return timeStep.invDt * VELOCITY_BAUMGARTE;
    return 0;
  }
  /**
   * 两个刚体绕世界坐标的单位轴 axis 转动的有效转动惯量。
   * 锚点不在重心上时，把绕锚点转动需要的平动也算进去
   */
  computeEffectiveInertiaMoment(axisX, axisY, axisZ) {
    const b1 = this.b1, b2 = this.b2;
    const ia1X = b1.ii00 * axisX + b1.ii01 * axisY + b1.ii02 * axisZ;
    const ia1Y = b1.ii10 * axisX + b1.ii11 * axisY + b1.ii12 * axisZ;
    const ia1Z = b1.ii20 * axisX + b1.ii21 * axisY + b1.ii22 * axisZ;
    const ia2X = b2.ii00 * axisX + b2.ii01 * axisY + b2.ii02 * axisZ;
    const ia2Y = b2.ii10 * axisX + b2.ii11 * axisY + b2.ii12 * axisZ;
    const ia2Z = b2.ii20 * axisX + b2.ii21 * axisY + b2.ii22 * axisZ;
    let invI1 = ia1X * axisX + ia1Y * axisY + ia1Z * axisZ;
    let invI2 = ia2X * axisX + ia2Y * axisY + ia2Z * axisZ;
    if (invI1 > 0) {
      const dot = axisX * this.ra1X + axisY * this.ra1Y + axisZ * this.ra1Z;
      const projsq = this.ra1X * this.ra1X + this.ra1Y * this.ra1Y + this.ra1Z * this.ra1Z - dot * dot;
      if (projsq > 0) {
        if (b1.invMass > 0) invI1 = 1 / (1 / invI1 + b1.mass * projsq);
        else invI1 = 0;
      }
    }
    if (invI2 > 0) {
      const dot = axisX * this.ra2X + axisY * this.ra2Y + axisZ * this.ra2Z;
      const projsq = this.ra2X * this.ra2X + this.ra2Y * this.ra2Y + this.ra2Z * this.ra2Z - dot * dot;
      if (projsq > 0) {
        if (b2.invMass > 0) invI2 = 1 / (1 / invI2 + b2.mass * projsq);
        else invI2 = 0;
      }
    }
    return invI1 + invI2 === 0 ? 0 : 1 / (invI1 + invI2);
  }
  /** 热启动时上一步冲量保留的比例 */
  getWarmStartingFactor() {
    return this.positionCorrectionAlgorithm === PositionCorrectionAlgorithm.BAUMGARTE ? JOINT_WARM_STARTING_FACTOR_FOR_BAUMGARTE : JOINT_WARM_STARTING_FACTOR;
  }
  /** 按当前变换更新锚点及约束坐标系，供约束求解使用。 */
  syncAnchors() {
    const tf1 = this.b1.transform, tf2 = this.b2.transform;
    this.ra1X = tf1.r00 * this.la1X + tf1.r01 * this.la1Y + tf1.r02 * this.la1Z;
    this.ra1Y = tf1.r10 * this.la1X + tf1.r11 * this.la1Y + tf1.r12 * this.la1Z;
    this.ra1Z = tf1.r20 * this.la1X + tf1.r21 * this.la1Y + tf1.r22 * this.la1Z;
    this.ra2X = tf2.r00 * this.la2X + tf2.r01 * this.la2Y + tf2.r02 * this.la2Z;
    this.ra2Y = tf2.r10 * this.la2X + tf2.r11 * this.la2Y + tf2.r12 * this.la2Z;
    this.ra2Z = tf2.r20 * this.la2X + tf2.r21 * this.la2Y + tf2.r22 * this.la2Z;
    this.a1X = this.ra1X + tf1.px;
    this.a1Y = this.ra1Y + tf1.py;
    this.a1Z = this.ra1Z + tf1.pz;
    this.a2X = this.ra2X + tf2.px;
    this.a2Y = this.ra2Y + tf2.py;
    this.a2Z = this.ra2Z + tf2.pz;
    this.bX1X = tf1.r00 * this.lbX1X + tf1.r01 * this.lbX1Y + tf1.r02 * this.lbX1Z;
    this.bX1Y = tf1.r10 * this.lbX1X + tf1.r11 * this.lbX1Y + tf1.r12 * this.lbX1Z;
    this.bX1Z = tf1.r20 * this.lbX1X + tf1.r21 * this.lbX1Y + tf1.r22 * this.lbX1Z;
    this.bY1X = tf1.r00 * this.lbY1X + tf1.r01 * this.lbY1Y + tf1.r02 * this.lbY1Z;
    this.bY1Y = tf1.r10 * this.lbY1X + tf1.r11 * this.lbY1Y + tf1.r12 * this.lbY1Z;
    this.bY1Z = tf1.r20 * this.lbY1X + tf1.r21 * this.lbY1Y + tf1.r22 * this.lbY1Z;
    this.bZ1X = tf1.r00 * this.lbZ1X + tf1.r01 * this.lbZ1Y + tf1.r02 * this.lbZ1Z;
    this.bZ1Y = tf1.r10 * this.lbZ1X + tf1.r11 * this.lbZ1Y + tf1.r12 * this.lbZ1Z;
    this.bZ1Z = tf1.r20 * this.lbZ1X + tf1.r21 * this.lbZ1Y + tf1.r22 * this.lbZ1Z;
    this.bX2X = tf2.r00 * this.lbX2X + tf2.r01 * this.lbX2Y + tf2.r02 * this.lbX2Z;
    this.bX2Y = tf2.r10 * this.lbX2X + tf2.r11 * this.lbX2Y + tf2.r12 * this.lbX2Z;
    this.bX2Z = tf2.r20 * this.lbX2X + tf2.r21 * this.lbX2Y + tf2.r22 * this.lbX2Z;
    this.bY2X = tf2.r00 * this.lbY2X + tf2.r01 * this.lbY2Y + tf2.r02 * this.lbY2Z;
    this.bY2Y = tf2.r10 * this.lbY2X + tf2.r11 * this.lbY2Y + tf2.r12 * this.lbY2Z;
    this.bY2Z = tf2.r20 * this.lbY2X + tf2.r21 * this.lbY2Y + tf2.r22 * this.lbY2Z;
    this.bZ2X = tf2.r00 * this.lbZ2X + tf2.r01 * this.lbZ2Y + tf2.r02 * this.lbZ2Z;
    this.bZ2Y = tf2.r10 * this.lbZ2X + tf2.r11 * this.lbZ2Y + tf2.r12 * this.lbZ2Z;
    this.bZ2Z = tf2.r20 * this.lbZ2X + tf2.r21 * this.lbZ2Y + tf2.r22 * this.lbZ2Z;
  }
  /** 初始化速度约束行，子类随后追加各自由度的约束。 */
  getVelocitySolverInfo(_timeStep, info) {
    info.b1 = this.b1;
    info.b2 = this.b2;
    info.numRows = 0;
  }
  /** 位置修正要用的约束行 */
  getPositionSolverInfo(info) {
    info.b1 = this.b1;
    info.b2 = this.b2;
    info.numRows = 0;
  }
  /** 约束力或力矩超过阈值时把关节从世界里删掉 */
  checkDestruction() {
    const forceSq = this.appliedForceX * this.appliedForceX + this.appliedForceY * this.appliedForceY + this.appliedForceZ * this.appliedForceZ;
    const torqueSq = this.appliedTorqueX * this.appliedTorqueX + this.appliedTorqueY * this.appliedTorqueY + this.appliedTorqueZ * this.appliedTorqueZ;
    if (this.breakForce > 0 && forceSq > this.breakForce * this.breakForce) {
      this.world.removeJoint(this);
      return;
    }
    if (this.breakTorque > 0 && torqueSq > this.breakTorque * this.breakTorque) {
      this.world.removeJoint(this);
      return;
    }
  }
  /** 挂到两个刚体的关节链表上，并唤醒它们 */
  attachLinks() {
    const b1 = this.b1, b2 = this.b2, l1 = this.link1, l2 = this.link2;
    l1.other = b2;
    l2.other = b1;
    if (b1.jointLinkList === null) {
      b1.jointLinkList = l1;
      b1.jointLinkListLast = l1;
    } else {
      b1.jointLinkListLast.next = l1;
      l1.prev = b1.jointLinkListLast;
      b1.jointLinkListLast = l1;
    }
    if (b2.jointLinkList === null) {
      b2.jointLinkList = l2;
      b2.jointLinkListLast = l2;
    } else {
      b2.jointLinkListLast.next = l2;
      l2.prev = b2.jointLinkListLast;
      b2.jointLinkListLast = l2;
    }
    b1.numJointLinks++;
    b2.numJointLinks++;
    b1.wakeUp();
    b2.wakeUp();
  }
  /** 从两个刚体的关节链表上摘下来，并唤醒它们 */
  detachLinks() {
    const b1 = this.b1, b2 = this.b2;
    removeLink(b1, this.link1);
    removeLink(b2, this.link2);
    this.link1.other = null;
    this.link2.other = null;
    b1.numJointLinks--;
    b2.numJointLinks--;
    b1.wakeUp();
    b2.wakeUp();
  }
};
function removeLink(b, l) {
  const prev = l.prev, next = l.next;
  if (prev !== null) prev.next = next;
  if (next !== null) next.prev = prev;
  if (l === b.jointLinkList) b.jointLinkList = b.jointLinkList.next;
  if (l === b.jointLinkListLast) b.jointLinkListLast = b.jointLinkListLast.prev;
  l.next = null;
  l.prev = null;
}

// src/core/dynamics/constraint/joint/rotational-limit-motor.ts
var RotationalLimitMotor = class _RotationalLimitMotor {
  constructor() {
    /** 转角下限（弧度）。lowerLimit > upperLimit 时不限制，相等时锁死在这个角度 */
    this.lowerLimit = 1;
    /** 转角上限（弧度） */
    this.upperLimit = 0;
    /** 马达目标角速度，单位为弧度/秒，默认为 0。 */
    this.motorSpeed = 0;
    /** 马达的最大力矩（N·m），不大于 0 时马达关闭 */
    this.motorTorque = 0;
  }
  setLimits(lower, upper) {
    this.lowerLimit = lower;
    this.upperLimit = upper;
    return this;
  }
  setMotor(speed, torque) {
    this.motorSpeed = speed;
    this.motorTorque = torque;
    return this;
  }
  clone() {
    const lm = new _RotationalLimitMotor();
    lm.lowerLimit = this.lowerLimit;
    lm.upperLimit = this.upperLimit;
    lm.motorSpeed = this.motorSpeed;
    lm.motorTorque = this.motorTorque;
    return lm;
  }
};

// src/core/dynamics/constraint/joint/spring-damper.ts
var SpringDamper = class _SpringDamper {
  constructor() {
    /** 弹簧的频率（Hz） */
    this.frequency = 0;
    /** 阻尼比：1 是临界阻尼 */
    this.dampingRatio = 0;
    /** true：辛欧拉（能量守恒好，但高频时不稳定）；false：隐式欧拉（会损失能量，但一定稳定） */
    this.useSymplecticEuler = false;
  }
  setSpring(frequency, dampingRatio) {
    this.frequency = frequency;
    this.dampingRatio = dampingRatio;
    return this;
  }
  setSymplecticEuler(useSymplecticEuler) {
    this.useSymplecticEuler = useSymplecticEuler;
    return this;
  }
  clone() {
    const sd = new _SpringDamper();
    sd.frequency = this.frequency;
    sd.dampingRatio = this.dampingRatio;
    sd.useSymplecticEuler = this.useSymplecticEuler;
    return sd;
  }
};

// src/core/dynamics/constraint/joint/translational-limit-motor.ts
var TranslationalLimitMotor = class _TranslationalLimitMotor {
  constructor() {
    /** 位移下限（米）。lowerLimit > upperLimit 时不限制，相等时锁死在这个位置 */
    this.lowerLimit = 1;
    /** 位移上限（米） */
    this.upperLimit = 0;
    /** 马达目标速度，单位为米/秒，默认为 0。 */
    this.motorSpeed = 0;
    /** 马达的最大推力（N），不大于 0 时马达关闭 */
    this.motorForce = 0;
  }
  setLimits(lower, upper) {
    this.lowerLimit = lower;
    this.upperLimit = upper;
    return this;
  }
  setMotor(speed, force) {
    this.motorSpeed = speed;
    this.motorForce = force;
    return this;
  }
  clone() {
    const lm = new _TranslationalLimitMotor();
    lm.lowerLimit = this.lowerLimit;
    lm.upperLimit = this.upperLimit;
    lm.motorSpeed = this.motorSpeed;
    lm.motorForce = this.motorForce;
    return lm;
  }
};

// src/core/dynamics/constraint/joint/cylindrical-joint.ts
var CylindricalJointConfig = class extends JointConfig {
  constructor() {
    super(...arguments);
    /** 轴在两个刚体上的局部方向 */
    this.localAxis1 = new Vec3(1, 0, 0);
    this.localAxis2 = new Vec3(1, 0, 0);
    /** 沿轴平移的限制和马达、弹簧阻尼 */
    this.translationalLimitMotor = new TranslationalLimitMotor();
    this.translationalSpringDamper = new SpringDamper();
    /** 绕轴转动的限制和马达、弹簧阻尼 */
    this.rotationalLimitMotor = new RotationalLimitMotor();
    this.rotationalSpringDamper = new SpringDamper();
  }
  /** 两个刚体在世界坐标的 anchor 处连起来，可以沿世界坐标的 axis 平移、绕它转动 */
  init(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ, axisX, axisY, axisZ) {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    toLocalVector(rigidBody1, axisX, axisY, axisZ, this.localAxis1);
    toLocalVector(rigidBody2, axisX, axisY, axisZ, this.localAxis2);
    return this;
  }
};
var CylindricalJoint = class extends Joint {
  constructor(config) {
    super(config, JointType.CYLINDRICAL);
    /** 绕轴的转角（弧度）、沿轴的位移，同步锚点时更新 */
    this.angle = 0;
    this.translation = 0;
    /** 两个刚体的轴之间的偏差、两个锚点之间的偏差，沿公共坐标系的 y、z 分解 */
    this.angularErrorY = 0;
    this.angularErrorZ = 0;
    this.linearErrorY = 0;
    this.linearErrorZ = 0;
    const a1 = config.localAxis1, a2 = config.localAxis2;
    this.lbX1X = a1.x;
    this.lbX1Y = a1.y;
    this.lbX1Z = a1.z;
    this.lbX2X = a2.x;
    this.lbX2Y = a2.y;
    this.lbX2Z = a2.z;
    this.buildLocalBasesFromX();
    this.basis = new BasisTracker(this);
    this.translSd = config.translationalSpringDamper.clone();
    this.translLm = config.translationalLimitMotor.clone();
    this.rotSd = config.rotationalSpringDamper.clone();
    this.rotLm = config.rotationalLimitMotor.clone();
  }
  getInfo(info, timeStep, isPositionPart) {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const angRhsY = this.angularErrorY * erp, angRhsZ = this.angularErrorZ * erp;
    const b = this.basis;
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const translationalMotorMass = 1 / (this.b1.invMass + this.b2.invMass);
    const rotationalMotorMass = this.computeEffectiveInertiaMoment(b.xX, b.xY, b.xZ);
    let row, j;
    if (this.translSd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[0]);
      this.setSolverInfoRowLinear(row, this.translation, this.translLm, translationalMotorMass, this.translSd, timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = b.xX;
      j.lin1Y = b.xY;
      j.lin1Z = b.xZ;
      j.lin2X = b.xX;
      j.lin2Y = b.xY;
      j.lin2Z = b.xZ;
      j.ang1X = r1Y * b.xZ - r1Z * b.xY;
      j.ang1Y = r1Z * b.xX - r1X * b.xZ;
      j.ang1Z = r1X * b.xY - r1Y * b.xX;
      j.ang2X = r2Y * b.xZ - r2Z * b.xY;
      j.ang2Y = r2Z * b.xX - r2X * b.xZ;
      j.ang2Z = r2X * b.xY - r2Y * b.xX;
    }
    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, 0);
    j = row.jacobian;
    j.lin1X = b.yX;
    j.lin1Y = b.yY;
    j.lin1Z = b.yZ;
    j.lin2X = b.yX;
    j.lin2Y = b.yY;
    j.lin2Z = b.yZ;
    j.ang1X = r1Y * b.yZ - r1Z * b.yY;
    j.ang1Y = r1Z * b.yX - r1X * b.yZ;
    j.ang1Z = r1X * b.yY - r1Y * b.yX;
    j.ang2X = r2Y * b.yZ - r2Z * b.yY;
    j.ang2Y = r2Z * b.yX - r2X * b.yZ;
    j.ang2Z = r2X * b.yY - r2Y * b.yX;
    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, 0);
    j = row.jacobian;
    j.lin1X = b.zX;
    j.lin1Y = b.zY;
    j.lin1Z = b.zZ;
    j.lin2X = b.zX;
    j.lin2Y = b.zY;
    j.lin2Z = b.zZ;
    j.ang1X = r1Y * b.zZ - r1Z * b.zY;
    j.ang1Y = r1Z * b.zX - r1X * b.zZ;
    j.ang1Z = r1X * b.zY - r1Y * b.zX;
    j.ang2X = r2Y * b.zZ - r2Z * b.zY;
    j.ang2Y = r2Z * b.zX - r2X * b.zZ;
    j.ang2Z = r2X * b.zY - r2Y * b.zX;
    if (this.rotSd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.angle, this.rotLm, rotationalMotorMass, this.rotSd, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = b.xX;
      j.ang1Y = b.xY;
      j.ang1Z = b.xZ;
      j.ang2X = b.xX;
      j.ang2Y = b.xY;
      j.ang2Z = b.xZ;
    }
    row = info.addRow(this.impulses[4]);
    row.equalLimit(angRhsY, 0);
    j = row.jacobian;
    j.ang1X = b.yX;
    j.ang1Y = b.yY;
    j.ang1Z = b.yZ;
    j.ang2X = b.yX;
    j.ang2Y = b.yY;
    j.ang2Z = b.yZ;
    row = info.addRow(this.impulses[5]);
    row.equalLimit(angRhsZ, 0);
    j = row.jacobian;
    j.ang1X = b.zX;
    j.ang1Y = b.zY;
    j.ang1Z = b.zZ;
    j.ang2X = b.zX;
    j.ang2Y = b.zY;
    j.ang2Z = b.zZ;
  }
  /** 同步锚点和坐标系，然后更新公共坐标系、转角、位移和偏差 */
  syncAnchors() {
    super.syncAnchors();
    const b = this.basis;
    b.trackByX();
    let errX = this.bX1Y * this.bX2Z - this.bX1Z * this.bX2Y;
    let errY = this.bX1Z * this.bX2X - this.bX1X * this.bX2Z;
    let errZ = this.bX1X * this.bX2Y - this.bX1Y * this.bX2X;
    let cos = this.bX1X * this.bX2X + this.bX1Y * this.bX2Y + this.bX1Z * this.bX2Z;
    const theta = cos <= -1 ? 3.14159265358979 : cos >= 1 ? 0 : Math.acos(cos);
    let l = errX * errX + errY * errY + errZ * errZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    errX *= l;
    errY *= l;
    errZ *= l;
    errX *= theta;
    errY *= theta;
    errZ *= theta;
    this.angularErrorY = errX * b.yX + errY * b.yY + errZ * b.yZ;
    this.angularErrorZ = errX * b.zX + errY * b.zY + errZ * b.zZ;
    const pcX = this.bY1Y * this.bY2Z - this.bY1Z * this.bY2Y;
    const pcY = this.bY1Z * this.bY2X - this.bY1X * this.bY2Z;
    const pcZ = this.bY1X * this.bY2Y - this.bY1Y * this.bY2X;
    cos = this.bY1X * this.bY2X + this.bY1Y * this.bY2Y + this.bY1Z * this.bY2Z;
    this.angle = cos <= -1 ? 3.14159265358979 : cos >= 1 ? 0 : Math.acos(cos);
    if (pcX * b.xX + pcY * b.xY + pcZ * b.xZ < 0) this.angle = -this.angle;
    const dX = this.a2X - this.a1X, dY = this.a2Y - this.a1Y, dZ = this.a2Z - this.a1Z;
    this.translation = dX * b.xX + dY * b.xY + dZ * b.xZ;
    this.linearErrorY = dX * b.yX + dY * b.yY + dZ * b.yZ;
    this.linearErrorZ = dX * b.zX + dY * b.zY + dZ * b.zZ;
  }
  getVelocitySolverInfo(timeStep, info) {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }
  getPositionSolverInfo(info) {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
};

// src/core/dynamics/constraint/joint/generic-joint.ts
var GenericJointConfig = class extends JointConfig {
  constructor() {
    super(...arguments);
    /** 两个刚体的约束坐标系（各自的局部坐标，三列是 X、Y、Z 轴），要是右手系，不会自动归一化、正交化 */
    this.localBasis1 = new Mat3();
    this.localBasis2 = new Mat3();
    /** 沿刚体 1 约束坐标系三个轴的平移：限制和马达、弹簧阻尼。默认上下限都是 0（锁死） */
    this.translationalLimitMotors = [
      new TranslationalLimitMotor().setLimits(0, 0),
      new TranslationalLimitMotor().setLimits(0, 0),
      new TranslationalLimitMotor().setLimits(0, 0)
    ];
    this.translationalSpringDampers = [new SpringDamper(), new SpringDamper(), new SpringDamper()];
    /** 绕相对转动的 XYZ 欧拉角三个转轴的转动：限制和马达、弹簧阻尼。默认上下限都是 0（锁死） */
    this.rotationalLimitMotors = [
      new RotationalLimitMotor().setLimits(0, 0),
      new RotationalLimitMotor().setLimits(0, 0),
      new RotationalLimitMotor().setLimits(0, 0)
    ];
    this.rotationalSpringDampers = [new SpringDamper(), new SpringDamper(), new SpringDamper()];
  }
  /** 两个刚体在世界坐标的 anchor 处连起来，约束坐标系是世界坐标的 basis1、basis2（三列是 X、Y、Z 轴） */
  init(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ, basis1, basis2) {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    const r1 = rigidBody1.transform, r2 = rigidBody2.transform, w1 = basis1, w2 = basis2;
    this.localBasis1.init(
      r1.r00 * w1.e00 + r1.r10 * w1.e10 + r1.r20 * w1.e20,
      r1.r00 * w1.e01 + r1.r10 * w1.e11 + r1.r20 * w1.e21,
      r1.r00 * w1.e02 + r1.r10 * w1.e12 + r1.r20 * w1.e22,
      r1.r01 * w1.e00 + r1.r11 * w1.e10 + r1.r21 * w1.e20,
      r1.r01 * w1.e01 + r1.r11 * w1.e11 + r1.r21 * w1.e21,
      r1.r01 * w1.e02 + r1.r11 * w1.e12 + r1.r21 * w1.e22,
      r1.r02 * w1.e00 + r1.r12 * w1.e10 + r1.r22 * w1.e20,
      r1.r02 * w1.e01 + r1.r12 * w1.e11 + r1.r22 * w1.e21,
      r1.r02 * w1.e02 + r1.r12 * w1.e12 + r1.r22 * w1.e22
    );
    this.localBasis2.init(
      r2.r00 * w2.e00 + r2.r10 * w2.e10 + r2.r20 * w2.e20,
      r2.r00 * w2.e01 + r2.r10 * w2.e11 + r2.r20 * w2.e21,
      r2.r00 * w2.e02 + r2.r10 * w2.e12 + r2.r20 * w2.e22,
      r2.r01 * w2.e00 + r2.r11 * w2.e10 + r2.r21 * w2.e20,
      r2.r01 * w2.e01 + r2.r11 * w2.e11 + r2.r21 * w2.e21,
      r2.r01 * w2.e02 + r2.r11 * w2.e12 + r2.r21 * w2.e22,
      r2.r02 * w2.e00 + r2.r12 * w2.e10 + r2.r22 * w2.e20,
      r2.r02 * w2.e01 + r2.r12 * w2.e11 + r2.r22 * w2.e21,
      r2.r02 * w2.e02 + r2.r12 * w2.e12 + r2.r22 * w2.e22
    );
    return this;
  }
};
var GenericJoint = class extends Joint {
  constructor(config) {
    super(config, JointType.GENERIC);
    /** 约束的三个转动方向（世界坐标），同步锚点时更新 */
    this.axisXX = 0;
    this.axisXY = 0;
    this.axisXZ = 0;
    this.axisYX = 0;
    this.axisYY = 0;
    this.axisYZ = 0;
    this.axisZX = 0;
    this.axisZY = 0;
    this.axisZZ = 0;
    /** 相对转动的 XYZ 欧拉角 */
    this.angleX = 0;
    this.angleY = 0;
    this.angleZ = 0;
    /** 约束方向退化成零向量时跳过对应的行 */
    this.xSingular = false;
    this.ySingular = false;
    this.zSingular = false;
    /** 两个锚点之间的偏差沿刚体 1 约束坐标系三个轴的分量 */
    this.translationX = 0;
    this.translationY = 0;
    this.translationZ = 0;
    const m1 = config.localBasis1, m2 = config.localBasis2;
    this.lbX1X = m1.e00;
    this.lbX1Y = m1.e10;
    this.lbX1Z = m1.e20;
    this.lbY1X = m1.e01;
    this.lbY1Y = m1.e11;
    this.lbY1Z = m1.e21;
    this.lbZ1X = m1.e02;
    this.lbZ1Y = m1.e12;
    this.lbZ1Z = m1.e22;
    this.lbX2X = m2.e00;
    this.lbX2Y = m2.e10;
    this.lbX2Z = m2.e20;
    this.lbY2X = m2.e01;
    this.lbY2Y = m2.e11;
    this.lbY2Z = m2.e21;
    this.lbZ2X = m2.e02;
    this.lbZ2Y = m2.e12;
    this.lbZ2Z = m2.e22;
    const tl = config.translationalLimitMotors, ts = config.translationalSpringDampers;
    const rl = config.rotationalLimitMotors, rs = config.rotationalSpringDampers;
    this.translLms = [tl[0].clone(), tl[1].clone(), tl[2].clone()];
    this.translSds = [ts[0].clone(), ts[1].clone(), ts[2].clone()];
    this.rotLms = [rl[0].clone(), rl[1].clone(), rl[2].clone()];
    this.rotSds = [rs[0].clone(), rs[1].clone(), rs[2].clone()];
  }
  getInfo(info, timeStep, isPositionPart) {
    const translMotorMass = 1 / (this.b1.invMass + this.b2.invMass);
    const motorMassX = this.computeEffectiveInertiaMoment(this.axisXX, this.axisXY, this.axisXZ);
    const motorMassY = this.computeEffectiveInertiaMoment(this.axisYX, this.axisYY, this.axisYZ);
    const motorMassZ = this.computeEffectiveInertiaMoment(this.axisZX, this.axisZY, this.axisZZ);
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const xX = this.bX1X, xY = this.bX1Y, xZ = this.bX1Z;
    const yX = this.bY1X, yY = this.bY1Y, yZ = this.bY1Z;
    const zX = this.bZ1X, zY = this.bZ1Y, zZ = this.bZ1Z;
    let row, j;
    if (this.translSds[0].frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[0]);
      this.setSolverInfoRowLinear(row, this.translationX, this.translLms[0], translMotorMass, this.translSds[0], timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = xX;
      j.lin1Y = xY;
      j.lin1Z = xZ;
      j.lin2X = xX;
      j.lin2Y = xY;
      j.lin2Z = xZ;
      j.ang1X = r1Y * xZ - r1Z * xY;
      j.ang1Y = r1Z * xX - r1X * xZ;
      j.ang1Z = r1X * xY - r1Y * xX;
      j.ang2X = r2Y * xZ - r2Z * xY;
      j.ang2Y = r2Z * xX - r2X * xZ;
      j.ang2Z = r2X * xY - r2Y * xX;
    }
    if (this.translSds[1].frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[1]);
      this.setSolverInfoRowLinear(row, this.translationY, this.translLms[1], translMotorMass, this.translSds[1], timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = yX;
      j.lin1Y = yY;
      j.lin1Z = yZ;
      j.lin2X = yX;
      j.lin2Y = yY;
      j.lin2Z = yZ;
      j.ang1X = r1Y * yZ - r1Z * yY;
      j.ang1Y = r1Z * yX - r1X * yZ;
      j.ang1Z = r1X * yY - r1Y * yX;
      j.ang2X = r2Y * yZ - r2Z * yY;
      j.ang2Y = r2Z * yX - r2X * yZ;
      j.ang2Z = r2X * yY - r2Y * yX;
    }
    if (this.translSds[2].frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[2]);
      this.setSolverInfoRowLinear(row, this.translationZ, this.translLms[2], translMotorMass, this.translSds[2], timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = zX;
      j.lin1Y = zY;
      j.lin1Z = zZ;
      j.lin2X = zX;
      j.lin2Y = zY;
      j.lin2Z = zZ;
      j.ang1X = r1Y * zZ - r1Z * zY;
      j.ang1Y = r1Z * zX - r1X * zZ;
      j.ang1Z = r1X * zY - r1Y * zX;
      j.ang2X = r2Y * zZ - r2Z * zY;
      j.ang2Y = r2Z * zX - r2X * zZ;
      j.ang2Z = r2X * zY - r2Y * zX;
    }
    if (!this.xSingular && (this.rotSds[0].frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.angleX, this.rotLms[0], motorMassX, this.rotSds[0], timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisXX;
      j.ang1Y = this.axisXY;
      j.ang1Z = this.axisXZ;
      j.ang2X = this.axisXX;
      j.ang2Y = this.axisXY;
      j.ang2Z = this.axisXZ;
    }
    if (!this.ySingular && (this.rotSds[1].frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[4]);
      this.setSolverInfoRowAngular(row, this.angleY, this.rotLms[1], motorMassY, this.rotSds[1], timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisYX;
      j.ang1Y = this.axisYY;
      j.ang1Z = this.axisYZ;
      j.ang2X = this.axisYX;
      j.ang2Y = this.axisYY;
      j.ang2Z = this.axisYZ;
    }
    if (!this.zSingular && (this.rotSds[2].frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[5]);
      this.setSolverInfoRowAngular(row, this.angleZ, this.rotLms[2], motorMassZ, this.rotSds[2], timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisZX;
      j.ang1Y = this.axisZY;
      j.ang1Z = this.axisZZ;
      j.ang2X = this.axisZX;
      j.ang2Y = this.axisZY;
      j.ang2Z = this.axisZZ;
    }
  }
  /** 同步锚点和坐标系，然后更新约束方向、欧拉角和位移 */
  syncAnchors() {
    super.syncAnchors();
    const aXX = this.bX1X, aXY = this.bX1Y, aXZ = this.bX1Z;
    const aZX = this.bZ2X, aZY = this.bZ2Y, aZZ = this.bZ2Z;
    const aYX = aZY * aXZ - aZZ * aXY, aYY = aZZ * aXX - aZX * aXZ, aYZ = aZX * aXY - aZY * aXX;
    this.axisXX = aYY * aZZ - aYZ * aZY;
    this.axisXY = aYZ * aZX - aYX * aZZ;
    this.axisXZ = aYX * aZY - aYY * aZX;
    this.axisYX = aYX;
    this.axisYY = aYY;
    this.axisYZ = aYZ;
    this.axisZX = aXY * aYZ - aXZ * aYY;
    this.axisZY = aXZ * aYX - aXX * aYZ;
    this.axisZZ = aXX * aYY - aXY * aYX;
    let l = this.axisXX * this.axisXX + this.axisXY * this.axisXY + this.axisXZ * this.axisXZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisXX *= l;
    this.axisXY *= l;
    this.axisXZ *= l;
    l = this.axisYX * this.axisYX + this.axisYY * this.axisYY + this.axisYZ * this.axisYZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisYX *= l;
    this.axisYY *= l;
    this.axisYZ *= l;
    l = this.axisZX * this.axisZX + this.axisZY * this.axisZY + this.axisZZ * this.axisZZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisZX *= l;
    this.axisZY *= l;
    this.axisZZ *= l;
    this.xSingular = this.axisXX * this.axisXX + this.axisXY * this.axisXY + this.axisXZ * this.axisXZ === 0;
    this.ySingular = this.axisYX * this.axisYX + this.axisYY * this.axisYY + this.axisYZ * this.axisYZ === 0;
    this.zSingular = this.axisZX * this.axisZX + this.axisZY * this.axisZY + this.axisZZ * this.axisZZ === 0;
    const m00 = this.bX1X * this.bX2X + this.bX1Y * this.bX2Y + this.bX1Z * this.bX2Z;
    const m01 = this.bX1X * this.bY2X + this.bX1Y * this.bY2Y + this.bX1Z * this.bY2Z;
    const m02 = this.bX1X * this.bZ2X + this.bX1Y * this.bZ2Y + this.bX1Z * this.bZ2Z;
    const m11 = this.bY1X * this.bY2X + this.bY1Y * this.bY2Y + this.bY1Z * this.bY2Z;
    const m12 = this.bY1X * this.bZ2X + this.bY1Y * this.bZ2Y + this.bY1Z * this.bZ2Z;
    const m21 = this.bZ1X * this.bY2X + this.bZ1Y * this.bY2Y + this.bZ1Z * this.bY2Z;
    const m22 = this.bZ1X * this.bZ2X + this.bZ1Y * this.bZ2Y + this.bZ1Z * this.bZ2Z;
    const sy = m02;
    if (sy <= -1) {
      const xSubZ = Math.atan2(m21, m11);
      this.angleX = xSubZ * 0.5;
      this.angleY = -1.570796326794895;
      this.angleZ = -xSubZ * 0.5;
    } else if (sy >= 1) {
      const xAddZ = Math.atan2(m21, m11);
      this.angleX = xAddZ * 0.5;
      this.angleY = 1.570796326794895;
      this.angleZ = xAddZ * 0.5;
    } else {
      this.angleX = Math.atan2(-m12, m22);
      this.angleY = Math.asin(sy);
      this.angleZ = Math.atan2(-m01, m00);
    }
    const dX = this.a2X - this.a1X, dY = this.a2Y - this.a1Y, dZ = this.a2Z - this.a1Z;
    this.translationX = dX * this.bX1X + dY * this.bX1Y + dZ * this.bX1Z;
    this.translationY = dX * this.bY1X + dY * this.bY1Y + dZ * this.bY1Z;
    this.translationZ = dX * this.bZ1X + dY * this.bZ1Y + dZ * this.bZ1Z;
  }
  getVelocitySolverInfo(timeStep, info) {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }
  getPositionSolverInfo(info) {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
};

// src/core/dynamics/constraint/joint/prismatic-joint.ts
var PrismaticJointConfig = class extends JointConfig {
  constructor() {
    super(...arguments);
    /** 滑动方向在两个刚体上的局部方向 */
    this.localAxis1 = new Vec3(1, 0, 0);
    this.localAxis2 = new Vec3(1, 0, 0);
    /** 平移方向的限制和马达 */
    this.limitMotor = new TranslationalLimitMotor();
    /** 平移方向的弹簧阻尼 */
    this.springDamper = new SpringDamper();
  }
  /** 两个刚体在世界坐标的 anchor 处连起来，沿世界坐标的 axis 滑动 */
  init(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ, axisX, axisY, axisZ) {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    toLocalVector(rigidBody1, axisX, axisY, axisZ, this.localAxis1);
    toLocalVector(rigidBody2, axisX, axisY, axisZ, this.localAxis2);
    return this;
  }
};
var PrismaticJoint = class extends Joint {
  constructor(config) {
    super(config, JointType.PRISMATIC);
    /** 沿滑动方向的位移，同步锚点时更新 */
    this.translation = 0;
    /** 两个锚点之间的偏差沿公共坐标系的 y、z 分解 */
    this.linearErrorY = 0;
    this.linearErrorZ = 0;
    /** 两个刚体的相对转动（转动向量，世界坐标） */
    this.angularErrorX = 0;
    this.angularErrorY = 0;
    this.angularErrorZ = 0;
    const a1 = config.localAxis1, a2 = config.localAxis2;
    this.lbX1X = a1.x;
    this.lbX1Y = a1.y;
    this.lbX1Z = a1.z;
    this.lbX2X = a2.x;
    this.lbX2Y = a2.y;
    this.lbX2Z = a2.z;
    this.buildLocalBasesFromX();
    this.basis = new BasisTracker(this);
    this.sd = config.springDamper.clone();
    this.lm = config.limitMotor.clone();
  }
  getInfo(info, timeStep, isPositionPart) {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const angRhsX = this.angularErrorX * erp, angRhsY = this.angularErrorY * erp, angRhsZ = this.angularErrorZ * erp;
    const b = this.basis;
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    let row, j;
    if (this.sd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[0]);
      this.setSolverInfoRowLinear(row, this.translation, this.lm, 1 / (this.b1.invMass + this.b2.invMass), this.sd, timeStep, isPositionPart);
      j = row.jacobian;
      j.lin1X = b.xX;
      j.lin1Y = b.xY;
      j.lin1Z = b.xZ;
      j.lin2X = b.xX;
      j.lin2Y = b.xY;
      j.lin2Z = b.xZ;
      j.ang1X = r1Y * b.xZ - r1Z * b.xY;
      j.ang1Y = r1Z * b.xX - r1X * b.xZ;
      j.ang1Z = r1X * b.xY - r1Y * b.xX;
      j.ang2X = r2Y * b.xZ - r2Z * b.xY;
      j.ang2Y = r2Z * b.xX - r2X * b.xZ;
      j.ang2Z = r2X * b.xY - r2Y * b.xX;
    }
    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, 0);
    j = row.jacobian;
    j.lin1X = b.yX;
    j.lin1Y = b.yY;
    j.lin1Z = b.yZ;
    j.lin2X = b.yX;
    j.lin2Y = b.yY;
    j.lin2Z = b.yZ;
    j.ang1X = r1Y * b.yZ - r1Z * b.yY;
    j.ang1Y = r1Z * b.yX - r1X * b.yZ;
    j.ang1Z = r1X * b.yY - r1Y * b.yX;
    j.ang2X = r2Y * b.yZ - r2Z * b.yY;
    j.ang2Y = r2Z * b.yX - r2X * b.yZ;
    j.ang2Z = r2X * b.yY - r2Y * b.yX;
    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, 0);
    j = row.jacobian;
    j.lin1X = b.zX;
    j.lin1Y = b.zY;
    j.lin1Z = b.zZ;
    j.lin2X = b.zX;
    j.lin2Y = b.zY;
    j.lin2Z = b.zZ;
    j.ang1X = r1Y * b.zZ - r1Z * b.zY;
    j.ang1Y = r1Z * b.zX - r1X * b.zZ;
    j.ang1Z = r1X * b.zY - r1Y * b.zX;
    j.ang2X = r2Y * b.zZ - r2Z * b.zY;
    j.ang2Y = r2Z * b.zX - r2X * b.zZ;
    j.ang2Z = r2X * b.zY - r2Y * b.zX;
    row = info.addRow(this.impulses[3]);
    row.equalLimit(angRhsX, 0);
    j = row.jacobian;
    j.ang1X = 1;
    j.ang1Y = 0;
    j.ang1Z = 0;
    j.ang2X = 1;
    j.ang2Y = 0;
    j.ang2Z = 0;
    row = info.addRow(this.impulses[4]);
    row.equalLimit(angRhsY, 0);
    j = row.jacobian;
    j.ang1X = 0;
    j.ang1Y = 1;
    j.ang1Z = 0;
    j.ang2X = 0;
    j.ang2Y = 1;
    j.ang2Z = 0;
    row = info.addRow(this.impulses[5]);
    row.equalLimit(angRhsZ, 0);
    j = row.jacobian;
    j.ang1X = 0;
    j.ang1Y = 0;
    j.ang1Z = 1;
    j.ang2X = 0;
    j.ang2Y = 0;
    j.ang2Z = 1;
  }
  /** 同步锚点和坐标系，然后更新公共坐标系、相对转动和位移 */
  syncAnchors() {
    super.syncAnchors();
    const b = this.basis;
    b.trackByX();
    const a00 = this.bX1X, a01 = this.bY1X, a02 = this.bZ1X;
    const a10 = this.bX1Y, a11 = this.bY1Y, a12 = this.bZ1Y;
    const a20 = this.bX1Z, a21 = this.bY1Z, a22 = this.bZ1Z;
    const c00 = this.bX2X, c01 = this.bY2X, c02 = this.bZ2X;
    const c10 = this.bX2Y, c11 = this.bY2Y, c12 = this.bZ2Y;
    const c20 = this.bX2Z, c21 = this.bY2Z, c22 = this.bZ2Z;
    const m00 = c00 * a00 + c01 * a01 + c02 * a02;
    const m01 = c00 * a10 + c01 * a11 + c02 * a12;
    const m02 = c00 * a20 + c01 * a21 + c02 * a22;
    const m10 = c10 * a00 + c11 * a01 + c12 * a02;
    const m11 = c10 * a10 + c11 * a11 + c12 * a12;
    const m12 = c10 * a20 + c11 * a21 + c12 * a22;
    const m20 = c20 * a00 + c21 * a01 + c22 * a02;
    const m21 = c20 * a10 + c21 * a11 + c22 * a12;
    const m22 = c20 * a20 + c21 * a21 + c22 * a22;
    let qX, qY, qZ, qW, s;
    const t = m00 + m11 + m22;
    if (t > 0) {
      s = Math.sqrt(t + 1);
      qW = 0.5 * s;
      s = 0.5 / s;
      qX = (m21 - m12) * s;
      qY = (m02 - m20) * s;
      qZ = (m10 - m01) * s;
    } else if (m00 > m11) {
      if (m00 > m22) {
        s = Math.sqrt(m00 - m11 - m22 + 1);
        qX = 0.5 * s;
        s = 0.5 / s;
        qY = (m01 + m10) * s;
        qZ = (m02 + m20) * s;
        qW = (m21 - m12) * s;
      } else {
        s = Math.sqrt(m22 - m00 - m11 + 1);
        qZ = 0.5 * s;
        s = 0.5 / s;
        qX = (m02 + m20) * s;
        qY = (m12 + m21) * s;
        qW = (m10 - m01) * s;
      }
    } else if (m11 > m22) {
      s = Math.sqrt(m11 - m22 - m00 + 1);
      qY = 0.5 * s;
      s = 0.5 / s;
      qX = (m01 + m10) * s;
      qZ = (m12 + m21) * s;
      qW = (m02 - m20) * s;
    } else {
      s = Math.sqrt(m22 - m00 - m11 + 1);
      qZ = 0.5 * s;
      s = 0.5 / s;
      qX = (m02 + m20) * s;
      qY = (m12 + m21) * s;
      qW = (m10 - m01) * s;
    }
    const theta = (qW <= -1 ? 3.14159265358979 : qW >= 1 ? 0 : Math.acos(qW)) * 2;
    let l = qX * qX + qY * qY + qZ * qZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.angularErrorX = qX * l * theta;
    this.angularErrorY = qY * l * theta;
    this.angularErrorZ = qZ * l * theta;
    const dX = this.a2X - this.a1X, dY = this.a2Y - this.a1Y, dZ = this.a2Z - this.a1Z;
    this.translation = dX * b.xX + dY * b.xY + dZ * b.xZ;
    this.linearErrorY = dX * b.yX + dY * b.yY + dZ * b.yZ;
    this.linearErrorZ = dX * b.zX + dY * b.zY + dZ * b.zZ;
  }
  getVelocitySolverInfo(timeStep, info) {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }
  getPositionSolverInfo(info) {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
};

// src/core/dynamics/constraint/joint/ragdoll-joint.ts
var RagdollJointConfig = class extends JointConfig {
  constructor() {
    super(...arguments);
    /** 两个刚体的扭转轴（各自的局部坐标） */
    this.localTwistAxis1 = new Vec3(1, 0, 0);
    this.localTwistAxis2 = new Vec3(1, 0, 0);
    /** 刚体 1 的第一个摆动轴（局部坐标）。第二个摆动轴和它、扭转轴都垂直，创建关节时自动算出 */
    this.localSwingAxis1 = new Vec3(0, 1, 0);
    /** 绕扭转轴的弹簧阻尼、限制和马达 */
    this.twistSpringDamper = new SpringDamper();
    this.twistLimitMotor = new RotationalLimitMotor();
    /** 摆动方向的弹簧阻尼 */
    this.swingSpringDamper = new SpringDamper();
    /** 绕两个摆动轴各自最多转多少（弧度，要大于 0）。π 是截断的 3.14159265358979 */
    this.maxSwingAngle1 = 3.14159265358979;
    this.maxSwingAngle2 = 3.14159265358979;
  }
  /**
   * 两个刚体在世界坐标的 anchor 处连起来，扭转轴是 twistAxis、第一个摆动轴是 swingAxis（世界坐标）。
   */
  init(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ, twistAxisX, twistAxisY, twistAxisZ, swingAxisX, swingAxisY, swingAxisZ) {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    toLocalVector(rigidBody1, twistAxisX, twistAxisY, twistAxisZ, this.localTwistAxis1);
    toLocalVector(rigidBody2, twistAxisX, twistAxisY, twistAxisZ, this.localTwistAxis2);
    toLocalVector(rigidBody1, swingAxisX, swingAxisY, swingAxisZ, this.localSwingAxis1);
    return this;
  }
};
var RagdollJoint = class extends Joint {
  constructor(config) {
    super(config, JointType.RAGDOLL);
    /** 摆出锥外时把它推回去的方向（世界坐标）。在锥里时不更新，保留上一次的值 */
    this.swingAxisX = 0;
    this.swingAxisY = 0;
    this.swingAxisZ = 0;
    /** 扭转约束的方向：两个扭转轴的角平分线（世界坐标） */
    this.twistAxisX = 0;
    this.twistAxisY = 0;
    this.twistAxisZ = 0;
    /** 两个锚点之间的偏差 */
    this.linearErrorX = 0;
    this.linearErrorY = 0;
    this.linearErrorZ = 0;
    /** 摆出锥外多少（弧度），在锥里时是 0 */
    this.swingError = 0;
    /** 摆动那一行用的限制：只能往回推（下限 -1、上限 0，误差总是正的） */
    this.dummySwingLm = new RotationalLimitMotor();
    /** 两个扭转轴的夹角、绕扭转轴转过的角度（弧度） */
    this.swingAngle = 0;
    this.twistAngle = 0;
    const t1 = config.localTwistAxis1, s1 = config.localSwingAxis1, t2 = config.localTwistAxis2;
    this.lbX1X = t1.x;
    this.lbX1Y = t1.y;
    this.lbX1Z = t1.z;
    this.lbY1X = s1.x;
    this.lbY1Y = s1.y;
    this.lbY1Z = s1.z;
    this.lbX2X = t2.x;
    this.lbX2Y = t2.y;
    this.lbX2Z = t2.z;
    this.buildLocalBasesFromXY1X2();
    this.twistSd = config.twistSpringDamper.clone();
    this.twistLm = config.twistLimitMotor.clone();
    this.swingSd = config.swingSpringDamper.clone();
    this.maxSwingAngle1 = config.maxSwingAngle1;
    this.maxSwingAngle2 = config.maxSwingAngle2;
    if (this.maxSwingAngle1 < MIN_RAGDOLL_MAX_SWING_ANGLE) this.maxSwingAngle1 = MIN_RAGDOLL_MAX_SWING_ANGLE;
    if (this.maxSwingAngle2 < MIN_RAGDOLL_MAX_SWING_ANGLE) this.maxSwingAngle2 = MIN_RAGDOLL_MAX_SWING_ANGLE;
    this.dummySwingLm.lowerLimit = -1;
    this.dummySwingLm.upperLimit = 0;
  }
  getInfo(info, timeStep, isPositionPart) {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsX = this.linearErrorX * erp, linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const swingMass = this.computeEffectiveInertiaMoment(this.swingAxisX, this.swingAxisY, this.swingAxisZ);
    const twistMass = this.computeEffectiveInertiaMoment(this.bX2X, this.bX2Y, this.bX2Z);
    let row = info.addRow(this.impulses[0]);
    row.equalLimit(linRhsX, 0);
    let j = row.jacobian;
    j.lin1X = 1;
    j.lin1Y = 0;
    j.lin1Z = 0;
    j.lin2X = 1;
    j.lin2Y = 0;
    j.lin2Z = 0;
    j.ang1X = -0;
    j.ang1Y = r1Z;
    j.ang1Z = -r1Y;
    j.ang2X = -0;
    j.ang2Y = r2Z;
    j.ang2Z = -r2Y;
    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, 0);
    j = row.jacobian;
    j.lin1X = 0;
    j.lin1Y = 1;
    j.lin1Z = 0;
    j.lin2X = 0;
    j.lin2Y = 1;
    j.lin2Z = 0;
    j.ang1X = -r1Z;
    j.ang1Y = -0;
    j.ang1Z = r1X;
    j.ang2X = -r2Z;
    j.ang2Y = -0;
    j.ang2Z = r2X;
    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, 0);
    j = row.jacobian;
    j.lin1X = 0;
    j.lin1Y = 0;
    j.lin1Z = 1;
    j.lin2X = 0;
    j.lin2Y = 0;
    j.lin2Z = 1;
    j.ang1X = r1Y;
    j.ang1Y = -r1X;
    j.ang1Z = -0;
    j.ang2X = r2Y;
    j.ang2Y = -r2X;
    j.ang2Z = -0;
    if (this.swingError > 0 && (this.swingSd.frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.swingError, this.dummySwingLm, swingMass, this.swingSd, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.swingAxisX;
      j.ang1Y = this.swingAxisY;
      j.ang1Z = this.swingAxisZ;
      j.ang2X = this.swingAxisX;
      j.ang2Y = this.swingAxisY;
      j.ang2Z = this.swingAxisZ;
    }
    if (this.twistSd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[4]);
      this.setSolverInfoRowAngular(row, this.twistAngle, this.twistLm, twistMass, this.twistSd, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.twistAxisX;
      j.ang1Y = this.twistAxisY;
      j.ang1Z = this.twistAxisZ;
      j.ang2X = this.twistAxisX;
      j.ang2Y = this.twistAxisY;
      j.ang2Z = this.twistAxisZ;
    }
  }
  /** 同步锚点和坐标系，然后更新摆动角、扭转角、约束方向和偏差 */
  syncAnchors() {
    super.syncAnchors();
    const x1X = this.bX1X, x1Y = this.bX1Y, x1Z = this.bX1Z;
    const x2X = this.bX2X, x2Y = this.bX2Y, x2Z = this.bX2Z;
    let qX, qY, qZ, qW;
    let d = x1X * x2X + x1Y * x2Y + x1Z * x2Z;
    if (d < -0.999999999) {
      const x22 = x1X * x1X, y22 = x1Y * x1Y, z22 = x1Z * x1Z;
      let e;
      if (x22 < y22) {
        if (x22 < z22) {
          e = 1 / Math.sqrt(y22 + z22);
          qX = 0;
          qY = x1Z * e;
          qZ = -x1Y * e;
        } else {
          e = 1 / Math.sqrt(x22 + y22);
          qX = x1Y * e;
          qY = -x1X * e;
          qZ = 0;
        }
      } else if (y22 < z22) {
        e = 1 / Math.sqrt(z22 + x22);
        qX = -x1Z * e;
        qY = 0;
        qZ = x1X * e;
      } else {
        e = 1 / Math.sqrt(x22 + y22);
        qX = x1Y * e;
        qY = -x1X * e;
        qZ = 0;
      }
      qW = 0;
    } else {
      let cX = x1Y * x2Z - x1Z * x2Y, cY = x1Z * x2X - x1X * x2Z, cZ = x1X * x2Y - x1Y * x2X;
      const w = Math.sqrt((1 + d) * 0.5);
      d = 0.5 / w;
      cX *= d;
      cY *= d;
      cZ *= d;
      qX = cX;
      qY = cY;
      qZ = cZ;
      qW = w;
    }
    const x2 = 2 * qX, y2 = 2 * qY, z2 = 2 * qZ;
    const xx = qX * x2, yy = qY * y2, zz = qZ * z2;
    const xy = qX * y2, yz = qY * z2, xz = qX * z2;
    const wx = qW * x2, wy = qW * y2, wz = qW * z2;
    const s00 = 1 - yy - zz, s01 = xy - wz, s02 = xz + wy;
    const s10 = xy + wz, s11 = 1 - xx - zz, s12 = yz - wx;
    const s20 = xz - wy, s21 = yz + wx, s22 = 1 - xx - yy;
    this.swingAngle = (qW <= -1 ? 3.14159265358979 : qW >= 1 ? 0 : Math.acos(qW)) * 2;
    const yInX = s00 * this.bY2X + s10 * this.bY2Y + s20 * this.bY2Z;
    const yInY = s01 * this.bY2X + s11 * this.bY2Y + s21 * this.bY2Z;
    const yInZ = s02 * this.bY2X + s12 * this.bY2Y + s22 * this.bY2Z;
    this.twistAngle = Math.atan2(
      this.bZ1X * yInX + this.bZ1Y * yInY + this.bZ1Z * yInZ,
      this.bY1X * yInX + this.bY1Y * yInY + this.bY1Z * yInZ
    );
    this.twistAxisX = this.bX1X + this.bX2X;
    this.twistAxisY = this.bX1Y + this.bX2Y;
    this.twistAxisZ = this.bX1Z + this.bX2Z;
    let l = this.twistAxisX * this.twistAxisX + this.twistAxisY * this.twistAxisY + this.twistAxisZ * this.twistAxisZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.twistAxisX *= l;
    this.twistAxisY *= l;
    this.twistAxisZ *= l;
    let vX = qX, vY = qY, vZ = qZ;
    let invLen = Math.sqrt(vX * vX + vY * vY + vZ * vZ);
    if (invLen > 0) invLen = 1 / invLen;
    vX *= invLen * this.swingAngle;
    vY *= invLen * this.swingAngle;
    vZ *= invLen * this.swingAngle;
    const x = this.bY1X * vX + this.bY1Y * vY + this.bY1Z * vZ;
    const y = this.bZ1X * vX + this.bZ1Y * vY + this.bZ1Z * vZ;
    const a = this.maxSwingAngle1, b = this.maxSwingAngle2;
    const invA2 = 1 / (a * a), invB2 = 1 / (b * b);
    const w1 = x * x * invA2 + y * y * invB2;
    if (w1 === 0) {
      this.swingAxisX = 0;
      this.swingAxisY = 0;
      this.swingAxisZ = 0;
      this.swingError = 0;
    } else {
      const t = Math.sqrt(1 / w1);
      const x0 = x * t, y0 = y * t;
      let nx = x0 * invA2, ny = y0 * invB2;
      invLen = 1 / Math.sqrt(nx * nx + ny * ny);
      nx *= invLen;
      ny *= invLen;
      const depth = (x - x0) * nx + (y - y0) * ny;
      if (depth > 0) {
        this.swingError = depth;
        const n0 = 0;
        const nX = this.bX1X * n0 + this.bY1X * nx + this.bZ1X * ny;
        const nY = this.bX1Y * n0 + this.bY1Y * nx + this.bZ1Y * ny;
        const nZ = this.bX1Z * n0 + this.bY1Z * nx + this.bZ1Z * ny;
        this.swingAxisX = s00 * nX + s01 * nY + s02 * nZ;
        this.swingAxisY = s10 * nX + s11 * nY + s12 * nZ;
        this.swingAxisZ = s20 * nX + s21 * nY + s22 * nZ;
      } else {
        this.swingError = 0;
      }
    }
    this.linearErrorX = this.a2X - this.a1X;
    this.linearErrorY = this.a2Y - this.a1Y;
    this.linearErrorZ = this.a2Z - this.a1Z;
  }
  getVelocitySolverInfo(timeStep, info) {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }
  getPositionSolverInfo(info) {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
};

// src/core/dynamics/constraint/joint/revolute-joint.ts
var RevoluteJointConfig = class extends JointConfig {
  constructor() {
    super(...arguments);
    /** 转轴在两个刚体上的局部方向 */
    this.localAxis1 = new Vec3(1, 0, 0);
    this.localAxis2 = new Vec3(1, 0, 0);
    /** 转动方向的弹簧阻尼 */
    this.springDamper = new SpringDamper();
    /** 转动方向的限制和马达 */
    this.limitMotor = new RotationalLimitMotor();
  }
  /** 两个刚体在世界坐标的 anchor 处连起来，绕世界坐标的 axis 转动 */
  init(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ, axisX, axisY, axisZ) {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    toLocalVector(rigidBody1, axisX, axisY, axisZ, this.localAxis1);
    toLocalVector(rigidBody2, axisX, axisY, axisZ, this.localAxis2);
    return this;
  }
};
var RevoluteJoint = class extends Joint {
  constructor(config) {
    super(config, JointType.REVOLUTE);
    /** 绕转轴的转角（弧度），同步锚点时更新 */
    this.angle = 0;
    /** 两个刚体的转轴之间的偏差，沿公共坐标系的 y、z 分解 */
    this.angularErrorY = 0;
    this.angularErrorZ = 0;
    /** 两个锚点之间的偏差 */
    this.linearErrorX = 0;
    this.linearErrorY = 0;
    this.linearErrorZ = 0;
    const a1 = config.localAxis1, a2 = config.localAxis2;
    this.lbX1X = a1.x;
    this.lbX1Y = a1.y;
    this.lbX1Z = a1.z;
    this.lbX2X = a2.x;
    this.lbX2Y = a2.y;
    this.lbX2Z = a2.z;
    this.buildLocalBasesFromX();
    this.basis = new BasisTracker(this);
    this.sd = config.springDamper.clone();
    this.lm = config.limitMotor.clone();
  }
  getInfo(info, timeStep, isPositionPart) {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsX = this.linearErrorX * erp, linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const angRhsY = this.angularErrorY * erp, angRhsZ = this.angularErrorZ * erp;
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const b = this.basis;
    const motorMass = this.computeEffectiveInertiaMoment(b.xX, b.xY, b.xZ);
    let row = info.addRow(this.impulses[0]);
    row.equalLimit(linRhsX, 0);
    let j = row.jacobian;
    j.lin1X = 1;
    j.lin1Y = 0;
    j.lin1Z = 0;
    j.lin2X = 1;
    j.lin2Y = 0;
    j.lin2Z = 0;
    j.ang1X = -0;
    j.ang1Y = r1Z;
    j.ang1Z = -r1Y;
    j.ang2X = -0;
    j.ang2Y = r2Z;
    j.ang2Z = -r2Y;
    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, 0);
    j = row.jacobian;
    j.lin1X = 0;
    j.lin1Y = 1;
    j.lin1Z = 0;
    j.lin2X = 0;
    j.lin2Y = 1;
    j.lin2Z = 0;
    j.ang1X = -r1Z;
    j.ang1Y = -0;
    j.ang1Z = r1X;
    j.ang2X = -r2Z;
    j.ang2Y = -0;
    j.ang2Z = r2X;
    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, 0);
    j = row.jacobian;
    j.lin1X = 0;
    j.lin1Y = 0;
    j.lin1Z = 1;
    j.lin2X = 0;
    j.lin2Y = 0;
    j.lin2Z = 1;
    j.ang1X = r1Y;
    j.ang1Y = -r1X;
    j.ang1Z = -0;
    j.ang2X = r2Y;
    j.ang2Y = -r2X;
    j.ang2Z = -0;
    if (this.sd.frequency <= 0 || !isPositionPart) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.angle, this.lm, motorMass, this.sd, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = b.xX;
      j.ang1Y = b.xY;
      j.ang1Z = b.xZ;
      j.ang2X = b.xX;
      j.ang2Y = b.xY;
      j.ang2Z = b.xZ;
    }
    row = info.addRow(this.impulses[4]);
    row.equalLimit(angRhsY, 0);
    j = row.jacobian;
    j.ang1X = b.yX;
    j.ang1Y = b.yY;
    j.ang1Z = b.yZ;
    j.ang2X = b.yX;
    j.ang2Y = b.yY;
    j.ang2Z = b.yZ;
    row = info.addRow(this.impulses[5]);
    row.equalLimit(angRhsZ, 0);
    j = row.jacobian;
    j.ang1X = b.zX;
    j.ang1Y = b.zY;
    j.ang1Z = b.zZ;
    j.ang2X = b.zX;
    j.ang2Y = b.zY;
    j.ang2Z = b.zZ;
  }
  /** 同步锚点和坐标系，然后更新公共坐标系、转角和偏差 */
  syncAnchors() {
    super.syncAnchors();
    const b = this.basis;
    b.trackByX();
    let errX = this.bX1Y * this.bX2Z - this.bX1Z * this.bX2Y;
    let errY = this.bX1Z * this.bX2X - this.bX1X * this.bX2Z;
    let errZ = this.bX1X * this.bX2Y - this.bX1Y * this.bX2X;
    let cos = this.bX1X * this.bX2X + this.bX1Y * this.bX2Y + this.bX1Z * this.bX2Z;
    const theta = cos <= -1 ? 3.14159265358979 : cos >= 1 ? 0 : Math.acos(cos);
    let l = errX * errX + errY * errY + errZ * errZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    errX *= l;
    errY *= l;
    errZ *= l;
    errX *= theta;
    errY *= theta;
    errZ *= theta;
    this.angularErrorY = errX * b.yX + errY * b.yY + errZ * b.yZ;
    this.angularErrorZ = errX * b.zX + errY * b.zY + errZ * b.zZ;
    const pcX = this.bY1Y * this.bY2Z - this.bY1Z * this.bY2Y;
    const pcY = this.bY1Z * this.bY2X - this.bY1X * this.bY2Z;
    const pcZ = this.bY1X * this.bY2Y - this.bY1Y * this.bY2X;
    cos = this.bY1X * this.bY2X + this.bY1Y * this.bY2Y + this.bY1Z * this.bY2Z;
    this.angle = cos <= -1 ? 3.14159265358979 : cos >= 1 ? 0 : Math.acos(cos);
    if (pcX * b.xX + pcY * b.xY + pcZ * b.xZ < 0) this.angle = -this.angle;
    this.linearErrorX = this.a2X - this.a1X;
    this.linearErrorY = this.a2Y - this.a1Y;
    this.linearErrorZ = this.a2Z - this.a1Z;
  }
  getVelocitySolverInfo(timeStep, info) {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }
  getPositionSolverInfo(info) {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
};

// src/core/dynamics/constraint/joint/spherical-joint.ts
var SphericalJointConfig = class extends JointConfig {
  constructor() {
    super(...arguments);
    /** 弹簧阻尼：frequency 大于 0 时锚点之间像弹簧一样连着 */
    this.springDamper = new SpringDamper();
  }
  /** 两个刚体在世界坐标的 anchor 处连起来 */
  init(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ) {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    return this;
  }
};
var SphericalJoint = class extends Joint {
  constructor(config) {
    super(config, JointType.SPHERICAL);
    this.sd = config.springDamper.clone();
  }
  getInfo(info, timeStep, isPositionPart) {
    const sd = this.sd;
    if (sd.frequency > 0 && isPositionPart) return;
    const errorX = this.a2X - this.a1X, errorY = this.a2Y - this.a1Y, errorZ = this.a2Z - this.a1Z;
    let cfm, erp;
    if (sd.frequency > 0) {
      computeSoftConstraintParameters(sd.frequency, sd.dampingRatio, timeStep.dt, sd.useSymplecticEuler);
      cfm = SOFT.cfmFactor;
      erp = SOFT.erp;
      cfm *= this.b1.invMass + this.b2.invMass;
    } else {
      cfm = 0;
      erp = this.getErp(timeStep, isPositionPart);
    }
    const linRhsX = errorX * erp, linRhsY = errorY * erp, linRhsZ = errorZ * erp;
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    let row = info.addRow(this.impulses[0]);
    row.equalLimit(linRhsX, cfm);
    let j = row.jacobian;
    j.lin1X = 1;
    j.lin1Y = 0;
    j.lin1Z = 0;
    j.lin2X = 1;
    j.lin2Y = 0;
    j.lin2Z = 0;
    j.ang1X = -0;
    j.ang1Y = r1Z;
    j.ang1Z = -r1Y;
    j.ang2X = -0;
    j.ang2Y = r2Z;
    j.ang2Z = -r2Y;
    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, cfm);
    j = row.jacobian;
    j.lin1X = 0;
    j.lin1Y = 1;
    j.lin1Z = 0;
    j.lin2X = 0;
    j.lin2Y = 1;
    j.lin2Z = 0;
    j.ang1X = -r1Z;
    j.ang1Y = -0;
    j.ang1Z = r1X;
    j.ang2X = -r2Z;
    j.ang2Y = -0;
    j.ang2Z = r2X;
    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, cfm);
    j = row.jacobian;
    j.lin1X = 0;
    j.lin1Y = 0;
    j.lin1Z = 1;
    j.lin2X = 0;
    j.lin2Y = 0;
    j.lin2Z = 1;
    j.ang1X = r1Y;
    j.ang1Y = -r1X;
    j.ang1Z = -0;
    j.ang2X = r2Y;
    j.ang2Y = -r2X;
    j.ang2Z = -0;
  }
  getVelocitySolverInfo(timeStep, info) {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }
  getPositionSolverInfo(info) {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
};

// src/core/dynamics/constraint/joint/universal-joint.ts
var UniversalJointConfig = class extends JointConfig {
  constructor() {
    super(...arguments);
    /** 刚体 1 的转轴、刚体 2 的转轴（各自的局部坐标），两者要垂直 */
    this.localAxis1 = new Vec3(1, 0, 0);
    this.localAxis2 = new Vec3(1, 0, 0);
    /** 绕两个转轴的弹簧阻尼、限制和马达 */
    this.springDamper1 = new SpringDamper();
    this.springDamper2 = new SpringDamper();
    this.limitMotor1 = new RotationalLimitMotor();
    this.limitMotor2 = new RotationalLimitMotor();
  }
  /** 两个刚体在世界坐标的 anchor 处连起来，刚体 1 绕 axis1、刚体 2 绕 axis2（世界坐标）转动 */
  init(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ, axis1X, axis1Y, axis1Z, axis2X, axis2Y, axis2Z) {
    this.initBase(rigidBody1, rigidBody2, anchorX, anchorY, anchorZ);
    toLocalVector(rigidBody1, axis1X, axis1Y, axis1Z, this.localAxis1);
    toLocalVector(rigidBody2, axis2X, axis2Y, axis2Z, this.localAxis2);
    return this;
  }
};
var UniversalJoint = class extends Joint {
  constructor(config) {
    super(config, JointType.UNIVERSAL);
    /** 约束的三个转动方向（世界坐标），同步锚点时更新 */
    this.axisXX = 0;
    this.axisXY = 0;
    this.axisXZ = 0;
    this.axisYX = 0;
    this.axisYY = 0;
    this.axisYZ = 0;
    this.axisZX = 0;
    this.axisZY = 0;
    this.axisZZ = 0;
    /** 相对转动的 XYZ 欧拉角：X、Z 是绕两个转轴的转角，Y 是偏差 */
    this.angleX = 0;
    this.angleY = 0;
    this.angleZ = 0;
    /** 约束方向退化成零向量时跳过对应的行 */
    this.xSingular = false;
    this.ySingular = false;
    this.zSingular = false;
    /** 两个锚点之间的偏差 */
    this.linearErrorX = 0;
    this.linearErrorY = 0;
    this.linearErrorZ = 0;
    const a1 = config.localAxis1, a2 = config.localAxis2;
    this.lbX1X = a1.x;
    this.lbX1Y = a1.y;
    this.lbX1Z = a1.z;
    this.lbZ2X = a2.x;
    this.lbZ2Y = a2.y;
    this.lbZ2Z = a2.z;
    this.buildLocalBasesFromX1Z2();
    this.sd1 = config.springDamper1.clone();
    this.sd2 = config.springDamper2.clone();
    this.lm1 = config.limitMotor1.clone();
    this.lm2 = config.limitMotor2.clone();
  }
  getInfo(info, timeStep, isPositionPart) {
    const erp = this.getErp(timeStep, isPositionPart);
    const linRhsX = this.linearErrorX * erp, linRhsY = this.linearErrorY * erp, linRhsZ = this.linearErrorZ * erp;
    const angRhsY = this.angleY * erp;
    const r1X = this.ra1X, r1Y = this.ra1Y, r1Z = this.ra1Z;
    const r2X = this.ra2X, r2Y = this.ra2Y, r2Z = this.ra2Z;
    const motorMassX = this.computeEffectiveInertiaMoment(this.axisXX, this.axisXY, this.axisXZ);
    const motorMassZ = this.computeEffectiveInertiaMoment(this.axisZX, this.axisZY, this.axisZZ);
    let row = info.addRow(this.impulses[0]);
    row.equalLimit(linRhsX, 0);
    let j = row.jacobian;
    j.lin1X = 1;
    j.lin1Y = 0;
    j.lin1Z = 0;
    j.lin2X = 1;
    j.lin2Y = 0;
    j.lin2Z = 0;
    j.ang1X = -0;
    j.ang1Y = r1Z;
    j.ang1Z = -r1Y;
    j.ang2X = -0;
    j.ang2Y = r2Z;
    j.ang2Z = -r2Y;
    row = info.addRow(this.impulses[1]);
    row.equalLimit(linRhsY, 0);
    j = row.jacobian;
    j.lin1X = 0;
    j.lin1Y = 1;
    j.lin1Z = 0;
    j.lin2X = 0;
    j.lin2Y = 1;
    j.lin2Z = 0;
    j.ang1X = -r1Z;
    j.ang1Y = -0;
    j.ang1Z = r1X;
    j.ang2X = -r2Z;
    j.ang2Y = -0;
    j.ang2Z = r2X;
    row = info.addRow(this.impulses[2]);
    row.equalLimit(linRhsZ, 0);
    j = row.jacobian;
    j.lin1X = 0;
    j.lin1Y = 0;
    j.lin1Z = 1;
    j.lin2X = 0;
    j.lin2Y = 0;
    j.lin2Z = 1;
    j.ang1X = r1Y;
    j.ang1Y = -r1X;
    j.ang1Z = -0;
    j.ang2X = r2Y;
    j.ang2Y = -r2X;
    j.ang2Z = -0;
    if (!this.xSingular && (this.sd1.frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[3]);
      this.setSolverInfoRowAngular(row, this.angleX, this.lm1, motorMassX, this.sd1, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisXX;
      j.ang1Y = this.axisXY;
      j.ang1Z = this.axisXZ;
      j.ang2X = this.axisXX;
      j.ang2Y = this.axisXY;
      j.ang2Z = this.axisXZ;
    }
    if (!this.ySingular) {
      row = info.addRow(this.impulses[4]);
      row.equalLimit(angRhsY, 0);
      j = row.jacobian;
      j.ang1X = this.axisYX;
      j.ang1Y = this.axisYY;
      j.ang1Z = this.axisYZ;
      j.ang2X = this.axisYX;
      j.ang2Y = this.axisYY;
      j.ang2Z = this.axisYZ;
    }
    if (!this.zSingular && (this.sd2.frequency <= 0 || !isPositionPart)) {
      row = info.addRow(this.impulses[5]);
      this.setSolverInfoRowAngular(row, this.angleZ, this.lm2, motorMassZ, this.sd2, timeStep, isPositionPart);
      j = row.jacobian;
      j.ang1X = this.axisZX;
      j.ang1Y = this.axisZY;
      j.ang1Z = this.axisZZ;
      j.ang2X = this.axisZX;
      j.ang2Y = this.axisZY;
      j.ang2Z = this.axisZZ;
    }
  }
  /** 同步锚点和坐标系，然后更新约束方向、欧拉角和偏差 */
  syncAnchors() {
    super.syncAnchors();
    const aXX = this.bX1X, aXY = this.bX1Y, aXZ = this.bX1Z;
    const aZX = this.bZ2X, aZY = this.bZ2Y, aZZ = this.bZ2Z;
    const aYX = aZY * aXZ - aZZ * aXY, aYY = aZZ * aXX - aZX * aXZ, aYZ = aZX * aXY - aZY * aXX;
    this.axisXX = aYY * aZZ - aYZ * aZY;
    this.axisXY = aYZ * aZX - aYX * aZZ;
    this.axisXZ = aYX * aZY - aYY * aZX;
    this.axisYX = aYX;
    this.axisYY = aYY;
    this.axisYZ = aYZ;
    this.axisZX = aXY * aYZ - aXZ * aYY;
    this.axisZY = aXZ * aYX - aXX * aYZ;
    this.axisZZ = aXX * aYY - aXY * aYX;
    let l = this.axisXX * this.axisXX + this.axisXY * this.axisXY + this.axisXZ * this.axisXZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisXX *= l;
    this.axisXY *= l;
    this.axisXZ *= l;
    l = this.axisYX * this.axisYX + this.axisYY * this.axisYY + this.axisYZ * this.axisYZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisYX *= l;
    this.axisYY *= l;
    this.axisYZ *= l;
    l = this.axisZX * this.axisZX + this.axisZY * this.axisZY + this.axisZZ * this.axisZZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    this.axisZX *= l;
    this.axisZY *= l;
    this.axisZZ *= l;
    this.xSingular = this.axisXX * this.axisXX + this.axisXY * this.axisXY + this.axisXZ * this.axisXZ === 0;
    this.ySingular = this.axisYX * this.axisYX + this.axisYY * this.axisYY + this.axisYZ * this.axisYZ === 0;
    this.zSingular = this.axisZX * this.axisZX + this.axisZY * this.axisZY + this.axisZZ * this.axisZZ === 0;
    const m00 = this.bX1X * this.bX2X + this.bX1Y * this.bX2Y + this.bX1Z * this.bX2Z;
    const m01 = this.bX1X * this.bY2X + this.bX1Y * this.bY2Y + this.bX1Z * this.bY2Z;
    const m02 = this.bX1X * this.bZ2X + this.bX1Y * this.bZ2Y + this.bX1Z * this.bZ2Z;
    const m11 = this.bY1X * this.bY2X + this.bY1Y * this.bY2Y + this.bY1Z * this.bY2Z;
    const m12 = this.bY1X * this.bZ2X + this.bY1Y * this.bZ2Y + this.bY1Z * this.bZ2Z;
    const m21 = this.bZ1X * this.bY2X + this.bZ1Y * this.bY2Y + this.bZ1Z * this.bY2Z;
    const m22 = this.bZ1X * this.bZ2X + this.bZ1Y * this.bZ2Y + this.bZ1Z * this.bZ2Z;
    const sy = m02;
    if (sy <= -1) {
      const xSubZ = Math.atan2(m21, m11);
      this.angleX = xSubZ * 0.5;
      this.angleY = -1.570796326794895;
      this.angleZ = -xSubZ * 0.5;
    } else if (sy >= 1) {
      const xAddZ = Math.atan2(m21, m11);
      this.angleX = xAddZ * 0.5;
      this.angleY = 1.570796326794895;
      this.angleZ = xAddZ * 0.5;
    } else {
      this.angleX = Math.atan2(-m12, m22);
      this.angleY = Math.asin(sy);
      this.angleZ = Math.atan2(-m01, m00);
    }
    this.linearErrorX = this.a2X - this.a1X;
    this.linearErrorY = this.a2Y - this.a1Y;
    this.linearErrorZ = this.a2Z - this.a1Z;
  }
  getVelocitySolverInfo(timeStep, info) {
    super.getVelocitySolverInfo(timeStep, info);
    this.getInfo(info, timeStep, false);
  }
  getPositionSolverInfo(info) {
    super.getPositionSolverInfo(info);
    this.getInfo(info, null, true);
  }
};

// src/core/dynamics/rigidbody/shape-config.ts
var ShapeConfig = class {
  constructor(geometry) {
    /** 相对刚体重心的局部位置和旋转 */
    this.transform = new Transform();
    this.friction = DEFAULT_FRICTION;
    this.restitution = DEFAULT_RESTITUTION;
    /** 密度（kg/m³） */
    this.density = DEFAULT_DENSITY;
    /** 两个形状只有在 (group1 & mask2) 和 (group2 & mask1) 都不为 0 时才会碰撞 */
    this.collisionGroup = DEFAULT_COLLISION_GROUP;
    this.collisionMask = DEFAULT_COLLISION_MASK;
    this.geometry = geometry;
  }
};

// src/core/collision/geometry/aabb.ts
var Aabb = class {
  constructor() {
    this.minX = 0;
    this.minY = 0;
    this.minZ = 0;
    this.maxX = 0;
    this.maxY = 0;
    this.maxZ = 0;
  }
  copyFrom(aabb) {
    this.minX = aabb.minX;
    this.minY = aabb.minY;
    this.minZ = aabb.minZ;
    this.maxX = aabb.maxX;
    this.maxY = aabb.maxY;
    this.maxZ = aabb.maxZ;
  }
};

// src/core/dynamics/rigidbody/shape.ts
var Shape = class {
  constructor(config) {
    /** 世界分配的编号，不在世界里时为 -1 */
    this.id = -1;
    this.prev = null;
    this.next = null;
    this.rigidBody = null;
    /** 相对刚体的局部变换 */
    this.localTransform = new Transform();
    /** 上一步、这一步的世界变换 */
    this.ptransform = new Transform();
    this.transform = new Transform();
    /** 上一步到这一步扫过的包围盒 */
    this.aabb = new Aabb();
    /** 宽相代理，形状加入世界后才有 */
    this.proxy = null;
    /**
     * 预测接触距离，单位为米。包围盒按此扩展，窄相使用两侧距离之和。
     * 未启用子步时为 0。
     */
    this.speculativeMargin = 0;
    /** 更新流形前的接触过滤器，供适配层剔除内部接触面。默认不过滤。 */
    this.contactFilter = null;
    /** 使用者的数据，内核不读 */
    this.userData = null;
    this.localTransform.copyFrom(config.transform);
    this.ptransform.copyFrom(this.localTransform);
    this.transform.copyFrom(this.localTransform);
    this.restitution = config.restitution;
    this.friction = config.friction;
    this.density = config.density;
    this.geom = config.geometry;
    this.collisionGroup = config.collisionGroup;
    this.collisionMask = config.collisionMask;
  }
  /** 按刚体上一步、这一步的变换更新形状的世界变换和扫过的包围盒，并通知宽相 */
  sync(tf1, tf2) {
    this.speculativeMargin = 0;
    transformMul(this.ptransform, this.localTransform, tf1);
    transformMul(this.transform, this.localTransform, tf2);
    const aabb = this.aabb;
    this.geom.computeAabb(aabb, this.ptransform);
    const minX = aabb.minX, minY = aabb.minY, minZ = aabb.minZ;
    const maxX = aabb.maxX, maxY = aabb.maxY, maxZ = aabb.maxZ;
    this.geom.computeAabb(aabb, this.transform);
    aabb.minX = minX < aabb.minX ? minX : aabb.minX;
    aabb.minY = minY < aabb.minY ? minY : aabb.minY;
    aabb.minZ = minZ < aabb.minZ ? minZ : aabb.minZ;
    aabb.maxX = maxX > aabb.maxX ? maxX : aabb.maxX;
    aabb.maxY = maxY > aabb.maxY ? maxY : aabb.maxY;
    aabb.maxZ = maxZ > aabb.maxZ ? maxZ : aabb.maxZ;
    if (this.proxy !== null) {
      const tf = this.transform, ptf = this.ptransform;
      this.rigidBody.world.broadPhase.moveProxy(this.proxy, aabb, tf.px - ptf.px, tf.py - ptf.py, tf.pz - ptf.pz);
    }
  }
  /**
   * 同步形状并扩展预测接触包围盒，每 tick 调用一次。
   * 扩展距离为 min(max, base + linearReach + angularSpeed × 半径 × t)。
   * linearReach 为线速度 × t，t 为 tick 时长。
   */
  syncSpeculative(tf1, tf2, base, max, linearReach, angularSpeed, t) {
    transformMul(this.ptransform, this.localTransform, tf1);
    transformMul(this.transform, this.localTransform, tf2);
    const aabb = this.aabb;
    this.geom.computeAabb(aabb, this.transform);
    const tf = this.rigidBody.transform;
    const hX = (aabb.maxX - aabb.minX) * 0.5, hY = (aabb.maxY - aabb.minY) * 0.5, hZ = (aabb.maxZ - aabb.minZ) * 0.5;
    const cX = aabb.minX + hX - tf.px, cY = aabb.minY + hY - tf.py, cZ = aabb.minZ + hZ - tf.pz;
    const radius = Math.sqrt(cX * cX + cY * cY + cZ * cZ) + Math.sqrt(hX * hX + hY * hY + hZ * hZ);
    let margin = base + linearReach + angularSpeed * radius * t;
    if (margin > max) margin = max;
    this.speculativeMargin = margin;
    let minX = aabb.minX, minY = aabb.minY, minZ = aabb.minZ, maxX = aabb.maxX, maxY = aabb.maxY, maxZ = aabb.maxZ;
    this.geom.computeAabb(aabb, this.ptransform);
    if (aabb.minX < minX) minX = aabb.minX;
    if (aabb.minY < minY) minY = aabb.minY;
    if (aabb.minZ < minZ) minZ = aabb.minZ;
    if (aabb.maxX > maxX) maxX = aabb.maxX;
    if (aabb.maxY > maxY) maxY = aabb.maxY;
    if (aabb.maxZ > maxZ) maxZ = aabb.maxZ;
    aabb.minX = minX - margin;
    aabb.minY = minY - margin;
    aabb.minZ = minZ - margin;
    aabb.maxX = maxX + margin;
    aabb.maxY = maxY + margin;
    aabb.maxZ = maxZ + margin;
    if (this.proxy !== null) {
      const ptf = this.ptransform;
      this.rigidBody.world.broadPhase.moveProxy(this.proxy, aabb, this.transform.px - ptf.px, this.transform.py - ptf.py, this.transform.pz - ptf.pz);
    }
  }
};

// src/mc/convert.ts
var DEG = Math.PI / 180;
function quaternionToMatrix(q, out = new Array(9)) {
  const len = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!(len > 0)) throw new Error("\u56DB\u5143\u6570\u7684\u6240\u6709\u5206\u91CF\u5747\u4E3A\u96F6");
  const x = q.x / len, y = q.y / len, z = q.z / len, w = q.w / len;
  const x2 = 2 * x, y2 = 2 * y, z2 = 2 * z;
  const xx = x * x2, yy = y * y2, zz = z * z2, xy = x * y2, yz = y * z2, xz = x * z2, wx = w * x2, wy = w * y2, wz = w * z2;
  out[0] = 1 - yy - zz;
  out[1] = xy - wz;
  out[2] = xz + wy;
  out[3] = xy + wz;
  out[4] = 1 - xx - zz;
  out[5] = yz - wx;
  out[6] = xz - wy;
  out[7] = yz + wx;
  out[8] = 1 - xx - yy;
  return out;
}
function matrixToQuaternion(r00, r01, r02, r10, r11, r12, r20, r21, r22) {
  const t = r00 + r11 + r22;
  let x, y, z, w, s;
  if (t > 0) {
    s = Math.sqrt(t + 1);
    w = 0.5 * s;
    s = 0.5 / s;
    x = (r21 - r12) * s;
    y = (r02 - r20) * s;
    z = (r10 - r01) * s;
  } else if (r00 > r11) {
    if (r00 > r22) {
      s = Math.sqrt(r00 - r11 - r22 + 1);
      x = 0.5 * s;
      s = 0.5 / s;
      y = (r01 + r10) * s;
      z = (r02 + r20) * s;
      w = (r21 - r12) * s;
    } else {
      s = Math.sqrt(r22 - r00 - r11 + 1);
      z = 0.5 * s;
      s = 0.5 / s;
      x = (r02 + r20) * s;
      y = (r12 + r21) * s;
      w = (r10 - r01) * s;
    }
  } else if (r11 > r22) {
    s = Math.sqrt(r11 - r22 - r00 + 1);
    y = 0.5 * s;
    s = 0.5 / s;
    x = (r01 + r10) * s;
    z = (r12 + r21) * s;
    w = (r02 - r20) * s;
  } else {
    s = Math.sqrt(r22 - r00 - r11 + 1);
    z = 0.5 * s;
    s = 0.5 / s;
    x = (r02 + r20) * s;
    y = (r12 + r21) * s;
    w = (r10 - r01) * s;
  }
  const l = Math.sqrt(x * x + y * y + z * z + w * w);
  return { x: x / l + 0, y: y / l + 0, z: z / l + 0, w: w / l + 0 };
}
function positive(name, v) {
  if (!(v > 0) || !Number.isFinite(v)) throw new Error(`${name} \u5FC5\u987B\u662F\u5927\u4E8E\u96F6\u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${v}`);
  return v;
}
function finite(name, v) {
  if (!Number.isFinite(v.x) || !Number.isFinite(v.y) || !Number.isFinite(v.z)) {
    throw new Error(`${name} \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 (${v.x}, ${v.y}, ${v.z})`);
  }
}
function makeGeometry(o) {
  switch (o.type) {
    case "box": {
      finite("box \u7684 size", o.size);
      positive("box \u7684 size.x", o.size.x);
      positive("box \u7684 size.y", o.size.y);
      positive("box \u7684 size.z", o.size.z);
      return new BoxGeometry(o.size.x / 2, o.size.y / 2, o.size.z / 2);
    }
    case "sphere":
      return new SphereGeometry(positive("sphere \u7684 radius", o.radius));
    case "capsule": {
      positive("capsule \u7684 radius", o.radius);
      if (!(o.height >= o.radius * 2)) throw new Error(`capsule \u7684 height \u4E0D\u5F97\u5C0F\u4E8E radius * 2\uFF0C\u6536\u5230 height = ${o.height}\uFF0Cradius = ${o.radius}`);
      return new CapsuleGeometry(o.radius, o.height / 2 - o.radius);
    }
    case "cylinder":
      return new CylinderGeometry(positive("cylinder \u7684 radius", o.radius), positive("cylinder \u7684 height", o.height) / 2);
    case "cone":
      return new ConeGeometry(positive("cone \u7684 radius", o.radius), positive("cone \u7684 height", o.height) / 2);
    case "convexHull": {
      if (o.points.length < 4) throw new Error(`convexHull \u81F3\u5C11\u9700\u8981 4 \u4E2A\u70B9\uFF0C\u6536\u5230 ${o.points.length} \u4E2A`);
      const vertices = [];
      for (const p of o.points) {
        finite("convexHull \u7684\u70B9", p);
        vertices.push(new Vec3(p.x, p.y, p.z));
      }
      return new ConvexHullGeometry(vertices);
    }
    default:
      throw new Error(`\u672A\u77E5\u7684\u5F62\u72B6\u7C7B\u578B\uFF1A${o.type}`);
  }
}
var SHAPE_DEFAULTS = { friction: 0.5, restitution: 0, density: 1, collisionGroup: 1, collisionMask: 1 };
function makeShape(o) {
  const sc = new ShapeConfig(makeGeometry(o));
  const offset = o.offset;
  if (offset !== void 0) {
    finite("\u5F62\u72B6\u7684 offset", offset);
    sc.transform.px = offset.x;
    sc.transform.py = offset.y;
    sc.transform.pz = offset.z;
  }
  if (o.rotation !== void 0) {
    const m = quaternionToMatrix(o.rotation);
    const t = sc.transform;
    t.r00 = m[0];
    t.r01 = m[1];
    t.r02 = m[2];
    t.r10 = m[3];
    t.r11 = m[4];
    t.r12 = m[5];
    t.r20 = m[6];
    t.r21 = m[7];
    t.r22 = m[8];
  }
  sc.friction = o.friction ?? SHAPE_DEFAULTS.friction;
  sc.restitution = o.restitution ?? SHAPE_DEFAULTS.restitution;
  sc.density = positive("density", o.density ?? SHAPE_DEFAULTS.density);
  sc.collisionGroup = o.collisionGroup ?? SHAPE_DEFAULTS.collisionGroup;
  sc.collisionMask = o.collisionMask ?? SHAPE_DEFAULTS.collisionMask;
  return new Shape(sc);
}
function isConvex(g) {
  return g instanceof ConvexGeometry;
}
function jointNumber(name, v, bound) {
  if (Number.isNaN(v)) throw new Error(`${name} \u4E0D\u80FD\u662F NaN`);
  return v === Infinity ? bound : v === -Infinity ? -bound : v;
}
function breakLimit(name, v) {
  if (v === void 0) return 0;
  if (Number.isNaN(v)) throw new Error(`${name} \u4E0D\u80FD\u662F NaN`);
  return v > 0 && v !== Infinity ? v : 0;
}
var UNLIMITED_MOTOR = 1e12;
function applySpring(sd, s) {
  if (s === void 0) return;
  if (!Number.isFinite(s.frequency)) throw new Error(`spring \u7684 frequency \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 ${s.frequency}`);
  const damping = s.dampingRatio ?? 1;
  if (!Number.isFinite(damping)) throw new Error(`spring \u7684 dampingRatio \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 ${damping}`);
  sd.setSpring(s.frequency, damping);
  sd.setSymplecticEuler(s.useSymplecticEuler ?? false);
}
function applyAngular(lm, sd, o) {
  if (o === void 0) return;
  if (o.lower !== void 0 || o.upper !== void 0) {
    lm.setLimits(jointNumber("limit \u7684 lower", o.lower ?? -180, 180) * DEG, jointNumber("limit \u7684 upper", o.upper ?? 180, 180) * DEG);
  }
  if (o.motorSpeed !== void 0 || o.motorMaxTorque !== void 0) {
    if (!Number.isFinite(o.motorSpeed ?? 0)) throw new Error(`motorSpeed \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 ${o.motorSpeed}`);
    lm.setMotor((o.motorSpeed ?? 0) * DEG, jointNumber("motorMaxTorque", o.motorMaxTorque ?? 0, UNLIMITED_MOTOR));
  }
  applySpring(sd, o.spring);
}
function applyLinear(lm, sd, o) {
  if (o === void 0) return;
  if (o.lower !== void 0 || o.upper !== void 0) {
    lm.setLimits(jointNumber("limit \u7684 lower", o.lower ?? -1e9, 1e9), jointNumber("limit \u7684 upper", o.upper ?? 1e9, 1e9));
  }
  if (o.motorSpeed !== void 0 || o.motorMaxForce !== void 0) {
    if (!Number.isFinite(o.motorSpeed ?? 0)) throw new Error(`motorSpeed \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 ${o.motorSpeed}`);
    lm.setMotor(o.motorSpeed ?? 0, jointNumber("motorMaxForce", o.motorMaxForce ?? 0, UNLIMITED_MOTOR));
  }
  applySpring(sd, o.spring);
}
function axisOf(name, v) {
  finite(name, v);
  if (v.x === 0 && v.y === 0 && v.z === 0) throw new Error(`${name} \u4E0D\u80FD\u662F\u96F6\u5411\u91CF`);
  return v;
}
function makeJointConfig(o, a, b) {
  finite("\u5173\u8282\u7684 anchor", o.anchor);
  const { x, y, z } = o.anchor;
  let config;
  switch (o.type) {
    case "ball": {
      const c = new SphericalJointConfig().init(a, b, x, y, z);
      applySpring(c.springDamper, o.spring);
      config = c;
      break;
    }
    case "hinge": {
      const ax = axisOf("hinge \u7684 axis", o.axis);
      const c = new RevoluteJointConfig().init(a, b, x, y, z, ax.x, ax.y, ax.z);
      applyAngular(c.limitMotor, c.springDamper, o.limit);
      config = c;
      break;
    }
    case "slider": {
      const ax = axisOf("slider \u7684 axis", o.axis);
      const c = new PrismaticJointConfig().init(a, b, x, y, z, ax.x, ax.y, ax.z);
      applyLinear(c.limitMotor, c.springDamper, o.limit);
      config = c;
      break;
    }
    case "cylinder": {
      const ax = axisOf("cylinder \u7684 axis", o.axis);
      const c = new CylindricalJointConfig().init(a, b, x, y, z, ax.x, ax.y, ax.z);
      applyLinear(c.translationalLimitMotor, c.translationalSpringDamper, o.linearLimit);
      applyAngular(c.rotationalLimitMotor, c.rotationalSpringDamper, o.angularLimit);
      config = c;
      break;
    }
    case "universal": {
      const a1 = axisOf("universal \u7684 axisA", o.axisA), a2 = axisOf("universal \u7684 axisB", o.axisB);
      const c = new UniversalJointConfig().init(a, b, x, y, z, a1.x, a1.y, a1.z, a2.x, a2.y, a2.z);
      applyAngular(c.limitMotor1, c.springDamper1, o.limitA);
      applyAngular(c.limitMotor2, c.springDamper2, o.limitB);
      config = c;
      break;
    }
    case "ragdoll": {
      const t = axisOf("ragdoll \u7684 twistAxis", o.twistAxis), s = axisOf("ragdoll \u7684 swingAxis", o.swingAxis);
      const c = new RagdollJointConfig().init(a, b, x, y, z, t.x, t.y, t.z, s.x, s.y, s.z);
      if (o.maxSwingAngleX !== void 0) c.maxSwingAngle1 = jointNumber("maxSwingAngleX", o.maxSwingAngleX, 180) * DEG;
      if (o.maxSwingAngleZ !== void 0) c.maxSwingAngle2 = jointNumber("maxSwingAngleZ", o.maxSwingAngleZ, 180) * DEG;
      applyAngular(c.twistLimitMotor, c.twistSpringDamper, o.twistLimit);
      applySpring(c.swingSpringDamper, o.swingSpring);
      config = c;
      break;
    }
    case "generic": {
      const c = new GenericJointConfig().init(a, b, x, y, z, new Mat3(), new Mat3());
      if (o.linearLimits !== void 0) {
        if (o.linearLimits.length !== 3) throw new Error("generic \u7684 linearLimits \u957F\u5EA6\u5FC5\u987B\u4E3A 3");
        for (let i = 0; i < 3; i++) applyLinear(c.translationalLimitMotors[i], c.translationalSpringDampers[i], o.linearLimits[i]);
      }
      if (o.angularLimits !== void 0) {
        if (o.angularLimits.length !== 3) throw new Error("generic \u7684 angularLimits \u957F\u5EA6\u5FC5\u987B\u4E3A 3");
        for (let i = 0; i < 3; i++) applyAngular(c.rotationalLimitMotors[i], c.rotationalSpringDampers[i], o.angularLimits[i]);
      }
      config = c;
      break;
    }
    default:
      throw new Error(`\u672A\u77E5\u7684\u5173\u8282\u7C7B\u578B\uFF1A${o.type}`);
  }
  config.allowCollision = o.allowCollision ?? false;
  config.breakForce = breakLimit("breakForce", o.breakForce);
  config.breakTorque = breakLimit("breakTorque", o.breakTorque);
  config.solverType = o.solver === "direct" ? ConstraintSolverType.DIRECT : ConstraintSolverType.ITERATIVE;
  return config;
}
function makeJoint(o, a, b) {
  const config = makeJointConfig(o, a, b);
  if (config instanceof SphericalJointConfig) return new SphericalJoint(config);
  if (config instanceof RevoluteJointConfig) return new RevoluteJoint(config);
  if (config instanceof PrismaticJointConfig) return new PrismaticJoint(config);
  if (config instanceof CylindricalJointConfig) return new CylindricalJoint(config);
  if (config instanceof UniversalJointConfig) return new UniversalJoint(config);
  if (config instanceof RagdollJointConfig) return new RagdollJoint(config);
  if (config instanceof GenericJointConfig) return new GenericJoint(config);
  throw new Error("\u672A\u77E5\u7684\u5173\u8282\u914D\u7F6E");
}

// src/core/dynamics/constraint/drag-constraint.ts
var DragConstraint = class {
  constructor(body) {
    this.body = body;
    this.lax = 0;
    this.lay = 0;
    this.laz = 0;
    this.x = 0;
    this.y = 0;
    this.z = 0;
    this.px = 0;
    this.py = 0;
    this.pz = 0;
    this.qx = 0;
    this.qy = 0;
    this.qz = 0;
    this.qw = 1;
    this.hertz = 4;
    this.dampingRatio = 1;
    this.maxAcceleration = 60;
    this.holdRotation = true;
    this.angularHertz = 3;
    this.angularDampingRatio = 1;
    this.maxAngularAcceleration = 40;
    this.ix = 0;
    this.iy = 0;
    this.iz = 0;
    this.ax = 0;
    this.ay = 0;
    this.az = 0;
  }
  finish() {
    this.px = this.x;
    this.py = this.y;
    this.pz = this.z;
  }
  warmStart() {
    const b = this.body, r = b.transform, m = b.invMass;
    if (m === 0) return;
    const x = r.r00 * this.lax + r.r01 * this.lay + r.r02 * this.laz;
    const y = r.r10 * this.lax + r.r11 * this.lay + r.r12 * this.laz;
    const z = r.r20 * this.lax + r.r21 * this.lay + r.r22 * this.laz;
    b.vx += m * this.ix;
    b.vy += m * this.iy;
    b.vz += m * this.iz;
    const tx = y * this.iz - z * this.iy + this.ax;
    const ty = z * this.ix - x * this.iz + this.ay;
    const tz = x * this.iy - y * this.ix + this.az;
    b.wx += b.ii00 * tx + b.ii01 * ty + b.ii02 * tz;
    b.wy += b.ii10 * tx + b.ii11 * ty + b.ii12 * tz;
    b.wz += b.ii20 * tx + b.ii21 * ty + b.ii22 * tz;
  }
  solve(h, fraction) {
    const b = this.body, r = b.transform, m = b.invMass;
    if (m === 0) return;
    const a = b.ii00, c = b.ii01, d = b.ii02, e = b.ii11, f = b.ii12, g = b.ii22;
    const x = r.r00 * this.lax + r.r01 * this.lay + r.r02 * this.laz;
    const y = r.r10 * this.lax + r.r11 * this.lay + r.r12 * this.laz;
    const z = r.r20 * this.lax + r.r21 * this.lay + r.r22 * this.laz;
    const ex = r.px + x - (this.px + (this.x - this.px) * fraction);
    const ey = r.py + y - (this.py + (this.y - this.py) * fraction);
    const ez = r.pz + z - (this.pz + (this.z - this.pz) * fraction);
    let omega = 2 * Math.PI * this.hertz, damp = 2 * this.dampingRatio + h * omega;
    let scale = h * omega * damp, soft = 1 / (1 + scale), bias = omega / damp;
    scale *= soft;
    const k00 = m + (-(-z * e + y * f) * z + (-z * f + y * g) * y);
    const k01 = (-z * c + y * d) * z - (-z * f + y * g) * x;
    const k02 = -(-z * c + y * d) * y + (-z * e + y * f) * x;
    const k11 = m + ((z * a - x * d) * z - (z * d - x * g) * x);
    const k12 = -(z * a - x * d) * y + (z * c - x * f) * x;
    const k22 = m + (-(-y * a + x * c) * y + (-y * c + x * e) * x);
    const co00 = k11 * k22 - k12 * k12, co01 = k02 * k12 - k01 * k22, co02 = k01 * k12 - k02 * k11;
    let det2 = k00 * co00 + k01 * co01 + k02 * co02;
    if (Math.abs(det2) > 1e-12) {
      det2 = 1 / det2;
      const vx = b.vx + b.wy * z - b.wz * y + bias * ex;
      const vy = b.vy + b.wz * x - b.wx * z + bias * ey;
      const vz = b.vz + b.wx * y - b.wy * x + bias * ez;
      const oldX2 = this.ix, oldY2 = this.iy, oldZ2 = this.iz;
      let jx2 = oldX2 - scale * (co00 * vx + co01 * vy + co02 * vz) * det2 - soft * oldX2;
      let jy2 = oldY2 - scale * (co01 * vx + (k00 * k22 - k02 * k02) * vy + (k01 * k02 - k00 * k12) * vz) * det2 - soft * oldY2;
      let jz2 = oldZ2 - scale * (co02 * vx + (k01 * k02 - k00 * k12) * vy + (k00 * k11 - k01 * k01) * vz) * det2 - soft * oldZ2;
      const max2 = h * this.maxAcceleration / m, length22 = jx2 * jx2 + jy2 * jy2 + jz2 * jz2;
      if (length22 > max2 * max2) {
        const s = max2 / Math.sqrt(length22);
        jx2 *= s;
        jy2 *= s;
        jz2 *= s;
      }
      this.ix = jx2;
      this.iy = jy2;
      this.iz = jz2;
      jx2 -= oldX2;
      jy2 -= oldY2;
      jz2 -= oldZ2;
      b.vx += m * jx2;
      b.vy += m * jy2;
      b.vz += m * jz2;
      const tx = y * jz2 - z * jy2, ty = z * jx2 - x * jz2, tz = x * jy2 - y * jx2;
      b.wx += a * tx + c * ty + d * tz;
      b.wy += c * tx + e * ty + f * tz;
      b.wz += d * tx + f * ty + g * tz;
    }
    if (!this.holdRotation) return;
    let qx, qy, qz, qw;
    const trace = r.r00 + r.r11 + r.r22;
    if (trace > 0) {
      const s = Math.sqrt(trace + 1), inv2 = 0.5 / s;
      qw = 0.5 * s;
      qx = (r.r21 - r.r12) * inv2;
      qy = (r.r02 - r.r20) * inv2;
      qz = (r.r10 - r.r01) * inv2;
    } else if (r.r00 > r.r11 && r.r00 > r.r22) {
      const s = Math.sqrt(r.r00 - r.r11 - r.r22 + 1), inv2 = 0.5 / s;
      qx = 0.5 * s;
      qy = (r.r01 + r.r10) * inv2;
      qz = (r.r02 + r.r20) * inv2;
      qw = (r.r21 - r.r12) * inv2;
    } else if (r.r11 > r.r22) {
      const s = Math.sqrt(r.r11 - r.r22 - r.r00 + 1), inv2 = 0.5 / s;
      qy = 0.5 * s;
      qx = (r.r01 + r.r10) * inv2;
      qz = (r.r12 + r.r21) * inv2;
      qw = (r.r02 - r.r20) * inv2;
    } else {
      const s = Math.sqrt(r.r22 - r.r00 - r.r11 + 1), inv2 = 0.5 / s;
      qz = 0.5 * s;
      qx = (r.r02 + r.r20) * inv2;
      qy = (r.r12 + r.r21) * inv2;
      qw = (r.r10 - r.r01) * inv2;
    }
    let rx = -this.qw * qx + this.qx * qw - this.qy * qz + this.qz * qy;
    let ry = -this.qw * qy + this.qx * qz + this.qy * qw - this.qz * qx;
    let rz = -this.qw * qz - this.qx * qy + this.qy * qx + this.qz * qw;
    let rw = this.qw * qw + this.qx * qx + this.qy * qy + this.qz * qz;
    if (rw < 0) {
      rx = -rx;
      ry = -ry;
      rz = -rz;
      rw = -rw;
    }
    const length = Math.sqrt(rx * rx + ry * ry + rz * rz);
    const angle = length > 1e-9 ? 2 * Math.atan2(length, rw) / length : 0;
    omega = 2 * Math.PI * this.angularHertz;
    damp = 2 * this.angularDampingRatio + h * omega;
    scale = h * omega * damp;
    soft = 1 / (1 + scale);
    bias = omega / damp;
    scale *= soft;
    const c00 = e * g - f * f, c01 = d * f - c * g, c02 = c * f - d * e;
    det2 = a * c00 + c * c01 + d * c02;
    if (Math.abs(det2) < 1e-18) return;
    det2 = 1 / det2;
    const i00 = c00 * det2, i01 = c01 * det2, i02 = c02 * det2;
    const i11 = (a * g - d * d) * det2, i12 = (c * d - a * f) * det2, i22 = (a * e - c * c) * det2;
    const wx = b.wx - bias * rx * angle, wy = b.wy - bias * ry * angle, wz = b.wz - bias * rz * angle;
    const oldX = this.ax, oldY = this.ay, oldZ = this.az;
    let jx = oldX - scale * (i00 * wx + i01 * wy + i02 * wz) - soft * oldX;
    let jy = oldY - scale * (i01 * wx + i11 * wy + i12 * wz) - soft * oldY;
    let jz = oldZ - scale * (i02 * wx + i12 * wy + i22 * wz) - soft * oldZ;
    const max = h * this.maxAngularAcceleration * Math.max(i00, i11, i22), length2 = jx * jx + jy * jy + jz * jz;
    if (length2 > max * max) {
      const s = max / Math.sqrt(length2);
      jx *= s;
      jy *= s;
      jz *= s;
    }
    this.ax = jx;
    this.ay = jy;
    this.az = jz;
    jx -= oldX;
    jy -= oldY;
    jz -= oldZ;
    b.wx += a * jx + c * jy + d * jz;
    b.wy += c * jx + e * jy + f * jz;
    b.wz += d * jx + f * jy + g * jz;
  }
};

// src/mc/drag.ts
function finite2(point2) {
  if (!Number.isFinite(point2.x) || !Number.isFinite(point2.y) || !Number.isFinite(point2.z)) throw new Error("\u6293\u53D6\u5750\u6807\u5FC5\u987B\u662F\u6709\u9650\u6570");
}
var Drag = class {
  /** @internal */
  constructor(body, options) {
    this.body = body;
    if (body.type !== "dynamic") throw new Error("\u53EA\u80FD\u6293\u53D6\u52A8\u6001\u521A\u4F53");
    finite2(options.anchor);
    const d = new DragConstraint(body.core), local = body.worldToLocal(options.anchor);
    this.constraint = d;
    d.lax = local.x;
    d.lay = local.y;
    d.laz = local.z;
    d.x = d.px = options.anchor.x;
    d.y = d.py = options.anchor.y;
    d.z = d.pz = options.anchor.z;
    const q = body.rotation;
    d.qx = q.x;
    d.qy = q.y;
    d.qz = q.z;
    d.qw = q.w;
    for (const [key2, value] of [
      ["hertz", options.frequency],
      ["dampingRatio", options.dampingRatio],
      ["maxAcceleration", options.maxAcceleration],
      ["angularHertz", options.angularFrequency],
      ["angularDampingRatio", options.angularDampingRatio],
      ["maxAngularAcceleration", options.maxAngularAcceleration]
    ]) {
      if (value === void 0) continue;
      if (!Number.isFinite(value) || value <= 0) throw new Error(`${key2} \u5FC5\u987B\u662F\u5927\u4E8E\u96F6\u7684\u6709\u9650\u6570`);
      d[key2] = value;
    }
    if (options.holdRotation !== void 0) d.holdRotation = options.holdRotation;
    body.core.drag = d;
    body.wakeUp();
  }
  /** 刚体移除、类型改变、约束替换或释放后为 false。 */
  get isValid() {
    return this.body.isValid && this.body.core.drag === this.constraint;
  }
  /** 更新下一次步进的目标位置和姿态。 */
  setTarget(location, rotation) {
    if (!this.isValid) throw new Error("\u6293\u53D6\u7EA6\u675F\u5DF2\u7ECF\u5931\u6548");
    finite2(location);
    const d = this.constraint;
    if (rotation !== void 0) {
      finite2(rotation);
      const length = Math.hypot(rotation.x, rotation.y, rotation.z, rotation.w);
      if (!(length > 0) || !Number.isFinite(length)) throw new Error("\u6293\u53D6\u59FF\u6001\u5FC5\u987B\u662F\u975E\u96F6\u6709\u9650\u56DB\u5143\u6570");
      d.qx = rotation.x / length;
      d.qy = rotation.y / length;
      d.qz = rotation.z / length;
      d.qw = rotation.w / length;
    }
    d.px = d.x;
    d.py = d.y;
    d.pz = d.z;
    d.x = location.x;
    d.y = location.y;
    d.z = location.z;
    this.body.wakeUp();
  }
  /** 释放约束，保留刚体当前速度。重复调用无作用。 */
  remove() {
    if (this.isValid) {
      this.body.core.drag = null;
      this.body.wakeUp();
    }
  }
};

// src/mc/body.ts
function finite3(name, v) {
  if (!Number.isFinite(v.x) || !Number.isFinite(v.y) || !Number.isFinite(v.z)) {
    throw new Error(`${name} \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 (${v.x}, ${v.y}, ${v.z})`);
  }
}
function nonNegative(name, v) {
  if (!(v >= 0) || !Number.isFinite(v)) throw new Error(`${name} \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E\u96F6\u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${v}`);
  return v;
}
var Body = class {
  /**
   * 由 PhysicsWorld.createBody 创建。
   * @internal
   */
  constructor(owner, core, userData) {
    /** 合并碰撞盒前的逻辑形状，用于存档。
     * @internal */
    this.snapshotShapes = null;
    this.valid = true;
    /** 上一次被世界推进之前的位置和朝向（用于插值），由世界在每一步开始时记录 */
    this.prevPos = { x: 0, y: 0, z: 0 };
    this.prevRot = { x: 0, y: 0, z: 0, w: 1 };
    this.owner = owner;
    this.core = core;
    this.userDataValue = userData;
    core.userData = this;
  }
  /** 内部使用：世界销毁或刚体移除时调用
   * @internal */
  invalidate() {
    this.core.drag = null;
    this.valid = false;
  }
  check() {
    if (!this.valid) throw new Error("\u521A\u4F53\u5DF2\u7ECF\u5931\u6548\uFF08\u5DF2\u88AB\u79FB\u9664\uFF0C\u6216\u6240\u5728\u7684\u4E16\u754C\u5DF2\u9500\u6BC1\uFF09");
  }
  /**
   * 刚体所在的世界。
   */
  get world() {
    return this.owner;
  }
  /**
   * 刚体是否有效。
   *
   * @remarks
   * 刚体被移除或所在的世界被销毁后为 `false`，此后访问该刚体会抛出错误。
   */
  get isValid() {
    return this.valid;
  }
  /**
   * 与刚体关联的自定义数据。
   */
  get userData() {
    return this.userDataValue;
  }
  set userData(value) {
    this.userDataValue = value;
    if (this.valid) this.owner.markChanged();
  }
  /**
   * 刚体的运动类型。
   */
  get type() {
    this.check();
    const t = this.core.type;
    return t === RigidBodyType.STATIC ? "static" : t === RigidBodyType.KINEMATIC ? "kinematic" : "dynamic";
  }
  set type(value) {
    this.check();
    this.owner.markChanged();
    const t = value === "static" ? RigidBodyType.STATIC : value === "kinematic" ? RigidBodyType.KINEMATIC : value === "dynamic" ? RigidBodyType.DYNAMIC : -1;
    if (t < 0) throw new Error(`\u672A\u77E5\u7684\u521A\u4F53\u7C7B\u578B\uFF1A${String(value)}`);
    if (t === this.core.type) return;
    this.core.drag = null;
    this.core.setType(t);
  }
  /**
   * 刚体的位置，单位为格。
   *
   * @remarks
   * 设置时保留速度、唤醒刚体，并同步更新 {@link Body.previousLocation}。
   */
  get location() {
    this.check();
    const t = this.core.transform;
    return { x: t.px, y: t.py, z: t.pz };
  }
  set location(value) {
    this.check();
    this.owner.markChanged();
    finite3("location", value);
    const c = this.core;
    c.transform.px = value.x;
    c.transform.py = value.y;
    c.transform.pz = value.z;
    c.updateTransformExt();
    this.prevPos = { x: value.x, y: value.y, z: value.z };
  }
  /**
   * 刚体的朝向。
   *
   * @remarks
   * 设置时唤醒刚体，并同步更新 {@link Body.previousRotation}。
   */
  get rotation() {
    this.check();
    const t = this.core.transform;
    return matrixToQuaternion(t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22);
  }
  set rotation(value) {
    this.check();
    this.owner.markChanged();
    const m = quaternionToMatrix(value);
    const c = this.core, t = c.transform;
    t.r00 = m[0];
    t.r01 = m[1];
    t.r02 = m[2];
    t.r10 = m[3];
    t.r11 = m[4];
    t.r12 = m[5];
    t.r20 = m[6];
    t.r21 = m[7];
    t.r22 = m[8];
    c.updateInvInertia();
    c.updateTransformExt();
    this.prevRot = this.rotation;
  }
  /**
   * 刚体的线速度，单位为格/秒。
   */
  get linearVelocity() {
    this.check();
    return { x: this.core.vx, y: this.core.vy, z: this.core.vz };
  }
  set linearVelocity(value) {
    this.check();
    this.owner.markChanged();
    finite3("linearVelocity", value);
    this.core.vx = value.x;
    this.core.vy = value.y;
    this.core.vz = value.z;
    this.core.wakeUp();
    this.core.speculativeStale = true;
  }
  /**
   * 刚体的角速度，单位为弧度/秒。
   *
   * @remarks
   * 方向为转轴方向，长度为转速。
   */
  get angularVelocity() {
    this.check();
    return { x: this.core.wx, y: this.core.wy, z: this.core.wz };
  }
  set angularVelocity(value) {
    this.check();
    this.owner.markChanged();
    finite3("angularVelocity", value);
    this.core.wx = value.x;
    this.core.wy = value.y;
    this.core.wz = value.z;
    this.core.wakeUp();
    this.core.speculativeStale = true;
  }
  /**
   * 线速度的阻尼。
   */
  get linearDamping() {
    this.check();
    return this.core.linearDamping;
  }
  set linearDamping(value) {
    this.check();
    this.owner.markChanged();
    this.core.linearDamping = nonNegative("linearDamping", value);
  }
  /**
   * 角速度的阻尼。
   */
  get angularDamping() {
    this.check();
    return this.core.angularDamping;
  }
  set angularDamping(value) {
    this.check();
    this.owner.markChanged();
    this.core.angularDamping = nonNegative("angularDamping", value);
  }
  /**
   * 重力倍数。
   */
  get gravityScale() {
    this.check();
    return this.core.gravityScale;
  }
  set gravityScale(value) {
    this.check();
    this.owner.markChanged();
    if (!Number.isFinite(value)) throw new Error(`gravityScale \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 ${value}`);
    this.core.gravityScale = value;
    this.core.wakeUp();
  }
  /**
   * 刚体的质量，单位为千克。
   *
   * @remarks
   * 静态刚体和运动学刚体的质量为 `0`。
   */
  get mass() {
    this.check();
    return this.core.type === RigidBodyType.DYNAMIC ? this.core.mass : 0;
  }
  /**
   * 刚体浸在水或岩浆里的体积，单位为格³。
   *
   * @remarks
   *
   * - 每个 tick 推进前更新，体积包括空心船体里托住的空气。
   * - 没有地形、关闭 `fluids` 或刚体不是动态刚体时为 `0`。
   * - 刚体休眠时保留休眠前的值。
   */
  get submerged() {
    this.check();
    return this.owner.submergedVolume(this);
  }
  /**
   * 刚体当前是否处于休眠状态。
   */
  get isSleeping() {
    this.check();
    return this.core.sleeping && !this.owner.isWaiting(this);
  }
  /**
   * 刚体上一步的位置，单位为格。
   *
   * @remarks
   * 与 {@link Body.location} 组成位置插值的两个端点。
   *
   * @example interpolate.ts
   * {@includeCode ../../examples/display-sync/interpolate.ts}
   */
  get previousLocation() {
    this.check();
    return { x: this.prevPos.x, y: this.prevPos.y, z: this.prevPos.z };
  }
  /**
   * 刚体上一步的朝向。
   */
  get previousRotation() {
    this.check();
    return { x: this.prevRot.x, y: this.prevRot.y, z: this.prevRot.z, w: this.prevRot.w };
  }
  /**
   * 记录步进前的位置和朝向，用于插值。
   * @internal
   */
  recordPrevious() {
    const t = this.core.transform;
    this.prevPos.x = t.px;
    this.prevPos.y = t.py;
    this.prevPos.z = t.pz;
    this.prevRot = matrixToQuaternion(t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22);
  }
  /**
   * 对刚体施加冲量，立即改变其速度。
   *
   * @remarks
   * 速度变化量等于冲量除以质量。仅对动态刚体有效，并会唤醒刚体。
   *
   * @param impulse
   * 冲量，单位为千克·格/秒，世界坐标。
   * @param point
   * 作用点，世界坐标。省略时作用于重心；作用点偏离重心时，同时产生旋转。
   *
   * @example apply-impulse.ts
   * {@includeCode ../../examples/body/apply-impulse.ts}
   */
  applyImpulse(impulse, point2) {
    this.check();
    this.owner.markChanged();
    finite3("impulse", impulse);
    if (this.core.type !== RigidBodyType.DYNAMIC) return;
    if (point2 === void 0) {
      this.core.applyLinearImpulse(impulse.x, impulse.y, impulse.z);
    } else {
      finite3("point", point2);
      this.core.applyImpulseAt(impulse.x, impulse.y, impulse.z, point2.x, point2.y, point2.z);
    }
  }
  /**
   * 对刚体施加力。
   *
   * @remarks
   * 力在下一步生效，步进结束后清零。仅对动态刚体有效，并会唤醒刚体。
   *
   * @param force
   * 力，单位为牛顿，世界坐标。
   * @param point
   * 作用点，世界坐标。省略时作用于重心。
   *
   * @example apply-force.ts
   * {@includeCode ../../examples/body/apply-force.ts}
   */
  applyForce(force, point2) {
    this.check();
    this.owner.markChanged();
    finite3("force", force);
    if (this.core.type !== RigidBodyType.DYNAMIC) return;
    if (point2 === void 0) {
      this.core.applyForceToCenter(force.x, force.y, force.z);
    } else {
      finite3("point", point2);
      this.core.applyForceAt(force.x, force.y, force.z, point2.x, point2.y, point2.z);
    }
  }
  /**
   * 对刚体施加角冲量，立即改变其角速度。
   *
   * @remarks
   * 仅对动态刚体有效，并会唤醒刚体。
   *
   * @param impulse
   * 角冲量，世界坐标。
   */
  applyAngularImpulse(impulse) {
    this.check();
    this.owner.markChanged();
    finite3("impulse", impulse);
    if (this.core.type !== RigidBodyType.DYNAMIC) return;
    this.core.applyAngularImpulse(impulse.x, impulse.y, impulse.z);
  }
  /**
   * 对刚体施加力矩。
   *
   * @remarks
   * 力矩仅在下一步生效，每步结束后清零。仅对动态刚体有效，并会唤醒刚体。
   *
   * @param torque
   * 力矩，单位为牛顿·格，世界坐标。
   */
  applyTorque(torque) {
    this.check();
    this.owner.markChanged();
    finite3("torque", torque);
    if (this.core.type !== RigidBodyType.DYNAMIC) return;
    this.core.applyTorque(torque.x, torque.y, torque.z);
  }
  /**
   * 唤醒刚体。
   */
  wakeUp() {
    this.check();
    this.owner.markChanged();
    this.core.wakeUp();
  }
  /**
   * 使刚体立即进入休眠状态。
   *
   * @remarks
   * 刚体的速度会被保留，被唤醒后继续运动。
   */
  sleep() {
    this.check();
    this.owner.markChanged();
    this.core.sleep();
  }
  /**
   * 将世界坐标转换为刚体的局部坐标。
   *
   * @param point
   * 世界坐标。
   * @returns
   * 刚体的局部坐标。
   */
  worldToLocal(point2) {
    this.check();
    finite3("point", point2);
    const t = this.core.transform;
    const dx = point2.x - t.px, dy = point2.y - t.py, dz = point2.z - t.pz;
    return {
      x: t.r00 * dx + t.r10 * dy + t.r20 * dz,
      y: t.r01 * dx + t.r11 * dy + t.r21 * dz,
      z: t.r02 * dx + t.r12 * dy + t.r22 * dz
    };
  }
  /**
   * 将刚体的局部坐标转换为世界坐标。
   *
   * @param point
   * 刚体的局部坐标。
   * @returns
   * 世界坐标。
   */
  localToWorld(point2) {
    this.check();
    finite3("point", point2);
    const t = this.core.transform;
    return {
      x: t.r00 * point2.x + t.r01 * point2.y + t.r02 * point2.z + t.px,
      y: t.r10 * point2.x + t.r11 * point2.y + t.r12 * point2.z + t.py,
      z: t.r20 * point2.x + t.r21 * point2.y + t.r22 * point2.z + t.pz
    };
  }
  /**
   * 返回刚体在某一点的速度。
   *
   * @remarks
   * 该速度等于线速度加上旋转产生的速度。
   *
   * @param point
   * 世界坐标。
   * @returns
   * 该点的速度，单位为格/秒，世界坐标。
   */
  getVelocityAtPoint(point2) {
    this.check();
    finite3("point", point2);
    const c = this.core, t = c.transform;
    const rx = point2.x - t.px, ry = point2.y - t.py, rz = point2.z - t.pz;
    return {
      x: c.vx + (c.wy * rz - c.wz * ry),
      y: c.vy + (c.wz * rx - c.wx * rz),
      z: c.vz + (c.wx * ry - c.wy * rx)
    };
  }
  /**
   * 将刚体从世界中移除。
   *
   * @remarks
   * 与该刚体相连的关节同时被移除。重复调用没有额外效果。
   *
   * @example remove.ts
   * {@includeCode ../../examples/body/remove.ts}
   */
  remove() {
    if (!this.valid) return;
    this.owner.removeBody(this);
  }
  /**
   * 创建抓取约束，替换此刚体已有的抓取约束。
   *
   * @example drag.ts
   * {@includeCode ../../examples/body/drag.ts}
   */
  createDrag(options) {
    this.check();
    return new Drag(this, options);
  }
};

// src/mc/joints.ts
var Joint2 = class {
  /**
   * 由 PhysicsWorld.createJoint 创建。
   * @internal
   */
  constructor(owner, core, a, b, kind) {
    this.valid = true;
    this.owner = owner;
    this.core = core;
    this.a = a;
    this.b = b;
    this.kind = kind;
  }
  /** 内部使用：关节被移除、断开或所在的世界销毁时调用
   * @internal */
  invalidate() {
    this.valid = false;
  }
  check() {
    if (!this.valid) throw new Error("\u5173\u8282\u5DF2\u7ECF\u5931\u6548\uFF08\u5DF2\u88AB\u79FB\u9664\u6216\u65AD\u5F00\uFF0C\u6216\u6240\u5728\u7684\u4E16\u754C\u5DF2\u9500\u6BC1\uFF09");
  }
  /**
   * 关节所在的世界。
   */
  get world() {
    return this.owner;
  }
  /**
   * 关节连接的第一个刚体。
   */
  get bodyA() {
    return this.a;
  }
  /**
   * 关节连接的第二个刚体。
   */
  get bodyB() {
    return this.b;
  }
  /**
   * 关节的种类，与创建时的 `type` 一致。
   */
  get type() {
    return this.kind;
  }
  /**
   * 关节是否有效。
   *
   * @remarks
   * 关节被移除或断开，或任一刚体被移除后为 `false`。
   */
  get isValid() {
    return this.valid;
  }
  /**
   * 连接点在刚体 A 上的世界坐标。
   */
  get anchorA() {
    this.check();
    const j = this.core;
    j.syncAnchors();
    return { x: j.a1X, y: j.a1Y, z: j.a1Z };
  }
  /**
   * 连接点在刚体 B 上的世界坐标。
   */
  get anchorB() {
    this.check();
    const j = this.core;
    j.syncAnchors();
    return { x: j.a2X, y: j.a2Y, z: j.a2Z };
  }
  /**
   * 上一步关节承受的力，单位为牛顿。
   *
   * @remarks
   * 可用于判断关节是否接近断开。
   */
  get appliedForce() {
    this.check();
    return { x: this.core.appliedForceX, y: this.core.appliedForceY, z: this.core.appliedForceZ };
  }
  /**
   * 上一步关节承受的力矩，单位为牛顿·格。
   */
  get appliedTorque() {
    this.check();
    return { x: this.core.appliedTorqueX, y: this.core.appliedTorqueY, z: this.core.appliedTorqueZ };
  }
  /**
   * 移除关节。
   *
   * @remarks
   * 两个刚体恢复自由。重复调用没有额外效果。
   */
  remove() {
    if (!this.valid) return;
    this.owner.removeJoint(this);
  }
};

// src/mc/world.ts
import {
  BlockTypes,
  BlockVolume,
  ListBlockVolume,
  LocationInUnloadedChunkError,
  LocationOutOfWorldBoundariesError,
  system as system2,
  UnloadedChunksError,
  world as world2
} from "@minecraft/server";

// src/core/collision/geometry/ray-cast-hit.ts
var RayCastHit = class {
  constructor() {
    /** 命中点 */
    this.positionX = 0;
    this.positionY = 0;
    this.positionZ = 0;
    /** 命中面的法线（朝外，单位向量） */
    this.normalX = 0;
    this.normalY = 0;
    this.normalZ = 0;
    /** 命中点在线段上的比例：0 是起点，1 是终点 */
    this.fraction = 0;
  }
};

// src/core/dynamics/callback/ray-cast-closest.ts
var RayCastClosest = class {
  constructor() {
    /** 命中的形状，没命中时为 null */
    this.shape = null;
    this.positionX = 0;
    this.positionY = 0;
    this.positionZ = 0;
    this.normalX = 0;
    this.normalY = 0;
    this.normalZ = 0;
    this.fraction = 1;
    /** 是否命中了任何形状 */
    this.hit = false;
  }
  clear() {
    this.shape = null;
    this.fraction = 1;
    this.positionX = 0;
    this.positionY = 0;
    this.positionZ = 0;
    this.normalX = 0;
    this.normalY = 0;
    this.normalZ = 0;
    this.hit = false;
  }
  process(shape, hit) {
    if (hit.fraction < this.fraction) {
      this.shape = shape;
      this.hit = true;
      this.fraction = hit.fraction;
      this.positionX = hit.positionX;
      this.positionY = hit.positionY;
      this.positionZ = hit.positionZ;
      this.normalX = hit.normalX;
      this.normalY = hit.normalY;
      this.normalZ = hit.normalZ;
    }
  }
};

// src/core/dynamics/rigidbody/rigid-body.ts
var RigidBody = class {
  constructor(config) {
    this.prev = null;
    this.next = null;
    this.world = null;
    this.shapeList = null;
    this.shapeListLast = null;
    this.numShapes = 0;
    /** 接触链表：通过它找到和这个刚体有关的接触 */
    this.contactLinkList = null;
    this.contactLinkListLast = null;
    this.numContactLinks = 0;
    /** 关节链表：通过它找到和这个刚体有关的关节 */
    this.jointLinkList = null;
    this.jointLinkListLast = null;
    this.numJointLinks = 0;
    /** 位置修正用的伪速度，一步之后清零 */
    this.pvx = 0;
    this.pvy = 0;
    this.pvz = 0;
    this.pwx = 0;
    this.pwy = 0;
    this.pwz = 0;
    /** 上一步、这一步重心的世界变换 */
    this.ptransform = new Transform();
    /** 最近一次达到撞击速度阈值的步进编号。 */
    this.impactStamp = -1;
    /** 速度变化后预测接触距离失效。下一步碰撞检测前按新速度重新同步。 */
    this.speculativeStale = false;
    /** 本 tick 是否跳过休眠计时，默认为 false。流体不平衡时由适配层设置。 */
    this.restless = false;
    this.drag = null;
    this.transform = new Transform();
    this.sleepTime = 0;
    this.sleeping = false;
    /** 是否锁定自动唤醒，用于等待地形时的冻结。 */
    this.wakeLocked = false;
    this.mass = 0;
    this.invMass = 0;
    /** 局部惯性张量 */
    this.li00 = 0;
    this.li01 = 0;
    this.li02 = 0;
    this.li10 = 0;
    this.li11 = 0;
    this.li12 = 0;
    this.li20 = 0;
    this.li21 = 0;
    this.li22 = 0;
    /** 每根轴是否允许转动（1 允许，0 锁住） */
    this.rotFactorX = 1;
    this.rotFactorY = 1;
    this.rotFactorZ = 1;
    /** 局部逆惯性张量（已乘 rotFactor） */
    this.ili00 = 0;
    this.ili01 = 0;
    this.ili02 = 0;
    this.ili10 = 0;
    this.ili11 = 0;
    this.ili12 = 0;
    this.ili20 = 0;
    this.ili21 = 0;
    this.ili22 = 0;
    /** 局部逆惯性张量（未乘 rotFactor） */
    this.iliw00 = 0;
    this.iliw01 = 0;
    this.iliw02 = 0;
    this.iliw10 = 0;
    this.iliw11 = 0;
    this.iliw12 = 0;
    this.iliw20 = 0;
    this.iliw21 = 0;
    this.iliw22 = 0;
    /** 世界坐标下的逆惯性张量 */
    this.ii00 = 0;
    this.ii01 = 0;
    this.ii02 = 0;
    this.ii10 = 0;
    this.ii11 = 0;
    this.ii12 = 0;
    this.ii20 = 0;
    this.ii21 = 0;
    this.ii22 = 0;
    /** 这一步累积的力和力矩，一步之后清零 */
    this.fx = 0;
    this.fy = 0;
    this.fz = 0;
    this.tx = 0;
    this.ty = 0;
    this.tz = 0;
    /** 这一步接触产生的冲量 */
    this.lciX = 0;
    this.lciY = 0;
    this.lciZ = 0;
    this.aciX = 0;
    this.aciY = 0;
    this.aciZ = 0;
    /** 子步接触产生的速度增量，供同一 tick 内的热启动复用。 */
    this.contactDeltaVX = 0;
    this.contactDeltaVY = 0;
    this.contactDeltaVZ = 0;
    this.contactDeltaWX = 0;
    this.contactDeltaWY = 0;
    this.contactDeltaWZ = 0;
    this.addedToIsland = false;
    this.gravityScale = 1;
    this.userData = null;
    this.vx = config.vx;
    this.vy = config.vy;
    this.vz = config.vz;
    this.wx = config.wx;
    this.wy = config.wy;
    this.wz = config.wz;
    this.ptransform.copyFrom(config.transform);
    this.transform.copyFrom(this.ptransform);
    this.type = config.type;
    this.autoSleep = config.autoSleep;
    this.sleepingVelocityThreshold = config.sleepingVelocityThreshold;
    this.sleepingAngularVelocityThreshold = config.sleepingAngularVelocityThreshold;
    this.sleepingTimeThreshold = config.sleepingTimeThreshold;
    this.linearDamping = config.linearDamping;
    this.angularDamping = config.angularDamping;
  }
  /** 按速度积分一步。静态刚体只把速度清零 */
  integrate(dt) {
    if (this.type === RigidBodyType.STATIC) {
      this.vx = 0;
      this.vy = 0;
      this.vz = 0;
      this.wx = 0;
      this.wy = 0;
      this.wz = 0;
      this.pvx = 0;
      this.pvy = 0;
      this.pvz = 0;
      this.pwx = 0;
      this.pwy = 0;
      this.pwz = 0;
      return;
    }
    let tX = this.vx * dt, tY = this.vy * dt, tZ = this.vz * dt;
    let rX = this.wx * dt, rY = this.wy * dt, rZ = this.wz * dt;
    const tLenSq = tX * tX + tY * tY + tZ * tZ;
    const rLenSq = rX * rX + rY * rY + rZ * rZ;
    if (tLenSq === 0 && rLenSq === 0) return;
    if (tLenSq > MAX_TRANSLATION_PER_STEP * MAX_TRANSLATION_PER_STEP) {
      const l = MAX_TRANSLATION_PER_STEP / Math.sqrt(tLenSq);
      this.vx *= l;
      this.vy *= l;
      this.vz *= l;
      tX *= l;
      tY *= l;
      tZ *= l;
    }
    if (rLenSq > MAX_ROTATION_PER_STEP * MAX_ROTATION_PER_STEP) {
      const l = MAX_ROTATION_PER_STEP / Math.sqrt(rLenSq);
      this.wx *= l;
      this.wy *= l;
      this.wz *= l;
      rX *= l;
      rY *= l;
      rZ *= l;
    }
    this.transform.px += tX;
    this.transform.py += tY;
    this.transform.pz += tZ;
    this.applyRotation(rX, rY, rZ);
  }
  /** 按位置修正得到的伪速度移动刚体，然后清零伪速度 */
  integratePseudoVelocity() {
    const pLenSq = this.pvx * this.pvx + this.pvy * this.pvy + this.pvz * this.pvz;
    const aLenSq = this.pwx * this.pwx + this.pwy * this.pwy + this.pwz * this.pwz;
    if (pLenSq === 0 && aLenSq === 0) return;
    const tX = this.pvx, tY = this.pvy, tZ = this.pvz;
    const rX = this.pwx, rY = this.pwy, rZ = this.pwz;
    this.pvx = 0;
    this.pvy = 0;
    this.pvz = 0;
    this.pwx = 0;
    this.pwy = 0;
    this.pwz = 0;
    if (this.type === RigidBodyType.STATIC) return;
    this.transform.px += tX;
    this.transform.py += tY;
    this.transform.pz += tZ;
    this.applyRotation(rX, rY, rZ);
  }
  /** 绕旋转向量 (rX, rY, rZ) 转动，然后更新世界逆惯性张量 */
  applyRotation(rX, rY, rZ) {
    const theta = Math.sqrt(rX * rX + rY * rY + rZ * rZ);
    const halfTheta = theta * 0.5;
    let factor, cosHalfTheta;
    if (halfTheta < 0.5) {
      const ht2 = halfTheta * halfTheta;
      factor = 1 / 2 * (1 - ht2 * (1 / 6) + ht2 * ht2 * (1 / 120));
      cosHalfTheta = 1 - ht2 * (1 / 2) + ht2 * ht2 * (1 / 24);
    } else {
      factor = Math.sin(halfTheta) / theta;
      cosHalfTheta = Math.cos(halfTheta);
    }
    const dqX = rX * factor, dqY = rY * factor, dqZ = rZ * factor, dqW = cosHalfTheta;
    const tf = this.transform;
    const e00 = tf.r00, e11 = tf.r11, e22 = tf.r22;
    const t = e00 + e11 + e22;
    let s, qX, qY, qZ, qW;
    if (t > 0) {
      s = Math.sqrt(t + 1);
      qW = 0.5 * s;
      s = 0.5 / s;
      qX = (tf.r21 - tf.r12) * s;
      qY = (tf.r02 - tf.r20) * s;
      qZ = (tf.r10 - tf.r01) * s;
    } else if (e00 > e11) {
      if (e00 > e22) {
        s = Math.sqrt(e00 - e11 - e22 + 1);
        qX = 0.5 * s;
        s = 0.5 / s;
        qY = (tf.r01 + tf.r10) * s;
        qZ = (tf.r02 + tf.r20) * s;
        qW = (tf.r21 - tf.r12) * s;
      } else {
        s = Math.sqrt(e22 - e00 - e11 + 1);
        qZ = 0.5 * s;
        s = 0.5 / s;
        qX = (tf.r02 + tf.r20) * s;
        qY = (tf.r12 + tf.r21) * s;
        qW = (tf.r10 - tf.r01) * s;
      }
    } else if (e11 > e22) {
      s = Math.sqrt(e11 - e22 - e00 + 1);
      qY = 0.5 * s;
      s = 0.5 / s;
      qX = (tf.r01 + tf.r10) * s;
      qZ = (tf.r12 + tf.r21) * s;
      qW = (tf.r02 - tf.r20) * s;
    } else {
      s = Math.sqrt(e22 - e00 - e11 + 1);
      qZ = 0.5 * s;
      s = 0.5 / s;
      qX = (tf.r02 + tf.r20) * s;
      qY = (tf.r12 + tf.r21) * s;
      qW = (tf.r10 - tf.r01) * s;
    }
    if (compat.keepOimoBugs) {
      qX = dqW * qX + dqX * qW + dqY * qZ - dqZ * qY;
      qY = dqW * qY - dqX * qZ + dqY * qW + dqZ * qX;
      qZ = dqW * qZ + dqX * qY - dqY * qX + dqZ * qW;
      qW = dqW * qW - dqX * qX - dqY * qY - dqZ * qZ;
    } else {
      const nX = dqW * qX + dqX * qW + dqY * qZ - dqZ * qY;
      const nY = dqW * qY - dqX * qZ + dqY * qW + dqZ * qX;
      const nZ = dqW * qZ + dqX * qY - dqY * qX + dqZ * qW;
      const nW = dqW * qW - dqX * qX - dqY * qY - dqZ * qZ;
      qX = nX;
      qY = nY;
      qZ = nZ;
      qW = nW;
    }
    let l = qX * qX + qY * qY + qZ * qZ + qW * qW;
    if (l > 1e-32) l = 1 / Math.sqrt(l);
    qX *= l;
    qY *= l;
    qZ *= l;
    qW *= l;
    const x2 = 2 * qX, y2 = 2 * qY, z2 = 2 * qZ;
    const xx = qX * x2, yy = qY * y2, zz = qZ * z2;
    const xy = qX * y2, yz = qY * z2, xz = qX * z2;
    const qwx = qW * x2, qwy = qW * y2, qwz = qW * z2;
    tf.r00 = 1 - yy - zz;
    tf.r01 = xy - qwz;
    tf.r02 = xz + qwy;
    tf.r10 = xy + qwz;
    tf.r11 = 1 - xx - zz;
    tf.r12 = yz - qwx;
    tf.r20 = xz - qwy;
    tf.r21 = yz + qwx;
    tf.r22 = 1 - xx - yy;
    this.updateInvInertia();
  }
  /** 根据形状重算质量和惯性张量，然后唤醒刚体 */
  updateMass() {
    let totalMass = 0;
    let i00 = 0, i01 = 0, i02 = 0, i10 = 0, i11 = 0, i12 = 0, i20 = 0, i21 = 0, i22 = 0;
    for (let s = this.shapeList; s !== null; s = s.next) {
      const g = s.geom;
      g.updateMass();
      const mass = s.density * g.volume;
      const r = s.localTransform;
      const a00 = r.r00 * g.ic00 + r.r01 * g.ic10 + r.r02 * g.ic20;
      const a01 = r.r00 * g.ic01 + r.r01 * g.ic11 + r.r02 * g.ic21;
      const a02 = r.r00 * g.ic02 + r.r01 * g.ic12 + r.r02 * g.ic22;
      const a10 = r.r10 * g.ic00 + r.r11 * g.ic10 + r.r12 * g.ic20;
      const a11 = r.r10 * g.ic01 + r.r11 * g.ic11 + r.r12 * g.ic21;
      const a12 = r.r10 * g.ic02 + r.r11 * g.ic12 + r.r12 * g.ic22;
      const a20 = r.r20 * g.ic00 + r.r21 * g.ic10 + r.r22 * g.ic20;
      const a21 = r.r20 * g.ic01 + r.r21 * g.ic11 + r.r22 * g.ic21;
      const a22 = r.r20 * g.ic02 + r.r21 * g.ic12 + r.r22 * g.ic22;
      let j00 = a00 * r.r00 + a01 * r.r01 + a02 * r.r02;
      let j01 = a00 * r.r10 + a01 * r.r11 + a02 * r.r12;
      let j02 = a00 * r.r20 + a01 * r.r21 + a02 * r.r22;
      let j10 = a10 * r.r00 + a11 * r.r01 + a12 * r.r02;
      let j11 = a10 * r.r10 + a11 * r.r11 + a12 * r.r12;
      let j12 = a10 * r.r20 + a11 * r.r21 + a12 * r.r22;
      let j20 = a20 * r.r00 + a21 * r.r01 + a22 * r.r02;
      let j21 = a20 * r.r10 + a21 * r.r11 + a22 * r.r12;
      let j22 = a20 * r.r20 + a21 * r.r21 + a22 * r.r22;
      j00 *= mass;
      j01 *= mass;
      j02 *= mass;
      j10 *= mass;
      j11 *= mass;
      j12 *= mass;
      j20 *= mass;
      j21 *= mass;
      j22 *= mass;
      const px = r.px, py = r.py, pz = r.pz;
      const xx = px * px, yy = py * py, zz = pz * pz;
      const xy = -px * py, yz = -py * pz, zx = -pz * px;
      j00 += (yy + zz) * mass;
      j01 += xy * mass;
      j02 += zx * mass;
      j10 += xy * mass;
      j11 += (xx + zz) * mass;
      j12 += yz * mass;
      j20 += zx * mass;
      j21 += yz * mass;
      j22 += (xx + yy) * mass;
      totalMass += mass;
      i00 += j00;
      i01 += j01;
      i02 += j02;
      i10 += j10;
      i11 += j11;
      i12 += j12;
      i20 += j20;
      i21 += j21;
      i22 += j22;
    }
    this.mass = totalMass;
    this.li00 = i00;
    this.li01 = i01;
    this.li02 = i02;
    this.li10 = i10;
    this.li11 = i11;
    this.li12 = i12;
    this.li20 = i20;
    this.li21 = i21;
    this.li22 = i22;
    this.completeMassData();
    this.wakeUp();
  }
  /** 由 mass 和局部惯性张量算出逆质量、逆惯性张量 */
  completeMassData() {
    const e00 = this.li00, e01 = this.li01, e02 = this.li02;
    const e10 = this.li10, e11 = this.li11, e12 = this.li12;
    const e20 = this.li20, e21 = this.li21, e22 = this.li22;
    const d00 = e11 * e22 - e12 * e21;
    const d01 = e10 * e22 - e12 * e20;
    const d02 = e10 * e21 - e11 * e20;
    const det2 = e00 * d00 - e01 * d01 + e02 * d02;
    if (this.mass > 0 && det2 > 0 && this.type === RigidBodyType.DYNAMIC) {
      this.invMass = 1 / this.mass;
      const d10 = e01 * e22 - e02 * e21;
      const d11 = e00 * e22 - e02 * e20;
      const d12 = e00 * e21 - e01 * e20;
      const d20 = e01 * e12 - e02 * e11;
      const d21 = e00 * e12 - e02 * e10;
      const d22 = e00 * e11 - e01 * e10;
      let inv2 = e00 * d00 - e01 * d01 + e02 * d02;
      if (inv2 < -1e-32 || inv2 > 1e-32) inv2 = 1 / inv2;
      this.iliw00 = d00 * inv2;
      this.iliw01 = -d10 * inv2;
      this.iliw02 = d20 * inv2;
      this.iliw10 = -d01 * inv2;
      this.iliw11 = d11 * inv2;
      this.iliw12 = -d21 * inv2;
      this.iliw20 = d02 * inv2;
      this.iliw21 = -d12 * inv2;
      this.iliw22 = d22 * inv2;
    } else {
      this.invMass = 0;
      this.iliw00 = 0;
      this.iliw01 = 0;
      this.iliw02 = 0;
      this.iliw10 = 0;
      this.iliw11 = 0;
      this.iliw12 = 0;
      this.iliw20 = 0;
      this.iliw21 = 0;
      this.iliw22 = 0;
      if (this.type === RigidBodyType.DYNAMIC) {
        this.invMass = 1e-9;
        this.iliw00 = 1e-9;
        this.iliw11 = 1e-9;
        this.iliw22 = 1e-9;
      }
    }
    const x = this.rotFactorX, y = this.rotFactorY, z = this.rotFactorZ;
    this.ili00 = this.iliw00 * x;
    this.ili01 = this.iliw01 * x;
    this.ili02 = this.iliw02 * x;
    this.ili10 = this.iliw10 * y;
    this.ili11 = this.iliw11 * y;
    this.ili12 = this.iliw12 * y;
    this.ili20 = this.iliw20 * z;
    this.ili21 = this.iliw21 * z;
    this.ili22 = this.iliw22 * z;
    this.updateInvInertia();
  }
  /** 世界逆惯性张量 = R · ili · Rᵀ，再按行乘 rotFactor */
  updateInvInertia() {
    const r = this.transform;
    const a00 = r.r00 * this.ili00 + r.r01 * this.ili10 + r.r02 * this.ili20;
    const a01 = r.r00 * this.ili01 + r.r01 * this.ili11 + r.r02 * this.ili21;
    const a02 = r.r00 * this.ili02 + r.r01 * this.ili12 + r.r02 * this.ili22;
    const a10 = r.r10 * this.ili00 + r.r11 * this.ili10 + r.r12 * this.ili20;
    const a11 = r.r10 * this.ili01 + r.r11 * this.ili11 + r.r12 * this.ili21;
    const a12 = r.r10 * this.ili02 + r.r11 * this.ili12 + r.r12 * this.ili22;
    const a20 = r.r20 * this.ili00 + r.r21 * this.ili10 + r.r22 * this.ili20;
    const a21 = r.r20 * this.ili01 + r.r21 * this.ili11 + r.r22 * this.ili21;
    const a22 = r.r20 * this.ili02 + r.r21 * this.ili12 + r.r22 * this.ili22;
    const x = this.rotFactorX, y = this.rotFactorY, z = this.rotFactorZ;
    this.ii00 = (a00 * r.r00 + a01 * r.r01 + a02 * r.r02) * x;
    this.ii01 = (a00 * r.r10 + a01 * r.r11 + a02 * r.r12) * x;
    this.ii02 = (a00 * r.r20 + a01 * r.r21 + a02 * r.r22) * x;
    this.ii10 = (a10 * r.r00 + a11 * r.r01 + a12 * r.r02) * y;
    this.ii11 = (a10 * r.r10 + a11 * r.r11 + a12 * r.r12) * y;
    this.ii12 = (a10 * r.r20 + a11 * r.r21 + a12 * r.r22) * y;
    this.ii20 = (a20 * r.r00 + a21 * r.r01 + a22 * r.r02) * z;
    this.ii21 = (a20 * r.r10 + a21 * r.r11 + a22 * r.r12) * z;
    this.ii22 = (a20 * r.r20 + a21 * r.r21 + a22 * r.r22) * z;
  }
  /** 外部直接改了变换后调用 */
  updateTransformExt() {
    this.ptransform.copyFrom(this.transform);
    this.syncShapes();
    this.wakeUp();
  }
  /** 形状增删或改动后调用：重算质量并同步形状 */
  shapeModified() {
    this.updateMass();
    this.syncShapes();
  }
  syncShapes() {
    if (this.type !== RigidBodyType.STATIC && this.world !== null) {
      const t = this.world.speculativeTick();
      if (t > 0) {
        const w = this.world;
        this.syncShapesSpeculative(t, w.speculativeDistance, w.maxSpeculativeDistance);
        return;
      }
    }
    for (let s = this.shapeList; s !== null; s = s.next) s.sync(this.ptransform, this.transform);
  }
  /** 按当前速度同步形状，并将包围盒扩展到预测接触范围。t 为下一 tick 的时长。 */
  syncShapesSpeculative(t, base, max) {
    this.speculativeStale = false;
    const speed = Math.sqrt(this.vx * this.vx + this.vy * this.vy + this.vz * this.vz);
    const angularSpeed = Math.sqrt(this.wx * this.wx + this.wy * this.wy + this.wz * this.wz);
    const linearReach = speed * t;
    for (let s = this.shapeList; s !== null; s = s.next) s.syncSpeculative(this.ptransform, this.transform, base, max, linearReach, angularSpeed, t);
  }
  /** 位置修正：沿位置冲量平移 */
  applyLinearPositionImpulse(ix, iy, iz) {
    this.transform.px += ix * this.invMass;
    this.transform.py += iy * this.invMass;
    this.transform.pz += iz * this.invMass;
  }
  /** 位置修正：按角位置冲量转动 */
  applyAngularPositionImpulse(ix, iy, iz) {
    this.applyRotation(
      this.ii00 * ix + this.ii01 * iy + this.ii02 * iz,
      this.ii10 * ix + this.ii11 * iy + this.ii12 * iz,
      this.ii20 * ix + this.ii21 * iy + this.ii22 * iz
    );
  }
  /** 速度低于休眠阈值 */
  isSleepy() {
    return this.autoSleep && this.vx * this.vx + this.vy * this.vy + this.vz * this.vz < this.sleepingVelocityThreshold * this.sleepingVelocityThreshold && this.wx * this.wx + this.wy * this.wy + this.wz * this.wz < this.sleepingAngularVelocityThreshold * this.sleepingAngularVelocityThreshold;
  }
  /** 平移（NGS 位置修正用） */
  applyTranslation(tX, tY, tZ) {
    this.transform.px += tX;
    this.transform.py += tY;
    this.transform.pz += tZ;
  }
  /** 没有任何接触和关节 */
  isAlone() {
    return this.numContactLinks === 0 && this.numJointLinks === 0;
  }
  /** 添加形状。刚体已经在世界里时，形状也立即加入世界 */
  addShape(shape) {
    if (this.shapeList === null) {
      this.shapeList = shape;
      this.shapeListLast = shape;
    } else {
      this.shapeListLast.next = shape;
      shape.prev = this.shapeListLast;
      this.shapeListLast = shape;
    }
    this.numShapes++;
    shape.rigidBody = this;
    if (this.world !== null) this.world.addShape(shape);
    this.shapeModified();
  }
  /** 批量添加形状，完成后统一重算质量。 */
  addShapes(shapes) {
    for (const shape of shapes) {
      if (this.shapeList === null) {
        this.shapeList = shape;
      } else {
        this.shapeListLast.next = shape;
        shape.prev = this.shapeListLast;
      }
      this.shapeListLast = shape;
      this.numShapes++;
      shape.rigidBody = this;
      if (this.world !== null) this.world.addShape(shape);
    }
    this.shapeModified();
  }
  removeShape(shape) {
    this.unlinkShape(shape);
    this.shapeModified();
  }
  /** 批量移除形状，完成后统一重算质量。 */
  removeShapes(shapes) {
    for (const shape of shapes) this.unlinkShape(shape);
    this.shapeModified();
  }
  unlinkShape(shape) {
    if (this.world !== null) this.world.removeShape(shape);
    const prev = shape.prev, next = shape.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (shape === this.shapeList) this.shapeList = next;
    if (shape === this.shapeListLast) this.shapeListLast = prev;
    shape.next = null;
    shape.prev = null;
    this.numShapes--;
    shape.rigidBody = null;
  }
  /** 在世界坐标 (px, py, pz) 处施加冲量：改变线速度和角速度，并唤醒刚体 */
  applyImpulseAt(ix, iy, iz, px, py, pz) {
    this.vx += ix * this.invMass;
    this.vy += iy * this.invMass;
    this.vz += iz * this.invMass;
    px -= this.transform.px;
    py -= this.transform.py;
    pz -= this.transform.pz;
    const ax = py * iz - pz * iy;
    const ay = pz * ix - px * iz;
    const az = px * iy - py * ix;
    this.wx += this.ii00 * ax + this.ii01 * ay + this.ii02 * az;
    this.wy += this.ii10 * ax + this.ii11 * ay + this.ii12 * az;
    this.wz += this.ii20 * ax + this.ii21 * ay + this.ii22 * az;
    this.sleeping = false;
    this.sleepTime = 0;
    this.speculativeStale = true;
  }
  /** 在重心施加冲量，不改变角速度 */
  applyLinearImpulse(ix, iy, iz) {
    this.vx += ix * this.invMass;
    this.vy += iy * this.invMass;
    this.vz += iz * this.invMass;
    this.sleeping = false;
    this.sleepTime = 0;
    this.speculativeStale = true;
  }
  /** 施加角冲量，不改变线速度 */
  applyAngularImpulse(ix, iy, iz) {
    this.wx += this.ii00 * ix + this.ii01 * iy + this.ii02 * iz;
    this.wy += this.ii10 * ix + this.ii11 * iy + this.ii12 * iz;
    this.wz += this.ii20 * ix + this.ii21 * iy + this.ii22 * iz;
    this.sleeping = false;
    this.sleepTime = 0;
    this.speculativeStale = true;
  }
  /** 在世界坐标 (px, py, pz) 处施力，累加到下一步，每步结束后清零 */
  applyForceAt(fx, fy, fz, px, py, pz) {
    this.fx += fx;
    this.fy += fy;
    this.fz += fz;
    px -= this.transform.px;
    py -= this.transform.py;
    pz -= this.transform.pz;
    this.tx += py * fz - pz * fy;
    this.ty += pz * fx - px * fz;
    this.tz += px * fy - py * fx;
    this.sleeping = false;
    this.sleepTime = 0;
  }
  /** 在重心施力 */
  applyForceToCenter(fx, fy, fz) {
    this.fx += fx;
    this.fy += fy;
    this.fz += fz;
    this.sleeping = false;
    this.sleepTime = 0;
  }
  /** 施加力矩，累加到下一步 */
  applyTorque(tx, ty, tz) {
    this.tx += tx;
    this.ty += ty;
    this.tz += tz;
    this.sleeping = false;
    this.sleepTime = 0;
  }
  setType(type) {
    this.type = type;
    this.updateMass();
  }
  wakeUp() {
    this.sleeping = false;
    this.sleepTime = 0;
  }
  sleep() {
    this.sleeping = true;
    this.sleepTime = 0;
  }
};

// src/core/collision/broadphase/convex-sweep.ts
var ConvexSweepGeometry = class extends ConvexGeometry {
  constructor() {
    super(-1);
    this.c = null;
    /** 平移量转到 c 的局部坐标 */
    this.localTranslation = new Vec3();
  }
  init(c, transform, translation) {
    this.c = c;
    const tX = translation.x, tY = translation.y, tZ = translation.z;
    this.localTranslation.init(
      transform.r00 * tX + transform.r10 * tY + transform.r20 * tZ,
      transform.r01 * tX + transform.r11 * tY + transform.r21 * tZ,
      transform.r02 * tX + transform.r12 * tY + transform.r22 * tZ
    );
    this.gjkMargin = c.gjkMargin;
  }
  computeLocalSupportingVertex(dir, out) {
    this.c.computeLocalSupportingVertex(dir, out);
    if (dir.dot(this.localTranslation) > 0) out.addEq(this.localTranslation);
  }
  // 仅供 GJK 支撑映射使用，其余几何接口为空实现。
  updateMass() {
  }
  computeAabb() {
  }
  rayCastLocal() {
    return false;
  }
};
var AabbGeometry = class extends ConvexGeometry {
  constructor() {
    super(-1);
    this.min = new Vec3();
    this.max = new Vec3();
  }
  computeLocalSupportingVertex(dir, out) {
    out.x = dir.x > 0 ? this.max.x : this.min.x;
    out.y = dir.y > 0 ? this.max.y : this.min.y;
    out.z = dir.z > 0 ? this.max.z : this.min.z;
  }
  updateMass() {
  }
  computeAabb() {
  }
  rayCastLocal() {
    return false;
  }
};

// src/core/collision/broadphase/proxy.ts
var Proxy = class {
  constructor(userData, id) {
    this.prev = null;
    this.next = null;
    // 宽相实际使用的包围盒。BVH 会在形状包围盒外加一圈余量，并沿位移方向延伸（“加厚”）
    this.minX = 0;
    this.minY = 0;
    this.minZ = 0;
    this.maxX = 0;
    this.maxY = 0;
    this.maxZ = 0;
    this.userData = userData;
    this.id = id;
  }
  setAabb(aabb) {
    this.minX = aabb.minX;
    this.minY = aabb.minY;
    this.minZ = aabb.minZ;
    this.maxX = aabb.maxX;
    this.maxY = aabb.maxY;
    this.maxZ = aabb.maxZ;
  }
};
var ProxyPair = class {
  constructor() {
    this.next = null;
    this.p1 = null;
    this.p2 = null;
  }
};

// src/core/collision/broadphase/broad-phase.ts
var BroadPhase = class {
  constructor(incremental) {
    this.numProxies = 0;
    this.proxyList = null;
    this.proxyListLast = null;
    /** collectPairs() 收集到的代理对 */
    this.proxyPairList = null;
    /** 上次 collectPairs() 做了多少次包围盒测试 */
    this.testCount = 0;
    this.idCount = 0;
    this.proxyPairPool = null;
    this.convexSweep = new ConvexSweepGeometry();
    this.aabbGeometry = new AabbGeometry();
    this.identity = new Transform();
    this.incremental = incremental;
  }
  /**
   * 使用 GJK 判断包围盒与凸体扫掠范围是否相交，不包含 GJK 外边距。
   * 未加厚的代理可能过滤仅在外边距内命中的形状。
   */
  aabbConvexSweepTest(minX, minY, minZ, maxX, maxY, maxZ, convex, begin, translation) {
    this.aabbGeometry.min.init(minX, minY, minZ);
    this.aabbGeometry.max.init(maxX, maxY, maxZ);
    this.convexSweep.init(convex, begin, translation);
    const gjkEpa = GjkEpa.getInstance();
    if (gjkEpa.computeDistance(this.convexSweep, this.aabbGeometry, begin, this.identity, null) === GjkEpaResultState.SUCCEEDED) {
      return gjkEpa.distance <= 0;
    }
    return false;
  }
  /** 两个代理（加厚后的）包围盒是否重叠 */
  isOverlapping(p1, p2) {
    return p1.minX < p2.maxX && p1.maxX > p2.minX && p1.minY < p2.maxY && p1.maxY > p2.minY && p1.minZ < p2.maxZ && p1.maxZ > p2.minZ;
  }
  addProxy(p) {
    this.numProxies++;
    if (this.proxyList === null) {
      this.proxyList = p;
      this.proxyListLast = p;
    } else {
      this.proxyListLast.next = p;
      p.prev = this.proxyListLast;
      this.proxyListLast = p;
    }
  }
  removeProxy(p) {
    this.numProxies--;
    const prev = p.prev, next = p.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (p === this.proxyList) this.proxyList = next;
    if (p === this.proxyListLast) this.proxyListLast = prev;
    p.next = null;
    p.prev = null;
  }
  /** 从对象池取一个代理对，插到链表头部 */
  pickAndPushProxyPair(p1, p2) {
    let pp = this.proxyPairPool;
    if (pp !== null) {
      this.proxyPairPool = pp.next;
      pp.next = null;
    } else {
      pp = new ProxyPair();
    }
    if (this.proxyPairList !== null) pp.next = this.proxyPairList;
    this.proxyPairList = pp;
    pp.p1 = p1;
    pp.p2 = p2;
  }
  /** 将全部代理对归还对象池，保留链表顺序。 */
  poolProxyPairs() {
    let p = this.proxyPairList;
    if (p === null) return;
    let last = p;
    while (p !== null) {
      p.p1 = null;
      p.p2 = null;
      last = p;
      p = p.next;
    }
    last.next = this.proxyPairPool;
    this.proxyPairPool = this.proxyPairList;
    this.proxyPairList = null;
  }
};

// src/core/collision/broadphase/brute-force-broad-phase.ts
var BruteForceBroadPhase = class extends BroadPhase {
  constructor() {
    super(false);
  }
  createProxy(userData, aabb) {
    const p = new Proxy(userData, this.idCount++);
    this.addProxy(p);
    p.setAabb(aabb);
    return p;
  }
  destroyProxy(proxy) {
    this.removeProxy(proxy);
    proxy.userData = null;
  }
  moveProxy(proxy, aabb) {
    proxy.setAabb(aabb);
  }
  rayCast(beginX, beginY, beginZ, endX, endY, endZ, callback) {
    const sMinX = beginX < endX ? beginX : endX;
    const sMinY = beginY < endY ? beginY : endY;
    const sMinZ = beginZ < endZ ? beginZ : endZ;
    const sMaxX = beginX > endX ? beginX : endX;
    const sMaxY = beginY > endY ? beginY : endY;
    const sMaxZ = beginZ > endZ ? beginZ : endZ;
    const dX = endX - beginX;
    const dY = endY - beginY;
    const dZ = endZ - beginZ;
    const adX = dX < 0 ? -dX : dX;
    const adY = dY < 0 ? -dY : dY;
    const adZ = dZ < 0 ? -dZ : dZ;
    let p = this.proxyList;
    while (p !== null) {
      const next = p.next;
      if (!(p.minX > sMaxX || p.maxX < sMinX || p.minY > sMaxY || p.maxY < sMinY || p.minZ > sMaxZ || p.maxZ < sMinZ)) {
        const extX = (p.maxX - p.minX) * 0.5;
        const extY = (p.maxY - p.minY) * 0.5;
        const extZ = (p.maxZ - p.minZ) * 0.5;
        const cpX = beginX - (p.maxX + p.minX) * 0.5;
        const cpY = beginY - (p.maxY + p.minY) * 0.5;
        const cpZ = beginZ - (p.maxZ + p.minZ) * 0.5;
        let s = cpY * dZ - cpZ * dY;
        if (!((s < 0 ? -s : s) - (extY * adZ + extZ * adY) > 0)) {
          s = cpZ * dX - cpX * dZ;
          if (!((s < 0 ? -s : s) - (extZ * adX + extX * adZ) > 0)) {
            s = cpX * dY - cpY * dX;
            if (!((s < 0 ? -s : s) - (extX * adY + extY * adX) > 0)) callback.process(p);
          }
        }
      }
      p = next;
    }
  }
  aabbTest(aabb, callback) {
    let p = this.proxyList;
    while (p !== null) {
      const next = p.next;
      if (aabb.minX < p.maxX && aabb.maxX > p.minX && aabb.minY < p.maxY && aabb.maxY > p.minY && aabb.minZ < p.maxZ && aabb.maxZ > p.minZ) {
        callback.process(p);
      }
      p = next;
    }
  }
  convexCast(convex, begin, translation, callback) {
    let p = this.proxyList;
    while (p !== null) {
      const next = p.next;
      if (this.aabbConvexSweepTest(p.minX, p.minY, p.minZ, p.maxX, p.maxY, p.maxZ, convex, begin, translation)) callback.process(p);
      p = next;
    }
  }
  collectPairs() {
    this.poolProxyPairs();
    this.testCount = 0;
    for (let p1 = this.proxyList; p1 !== null; p1 = p1.next) {
      for (let p2 = p1.next; p2 !== null; p2 = p2.next) {
        this.testCount++;
        if (p1.minX < p2.maxX && p1.maxX > p2.minX && p1.minY < p2.maxY && p1.maxY > p2.minY && p1.minZ < p2.maxZ && p1.maxZ > p2.minZ) {
          this.pickAndPushProxyPair(p1, p2);
        }
      }
    }
  }
};

// src/core/collision/broadphase/bvh-tree.ts
var BvhNode = class {
  constructor() {
    /** 对象池链表 */
    this.next = null;
    this.child0 = null;
    this.child1 = null;
    /** 自己是父节点的第几个子节点（0 或 1） */
    this.childIndex = 0;
    this.parent = null;
    /** 叶子为 0 */
    this.height = 0;
    /** 只有叶子才有 */
    this.proxy = null;
    this.minX = 0;
    this.minY = 0;
    this.minZ = 0;
    this.maxX = 0;
    this.maxY = 0;
    this.maxZ = 0;
  }
  setChild(index, child) {
    if (index === 0) this.child0 = child;
    else this.child1 = child;
    child.parent = this;
    child.childIndex = index;
  }
};
var BvhProxy = class extends Proxy {
  constructor() {
    super(...arguments);
    this.leaf = null;
    /** 这一步移出了加厚包围盒，collectPairs() 时要在树里重新插入 */
    this.moved = false;
  }
};
var BvhTree = class {
  constructor() {
    this.root = null;
    this.nodePool = null;
  }
  /** 为代理创建叶子并插入树 */
  insertProxy(proxy) {
    const leaf = this.pick();
    leaf.proxy = proxy;
    proxy.leaf = leaf;
    leaf.minX = proxy.minX;
    leaf.minY = proxy.minY;
    leaf.minZ = proxy.minZ;
    leaf.maxX = proxy.maxX;
    leaf.maxY = proxy.maxY;
    leaf.maxZ = proxy.maxZ;
    this.insertLeaf(leaf);
  }
  /** 从树里删掉代理的叶子 */
  deleteProxy(proxy) {
    const leaf = proxy.leaf;
    this.deleteLeaf(leaf);
    proxy.leaf = null;
    this.pool(leaf);
  }
  insertLeaf(leaf) {
    if (this.root === null) {
      this.root = leaf;
      return;
    }
    const cX = leaf.minX + leaf.maxX, cY = leaf.minY + leaf.maxY, cZ = leaf.minZ + leaf.maxZ;
    let sibling = this.root;
    while (sibling.height > 0) {
      const c1 = sibling.child0, c2 = sibling.child1;
      const d1X = c1.minX + c1.maxX - cX, d1Y = c1.minY + c1.maxY - cY, d1Z = c1.minZ + c1.maxZ - cZ;
      const d2X = c2.minX + c2.maxX - cX, d2Y = c2.minY + c2.maxY - cY, d2Z = c2.minZ + c2.maxZ - cZ;
      const dist1 = d1X * d1X + d1Y * d1Y + d1Z * d1Z;
      const dist2 = d2X * d2X + d2Y * d2Y + d2Z * d2Z;
      sibling = dist1 < dist2 ? c1 : c2;
    }
    const parent = sibling.parent;
    const node = this.pick();
    if (parent === null) {
      this.root = node;
    } else {
      parent.setChild(sibling.childIndex, node);
    }
    node.setChild(sibling.childIndex, sibling);
    node.setChild(sibling.childIndex ^ 1, leaf);
    this.fixUpward(node);
  }
  deleteLeaf(leaf) {
    if (this.root === leaf) {
      this.root = null;
      return;
    }
    const parent = leaf.parent;
    const sibling = leaf.childIndex === 0 ? parent.child1 : parent.child0;
    const grandParent = parent.parent;
    if (grandParent === null) {
      sibling.parent = null;
      sibling.childIndex = 0;
      this.root = sibling;
      this.pool(parent);
      return;
    }
    sibling.parent = grandParent;
    grandParent.setChild(parent.childIndex, sibling);
    this.pool(parent);
    this.fixUpward(grandParent);
  }
  /** 从 node 往上逐层更新高度和包围盒 */
  fixUpward(start) {
    for (let node = start; node !== null; node = node.parent) {
      const c1 = node.child0, c2 = node.child1;
      const h1 = c1.height, h2 = c2.height;
      node.height = (h1 > h2 ? h1 : h2) + 1;
      node.minX = c1.minX < c2.minX ? c1.minX : c2.minX;
      node.minY = c1.minY < c2.minY ? c1.minY : c2.minY;
      node.minZ = c1.minZ < c2.minZ ? c1.minZ : c2.minZ;
      node.maxX = c1.maxX > c2.maxX ? c1.maxX : c2.maxX;
      node.maxY = c1.maxY > c2.maxY ? c1.maxY : c2.maxY;
      node.maxZ = c1.maxZ > c2.maxZ ? c1.maxZ : c2.maxZ;
    }
  }
  pool(node) {
    node.childIndex = 0;
    node.child0 = null;
    node.child1 = null;
    node.parent = null;
    node.height = 0;
    node.proxy = null;
    node.next = this.nodePool;
    this.nodePool = node;
  }
  pick() {
    const node = this.nodePool;
    if (node === null) return new BvhNode();
    this.nodePool = node.next;
    node.next = null;
    return node;
  }
};

// src/core/collision/broadphase/bvh-broad-phase.ts
var BvhBroadPhase = class extends BroadPhase {
  constructor() {
    super(true);
    this.tree = new BvhTree();
    this.movedProxies = [];
    this.numMovedProxies = 0;
  }
  createProxy(userData, aabb) {
    const p = new BvhProxy(userData, this.idCount++);
    this.addProxy(p);
    p.minX = aabb.minX - BVH_PROXY_PADDING;
    p.minY = aabb.minY - BVH_PROXY_PADDING;
    p.minZ = aabb.minZ - BVH_PROXY_PADDING;
    p.maxX = aabb.maxX + BVH_PROXY_PADDING;
    p.maxY = aabb.maxY + BVH_PROXY_PADDING;
    p.maxZ = aabb.maxZ + BVH_PROXY_PADDING;
    this.tree.insertProxy(p);
    this.addToMovedProxies(p);
    return p;
  }
  destroyProxy(proxy) {
    this.removeProxy(proxy);
    const p = proxy;
    this.tree.deleteProxy(p);
    p.userData = null;
    p.next = null;
    p.prev = null;
    p.moved = false;
  }
  moveProxy(proxy, aabb, dx, dy, dz) {
    const p = proxy;
    if (p.minX <= aabb.minX && p.maxX >= aabb.maxX && p.minY <= aabb.minY && p.maxY >= aabb.maxY && p.minZ <= aabb.minZ && p.maxZ >= aabb.maxZ) {
      return;
    }
    p.minX = aabb.minX - BVH_PROXY_PADDING;
    p.minY = aabb.minY - BVH_PROXY_PADDING;
    p.minZ = aabb.minZ - BVH_PROXY_PADDING;
    p.maxX = aabb.maxX + BVH_PROXY_PADDING;
    p.maxY = aabb.maxY + BVH_PROXY_PADDING;
    p.maxZ = aabb.maxZ + BVH_PROXY_PADDING;
    p.minX += 0 < dx ? 0 : dx;
    p.minY += 0 < dy ? 0 : dy;
    p.minZ += 0 < dz ? 0 : dz;
    p.maxX += 0 > dx ? 0 : dx;
    p.maxY += 0 > dy ? 0 : dy;
    p.maxZ += 0 > dz ? 0 : dz;
    this.addToMovedProxies(p);
  }
  collectPairs() {
    this.poolProxyPairs();
    this.testCount = 0;
    if (this.numProxies < 2) {
      if (!compat.keepOimoBugs) this.reinsertMovedProxies();
      return;
    }
    const tree = this.tree;
    const incremental = this.numMovedProxies / this.numProxies < BVH_INCREMENTAL_COLLISION_THRESHOLD;
    for (let i = 0; i < this.numMovedProxies; i++) {
      const p = this.movedProxies[i];
      if (p.moved) {
        tree.deleteProxy(p);
        tree.insertProxy(p);
        if (incremental) this.collide(tree.root, p.leaf);
        p.moved = false;
      }
      this.movedProxies[i] = null;
    }
    if (!incremental) this.collide(tree.root, tree.root);
    this.numMovedProxies = 0;
  }
  /** 只有一个代理时用：把移动过的代理在树里更新位置，不找重叠对 */
  reinsertMovedProxies() {
    const tree = this.tree;
    for (let i = 0; i < this.numMovedProxies; i++) {
      const p = this.movedProxies[i];
      if (p.moved) {
        tree.deleteProxy(p);
        tree.insertProxy(p);
        p.moved = false;
      }
      this.movedProxies[i] = null;
    }
    this.numMovedProxies = 0;
  }
  rayCast(beginX, beginY, beginZ, endX, endY, endZ, callback) {
    if (this.tree.root === null) return;
    this.rayCastRecursive(this.tree.root, beginX, beginY, beginZ, endX, endY, endZ, callback);
  }
  aabbTest(aabb, callback) {
    if (this.tree.root === null) return;
    this.aabbTestRecursive(this.tree.root, aabb, callback);
  }
  convexCast(convex, begin, translation, callback) {
    if (this.tree.root === null) return;
    this.convexCastRecursive(this.tree.root, convex, begin, translation, callback);
  }
  convexCastRecursive(node, convex, begin, translation, callback) {
    if (!this.aabbConvexSweepTest(node.minX, node.minY, node.minZ, node.maxX, node.maxY, node.maxZ, convex, begin, translation)) return;
    if (node.height === 0) {
      callback.process(node.proxy);
      return;
    }
    this.convexCastRecursive(node.child0, convex, begin, translation, callback);
    this.convexCastRecursive(node.child1, convex, begin, translation, callback);
  }
  rayCastRecursive(node, x1, y1, z1, x2, y2, z2, callback) {
    if (node.minX > (x1 > x2 ? x1 : x2) || node.maxX < (x1 < x2 ? x1 : x2) || node.minY > (y1 > y2 ? y1 : y2) || node.maxY < (y1 < y2 ? y1 : y2) || node.minZ > (z1 > z2 ? z1 : z2) || node.maxZ < (z1 < z2 ? z1 : z2)) {
      return;
    }
    const dX = x2 - x1;
    const dY = y2 - y1;
    const dZ = z2 - z1;
    const adX = dX < 0 ? -dX : dX;
    const adY = dY < 0 ? -dY : dY;
    const adZ = dZ < 0 ? -dZ : dZ;
    const extX = (node.maxX - node.minX) * 0.5;
    const extY = (node.maxY - node.minY) * 0.5;
    const extZ = (node.maxZ - node.minZ) * 0.5;
    const cpX = x1 - (node.maxX + node.minX) * 0.5;
    const cpY = y1 - (node.maxY + node.minY) * 0.5;
    const cpZ = z1 - (node.maxZ + node.minZ) * 0.5;
    let s = cpY * dZ - cpZ * dY;
    if ((s < 0 ? -s : s) - (extY * adZ + extZ * adY) > 0) return;
    s = cpZ * dX - cpX * dZ;
    if ((s < 0 ? -s : s) - (extZ * adX + extX * adZ) > 0) return;
    s = cpX * dY - cpY * dX;
    if ((s < 0 ? -s : s) - (extX * adY + extY * adX) > 0) return;
    if (node.height === 0) {
      callback.process(node.proxy);
      return;
    }
    this.rayCastRecursive(node.child0, x1, y1, z1, x2, y2, z2, callback);
    this.rayCastRecursive(node.child1, x1, y1, z1, x2, y2, z2, callback);
  }
  aabbTestRecursive(node, aabb, callback) {
    if (!(node.minX < aabb.maxX && node.maxX > aabb.minX && node.minY < aabb.maxY && node.maxY > aabb.minY && node.minZ < aabb.maxZ && node.maxZ > aabb.minZ)) {
      return;
    }
    if (node.height === 0) {
      callback.process(node.proxy);
      return;
    }
    this.aabbTestRecursive(node.child0, aabb, callback);
    this.aabbTestRecursive(node.child1, aabb, callback);
  }
  addToMovedProxies(p) {
    if (p.moved) return;
    p.moved = true;
    this.movedProxies[this.numMovedProxies++] = p;
  }
  /** 递归找出 n1、n2 两棵子树之间包围盒重叠的叶子对 */
  collide(n1, n2) {
    this.testCount++;
    const l1 = n1.height === 0;
    const l2 = n2.height === 0;
    if (n1 === n2) {
      if (l1) return;
      this.collide(n1.child0, n2);
      this.collide(n1.child1, n2);
      return;
    }
    if (!(n1.minX < n2.maxX && n1.maxX > n2.minX && n1.minY < n2.maxY && n1.maxY > n2.minY && n1.minZ < n2.maxZ && n1.maxZ > n2.minZ)) {
      return;
    }
    if (l1 && l2) {
      this.pickAndPushProxyPair(n1.proxy, n2.proxy);
      return;
    }
    if (l2 || n1.height > n2.height) {
      this.collide(n1.child0, n2);
      this.collide(n1.child1, n2);
    } else {
      this.collide(n2.child0, n1);
      this.collide(n2.child1, n1);
    }
  }
};

// src/core/dynamics/constraint/solver/soft-contact-solver.ts
var Softness = class {
  constructor() {
    this.biasRate = 0;
    this.massScale = 1;
    this.impulseScale = 0;
  }
  /** 频率 hertz（Hz）、阻尼比 zeta、步长 h（秒） */
  set(hertz, zeta, h) {
    if (hertz === 0) {
      this.biasRate = 0;
      this.massScale = 1;
      this.impulseScale = 0;
      return;
    }
    const omega = 2 * Math.PI * hertz;
    const a1 = 2 * zeta + h * omega;
    const a2 = h * omega * a1;
    const a3 = 1 / (1 + a2);
    this.biasRate = omega / a1;
    this.massScale = a2 * a3;
    this.impulseScale = a3;
  }
};
var SoftPoint = class {
  constructor() {
    /** 力臂：接触点相对两个刚体原点的位置（世界方向） */
    this.r1X = 0;
    this.r1Y = 0;
    this.r1Z = 0;
    this.r2X = 0;
    this.r2Y = 0;
    this.r2Z = 0;
    /** 逆惯性张量 × (r × 方向)：施加单位冲量时的角速度变化。N 法向、T 切向、B 副切向 */
    this.iaN1X = 0;
    this.iaN1Y = 0;
    this.iaN1Z = 0;
    this.iaN2X = 0;
    this.iaN2Y = 0;
    this.iaN2Z = 0;
    this.iaT1X = 0;
    this.iaT1Y = 0;
    this.iaT1Z = 0;
    this.iaT2X = 0;
    this.iaT2Y = 0;
    this.iaT2Z = 0;
    this.iaB1X = 0;
    this.iaB1Y = 0;
    this.iaB1Z = 0;
    this.iaB2X = 0;
    this.iaB2Y = 0;
    this.iaB2Z = 0;
    /** 单位法向冲量引起的切向相对速度变化。 */
    this.couplingT = 0;
    this.couplingB = 0;
    this.massN = 0;
    this.massT = 0;
    this.massB = 0;
    /** 这一步开始时的分离距离 */
    this.separation = 0;
    /** 最近一次求解使用的分离距离；位姿未变化时复用 */
    this.currentSeparation = 0;
    /** 分离距离的修正量：用当前位姿重算的 (p1 - p2)·n 加上它，等于检测器给出的分离距离 */
    this.offset = 0;
    /** 这一步开始时的法向相对速度（反弹用） */
    this.relativeVelocity = 0;
    /** 累积冲量：法向、切向、副切向 */
    this.impulseN = 0;
    this.impulseT = 0;
    this.impulseB = 0;
    /** 本步的最大法向冲量；未施加法向冲量的点不反弹。 */
    this.maxImpulseN = 0;
    /** 是否满足撞击速度及非持续接触条件。 */
    this.bounce = false;
  }
};
var SoftContactSolver = class {
  constructor(constraint) {
    this.points = [];
    this.numPoints = 0;
    this.friction = 0;
    this.restitution = 0;
    /** 任一侧为静态、运动学或休眠刚体时，使用更硬的软约束参数。 */
    this.fixed = false;
    /** 本步接触双方的逆质量。休眠刚体按静止对象求解，逆质量、逆惯量和速度视为 0。 */
    this.invM1 = 0;
    this.invM2 = 0;
    this.live1 = true;
    this.live2 = true;
    /** 子步模式是否按当前位姿重新计算分离距离。 */
    this.substepped = false;
    /** 本 tick 接触到被撞刚体，参与额外迭代。 */
    this.impacted = false;
    this.constraint = constraint;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.points.push(new SoftPoint());
  }
  /**
   * 准备力臂、有效质量、分离距离和热启动冲量，返回最大法向靠近速度。
   * impulseRatio 缩放热启动冲量；substepDt > 0 时改用本接触的历史步长比。
   * substepped 允许未施加过冲量的预测接触反弹，skin 为静态接触面的偏移。
   */
  prepare(impulseRatio = 1, substepped = false, skin = 0, substepDt = 0, bounceThreshold = CONTACT_ENABLE_BOUNCE_THRESHOLD) {
    this.substepped = substepped;
    const c = this.constraint, m = c.manifold, b1 = c.b1, b2 = c.b2;
    if (substepDt > 0 && c.substepDt > 0) impulseRatio = substepDt / c.substepDt;
    if (substepped && this.impacted) impulseRatio = 0;
    c.substepDt = substepDt;
    this.friction = Math.sqrt(c.s1.friction * c.s2.friction);
    if (b1.drag !== null || b2.drag !== null) this.friction *= 0.3;
    this.restitution = c.maxRestitution ? Math.max(c.s1.restitution, c.s2.restitution) : Math.sqrt(c.s1.restitution * c.s2.restitution);
    const surface = b1.invMass === 0 || b2.invMass === 0 ? skin : 0;
    const live1 = !b1.sleeping, live2 = !b2.sleeping;
    const invM1 = live1 ? b1.invMass : 0, invM2 = live2 ? b2.invMass : 0;
    this.invM1 = invM1;
    this.invM2 = invM2;
    this.live1 = live1;
    this.live2 = live2;
    this.fixed = invM1 === 0 || invM2 === 0;
    const nX = m.nX, nY = m.nY, nZ = m.nZ, tX = m.tX, tY = m.tY, tZ = m.tZ, bX = m.bX, bY = m.bY, bZ = m.bZ;
    let i100 = 0, i101 = 0, i102 = 0, i110 = 0, i111 = 0, i112 = 0, i120 = 0, i121 = 0, i122 = 0;
    let i200 = 0, i201 = 0, i202 = 0, i210 = 0, i211 = 0, i212 = 0, i220 = 0, i221 = 0, i222 = 0;
    let v1X = 0, v1Y = 0, v1Z = 0, w1X = 0, w1Y = 0, w1Z = 0;
    let v2X = 0, v2Y = 0, v2Z = 0, w2X = 0, w2Y = 0, w2Z = 0;
    if (live1) {
      i100 = b1.ii00;
      i101 = b1.ii01;
      i102 = b1.ii02;
      i110 = b1.ii10;
      i111 = b1.ii11;
      i112 = b1.ii12;
      i120 = b1.ii20;
      i121 = b1.ii21;
      i122 = b1.ii22;
      v1X = b1.vx;
      v1Y = b1.vy;
      v1Z = b1.vz;
      w1X = b1.wx;
      w1Y = b1.wy;
      w1Z = b1.wz;
    }
    if (live2) {
      i200 = b2.ii00;
      i201 = b2.ii01;
      i202 = b2.ii02;
      i210 = b2.ii10;
      i211 = b2.ii11;
      i212 = b2.ii12;
      i220 = b2.ii20;
      i221 = b2.ii21;
      i222 = b2.ii22;
      v2X = b2.vx;
      v2Y = b2.vy;
      v2Z = b2.vz;
      w2X = b2.wx;
      w2Y = b2.wy;
      w2Z = b2.wz;
    }
    const p1X = b1.transform.px, p1Y = b1.transform.py, p1Z = b1.transform.pz;
    const p2X = b2.transform.px, p2Y = b2.transform.py, p2Z = b2.transform.pz;
    let approach = 0;
    this.numPoints = m.numPoints;
    for (let i = 0; i < m.numPoints; i++) {
      const p = m.points[i], sp = this.points[i];
      p.disabled = false;
      if (!p.warmStarted) p.impulse.clear();
      const r1X = p.p1X - p1X, r1Y = p.p1Y - p1Y, r1Z = p.p1Z - p1Z;
      const r2X = p.p2X - p2X, r2Y = p.p2Y - p2Y, r2Z = p.p2Z - p2Z;
      sp.r1X = r1X;
      sp.r1Y = r1Y;
      sp.r1Z = r1Z;
      sp.r2X = r2X;
      sp.r2Y = r2Y;
      sp.r2Z = r2Z;
      let a1X = r1Y * nZ - r1Z * nY, a1Y = r1Z * nX - r1X * nZ, a1Z = r1X * nY - r1Y * nX;
      let a2X = r2Y * nZ - r2Z * nY, a2Y = r2Z * nX - r2X * nZ, a2Z = r2X * nY - r2Y * nX;
      sp.iaN1X = i100 * a1X + i101 * a1Y + i102 * a1Z;
      sp.iaN1Y = i110 * a1X + i111 * a1Y + i112 * a1Z;
      sp.iaN1Z = i120 * a1X + i121 * a1Y + i122 * a1Z;
      sp.iaN2X = i200 * a2X + i201 * a2Y + i202 * a2Z;
      sp.iaN2Y = i210 * a2X + i211 * a2Y + i212 * a2Z;
      sp.iaN2Z = i220 * a2X + i221 * a2Y + i222 * a2Z;
      let k = invM1 + invM2 + (a1X * sp.iaN1X + a1Y * sp.iaN1Y + a1Z * sp.iaN1Z) + (a2X * sp.iaN2X + a2Y * sp.iaN2Y + a2Z * sp.iaN2Z);
      sp.massN = k > 0 ? 1 / k : 0;
      a1X = r1Y * tZ - r1Z * tY;
      a1Y = r1Z * tX - r1X * tZ;
      a1Z = r1X * tY - r1Y * tX;
      a2X = r2Y * tZ - r2Z * tY;
      a2Y = r2Z * tX - r2X * tZ;
      a2Z = r2X * tY - r2Y * tX;
      sp.iaT1X = i100 * a1X + i101 * a1Y + i102 * a1Z;
      sp.iaT1Y = i110 * a1X + i111 * a1Y + i112 * a1Z;
      sp.iaT1Z = i120 * a1X + i121 * a1Y + i122 * a1Z;
      sp.iaT2X = i200 * a2X + i201 * a2Y + i202 * a2Z;
      sp.iaT2Y = i210 * a2X + i211 * a2Y + i212 * a2Z;
      sp.iaT2Z = i220 * a2X + i221 * a2Y + i222 * a2Z;
      k = invM1 + invM2 + (a1X * sp.iaT1X + a1Y * sp.iaT1Y + a1Z * sp.iaT1Z) + (a2X * sp.iaT2X + a2Y * sp.iaT2Y + a2Z * sp.iaT2Z);
      sp.massT = k > 0 ? 1 / k : 0;
      sp.couplingT = (invM1 + invM2) * (tX * nX + tY * nY + tZ * nZ) + a1X * sp.iaN1X + a1Y * sp.iaN1Y + a1Z * sp.iaN1Z + a2X * sp.iaN2X + a2Y * sp.iaN2Y + a2Z * sp.iaN2Z;
      a1X = r1Y * bZ - r1Z * bY;
      a1Y = r1Z * bX - r1X * bZ;
      a1Z = r1X * bY - r1Y * bX;
      a2X = r2Y * bZ - r2Z * bY;
      a2Y = r2Z * bX - r2X * bZ;
      a2Z = r2X * bY - r2Y * bX;
      sp.iaB1X = i100 * a1X + i101 * a1Y + i102 * a1Z;
      sp.iaB1Y = i110 * a1X + i111 * a1Y + i112 * a1Z;
      sp.iaB1Z = i120 * a1X + i121 * a1Y + i122 * a1Z;
      sp.iaB2X = i200 * a2X + i201 * a2Y + i202 * a2Z;
      sp.iaB2Y = i210 * a2X + i211 * a2Y + i212 * a2Z;
      sp.iaB2Z = i220 * a2X + i221 * a2Y + i222 * a2Z;
      k = invM1 + invM2 + (a1X * sp.iaB1X + a1Y * sp.iaB1Y + a1Z * sp.iaB1Z) + (a2X * sp.iaB2X + a2Y * sp.iaB2Y + a2Z * sp.iaB2Z);
      sp.massB = k > 0 ? 1 / k : 0;
      sp.couplingB = (invM1 + invM2) * (bX * nX + bY * nY + bZ * nZ) + a1X * sp.iaN1X + a1Y * sp.iaN1Y + a1Z * sp.iaN1Z + a2X * sp.iaN2X + a2Y * sp.iaN2Y + a2Z * sp.iaN2Z;
      sp.separation = -p.depth - surface;
      sp.offset = sp.separation - ((p.p1X - p.p2X) * nX + (p.p1Y - p.p2Y) * nY + (p.p1Z - p.p2Z) * nZ);
      const dvX = v1X + (w1Y * r1Z - w1Z * r1Y) - v2X - (w2Y * r2Z - w2Z * r2Y);
      const dvY = v1Y + (w1Z * r1X - w1X * r1Z) - v2Y - (w2Z * r2X - w2X * r2Z);
      const dvZ = v1Z + (w1X * r1Y - w1Y * r1X) - v2Z - (w2X * r2Y - w2Y * r2X);
      const vn = dvX * nX + dvY * nY + dvZ * nZ;
      sp.relativeVelocity = vn;
      if (-vn > approach) approach = -vn;
      const imp = p.impulse;
      sp.bounce = substepped ? vn <= -bounceThreshold : vn < -bounceThreshold && !p.warmStarted;
      sp.impulseN = imp.impulseN * impulseRatio;
      sp.impulseT = (imp.impulseLX * tX + imp.impulseLY * tY + imp.impulseLZ * tZ) * impulseRatio;
      sp.impulseB = (imp.impulseLX * bX + imp.impulseLY * bY + imp.impulseLZ * bZ) * impulseRatio;
      sp.maxImpulseN = 0;
    }
    c.approachSpeed = approach;
    return approach;
  }
  /** 热启动：把累积冲量施加上去。每一步都要做 */
  warmStart() {
    const c = this.constraint, m = c.manifold, b1 = c.b1, b2 = c.b2;
    const invM1 = this.invM1, invM2 = this.invM2, live1 = this.live1, live2 = this.live2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ, tX = m.tX, tY = m.tY, tZ = m.tZ, bX = m.bX, bY = m.bY, bZ = m.bZ;
    let v1X = 0, v1Y = 0, v1Z = 0, w1X = 0, w1Y = 0, w1Z = 0;
    let v2X = 0, v2Y = 0, v2Z = 0, w2X = 0, w2Y = 0, w2Z = 0;
    if (live1) {
      v1X = b1.vx;
      v1Y = b1.vy;
      v1Z = b1.vz;
      w1X = b1.wx;
      w1Y = b1.wy;
      w1Z = b1.wz;
    }
    if (live2) {
      v2X = b2.vx;
      v2Y = b2.vy;
      v2Z = b2.vz;
      w2X = b2.wx;
      w2Y = b2.wy;
      w2Z = b2.wz;
    }
    for (let i = 0; i < this.numPoints; i++) {
      const sp = this.points[i];
      const iN = sp.impulseN, iT = sp.impulseT, iB = sp.impulseB;
      if (iN === 0 && iT === 0 && iB === 0) continue;
      const pX = nX * iN + tX * iT + bX * iB, pY = nY * iN + tY * iT + bY * iB, pZ = nZ * iN + tZ * iT + bZ * iB;
      v1X += invM1 * pX;
      v1Y += invM1 * pY;
      v1Z += invM1 * pZ;
      v2X -= invM2 * pX;
      v2Y -= invM2 * pY;
      v2Z -= invM2 * pZ;
      w1X += sp.iaN1X * iN + sp.iaT1X * iT + sp.iaB1X * iB;
      w1Y += sp.iaN1Y * iN + sp.iaT1Y * iT + sp.iaB1Y * iB;
      w1Z += sp.iaN1Z * iN + sp.iaT1Z * iT + sp.iaB1Z * iB;
      w2X -= sp.iaN2X * iN + sp.iaT2X * iT + sp.iaB2X * iB;
      w2Y -= sp.iaN2Y * iN + sp.iaT2Y * iT + sp.iaB2Y * iB;
      w2Z -= sp.iaN2Z * iN + sp.iaT2Z * iT + sp.iaB2Z * iB;
    }
    if (live1) {
      b1.vx = v1X;
      b1.vy = v1Y;
      b1.vz = v1Z;
      b1.wx = w1X;
      b1.wy = w1Y;
      b1.wz = w1Z;
    }
    if (live2) {
      b2.vx = v2X;
      b2.vy = v2Y;
      b2.vz = v2Z;
      b2.wx = w2X;
      b2.wy = w2Y;
      b2.wz = w2Z;
    }
  }
  /**
   * 先求法向支撑，再求摩擦；返回冲量变化的绝对值之和。
   * useBias 启用穿透修正；关闭时仅阻止继续靠近。
   * reuseSeparation 仅在位姿未变时有效，reverse 控制接触点迭代方向。
   */
  solve(useBias, reverse, invH, dynamic, fixed, maxPushVelocity, reuseSeparation = false) {
    const c = this.constraint, m = c.manifold, b1 = c.b1, b2 = c.b2;
    const invM1 = this.invM1, invM2 = this.invM2, live1 = this.live1, live2 = this.live2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ, tX = m.tX, tY = m.tY, tZ = m.tZ, bX = m.bX, bY = m.bY, bZ = m.bZ;
    const soft = this.fixed ? fixed : dynamic;
    const biasRate = soft.biasRate, softMass = soft.massScale, softImpulse = soft.impulseScale;
    const n = this.numPoints;
    if (!reuseSeparation) {
      if (useBias && !this.substepped) {
        for (let i = 0; i < n; i++) this.points[i].currentSeparation = this.points[i].separation;
      } else {
        const tf1 = b1.transform, tf2 = b2.transform;
        const t1X = tf1.px, t1Y = tf1.py, t1Z = tf1.pz, t2X = tf2.px, t2Y = tf2.py, t2Z = tf2.pz;
        const a00 = tf1.r00, a01 = tf1.r01, a02 = tf1.r02, a10 = tf1.r10, a11 = tf1.r11, a12 = tf1.r12, a20 = tf1.r20, a21 = tf1.r21, a22 = tf1.r22;
        const c00 = tf2.r00, c01 = tf2.r01, c02 = tf2.r02, c10 = tf2.r10, c11 = tf2.r11, c12 = tf2.r12, c20 = tf2.r20, c21 = tf2.r21, c22 = tf2.r22;
        for (let i = 0; i < n; i++) {
          const p = m.points[i];
          const l1X = p.lp1X, l1Y = p.lp1Y, l1Z = p.lp1Z, l2X = p.lp2X, l2Y = p.lp2Y, l2Z = p.lp2Z;
          const p1X = t1X + a00 * l1X + a01 * l1Y + a02 * l1Z;
          const p1Y = t1Y + a10 * l1X + a11 * l1Y + a12 * l1Z;
          const p1Z = t1Z + a20 * l1X + a21 * l1Y + a22 * l1Z;
          const p2X = t2X + c00 * l2X + c01 * l2Y + c02 * l2Z;
          const p2Y = t2Y + c10 * l2X + c11 * l2Y + c12 * l2Z;
          const p2Z = t2Z + c20 * l2X + c21 * l2Y + c22 * l2Z;
          this.points[i].currentSeparation = (p1X - p2X) * nX + (p1Y - p2Y) * nY + (p1Z - p2Z) * nZ + this.points[i].offset;
        }
      }
    }
    let v1X = 0, v1Y = 0, v1Z = 0, w1X = 0, w1Y = 0, w1Z = 0;
    let v2X = 0, v2Y = 0, v2Z = 0, w2X = 0, w2Y = 0, w2Z = 0;
    if (live1) {
      v1X = b1.vx;
      v1Y = b1.vy;
      v1Z = b1.vz;
      w1X = b1.wx;
      w1Y = b1.wy;
      w1Z = b1.wz;
    }
    if (live2) {
      v2X = b2.vx;
      v2Y = b2.vy;
      v2Z = b2.vz;
      w2X = b2.wx;
      w2Y = b2.wy;
      w2Z = b2.wz;
    }
    const friction = this.friction;
    let error = 0;
    for (let k = 0; k < n; k++) {
      const sp = this.points[reverse ? n - 1 - k : k];
      const r1X = sp.r1X, r1Y = sp.r1Y, r1Z = sp.r1Z, r2X = sp.r2X, r2Y = sp.r2Y, r2Z = sp.r2Z;
      const s = sp.currentSeparation;
      let bias = 0, massScale = 1, impulseScale = 0;
      if (s > 0) {
        bias = s * invH;
      } else if (useBias && s < -(this.substepped ? 0 : LINEAR_SLOP)) {
        bias = biasRate * (s + (this.substepped ? 0 : LINEAR_SLOP));
        if (bias < -maxPushVelocity) bias = -maxPushVelocity;
        massScale = softMass;
        impulseScale = softImpulse;
      }
      const dvX = v1X + (w1Y * r1Z - w1Z * r1Y) - v2X - (w2Y * r2Z - w2Z * r2Y);
      const dvY = v1Y + (w1Z * r1X - w1X * r1Z) - v2Y - (w2Z * r2X - w2X * r2Z);
      const dvZ = v1Z + (w1X * r1Y - w1Y * r1X) - v2Z - (w2X * r2Y - w2Y * r2X);
      const vn = dvX * nX + dvY * nY + dvZ * nZ;
      const old = sp.impulseN;
      let impulse = -sp.massN * massScale * (vn + bias) - impulseScale * old;
      const sum = old + impulse;
      const total = sum > 0 ? sum : 0;
      impulse = total - old;
      sp.impulseN = total;
      if (this.substepped && impulse > 0 && -vn > c.approachSpeed) c.approachSpeed = -vn;
      if (total > sp.maxImpulseN) sp.maxImpulseN = total;
      let iT = 0, iB = 0;
      if (total !== 0 || sp.impulseT !== 0 || sp.impulseB !== 0) {
        let newT = sp.impulseT - sp.massT * (dvX * tX + dvY * tY + dvZ * tZ + sp.couplingT * impulse);
        let newB = sp.impulseB - sp.massB * (dvX * bX + dvY * bY + dvZ * bZ + sp.couplingB * impulse);
        const limit = friction * total, lenSq = newT * newT + newB * newB;
        if (lenSq > limit * limit) {
          const scale = limit / Math.sqrt(lenSq);
          newT *= scale;
          newB *= scale;
        }
        iT = newT - sp.impulseT;
        iB = newB - sp.impulseB;
        sp.impulseT = newT;
        sp.impulseB = newB;
      }
      error += Math.abs(impulse) + Math.abs(iT) + Math.abs(iB);
      if (impulse === 0 && iT === 0 && iB === 0) continue;
      const pX = nX * impulse + tX * iT + bX * iB;
      const pY = nY * impulse + tY * iT + bY * iB;
      const pZ = nZ * impulse + tZ * iT + bZ * iB;
      v1X += invM1 * pX;
      v1Y += invM1 * pY;
      v1Z += invM1 * pZ;
      v2X -= invM2 * pX;
      v2Y -= invM2 * pY;
      v2Z -= invM2 * pZ;
      w1X += sp.iaN1X * impulse + sp.iaT1X * iT + sp.iaB1X * iB;
      w1Y += sp.iaN1Y * impulse + sp.iaT1Y * iT + sp.iaB1Y * iB;
      w1Z += sp.iaN1Z * impulse + sp.iaT1Z * iT + sp.iaB1Z * iB;
      w2X -= sp.iaN2X * impulse + sp.iaT2X * iT + sp.iaB2X * iB;
      w2Y -= sp.iaN2Y * impulse + sp.iaT2Y * iT + sp.iaB2Y * iB;
      w2Z -= sp.iaN2Z * impulse + sp.iaT2Z * iT + sp.iaB2Z * iB;
    }
    if (live1) {
      b1.vx = v1X;
      b1.vy = v1Y;
      b1.vz = v1Z;
      b1.wx = w1X;
      b1.wy = w1Y;
      b1.wz = w1Z;
    }
    if (live2) {
      b2.vx = v2X;
      b2.vy = v2Y;
      b2.vz = v2Z;
      b2.wx = w2X;
      b2.wy = w2Y;
      b2.wz = w2Z;
    }
    return error;
  }
  /** 有没有要反弹的点（applyRestitution 会处理的点） */
  wantsRestitution() {
    if (this.restitution === 0) return false;
    for (let i = 0; i < this.numPoints; i++) {
      const sp = this.points[i];
      if (sp.bounce && sp.maxImpulseN !== 0) return true;
    }
    return false;
  }
  /** 反弹：求解完之后，让法向速度变成撞击时的 -restitution 倍。只处理真的推过的点 */
  applyRestitution() {
    if (this.restitution === 0) return;
    const c = this.constraint, m = c.manifold, b1 = c.b1, b2 = c.b2;
    const invM1 = this.invM1, invM2 = this.invM2, live1 = this.live1, live2 = this.live2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ;
    const e = this.restitution;
    let v1X = 0, v1Y = 0, v1Z = 0, w1X = 0, w1Y = 0, w1Z = 0;
    let v2X = 0, v2Y = 0, v2Z = 0, w2X = 0, w2Y = 0, w2Z = 0;
    if (live1) {
      v1X = b1.vx;
      v1Y = b1.vy;
      v1Z = b1.vz;
      w1X = b1.wx;
      w1Y = b1.wy;
      w1Z = b1.wz;
    }
    if (live2) {
      v2X = b2.vx;
      v2Y = b2.vy;
      v2Z = b2.vz;
      w2X = b2.wx;
      w2Y = b2.wy;
      w2Z = b2.wz;
    }
    for (let i = 0; i < this.numPoints; i++) {
      const sp = this.points[i];
      if (!sp.bounce || sp.maxImpulseN === 0) continue;
      const r1X = sp.r1X, r1Y = sp.r1Y, r1Z = sp.r1Z, r2X = sp.r2X, r2Y = sp.r2Y, r2Z = sp.r2Z;
      const dvX = v1X + (w1Y * r1Z - w1Z * r1Y) - v2X - (w2Y * r2Z - w2Z * r2Y);
      const dvY = v1Y + (w1Z * r1X - w1X * r1Z) - v2Y - (w2Z * r2X - w2X * r2Z);
      const dvZ = v1Z + (w1X * r1Y - w1Y * r1X) - v2Z - (w2X * r2Y - w2Y * r2X);
      const vn = dvX * nX + dvY * nY + dvZ * nZ;
      let impulse = -sp.massN * (vn + e * sp.relativeVelocity);
      const total = sp.impulseN + impulse > 0 ? sp.impulseN + impulse : 0;
      impulse = total - sp.impulseN;
      sp.impulseN = total;
      if (impulse === 0) continue;
      v1X += invM1 * nX * impulse;
      v1Y += invM1 * nY * impulse;
      v1Z += invM1 * nZ * impulse;
      v2X -= invM2 * nX * impulse;
      v2Y -= invM2 * nY * impulse;
      v2Z -= invM2 * nZ * impulse;
      w1X += sp.iaN1X * impulse;
      w1Y += sp.iaN1Y * impulse;
      w1Z += sp.iaN1Z * impulse;
      w2X -= sp.iaN2X * impulse;
      w2Y -= sp.iaN2Y * impulse;
      w2Z -= sp.iaN2Z * impulse;
    }
    if (live1) {
      b1.vx = v1X;
      b1.vy = v1Y;
      b1.vz = v1Z;
      b1.wx = w1X;
      b1.wy = w1Y;
      b1.wz = w1Z;
    }
    if (live2) {
      b2.vx = v2X;
      b2.vy = v2Y;
      b2.vz = v2Z;
      b2.wx = w2X;
      b2.wy = w2Y;
      b2.wz = w2Z;
    }
  }
  /**
   * 保存累积冲量并同步接触点。
   * stamp 记录步进编号和法向冲量状态，供子步模式在求解后判断接触。
   */
  store(stamp = -1) {
    const c = this.constraint, m = c.manifold;
    const tX = m.tX, tY = m.tY, tZ = m.tZ, bX = m.bX, bY = m.bY, bZ = m.bZ;
    let pushed = false;
    for (let i = 0; i < this.numPoints; i++) {
      const sp = this.points[i], imp = m.points[i].impulse;
      imp.impulseN = sp.impulseN;
      imp.impulseT = sp.impulseT;
      imp.impulseB = sp.impulseB;
      imp.impulseLX = tX * sp.impulseT + bX * sp.impulseB;
      imp.impulseLY = tY * sp.impulseT + bY * sp.impulseB;
      imp.impulseLZ = tZ * sp.impulseT + bZ * sp.impulseB;
      if (sp.maxImpulseN > 0) pushed = true;
    }
    if (stamp >= 0) {
      c.solvedStamp = stamp;
      c.pushed = pushed;
    }
    c.syncManifold();
  }
};

// src/core/collision/narrowphase/detector.ts
var Detector = class {
  constructor(swapped) {
    this.swapped = swapped;
  }
  /**
   * 计算接触流形并写入 result，cachedData 保存 GJK/EPA 的跨步缓存。
   * margin 默认为 0；分离距离小于 margin 时生成负深度的预测接触点。
   */
  detect(result, geom1, geom2, tf1, tf2, cachedData, margin = 0) {
    result.clear();
    if (this.swapped) {
      this.detectImpl(result, geom2, geom1, tf2, tf1, cachedData, margin);
    } else {
      this.detectImpl(result, geom1, geom2, tf1, tf2, cachedData, margin);
    }
  }
  setNormal(result, nx, ny, nz) {
    if (this.swapped) {
      result.nx = -nx;
      result.ny = -ny;
      result.nz = -nz;
    } else {
      result.nx = nx;
      result.ny = ny;
      result.nz = nz;
    }
  }
  addPoint(result, x1, y1, z1, x2, y2, z2, depth, id) {
    const p = result.points[result.numPoints++];
    p.depth = depth;
    p.id = id;
    if (this.swapped) {
      p.p1x = x2;
      p.p1y = y2;
      p.p1z = z2;
      p.p2x = x1;
      p.p2y = y1;
      p.p2z = z1;
    } else {
      p.p1x = x1;
      p.p1y = y1;
      p.p1z = z1;
      p.p2x = x2;
      p.p2y = y2;
      p.p2z = z2;
    }
  }
};

// src/core/collision/narrowphase/face-clipper.ts
var IncidentVertex = class {
  constructor() {
    this.x = 0;
    this.y = 0;
    this.wx = 0;
    this.wy = 0;
    this.wz = 0;
  }
};
function copyVertex(o, v) {
  o.x = v.x;
  o.y = v.y;
  o.wx = v.wx;
  o.wy = v.wy;
  o.wz = v.wz;
}
var FaceClipper = class {
  constructor() {
    this.w = 0;
    this.h = 0;
    this.numVertices = 0;
    this.vertices = [];
    this.tmpVertices = [];
    for (let i = 0; i < 8; i++) {
      this.vertices.push(new IncidentVertex());
      this.tmpVertices.push(new IncidentVertex());
    }
  }
  init(w, h) {
    this.w = w;
    this.h = h;
    this.numVertices = 0;
  }
  addIncidentVertex(x, y, wx, wy, wz) {
    const v = this.vertices[this.numVertices++];
    v.x = x;
    v.y = y;
    v.wx = wx;
    v.wy = wy;
    v.wz = wz;
  }
  /** 依次用参考面的左、右、上、下四条边裁剪，最多得到 8 个点 */
  clip() {
    let inside2 = true;
    for (let i = 0; i < this.numVertices; i++) {
      const v = this.vertices[i];
      if (!(this.w + v.x > 0 && this.w - v.x > 0 && this.h + v.y > 0 && this.h - v.y > 0)) {
        inside2 = false;
        break;
      }
    }
    if (inside2) return;
    this.clipPass(this.w, false, false);
    this.clipPass(this.w, false, true);
    this.clipPass(this.h, true, false);
    this.clipPass(this.h, true, true);
  }
  /** 超过 4 个点时保留两条对角线方向的极值点。 */
  reduce() {
    if (this.numVertices < 4) return;
    const src = this.vertices;
    const first = src[0];
    let max1 = first.x + first.y, min1 = max1;
    let max2 = first.y - first.x, min2 = max2;
    let max1V = first, min1V = first, max2V = first, min2V = first;
    for (let i = 1; i < this.numVertices; i++) {
      const v = src[i];
      const dot1 = v.x + v.y;
      const dot2 = v.y - v.x;
      if (dot1 > max1) {
        max1 = dot1;
        max1V = v;
      }
      if (dot1 < min1) {
        min1 = dot1;
        min1V = v;
      }
      if (dot2 > max2) {
        max2 = dot2;
        max2V = v;
      }
      if (dot2 < min2) {
        min2 = dot2;
        min2V = v;
      }
    }
    const dst = this.tmpVertices;
    copyVertex(dst[0], max1V);
    copyVertex(dst[1], max2V);
    copyVertex(dst[2], min1V);
    copyVertex(dst[3], min2V);
    this.vertices = dst;
    this.tmpVertices = src;
    this.numVertices = 4;
  }
  /**
   * 用一条边裁剪一遍，结果写进另一个缓冲区再交换。
   * 顶点到边的有向距离是 negate ? bound - c : bound + c（c 是顶点的 x 或 y），正数表示在内侧。
   */
  clipPass(bound, useY, negate) {
    const src = this.vertices, dst = this.tmpVertices, n = this.numVertices;
    let m = 0;
    let o, t;
    for (let i = 0; i < n; i++) {
      const v1 = src[i], v2 = src[(i + 1) % n];
      const c1 = useY ? v1.y : v1.x, c2 = useY ? v2.y : v2.x;
      const s1 = negate ? bound - c1 : bound + c1;
      const s2 = negate ? bound - c2 : bound + c2;
      if (s1 > 0 && s2 > 0) {
        o = dst[m++];
        o.x = v1.x;
        o.y = v1.y;
        o.wx = v1.wx;
        o.wy = v1.wy;
        o.wz = v1.wz;
      } else if (s1 > 0 && s2 <= 0) {
        o = dst[m++];
        o.x = v1.x;
        o.y = v1.y;
        o.wx = v1.wx;
        o.wy = v1.wy;
        o.wz = v1.wz;
        t = s1 / (s1 - s2);
        o = dst[m++];
        o.x = v1.x + (v2.x - v1.x) * t;
        o.y = v1.y + (v2.y - v1.y) * t;
        o.wx = v1.wx + (v2.wx - v1.wx) * t;
        o.wy = v1.wy + (v2.wy - v1.wy) * t;
        o.wz = v1.wz + (v2.wz - v1.wz) * t;
      } else if (s1 <= 0 && s2 > 0) {
        t = s1 / (s1 - s2);
        o = dst[m++];
        o.x = v1.x + (v2.x - v1.x) * t;
        o.y = v1.y + (v2.y - v1.y) * t;
        o.wx = v1.wx + (v2.wx - v1.wx) * t;
        o.wy = v1.wy + (v2.wy - v1.wy) * t;
        o.wz = v1.wz + (v2.wz - v1.wz) * t;
      }
    }
    this.vertices = dst;
    this.tmpVertices = src;
    this.numVertices = m;
  }
};

// src/core/collision/narrowphase/box-box-detector.ts
var BoxBoxDetector = class extends Detector {
  constructor() {
    super(false);
    this.clipper = new FaceClipper();
  }
  detectImpl(result, geom1, geom2, tf1, tf2, _cachedData, margin) {
    result.incremental = false;
    const b1 = geom1;
    const b2 = geom2;
    let c1X = tf1.px, c1Y = tf1.py, c1Z = tf1.pz;
    const c2X = tf2.px, c2Y = tf2.py, c2Z = tf2.pz;
    let c12X = c2X - c1X, c12Y = c2Y - c1Y, c12Z = c2Z - c1Z;
    let x1X = tf1.r00, x1Y = tf1.r10, x1Z = tf1.r20;
    let y1X = tf1.r01, y1Y = tf1.r11, y1Z = tf1.r21;
    let z1X = tf1.r02, z1Y = tf1.r12, z1Z = tf1.r22;
    let x2X = tf2.r00, x2Y = tf2.r10, x2Z = tf2.r20;
    let y2X = tf2.r01, y2Y = tf2.r11, y2Z = tf2.r21;
    let z2X = tf2.r02, z2Y = tf2.r12, z2Z = tf2.r22;
    let w1 = b1.hx, h1 = b1.hy, d1 = b1.hz;
    const w2 = b2.hx, h2 = b2.hy, d2 = b2.hz;
    let sx1X = x1X * w1, sx1Y = x1Y * w1, sx1Z = x1Z * w1;
    let sy1X = y1X * h1, sy1Y = y1Y * h1, sy1Z = y1Z * h1;
    let sz1X = z1X * d1, sz1Y = z1Y * d1, sz1Z = z1Z * d1;
    let sx2X = x2X * w2, sx2Y = x2Y * w2, sx2Z = x2Z * w2;
    let sy2X = y2X * h2, sy2Y = y2Y * h2, sy2Z = y2Z * h2;
    let sz2X = z2X * d2, sz2Y = z2Y * d2, sz2Z = z2Z * d2;
    let mDepth = Infinity;
    let mId = -1;
    let mSign = 0;
    let mAxisX = 0, mAxisY = 0, mAxisZ = 0;
    let proj1, proj2, projC12, sum, abs, depth;
    let neg;
    let dx, dy, dz;
    let eX, eY, eZ, l;
    dx = x1X * sx2X + x1Y * sx2Y + x1Z * sx2Z;
    if (dx < 0) dx = -dx;
    dy = x1X * sy2X + x1Y * sy2Y + x1Z * sy2Z;
    if (dy < 0) dy = -dy;
    dz = x1X * sz2X + x1Y * sz2Y + x1Z * sz2Z;
    if (dz < 0) dz = -dz;
    proj1 = w1;
    proj2 = dx + dy + dz;
    projC12 = x1X * c12X + x1Y * c12Y + x1Z * c12Z;
    sum = proj1 + proj2;
    neg = projC12 < 0;
    abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) {
      mDepth = depth;
      mId = 0;
      mAxisX = x1X;
      mAxisY = x1Y;
      mAxisZ = x1Z;
      mSign = neg ? -1 : 1;
    }
    dx = y1X * sx2X + y1Y * sx2Y + y1Z * sx2Z;
    if (dx < 0) dx = -dx;
    dy = y1X * sy2X + y1Y * sy2Y + y1Z * sy2Z;
    if (dy < 0) dy = -dy;
    dz = y1X * sz2X + y1Y * sz2Y + y1Z * sz2Z;
    if (dz < 0) dz = -dz;
    proj1 = h1;
    proj2 = dx + dy + dz;
    projC12 = y1X * c12X + y1Y * c12Y + y1Z * c12Z;
    sum = proj1 + proj2;
    neg = projC12 < 0;
    abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) {
      mDepth = depth;
      mId = 1;
      mAxisX = y1X;
      mAxisY = y1Y;
      mAxisZ = y1Z;
      mSign = neg ? -1 : 1;
    }
    dx = z1X * sx2X + z1Y * sx2Y + z1Z * sx2Z;
    if (dx < 0) dx = -dx;
    dy = z1X * sy2X + z1Y * sy2Y + z1Z * sy2Z;
    if (dy < 0) dy = -dy;
    dz = z1X * sz2X + z1Y * sz2Y + z1Z * sz2Z;
    if (dz < 0) dz = -dz;
    proj1 = d1;
    proj2 = dx + dy + dz;
    projC12 = z1X * c12X + z1Y * c12Y + z1Z * c12Z;
    sum = proj1 + proj2;
    neg = projC12 < 0;
    abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) {
      mDepth = depth;
      mId = 2;
      mAxisX = z1X;
      mAxisY = z1Y;
      mAxisZ = z1Z;
      mSign = neg ? -1 : 1;
    }
    if (mDepth > LINEAR_SLOP) mDepth -= LINEAR_SLOP;
    else if (mDepth < 0) mDepth -= LINEAR_SLOP;
    else mDepth = 0;
    dx = x2X * sx1X + x2Y * sx1Y + x2Z * sx1Z;
    if (dx < 0) dx = -dx;
    dy = x2X * sy1X + x2Y * sy1Y + x2Z * sy1Z;
    if (dy < 0) dy = -dy;
    dz = x2X * sz1X + x2Y * sz1Y + x2Z * sz1Z;
    if (dz < 0) dz = -dz;
    proj1 = dx + dy + dz;
    proj2 = w2;
    projC12 = x2X * c12X + x2Y * c12Y + x2Z * c12Z;
    sum = proj1 + proj2;
    neg = projC12 < 0;
    abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) {
      mDepth = depth;
      mId = 3;
      mAxisX = x2X;
      mAxisY = x2Y;
      mAxisZ = x2Z;
      mSign = neg ? -1 : 1;
    }
    dx = y2X * sx1X + y2Y * sx1Y + y2Z * sx1Z;
    if (dx < 0) dx = -dx;
    dy = y2X * sy1X + y2Y * sy1Y + y2Z * sy1Z;
    if (dy < 0) dy = -dy;
    dz = y2X * sz1X + y2Y * sz1Y + y2Z * sz1Z;
    if (dz < 0) dz = -dz;
    proj1 = dx + dy + dz;
    proj2 = h2;
    projC12 = y2X * c12X + y2Y * c12Y + y2Z * c12Z;
    sum = proj1 + proj2;
    neg = projC12 < 0;
    abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) {
      mDepth = depth;
      mId = 4;
      mAxisX = y2X;
      mAxisY = y2Y;
      mAxisZ = y2Z;
      mSign = neg ? -1 : 1;
    }
    dx = z2X * sx1X + z2Y * sx1Y + z2Z * sx1Z;
    if (dx < 0) dx = -dx;
    dy = z2X * sy1X + z2Y * sy1Y + z2Z * sy1Z;
    if (dy < 0) dy = -dy;
    dz = z2X * sz1X + z2Y * sz1Y + z2Z * sz1Z;
    if (dz < 0) dz = -dz;
    proj1 = dx + dy + dz;
    proj2 = d2;
    projC12 = z2X * c12X + z2Y * c12Y + z2Z * c12Z;
    sum = proj1 + proj2;
    neg = projC12 < 0;
    abs = neg ? -projC12 : projC12;
    if (!(abs < sum + margin)) return;
    depth = sum - abs;
    if (depth < mDepth) {
      mDepth = depth;
      mId = 5;
      mAxisX = z2X;
      mAxisY = z2Y;
      mAxisZ = z2Z;
      mSign = neg ? -1 : 1;
    }
    if (mDepth > LINEAR_SLOP) mDepth -= LINEAR_SLOP;
    else if (mDepth < 0) mDepth -= LINEAR_SLOP;
    else mDepth = 0;
    eX = x1Y * x2Z - x1Z * x2Y;
    eY = x1Z * x2X - x1X * x2Z;
    eZ = x1X * x2Y - x1Y * x2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sy1X + eY * sy1Y + eZ * sy1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sy2X + eY * sy2Y + eZ * sy2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 6;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    eX = x1Y * y2Z - x1Z * y2Y;
    eY = x1Z * y2X - x1X * y2Z;
    eZ = x1X * y2Y - x1Y * y2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sy1X + eY * sy1Y + eZ * sy1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 7;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    eX = x1Y * z2Z - x1Z * z2Y;
    eY = x1Z * z2X - x1X * z2Z;
    eZ = x1X * z2Y - x1Y * z2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sy1X + eY * sy1Y + eZ * sy1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sy2X + eY * sy2Y + eZ * sy2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 8;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    eX = y1Y * x2Z - y1Z * x2Y;
    eY = y1Z * x2X - y1X * x2Z;
    eZ = y1X * x2Y - y1Y * x2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sy2X + eY * sy2Y + eZ * sy2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 9;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    eX = y1Y * y2Z - y1Z * y2Y;
    eY = y1Z * y2X - y1X * y2Z;
    eZ = y1X * y2Y - y1Y * y2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 10;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    eX = y1Y * z2Z - y1Z * z2Y;
    eY = y1Z * z2X - y1X * z2Z;
    eZ = y1X * z2Y - y1Y * z2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz1X + eY * sz1Y + eZ * sz1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sy2X + eY * sy2Y + eZ * sy2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 11;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    eX = z1Y * x2Z - z1Z * x2Y;
    eY = z1Z * x2X - z1X * x2Z;
    eZ = z1X * x2Y - z1Y * x2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sy1X + eY * sy1Y + eZ * sy1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sy2X + eY * sy2Y + eZ * sy2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 12;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    eX = z1Y * y2Z - z1Z * y2Y;
    eY = z1Z * y2X - z1X * y2Z;
    eZ = z1X * y2Y - z1Y * y2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sy1X + eY * sy1Y + eZ * sy1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sz2X + eY * sz2Y + eZ * sz2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 13;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    eX = z1Y * z2Z - z1Z * z2Y;
    eY = z1Z * z2X - z1X * z2Z;
    eZ = z1X * z2Y - z1Y * z2X;
    if (!(eX === 0 && eY === 0 && eZ === 0)) {
      l = eX * eX + eY * eY + eZ * eZ;
      if (l > 0) l = 1 / Math.sqrt(l);
      eX *= l;
      eY *= l;
      eZ *= l;
      dx = eX * sx1X + eY * sx1Y + eZ * sx1Z;
      if (dx < 0) dx = -dx;
      dy = eX * sy1X + eY * sy1Y + eZ * sy1Z;
      if (dy < 0) dy = -dy;
      proj1 = dx + dy;
      dx = eX * sx2X + eY * sx2Y + eZ * sx2Z;
      if (dx < 0) dx = -dx;
      dy = eX * sy2X + eY * sy2Y + eZ * sy2Z;
      if (dy < 0) dy = -dy;
      proj2 = dx + dy;
      projC12 = eX * c12X + eY * c12Y + eZ * c12Z;
      sum = proj1 + proj2;
      neg = projC12 < 0;
      abs = neg ? -projC12 : projC12;
      if (!(abs < sum + margin)) return;
      depth = sum - abs;
      if (depth < mDepth) {
        mDepth = depth;
        mId = 14;
        mAxisX = eX;
        mAxisY = eY;
        mAxisZ = eZ;
        mSign = neg ? -1 : 1;
      }
    }
    if (mId >= 6) {
      mAxisX *= mSign;
      mAxisY *= mSign;
      mAxisZ *= mSign;
      const id1 = (mId - 6) / 3 | 0;
      const id2 = mId - 6 - id1 * 3;
      let e1X, e1Y, e1Z, a1X, a1Y, a1Z, b1X, b1Y, b1Z;
      if (id1 === 0) {
        e1X = x1X;
        e1Y = x1Y;
        e1Z = x1Z;
        a1X = sy1X;
        a1Y = sy1Y;
        a1Z = sy1Z;
        b1X = sz1X;
        b1Y = sz1Y;
        b1Z = sz1Z;
      } else if (id1 === 1) {
        e1X = y1X;
        e1Y = y1Y;
        e1Z = y1Z;
        a1X = sx1X;
        a1Y = sx1Y;
        a1Z = sx1Z;
        b1X = sz1X;
        b1Y = sz1Y;
        b1Z = sz1Z;
      } else {
        e1X = z1X;
        e1Y = z1Y;
        e1Z = z1Z;
        a1X = sx1X;
        a1Y = sx1Y;
        a1Z = sx1Z;
        b1X = sy1X;
        b1Y = sy1Y;
        b1Z = sy1Z;
      }
      let e2X, e2Y, e2Z, a2X, a2Y, a2Z, b2X, b2Y, b2Z;
      if (id2 === 0) {
        e2X = x2X;
        e2Y = x2Y;
        e2Z = x2Z;
        a2X = sy2X;
        a2Y = sy2Y;
        a2Z = sy2Z;
        b2X = sz2X;
        b2Y = sz2Y;
        b2Z = sz2Z;
      } else if (id2 === 1) {
        e2X = y2X;
        e2Y = y2Y;
        e2Z = y2Z;
        a2X = sx2X;
        a2Y = sx2Y;
        a2Z = sx2Z;
        b2X = sz2X;
        b2Y = sz2Y;
        b2Z = sz2Z;
      } else {
        e2X = z2X;
        e2Y = z2Y;
        e2Z = z2Z;
        a2X = sx2X;
        a2Y = sx2Y;
        a2Z = sx2Z;
        b2X = sy2X;
        b2Y = sy2Y;
        b2Z = sy2Z;
      }
      let sa = a1X * mAxisX + a1Y * mAxisY + a1Z * mAxisZ > 0;
      let sb = b1X * mAxisX + b1Y * mAxisY + b1Z * mAxisZ > 0;
      let p1X, p1Y, p1Z;
      if (sa) {
        if (sb) {
          p1X = a1X + b1X;
          p1Y = a1Y + b1Y;
          p1Z = a1Z + b1Z;
        } else {
          p1X = a1X - b1X;
          p1Y = a1Y - b1Y;
          p1Z = a1Z - b1Z;
        }
      } else if (sb) {
        p1X = b1X - a1X;
        p1Y = b1Y - a1Y;
        p1Z = b1Z - a1Z;
      } else {
        p1X = -(a1X + b1X);
        p1Y = -(a1Y + b1Y);
        p1Z = -(a1Z + b1Z);
      }
      p1X = c1X + p1X;
      p1Y = c1Y + p1Y;
      p1Z = c1Z + p1Z;
      sa = a2X * mAxisX + a2Y * mAxisY + a2Z * mAxisZ > 0;
      sb = b2X * mAxisX + b2Y * mAxisY + b2Z * mAxisZ > 0;
      let p2X, p2Y, p2Z;
      if (sa) {
        if (sb) {
          p2X = a2X + b2X;
          p2Y = a2Y + b2Y;
          p2Z = a2Z + b2Z;
        } else {
          p2X = a2X - b2X;
          p2Y = a2Y - b2Y;
          p2Z = a2Z - b2Z;
        }
      } else if (sb) {
        p2X = b2X - a2X;
        p2Y = b2Y - a2Y;
        p2Z = b2Z - a2Z;
      } else {
        p2X = -(a2X + b2X);
        p2Y = -(a2Y + b2Y);
        p2Z = -(a2Z + b2Z);
      }
      p2X = c2X - p2X;
      p2Y = c2Y - p2Y;
      p2Z = c2Z - p2Z;
      const rX = p1X - p2X, rY = p1Y - p2Y, rZ = p1Z - p2Z;
      const dot12 = e1X * e2X + e1Y * e2Y + e1Z * e2Z;
      const dot1r = e1X * rX + e1Y * rY + e1Z * rZ;
      const dot2r = e2X * rX + e2Y * rY + e2Z * rZ;
      const invDet = 1 / (1 - dot12 * dot12);
      const t1 = (dot12 * dot2r - dot1r) * invDet;
      const t2 = (dot2r - dot12 * dot1r) * invDet;
      this.setNormal(result, -mAxisX, -mAxisY, -mAxisZ);
      this.addPoint(result, p1X + e1X * t1, p1Y + e1Y * t1, p1Z + e1Z * t1, p2X + e2X * t2, p2Y + e2Y * t2, p2Z + e2Z * t2, mDepth, 4);
      return;
    }
    let t;
    const refOnBox2 = mId >= 3;
    if (refOnBox2) {
      mSign = -mSign;
      c12X = -c12X;
      c12Y = -c12Y;
      c12Z = -c12Z;
      w1 = w2;
      h1 = h2;
      d1 = d2;
      c1X = c2X;
      c1Y = c2Y;
      c1Z = c2Z;
      t = x1X;
      x1X = x2X;
      x2X = t;
      t = x1Y;
      x1Y = x2Y;
      x2Y = t;
      t = x1Z;
      x1Z = x2Z;
      x2Z = t;
      t = y1X;
      y1X = y2X;
      y2X = t;
      t = y1Y;
      y1Y = y2Y;
      y2Y = t;
      t = y1Z;
      y1Z = y2Z;
      y2Z = t;
      t = z1X;
      z1X = z2X;
      z2X = t;
      t = z1Y;
      z1Y = z2Y;
      z2Y = t;
      t = z1Z;
      z1Z = z2Z;
      z2Z = t;
      t = sx1X;
      sx1X = sx2X;
      sx2X = t;
      t = sx1Y;
      sx1Y = sx2Y;
      sx2Y = t;
      t = sx1Z;
      sx1Z = sx2Z;
      sx2Z = t;
      t = sy1X;
      sy1X = sy2X;
      sy2X = t;
      t = sy1Y;
      sy1Y = sy2Y;
      sy2Y = t;
      t = sy1Z;
      sy1Z = sy2Z;
      sy2Z = t;
      t = sz1X;
      sz1X = sz2X;
      sz2X = t;
      t = sz1Y;
      sz1Y = sz2Y;
      sz2Y = t;
      t = sz1Z;
      sz1Z = sz2Z;
      sz2Z = t;
      mId -= 3;
    }
    let refCX, refCY, refCZ, refNX, refNY, refNZ;
    let refXX, refXY, refXZ, refYX, refYY, refYZ;
    let refW, refH;
    if (mId === 0) {
      refCX = sx1X;
      refCY = sx1Y;
      refCZ = sx1Z;
      refNX = x1X;
      refNY = x1Y;
      refNZ = x1Z;
      refXX = y1X;
      refXY = y1Y;
      refXZ = y1Z;
      refYX = z1X;
      refYY = z1Y;
      refYZ = z1Z;
      refW = h1;
      refH = d1;
    } else if (mId === 1) {
      refCX = sy1X;
      refCY = sy1Y;
      refCZ = sy1Z;
      refNX = y1X;
      refNY = y1Y;
      refNZ = y1Z;
      refXX = z1X;
      refXY = z1Y;
      refXZ = z1Z;
      refYX = x1X;
      refYY = x1Y;
      refYZ = x1Z;
      refW = d1;
      refH = w1;
    } else {
      refCX = sz1X;
      refCY = sz1Y;
      refCZ = sz1Z;
      refNX = z1X;
      refNY = z1Y;
      refNZ = z1Z;
      refXX = x1X;
      refXY = x1Y;
      refXZ = x1Z;
      refYX = y1X;
      refYY = y1Y;
      refYZ = y1Z;
      refW = w1;
      refH = h1;
    }
    if (mSign < 0) {
      refCX = -refCX;
      refCY = -refCY;
      refCZ = -refCZ;
      refNX = -refNX;
      refNY = -refNY;
      refNZ = -refNZ;
      t = refXX;
      refXX = refYX;
      refYX = t;
      t = refXY;
      refXY = refYY;
      refYY = t;
      t = refXZ;
      refXZ = refYZ;
      refYZ = t;
      t = refW;
      refW = refH;
      refH = t;
    }
    refCX += c1X;
    refCY += c1Y;
    refCZ += c1Z;
    let minIncDot = 1;
    let incId = 0;
    let incDot = refNX * x2X + refNY * x2Y + refNZ * x2Z;
    if (incDot < minIncDot) {
      minIncDot = incDot;
      incId = 0;
    }
    if (-incDot < minIncDot) {
      minIncDot = -incDot;
      incId = 1;
    }
    incDot = refNX * y2X + refNY * y2Y + refNZ * y2Z;
    if (incDot < minIncDot) {
      minIncDot = incDot;
      incId = 2;
    }
    if (-incDot < minIncDot) {
      minIncDot = -incDot;
      incId = 3;
    }
    incDot = refNX * z2X + refNY * z2Y + refNZ * z2Z;
    if (incDot < minIncDot) {
      minIncDot = incDot;
      incId = 4;
    }
    if (-incDot < minIncDot) incId = 5;
    let v1X, v1Y, v1Z, v2X, v2Y, v2Z;
    let v3X, v3Y, v3Z, v4X, v4Y, v4Z;
    switch (incId) {
      case 0:
        v1X = sx2X + sy2X + sz2X;
        v1Y = sx2Y + sy2Y + sz2Y;
        v1Z = sx2Z + sy2Z + sz2Z;
        v2X = sx2X - sy2X + sz2X;
        v2Y = sx2Y - sy2Y + sz2Y;
        v2Z = sx2Z - sy2Z + sz2Z;
        v3X = sx2X - sy2X - sz2X;
        v3Y = sx2Y - sy2Y - sz2Y;
        v3Z = sx2Z - sy2Z - sz2Z;
        v4X = sx2X + sy2X - sz2X;
        v4Y = sx2Y + sy2Y - sz2Y;
        v4Z = sx2Z + sy2Z - sz2Z;
        break;
      case 1:
        v1X = sy2X - sx2X + sz2X;
        v1Y = sy2Y - sx2Y + sz2Y;
        v1Z = sy2Z - sx2Z + sz2Z;
        v2X = sy2X - sx2X - sz2X;
        v2Y = sy2Y - sx2Y - sz2Y;
        v2Z = sy2Z - sx2Z - sz2Z;
        v3X = -(sx2X + sy2X) - sz2X;
        v3Y = -(sx2Y + sy2Y) - sz2Y;
        v3Z = -(sx2Z + sy2Z) - sz2Z;
        v4X = -(sx2X + sy2X) + sz2X;
        v4Y = -(sx2Y + sy2Y) + sz2Y;
        v4Z = -(sx2Z + sy2Z) + sz2Z;
        break;
      case 2:
        v1X = sx2X + sy2X + sz2X;
        v1Y = sx2Y + sy2Y + sz2Y;
        v1Z = sx2Z + sy2Z + sz2Z;
        v2X = sx2X + sy2X - sz2X;
        v2Y = sx2Y + sy2Y - sz2Y;
        v2Z = sx2Z + sy2Z - sz2Z;
        v3X = sy2X - sx2X - sz2X;
        v3Y = sy2Y - sx2Y - sz2Y;
        v3Z = sy2Z - sx2Z - sz2Z;
        v4X = sy2X - sx2X + sz2X;
        v4Y = sy2Y - sx2Y + sz2Y;
        v4Z = sy2Z - sx2Z + sz2Z;
        break;
      case 3:
        v1X = sx2X - sy2X + sz2X;
        v1Y = sx2Y - sy2Y + sz2Y;
        v1Z = sx2Z - sy2Z + sz2Z;
        v2X = -(sx2X + sy2X) + sz2X;
        v2Y = -(sx2Y + sy2Y) + sz2Y;
        v2Z = -(sx2Z + sy2Z) + sz2Z;
        v3X = -(sx2X + sy2X) - sz2X;
        v3Y = -(sx2Y + sy2Y) - sz2Y;
        v3Z = -(sx2Z + sy2Z) - sz2Z;
        v4X = sx2X - sy2X - sz2X;
        v4Y = sx2Y - sy2Y - sz2Y;
        v4Z = sx2Z - sy2Z - sz2Z;
        break;
      case 4:
        v1X = sx2X + sy2X + sz2X;
        v1Y = sx2Y + sy2Y + sz2Y;
        v1Z = sx2Z + sy2Z + sz2Z;
        v2X = sy2X - sx2X + sz2X;
        v2Y = sy2Y - sx2Y + sz2Y;
        v2Z = sy2Z - sx2Z + sz2Z;
        v3X = -(sx2X + sy2X) + sz2X;
        v3Y = -(sx2Y + sy2Y) + sz2Y;
        v3Z = -(sx2Z + sy2Z) + sz2Z;
        v4X = sx2X - sy2X + sz2X;
        v4Y = sx2Y - sy2Y + sz2Y;
        v4Z = sx2Z - sy2Z + sz2Z;
        break;
      default:
        v1X = sx2X + sy2X - sz2X;
        v1Y = sx2Y + sy2Y - sz2Y;
        v1Z = sx2Z + sy2Z - sz2Z;
        v2X = sx2X - sy2X - sz2X;
        v2Y = sx2Y - sy2Y - sz2Y;
        v2Z = sx2Z - sy2Z - sz2Z;
        v3X = -(sx2X + sy2X) - sz2X;
        v3Y = -(sx2Y + sy2Y) - sz2Y;
        v3Z = -(sx2Z + sy2Z) - sz2Z;
        v4X = sy2X - sx2X - sz2X;
        v4Y = sy2Y - sx2Y - sz2Y;
        v4Z = sy2Z - sx2Z - sz2Z;
    }
    v1X += c12X;
    v1Y += c12Y;
    v1Z += c12Z;
    v2X += c12X;
    v2Y += c12Y;
    v2Z += c12Z;
    v3X += c12X;
    v3Y += c12Y;
    v3Z += c12Z;
    v4X += c12X;
    v4Y += c12Y;
    v4Z += c12Z;
    const clipper = this.clipper;
    clipper.init(refW, refH);
    clipper.addIncidentVertex(v1X * refXX + v1Y * refXY + v1Z * refXZ, v1X * refYX + v1Y * refYY + v1Z * refYZ, v1X, v1Y, v1Z);
    clipper.addIncidentVertex(v2X * refXX + v2Y * refXY + v2Z * refXZ, v2X * refYX + v2Y * refYY + v2Z * refYZ, v2X, v2Y, v2Z);
    clipper.addIncidentVertex(v3X * refXX + v3Y * refXY + v3Z * refXZ, v3X * refYX + v3Y * refYY + v3Z * refYZ, v3X, v3Y, v3Z);
    clipper.addIncidentVertex(v4X * refXX + v4Y * refXY + v4Z * refXZ, v4X * refYX + v4Y * refYY + v4Z * refYZ, v4X, v4Y, v4Z);
    clipper.clip();
    clipper.reduce();
    if (refOnBox2) this.setNormal(result, refNX, refNY, refNZ);
    else this.setNormal(result, -refNX, -refNY, -refNZ);
    for (let i = 0; i < clipper.numVertices; i++) {
      const v = clipper.vertices[i];
      const cX = v.wx + c1X, cY = v.wy + c1Y, cZ = v.wz + c1Z;
      depth = (refCX - cX) * refNX + (refCY - cY) * refNY + (refCZ - cZ) * refNZ;
      const fX = cX + refNX * depth, fY = cY + refNY * depth, fZ = cZ + refNZ * depth;
      if (depth > -CONTACT_PERSISTENCE_THRESHOLD - margin) {
        if (refOnBox2) this.addPoint(result, cX, cY, cZ, fX, fY, fZ, depth, i);
        else this.addPoint(result, fX, fY, fZ, cX, cY, cZ, depth, i);
      }
    }
  }
};

// src/core/collision/narrowphase/capsule-capsule-detector.ts
var CapsuleCapsuleDetector = class extends Detector {
  constructor() {
    super(false);
  }
  detectImpl(result, geom1, geom2, tf1, tf2, _cachedData, margin) {
    result.incremental = false;
    const c1 = geom1;
    const c2 = geom2;
    const axis1X = tf1.r01, axis1Y = tf1.r11, axis1Z = tf1.r21;
    const axis2X = tf2.r01, axis2Y = tf2.r11, axis2Z = tf2.r21;
    const hh1 = c1.halfHeight;
    const hh2 = c2.halfHeight;
    const r1 = c1.radius;
    const r2 = c2.radius;
    const p1X = tf1.px + axis1X * -hh1;
    const p1Y = tf1.py + axis1Y * -hh1;
    const p1Z = tf1.pz + axis1Z * -hh1;
    const q1X = tf1.px + axis1X * hh1;
    const q1Y = tf1.py + axis1Y * hh1;
    const q1Z = tf1.pz + axis1Z * hh1;
    const p2X = tf2.px + axis2X * -hh2;
    const p2Y = tf2.py + axis2Y * -hh2;
    const p2Z = tf2.pz + axis2Z * -hh2;
    const q2X = tf2.px + axis2X * hh2;
    const q2Y = tf2.py + axis2Y * hh2;
    const q2Z = tf2.pz + axis2Z * hh2;
    const p12X = p1X - p2X, p12Y = p1Y - p2Y, p12Z = p1Z - p2Z;
    const d1X = q1X - p1X, d1Y = q1Y - p1Y, d1Z = q1Z - p1Z;
    const d2X = q2X - p2X, d2Y = q2Y - p2Y, d2Z = q2Z - p2Z;
    const p21d1 = -(p12X * d1X + p12Y * d1Y + p12Z * d1Z);
    const p12d2 = p12X * d2X + p12Y * d2Y + p12Z * d2Z;
    const d11 = hh1 * hh1 * 4;
    const d12 = d1X * d2X + d1Y * d2Y + d1Z * d2Z;
    const d22 = hh2 * hh2 * 4;
    let t1, t2;
    if (d11 === 0 && d22 === 0) {
      t1 = 0;
      t2 = 0;
    } else if (d11 === 0) {
      t1 = 0;
      if (p12d2 < 0) t2 = 0;
      else if (p12d2 > d22) t2 = 1;
      else t2 = p12d2 / d22;
    } else if (d22 === 0) {
      t2 = 0;
      if (p21d1 < 0) t1 = 0;
      else if (p21d1 > d11) t1 = 1;
      else t1 = p21d1 / d11;
    } else {
      const det2 = d11 * d22 - d12 * d12;
      if (det2 === 0) {
        t1 = 0;
      } else {
        t1 = d12 * p12d2 + d22 * p21d1;
        if (t1 < 0) t1 = 0;
        else if (t1 > det2) t1 = 1;
        else t1 /= det2;
      }
      t2 = t1 * d12 + p12d2;
      if (t2 < 0) {
        t2 = 0;
        if (p21d1 < 0) t1 = 0;
        else if (p21d1 > d11) t1 = 1;
        else t1 = p21d1 / d11;
      } else if (t2 > d22) {
        t2 = 1;
        t1 = d12 + p21d1;
        if (t1 < 0) t1 = 0;
        else if (t1 > d11) t1 = 1;
        else t1 /= d11;
      } else {
        t2 /= d22;
      }
    }
    const cp1X = p1X + d1X * t1, cp1Y = p1Y + d1Y * t1, cp1Z = p1Z + d1Z * t1;
    const cp2X = p2X + d2X * t2, cp2Y = p2Y + d2Y * t2, cp2Z = p2Z + d2Z * t2;
    const dX = cp1X - cp2X, dY = cp1Y - cp2Y, dZ = cp1Z - cp2Z;
    const len2 = dX * dX + dY * dY + dZ * dZ;
    const reach = r1 + r2 + margin;
    if (len2 >= reach * reach) return;
    const len = Math.sqrt(len2);
    let nX, nY, nZ;
    if (len > 0) {
      nX = dX * (1 / len);
      nY = dY * (1 / len);
      nZ = dZ * (1 / len);
    } else {
      nX = 1;
      nY = 0;
      nZ = 0;
    }
    this.setNormal(result, nX, nY, nZ);
    this.addPoint(
      result,
      cp1X + nX * -r1,
      cp1Y + nY * -r1,
      cp1Z + nZ * -r1,
      cp2X + nX * r2,
      cp2Y + nY * r2,
      cp2Z + nZ * r2,
      r1 + r2 - len,
      0
    );
  }
};

// src/core/collision/narrowphase/gjk-epa-detector.ts
var GjkEpaDetector = class extends Detector {
  constructor() {
    super(false);
  }
  detectImpl(result, geom1, geom2, tf1, tf2, cachedData, margin) {
    const gjkEpa = GjkEpa.getInstance();
    const g1 = geom1, g2 = geom2;
    const status = gjkEpa.computeClosestPoints(g1, g2, tf1, tf2, ENABLE_GJK_CACHING ? cachedData : null);
    result.incremental = true;
    if (status !== GjkEpaResultState.SUCCEEDED) return;
    if (gjkEpa.distance > g1.gjkMargin + g2.gjkMargin + margin) return;
    let pos1X = gjkEpa.closestPoint1.x, pos1Y = gjkEpa.closestPoint1.y, pos1Z = gjkEpa.closestPoint1.z;
    let pos2X = gjkEpa.closestPoint2.x, pos2Y = gjkEpa.closestPoint2.y, pos2Z = gjkEpa.closestPoint2.z;
    let nX = pos1X - pos2X, nY = pos1Y - pos2Y, nZ = pos1Z - pos2Z;
    if (nX * nX + nY * nY + nZ * nZ === 0) return;
    if (gjkEpa.distance < 0) {
      nX = -nX;
      nY = -nY;
      nZ = -nZ;
    }
    let l = nX * nX + nY * nY + nZ * nZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    nX *= l;
    nY *= l;
    nZ *= l;
    this.setNormal(result, nX, nY, nZ);
    pos1X += nX * -g1.gjkMargin;
    pos1Y += nY * -g1.gjkMargin;
    pos1Z += nZ * -g1.gjkMargin;
    pos2X += nX * g2.gjkMargin;
    pos2Y += nY * g2.gjkMargin;
    pos2Z += nZ * g2.gjkMargin;
    this.addPoint(result, pos1X, pos1Y, pos1Z, pos2X, pos2Y, pos2Z, g1.gjkMargin + g2.gjkMargin - gjkEpa.distance, 0);
  }
};

// src/core/collision/narrowphase/sphere-box-detector.ts
var SphereBoxDetector = class extends Detector {
  constructor(swapped) {
    super(swapped);
  }
  detectImpl(result, geom1, geom2, tf1, tf2, _cachedData, margin) {
    result.incremental = false;
    const b = geom2;
    let halfExtX = b.hx, halfExtY = b.hy, halfExtZ = b.hz;
    let negHalfExtX = -halfExtX, negHalfExtY = -halfExtY, negHalfExtZ = -halfExtZ;
    const r = geom1.radius;
    const wX = tf1.px - tf2.px, wY = tf1.py - tf2.py, wZ = tf1.pz - tf2.pz;
    const sX = tf2.r00 * wX + tf2.r10 * wY + tf2.r20 * wZ;
    const sY = tf2.r01 * wX + tf2.r11 * wY + tf2.r21 * wZ;
    const sZ = tf2.r02 * wX + tf2.r12 * wY + tf2.r22 * wZ;
    if (negHalfExtX < sX && halfExtX > sX && negHalfExtY < sY && halfExtY > sY && negHalfExtZ < sZ && halfExtZ > sZ) {
      const distX = halfExtX - (sX < 0 ? -sX : sX);
      const distY = halfExtY - (sY < 0 ? -sY : sY);
      const distZ = halfExtZ - (sZ < 0 ? -sZ : sZ);
      let nbX, nbY, nbZ;
      let maskX, maskY, maskZ;
      let depth;
      if (distX < distY) {
        if (distX < distZ) {
          nbX = sX > 0 ? 1 : -1;
          nbY = 0;
          nbZ = 0;
          maskX = 0;
          maskY = 1;
          maskZ = 1;
          depth = distX;
        } else {
          nbX = 0;
          nbY = 0;
          nbZ = sZ > 0 ? 1 : -1;
          maskX = 1;
          maskY = 1;
          maskZ = 0;
          depth = distZ;
        }
      } else if (distY < distZ) {
        nbX = 0;
        nbY = sY > 0 ? 1 : -1;
        nbZ = 0;
        maskX = 1;
        maskY = 0;
        maskZ = 1;
        depth = distY;
      } else {
        nbX = 0;
        nbY = 0;
        nbZ = sZ > 0 ? 1 : -1;
        maskX = 1;
        maskY = 1;
        maskZ = 0;
        depth = distZ;
      }
      let cbX2 = nbX * halfExtX, cbY2 = nbY * halfExtY, cbZ2 = nbZ * halfExtZ;
      cbX2 += maskX * sX;
      cbY2 += maskY * sY;
      cbZ2 += maskZ * sZ;
      const cX2 = tf2.r00 * cbX2 + tf2.r01 * cbY2 + tf2.r02 * cbZ2;
      const cY2 = tf2.r10 * cbX2 + tf2.r11 * cbY2 + tf2.r12 * cbZ2;
      const cZ2 = tf2.r20 * cbX2 + tf2.r21 * cbY2 + tf2.r22 * cbZ2;
      const nX2 = tf2.r00 * nbX + tf2.r01 * nbY + tf2.r02 * nbZ;
      const nY2 = tf2.r10 * nbX + tf2.r11 * nbY + tf2.r12 * nbZ;
      const nZ2 = tf2.r20 * nbX + tf2.r21 * nbY + tf2.r22 * nbZ;
      this.setNormal(result, nX2, nY2, nZ2);
      this.addPoint(
        result,
        tf1.px + nX2 * -r,
        tf1.py + nY2 * -r,
        tf1.pz + nZ2 * -r,
        tf2.px + cX2,
        tf2.py + cY2,
        tf2.pz + cZ2,
        depth,
        0
      );
      return;
    }
    halfExtX -= 1e-9;
    halfExtY -= 1e-9;
    halfExtZ -= 1e-9;
    negHalfExtX += 1e-9;
    negHalfExtY += 1e-9;
    negHalfExtZ += 1e-9;
    let cbX = sX < halfExtX ? sX : halfExtX;
    let cbY = sY < halfExtY ? sY : halfExtY;
    let cbZ = sZ < halfExtZ ? sZ : halfExtZ;
    if (!(cbX > negHalfExtX)) cbX = negHalfExtX;
    if (!(cbY > negHalfExtY)) cbY = negHalfExtY;
    if (!(cbZ > negHalfExtZ)) cbZ = negHalfExtZ;
    const csX = sX - cbX, csY = sY - cbY, csZ = sZ - cbZ;
    let dist = csX * csX + csY * csY + csZ * csZ;
    const reach = r + margin;
    if (dist >= reach * reach) return;
    dist = Math.sqrt(dist);
    const cX = tf2.r00 * cbX + tf2.r01 * cbY + tf2.r02 * cbZ;
    const cY = tf2.r10 * cbX + tf2.r11 * cbY + tf2.r12 * cbZ;
    const cZ = tf2.r20 * cbX + tf2.r21 * cbY + tf2.r22 * cbZ;
    const wsX = tf2.r00 * csX + tf2.r01 * csY + tf2.r02 * csZ;
    const wsY = tf2.r10 * csX + tf2.r11 * csY + tf2.r12 * csZ;
    const wsZ = tf2.r20 * csX + tf2.r21 * csY + tf2.r22 * csZ;
    let l = wsX * wsX + wsY * wsY + wsZ * wsZ;
    if (l > 0) l = 1 / Math.sqrt(l);
    const nX = wsX * l, nY = wsY * l, nZ = wsZ * l;
    this.setNormal(result, nX, nY, nZ);
    this.addPoint(
      result,
      tf1.px + nX * -r,
      tf1.py + nY * -r,
      tf1.pz + nZ * -r,
      tf2.px + cX,
      tf2.py + cY,
      tf2.pz + cZ,
      r - dist,
      0
    );
  }
};

// src/core/collision/narrowphase/sphere-capsule-detector.ts
var SphereCapsuleDetector = class extends Detector {
  constructor(swapped) {
    super(swapped);
  }
  detectImpl(result, geom1, geom2, tf1, tf2, _cachedData, margin) {
    result.incremental = false;
    const c2 = geom2;
    const hh2 = c2.halfHeight;
    const r1 = geom1.radius;
    const r2 = c2.radius;
    const axis2X = tf2.r01, axis2Y = tf2.r11, axis2Z = tf2.r21;
    const cp1X = tf1.px, cp1Y = tf1.py, cp1Z = tf1.pz;
    const p2X = tf2.px + axis2X * -hh2;
    const p2Y = tf2.py + axis2Y * -hh2;
    const p2Z = tf2.pz + axis2Z * -hh2;
    const q2X = tf2.px + axis2X * hh2;
    const q2Y = tf2.py + axis2Y * hh2;
    const q2Z = tf2.pz + axis2Z * hh2;
    const p12X = cp1X - p2X, p12Y = cp1Y - p2Y, p12Z = cp1Z - p2Z;
    const d2X = q2X - p2X, d2Y = q2Y - p2Y, d2Z = q2Z - p2Z;
    const d22 = hh2 * hh2 * 4;
    let t = p12X * d2X + p12Y * d2Y + p12Z * d2Z;
    if (t < 0) {
      t = 0;
    } else if (t > d22) {
      t = 1;
    } else if (d22 > 0 || compat.keepOimoBugs) {
      t /= d22;
    } else {
      t = 0;
    }
    const cp2X = p2X + d2X * t;
    const cp2Y = p2Y + d2Y * t;
    const cp2Z = p2Z + d2Z * t;
    const dX = cp1X - cp2X, dY = cp1Y - cp2Y, dZ = cp1Z - cp2Z;
    const len2 = dX * dX + dY * dY + dZ * dZ;
    const reach = r1 + r2 + margin;
    if (len2 >= reach * reach) return;
    const len = Math.sqrt(len2);
    let nX, nY, nZ;
    if (len > 0) {
      nX = dX * (1 / len);
      nY = dY * (1 / len);
      nZ = dZ * (1 / len);
    } else {
      nX = 1;
      nY = 0;
      nZ = 0;
    }
    this.setNormal(result, nX, nY, nZ);
    this.addPoint(
      result,
      cp1X + nX * -r1,
      cp1Y + nY * -r1,
      cp1Z + nZ * -r1,
      cp2X + nX * r2,
      cp2Y + nY * r2,
      cp2Z + nZ * r2,
      r1 + r2 - len,
      0
    );
  }
};

// src/core/collision/narrowphase/sphere-sphere-detector.ts
var SphereSphereDetector = class extends Detector {
  constructor() {
    super(false);
  }
  detectImpl(result, geom1, geom2, tf1, tf2, _cachedData, margin) {
    result.incremental = false;
    const dX = tf1.px - tf2.px;
    const dY = tf1.py - tf2.py;
    const dZ = tf1.pz - tf2.pz;
    const r1 = geom1.radius;
    const r2 = geom2.radius;
    const len2 = dX * dX + dY * dY + dZ * dZ;
    const reach = r1 + r2 + margin;
    if (len2 >= reach * reach) return;
    const len = Math.sqrt(len2);
    let nX, nY, nZ;
    if (len > 0) {
      nX = dX * (1 / len);
      nY = dY * (1 / len);
      nZ = dZ * (1 / len);
    } else {
      nX = 1;
      nY = 0;
      nZ = 0;
    }
    this.setNormal(result, nX, nY, nZ);
    this.addPoint(
      result,
      tf1.px + nX * -r1,
      tf1.py + nY * -r1,
      tf1.pz + nZ * -r1,
      tf2.px + nX * r2,
      tf2.py + nY * r2,
      tf2.pz + nZ * r2,
      r1 + r2 - len,
      0
    );
  }
};

// src/core/collision/narrowphase/collision-matrix.ts
var CollisionMatrix = class {
  constructor() {
    this.detectors = [];
    const gjkEpa = new GjkEpaDetector();
    for (let i = 0; i < 6; i++) this.detectors.push([gjkEpa, gjkEpa, gjkEpa, gjkEpa, gjkEpa, gjkEpa]);
    const d = this.detectors;
    d[GeometryType.SPHERE][GeometryType.SPHERE] = new SphereSphereDetector();
    d[GeometryType.SPHERE][GeometryType.BOX] = new SphereBoxDetector(false);
    d[GeometryType.SPHERE][GeometryType.CAPSULE] = new SphereCapsuleDetector(false);
    d[GeometryType.BOX][GeometryType.SPHERE] = new SphereBoxDetector(true);
    d[GeometryType.BOX][GeometryType.BOX] = new BoxBoxDetector();
    d[GeometryType.CAPSULE][GeometryType.SPHERE] = new SphereCapsuleDetector(true);
    d[GeometryType.CAPSULE][GeometryType.CAPSULE] = new CapsuleCapsuleDetector();
  }
  getDetector(type1, type2) {
    return this.detectors[type1][type2];
  }
};

// src/core/collision/narrowphase/cached-detector-data.ts
var CachedDetectorData = class {
  constructor() {
    this.gjkCache = null;
  }
  clear() {
    if (this.gjkCache !== null) this.gjkCache.clear();
  }
};

// src/core/collision/narrowphase/detector-result.ts
var DetectorResultPoint = class {
  constructor() {
    /** 几何体 1 上的点 */
    this.p1x = 0;
    this.p1y = 0;
    this.p1z = 0;
    /** 几何体 2 上的点 */
    this.p2x = 0;
    this.p2y = 0;
    this.p2z = 0;
    /** 重叠深度，分离时为负 */
    this.depth = 0;
    /** 接触点编号，帧间据此认出同一个接触点 */
    this.id = 0;
  }
};
var DetectorResult = class {
  constructor() {
    this.numPoints = 0;
    this.points = [];
    /** 接触法线（单位向量），从几何体 2 指向几何体 1 */
    this.nx = 0;
    this.ny = 0;
    this.nz = 0;
    /**
     * true：每次只给出 1 个接触点，要和流形里已有的点合并（GJK/EPA）。
     * false：每次给出完整的流形，直接替换。由检测器设置，clear() 不动它
     */
    this.incremental = false;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.points.push(new DetectorResultPoint());
  }
  /** 所有接触点里最大的深度，没有接触点时为 0 */
  getMaxDepth() {
    let max = 0;
    for (let i = 0; i < this.numPoints; i++) {
      if (this.points[i].depth > max) max = this.points[i].depth;
    }
    return max;
  }
  clear() {
    this.numPoints = 0;
    for (let i = 0; i < this.points.length; i++) {
      const p = this.points[i];
      p.p1x = 0;
      p.p1y = 0;
      p.p1z = 0;
      p.p2x = 0;
      p.p2y = 0;
      p.p2z = 0;
      p.depth = 0;
      p.id = 0;
    }
    this.nx = 0;
    this.ny = 0;
    this.nz = 0;
  }
};

// src/core/collision/narrowphase/compact-box-manifold.ts
var manifoldPoints = new Array(24).fill(0);
var manifoldSeparations = new Array(8).fill(0);
var manifoldNormal = new Array(3).fill(0);
var clipped = new Array(48).fill(0);
var scratch = new Array(48).fill(0);
var transpose = new Array(9).fill(0);
var candidates = new Array(48).fill(0);
var gaps = new Array(16).fill(0);
var selected = new Array(4).fill(0);
var faceSign = 1;
function clip(input, count, output, axis, sign, limit) {
  if (count === 0) return 0;
  let result = 0;
  let px = input[(count - 1) * 3], py = input[(count - 1) * 3 + 1], pz = input[(count - 1) * 3 + 2];
  let previous = sign * input[(count - 1) * 3 + axis] - limit;
  for (let i = 0; i < count; i++) {
    const x = input[i * 3], y = input[i * 3 + 1], z = input[i * 3 + 2], distance = sign * input[i * 3 + axis] - limit;
    if (distance <= 0) {
      if (previous > 0) {
        const t = previous / (previous - distance);
        output[result * 3] = px + (x - px) * t;
        output[result * 3 + 1] = py + (y - py) * t;
        output[result * 3 + 2] = pz + (z - pz) * t;
        result++;
      }
      output[result * 3] = x;
      output[result * 3 + 1] = y;
      output[result * 3 + 2] = z;
      result++;
    } else if (previous <= 0) {
      const t = previous / (previous - distance);
      output[result * 3] = px + (x - px) * t;
      output[result * 3 + 1] = py + (y - py) * t;
      output[result * 3 + 2] = pz + (z - pz) * t;
      result++;
    }
    px = x;
    py = y;
    pz = z;
    previous = distance;
  }
  return result;
}
function face(ax, ay, az, x, y, z, r, bx, by, bz, axis, margin) {
  const sign = (axis === 0 ? x : axis === 1 ? y : z) >= 0 ? 1 : -1;
  faceSign = sign;
  const half = axis === 0 ? ax : axis === 1 ? ay : az, row = axis * 3;
  const rx = Math.abs(r[row]), ry = Math.abs(r[row + 1]), rz = Math.abs(r[row + 2]);
  let incident = 0;
  if (ry > rx && ry >= rz) incident = 1;
  else if (rz > rx && rz > ry) incident = 2;
  const direction = sign * r[row + incident] > 0 ? -1 : 1;
  const depth = incident === 0 ? bx : incident === 1 ? by : bz;
  const u = incident === 0 ? 1 : 0, v = incident === 2 ? 1 : 2;
  const hu = u === 0 ? bx : by, hv = v === 1 ? by : bz;
  const cx = x + direction * depth * r[incident], cy = y + direction * depth * r[3 + incident], cz = z + direction * depth * r[6 + incident];
  const ux = r[u] * hu, uy = r[3 + u] * hu, uz = r[6 + u] * hu;
  const vx = r[v] * hv, vy = r[3 + v] * hv, vz = r[6 + v] * hv;
  clipped[0] = cx + ux + vx;
  clipped[1] = cy + uy + vy;
  clipped[2] = cz + uz + vz;
  clipped[3] = cx - ux + vx;
  clipped[4] = cy - uy + vy;
  clipped[5] = cz - uz + vz;
  clipped[6] = cx - ux - vx;
  clipped[7] = cy - uy - vy;
  clipped[8] = cz - uz - vz;
  clipped[9] = cx + ux - vx;
  clipped[10] = cy + uy - vy;
  clipped[11] = cz + uz - vz;
  const a = axis === 0 ? 1 : 0, b = axis === 2 ? 1 : 2;
  const ha = a === 0 ? ax : ay, hb = b === 1 ? ay : az;
  let count = clip(clipped, 4, scratch, a, 1, ha);
  count = clip(scratch, count, clipped, a, -1, ha);
  count = clip(clipped, count, scratch, b, 1, hb);
  count = clip(scratch, count, clipped, b, -1, hb);
  let result = 0;
  for (let i = 0; i < count; i++) {
    const gap = sign * clipped[i * 3 + axis] - half;
    if (gap > margin) continue;
    candidates[result * 3] = clipped[i * 3];
    candidates[result * 3 + 1] = clipped[i * 3 + 1];
    candidates[result * 3 + 2] = clipped[i * 3 + 2];
    candidates[result * 3 + axis] -= sign * gap * 0.5;
    gaps[result++] = gap;
  }
  return result;
}
function reduce(count) {
  if (count <= 4) return count;
  let first = 0;
  for (let i = 1; i < count; i++) if (gaps[i] < gaps[first]) first = i;
  const x = candidates[first * 3], y = candidates[first * 3 + 1], z = candidates[first * 3 + 2];
  let second = -1, maximum = -1;
  for (let i = 0; i < count; i++) {
    if (i === first) continue;
    const dx2 = candidates[i * 3] - x, dy2 = candidates[i * 3 + 1] - y, dz2 = candidates[i * 3 + 2] - z;
    const distance = dx2 * dx2 + dy2 * dy2 + dz2 * dz2;
    if (distance > maximum) {
      maximum = distance;
      second = i;
    }
  }
  const dx = candidates[second * 3] - x, dy = candidates[second * 3 + 1] - y, dz = candidates[second * 3 + 2] - z;
  let third = -1, nx = 0, ny = 0, nz = 0;
  maximum = -1;
  for (let i = 0; i < count; i++) {
    if (i === first || i === second) continue;
    const px = candidates[i * 3] - x, py = candidates[i * 3 + 1] - y, pz = candidates[i * 3 + 2] - z;
    const cx = dy * pz - dz * py, cy = dz * px - dx * pz, cz = dx * py - dy * px;
    const area = cx * cx + cy * cy + cz * cz;
    if (area > maximum) {
      maximum = area;
      third = i;
      nx = cx;
      ny = cy;
      nz = cz;
    }
  }
  let fourth = -1;
  maximum = 0;
  for (let i = 0; i < count; i++) {
    if (i === first || i === second || i === third) continue;
    const px = candidates[i * 3] - x, py = candidates[i * 3 + 1] - y, pz = candidates[i * 3 + 2] - z;
    const cx = dy * pz - dz * py, cy = dz * px - dx * pz, cz = dx * py - dy * px;
    const area = -(cx * nx + cy * ny + cz * nz);
    if (area > maximum) {
      maximum = area;
      fourth = i;
    }
  }
  const retained2 = fourth >= 0 ? 4 : 3;
  selected[0] = first;
  selected[1] = second;
  selected[2] = third;
  selected[3] = fourth;
  for (let i = 1; i < retained2; i++) {
    const value = selected[i];
    let j = i;
    while (j > 0 && selected[j - 1] > value) {
      selected[j] = selected[j - 1];
      j--;
    }
    selected[j] = value;
  }
  for (let i = 0; i < retained2; i++) {
    const j = selected[i];
    if (i === j) continue;
    candidates[i * 3] = candidates[j * 3];
    candidates[i * 3 + 1] = candidates[j * 3 + 1];
    candidates[i * 3 + 2] = candidates[j * 3 + 2];
    gaps[i] = gaps[j];
  }
  return retained2;
}
function boxManifold(ax, ay, az, x, y, z, r, bx, by, bz, margin) {
  const r00 = r[0], r01 = r[1], r02 = r[2], r10 = r[3], r11 = r[4], r12 = r[5], r20 = r[6], r21 = r[7], r22 = r[8];
  const a00 = Math.abs(r00), a01 = Math.abs(r01), a02 = Math.abs(r02), a10 = Math.abs(r10), a11 = Math.abs(r11), a12 = Math.abs(r12), a20 = Math.abs(r20), a21 = Math.abs(r21), a22 = Math.abs(r22);
  let gap = Math.abs(x) - (ax + bx * a00 + by * a01 + bz * a02);
  if (gap > margin) return 0;
  let faceGap = gap, axis = 0;
  gap = Math.abs(y) - (ay + bx * a10 + by * a11 + bz * a12);
  if (gap > margin) return 0;
  if (gap > faceGap) {
    faceGap = gap;
    axis = 1;
  }
  gap = Math.abs(z) - (az + bx * a20 + by * a21 + bz * a22);
  if (gap > margin) return 0;
  if (gap > faceGap) {
    faceGap = gap;
    axis = 2;
  }
  const epsilon = 1e-12;
  const row0 = a00 > 1 - epsilon ? a01 < epsilon && a02 < epsilon ? 0 : -1 : a01 > 1 - epsilon ? a00 < epsilon && a02 < epsilon ? 1 : -1 : a02 > 1 - epsilon && a00 < epsilon && a01 < epsilon ? 2 : -1;
  let row1 = -1, row2 = -1;
  if (row0 >= 0) {
    row1 = a10 > 1 - epsilon ? a11 < epsilon && a12 < epsilon ? 0 : -1 : a11 > 1 - epsilon ? a10 < epsilon && a12 < epsilon ? 1 : -1 : a12 > 1 - epsilon && a10 < epsilon && a11 < epsilon ? 2 : -1;
    if (row1 >= 0 && row1 !== row0) row2 = a20 > 1 - epsilon ? a21 < epsilon && a22 < epsilon ? 0 : -1 : a21 > 1 - epsilon ? a20 < epsilon && a22 < epsilon ? 1 : -1 : a22 > 1 - epsilon && a20 < epsilon && a21 < epsilon ? 2 : -1;
  }
  const aligned = row2 >= 0 && row2 !== row0 && row2 !== row1;
  let px = 0, py = 0, pz = 0, edgeGap = -Infinity, edgeA = -1, edgeB = -1, nx = 0, ny = 0, nz = 0;
  if (!aligned) {
    px = x * r00 + y * r10 + z * r20;
    py = x * r01 + y * r11 + z * r21;
    pz = x * r02 + y * r12 + z * r22;
    gap = Math.abs(px) - (bx + ax * a00 + ay * a10 + az * a20);
    if (gap > margin) return 0;
    if (gap > faceGap) {
      faceGap = gap;
      axis = 3;
    }
    gap = Math.abs(py) - (by + ax * a01 + ay * a11 + az * a21);
    if (gap > margin) return 0;
    if (gap > faceGap) {
      faceGap = gap;
      axis = 4;
    }
    gap = Math.abs(pz) - (bz + ax * a02 + ay * a12 + az * a22);
    if (gap > margin) return 0;
    if (gap > faceGap) {
      faceGap = gap;
      axis = 5;
    }
    for (let i = 0; i < 3; i++) {
      const rx = Math.abs(r[i * 3]), ry = Math.abs(r[i * 3 + 1]), rz = Math.abs(r[i * 3 + 2]);
      for (let j = 0; j < 3; j++) {
        const ux = r[j], uy = r[3 + j], uz = r[6 + j];
        const cx = i === 0 ? 0 : i === 1 ? uz : -uy;
        const cy = i === 0 ? -uz : i === 1 ? 0 : ux;
        const cz = i === 0 ? uy : i === 1 ? -ux : 0;
        const lengthSq = cx * cx + cy * cy + cz * cz;
        if (lengthSq < 1e-6) continue;
        const projection = x * cx + y * cy + z * cz;
        const extentA = ax * Math.abs(cx) + ay * Math.abs(cy) + az * Math.abs(cz);
        const extentB = j === 0 ? by * rz + bz * ry : j === 1 ? bx * rz + bz * rx : bx * ry + by * rx;
        const inverse = 1 / Math.sqrt(lengthSq);
        gap = (Math.abs(projection) - extentA - extentB) * inverse;
        if (gap > margin) return 0;
        if (gap > edgeGap) {
          edgeGap = gap;
          edgeA = i;
          edgeB = j;
          const sign2 = projection >= 0 ? inverse : -inverse;
          nx = cx * sign2;
          ny = cy * sign2;
          nz = cz * sign2;
        }
      }
    }
  }
  if (edgeA >= 0 && edgeGap > faceGap + 4e-3 + 0.05 * Math.abs(faceGap)) {
    let sx = nx >= 0 ? ax : -ax, sy = ny >= 0 ? ay : -ay, sz = nz >= 0 ? az : -az;
    if (edgeA === 0) sx = 0;
    else if (edgeA === 1) sy = 0;
    else sz = 0;
    let tx = x, ty = y, tz = z;
    for (let j = 0; j < 3; j++) {
      if (j === edgeB) continue;
      const ux2 = r[j], uy2 = r[3 + j], uz2 = r[6 + j], half = j === 0 ? bx : j === 1 ? by : bz;
      const sign2 = nx * ux2 + ny * uy2 + nz * uz2 >= 0 ? -half : half;
      tx += ux2 * sign2;
      ty += uy2 * sign2;
      tz += uz2 * sign2;
    }
    const ux = r[edgeB], uy = r[3 + edgeB], uz = r[6 + edgeB], dx = sx - tx, dy = sy - ty, dz = sz - tz;
    const dot = edgeA === 0 ? ux : edgeA === 1 ? uy : uz;
    const da = edgeA === 0 ? dx : edgeA === 1 ? dy : dz, db = ux * dx + uy * dy + uz * dz, divisor = 1 - dot * dot;
    let ta = 0, tb = 0;
    if (divisor > 1e-9) {
      ta = (dot * db - da) / divisor;
      tb = (db - dot * da) / divisor;
    }
    const ha = edgeA === 0 ? ax : edgeA === 1 ? ay : az, hb = edgeB === 0 ? bx : edgeB === 1 ? by : bz;
    ta = Math.max(-ha, Math.min(ha, ta));
    tb = Math.max(-hb, Math.min(hb, tb));
    if (edgeA === 0) sx = ta;
    else if (edgeA === 1) sy = ta;
    else sz = ta;
    tx += ux * tb;
    ty += uy * tb;
    tz += uz * tb;
    manifoldNormal[0] = nx;
    manifoldNormal[1] = ny;
    manifoldNormal[2] = nz;
    manifoldPoints[0] = (sx + tx) * 0.5;
    manifoldPoints[1] = (sy + ty) * 0.5;
    manifoldPoints[2] = (sz + tz) * 0.5;
    manifoldSeparations[0] = edgeGap;
    return 1;
  }
  if (axis < 3) {
    const count2 = reduce(face(ax, ay, az, x, y, z, r, bx, by, bz, axis, margin));
    manifoldNormal[0] = 0;
    manifoldNormal[1] = 0;
    manifoldNormal[2] = 0;
    manifoldNormal[axis] = faceSign;
    for (let i = 0; i < count2; i++) {
      manifoldPoints[i * 3] = candidates[i * 3];
      manifoldPoints[i * 3 + 1] = candidates[i * 3 + 1];
      manifoldPoints[i * 3 + 2] = candidates[i * 3 + 2];
      manifoldSeparations[i] = gaps[i];
    }
    return count2;
  }
  const column = axis - 3;
  transpose[0] = r00;
  transpose[1] = r10;
  transpose[2] = r20;
  transpose[3] = r01;
  transpose[4] = r11;
  transpose[5] = r21;
  transpose[6] = r02;
  transpose[7] = r12;
  transpose[8] = r22;
  const count = reduce(face(bx, by, bz, -px, -py, -pz, transpose, ax, ay, az, column, margin)), sign = -faceSign;
  manifoldNormal[0] = sign * r[column];
  manifoldNormal[1] = sign * r[3 + column];
  manifoldNormal[2] = sign * r[6 + column];
  for (let i = 0; i < count; i++) {
    const cx = candidates[i * 3], cy = candidates[i * 3 + 1], cz = candidates[i * 3 + 2];
    manifoldPoints[i * 3] = r00 * cx + r01 * cy + r02 * cz + x;
    manifoldPoints[i * 3 + 1] = r10 * cx + r11 * cy + r12 * cz + y;
    manifoldPoints[i * 3 + 2] = r20 * cx + r21 * cy + r22 * cz + z;
    manifoldSeparations[i] = gaps[i];
  }
  return count;
}

// src/core/collision/narrowphase/soft-box-box-detector.ts
var SoftBoxBoxDetector = class extends Detector {
  constructor() {
    super(false);
    this.relative = new Array(9).fill(0);
  }
  detectImpl(result, ga, gb, ta, tb, _cache, margin) {
    result.incremental = false;
    const a = ga, b = gb, r = this.relative;
    r[0] = ta.r00 * tb.r00 + ta.r10 * tb.r10 + ta.r20 * tb.r20;
    r[1] = ta.r00 * tb.r01 + ta.r10 * tb.r11 + ta.r20 * tb.r21;
    r[2] = ta.r00 * tb.r02 + ta.r10 * tb.r12 + ta.r20 * tb.r22;
    r[3] = ta.r01 * tb.r00 + ta.r11 * tb.r10 + ta.r21 * tb.r20;
    r[4] = ta.r01 * tb.r01 + ta.r11 * tb.r11 + ta.r21 * tb.r21;
    r[5] = ta.r01 * tb.r02 + ta.r11 * tb.r12 + ta.r21 * tb.r22;
    r[6] = ta.r02 * tb.r00 + ta.r12 * tb.r10 + ta.r22 * tb.r20;
    r[7] = ta.r02 * tb.r01 + ta.r12 * tb.r11 + ta.r22 * tb.r21;
    r[8] = ta.r02 * tb.r02 + ta.r12 * tb.r12 + ta.r22 * tb.r22;
    const dx = tb.px - ta.px, dy = tb.py - ta.py, dz = tb.pz - ta.pz;
    const count = boxManifold(
      a.hx,
      a.hy,
      a.hz,
      ta.r00 * dx + ta.r10 * dy + ta.r20 * dz,
      ta.r01 * dx + ta.r11 * dy + ta.r21 * dz,
      ta.r02 * dx + ta.r12 * dy + ta.r22 * dz,
      r,
      b.hx,
      b.hy,
      b.hz,
      margin
    );
    if (count === 0) return;
    const nx = ta.r00 * manifoldNormal[0] + ta.r01 * manifoldNormal[1] + ta.r02 * manifoldNormal[2];
    const ny = ta.r10 * manifoldNormal[0] + ta.r11 * manifoldNormal[1] + ta.r12 * manifoldNormal[2];
    const nz = ta.r20 * manifoldNormal[0] + ta.r21 * manifoldNormal[1] + ta.r22 * manifoldNormal[2];
    result.nx = -nx;
    result.ny = -ny;
    result.nz = -nz;
    result.numPoints = count;
    for (let i = 0; i < count; i++) {
      const p = result.points[i], x = manifoldPoints[i * 3], y = manifoldPoints[i * 3 + 1], z = manifoldPoints[i * 3 + 2];
      const wx = ta.px + ta.r00 * x + ta.r01 * y + ta.r02 * z;
      const wy = ta.py + ta.r10 * x + ta.r11 * y + ta.r12 * z;
      const wz = ta.pz + ta.r20 * x + ta.r21 * y + ta.r22 * z;
      const half = manifoldSeparations[i] * 0.5;
      p.p1x = wx - nx * half;
      p.p1y = wy - ny * half;
      p.p1z = wz - nz * half;
      p.p2x = wx + nx * half;
      p.p2y = wy + ny * half;
      p.p2z = wz + nz * half;
      p.depth = -manifoldSeparations[i];
      p.id = i;
    }
  }
};

// src/core/dynamics/constraint/solver/pgs-contact-constraint-solver.ts
var MassDataRow = class {
  constructor() {
    // 法向（N）、切向（T）、副切向（B），1、2 是两个刚体
    this.invMLinN1X = 0;
    this.invMLinN1Y = 0;
    this.invMLinN1Z = 0;
    this.invMLinN2X = 0;
    this.invMLinN2Y = 0;
    this.invMLinN2Z = 0;
    this.invMAngN1X = 0;
    this.invMAngN1Y = 0;
    this.invMAngN1Z = 0;
    this.invMAngN2X = 0;
    this.invMAngN2Y = 0;
    this.invMAngN2Z = 0;
    this.invMLinT1X = 0;
    this.invMLinT1Y = 0;
    this.invMLinT1Z = 0;
    this.invMLinT2X = 0;
    this.invMLinT2Y = 0;
    this.invMLinT2Z = 0;
    this.invMAngT1X = 0;
    this.invMAngT1Y = 0;
    this.invMAngT1Z = 0;
    this.invMAngT2X = 0;
    this.invMAngT2Y = 0;
    this.invMAngT2Z = 0;
    this.invMLinB1X = 0;
    this.invMLinB1Y = 0;
    this.invMLinB1Z = 0;
    this.invMLinB2X = 0;
    this.invMLinB2Y = 0;
    this.invMLinB2Z = 0;
    this.invMAngB1X = 0;
    this.invMAngB1Y = 0;
    this.invMAngB1Z = 0;
    this.invMAngB2X = 0;
    this.invMAngB2Y = 0;
    this.invMAngB2Z = 0;
    /** 法向有效质量 */
    this.massN = 0;
    /** 切向、副切向的 2×2 有效质量矩阵 */
    this.massTB00 = 0;
    this.massTB01 = 0;
    this.massTB10 = 0;
    this.massTB11 = 0;
  }
};
var PgsContactConstraintSolver = class extends ConstraintSolver {
  constructor(constraint) {
    super();
    this.info = new ContactSolverInfo();
    this.massData = [];
    this.constraint = constraint;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.massData.push(new MassDataRow());
  }
  /** 速度求解前：整理接触数据，算每个接触点的有效质量 */
  preSolveVelocity(timeStep) {
    const info = this.info;
    this.constraint.getVelocitySolverInfo(timeStep, info);
    const b1 = this.b1 = info.b1, b2 = this.b2 = info.b2;
    const invM1 = b1.invMass, invM2 = b2.invMass;
    const i100 = b1.ii00, i101 = b1.ii01, i102 = b1.ii02, i110 = b1.ii10, i111 = b1.ii11, i112 = b1.ii12, i120 = b1.ii20, i121 = b1.ii21, i122 = b1.ii22;
    const i200 = b2.ii00, i201 = b2.ii01, i202 = b2.ii02, i210 = b2.ii10, i211 = b2.ii11, i212 = b2.ii12, i220 = b2.ii20, i221 = b2.ii21, i222 = b2.ii22;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i];
      const j = row.jacobianN;
      md.invMLinN1X = j.lin1X * invM1;
      md.invMLinN1Y = j.lin1Y * invM1;
      md.invMLinN1Z = j.lin1Z * invM1;
      md.invMLinN2X = j.lin2X * invM2;
      md.invMLinN2Y = j.lin2Y * invM2;
      md.invMLinN2Z = j.lin2Z * invM2;
      md.invMAngN1X = i100 * j.ang1X + i101 * j.ang1Y + i102 * j.ang1Z;
      md.invMAngN1Y = i110 * j.ang1X + i111 * j.ang1Y + i112 * j.ang1Z;
      md.invMAngN1Z = i120 * j.ang1X + i121 * j.ang1Y + i122 * j.ang1Z;
      md.invMAngN2X = i200 * j.ang2X + i201 * j.ang2Y + i202 * j.ang2Z;
      md.invMAngN2Y = i210 * j.ang2X + i211 * j.ang2Y + i212 * j.ang2Z;
      md.invMAngN2Z = i220 * j.ang2X + i221 * j.ang2Y + i222 * j.ang2Z;
      md.massN = invM1 + invM2 + (md.invMAngN1X * j.ang1X + md.invMAngN1Y * j.ang1Y + md.invMAngN1Z * j.ang1Z) + (md.invMAngN2X * j.ang2X + md.invMAngN2Y * j.ang2Y + md.invMAngN2Z * j.ang2Z);
      if (md.massN !== 0) md.massN = 1 / md.massN;
      const jt = row.jacobianT, jb = row.jacobianB;
      md.invMLinT1X = jt.lin1X * invM1;
      md.invMLinT1Y = jt.lin1Y * invM1;
      md.invMLinT1Z = jt.lin1Z * invM1;
      md.invMLinT2X = jt.lin2X * invM2;
      md.invMLinT2Y = jt.lin2Y * invM2;
      md.invMLinT2Z = jt.lin2Z * invM2;
      md.invMLinB1X = jb.lin1X * invM1;
      md.invMLinB1Y = jb.lin1Y * invM1;
      md.invMLinB1Z = jb.lin1Z * invM1;
      md.invMLinB2X = jb.lin2X * invM2;
      md.invMLinB2Y = jb.lin2Y * invM2;
      md.invMLinB2Z = jb.lin2Z * invM2;
      md.invMAngT1X = i100 * jt.ang1X + i101 * jt.ang1Y + i102 * jt.ang1Z;
      md.invMAngT1Y = i110 * jt.ang1X + i111 * jt.ang1Y + i112 * jt.ang1Z;
      md.invMAngT1Z = i120 * jt.ang1X + i121 * jt.ang1Y + i122 * jt.ang1Z;
      md.invMAngT2X = i200 * jt.ang2X + i201 * jt.ang2Y + i202 * jt.ang2Z;
      md.invMAngT2Y = i210 * jt.ang2X + i211 * jt.ang2Y + i212 * jt.ang2Z;
      md.invMAngT2Z = i220 * jt.ang2X + i221 * jt.ang2Y + i222 * jt.ang2Z;
      md.invMAngB1X = i100 * jb.ang1X + i101 * jb.ang1Y + i102 * jb.ang1Z;
      md.invMAngB1Y = i110 * jb.ang1X + i111 * jb.ang1Y + i112 * jb.ang1Z;
      md.invMAngB1Z = i120 * jb.ang1X + i121 * jb.ang1Y + i122 * jb.ang1Z;
      md.invMAngB2X = i200 * jb.ang2X + i201 * jb.ang2Y + i202 * jb.ang2Z;
      md.invMAngB2Y = i210 * jb.ang2X + i211 * jb.ang2Y + i212 * jb.ang2Z;
      md.invMAngB2Z = i220 * jb.ang2X + i221 * jb.ang2Y + i222 * jb.ang2Z;
      const invMassTB00 = invM1 + invM2 + (md.invMAngT1X * jt.ang1X + md.invMAngT1Y * jt.ang1Y + md.invMAngT1Z * jt.ang1Z) + (md.invMAngT2X * jt.ang2X + md.invMAngT2Y * jt.ang2Y + md.invMAngT2Z * jt.ang2Z);
      const invMassTB01 = md.invMAngT1X * jb.ang1X + md.invMAngT1Y * jb.ang1Y + md.invMAngT1Z * jb.ang1Z + (md.invMAngT2X * jb.ang2X + md.invMAngT2Y * jb.ang2Y + md.invMAngT2Z * jb.ang2Z);
      const invMassTB11 = invM1 + invM2 + (md.invMAngB1X * jb.ang1X + md.invMAngB1Y * jb.ang1Y + md.invMAngB1Z * jb.ang1Z) + (md.invMAngB2X * jb.ang2X + md.invMAngB2Y * jb.ang2Y + md.invMAngB2Z * jb.ang2Z);
      let invDet = invMassTB00 * invMassTB11 - invMassTB01 * invMassTB01;
      if (invDet !== 0) invDet = 1 / invDet;
      md.massTB00 = invMassTB11 * invDet;
      md.massTB01 = -invMassTB01 * invDet;
      md.massTB10 = -invMassTB01 * invDet;
      md.massTB11 = invMassTB00 * invDet;
    }
  }
  /** 热启动：把上一步的累积冲量先施加上去 */
  warmStart(timeStep) {
    const info = this.info, b1 = this.b1, b2 = this.b2;
    let lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz, lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    let av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz, av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse, md = this.massData[i];
      const jt = row.jacobianT, jb = row.jacobianB;
      const impulseN = imp.impulseN;
      const impulseT = imp.impulseLX * jt.lin1X + imp.impulseLY * jt.lin1Y + imp.impulseLZ * jt.lin1Z;
      const impulseB = imp.impulseLX * jb.lin1X + imp.impulseLY * jb.lin1Y + imp.impulseLZ * jb.lin1Z;
      imp.impulseT = impulseT;
      imp.impulseB = impulseB;
      imp.impulseN *= timeStep.dtRatio;
      imp.impulseT *= timeStep.dtRatio;
      imp.impulseB *= timeStep.dtRatio;
      lv1X += md.invMLinN1X * impulseN;
      lv1Y += md.invMLinN1Y * impulseN;
      lv1Z += md.invMLinN1Z * impulseN;
      lv1X += md.invMLinT1X * impulseT;
      lv1Y += md.invMLinT1Y * impulseT;
      lv1Z += md.invMLinT1Z * impulseT;
      lv1X += md.invMLinB1X * impulseB;
      lv1Y += md.invMLinB1Y * impulseB;
      lv1Z += md.invMLinB1Z * impulseB;
      lv2X += md.invMLinN2X * -impulseN;
      lv2Y += md.invMLinN2Y * -impulseN;
      lv2Z += md.invMLinN2Z * -impulseN;
      lv2X += md.invMLinT2X * -impulseT;
      lv2Y += md.invMLinT2Y * -impulseT;
      lv2Z += md.invMLinT2Z * -impulseT;
      lv2X += md.invMLinB2X * -impulseB;
      lv2Y += md.invMLinB2Y * -impulseB;
      lv2Z += md.invMLinB2Z * -impulseB;
      av1X += md.invMAngN1X * impulseN;
      av1Y += md.invMAngN1Y * impulseN;
      av1Z += md.invMAngN1Z * impulseN;
      av1X += md.invMAngT1X * impulseT;
      av1Y += md.invMAngT1Y * impulseT;
      av1Z += md.invMAngT1Z * impulseT;
      av1X += md.invMAngB1X * impulseB;
      av1Y += md.invMAngB1Y * impulseB;
      av1Z += md.invMAngB1Z * impulseB;
      av2X += md.invMAngN2X * -impulseN;
      av2Y += md.invMAngN2Y * -impulseN;
      av2Z += md.invMAngN2Z * -impulseN;
      av2X += md.invMAngT2X * -impulseT;
      av2Y += md.invMAngT2Y * -impulseT;
      av2Z += md.invMAngT2Z * -impulseT;
      av2X += md.invMAngB2X * -impulseB;
      av2Y += md.invMAngB2Y * -impulseB;
      av2Z += md.invMAngB2Z * -impulseB;
    }
    b1.vx = lv1X;
    b1.vy = lv1Y;
    b1.vz = lv1Z;
    b2.vx = lv2X;
    b2.vy = lv2Y;
    b2.vz = lv2Z;
    b1.wx = av1X;
    b1.wy = av1Y;
    b1.wz = av1Z;
    b2.wx = av2X;
    b2.wy = av2Y;
    b2.wz = av2Z;
  }
  /** 一次速度迭代：先解摩擦，再解法向 */
  solveVelocity() {
    const info = this.info, b1 = this.b1, b2 = this.b2;
    let lv1X = b1.vx, lv1Y = b1.vy, lv1Z = b1.vz, lv2X = b2.vx, lv2Y = b2.vy, lv2Z = b2.vz;
    let av1X = b1.wx, av1Y = b1.wy, av1Z = b1.wz, av2X = b2.wx, av2Y = b2.wy, av2Z = b2.wz;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse;
      let j = row.jacobianT;
      let rvt = 0;
      rvt += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rvt -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rvt += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rvt -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      j = row.jacobianB;
      let rvb = 0;
      rvb += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rvb -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rvb += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rvb -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseT = -(rvt * md.massTB00 + rvb * md.massTB01);
      let impulseB = -(rvt * md.massTB10 + rvb * md.massTB11);
      const oldImpulseT = imp.impulseT, oldImpulseB = imp.impulseB;
      imp.impulseT += impulseT;
      imp.impulseB += impulseB;
      const maxImpulse = row.friction * imp.impulseN;
      if (maxImpulse === 0) {
        imp.impulseT = 0;
        imp.impulseB = 0;
      } else {
        const lenSq = imp.impulseT * imp.impulseT + imp.impulseB * imp.impulseB;
        if (lenSq > maxImpulse * maxImpulse) {
          const invL = maxImpulse / Math.sqrt(lenSq);
          imp.impulseT *= invL;
          imp.impulseB *= invL;
        }
      }
      impulseT = imp.impulseT - oldImpulseT;
      impulseB = imp.impulseB - oldImpulseB;
      lv1X += md.invMLinT1X * impulseT;
      lv1Y += md.invMLinT1Y * impulseT;
      lv1Z += md.invMLinT1Z * impulseT;
      lv1X += md.invMLinB1X * impulseB;
      lv1Y += md.invMLinB1Y * impulseB;
      lv1Z += md.invMLinB1Z * impulseB;
      lv2X += md.invMLinT2X * -impulseT;
      lv2Y += md.invMLinT2Y * -impulseT;
      lv2Z += md.invMLinT2Z * -impulseT;
      lv2X += md.invMLinB2X * -impulseB;
      lv2Y += md.invMLinB2Y * -impulseB;
      lv2Z += md.invMLinB2Z * -impulseB;
      av1X += md.invMAngT1X * impulseT;
      av1Y += md.invMAngT1Y * impulseT;
      av1Z += md.invMAngT1Z * impulseT;
      av1X += md.invMAngB1X * impulseB;
      av1Y += md.invMAngB1Y * impulseB;
      av1Z += md.invMAngB1Z * impulseB;
      av2X += md.invMAngT2X * -impulseT;
      av2Y += md.invMAngT2Y * -impulseT;
      av2Z += md.invMAngT2Z * -impulseT;
      av2X += md.invMAngB2X * -impulseB;
      av2Y += md.invMAngB2Y * -impulseB;
      av2Z += md.invMAngB2Z * -impulseB;
    }
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse, j = row.jacobianN;
      let rvn = 0;
      rvn += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rvn -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rvn += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rvn -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseN = (row.rhs - rvn) * md.massN;
      const oldImpulseN = imp.impulseN;
      imp.impulseN += impulseN;
      if (imp.impulseN < 0) imp.impulseN = 0;
      impulseN = imp.impulseN - oldImpulseN;
      lv1X += md.invMLinN1X * impulseN;
      lv1Y += md.invMLinN1Y * impulseN;
      lv1Z += md.invMLinN1Z * impulseN;
      lv2X += md.invMLinN2X * -impulseN;
      lv2Y += md.invMLinN2Y * -impulseN;
      lv2Z += md.invMLinN2Z * -impulseN;
      av1X += md.invMAngN1X * impulseN;
      av1Y += md.invMAngN1Y * impulseN;
      av1Z += md.invMAngN1Z * impulseN;
      av2X += md.invMAngN2X * -impulseN;
      av2Y += md.invMAngN2Y * -impulseN;
      av2Z += md.invMAngN2Z * -impulseN;
    }
    b1.vx = lv1X;
    b1.vy = lv1Y;
    b1.vz = lv1Z;
    b2.vx = lv2X;
    b2.vy = lv2Y;
    b2.vz = lv2Z;
    b1.wx = av1X;
    b1.wy = av1Y;
    b1.wz = av1Z;
    b2.wx = av2X;
    b2.wy = av2Y;
    b2.wz = av2Z;
  }
  /** 位置修正前：刚体已经积分过，重新同步接触点、算法向有效质量 */
  preSolvePosition() {
    const c = this.constraint, info = this.info;
    c.syncManifold();
    c.getPositionSolverInfo(info);
    const b1 = this.b1, b2 = this.b2;
    const invM1 = b1.invMass, invM2 = b2.invMass;
    const i100 = b1.ii00, i101 = b1.ii01, i102 = b1.ii02, i110 = b1.ii10, i111 = b1.ii11, i112 = b1.ii12, i120 = b1.ii20, i121 = b1.ii21, i122 = b1.ii22;
    const i200 = b2.ii00, i201 = b2.ii01, i202 = b2.ii02, i210 = b2.ii10, i211 = b2.ii11, i212 = b2.ii12, i220 = b2.ii20, i221 = b2.ii21, i222 = b2.ii22;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], j = row.jacobianN;
      md.invMLinN1X = j.lin1X * invM1;
      md.invMLinN1Y = j.lin1Y * invM1;
      md.invMLinN1Z = j.lin1Z * invM1;
      md.invMLinN2X = j.lin2X * invM2;
      md.invMLinN2Y = j.lin2Y * invM2;
      md.invMLinN2Z = j.lin2Z * invM2;
      md.invMAngN1X = i100 * j.ang1X + i101 * j.ang1Y + i102 * j.ang1Z;
      md.invMAngN1Y = i110 * j.ang1X + i111 * j.ang1Y + i112 * j.ang1Z;
      md.invMAngN1Z = i120 * j.ang1X + i121 * j.ang1Y + i122 * j.ang1Z;
      md.invMAngN2X = i200 * j.ang2X + i201 * j.ang2Y + i202 * j.ang2Z;
      md.invMAngN2Y = i210 * j.ang2X + i211 * j.ang2Y + i212 * j.ang2Z;
      md.invMAngN2Z = i220 * j.ang2X + i221 * j.ang2Y + i222 * j.ang2Z;
      md.massN = invM1 + invM2 + (md.invMAngN1X * j.ang1X + md.invMAngN1Y * j.ang1Y + md.invMAngN1Z * j.ang1Z) + (md.invMAngN2X * j.ang2X + md.invMAngN2Y * j.ang2Y + md.invMAngN2Z * j.ang2Z);
      if (md.massN !== 0) md.massN = 1 / md.massN;
      row.impulse.impulseP = 0;
    }
  }
  /** 一次 split impulse 位置迭代：修改伪速度，把穿透推开，不影响真实速度 */
  solvePositionSplitImpulse() {
    const info = this.info, b1 = this.b1, b2 = this.b2;
    let lv1X = b1.pvx, lv1Y = b1.pvy, lv1Z = b1.pvz, lv2X = b2.pvx, lv2Y = b2.pvy, lv2Z = b2.pvz;
    let av1X = b1.pwx, av1Y = b1.pwy, av1Z = b1.pwz, av2X = b2.pwx, av2Y = b2.pwy, av2Z = b2.pwz;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], md = this.massData[i], imp = row.impulse, j = row.jacobianN;
      let rvn = 0;
      rvn += lv1X * j.lin1X + lv1Y * j.lin1Y + lv1Z * j.lin1Z;
      rvn -= lv2X * j.lin2X + lv2Y * j.lin2Y + lv2Z * j.lin2Z;
      rvn += av1X * j.ang1X + av1Y * j.ang1Y + av1Z * j.ang1Z;
      rvn -= av2X * j.ang2X + av2Y * j.ang2Y + av2Z * j.ang2Z;
      let impulseP = (row.rhs - rvn) * md.massN * POSITION_SPLIT_IMPULSE_BAUMGARTE;
      const oldImpulseP = imp.impulseP;
      imp.impulseP += impulseP;
      if (imp.impulseP < 0) imp.impulseP = 0;
      impulseP = imp.impulseP - oldImpulseP;
      lv1X += md.invMLinN1X * impulseP;
      lv1Y += md.invMLinN1Y * impulseP;
      lv1Z += md.invMLinN1Z * impulseP;
      lv2X += md.invMLinN2X * -impulseP;
      lv2Y += md.invMLinN2Y * -impulseP;
      lv2Z += md.invMLinN2Z * -impulseP;
      av1X += md.invMAngN1X * impulseP;
      av1Y += md.invMAngN1Y * impulseP;
      av1Z += md.invMAngN1Z * impulseP;
      av2X += md.invMAngN2X * -impulseP;
      av2Y += md.invMAngN2Y * -impulseP;
      av2Z += md.invMAngN2Z * -impulseP;
    }
    b1.pvx = lv1X;
    b1.pvy = lv1Y;
    b1.pvz = lv1Z;
    b2.pvx = lv2X;
    b2.pvy = lv2Y;
    b2.pvz = lv2Z;
    b1.pwx = av1X;
    b1.pwy = av1Y;
    b1.pwz = av1Z;
    b2.pwx = av2X;
    b2.pwy = av2Y;
    b2.pwz = av2Z;
  }
  /** 求解结束：记下摩擦冲量向量（下一步热启动用），把接触冲量累加到刚体上 */
  postSolve() {
    const info = this.info, b1 = this.b1, b2 = this.b2;
    let lin1X = 0, lin1Y = 0, lin1Z = 0;
    let ang1X = 0, ang1Y = 0, ang1Z = 0;
    let ang2X = 0, ang2Y = 0, ang2Z = 0;
    for (let i = 0; i < info.numRows; i++) {
      const row = info.rows[i], imp = row.impulse;
      const jn = row.jacobianN, jt = row.jacobianT, jb = row.jacobianB;
      const impN = imp.impulseN, impT = imp.impulseT, impB = imp.impulseB;
      let lX = 0, lY = 0, lZ = 0;
      lX += jt.lin1X * impT;
      lY += jt.lin1Y * impT;
      lZ += jt.lin1Z * impT;
      lX += jb.lin1X * impB;
      lY += jb.lin1Y * impB;
      lZ += jb.lin1Z * impB;
      imp.impulseLX = lX;
      imp.impulseLY = lY;
      imp.impulseLZ = lZ;
      lin1X += jn.lin1X * impN;
      lin1Y += jn.lin1Y * impN;
      lin1Z += jn.lin1Z * impN;
      ang1X += jn.ang1X * impN;
      ang1Y += jn.ang1Y * impN;
      ang1Z += jn.ang1Z * impN;
      ang2X += jn.ang2X * impN;
      ang2Y += jn.ang2Y * impN;
      ang2Z += jn.ang2Z * impN;
      lin1X += jt.lin1X * impT;
      lin1Y += jt.lin1Y * impT;
      lin1Z += jt.lin1Z * impT;
      ang1X += jt.ang1X * impT;
      ang1Y += jt.ang1Y * impT;
      ang1Z += jt.ang1Z * impT;
      ang2X += jt.ang2X * impT;
      ang2Y += jt.ang2Y * impT;
      ang2Z += jt.ang2Z * impT;
      lin1X += jb.lin1X * impB;
      lin1Y += jb.lin1Y * impB;
      lin1Z += jb.lin1Z * impB;
      ang1X += jb.ang1X * impB;
      ang1Y += jb.ang1Y * impB;
      ang1Z += jb.ang1Z * impB;
      ang2X += jb.ang2X * impB;
      ang2Y += jb.ang2Y * impB;
      ang2Z += jb.ang2Z * impB;
    }
    b1.lciX += lin1X;
    b1.lciY += lin1Y;
    b1.lciZ += lin1Z;
    b1.aciX += ang1X;
    b1.aciY += ang1Y;
    b1.aciZ += ang1Z;
    b2.lciX -= lin1X;
    b2.lciY -= lin1Y;
    b2.lciZ -= lin1Z;
    b2.aciX -= ang2X;
    b2.aciY -= ang2Y;
    b2.aciZ -= ang2Z;
    this.constraint.syncManifold();
  }
  /** 此阶段无额外接触处理。 */
  postSolveVelocity() {
  }
  /** 接触支持 Baumgarte 和 split impulse，此 NGS 接口为空实现。 */
  solvePositionNgs() {
  }
};

// src/core/dynamics/constraint/contact/contact-constraint.ts
function setJacobian(j, dX, dY, dZ, p) {
  j.lin1X = dX;
  j.lin1Y = dY;
  j.lin1Z = dZ;
  j.lin2X = dX;
  j.lin2Y = dY;
  j.lin2Z = dZ;
  j.ang1X = p.rp1Y * dZ - p.rp1Z * dY;
  j.ang1Y = p.rp1Z * dX - p.rp1X * dZ;
  j.ang1Z = p.rp1X * dY - p.rp1Y * dX;
  j.ang2X = p.rp2Y * dZ - p.rp2Z * dY;
  j.ang2Y = p.rp2Z * dX - p.rp2X * dZ;
  j.ang2Z = p.rp2X * dY - p.rp2Y * dX;
}
var ContactConstraint = class {
  constructor(manifold) {
    /** 这一步用的位置修正算法（Contact 按穿透深度选择） */
    this.positionCorrectionAlgorithm = DEFAULT_CONTACT_POSITION_CORRECTION_ALGORITHM;
    /** true 时恢复系数取两个形状的较大值，false 时取几何平均。由 ContactManager 在创建接触时设置 */
    this.maxRestitution = false;
    this.s1 = null;
    this.s2 = null;
    this.b1 = null;
    this.b2 = null;
    this.tf1 = null;
    this.tf2 = null;
    /** 最近参与求解的步进编号，-1 表示尚未参与。预测接触是否生效由 pushed 在求解后确定。 */
    this.solvedStamp = -1;
    /** 最近一次求解是否施加过正的法向冲量。 */
    this.pushed = false;
    /** 最近一次求解开始时的最大法向靠近速度，单位为米/秒，用于撞击判定。 */
    this.approachSpeed = 0;
    /** 上次求解的子步时长，用于缩放热启动冲量。 */
    this.substepDt = 0;
    this.solver = new PgsContactConstraintSolver(this);
    this.manifold = manifold;
    this.softSolver = new SoftContactSolver(this);
  }
  attach(s1, s2) {
    this.s1 = s1;
    this.s2 = s2;
    this.b1 = s1.rigidBody;
    this.b2 = s2.rigidBody;
    this.tf1 = this.b1.transform;
    this.tf2 = this.b2.transform;
    this.solvedStamp = -1;
    this.pushed = false;
    this.approachSpeed = 0;
    this.substepDt = 0;
  }
  detach() {
    this.s1 = null;
    this.s2 = null;
    this.b1 = null;
    this.b2 = null;
    this.tf1 = null;
    this.tf2 = null;
  }
  /** 速度求解的数据：每个没分开的接触点一行 */
  getVelocitySolverInfo(timeStep, info) {
    const b1 = this.b1, b2 = this.b2, m = this.manifold;
    info.b1 = b1;
    info.b2 = b2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ;
    const tX = m.tX, tY = m.tY, tZ = m.tZ;
    const bX = m.bX, bY = m.bY, bZ = m.bZ;
    const friction = Math.sqrt(this.s1.friction * this.s2.friction);
    const restitution = this.maxRestitution ? Math.max(this.s1.restitution, this.s2.restitution) : Math.sqrt(this.s1.restitution * this.s2.restitution);
    info.numRows = 0;
    for (let i = 0; i < m.numPoints; i++) {
      const p = m.points[i];
      if (p.depth < 0) {
        p.disabled = true;
        p.impulse.clear();
        continue;
      }
      p.disabled = false;
      const row = info.rows[info.numRows++];
      row.friction = friction;
      setJacobian(row.jacobianN, nX, nY, nZ, p);
      setJacobian(row.jacobianT, tX, tY, tZ, p);
      setJacobian(row.jacobianB, bX, bY, bZ, p);
      const j = row.jacobianN;
      const rvn = j.lin1X * b1.vx + j.lin1Y * b1.vy + j.lin1Z * b1.vz + (j.ang1X * b1.wx + j.ang1Y * b1.wy + j.ang1Z * b1.wz) - (j.lin2X * b2.vx + j.lin2Y * b2.vy + j.lin2Z * b2.vz + (j.ang2X * b2.wx + j.ang2Y * b2.wy + j.ang2Z * b2.wz));
      if (rvn < -CONTACT_ENABLE_BOUNCE_THRESHOLD && !p.warmStarted) {
        row.rhs = -rvn * restitution;
      } else {
        row.rhs = 0;
      }
      if (this.positionCorrectionAlgorithm === PositionCorrectionAlgorithm.BAUMGARTE && p.depth > LINEAR_SLOP) {
        const minRhs = (p.depth - LINEAR_SLOP) * VELOCITY_BAUMGARTE * timeStep.invDt;
        if (row.rhs < minRhs) row.rhs = minRhs;
      }
      if (!p.warmStarted) p.impulse.clear();
      row.impulse = p.impulse;
    }
  }
  /** 位置修正（split impulse）的数据：每个点只有法向一行 */
  getPositionSolverInfo(info) {
    const m = this.manifold;
    info.b1 = this.b1;
    info.b2 = this.b2;
    const nX = m.nX, nY = m.nY, nZ = m.nZ;
    info.numRows = 0;
    for (let i = 0; i < m.numPoints; i++) {
      const p = m.points[i];
      if (p.disabled) continue;
      const row = info.rows[info.numRows++];
      setJacobian(row.jacobianN, nX, nY, nZ, p);
      row.rhs = p.depth - LINEAR_SLOP;
      if (row.rhs < 0) row.rhs = 0;
      row.impulse = p.impulse;
    }
  }
  /** 刚体移动后同步接触点，供后续求解使用。 */
  syncManifold() {
    this.manifold.updateDepthsAndPositions(this.tf1, this.tf2);
  }
  /** 是否还有接触点在接触 */
  isTouching() {
    for (let i = 0; i < this.manifold.numPoints; i++) {
      if (this.manifold.points[i].depth >= 0) return true;
    }
    return false;
  }
};

// src/core/dynamics/constraint/contact/contact-impulse.ts
var ContactImpulse = class {
  constructor() {
    /** 法向冲量 */
    this.impulseN = 0;
    /** 两个切向（tangent、binormal）的摩擦冲量 */
    this.impulseT = 0;
    this.impulseB = 0;
    /** 位置修正（split impulse）的冲量 */
    this.impulseP = 0;
    /** 摩擦冲量在世界坐标里的向量。切向每步都会重建，热启动时把它投影到新的切向上 */
    this.impulseLX = 0;
    this.impulseLY = 0;
    this.impulseLZ = 0;
  }
  clear() {
    this.impulseN = 0;
    this.impulseT = 0;
    this.impulseB = 0;
    this.impulseP = 0;
    this.impulseLX = 0;
    this.impulseLY = 0;
    this.impulseLZ = 0;
  }
  /** 复制另一个冲量。不复制 impulseP */
  copyFrom(imp) {
    this.impulseN = imp.impulseN;
    this.impulseT = imp.impulseT;
    this.impulseB = imp.impulseB;
    this.impulseLX = imp.impulseLX;
    this.impulseLY = imp.impulseLY;
    this.impulseLZ = imp.impulseLZ;
  }
};

// src/core/dynamics/constraint/contact/manifold-point.ts
var ManifoldPoint = class {
  constructor() {
    /** 刚体局部坐标 */
    this.lp1X = 0;
    this.lp1Y = 0;
    this.lp1Z = 0;
    this.lp2X = 0;
    this.lp2Y = 0;
    this.lp2Z = 0;
    /** 相对刚体重心、世界方向的位置 */
    this.rp1X = 0;
    this.rp1Y = 0;
    this.rp1Z = 0;
    this.rp2X = 0;
    this.rp2Y = 0;
    this.rp2Z = 0;
    /** 世界坐标 */
    this.p1X = 0;
    this.p1Y = 0;
    this.p1Z = 0;
    this.p2X = 0;
    this.p2Y = 0;
    this.p2Z = 0;
    this.depth = 0;
    this.impulse = new ContactImpulse();
    /** 冲量是从上一步接过来的 */
    this.warmStarted = false;
    /** 已分离接触点的停用标志。 */
    this.disabled = false;
    this.id = -1;
  }
  clear() {
    this.lp1X = 0;
    this.lp1Y = 0;
    this.lp1Z = 0;
    this.lp2X = 0;
    this.lp2Y = 0;
    this.lp2Z = 0;
    this.rp1X = 0;
    this.rp1Y = 0;
    this.rp1Z = 0;
    this.rp2X = 0;
    this.rp2Y = 0;
    this.rp2Z = 0;
    this.p1X = 0;
    this.p1Y = 0;
    this.p1Z = 0;
    this.p2X = 0;
    this.p2Y = 0;
    this.p2Z = 0;
    this.depth = 0;
    this.impulse.clear();
    this.warmStarted = false;
    this.disabled = false;
    this.id = -1;
  }
  /** 用检测结果初始化 */
  initialize(r, tf1, tf2) {
    this.p1X = r.p1x;
    this.p1Y = r.p1y;
    this.p1Z = r.p1z;
    this.p2X = r.p2x;
    this.p2Y = r.p2y;
    this.p2Z = r.p2z;
    this.rp1X = this.p1X - tf1.px;
    this.rp1Y = this.p1Y - tf1.py;
    this.rp1Z = this.p1Z - tf1.pz;
    this.rp2X = this.p2X - tf2.px;
    this.rp2Y = this.p2Y - tf2.py;
    this.rp2Z = this.p2Z - tf2.pz;
    this.lp1X = tf1.r00 * this.rp1X + tf1.r10 * this.rp1Y + tf1.r20 * this.rp1Z;
    this.lp1Y = tf1.r01 * this.rp1X + tf1.r11 * this.rp1Y + tf1.r21 * this.rp1Z;
    this.lp1Z = tf1.r02 * this.rp1X + tf1.r12 * this.rp1Y + tf1.r22 * this.rp1Z;
    this.lp2X = tf2.r00 * this.rp2X + tf2.r10 * this.rp2Y + tf2.r20 * this.rp2Z;
    this.lp2Y = tf2.r01 * this.rp2X + tf2.r11 * this.rp2Y + tf2.r21 * this.rp2Z;
    this.lp2Z = tf2.r02 * this.rp2X + tf2.r12 * this.rp2Y + tf2.r22 * this.rp2Z;
    this.depth = r.depth;
    this.impulse.clear();
    this.id = r.id;
    this.warmStarted = false;
    this.disabled = false;
  }
  /** 用新的检测结果更新位置和深度，保留冲量、编号（增量更新用） */
  updateDepthAndPositions(r, tf1, tf2) {
    this.p1X = r.p1x;
    this.p1Y = r.p1y;
    this.p1Z = r.p1z;
    this.p2X = r.p2x;
    this.p2Y = r.p2y;
    this.p2Z = r.p2z;
    this.rp1X = this.p1X - tf1.px;
    this.rp1Y = this.p1Y - tf1.py;
    this.rp1Z = this.p1Z - tf1.pz;
    this.rp2X = this.p2X - tf2.px;
    this.rp2Y = this.p2Y - tf2.py;
    this.rp2Z = this.p2Z - tf2.pz;
    this.lp1X = tf1.r00 * this.rp1X + tf1.r10 * this.rp1Y + tf1.r20 * this.rp1Z;
    this.lp1Y = tf1.r01 * this.rp1X + tf1.r11 * this.rp1Y + tf1.r21 * this.rp1Z;
    this.lp1Z = tf1.r02 * this.rp1X + tf1.r12 * this.rp1Y + tf1.r22 * this.rp1Z;
    this.lp2X = tf2.r00 * this.rp2X + tf2.r10 * this.rp2Y + tf2.r20 * this.rp2Z;
    this.lp2Y = tf2.r01 * this.rp2X + tf2.r11 * this.rp2Y + tf2.r21 * this.rp2Z;
    this.lp2Z = tf2.r02 * this.rp2X + tf2.r12 * this.rp2Y + tf2.r22 * this.rp2Z;
    this.depth = r.depth;
  }
  /** 复制另一个点。disabled 总是清掉 */
  copyFrom(cp) {
    this.lp1X = cp.lp1X;
    this.lp1Y = cp.lp1Y;
    this.lp1Z = cp.lp1Z;
    this.lp2X = cp.lp2X;
    this.lp2Y = cp.lp2Y;
    this.lp2Z = cp.lp2Z;
    this.rp1X = cp.rp1X;
    this.rp1Y = cp.rp1Y;
    this.rp1Z = cp.rp1Z;
    this.rp2X = cp.rp2X;
    this.rp2Y = cp.rp2Y;
    this.rp2Z = cp.rp2Z;
    this.p1X = cp.p1X;
    this.p1Y = cp.p1Y;
    this.p1Z = cp.p1Z;
    this.p2X = cp.p2X;
    this.p2Y = cp.p2Y;
    this.p2Z = cp.p2Z;
    this.depth = cp.depth;
    this.impulse.copyFrom(cp.impulse);
    this.id = cp.id;
    this.warmStarted = cp.warmStarted;
    this.disabled = false;
  }
};

// src/core/dynamics/constraint/contact/manifold.ts
var Manifold = class {
  constructor() {
    /** 法线、切向、副切向，三者两两正交 */
    this.nX = 0;
    this.nY = 0;
    this.nZ = 0;
    this.tX = 0;
    this.tY = 0;
    this.tZ = 0;
    this.bX = 0;
    this.bY = 0;
    this.bZ = 0;
    this.numPoints = 0;
    this.points = [];
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) this.points.push(new ManifoldPoint());
  }
  clear() {
    for (let i = 0; i < this.numPoints; i++) this.points[i].clear();
    this.numPoints = 0;
  }
  /** 由法线构造正交基：用法线分量最小的那根轴构造切向，数值最稳 */
  buildBasis(nx, ny, nz) {
    this.nX = nx;
    this.nY = ny;
    this.nZ = nz;
    const nx2 = nx * nx, ny2 = ny * ny, nz2 = nz * nz;
    let tx, ty, tz, bx, by, bz;
    const axis = nx2 < ny2 ? nx2 < nz2 ? 0 : 2 : ny2 < nz2 ? 1 : 2;
    if (axis === 0) {
      const invL = 1 / Math.sqrt(ny2 + nz2);
      tx = 0;
      ty = -nz * invL;
      tz = ny * invL;
      bx = ny * tz - nz * ty;
      by = -nx * tz;
      bz = nx * ty;
    } else if (axis === 1) {
      const invL = 1 / Math.sqrt(nx2 + nz2);
      tx = nz * invL;
      ty = 0;
      tz = -nx * invL;
      bx = ny * tz;
      by = nz * tx - nx * tz;
      bz = -ny * tx;
    } else {
      const invL = 1 / Math.sqrt(nx2 + ny2);
      tx = -ny * invL;
      ty = nx * invL;
      tz = 0;
      bx = -nz * ty;
      by = nz * tx;
      bz = nx * ty - ny * tx;
    }
    this.tX = tx;
    this.tY = ty;
    this.tZ = tz;
    this.bX = bx;
    this.bY = by;
    this.bZ = bz;
  }
  /** 刚体移动后，按局部坐标重新算接触点的世界位置和深度 */
  updateDepthsAndPositions(tf1, tf2) {
    for (let i = 0; i < this.numPoints; i++) {
      const p = this.points[i];
      p.rp1X = tf1.r00 * p.lp1X + tf1.r01 * p.lp1Y + tf1.r02 * p.lp1Z;
      p.rp1Y = tf1.r10 * p.lp1X + tf1.r11 * p.lp1Y + tf1.r12 * p.lp1Z;
      p.rp1Z = tf1.r20 * p.lp1X + tf1.r21 * p.lp1Y + tf1.r22 * p.lp1Z;
      p.rp2X = tf2.r00 * p.lp2X + tf2.r01 * p.lp2Y + tf2.r02 * p.lp2Z;
      p.rp2Y = tf2.r10 * p.lp2X + tf2.r11 * p.lp2Y + tf2.r12 * p.lp2Z;
      p.rp2Z = tf2.r20 * p.lp2X + tf2.r21 * p.lp2Y + tf2.r22 * p.lp2Z;
      p.p1X = p.rp1X + tf1.px;
      p.p1Y = p.rp1Y + tf1.py;
      p.p1Z = p.rp1Z + tf1.pz;
      p.p2X = p.rp2X + tf2.px;
      p.p2Y = p.rp2Y + tf2.py;
      p.p2Z = p.rp2Z + tf2.pz;
      const dX = p.p1X - p.p2X, dY = p.p1Y - p.p2Y, dZ = p.p1Z - p.p2Z;
      const dotN = dX * this.nX + dY * this.nY + dZ * this.nZ;
      p.depth = -dotN;
    }
  }
};

// src/core/dynamics/constraint/contact/manifold-updater.ts
function quadAreaFast(p1X, p1Y, p1Z, p2X, p2Y, p2Z, p3X, p3Y, p3Z, p4X, p4Y, p4Z) {
  const v12X = p2X - p1X, v12Y = p2Y - p1Y, v12Z = p2Z - p1Z;
  const v34X = p4X - p3X, v34Y = p4Y - p3Y, v34Z = p4Z - p3Z;
  const v13X = p3X - p1X, v13Y = p3Y - p1Y, v13Z = p3Z - p1Z;
  const v24X = p4X - p2X, v24Y = p4Y - p2Y, v24Z = p4Z - p2Z;
  const v14X = p4X - p1X, v14Y = p4Y - p1Y, v14Z = p4Z - p1Z;
  const v23X = p3X - p2X, v23Y = p3Y - p2Y, v23Z = p3Z - p2Z;
  const c1X = v12Y * v34Z - v12Z * v34Y, c1Y = v12Z * v34X - v12X * v34Z, c1Z = v12X * v34Y - v12Y * v34X;
  const c2X = v13Y * v24Z - v13Z * v24Y, c2Y = v13Z * v24X - v13X * v24Z, c2Z = v13X * v24Y - v13Y * v24X;
  const c3X = v14Y * v23Z - v14Z * v23Y, c3Y = v14Z * v23X - v14X * v23Z, c3Z = v14X * v23Y - v14Y * v23X;
  const a1 = c1X * c1X + c1Y * c1Y + c1Z * c1Z;
  const a2 = c2X * c2X + c2Y * c2Y + c2Z * c2Z;
  const a3 = c3X * c3X + c3Y * c3Y + c3Z * c3Z;
  if (a1 > a2) return a1 > a3 ? a1 : a3;
  return a2 > a3 ? a2 : a3;
}
var ManifoldUpdater = class {
  constructor(manifold) {
    this.oldIds = [];
    this.oldImpulses = [];
    this.oldAnchors = [];
    this.manifold = manifold;
    for (let i = 0; i < MAX_MANIFOLD_POINTS; i++) {
      this.oldIds.push(-1);
      this.oldImpulses.push(new ContactImpulse());
      this.oldAnchors.push({ ax: 0, ay: 0, az: 0, bx: 0, by: 0, bz: 0 });
    }
  }
  /** 一次性更新；子步模式按两侧局部锚点匹配，每个旧点最多使用一次。 */
  totalUpdate(result, tf1, tf2, checkAnchors = false) {
    const m = this.manifold;
    const numOldPoints = m.numPoints;
    const num = result.numPoints;
    let unchanged = !checkAnchors && num === numOldPoints;
    for (let i = 0; unchanged && i < num; i++) {
      const id = result.points[i].id;
      if (id !== m.points[i].id || i > 0 && id <= result.points[i - 1].id) unchanged = false;
    }
    if (unchanged) {
      for (let i = 0; i < num; i++) {
        const p = m.points[i];
        p.updateDepthAndPositions(result.points[i], tf1, tf2);
        p.impulse.impulseP = 0;
        p.id = result.points[i].id;
        p.warmStarted = true;
        p.disabled = false;
      }
      return;
    }
    for (let i = 0; i < numOldPoints; i++) {
      this.oldIds[i] = m.points[i].id;
      this.oldImpulses[i].copyFrom(m.points[i].impulse);
      if (checkAnchors) {
        const p = m.points[i], a = this.oldAnchors[i];
        a.ax = p.lp1X;
        a.ay = p.lp1Y;
        a.az = p.lp1Z;
        a.bx = p.lp2X;
        a.by = p.lp2Y;
        a.bz = p.lp2Z;
      }
    }
    m.numPoints = num;
    let used = 0;
    for (let i = 0; i < num; i++) {
      const p = m.points[i];
      p.initialize(result.points[i], tf1, tf2);
      for (let k = 0; k < numOldPoints; k++) {
        if (checkAnchors || p.id === this.oldIds[k]) {
          if (checkAnchors) {
            if ((used & 1 << k) !== 0) continue;
            const a = this.oldAnchors[k];
            const dx = p.lp1X - a.ax, dy = p.lp1Y - a.ay, dz = p.lp1Z - a.az;
            const ex = p.lp2X - a.bx, ey = p.lp2Y - a.by, ez = p.lp2Z - a.bz;
            if (dx * dx + dy * dy + dz * dz >= 25e-4 || ex * ex + ey * ey + ez * ez >= 25e-4) continue;
            used |= 1 << k;
          }
          p.impulse.copyFrom(this.oldImpulses[k]);
          p.warmStarted = true;
          break;
        }
      }
    }
  }
  /**
   * 增量更新接触流形：相近点替换，新增点加入，分离或切向偏移过大的点移除。
   * 流形最多保留 4 个点，法向分离阈值包含预测接触余量 margin。
   */
  incrementalUpdate(result, tf1, tf2, margin = 0) {
    const m = this.manifold;
    m.updateDepthsAndPositions(tf1, tf2);
    for (let i = 0; i < m.numPoints; i++) m.points[i].warmStarted = true;
    const newPoint = result.points[0];
    const index = this.findNearestContactPointIndex(newPoint, tf1, tf2);
    if (index === -1) {
      this.addManifoldPoint(newPoint, tf1, tf2);
    } else {
      m.points[index].updateDepthAndPositions(newPoint, tf1, tf2);
    }
    this.removeOutdatedPoints(margin);
  }
  /** 删掉沿法线分开超过阈值、或者沿切向滑开超过阈值的点 */
  removeOutdatedPoints(margin) {
    const m = this.manifold;
    let index = m.numPoints;
    while (--index >= 0) {
      const p = m.points[index];
      let dX = p.p1X - p.p2X, dY = p.p1Y - p.p2Y, dZ = p.p1Z - p.p2Z;
      const dotN = m.nX * dX + m.nY * dY + m.nZ * dZ;
      if (dotN > CONTACT_PERSISTENCE_THRESHOLD + margin) {
        this.removeManifoldPoint(index);
        continue;
      }
      dX += m.nX * -dotN;
      dY += m.nY * -dotN;
      dZ += m.nZ * -dotN;
      if (dX * dX + dY * dY + dZ * dZ > CONTACT_PERSISTENCE_THRESHOLD * CONTACT_PERSISTENCE_THRESHOLD) {
        this.removeManifoldPoint(index);
        continue;
      }
    }
  }
  /** 把第 index 个点和最后一个点交换，再删掉最后一个 */
  removeManifoldPoint(index) {
    const m = this.manifold;
    const lastIndex = --m.numPoints;
    if (index !== lastIndex) {
      const tmp = m.points[index];
      m.points[index] = m.points[lastIndex];
      m.points[lastIndex] = tmp;
    }
    m.points[lastIndex].clear();
  }
  addManifoldPoint(point2, tf1, tf2) {
    const m = this.manifold;
    const num = m.numPoints;
    if (num === MAX_MANIFOLD_POINTS) {
      m.points[this.computeTargetIndex(point2, tf1)].initialize(point2, tf1, tf2);
      return;
    }
    m.points[num].initialize(point2, tf1, tf2);
    m.numPoints++;
  }
  /** 流形满时保留最深点，并选择替换后四边形面积最大的组合。 */
  computeTargetIndex(newPoint, tf1) {
    const ps = this.manifold.points;
    const p1 = ps[0], p2 = ps[1], p3 = ps[2], p4 = ps[3];
    let maxDepth = p1.depth;
    let maxDepthIndex = 0;
    if (p2.depth > maxDepth) {
      maxDepth = p2.depth;
      maxDepthIndex = 1;
    }
    if (p3.depth > maxDepth) {
      maxDepth = p3.depth;
      maxDepthIndex = 2;
    }
    if (p4.depth > maxDepth) maxDepthIndex = 3;
    const rX = newPoint.p1x - tf1.px, rY = newPoint.p1y - tf1.py, rZ = newPoint.p1z - tf1.pz;
    const a1 = quadAreaFast(p2.rp1X, p2.rp1Y, p2.rp1Z, p3.rp1X, p3.rp1Y, p3.rp1Z, p4.rp1X, p4.rp1Y, p4.rp1Z, rX, rY, rZ);
    const a2 = quadAreaFast(p1.rp1X, p1.rp1Y, p1.rp1Z, p3.rp1X, p3.rp1Y, p3.rp1Z, p4.rp1X, p4.rp1Y, p4.rp1Z, rX, rY, rZ);
    const a3 = quadAreaFast(p1.rp1X, p1.rp1Y, p1.rp1Z, p2.rp1X, p2.rp1Y, p2.rp1Z, p4.rp1X, p4.rp1Y, p4.rp1Z, rX, rY, rZ);
    const a4 = quadAreaFast(p1.rp1X, p1.rp1Y, p1.rp1Z, p2.rp1X, p2.rp1Y, p2.rp1Z, p3.rp1X, p3.rp1Y, p3.rp1Z, rX, rY, rZ);
    let max = a1;
    let target = 0;
    if (a2 > max && maxDepthIndex !== 1 || maxDepthIndex === 0) {
      max = a2;
      target = 1;
    }
    if (a3 > max && maxDepthIndex !== 2) {
      max = a3;
      target = 2;
    }
    if (a4 > max && maxDepthIndex !== 3) target = 3;
    return target;
  }
  /** 流形里离新点最近（两个刚体上的距离取较小的）且在保留阈值内的点，没有返回 -1 */
  findNearestContactPointIndex(target, tf1, tf2) {
    const m = this.manifold;
    let nearestSq = CONTACT_PERSISTENCE_THRESHOLD * CONTACT_PERSISTENCE_THRESHOLD;
    let idx = -1;
    for (let i = 0; i < m.numPoints; i++) {
      const mp = m.points[i];
      const rp1X = target.p1x - tf1.px, rp1Y = target.p1y - tf1.py, rp1Z = target.p1z - tf1.pz;
      const rp2X = target.p2x - tf2.px, rp2Y = target.p2y - tf2.py, rp2Z = target.p2z - tf2.pz;
      const d1X = mp.rp1X - rp1X, d1Y = mp.rp1Y - rp1Y, d1Z = mp.rp1Z - rp1Z;
      const d2X = mp.rp2X - rp2X, d2Y = mp.rp2Y - rp2Y, d2Z = mp.rp2Z - rp2Z;
      const sq1 = d1X * d1X + d1Y * d1Y + d1Z * d1Z;
      const sq2 = d2X * d2X + d2Y * d2Y + d2Z * d2Z;
      const d = sq1 < sq2 ? sq1 : sq2;
      if (d < nearestSq) {
        nearestSq = d;
        idx = i;
      }
    }
    return idx;
  }
};

// src/core/dynamics/contact.ts
var softBoxDetector = new SoftBoxBoxDetector();
var ContactLink = class {
  constructor() {
    this.prev = null;
    this.next = null;
    this.contact = null;
    this.other = null;
  }
};
function pushLink(b, l) {
  if (b.contactLinkList === null) {
    b.contactLinkList = l;
    b.contactLinkListLast = l;
  } else {
    b.contactLinkListLast.next = l;
    l.prev = b.contactLinkListLast;
    b.contactLinkListLast = l;
  }
}
function removeLink2(b, l) {
  const prev = l.prev, next = l.next;
  if (prev !== null) prev.next = next;
  if (next !== null) next.prev = prev;
  if (l === b.contactLinkList) b.contactLinkList = next;
  if (l === b.contactLinkListLast) b.contactLinkListLast = prev;
  l.next = null;
  l.prev = null;
}
var Contact = class {
  constructor() {
    this.prev = null;
    this.next = null;
    this.link1 = new ContactLink();
    this.link2 = new ContactLink();
    this.s1 = null;
    this.s2 = null;
    this.b1 = null;
    this.b2 = null;
    this.detector = null;
    this.detectorResult = new DetectorResult();
    /** 这对形状跨步保存的窄相数据（GJK 缓存）。Contact 对象复用时在 detach 里清空 */
    this.cachedDetectorData = new CachedDetectorData();
    /** 这一步宽相又收集到了这一对 */
    this.latest = false;
    /** 这一步跳过窄相检测 */
    this.shouldBeSkipped = false;
    this.manifold = new Manifold();
    this.updater = new ManifoldUpdater(this.manifold);
    this.contactConstraint = new ContactConstraint(this.manifold);
    /** 两个形状是否真的接触 */
    this.touching = false;
  }
  attach(s1, s2, detector) {
    this.s1 = s1;
    this.s2 = s2;
    const b1 = this.b1 = s1.rigidBody, b2 = this.b2 = s2.rigidBody;
    this.touching = false;
    pushLink(b1, this.link1);
    pushLink(b2, this.link2);
    b1.numContactLinks++;
    b2.numContactLinks++;
    this.link1.other = b2;
    this.link2.other = b1;
    this.link1.contact = this;
    this.link2.contact = this;
    this.detector = detector;
    this.contactConstraint.attach(s1, s2);
  }
  detach() {
    const b1 = this.b1, b2 = this.b2;
    removeLink2(b1, this.link1);
    removeLink2(b2, this.link2);
    b1.numContactLinks--;
    b2.numContactLinks--;
    this.link1.other = null;
    this.link2.other = null;
    this.link1.contact = null;
    this.link2.contact = null;
    this.s1 = null;
    this.s2 = null;
    this.b1 = null;
    this.b2 = null;
    this.touching = false;
    this.cachedDetectorData.clear();
    this.manifold.clear();
    this.detector = null;
    this.contactConstraint.detach();
  }
  /**
   * 执行窄相检测并更新流形。
   * 预测接触模式使用形状的 speculativeMargin，touching 延迟至求解结束后更新。
   */
  updateManifold(speculative = false) {
    if (this.detector === null) return;
    const s1 = this.s1, s2 = this.s2, result = this.detectorResult;
    const b1 = this.b1, b2 = this.b2;
    const boxPair = s1.geom.type === GeometryType.BOX && s2.geom.type === GeometryType.BOX;
    const detector = speculative && boxPair ? softBoxDetector : this.detector;
    let margin = s1.speculativeMargin + s2.speculativeMargin;
    if (margin > 0 && b1.type !== RigidBodyType.STATIC && b2.type !== RigidBodyType.STATIC) {
      const w = b1.world;
      const dvX = b2.vx - b1.vx, dvY = b2.vy - b1.vy, dvZ = b2.vz - b1.vz;
      let speed = Math.sqrt(dvX * dvX + dvY * dvY + dvZ * dvZ) + Math.sqrt(b1.wx * b1.wx + b1.wy * b1.wy + b1.wz * b1.wz) + Math.sqrt(b2.wx * b2.wx + b2.wy * b2.wy + b2.wz * b2.wz);
      if (speed > w.pairSpeculativeSpeed) speed = w.pairSpeculativeSpeed;
      margin = w.speculativeDistance + w.speculativeTick() * speed;
      if (margin > w.maxSpeculativeDistance) margin = w.maxSpeculativeDistance;
    }
    if (speculative && b1.type !== RigidBodyType.STATIC && b2.type === RigidBodyType.STATIC && boxPair) {
      detector.detect(result, s2.geom, s1.geom, s2.transform, s1.transform, this.cachedDetectorData, margin);
      result.nx = -result.nx;
      result.ny = -result.ny;
      result.nz = -result.nz;
      for (let i = 0; i < result.numPoints; i++) {
        const p = result.points[i], x = p.p1x, y = p.p1y, z = p.p1z;
        p.p1x = p.p2x;
        p.p1y = p.p2y;
        p.p1z = p.p2z;
        p.p2x = x;
        p.p2y = y;
        p.p2z = z;
      }
    } else {
      detector.detect(result, s1.geom, s2.geom, s1.transform, s2.transform, this.cachedDetectorData, margin);
    }
    if (s1.contactFilter !== null) s1.contactFilter(s1, result, true, s2);
    if (s2.contactFilter !== null) s2.contactFilter(s2, result, false, s1);
    const found = result.numPoints > 0;
    if (!speculative) this.touching = found;
    if (found) {
      if (speculative && this.manifold.nX * result.nx + this.manifold.nY * result.ny + this.manifold.nZ * result.nz <= 0.99) {
        this.manifold.clear();
      }
      this.manifold.buildBasis(result.nx, result.ny, result.nz);
      this.contactConstraint.positionCorrectionAlgorithm = result.getMaxDepth() > ALTERNATIVE_POSITION_CORRECTION_DEPTH_THRESHOLD ? ALTERNATIVE_CONTACT_POSITION_CORRECTION_ALGORITHM : DEFAULT_CONTACT_POSITION_CORRECTION_ALGORITHM;
      if (result.incremental) {
        this.updater.incrementalUpdate(result, this.b1.transform, this.b2.transform, margin);
      } else {
        this.updater.totalUpdate(result, this.b1.transform, this.b2.transform, speculative);
      }
    } else {
      this.manifold.clear();
    }
  }
};

// src/core/dynamics/contact-manager.ts
var ContactManager = class {
  constructor(broadPhase) {
    this.numContacts = 0;
    this.contactList = null;
    this.contactListLast = null;
    this.contactPool = null;
    this.collisionMatrix = new CollisionMatrix();
    this.maxRestitutionFlag = false;
    /** 接触事件监听者，null 表示不发送通知。 */
    this.listener = null;
    /** 预测接触模式。窄相仅更新流形，touching 和事件由求解后的 updateTouching 更新。 */
    this.speculative = false;
    this.broadPhase = broadPhase;
  }
  /** 是否使用两侧恢复系数的较大值，默认使用几何平均。修改时同步到已有接触。 */
  get maxRestitution() {
    return this.maxRestitutionFlag;
  }
  set maxRestitution(value) {
    this.maxRestitutionFlag = value;
    for (let c = this.contactList; c !== null; c = c.next) c.contactConstraint.maxRestitution = value;
  }
  /** 宽相：收集重叠的代理对，创建新接触，销毁过期的接触 */
  updateContacts() {
    this.broadPhase.collectPairs();
    this.createContacts();
    this.destroyOutdatedContacts();
  }
  /** 窄相：更新不需要跳过的接触的流形 */
  updateManifolds() {
    const listener = this.listener;
    if (this.speculative) {
      for (let c = this.contactList; c !== null; c = c.next) if (!c.shouldBeSkipped) c.updateManifold(true);
      return;
    }
    for (let c = this.contactList; c !== null; c = c.next) {
      if (c.shouldBeSkipped) continue;
      if (listener === null) {
        c.updateManifold();
      } else {
        const was = c.touching;
        c.updateManifold();
        if (c.touching !== was) {
          if (c.touching) listener.beginContact(c);
          else listener.endContact(c);
        }
      }
    }
  }
  /**
   * 求解后更新接触状态并通知。stamp 为本次步进编号。
   * 法向冲量大于 0 的点视为接触；空流形视为分离，未参与求解的接触保留状态。
   */
  updateTouching(stamp) {
    const listener = this.listener;
    for (let c = this.contactList; c !== null; c = c.next) {
      const cc = c.contactConstraint;
      let touching;
      if (c.manifold.numPoints === 0) touching = false;
      else if (cc.solvedStamp === stamp) touching = cc.pushed;
      else continue;
      if (touching === c.touching) continue;
      c.touching = touching;
      if (listener !== null) {
        if (touching) listener.beginContact(c);
        else listener.endContact(c);
      }
    }
  }
  destroyContact(c) {
    const prev = c.prev, next = c.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (c === this.contactList) this.contactList = next;
    if (c === this.contactListLast) this.contactListLast = prev;
    c.next = null;
    c.prev = null;
    if (this.listener !== null && c.touching) this.listener.endContact(c);
    c.detach();
    c.next = this.contactPool;
    this.contactPool = c;
    this.numContacts--;
  }
  createContacts() {
    for (let pp = this.broadPhase.proxyPairList; pp !== null; pp = pp.next) {
      const p1 = pp.p1, p2 = pp.p2;
      const s1 = p1.id < p2.id ? p1.userData : p2.userData;
      const s2 = p1.id < p2.id ? p2.userData : p1.userData;
      if (!this.shouldCollide(s1, s2)) continue;
      const b1 = s1.rigidBody, b2 = s2.rigidBody;
      const id1 = s1.id, id2 = s2.id;
      let found = false;
      for (let l = b1.numContactLinks < b2.numContactLinks ? b1.contactLinkList : b2.contactLinkList; l !== null; l = l.next) {
        const c2 = l.contact;
        if (c2.s1.id === id1 && c2.s2.id === id2) {
          c2.latest = true;
          found = true;
          break;
        }
      }
      if (found) continue;
      let c = this.contactPool;
      if (c !== null) {
        this.contactPool = c.next;
        c.next = null;
      } else {
        c = new Contact();
      }
      if (this.contactList === null) {
        this.contactList = c;
        this.contactListLast = c;
      } else {
        this.contactListLast.next = c;
        c.prev = this.contactListLast;
        this.contactListLast = c;
      }
      c.latest = true;
      c.attach(s1, s2, this.collisionMatrix.getDetector(s1.geom.type, s2.geom.type));
      c.contactConstraint.maxRestitution = this.maxRestitutionFlag;
      this.numContacts++;
    }
  }
  destroyOutdatedContacts() {
    const incremental = this.broadPhase.incremental;
    let c = this.contactList;
    while (c !== null) {
      const next = c.next;
      if (c.latest) {
        c.latest = false;
        c.shouldBeSkipped = false;
      } else if (!incremental) {
        this.destroyContact(c);
      } else {
        const s1 = c.s1, s2 = c.s2, r1 = s1.rigidBody, r2 = s2.rigidBody;
        const active1 = !r1.sleeping && r1.type !== RigidBodyType.STATIC;
        const active2 = !r2.sleeping && r2.type !== RigidBodyType.STATIC;
        if (!active1 && !active2) {
          c.shouldBeSkipped = true;
        } else if (!this.broadPhase.isOverlapping(s1.proxy, s2.proxy) || !this.shouldCollide(s1, s2)) {
          this.destroyContact(c);
        } else {
          const a1 = s1.aabb, a2 = s2.aabb;
          c.shouldBeSkipped = !(a1.minX < a2.maxX && a1.maxX > a2.minX && a1.minY < a2.maxY && a1.maxY > a2.minY && a1.minZ < a2.maxZ && a1.maxZ > a2.minZ);
        }
      }
      c = next;
    }
  }
  shouldCollide(s1, s2) {
    const r1 = s1.rigidBody, r2 = s2.rigidBody;
    if (r1 === r2) return false;
    if (r1.type !== RigidBodyType.DYNAMIC && r2.type !== RigidBodyType.DYNAMIC) return false;
    if ((s1.collisionGroup & s2.collisionMask) === 0 || (s2.collisionGroup & s1.collisionMask) === 0) return false;
    let jl, other;
    if (r1.numJointLinks < r2.numJointLinks) {
      jl = r1.jointLinkList;
      other = r2;
    } else {
      jl = r2.jointLinkList;
      other = r1;
    }
    for (; jl !== null; jl = jl.next) {
      if (jl.other === other && !jl.joint.allowCollision) return false;
    }
    return true;
  }
};

// src/core/dynamics/island.ts
function fastInvExp(x) {
  const x2 = x * x;
  return 1 / (1 + x + x2 * (1 / 2 + x * (1 / 6) + x2 * (1 / 24)));
}
var SoftSubstepParams = class {
  /** 两个动态刚体之间、一边不会动时的软约束参数 */
  constructor(dynamic, fixed) {
    this.dynamic = dynamic;
    this.fixed = fixed;
    /** 子步数 */
    this.substeps = 1;
    /**
     * 接触带偏置、松弛各解几遍。这个 tick 有接触的靠近速度超过 impactVelocity（m/s）时，两个阶段都解 impactIterations 遍，
     * 超出常规遍数的部分仅处理被撞刚体的接触。
     */
    this.iterations = 1;
    this.relaxIterations = 1;
    this.tolerance = 0;
    this.adaptiveIterations = false;
    this.impactIterations = 0;
    this.impactVelocity = 0;
    /** 子步结束后的反弹遍数；靠近速度超过 restitutionThreshold（m/s）的接触才反弹 */
    this.restitutionIterations = 1;
    this.restitutionThreshold = 0;
    /** 每个 tick 结束时的速度上限（m/s）、角速度上限（rad/s） */
    this.maxLinearVelocity = Infinity;
    this.maxAngularVelocity = Infinity;
    /** 关节每个子步的速度迭代次数、位置迭代次数 */
    this.jointIterations = 0;
    this.positionIterations = 0;
    /** 推开穿透的最大速度（m/s） */
    this.maxPushVelocity = 0;
    /** 第一个子步的求解顺序（之后每个子步交替） */
    this.parity = 0;
    /** 这次 World.step 的编号（接触记下它，求解之后据此判断是否接触） */
    this.stamp = 0;
    /** 预测接触距离：基础值、上限（米） */
    this.speculativeDistance = 0;
    this.maxSpeculativeDistance = 0;
    /** 接触面向外移的距离（米），见 World.softContactSkin */
    this.skin = 0;
    /** 虚空海（见 World.voidSea）：是否打开、开始的高度、深度系数、阻力系数 */
    this.sea = false;
    this.seaStartY = 0;
    this.seaDepthGradient = 1;
    this.seaDrag = 1;
  }
};
var Island = class {
  constructor() {
    this.gravityX = 0;
    this.gravityY = 0;
    this.gravityZ = 0;
    this.rigidBodies = [];
    this.drags = [];
    this.solvers = [];
    /** 位置修正用 split impulse、NGS 的约束 */
    this.solversSi = [];
    this.solversNgs = [];
    /** 软接触求解器列表；关节求解器保存在 solvers 中。 */
    this.softContacts = [];
    this.orderedContacts = [];
    /** beginSubsteps 计算的带偏置、松弛及速度迭代次数，供 runSubsteps 使用。 */
    this.biasPasses = 0;
    this.relaxPasses = 0;
    this.solveTotal = 0;
    this.conditionedContacts = false;
    this.quietContacts = false;
    this.isotropicContacts = false;
    this.minMass = Infinity;
    this.maxMass = 0;
    this.extendedBody = false;
  }
  clear() {
    this.rigidBodies.length = 0;
    this.drags.length = 0;
    this.solvers.length = 0;
    this.solversSi.length = 0;
    this.solversNgs.length = 0;
    this.softContacts.length = 0;
    this.orderedContacts.length = 0;
    this.minMass = Infinity;
    this.maxMass = 0;
    this.extendedBody = false;
  }
  setGravity(x, y, z) {
    this.gravityX = x;
    this.gravityY = y;
    this.gravityZ = z;
  }
  addRigidBody(rb) {
    rb.addedToIsland = true;
    this.rigidBodies.push(rb);
    if (rb.drag !== null) this.drags.push(rb.drag);
    if (rb.type === RigidBodyType.DYNAMIC) {
      if (rb.mass < this.minMass) this.minMass = rb.mass;
      if (rb.mass > this.maxMass) this.maxMass = rb.mass;
      if (Math.max(rb.li00, rb.li11, rb.li22) > 1.5 * Math.min(rb.li00, rb.li11, rb.li22)) this.extendedBody = true;
    }
  }
  /** 大质量比接触保留支撑所需的子步数。 */
  supportSubsteps(base, minimum) {
    return base < minimum && this.softContacts.length > 0 && this.maxMass >= this.minMass * 8 ? minimum : base;
  }
  addConstraintSolver(solver, positionCorrection) {
    solver.addedToIsland = true;
    this.solvers.push(solver);
    if (positionCorrection === PositionCorrectionAlgorithm.SPLIT_IMPULSE) this.solversSi.push(solver);
    if (positionCorrection === PositionCorrectionAlgorithm.NGS) this.solversNgs.push(solver);
  }
  /** 将接触加入软接触求解列表。 */
  addSoftContact(solver, owner) {
    owner.addedToIsland = true;
    this.softContacts.push(solver);
  }
  /** 单独一个刚体（没有接触）的一步 */
  stepSingleRigidBody(timeStep, rb) {
    const dt = timeStep.dt;
    rb.ptransform.copyFrom(rb.transform);
    rb.lciX = 0;
    rb.lciY = 0;
    rb.lciZ = 0;
    rb.aciX = 0;
    rb.aciY = 0;
    rb.aciZ = 0;
    if (rb.isSleepy() && !rb.restless) {
      rb.sleepTime += dt;
      if (rb.sleepTime >= rb.sleepingTimeThreshold) rb.sleep();
    } else {
      rb.sleepTime = 0;
    }
    if (rb.sleeping) return;
    if (rb.type === RigidBodyType.DYNAMIC) this.applyForces(rb, dt);
    rb.integrate(dt);
    rb.syncShapes();
  }
  /** 多个刚体和约束的一步：速度求解 → 积分 → 位置修正 */
  step(timeStep, numVelocityIterations, numPositionIterations) {
    const dt = timeStep.dt, bodies = this.rigidBodies, solvers = this.solvers, solversSi = this.solversSi, solversNgs = this.solversNgs;
    let sleepIsland = true;
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      rb.ptransform.copyFrom(rb.transform);
      rb.lciX = 0;
      rb.lciY = 0;
      rb.lciZ = 0;
      rb.aciX = 0;
      rb.aciY = 0;
      rb.aciZ = 0;
      rb.sleeping = false;
      if (rb.isSleepy() && !rb.restless) rb.sleepTime += dt;
      else rb.sleepTime = 0;
      if (rb.sleepTime < rb.sleepingTimeThreshold) sleepIsland = false;
      if (rb.type === RigidBodyType.DYNAMIC) this.applyForces(rb, dt);
    }
    if (sleepIsland) {
      for (let i = 0; i < bodies.length; i++) bodies[i].sleep();
      return;
    }
    for (let i = 0; i < solvers.length; i++) solvers[i].preSolveVelocity(timeStep);
    for (let i = 0; i < solvers.length; i++) solvers[i].warmStart(timeStep);
    for (let t = 0; t < numVelocityIterations; t++) {
      for (let i = 0; i < solvers.length; i++) solvers[i].solveVelocity();
    }
    for (let i = 0; i < solvers.length; i++) solvers[i].postSolveVelocity(timeStep);
    for (let i = 0; i < bodies.length; i++) bodies[i].integrate(dt);
    for (let i = 0; i < solversSi.length; i++) solversSi[i].preSolvePosition(timeStep);
    for (let t = 0; t < numPositionIterations; t++) {
      for (let i = 0; i < solversSi.length; i++) solversSi[i].solvePositionSplitImpulse();
    }
    for (let i = 0; i < bodies.length; i++) bodies[i].integratePseudoVelocity();
    for (let i = 0; i < solversNgs.length; i++) solversNgs[i].preSolvePosition(timeStep);
    for (let t = 0; t < numPositionIterations; t++) {
      for (let i = 0; i < solversNgs.length; i++) solversNgs[i].solvePositionNgs(timeStep);
    }
    for (let i = 0; i < solvers.length; i++) solvers[i].postSolve();
    for (let i = 0; i < bodies.length; i++) bodies[i].syncShapes();
  }
  /**
   * 单步软接触求解：带偏置求解、积分、松弛。
   * 撞击增加松弛迭代次数，关节使用独立迭代次数及位置修正。
   */
  stepSoft(timeStep, iterations, relaxIterations, impactIterations, impactVelocity, dynamic, fixed, maxPushVelocity, jointIterations, numPositionIterations, parity) {
    const dt = timeStep.dt, invDt = timeStep.invDt, bodies = this.rigidBodies, joints = this.solvers, contacts = this.softContacts;
    const jointsSi = this.solversSi, jointsNgs = this.solversNgs;
    let sleepIsland = true;
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      rb.ptransform.copyFrom(rb.transform);
      rb.lciX = 0;
      rb.lciY = 0;
      rb.lciZ = 0;
      rb.aciX = 0;
      rb.aciY = 0;
      rb.aciZ = 0;
      rb.sleeping = false;
      if (rb.isSleepy() && !rb.restless) rb.sleepTime += dt;
      else rb.sleepTime = 0;
      if (rb.sleepTime < rb.sleepingTimeThreshold) sleepIsland = false;
      if (rb.type === RigidBodyType.DYNAMIC) this.applyForces(rb, dt);
    }
    if (sleepIsland) {
      for (let i = 0; i < bodies.length; i++) bodies[i].sleep();
      return;
    }
    let approach = 0;
    for (let i = 0; i < contacts.length; i++) {
      const a = contacts[i].prepare();
      if (a > approach) approach = a;
    }
    const impact = approach > impactVelocity;
    const passes = iterations;
    const relaxPasses = impact ? relaxIterations + impactIterations : relaxIterations;
    for (let i = 0; i < joints.length; i++) joints[i].preSolveVelocity(timeStep);
    for (let i = 0; i < joints.length; i++) joints[i].warmStart(timeStep);
    for (let i = 0; i < contacts.length; i++) contacts[i].warmStart();
    const total = joints.length > 0 && jointIterations > passes ? jointIterations : passes;
    for (let k = 0; k < total; k++) {
      for (let i = 0; i < joints.length; i++) joints[i].solveVelocity();
      const t = k - (total - passes);
      if (t < 0) continue;
      if ((t + parity & 1) === 0) for (let i = 0; i < contacts.length; i++) contacts[i].solve(true, false, invDt, dynamic, fixed, maxPushVelocity);
      else for (let i = contacts.length - 1; i >= 0; i--) contacts[i].solve(true, true, invDt, dynamic, fixed, maxPushVelocity);
    }
    for (let i = 0; i < joints.length; i++) joints[i].postSolveVelocity(timeStep);
    for (let i = 0; i < bodies.length; i++) bodies[i].integrate(dt);
    for (let t = 0; t < relaxPasses; t++) {
      if ((t + parity & 1) === 0) for (let i = contacts.length - 1; i >= 0; i--) contacts[i].solve(false, true, invDt, dynamic, fixed, maxPushVelocity);
      else for (let i = 0; i < contacts.length; i++) contacts[i].solve(false, false, invDt, dynamic, fixed, maxPushVelocity);
    }
    for (let i = 0; i < contacts.length; i++) contacts[i].applyRestitution();
    for (let i = 0; i < jointsSi.length; i++) jointsSi[i].preSolvePosition(timeStep);
    for (let t = 0; t < numPositionIterations; t++) {
      for (let i = 0; i < jointsSi.length; i++) jointsSi[i].solvePositionSplitImpulse();
    }
    for (let i = 0; i < bodies.length; i++) bodies[i].integratePseudoVelocity();
    for (let i = 0; i < jointsNgs.length; i++) jointsNgs[i].preSolvePosition(timeStep);
    for (let t = 0; t < numPositionIterations; t++) {
      for (let i = 0; i < jointsNgs.length; i++) jointsNgs[i].solvePositionNgs(timeStep);
    }
    for (let i = 0; i < joints.length; i++) joints[i].postSolve();
    for (let i = 0; i < contacts.length; i++) contacts[i].store();
    for (let i = 0; i < bodies.length; i++) bodies[i].syncShapes();
  }
  /**
   * 准备多个子步共用的接触数据；整个岛休眠时返回 false。
   * 随后由 runSubsteps 求解和积分，finishSubsteps 完成反弹、断开检查及形状同步。
   * tick 为整步时长，sub 为子步时长。
   */
  beginSubsteps(tick, sub, p) {
    const bodies = this.rigidBodies, joints = this.solvers, contacts = this.softContacts;
    const ordered = this.orderedContacts;
    ordered.length = 0;
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i].constraint;
      if (c.b1.type === RigidBodyType.STATIC || c.b2.type === RigidBodyType.STATIC) ordered.push(contacts[i]);
    }
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i].constraint;
      if (c.b1.type !== RigidBodyType.STATIC && c.b2.type !== RigidBodyType.STATIC) ordered.push(contacts[i]);
    }
    for (let i = 0; i < contacts.length; i++) contacts[i] = ordered[i];
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      rb.ptransform.copyFrom(rb.transform);
      rb.lciX = 0;
      rb.lciY = 0;
      rb.lciZ = 0;
      rb.aciX = 0;
      rb.aciY = 0;
      rb.aciZ = 0;
      rb.sleeping = false;
    }
    let impact = false;
    const stamp = p.stamp;
    for (let i = 0; i < contacts.length; i++) {
      if (contacts[i].prepare(sub.dtRatio, true, p.skin, sub.dt, p.restitutionThreshold) > p.impactVelocity) {
        impact = true;
        const c = contacts[i].constraint, b1 = c.b1, b2 = c.b2;
        if (b1.type === RigidBodyType.DYNAMIC && !b1.sleeping) b1.impactStamp = stamp;
        if (b2.type === RigidBodyType.DYNAMIC && !b2.sleeping) b2.impactStamp = stamp;
      }
    }
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i].constraint;
      contacts[i].impacted = impact && (c.b1.impactStamp === stamp || c.b2.impactStamp === stamp);
    }
    this.conditionedContacts = (contacts.length > 8 || this.extendedBody) && this.maxMass >= this.minMass * 8;
    const minimumPasses = this.conditionedContacts ? Math.min(40, Math.max(10, Math.ceil(this.maxMass / this.minMass / 4))) : 0;
    const passes = Math.max(minimumPasses, impact ? Math.max(p.impactIterations, p.iterations) : p.iterations);
    this.biasPasses = passes;
    this.relaxPasses = Math.max(minimumPasses, impact ? Math.max(p.impactIterations, p.relaxIterations) : p.relaxIterations);
    this.solveTotal = joints.length > 0 && p.jointIterations > passes ? p.jointIterations : passes;
    let quiet = p.adaptiveIterations && !impact && !this.conditionedContacts && joints.length === 0 && (p.substeps >= 4 || p.substeps === 1);
    let isotropic = true;
    for (let i = 0; quiet && i < bodies.length; i++) {
      const b = bodies[i];
      if (b.type === RigidBodyType.STATIC) continue;
      if (b.type !== RigidBodyType.DYNAMIC || b.drag !== null) {
        quiet = false;
        break;
      }
      const limit = Math.max(b.sleepingVelocityThreshold, b.sleepingAngularVelocityThreshold);
      if (b.vx * b.vx + b.vy * b.vy + b.vz * b.vz + b.wx * b.wx + b.wy * b.wy + b.wz * b.wz >= limit * limit || b.fx !== 0 || b.fy !== 0 || b.fz !== 0 || b.tx !== 0 || b.ty !== 0 || b.tz !== 0 || p.sea && (p.seaStartY - b.transform.py) * p.seaDepthGradient >= 0) quiet = false;
      if (b.li01 !== 0 || b.li02 !== 0 || b.li12 !== 0 || b.li00 !== b.li11 || b.li11 !== b.li22) isotropic = false;
    }
    this.quietContacts = quiet;
    this.isotropicContacts = isotropic;
    return true;
  }
  /** 执行 substeps 个子步，仅在 beginSubsteps 返回 true 时调用。 */
  runSubsteps(sub, p) {
    const h = sub.dt, invH = sub.invDt, n = p.substeps;
    const bodies = this.rigidBodies, joints = this.solvers, contacts = this.softContacts;
    const jointsSi = this.solversSi, jointsNgs = this.solversNgs;
    const dynamic = p.dynamic, fixed = p.fixed, push = p.maxPushVelocity;
    const reuseAcrossSubsteps = this.relaxPasses > 0 && jointsSi.length === 0 && jointsNgs.length === 0;
    const drags = this.drags;
    const aggregateWarmStart = joints.length === 0 && drags.length === 0;
    let parity = p.parity;
    let quiet = this.quietContacts;
    for (let s = 0; s < n; s++) {
      if (s > 0) sub.dtRatio = 1;
      for (let i = 0; i < bodies.length; i++) if (bodies[i].type === RigidBodyType.DYNAMIC) this.applySoftForces(bodies[i], h, p);
      for (let i = 0; i < joints.length; i++) joints[i].preSolveVelocity(sub);
      const ratio = sub.dtRatio;
      for (let i = 0; i < joints.length; i++) {
        const joint = joints[i];
        sub.dtRatio = s > 0 || joint.softSubstepDt === 0 ? ratio : h / joint.softSubstepDt;
        joint.warmStart(sub);
        joint.softSubstepDt = h;
      }
      for (let i = 0; i < drags.length; i++) drags[i].warmStart();
      if (aggregateWarmStart) for (let i = 0; i < bodies.length; i++) {
        const b = bodies[i];
        if (b.type !== RigidBodyType.DYNAMIC) continue;
        const vx = b.vx, vy = b.vy, vz = b.vz, wx = b.wx, wy = b.wy, wz = b.wz;
        if (s > 0) {
          b.vx += b.contactDeltaVX;
          b.vy += b.contactDeltaVY;
          b.vz += b.contactDeltaVZ;
          b.wx += b.contactDeltaWX;
          b.wy += b.contactDeltaWY;
          b.wz += b.contactDeltaWZ;
        }
        b.contactDeltaVX = vx;
        b.contactDeltaVY = vy;
        b.contactDeltaVZ = vz;
        b.contactDeltaWX = wx;
        b.contactDeltaWY = wy;
        b.contactDeltaWZ = wz;
      }
      if (!aggregateWarmStart || s === 0) for (let i = 0; i < contacts.length; i++) contacts[i].warmStart();
      for (let i = 0; i < drags.length; i++) drags[i].solve(h, (s + 1) / n);
      for (let i = 0; quiet && i < bodies.length; i++) {
        const b = bodies[i], limit = Math.max(b.sleepingVelocityThreshold, b.sleepingAngularVelocityThreshold);
        if (b.type === RigidBodyType.STATIC) continue;
        if (b.vx * b.vx + b.vy * b.vy + b.vz * b.vz + b.wx * b.wx + b.wy * b.wy + b.wz * b.wz >= 4 * limit * limit) quiet = false;
      }
      const passes = quiet ? Math.min(this.biasPasses, this.isotropicContacts && (s > 0 || n === 1) ? 1 : 2) : this.biasPasses;
      const relaxPasses = quiet ? Math.min(this.relaxPasses, n > 1 || this.isotropicContacts ? 1 : 2) : this.relaxPasses;
      const total = quiet ? passes : this.solveTotal;
      for (let k = 0; k < total; k++) {
        for (let i = 0; i < joints.length; i++) joints[i].solveVelocity();
        const t = k - (total - passes);
        if (t < 0) continue;
        const reuse = t > 0 || s > 0 && reuseAcrossSubsteps;
        let error = 0;
        if ((t + parity & 1) === 0) {
          for (let i = 0; i < contacts.length; i++) if (this.conditionedContacts || t < p.iterations || contacts[i].impacted) error += contacts[i].solve(true, false, invH, dynamic, fixed, push, reuse);
        } else {
          for (let i = contacts.length - 1; i >= 0; i--) if (this.conditionedContacts || t < p.iterations || contacts[i].impacted) error += contacts[i].solve(true, true, invH, dynamic, fixed, push, reuse);
        }
        if (joints.length === 0 && error < p.tolerance) break;
      }
      for (let i = 0; i < joints.length; i++) joints[i].postSolveVelocity(sub);
      for (let i = 0; i < bodies.length; i++) bodies[i].integrate(h);
      for (let t = 0; t < relaxPasses; t++) {
        const reuse = t > 0;
        let error = 0;
        if ((t + parity & 1) === 0) {
          for (let i = contacts.length - 1; i >= 0; i--) if (this.conditionedContacts || t < p.relaxIterations || contacts[i].impacted) error += contacts[i].solve(false, true, invH, dynamic, fixed, push, reuse);
        } else {
          for (let i = 0; i < contacts.length; i++) if (this.conditionedContacts || t < p.relaxIterations || contacts[i].impacted) error += contacts[i].solve(false, false, invH, dynamic, fixed, push, reuse);
        }
        if (joints.length === 0 && error < p.tolerance) break;
      }
      if (aggregateWarmStart) for (let i = 0; i < bodies.length; i++) {
        const b = bodies[i];
        if (b.type !== RigidBodyType.DYNAMIC) continue;
        b.contactDeltaVX = b.vx - b.contactDeltaVX;
        b.contactDeltaVY = b.vy - b.contactDeltaVY;
        b.contactDeltaVZ = b.vz - b.contactDeltaVZ;
        b.contactDeltaWX = b.wx - b.contactDeltaWX;
        b.contactDeltaWY = b.wy - b.contactDeltaWY;
        b.contactDeltaWZ = b.wz - b.contactDeltaWZ;
      }
      for (let i = 0; i < jointsSi.length; i++) jointsSi[i].preSolvePosition(sub);
      for (let t = 0; t < p.positionIterations; t++) {
        for (let i = 0; i < jointsSi.length; i++) jointsSi[i].solvePositionSplitImpulse();
      }
      for (let i = 0; i < bodies.length; i++) bodies[i].integratePseudoVelocity();
      for (let i = 0; i < jointsNgs.length; i++) jointsNgs[i].preSolvePosition(sub);
      for (let t = 0; t < p.positionIterations; t++) {
        for (let i = 0; i < jointsNgs.length; i++) jointsNgs[i].solvePositionNgs(sub);
      }
      parity ^= 1;
    }
  }
  /** 完成本步的反弹、关节断开检查、冲量存储和形状同步。 */
  finishSubsteps(tick, p) {
    const dt = tick.dt;
    const bodies = this.rigidBodies, joints = this.solvers, contacts = this.softContacts;
    let bounce = false;
    for (let i = 0; i < contacts.length; i++) {
      if (contacts[i].wantsRestitution()) {
        bounce = true;
        break;
      }
    }
    if (bounce) {
      for (let t = 0; t < p.restitutionIterations; t++) for (let i = 0; i < contacts.length; i++) contacts[i].applyRestitution();
    }
    for (let i = 0; i < joints.length; i++) joints[i].postSolve();
    for (let i = 0; i < contacts.length; i++) contacts[i].store(p.stamp);
    let sleepIsland = true;
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      if (rb.type === RigidBodyType.DYNAMIC) this.limitState(rb, p);
      rb.drag?.finish();
      if (rb.isSleepy() && !rb.restless && rb.drag === null) rb.sleepTime += dt;
      else rb.sleepTime = 0;
      if (rb.sleepTime < rb.sleepingTimeThreshold) sleepIsland = false;
    }
    for (let i = 0; i < bodies.length; i++) {
      const rb = bodies[i];
      if (sleepIsland) {
        rb.sleep();
        rb.vx = 0;
        rb.vy = 0;
        rb.vz = 0;
        rb.wx = 0;
        rb.wy = 0;
        rb.wz = 0;
      }
      if (rb.type !== RigidBodyType.STATIC) rb.syncShapesSpeculative(dt, p.speculativeDistance, p.maxSpeculativeDistance);
    }
  }
  /** 记录孤立刚体的步进起始位姿，供子步积分与异常回滚使用。 */
  beginSingleRigidBodySubsteps(tick, rb) {
    rb.ptransform.copyFrom(rb.transform);
    rb.lciX = 0;
    rb.lciY = 0;
    rb.lciZ = 0;
    rb.aciX = 0;
    rb.aciY = 0;
    rb.aciZ = 0;
    return !rb.sleeping;
  }
  /** 执行孤立刚体的子步积分。 */
  runSingleRigidBodySubsteps(sub, p, rb) {
    const h = sub.dt;
    const dynamic = rb.type === RigidBodyType.DYNAMIC;
    for (let s = 0; s < p.substeps; s++) {
      if (dynamic) this.applySoftForces(rb, h, p);
      rb.drag?.warmStart();
      rb.drag?.solve(h, (s + 1) / p.substeps);
      rb.integrate(h);
    }
  }
  /** 完成孤立刚体的限速、休眠计时和形状同步。 */
  finishSingleRigidBodySubsteps(tick, p, rb) {
    rb.drag?.finish();
    if (rb.type === RigidBodyType.DYNAMIC) this.limitState(rb, p);
    if (rb.isSleepy() && !rb.restless && rb.drag === null) {
      rb.sleepTime += tick.dt;
      if (rb.sleepTime >= rb.sleepingTimeThreshold) {
        rb.sleep();
        rb.vx = 0;
        rb.vy = 0;
        rb.vz = 0;
        rb.wx = 0;
        rb.wy = 0;
        rb.wz = 0;
      }
    } else {
      rb.sleepTime = 0;
    }
    rb.syncShapesSpeculative(tick.dt, p.speculativeDistance, p.maxSpeculativeDistance);
  }
  /**
   * 累加重力、外力和力矩，应用隐式陀螺项及 1 / (1 + h·d) 阻尼。
   * 存在锁定转轴时跳过陀螺项；p.sea 启用时先施加虚空海作用力。
   */
  applySoftForces(rb, h, p) {
    const gs = rb.gravityScale, invMass = rb.invMass;
    let vX = rb.vx, vY = rb.vy, vZ = rb.vz, wX = rb.wx, wY = rb.wy, wZ = rb.wz;
    if (p.sea) {
      const depth = (p.seaStartY - rb.transform.py) * p.seaDepthGradient;
      if (depth >= 0) {
        const lin2 = 1 - h * 3.5 * p.seaDrag, ang2 = 1 - h * 3 * p.seaDrag;
        const lift = -Math.sign(p.seaDepthGradient) * depth * h;
        vX = vX * lin2 + this.gravityX * lift;
        vY = vY * lin2 + this.gravityY * lift;
        vZ = vZ * lin2 + this.gravityZ * lift;
        wX *= ang2;
        wY *= ang2;
        wZ *= ang2;
      }
    }
    vX += h * (this.gravityX * gs + rb.fx * invMass);
    vY += h * (this.gravityY * gs + rb.fy * invMass);
    vZ += h * (this.gravityZ * gs + rb.fz * invMass);
    if (rb.tx !== 0 || rb.ty !== 0 || rb.tz !== 0) {
      wX += h * (rb.ii00 * rb.tx + rb.ii01 * rb.ty + rb.ii02 * rb.tz);
      wY += h * (rb.ii10 * rb.tx + rb.ii11 * rb.ty + rb.ii12 * rb.tz);
      wZ += h * (rb.ii20 * rb.tx + rb.ii21 * rb.ty + rb.ii22 * rb.tz);
    }
    if (wX * wX + wY * wY + wZ * wZ > 1e-6 && rb.rotFactorX === 1 && rb.rotFactorY === 1 && rb.rotFactorZ === 1) {
      const tf = rb.transform;
      const r00 = tf.r00, r01 = tf.r01, r02 = tf.r02, r10 = tf.r10, r11 = tf.r11, r12 = tf.r12, r20 = tf.r20, r21 = tf.r21, r22 = tf.r22;
      const oX = r00 * wX + r10 * wY + r20 * wZ, oY = r01 * wX + r11 * wY + r21 * wZ, oZ = r02 * wX + r12 * wY + r22 * wZ;
      const i0 = rb.li00, i1 = rb.li01, i2 = rb.li02, i3 = rb.li11, i4 = rb.li12, i5 = rb.li22;
      const lX = i0 * oX + i1 * oY + i2 * oZ, lY = i1 * oX + i3 * oY + i4 * oZ, lZ = i2 * oX + i4 * oY + i5 * oZ;
      const fX = h * (oY * lZ - oZ * lY), fY = h * (oZ * lX - oX * lZ), fZ = h * (oX * lY - oY * lX);
      const j00 = i0 + h * (-oZ * i1 + oY * i2), j01 = i1 + h * (-oZ * i3 + oY * i4 + lZ), j02 = i2 + h * (-oZ * i4 + oY * i5 - lY);
      const j10 = i1 + h * (oZ * i0 - oX * i2 - lZ), j11 = i3 + h * (oZ * i1 - oX * i4), j12 = i4 + h * (oZ * i2 - oX * i5 + lX);
      const j20 = i2 + h * (-oY * i0 + oX * i1 + lY), j21 = i4 + h * (-oY * i1 + oX * i3 - lX), j22 = i5 + h * (-oY * i2 + oX * i4);
      const c0 = j11 * j22 - j12 * j21, c1 = j12 * j20 - j10 * j22, c2 = j10 * j21 - j11 * j20;
      const det2 = j00 * c0 + j01 * c1 + j02 * c2;
      if (Math.abs(det2) > 1e-12) {
        const inv2 = 1 / det2;
        const dX = (fX * c0 + fY * (j02 * j21 - j01 * j22) + fZ * (j01 * j12 - j02 * j11)) * inv2;
        const dY = (fX * c1 + fY * (j00 * j22 - j02 * j20) + fZ * (j02 * j10 - j00 * j12)) * inv2;
        const dZ = (fX * c2 + fY * (j01 * j20 - j00 * j21) + fZ * (j00 * j11 - j01 * j10)) * inv2;
        const nX = oX - dX, nY = oY - dY, nZ = oZ - dZ;
        wX = r00 * nX + r01 * nY + r02 * nZ;
        wY = r10 * nX + r11 * nY + r12 * nZ;
        wZ = r20 * nX + r21 * nY + r22 * nZ;
      }
    }
    const lin = 1 / (1 + h * rb.linearDamping), ang = 1 / (1 + h * rb.angularDamping);
    rb.vx = vX * lin;
    rb.vy = vY * lin;
    rb.vz = vZ * lin;
    rb.wx = wX * ang;
    rb.wy = wY * ang;
    rb.wz = wZ * ang;
  }
  /**
   * 结束步进并限制速度。
   * 非有限状态回滚至起始位姿，清零速度并丢弃接触点。
   */
  limitState(rb, p) {
    const tf = rb.transform;
    const sum = rb.vx + rb.vy + rb.vz + rb.wx + rb.wy + rb.wz + tf.px + tf.py + tf.pz + tf.r00 + tf.r01 + tf.r02 + tf.r10 + tf.r11 + tf.r12 + tf.r20 + tf.r21 + tf.r22;
    if (sum !== sum || sum === Infinity || sum === -Infinity) {
      tf.copyFrom(rb.ptransform);
      rb.vx = 0;
      rb.vy = 0;
      rb.vz = 0;
      rb.wx = 0;
      rb.wy = 0;
      rb.wz = 0;
      rb.updateInvInertia();
      for (let l = rb.contactLinkList; l !== null; l = l.next) l.contact.manifold.clear();
      return;
    }
    const v2 = rb.vx * rb.vx + rb.vy * rb.vy + rb.vz * rb.vz, maxV = p.maxLinearVelocity;
    if (v2 > maxV * maxV) {
      const s = maxV / Math.sqrt(v2);
      rb.vx *= s;
      rb.vy *= s;
      rb.vz *= s;
    }
    const w2 = rb.wx * rb.wx + rb.wy * rb.wy + rb.wz * rb.wz, maxW = p.maxAngularVelocity;
    if (w2 > maxW * maxW) {
      const s = maxW / Math.sqrt(w2);
      rb.wx *= s;
      rb.wy *= s;
      rb.wz *= s;
    }
  }
  /** 重力、外力、阻尼更新速度 */
  applyForces(rb, dt) {
    const linScale = fastInvExp(dt * rb.linearDamping);
    const angScale = fastInvExp(dt * rb.angularDamping);
    const gs = rb.gravityScale, invMass = rb.invMass;
    const laX = this.gravityX * gs + rb.fx * invMass;
    const laY = this.gravityY * gs + rb.fy * invMass;
    const laZ = this.gravityZ * gs + rb.fz * invMass;
    const aaX = rb.ii00 * rb.tx + rb.ii01 * rb.ty + rb.ii02 * rb.tz;
    const aaY = rb.ii10 * rb.tx + rb.ii11 * rb.ty + rb.ii12 * rb.tz;
    const aaZ = rb.ii20 * rb.tx + rb.ii21 * rb.ty + rb.ii22 * rb.tz;
    rb.vx = (rb.vx + laX * dt) * linScale;
    rb.vy = (rb.vy + laY * dt) * linScale;
    rb.vz = (rb.vz + laZ * dt) * linScale;
    rb.wx = (rb.wx + aaX * dt) * angScale;
    rb.wy = (rb.wy + aaY * dt) * angScale;
    rb.wz = (rb.wz + aaZ * dt) * angScale;
  }
};

// src/core/dynamics/time-step.ts
var TimeStep = class {
  constructor() {
    this.dt = 0;
    this.invDt = 0;
    /** 这一步和上一步 dt 的比值，热启动时按它缩放上一步的冲量 */
    this.dtRatio = 1;
  }
};

// src/core/dynamics/world.ts
var BroadPhaseType = { BRUTE_FORCE: 1, BVH: 2 };
var World = class {
  constructor(broadPhaseType = BroadPhaseType.BVH) {
    this.rigidBodyList = null;
    this.rigidBodyListLast = null;
    this.jointList = null;
    this.jointListLast = null;
    this.numRigidBodies = 0;
    this.numJoints = 0;
    this.numShapes = 0;
    this.numIslands = 0;
    this.numVelocityIterations = 10;
    this.numPositionIterations = 5;
    /**
     * 是否使用软接触求解，默认为 false。
     * 开启后接触使用 softIterations，关节保留独立的速度和位置迭代设置。
     */
    this.softContacts = false;
    /** 软接触带偏置阶段的迭代次数。 */
    this.softIterations = 3;
    /** 软接触：积分之后松弛解几遍 */
    this.softRelaxIterations = 3;
    /** 冲量变化总量低于此值时结束本阶段迭代；0 表示固定遍数。 */
    this.softTolerance = 0;
    /** 低速且无关节的接触岛减少常规迭代，受力或速度增大时恢复。 */
    this.softAdaptiveIterations = false;
    /**
     * 撞击迭代次数。子步模式下，超出常规遍数的部分仅处理被撞刚体的接触。
     * 单步模式将此值加到松弛遍数上。
     */
    this.softImpactIterations = 4;
    /** 软接触：靠近速度超过它（m/s）算撞击 */
    this.softImpactVelocity = 1.5;
    /** 软接触，多个子步时：子步结束后反弹解几遍；靠近速度达到多少（m/s）才反弹 */
    this.softRestitutionIterations = 6;
    this.softRestitutionThreshold = 1;
    /** 软接触，多个子步时：每个 tick 结束时的速度上限（m/s）、角速度上限（rad/s） */
    this.softMaxLinearVelocity = 60;
    this.softMaxAngularVelocity = 30;
    /** 动态接触频率上限，单位为 Hz，实际取 min(上限, softHertzStepRatio / dt)。 */
    this.softContactHertz = 30;
    /** 软接触：接触频率最多是子步频率的几倍 */
    this.softHertzStepRatio = 0.25;
    /** 软接触：一边不会动（地面）时的接触频率是动态之间的几倍 */
    this.softFixedHertzScale = 2;
    /** 软接触：阻尼比 */
    this.softDampingRatio = 10;
    /** 软接触：推开穿透的最大速度（m/s） */
    this.softMaxPushVelocity = 3;
    /**
     * 每次 step 的软接触子步数，0 表示单步。
     * 正值使碰撞检测和接触准备每步仅执行一次，子步执行求解和积分；仅在 softContacts 开启时有效。
     */
    this.softSubsteps = 0;
    /** 大质量比接触岛的子步数下限，0 表示不单独调整。 */
    this.softSupportSubsteps = 0;
    /** 最近一次步进中各岛使用的最大子步数。 */
    this.lastMaxSubsteps = 0;
    /**
     * 预测接触的基础距离，单位为米，仅在 softSubsteps > 0 时生效。
     * 静态接触按形状运动范围扩展，动态接触按受限的相对运动速度扩展，上限为 maxSpeculativeDistance。
     */
    this.speculativeDistance = 0.04;
    /** 预测接触距离的上限，单位为米。 */
    this.maxSpeculativeDistance = Infinity;
    /** 动态接触中用于计算预测距离的相对运动速度上限。 */
    this.pairSpeculativeSpeed = 1.5;
    /**
     * 虚空海参数，null 表示关闭，仅在 softSubsteps > 0 时有效。
     * 低于 startY 时施加深度相关的反重力冲量及速度阻尼。
     */
    this.voidSea = null;
    /**
     * 静态接触面的法向偏移，单位为米，仅在 softSubsteps > 0 时生效。
     * 用于控制地形接缝处的接触余量，不作用于两个动态刚体之间的接触。
     */
    this.softContactSkin = 0.012;
    this.softDynamic = new Softness();
    this.softFixed = new Softness();
    /** 软接触：每步翻转一次，让求解顺序在相邻两步之间交替 */
    this.softParity = 0;
    /** 可选的毫秒时钟，供子步耗时统计使用，不影响模拟结果。 */
    this.clock = null;
    /**
     * 上一步所有活动岛和孤立刚体的子步循环耗时，单位为毫秒。
     * 不含碰撞检测及步进前后的准备、同步；未提供 clock 或未启用子步时为 0。
     */
    this.lastSubstepMs = 0;
    this.subTimeStep = new TimeStep();
    this.substepParams = new SoftSubstepParams(this.softDynamic, this.softFixed);
    /** 步进编号，供接触判断本步是否参与求解。 */
    this.stepStamp = 0;
    /** 重力加速度（m/s²） */
    this.gravityX = 0;
    this.gravityY = DEFAULT_GRAVITY_Y;
    this.gravityZ = 0;
    this.timeStep = new TimeStep();
    this.island = new Island();
    this.rigidBodyStack = [];
    this.solversInIslands = [];
    this.shapeIdCount = 0;
    this.rayCastWrapper = new RayCastWrapper();
    this.convexCastWrapper = new ConvexCastWrapper();
    this.aabbTestWrapper = new AabbTestWrapper();
    /** wakeTouched 复用的包围盒和回调。 */
    this.wakeAabb = new Aabb();
    this.wakeCallback = {
      process(shape) {
        const b = shape.rigidBody;
        if (b.sleeping && b.type !== RigidBodyType.STATIC && !b.wakeLocked) b.wakeUp();
      }
    };
    this.broadPhase = broadPhaseType === BroadPhaseType.BRUTE_FORCE ? new BruteForceBroadPhase() : new BvhBroadPhase();
    this.contactManager = new ContactManager(this.broadPhase);
  }
  /** 前进 dt 秒：宽相 → 窄相 → 按岛求解 */
  step(dt) {
    const ts = this.timeStep;
    if (ts.dt > 0) ts.dtRatio = dt / ts.dt;
    ts.dt = dt;
    ts.invDt = 1 / dt;
    const substepped = this.softContacts && this.softSubsteps > 0;
    this.contactManager.speculative = substepped;
    if (substepped) this.wakeContacts(dt);
    this.contactManager.updateContacts();
    this.contactManager.updateManifolds();
    this.clearRestless();
    if (substepped) {
      this.stepStamp++;
      this.solveIslandsSubstepped();
      this.contactManager.updateTouching(this.stepStamp);
    } else {
      this.solveIslands();
    }
  }
  addRigidBody(rb) {
    if (rb.world !== null) throw new Error("\u521A\u4F53\u4E0D\u80FD\u540C\u65F6\u5C5E\u4E8E\u591A\u4E2A\u4E16\u754C");
    if (this.rigidBodyList === null) {
      this.rigidBodyList = rb;
      this.rigidBodyListLast = rb;
    } else {
      this.rigidBodyListLast.next = rb;
      rb.prev = this.rigidBodyListLast;
      this.rigidBodyListLast = rb;
    }
    rb.world = this;
    for (let s = rb.shapeList; s !== null; s = s.next) this.addShape(s);
    this.numRigidBodies++;
    if (rb.type !== RigidBodyType.STATIC && this.speculativeTick() > 0) rb.syncShapes();
  }
  /**
   * 移除刚体及其形状和接触，关联关节不自动移除。
   * wakeContacts=false 时保留接触另一侧刚体的休眠状态。
   */
  removeRigidBody(rb, wakeContacts = true) {
    if (rb.world !== this) throw new Error("\u521A\u4F53\u4E0D\u5C5E\u4E8E\u8FD9\u4E2A\u4E16\u754C");
    const prev = rb.prev, next = rb.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (rb === this.rigidBodyList) this.rigidBodyList = next;
    if (rb === this.rigidBodyListLast) this.rigidBodyListLast = prev;
    rb.next = null;
    rb.prev = null;
    rb.world = null;
    for (let s = rb.shapeList; s !== null; s = s.next) this.removeShape(s, wakeContacts);
    this.numRigidBodies--;
  }
  /** 加入关节，并唤醒它连着的两个刚体。两个刚体要先加入世界 */
  addJoint(joint) {
    if (joint.world !== null) throw new Error("\u5173\u8282\u4E0D\u80FD\u540C\u65F6\u5C5E\u4E8E\u591A\u4E2A\u4E16\u754C");
    if (this.jointList === null) {
      this.jointList = joint;
      this.jointListLast = joint;
    } else {
      this.jointListLast.next = joint;
      joint.prev = this.jointListLast;
      this.jointListLast = joint;
    }
    joint.world = this;
    joint.attachLinks();
    joint.syncAnchors();
    this.numJoints++;
  }
  /** 移除关节，并唤醒它连着的两个刚体。关节被拉断时也会自动移除 */
  removeJoint(joint) {
    if (joint.world !== this) throw new Error("\u5173\u8282\u4E0D\u5C5E\u4E8E\u8FD9\u4E2A\u4E16\u754C");
    const prev = joint.prev, next = joint.next;
    if (prev !== null) prev.next = next;
    if (next !== null) next.prev = prev;
    if (joint === this.jointList) this.jointList = next;
    if (joint === this.jointListLast) this.jointListLast = prev;
    joint.next = null;
    joint.prev = null;
    joint.world = null;
    joint.detachLinks();
    this.numJoints--;
  }
  /**
   * 逐个回调射线命中的形状，顺序与距离无关。
   * 回调期间不支持增删刚体、形状或嵌套查询。最近命中查询使用 RayCastClosest。
   */
  rayCast(beginX, beginY, beginZ, endX, endY, endZ, callback) {
    const w = this.rayCastWrapper;
    w.beginX = beginX;
    w.beginY = beginY;
    w.beginZ = beginZ;
    w.endX = endX;
    w.endY = endY;
    w.endZ = endZ;
    w.callback = callback;
    this.broadPhase.rayCast(beginX, beginY, beginZ, endX, endY, endZ, w);
  }
  /**
   * 沿平移量扫掠凸体，并逐个回调命中结果；初始重叠不报告。
   * hit 包含比例、碰撞点和法线，回调顺序与距离无关。回调期间不支持修改世界或嵌套查询。
   */
  convexCast(convex, begin, tx, ty, tz, callback) {
    const w = this.convexCastWrapper;
    w.convex = convex;
    w.begin.copyFrom(begin);
    w.translation.init(tx, ty, tz);
    w.callback = callback;
    this.broadPhase.convexCast(convex, begin, w.translation, w);
  }
  /**
   * 逐个回调包围盒与 aabb 重叠的形状。
   * 查询包含形状的扫掠范围；回调期间不支持修改世界或嵌套查询。
   */
  aabbTest(aabb, callback) {
    const w = this.aabbTestWrapper;
    w.aabb.copyFrom(aabb);
    w.callback = callback;
    this.broadPhase.aabbTest(aabb, w);
  }
  /** 形状加入世界：在宽相里创建代理 */
  addShape(shape) {
    shape.proxy = this.broadPhase.createProxy(shape, shape.aabb);
    shape.id = this.shapeIdCount++;
    this.numShapes++;
  }
  /** 移除形状的代理及相关接触；wakeContacts 控制是否唤醒接触另一侧的刚体。 */
  removeShape(shape, wakeContacts = true) {
    this.broadPhase.destroyProxy(shape.proxy);
    shape.proxy = null;
    shape.id = -1;
    let cl = shape.rigidBody.contactLinkList;
    while (cl !== null) {
      const next = cl.next;
      const c = cl.contact;
      if (c.s1 === shape || c.s2 === shape) {
        if (wakeContacts) cl.other.wakeUp();
        this.contactManager.destroyContact(c);
      }
      cl = next;
    }
    this.numShapes--;
  }
  solveIslands() {
    const island = this.island;
    island.setGravity(this.gravityX, this.gravityY, this.gravityZ);
    this.numIslands = 0;
    const soft = this.softContacts;
    if (soft) {
      this.softParity ^= 1;
      const h = this.timeStep.dt;
      const cap = this.softHertzStepRatio / h;
      this.softDynamic.set(Math.min(this.softContactHertz, cap), this.softDampingRatio, h);
      this.softFixed.set(Math.min(this.softContactHertz * this.softFixedHertzScale, cap * this.softFixedHertzScale), this.softDampingRatio, h);
    }
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.addedToIsland || b.sleeping || b.type === RigidBodyType.STATIC) continue;
      if (b.isAlone()) {
        island.stepSingleRigidBody(this.timeStep, b);
      } else {
        this.buildIsland(b);
        if (soft) {
          island.stepSoft(
            this.timeStep,
            this.softIterations,
            this.softRelaxIterations,
            this.softImpactIterations,
            this.softImpactVelocity,
            this.softDynamic,
            this.softFixed,
            this.softMaxPushVelocity,
            this.numVelocityIterations,
            this.numPositionIterations,
            this.softParity
          );
        } else {
          island.step(this.timeStep, this.numVelocityIterations, this.numPositionIterations);
        }
        island.clear();
      }
      this.numIslands++;
    }
    this.finishIslands();
  }
  /** 清掉岛的标记，以及这一步的力和力矩 */
  finishIslands() {
    for (let b = this.rigidBodyList; b !== null; b = b.next) b.addedToIsland = false;
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      b.fx = 0;
      b.fy = 0;
      b.fz = 0;
      b.tx = 0;
      b.ty = 0;
      b.tz = 0;
    }
    const solvers = this.solversInIslands;
    for (let i = 0; i < solvers.length; i++) solvers[i].addedToIsland = false;
    solvers.length = 0;
  }
  /**
   * 返回子步模式的 tick 时长，供形状计算预测距离。
   * 尚未推进时为 1/20 秒；未启用 softContacts 或 softSubsteps 时为 0。
   */
  speculativeTick() {
    if (!this.softContacts || this.softSubsteps <= 0) return 0;
    return this.timeStep.dt > 0 ? this.timeStep.dt : 0.05;
  }
  /** 窄相检测后清除有接触刚体的 restless 标记，未接触刚体保留流体不平衡标记。 */
  clearRestless() {
    for (let c = this.contactManager.contactList; c !== null; c = c.next) {
      if (c.manifold.numPoints === 0) continue;
      if (c.b1 !== null) c.b1.restless = false;
      if (c.b2 !== null) c.b2.restless = false;
    }
  }
  /** 碰撞检测前按当前速度同步 speculativeStale 刚体的预测距离和包围盒。 */
  syncStaleBodies(t) {
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.speculativeStale && b.type !== RigidBodyType.STATIC) b.syncShapesSpeculative(t, this.speculativeDistance, this.maxSpeculativeDistance);
    }
  }
  /** 在地形请求之前唤醒可能受撞击的刚体，使其支撑碰撞体在本步恢复。 */
  wakeContacts(dt) {
    if (!this.softContacts || this.softSubsteps <= 0) return;
    this.syncStaleBodies(dt);
    this.wakeTouched();
  }
  /**
   * 唤醒与高速活动刚体包围盒重叠的休眠刚体，跳过 wakeLocked 对象。
   * 速度门槛为自身休眠阈值的两倍；包围盒包含预测接触距离。
   */
  wakeTouched() {
    let sleepers = false;
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.sleeping && b.type !== RigidBodyType.STATIC && !b.wakeLocked) {
        sleepers = true;
        break;
      }
    }
    if (!sleepers) return;
    const a = this.wakeAabb, callback = this.wakeCallback;
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.sleeping || b.type === RigidBodyType.STATIC || b.shapeList === null) continue;
      const lv = b.sleepingVelocityThreshold * 2, av = b.sleepingAngularVelocityThreshold * 2;
      if (!(b.vx * b.vx + b.vy * b.vy + b.vz * b.vz > lv * lv || b.wx * b.wx + b.wy * b.wy + b.wz * b.wz > av * av)) continue;
      a.copyFrom(b.shapeList.aabb);
      for (let s = b.shapeList.next; s !== null; s = s.next) {
        const t = s.aabb;
        if (t.minX < a.minX) a.minX = t.minX;
        if (t.minY < a.minY) a.minY = t.minY;
        if (t.minZ < a.minZ) a.minZ = t.minZ;
        if (t.maxX > a.maxX) a.maxX = t.maxX;
        if (t.maxY > a.maxY) a.maxY = t.maxY;
        if (t.maxZ > a.maxZ) a.maxZ = t.maxZ;
      }
      this.aabbTest(a, callback);
    }
  }
  /** 按岛执行软接触子步求解。 */
  solveIslandsSubstepped() {
    const island = this.island;
    island.setGravity(this.gravityX, this.gravityY, this.gravityZ);
    this.numIslands = 0;
    const n = this.softSubsteps;
    this.lastMaxSubsteps = n;
    const h = this.timeStep.dt / n;
    const sub = this.subTimeStep;
    const firstRatio = sub.dt > 0 ? h / sub.dt : 1;
    sub.dt = h;
    sub.invDt = 1 / h;
    const cap = this.softHertzStepRatio / h;
    this.softDynamic.set(Math.min(this.softContactHertz, cap), this.softDampingRatio, h);
    this.softFixed.set(Math.min(this.softContactHertz * this.softFixedHertzScale, cap * this.softFixedHertzScale), this.softDampingRatio, h);
    const p = this.substepParams;
    p.substeps = n;
    p.iterations = this.softIterations;
    p.relaxIterations = this.softRelaxIterations;
    p.tolerance = this.softTolerance;
    p.adaptiveIterations = this.softAdaptiveIterations;
    p.impactIterations = this.softImpactIterations;
    p.impactVelocity = this.softImpactVelocity;
    p.jointIterations = this.numVelocityIterations;
    p.positionIterations = this.numPositionIterations;
    p.maxPushVelocity = this.softMaxPushVelocity;
    p.restitutionIterations = this.softRestitutionIterations;
    p.restitutionThreshold = this.softRestitutionThreshold;
    p.maxLinearVelocity = this.softMaxLinearVelocity;
    p.maxAngularVelocity = this.softMaxAngularVelocity;
    const sea = this.voidSea;
    p.sea = sea !== null;
    if (sea !== null) {
      p.seaStartY = sea.startY;
      p.seaDepthGradient = sea.depthGradient;
      p.seaDrag = sea.drag;
    }
    p.parity = this.softParity;
    p.stamp = this.stepStamp;
    p.speculativeDistance = this.speculativeDistance;
    p.maxSpeculativeDistance = this.maxSpeculativeDistance;
    p.skin = this.softContactSkin;
    const clock = this.clock;
    const tick = this.timeStep;
    let minMass = Infinity;
    if (n < this.softSupportSubsteps) {
      for (let b = this.rigidBodyList; b !== null; b = b.next) {
        if (b.type === RigidBodyType.DYNAMIC && b.mass > 0 && b.mass < minMass) minMass = b.mass;
      }
    }
    let substepMs = 0;
    for (let b = this.rigidBodyList; b !== null; b = b.next) {
      if (b.addedToIsland || b.sleeping || b.type === RigidBodyType.STATIC) continue;
      const alone = b.isAlone();
      if (!alone) this.buildIsland(b, true);
      const count = alone ? b.mass >= minMass * 8 ? Math.max(n, this.softSupportSubsteps) : n : island.supportSubsteps(n, this.softSupportSubsteps);
      if (count !== p.substeps) {
        p.substeps = count;
        const dt = tick.dt / count, limit = this.softHertzStepRatio / dt;
        sub.dt = dt;
        sub.invDt = 1 / dt;
        this.softDynamic.set(Math.min(this.softContactHertz, limit), this.softDampingRatio, dt);
        this.softFixed.set(Math.min(this.softContactHertz * this.softFixedHertzScale, limit * this.softFixedHertzScale), this.softDampingRatio, dt);
      }
      if (count > this.lastMaxSubsteps) this.lastMaxSubsteps = count;
      sub.dtRatio = firstRatio;
      if (alone) {
        if (island.beginSingleRigidBodySubsteps(tick, b)) {
          const t0 = clock === null ? 0 : clock();
          island.runSingleRigidBodySubsteps(sub, p, b);
          if (clock !== null) substepMs += clock() - t0;
          island.finishSingleRigidBodySubsteps(tick, p, b);
        }
      } else {
        if (island.beginSubsteps(tick, sub, p)) {
          const t0 = clock === null ? 0 : clock();
          island.runSubsteps(sub, p);
          if (clock !== null) substepMs += clock() - t0;
          island.finishSubsteps(tick, p);
        }
        island.clear();
      }
      this.numIslands++;
    }
    this.lastSubstepMs = substepMs;
    sub.dtRatio = 1;
    sub.dt = h;
    sub.invDt = 1 / h;
    this.finishIslands();
  }
  /**
   * 从 base 沿接触和关节深度优先构建模拟岛。
   * 预测模式包含尚未接触的流形点；休眠刚体不加入岛，接触求解将其逆质量视为 0。
   */
  buildIsland(base, speculative = false) {
    const island = this.island, stack = this.rigidBodyStack;
    island.addRigidBody(base);
    stack.push(base);
    while (stack.length > 0) {
      const rb = stack.pop();
      if (rb.type === RigidBodyType.STATIC) continue;
      for (let cl = rb.contactLinkList; cl !== null; cl = cl.next) {
        const cc = cl.contact.contactConstraint, ccs = cc.solver;
        if ((speculative ? cc.manifold.numPoints > 0 : cc.isTouching()) && !ccs.addedToIsland) {
          this.solversInIslands.push(ccs);
          if (this.softContacts) island.addSoftContact(cc.softSolver, ccs);
          else island.addConstraintSolver(ccs, cc.positionCorrectionAlgorithm);
          const other = cl.other;
          if (speculative && other.sleeping && other.type !== RigidBodyType.STATIC) continue;
          if (!other.addedToIsland) {
            island.addRigidBody(other);
            stack.push(other);
          }
        }
      }
      for (let jl = rb.jointLinkList; jl !== null; jl = jl.next) {
        const j = jl.joint, js = j.solver;
        if (!js.addedToIsland) {
          this.solversInIslands.push(js);
          island.addConstraintSolver(js, j.positionCorrectionAlgorithm);
          const other = jl.other;
          if (!other.addedToIsland) {
            island.addRigidBody(other);
            stack.push(other);
          }
        }
      }
    }
  }
};
var RayCastWrapper = class {
  constructor() {
    this.rayCastHit = new RayCastHit();
    this.beginX = 0;
    this.beginY = 0;
    this.beginZ = 0;
    this.endX = 0;
    this.endY = 0;
    this.endZ = 0;
    this.callback = null;
  }
  process(proxy) {
    const shape = proxy.userData;
    if (shape.geom.rayCast(this.beginX, this.beginY, this.beginZ, this.endX, this.endY, this.endZ, shape.transform, this.rayCastHit)) {
      this.callback.process(shape, this.rayCastHit);
    }
  }
};
var ConvexCastWrapper = class {
  constructor() {
    this.rayCastHit = new RayCastHit();
    this.begin = new Transform();
    this.translation = new Vec3();
    this.zero = new Vec3();
    this.convex = null;
    this.callback = null;
  }
  process(proxy) {
    const shape = proxy.userData;
    const type = shape.geom.type;
    if (type < 0 || type > 5) return;
    const hit = this.rayCastHit;
    if (GjkEpa.getInstance().convexCast(this.convex, shape.geom, this.begin, shape.transform, this.translation, this.zero, hit)) {
      this.callback.process(shape, hit);
    }
  }
};
var AabbTestWrapper = class {
  constructor() {
    this.aabb = new Aabb();
    this.callback = null;
  }
  process(proxy) {
    const shape = proxy.userData;
    const s = shape.aabb, a = this.aabb;
    if (s.minX < a.maxX && s.maxX > a.minX && s.minY < a.maxY && s.maxY > a.minY && s.minZ < a.maxZ && s.maxZ > a.minZ) {
      this.callback.process(shape);
    }
  }
};

// src/world/chunk-boxes.ts
var CHUNK_SIZE = 8;
var CHUNK_CELLS = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;
var voxels = new Uint8Array(CHUNK_CELLS);
var groups = new Uint32Array(CHUNK_CELLS);
var present = new Uint8Array(MAX_MATERIALS);
var STRIDE = CHUNK_SIZE + 1;
var STRIDE_XZ = STRIDE * STRIDE;
var SINGLE = 1 + STRIDE + STRIDE_XZ;
function chunkBoxes(cells, table, ox, oy, oz, out) {
  const first = out.length;
  present.fill(0);
  for (let i = 0; i < CHUNK_CELLS; i++) {
    const id = cells[i], shape = shapeOf(id);
    if (shape === SHAPE_NONE) continue;
    const material = materialOf(id);
    const x = i & 7, z = i >> 3 & 7, y = i >> 6;
    if (material >= MAX_MATERIALS) throw new Error(`\u65B9\u5757 (${ox + x}, ${oy + y}, ${oz + z}) \u7684\u6750\u8D28\u7F16\u53F7 ${material} \u8D85\u51FA\u4E0A\u9650 ${MAX_MATERIALS - 1}`);
    if (shape === SHAPE_FULL_CUBE) {
      present[material] = 1;
      continue;
    }
    const boxes = table.get(shape);
    if (boxes === void 0) throw new Error(`\u65B9\u5757 (${ox + x}, ${oy + y}, ${oz + z}) \u7684\u5F62\u72B6\u7F16\u53F7 ${shape} \u6CA1\u6709\u767B\u8BB0`);
    for (const b of boxes) {
      out.push({ minX: ox + x + b[0], minY: oy + y + b[1], minZ: oz + z + b[2], sizeX: b[3] - b[0], sizeY: b[4] - b[1], sizeZ: b[5] - b[2], material });
    }
  }
  for (let m = 0; m < MAX_MATERIALS; m++) {
    if (present[m] === 0) continue;
    const full2 = packBlock(SHAPE_FULL_CUBE, m);
    for (let i = 0; i < CHUNK_CELLS; i++) voxels[i] = cells[i] === full2 ? 1 : 0;
    mergeVoxels();
    for (let i = 0; i < CHUNK_CELLS; i++) {
      const g = groups[i];
      if (g === 0) continue;
      const sizeX = g % STRIDE, sizeZ = Math.floor(g / STRIDE) % STRIDE, sizeY = Math.floor(g / STRIDE_XZ) % STRIDE;
      out.push({
        minX: ox + (i & 7) + 1 - sizeX,
        minY: oy + (i >> 6) + 1 - sizeY,
        minZ: oz + (i >> 3 & 7) + 1 - sizeZ,
        sizeX,
        sizeY,
        sizeZ,
        material: m
      });
    }
  }
  sortBoxes(out, first);
}
function mergeVoxels() {
  for (let i = 0; i < CHUNK_CELLS; i++) groups[i] = voxels[i] === 0 ? 0 : SINGLE;
  for (let row = 0; row < CHUNK_CELLS; row += CHUNK_SIZE) {
    for (let x = 1; x < CHUNK_SIZE; x++) {
      const i = row + x;
      if (voxels[i] === 0 || voxels[i - 1] === 0 || groups[i - 1] === 0) continue;
      groups[i] += groups[i - 1] % STRIDE;
      groups[i - 1] = 0;
    }
  }
  for (let y = 0; y < CHUNK_SIZE; y++) {
    for (let z = 1; z < CHUNK_SIZE; z++) {
      const row = y * 64 + z * CHUNK_SIZE;
      for (let x = 0; x < CHUNK_SIZE; x++) {
        const i = row + x, p = i - CHUNK_SIZE, pg = groups[p], cg = groups[i];
        if (voxels[i] === 0 || voxels[p] === 0 || pg === 0 || cg === 0 || pg % STRIDE !== cg % STRIDE) continue;
        groups[i] += Math.floor(pg / STRIDE) % STRIDE * STRIDE;
        groups[p] = 0;
      }
    }
  }
  for (let i = 64; i < CHUNK_CELLS; i++) {
    const p = i - 64, pg = groups[p], cg = groups[i];
    if (voxels[i] === 0 || voxels[p] === 0 || pg === 0 || cg === 0 || pg % STRIDE !== cg % STRIDE || Math.floor(pg / STRIDE) % STRIDE !== Math.floor(cg / STRIDE) % STRIDE) continue;
    groups[i] += Math.floor(pg / STRIDE_XZ) % STRIDE * STRIDE_XZ;
    groups[p] = 0;
  }
}
function sortBoxes(out, first) {
  const part = out.splice(first);
  part.sort((a, b) => a.material - b.material || a.minY - b.minY || a.minZ - b.minZ || a.minX - b.minX || a.sizeY - b.sizeY || a.sizeZ - b.sizeZ || a.sizeX - b.sizeX);
  for (let i = 0; i < part.length; i++) out.push(part[i]);
}

// src/world/terrain-scan.ts
var XZ_LIMIT = 4194304;
var XZ_RANGE = 8388608;
var Y_LIMIT = 64;
var Y_RANGE = 128;
var MAX_VELOCITY_MARGIN = 3;
var MAX_PREFETCH_MARGIN = CHUNK_SIZE;
var SHADOW_MAX_SHAPES = 4;
var SHADOW_MIN_VOLUME = 256;
function chunkKey(cx, cy, cz) {
  if (cx < -XZ_LIMIT || cx >= XZ_LIMIT || cz < -XZ_LIMIT || cz >= XZ_LIMIT || cy < -Y_LIMIT || cy >= Y_LIMIT) return -1;
  return ((cx + XZ_LIMIT) * XZ_RANGE + (cz + XZ_LIMIT)) * Y_RANGE + (cy + Y_LIMIT);
}
function chunkOrigin(key2, out) {
  const rest = Math.floor(key2 / Y_RANGE);
  out[0] = (Math.floor(rest / XZ_RANGE) - XZ_LIMIT) * CHUNK_SIZE;
  out[1] = (key2 % Y_RANGE - Y_LIMIT) * CHUNK_SIZE;
  out[2] = (rest % XZ_RANGE - XZ_LIMIT) * CHUNK_SIZE;
}
var localAabb = new Aabb();
var lb = [0, 0, 0, 0, 0, 0];
var cur = [0, 0, 0, 0, 0, 0];
var pred = [0, 0, 0, 0, 0, 0];
var rot = [1, 0, 0, 0, 1, 0, 0, 0, 1];
var prot = [1, 0, 0, 0, 1, 0, 0, 0, 1];
var members = [];
var shadows = [];
var shadowX = 0;
var shadowY = 0;
var shadowZ = 0;
var halfX = 0;
var halfY = 0;
var halfZ = 0;
function localBounds(shape) {
  shape.geom.computeAabb(localAabb, shape.localTransform);
  const a = localAabb;
  lb[0] = (a.minX + a.maxX) / 2;
  lb[1] = (a.minY + a.maxY) / 2;
  lb[2] = (a.minZ + a.maxZ) / 2;
  lb[3] = (a.maxX - a.minX) / 2;
  lb[4] = (a.maxY - a.minY) / 2;
  lb[5] = (a.maxZ - a.minZ) / 2;
  return Math.hypot(Math.max(Math.abs(a.minX), Math.abs(a.maxX)), Math.max(Math.abs(a.minY), Math.abs(a.maxY)), Math.max(Math.abs(a.minZ), Math.abs(a.maxZ)));
}
function worldBounds(r, px, py, pz, out) {
  const cx = lb[0], cy = lb[1], cz = lb[2], hx = lb[3], hy = lb[4], hz = lb[5];
  const wx = px + r[0] * cx + r[1] * cy + r[2] * cz, wy = py + r[3] * cx + r[4] * cy + r[5] * cz, wz = pz + r[6] * cx + r[7] * cy + r[8] * cz;
  const ex = Math.abs(r[0]) * hx + Math.abs(r[1]) * hy + Math.abs(r[2]) * hz;
  const ey = Math.abs(r[3]) * hx + Math.abs(r[4]) * hy + Math.abs(r[5]) * hz;
  const ez = Math.abs(r[6]) * hx + Math.abs(r[7]) * hy + Math.abs(r[8]) * hz;
  out[0] = wx - ex;
  out[1] = wy - ey;
  out[2] = wz - ez;
  out[3] = wx + ex;
  out[4] = wy + ey;
  out[5] = wz + ez;
}
function velocityMargin(v, t, margin, prefetch) {
  if (prefetch) return margin + Math.min(MAX_PREFETCH_MARGIN, Math.max(0, v * t));
  return Math.min(MAX_VELOCITY_MARGIN, margin + Math.abs(v) * t);
}
function loadRotation(body) {
  const tf = body.transform;
  rot[0] = tf.r00;
  rot[1] = tf.r01;
  rot[2] = tf.r02;
  rot[3] = tf.r10;
  rot[4] = tf.r11;
  rot[5] = tf.r12;
  rot[6] = tf.r20;
  rot[7] = tf.r21;
  rot[8] = tf.r22;
}
function predictedChunks(body, t, margin, prefetch, out) {
  out.length = 0;
  members.length = 0;
  const tf = body.transform;
  const px = tf.px, py = tf.py, pz = tf.pz, vx = body.vx, vy = body.vy, vz = body.vz, wx = body.wx, wy = body.wy, wz = body.wz;
  if (!Number.isFinite(px + py + pz + vx + vy + vz + wx + wy + wz) || body.shapeList === null) return;
  const mxP = velocityMargin(vx, t, margin, prefetch), myP = velocityMargin(vy, t, margin, prefetch), mzP = velocityMargin(vz, t, margin, prefetch);
  const mxN = velocityMargin(-vx, t, margin, prefetch), myN = velocityMargin(-vy, t, margin, prefetch), mzN = velocityMargin(-vz, t, margin, prefetch);
  const w = Math.sqrt(wx * wx + wy * wy + wz * wz), travel = w * t;
  loadRotation(body);
  const rotate = travel > 1e-12 && travel < Math.PI;
  if (rotate) {
    const ax = wx / w, ay = wy / w, az = wz / w, s = Math.sin(travel), c = Math.cos(travel), k = 1 - c;
    const d00 = c + ax * ax * k, d01 = ax * ay * k - az * s, d02 = ax * az * k + ay * s;
    const d10 = ay * ax * k + az * s, d11 = c + ay * ay * k, d12 = ay * az * k - ax * s;
    const d20 = az * ax * k - ay * s, d21 = az * ay * k + ax * s, d22 = c + az * az * k;
    for (let i = 0; i < 3; i++) {
      const r0 = rot[i], r1 = rot[3 + i], r2 = rot[6 + i];
      prot[i] = d00 * r0 + d01 * r1 + d02 * r2;
      prot[3 + i] = d10 * r0 + d11 * r1 + d12 * r2;
      prot[6 + i] = d20 * r0 + d21 * r1 + d22 * r2;
    }
  }
  let count = 0;
  for (let s = body.shapeList; s !== null; s = s.next) {
    const radius = localBounds(s);
    let x0, y0, z0, x1, y1, z1;
    if (travel >= Math.PI) {
      x0 = px - radius;
      y0 = py - radius;
      z0 = pz - radius;
      x1 = px + radius;
      y1 = py + radius;
      z1 = pz + radius;
    } else {
      worldBounds(rot, px, py, pz, cur);
      if (!rotate) {
        x0 = cur[0];
        y0 = cur[1];
        z0 = cur[2];
        x1 = cur[3];
        y1 = cur[4];
        z1 = cur[5];
      } else {
        worldBounds(prot, px, py, pz, pred);
        const sag = radius * (1 - Math.cos(travel / 2));
        x0 = Math.min(cur[0], pred[0]) - sag;
        y0 = Math.min(cur[1], pred[1]) - sag;
        z0 = Math.min(cur[2], pred[2]) - sag;
        x1 = Math.max(cur[3], pred[3]) + sag;
        y1 = Math.max(cur[4], pred[4]) + sag;
        z1 = Math.max(cur[5], pred[5]) + sag;
      }
    }
    members.push(Math.floor(x0 - mxN), Math.floor(y0 - myN), Math.floor(z0 - mzN), Math.floor(x1 + mxP), Math.floor(y1 + myP), Math.floor(z1 + mzP));
    count++;
  }
  let shadow = false;
  if (count <= SHADOW_MAX_SHAPES) {
    let x0 = Infinity, y0 = Infinity, z0 = Infinity, x1 = -Infinity, y1 = -Infinity, z1 = -Infinity;
    for (let i = 0; i < members.length; i += 6) {
      x0 = Math.min(x0, members[i]);
      y0 = Math.min(y0, members[i + 1]);
      z0 = Math.min(z0, members[i + 2]);
      x1 = Math.max(x1, members[i + 3]);
      y1 = Math.max(y1, members[i + 4]);
      z1 = Math.max(z1, members[i + 5]);
    }
    shadow = (x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) >= SHADOW_MIN_VOLUME;
  }
  if (shadow) prepareShadows(body, t, margin);
  enumerate(shadow, out);
}
function sleepingChunks(body, margin, out, bounds2) {
  out.length = 0;
  members.length = 0;
  if (!bodyBounds(body, cur)) return;
  for (let s = body.shapeList; s !== null; s = s.next) {
    localBounds(s);
    worldBounds(rot, body.transform.px, body.transform.py, body.transform.pz, pred);
    members.push(Math.floor(pred[0] - margin), Math.floor(pred[1] - margin), Math.floor(pred[2] - margin), Math.floor(pred[3] + margin), Math.floor(pred[4] + margin), Math.floor(pred[5] + margin));
  }
  if (bounds2 !== void 0) {
    bounds2[0] = Math.floor(cur[0] - margin);
    bounds2[1] = Math.floor(cur[1] - margin);
    bounds2[2] = Math.floor(cur[2] - margin);
    bounds2[3] = Math.floor(cur[3] + margin);
    bounds2[4] = Math.floor(cur[4] + margin);
    bounds2[5] = Math.floor(cur[5] + margin);
  }
  enumerate(false, out);
}
function fluidRange(body, up, range) {
  if (!bodyBounds(body, cur)) return false;
  range[0] = Math.floor(cur[0] - 1);
  range[1] = Math.floor(cur[1] - 1.5);
  range[2] = Math.floor(cur[2] - 1);
  range[3] = Math.floor(cur[3] + 1);
  range[4] = Math.floor(cur[4] + (up ? 64 : 0));
  range[5] = Math.floor(cur[5] + 1);
  return true;
}
function rangeChunks(range, out) {
  out.length = 0;
  members.length = 0;
  members.push(range[0], range[1], range[2], range[3], range[4], range[5]);
  enumerate(false, out);
}
function bodyBounds(body, out) {
  const tf = body.transform;
  if (!Number.isFinite(tf.px + tf.py + tf.pz) || body.shapeList === null) return false;
  loadRotation(body);
  out[0] = Infinity;
  out[1] = Infinity;
  out[2] = Infinity;
  out[3] = -Infinity;
  out[4] = -Infinity;
  out[5] = -Infinity;
  for (let s = body.shapeList; s !== null; s = s.next) {
    localBounds(s);
    worldBounds(rot, tf.px, tf.py, tf.pz, pred);
    for (let i = 0; i < 3; i++) {
      if (pred[i] < out[i]) out[i] = pred[i];
      if (pred[i + 3] > out[i + 3]) out[i + 3] = pred[i + 3];
    }
  }
  return true;
}
function prepareShadows(body, t, margin) {
  const tf = body.transform;
  shadowX = tf.px;
  shadowY = tf.py;
  shadowZ = tf.pz;
  halfX = 0.5 * (Math.abs(rot[0]) + Math.abs(rot[3]) + Math.abs(rot[6]));
  halfY = 0.5 * (Math.abs(rot[1]) + Math.abs(rot[4]) + Math.abs(rot[7]));
  halfZ = 0.5 * (Math.abs(rot[2]) + Math.abs(rot[5]) + Math.abs(rot[8]));
  const linear = Math.hypot(body.vx, body.vy, body.vz) * t, angular = Math.hypot(body.wx, body.wy, body.wz) * t;
  shadows.length = 0;
  for (let s = body.shapeList; s !== null; s = s.next) {
    const radius = localBounds(s);
    const e = margin + linear + (angular >= Math.PI ? radius * 2 : radius * angular) + 1e-6;
    shadows.push(lb[0] - lb[3] - e, lb[1] - lb[4] - e, lb[2] - lb[5] - e, lb[0] + lb[3] + e, lb[1] + lb[4] + e, lb[2] + lb[5] + e);
  }
}
function retained(ox, oy, oz) {
  const dx = ox + 4 - shadowX, dy = oy + 4 - shadowY, dz = oz + 4 - shadowZ;
  const lx = rot[0] * dx + rot[3] * dy + rot[6] * dz, ly = rot[1] * dx + rot[4] * dy + rot[7] * dz, lz = rot[2] * dx + rot[5] * dy + rot[8] * dz;
  const ex = halfX * 8, ey = halfY * 8, ez = halfZ * 8;
  for (let i = 0; i < shadows.length; i += 6) {
    if (lx + ex >= shadows[i] && lx - ex <= shadows[i + 3] && ly + ey >= shadows[i + 1] && ly - ey <= shadows[i + 4] && lz + ez >= shadows[i + 2] && lz - ez <= shadows[i + 5]) return true;
  }
  return false;
}
var seen = /* @__PURE__ */ new Set();
function enumerate(shadow, out) {
  seen.clear();
  for (let i = 0; i < members.length; i += 6) {
    const cx0 = Math.max(-XZ_LIMIT, Math.floor(members[i] / CHUNK_SIZE)), cx1 = Math.min(XZ_LIMIT - 1, Math.floor(members[i + 3] / CHUNK_SIZE));
    const cy0 = Math.max(-Y_LIMIT, Math.floor(members[i + 1] / CHUNK_SIZE)), cy1 = Math.min(Y_LIMIT - 1, Math.floor(members[i + 4] / CHUNK_SIZE));
    const cz0 = Math.max(-XZ_LIMIT, Math.floor(members[i + 2] / CHUNK_SIZE)), cz1 = Math.min(XZ_LIMIT - 1, Math.floor(members[i + 5] / CHUNK_SIZE));
    for (let cy = cy0; cy <= cy1; cy++) for (let cz = cz0; cz <= cz1; cz++) for (let cx = cx0; cx <= cx1; cx++) {
      const key2 = chunkKey(cx, cy, cz);
      if (seen.has(key2)) continue;
      seen.add(key2);
      if (shadow && !retained(cx * CHUNK_SIZE, cy * CHUNK_SIZE, cz * CHUNK_SIZE)) continue;
      out.push(key2);
    }
  }
}

// src/world/voxel-collider.ts
var PRUNE_INTERVAL = 20;
var NORMAL_BUILDS = 2;
var TICK = 0.05;
var FACE_INSET = 1e-5;
var scratchCells = new Int32Array(CHUNK_CELLS);
var scratchTypes = new Int32Array(CHUNK_CELLS);
var scratchFluids = new Uint8Array(CHUNK_CELLS);
var boxScratch = [];
var fluidScratch = [0, 0, 0, 0, 0, 0];
var keyScratch = [];
var origin = [0, 0, 0];
var bounds = [0, 0, 0, 0, 0, 0];
var dirtyScratch = [];
var changedScratch = [];
var EMPTY = [];
var EMPTY_SET = /* @__PURE__ */ new Set();
function sameKeys(set, count, next) {
  if (count !== next.length) return false;
  for (let i = 0; i < next.length; i++) if (!set.has(next[i])) return false;
  return true;
}
var VoxelCollider = class {
  constructor(world5, source, options = {}) {
    /** 已缓存的、有流体的块数 */
    this.fluidChunks = 0;
    this.chunks = /* @__PURE__ */ new Map();
    /** 所有请求（当前、睡眠、预读）对每个块的引用数，和其中 active 请求的引用数 */
    this.refs = /* @__PURE__ */ new Map();
    this.activeRefs = /* @__PURE__ */ new Map();
    /** 两条队列：Set 的插入顺序就是先进先出的顺序 */
    this.high = /* @__PURE__ */ new Set();
    this.normal = /* @__PURE__ */ new Set();
    this.states = /* @__PURE__ */ new Map();
    /** 停用的刚体（见 setParked） */
    this.parked = /* @__PURE__ */ new Set();
    this.updateCount = 0;
    this.lastPrune = 0;
    /** chunkAt 上一次找到的块（删掉块时清空） */
    this.lastChunk = null;
    this.group = /* @__PURE__ */ new Set();
    this.groupQueue = [];
    this.wakeAabb = new Aabb();
    this.wakeQueue = [];
    this.wakeCallback = {
      process: (shape) => {
        const b = shape.rigidBody;
        if (!b.sleeping || b.type !== RigidBodyType.DYNAMIC || b.wakeLocked) return;
        b.wakeUp();
        this.wakeQueue.push(b);
      }
    };
    /**
     * 过滤地形内部面的接触点。
     * 将接触点限制在盒内并沿法线外推；探测点位于相邻盒内时丢弃该接触。
     * 探测点仍在当前盒内时保留。
     */
    this.filterContact = (shape, result, first) => {
      const g = shape.geom, t = shape.transform;
      const nx = first ? -result.nx : result.nx, ny = first ? -result.ny : result.ny, nz = first ? -result.nz : result.nz;
      const ix = Math.min(FACE_INSET, g.hx / 4), iy = Math.min(FACE_INSET, g.hy / 4), iz = Math.min(FACE_INSET, g.hz / 4);
      const ox = nx * ix * 2, oy = ny * iy * 2, oz = nz * iz * 2;
      for (let i = 0; i < result.numPoints; ) {
        const p = result.points[i];
        const lx = (first ? p.p1x : p.p2x) - t.px, ly = (first ? p.p1y : p.p2y) - t.py, lz = (first ? p.p1z : p.p2z) - t.pz;
        const x = Math.max(-g.hx + ix, Math.min(g.hx - ix, lx)) + ox;
        const y = Math.max(-g.hy + iy, Math.min(g.hy - iy, ly)) + oy;
        const z = Math.max(-g.hz + iz, Math.min(g.hz - iz, lz)) + oz;
        const own = Math.abs(x) < g.hx && Math.abs(y) < g.hy && Math.abs(z) < g.hz;
        if (!own && this.containsPoint(t.px + x, t.py + y, t.pz + z)) {
          result.points[i] = result.points[--result.numPoints];
          result.points[result.numPoints] = p;
        } else {
          i++;
        }
      }
    };
    this.world = world5;
    this.source = source;
    this.table = options.table ?? new BlockShapeTable();
    this.materials = options.materials?.map((m) => ({ ...m })) ?? [{ friction: 0.6, restitution: 0 }];
    if (this.materials.length === 0) throw new Error("materials \u81F3\u5C11\u8981\u6709\u4E00\u9879\uFF080 \u53F7\u6750\u8D28\uFF09");
    this.margin = options.margin ?? 1;
    this.preloadTicks = options.preloadTicks ?? 3;
    this.maxChunksPerUpdate = options.maxChunksPerUpdate ?? 4;
    this.revalidateBlocksPerUpdate = options.revalidateBlocksPerUpdate ?? 12;
    this.unloadDelay = options.unloadDelay ?? 400;
    this.suspend = options.suspend ?? true;
    this.fluids = options.fluids ?? false;
    if (!(this.margin >= 0) || !Number.isFinite(this.margin)) throw new Error(`margin \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 0 \u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${this.margin}`);
    if (!Number.isSafeInteger(this.preloadTicks) || this.preloadTicks < 0) throw new Error(`preloadTicks \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 0 \u7684\u6574\u6570\uFF0C\u6536\u5230 ${this.preloadTicks}`);
    if (!Number.isSafeInteger(this.maxChunksPerUpdate) || this.maxChunksPerUpdate < 1) throw new Error(`maxChunksPerUpdate \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 1 \u7684\u6574\u6570\uFF0C\u6536\u5230 ${this.maxChunksPerUpdate}`);
    if (!Number.isSafeInteger(this.revalidateBlocksPerUpdate) || this.revalidateBlocksPerUpdate < 0) throw new Error(`revalidateBlocksPerUpdate \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 0 \u7684\u6574\u6570\uFF0C\u6536\u5230 ${this.revalidateBlocksPerUpdate}`);
    if (!Number.isSafeInteger(this.unloadDelay) || this.unloadDelay < 0) throw new Error(`unloadDelay \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 0 \u7684\u6574\u6570\uFF0C\u6536\u5230 ${this.unloadDelay}`);
  }
  /** 缓存的块数 */
  get cachedChunks() {
    return this.chunks.size;
  }
  /** 挂在世界上的块数（包括没有碰撞的块） */
  get attachedChunks() {
    let n = 0;
    for (const c of this.chunks.values()) if (c.attached) n++;
    return n;
  }
  /** 排队等着读的块数 */
  get pendingChunks() {
    return this.high.size + this.normal.size;
  }
  /** 刚体是否等待地形，包括已停用的刚体。 */
  isWaiting(body) {
    return this.states.get(body)?.waiting === true;
  }
  /**
   * 切换刚体停用状态。
   * 停用时释放地形请求并冻结关节相连的一组刚体；恢复时等待当前地形就绪。suspend=false 时无效果。
   */
  setParked(body, parked) {
    if (!this.suspend) return;
    if (parked) this.parked.add(body);
    else this.parked.delete(body);
  }
  isParked(body) {
    return this.parked.has(body);
  }
  /**
   * 一格的流体码（见 block-shapes.ts）。只查缓存，块没读到时返回 0（当作干的）。
   * 刚体当前范围、睡眠范围每侧外扩 margin，margin 不小于 1 时覆盖流体计算要查的格子
   */
  fluidAt(x, y, z) {
    const c = this.chunkAt(x, y, z);
    return c === null ? 0 : c.fluids[x - c.ox + (z - c.oz) * CHUNK_SIZE + (y - c.oy) * 64];
  }
  /** 已缓存块中是否存在流体。 */
  hasAnyFluid() {
    return this.fluidChunks > 0;
  }
  /** 一格没有碰撞（只查缓存，块没读到时返回 false） */
  isTerrainEmpty(x, y, z) {
    const c = this.chunkAt(x, y, z);
    return c !== null && shapeOf(c.cells[x - c.ox + (z - c.oz) * CHUNK_SIZE + (y - c.oy) * 64]) === SHAPE_NONE;
  }
  /** 方块坐标闭区间里有没有流体：只看覆盖到的已缓存块的流体格数 */
  hasFluidNear(minX, minY, minZ, maxX, maxY, maxZ) {
    const x0 = Math.floor(minX / CHUNK_SIZE), y0 = Math.floor(minY / CHUNK_SIZE), z0 = Math.floor(minZ / CHUNK_SIZE);
    const x1 = Math.floor(maxX / CHUNK_SIZE), y1 = Math.floor(maxY / CHUNK_SIZE), z1 = Math.floor(maxZ / CHUNK_SIZE);
    for (let cy = y0; cy <= y1; cy++) {
      for (let cz = z0; cz <= z1; cz++) {
        for (let cx = x0; cx <= x1; cx++) {
          const c = this.chunks.get(chunkKey(cx, cy, cz));
          if (c !== void 0 && c.fluidCount > 0) return true;
        }
      }
    }
    return false;
  }
  /** 方块所在的已缓存块，没有时返回 null。连续查同一块时不查表 */
  chunkAt(x, y, z) {
    let c = this.lastChunk;
    if (c === null || x < c.ox || x >= c.ox + CHUNK_SIZE || y < c.oy || y >= c.oy + CHUNK_SIZE || z < c.oz || z >= c.oz + CHUNK_SIZE) {
      c = this.chunks.get(chunkKey(Math.floor(x / CHUNK_SIZE), Math.floor(y / CHUNK_SIZE), Math.floor(z / CHUNK_SIZE))) ?? null;
      if (c === null) return null;
      this.lastChunk = c;
    }
    return c;
  }
  /**
   * 使方块闭区间内的缓存失效，向外扩展 1 格以覆盖邻接形状。
   * cachedOnly=true 仅处理已缓存块；false 同时重新排队未缓存的被引用块。
   */
  blocksChanged(minX, minY, minZ, maxX, maxY, maxZ, cachedOnly = false) {
    if (!Number.isFinite(minX + minY + minZ + maxX + maxY + maxZ)) return;
    const x0 = Math.floor(minX) - 1, y0 = Math.floor(minY) - 1, z0 = Math.floor(minZ) - 1;
    const x1 = Math.floor(maxX) + 1, y1 = Math.floor(maxY) + 1, z1 = Math.floor(maxZ) + 1;
    if (x0 > x1 || y0 > y1 || z0 > z1) return;
    const cx0 = Math.floor(x0 / CHUNK_SIZE), cy0 = Math.floor(y0 / CHUNK_SIZE), cz0 = Math.floor(z0 / CHUNK_SIZE);
    const cx1 = Math.floor(x1 / CHUNK_SIZE), cy1 = Math.floor(y1 / CHUNK_SIZE), cz1 = Math.floor(z1 / CHUNK_SIZE);
    const span = (cx1 - cx0 + 1) * (cy1 - cy0 + 1) * (cz1 - cz0 + 1);
    if (span > this.chunks.size + this.refs.size) {
      keyScratch.length = 0;
      for (const key2 of this.chunks.keys()) keyScratch.push(key2);
      if (!cachedOnly) {
        for (const key2 of this.refs.keys()) if (!this.chunks.has(key2)) keyScratch.push(key2);
      }
      for (let i = 0; i < keyScratch.length; i++) {
        const key2 = keyScratch[i];
        chunkOrigin(key2, origin);
        const cx = origin[0] / CHUNK_SIZE, cy = origin[1] / CHUNK_SIZE, cz = origin[2] / CHUNK_SIZE;
        if (cx >= cx0 && cx <= cx1 && cy >= cy0 && cy <= cy1 && cz >= cz0 && cz <= cz1) this.invalidate(key2, x0, y0, z0, x1, y1, z1);
      }
      return;
    }
    for (let cy = cy0; cy <= cy1; cy++) for (let cz = cz0; cz <= cz1; cz++) for (let cx = cx0; cx <= cx1; cx++) {
      const key2 = chunkKey(cx, cy, cz);
      if (key2 < 0 || cachedOnly && !this.chunks.has(key2)) continue;
      this.invalidate(key2, x0, y0, z0, x1, y1, z1);
    }
  }
  /**
   * 更新地形缓存和刚体等待状态。
   * 在 world.step 前调用；dt 为下一步的时长，单位为秒。
   */
  update(dt = TICK) {
    this.world.wakeContacts(dt);
    this.updateCount++;
    if (this.updateCount - this.lastPrune >= PRUNE_INTERVAL) {
      this.prune();
      this.lastPrune = this.updateCount;
    }
    this.revalidate();
    const tick = this.updateCount;
    for (let b = this.world.rigidBodyList; b !== null; b = b.next) {
      if (b.type !== RigidBodyType.DYNAMIC) continue;
      let s = this.states.get(b);
      if (s === void 0) {
        s = this.newState(b);
        this.states.set(b, s);
      }
      s.seen = tick;
      if (this.parked.has(b)) {
        this.setRequest(s, EMPTY, false);
        this.releasePrefetch(s);
        this.releaseFluid(s);
        this.pause(s);
        continue;
      }
      if (s.waiting) {
        b.fx = 0;
        b.fy = 0;
        b.fz = 0;
        b.tx = 0;
        b.ty = 0;
        b.tz = 0;
        this.requestCurrent(s, dt);
        if (!this.ready(s)) {
          this.pause(s);
          continue;
        }
        this.resume(s);
      }
      if (b.sleeping) {
        this.releaseFluid(s);
        this.requestSleeping(s);
        continue;
      }
      this.requestCurrent(s, dt);
      this.requestPrefetch(s, dt);
      this.requestFluid(s);
    }
    this.dropLeft(tick);
    const built = this.processQueue(this.high, this.maxChunksPerUpdate);
    this.processQueue(this.normal, Math.max(0, NORMAL_BUILDS + this.maxChunksPerUpdate - built));
    if (this.suspend) this.settle(dt);
  }
  /** 移除全部块并解除刚体的地形等待状态。 */
  dispose() {
    for (const c of this.chunks.values()) this.detach(c);
    for (const s of this.states.values()) {
      if (!s.waiting) continue;
      s.body.wakeLocked = false;
      s.body.wakeUp();
    }
    this.chunks.clear();
    this.fluidChunks = 0;
    this.lastChunk = null;
    this.refs.clear();
    this.activeRefs.clear();
    this.high.clear();
    this.normal.clear();
    this.states.clear();
    this.parked.clear();
  }
  /** 按当前请求的就绪状态冻结或恢复刚体，关节相连的一组保持一致。 */
  settle(dt) {
    const group = this.group, queue = this.groupQueue;
    group.clear();
    queue.length = 0;
    for (const s of this.states.values()) {
      const b = s.body;
      if (this.parked.has(b)) {
        if (!group.has(b)) {
          group.add(b);
          queue.push(b);
        }
        continue;
      }
      if (b.sleeping && !s.waiting) continue;
      if (!s.active) this.requestCurrent(s, dt);
      if (!this.ready(s) && !group.has(b)) {
        group.add(b);
        queue.push(b);
      }
    }
    for (let i = 0; i < queue.length; i++) {
      for (let l = queue[i].jointLinkList; l !== null; l = l.next) {
        const other = l.other;
        if (other.type !== RigidBodyType.DYNAMIC || group.has(other) || !this.states.has(other)) continue;
        group.add(other);
        queue.push(other);
      }
    }
    for (const s of this.states.values()) {
      if (group.has(s.body)) this.pause(s);
      else if (s.waiting) this.resume(s);
    }
  }
  newState(body) {
    return {
      body,
      keys: [],
      keySet: /* @__PURE__ */ new Set(),
      active: false,
      prefetch: [],
      prefetchSet: /* @__PURE__ */ new Set(),
      fluid: [],
      fluidSet: /* @__PURE__ */ new Set(),
      fluidRange: [0, 0, 0, 0, 0, 0],
      fluidKeys: null,
      fluidUp: false,
      waiting: false,
      cursor: 0,
      seen: 0,
      pose: [NaN, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    };
  }
  /** 当前请求的块是否全部缓存且未失效。 */
  ready(s) {
    const keys = s.keys;
    for (let i = 0; i < keys.length; i++) {
      const c = this.chunks.get(keys[i]);
      if (c === void 0 || c.dirty.size > 0) return false;
    }
    return true;
  }
  /** 冻结刚体，保留速度、锁定唤醒并清除本步外力。 */
  pause(s) {
    const b = s.body;
    s.waiting = true;
    b.wakeLocked = true;
    if (!b.sleeping) b.sleep();
    b.fx = 0;
    b.fy = 0;
    b.fz = 0;
    b.tx = 0;
    b.ty = 0;
    b.tz = 0;
  }
  resume(s) {
    s.waiting = false;
    s.body.wakeLocked = false;
    this.wakeWithNeighbors(s.body);
  }
  requestCurrent(s, dt) {
    predictedChunks(s.body, dt, this.margin, false, keyScratch);
    this.setRequest(s, keyScratch, true);
  }
  /** 睡眠范围：位姿没变就沿用上一次的 */
  requestSleeping(s) {
    const tf = s.body.transform, p = s.pose;
    if (!s.active && p[0] === tf.px && p[1] === tf.py && p[2] === tf.pz && p[3] === tf.r00 && p[4] === tf.r01 && p[5] === tf.r02 && p[6] === tf.r10 && p[7] === tf.r11 && p[8] === tf.r12 && p[9] === tf.r20 && p[10] === tf.r21 && p[11] === tf.r22) return;
    p[0] = tf.px;
    p[1] = tf.py;
    p[2] = tf.pz;
    p[3] = tf.r00;
    p[4] = tf.r01;
    p[5] = tf.r02;
    p[6] = tf.r10;
    p[7] = tf.r11;
    p[8] = tf.r12;
    p[9] = tf.r20;
    p[10] = tf.r21;
    p[11] = tf.r22;
    this.releasePrefetch(s);
    sleepingChunks(s.body, this.margin, keyScratch);
    this.setRequest(s, keyScratch, false);
  }
  /** 更新当前范围外的预读请求。使用普通队列，仅计总引用，不影响就绪状态。 */
  requestPrefetch(s, dt) {
    if (this.preloadTicks === 0) return;
    predictedChunks(s.body, dt * this.preloadTicks, this.margin, true, keyScratch);
    let n = 0;
    for (let i = 0; i < keyScratch.length; i++) if (!s.keySet.has(keyScratch[i])) keyScratch[n++] = keyScratch[i];
    keyScratch.length = n;
    if (sameKeys(s.prefetchSet, s.prefetch.length, keyScratch)) return;
    const old = s.prefetch, oldSet = s.prefetchSet;
    const next = keyScratch.slice(), nextSet = new Set(next);
    for (let i = 0; i < next.length; i++) if (!oldSet.has(next[i])) this.addRef(next[i]);
    for (let i = 0; i < old.length; i++) if (!nextSet.has(old[i])) this.releaseRef(old[i]);
    s.prefetch = next;
    s.prefetchSet = nextSet;
  }
  /** 是否将流体请求向上扩展 64 格，供气腔液面查询使用；仅影响后续请求。 */
  setFluidReach(body, up) {
    const s = this.states.get(body);
    if (s !== void 0) s.fluidUp = up;
  }
  /** 更新当前范围外的流体请求。使用普通队列，仅计总引用，不影响就绪状态。 */
  requestFluid(s) {
    if (!this.fluids) return;
    const r = fluidScratch;
    if (!fluidRange(s.body, s.fluidUp, r)) {
      this.releaseFluid(s);
      return;
    }
    const p = s.fluidRange;
    if (s.fluidKeys === s.keys && p[0] === r[0] && p[1] === r[1] && p[2] === r[2] && p[3] === r[3] && p[4] === r[4] && p[5] === r[5]) return;
    p[0] = r[0];
    p[1] = r[1];
    p[2] = r[2];
    p[3] = r[3];
    p[4] = r[4];
    p[5] = r[5];
    s.fluidKeys = s.keys;
    rangeChunks(r, keyScratch);
    let n = 0;
    for (let i = 0; i < keyScratch.length; i++) if (!s.keySet.has(keyScratch[i])) keyScratch[n++] = keyScratch[i];
    keyScratch.length = n;
    if (sameKeys(s.fluidSet, s.fluid.length, keyScratch)) return;
    const old = s.fluid, oldSet = s.fluidSet;
    const next = keyScratch.slice(), nextSet = new Set(next);
    for (let i = 0; i < next.length; i++) if (!oldSet.has(next[i])) this.addRef(next[i]);
    for (let i = 0; i < old.length; i++) if (!nextSet.has(old[i])) this.releaseRef(old[i]);
    s.fluid = next;
    s.fluidSet = nextSet;
  }
  releaseFluid(s) {
    s.fluidKeys = null;
    if (s.fluid.length === 0) return;
    for (let i = 0; i < s.fluid.length; i++) this.releaseRef(s.fluid[i]);
    s.fluid = EMPTY;
    s.fluidSet = EMPTY_SET;
  }
  releasePrefetch(s) {
    for (let i = 0; i < s.prefetch.length; i++) this.releaseRef(s.prefetch[i]);
    s.prefetch = EMPTY;
    s.prefetchSet = EMPTY_SET;
  }
  /** 替换当前请求，先更新活动引用，再更新总引用。等待地形的请求计为活动。 */
  setRequest(s, next, active) {
    const same = sameKeys(s.keySet, s.keys.length, next);
    if (same && s.active === active) return;
    const old = s.keys, oldSet = s.keySet;
    const keys = same ? old : next.slice(), keySet = same ? oldSet : new Set(keys);
    if (s.active) {
      for (let i = 0; i < old.length; i++) if (!active || !keySet.has(old[i])) this.releaseActive(old[i]);
    }
    if (active) {
      for (let i = 0; i < keys.length; i++) if (!s.active || !oldSet.has(keys[i])) this.addActive(keys[i]);
    }
    if (!same) {
      for (let i = 0; i < keys.length; i++) if (!oldSet.has(keys[i])) this.addRef(keys[i]);
      for (let i = 0; i < old.length; i++) if (!keySet.has(old[i])) this.releaseRef(old[i]);
    }
    s.keys = keys;
    s.keySet = keySet;
    s.active = active;
  }
  addRef(key2) {
    const n = this.refs.get(key2) ?? 0;
    this.refs.set(key2, n + 1);
    if (n === 0) this.enqueue(key2, false);
  }
  releaseRef(key2) {
    const n = this.refs.get(key2) ?? 0;
    if (n > 1) {
      this.refs.set(key2, n - 1);
      return;
    }
    this.refs.delete(key2);
    this.high.delete(key2);
    this.normal.delete(key2);
    const c = this.chunks.get(key2);
    if (c !== void 0) c.lastUsed = this.updateCount;
  }
  addActive(key2) {
    const n = this.activeRefs.get(key2) ?? 0;
    this.activeRefs.set(key2, n + 1);
    if (n > 0) return;
    const c = this.chunks.get(key2);
    if (c !== void 0 && c.dirty.size === 0) this.attach(c);
    else this.enqueue(key2, true);
  }
  releaseActive(key2) {
    const n = this.activeRefs.get(key2) ?? 0;
    if (n > 1) {
      this.activeRefs.set(key2, n - 1);
      return;
    }
    this.activeRefs.delete(key2);
    const c = this.chunks.get(key2);
    if (c !== void 0) this.detach(c);
    if (this.high.delete(key2) && this.refs.has(key2)) this.normal.add(key2);
  }
  /** 未缓存或已失效的块入队；高优先请求升级队列，已入队项保留顺序。 */
  enqueue(key2, highPriority) {
    const c = this.chunks.get(key2);
    if (c !== void 0 && c.dirty.size === 0) return;
    if (highPriority) {
      this.normal.delete(key2);
      this.high.add(key2);
    } else if (!this.high.has(key2)) {
      this.normal.add(key2);
    }
  }
  /** 释放离开世界或不再动态的刚体请求，并解除其地形等待状态。 */
  dropLeft(tick) {
    for (const [b, s] of this.states) {
      if (s.seen === tick) continue;
      this.setRequest(s, EMPTY, false);
      this.releasePrefetch(s);
      this.releaseFluid(s);
      if (s.waiting) {
        b.wakeLocked = false;
        b.wakeUp();
      }
      this.states.delete(b);
      this.parked.delete(b);
    }
  }
  /**
   * 按队列顺序尝试读取最多 budget 个块，返回尝试次数。
   * 读取失败的块移至队尾，并停止本次队列读取。
   */
  processQueue(queue, budget) {
    if (budget <= 0) return 0;
    let built = 0;
    for (const key2 of queue) {
      queue.delete(key2);
      const c = this.chunks.get(key2);
      if (c !== void 0 && c.dirty.size === 0) continue;
      built++;
      if (!this.scan(key2, c)) {
        queue.add(key2);
        break;
      }
      if (built >= budget) break;
    }
    return built;
  }
  /** 读一块：没缓存的整块读，缓存了的只重读脏格。读完返回 true */
  scan(key2, old) {
    if (old === void 0) {
      chunkOrigin(key2, origin);
      const ox = origin[0], oy = origin[1], oz = origin[2];
      const cells = new Int32Array(CHUNK_CELLS), types = new Int32Array(CHUNK_CELLS), fluids = new Uint8Array(CHUNK_CELLS);
      if (!this.source.readChunk(ox, oy, oz, cells, types, fluids)) return false;
      const c = {
        ox,
        oy,
        oz,
        cells,
        types,
        fluids,
        fluidCount: countFluids(fluids),
        dirty: /* @__PURE__ */ new Set(),
        body: null,
        attached: false,
        lastUsed: this.updateCount
      };
      c.body = this.buildBody(c);
      this.chunks.set(key2, c);
      if (c.fluidCount > 0) this.fluidChunks++;
      if (this.activeRefs.has(key2)) this.attach(c);
      return true;
    }
    const indices = dirtyScratch;
    indices.length = 0;
    for (const i of old.dirty) indices.push(i);
    scratchCells.set(old.cells);
    scratchTypes.set(old.types);
    scratchFluids.set(old.fluids);
    if (!this.source.refreshCells(old.ox, old.oy, old.oz, scratchCells, scratchTypes, scratchFluids, indices)) return false;
    const changed = changedScratch;
    changed.length = 0;
    let solid = false;
    for (let n = 0; n < indices.length; n++) {
      const i = indices[n];
      if (scratchCells[i] !== old.cells[i]) {
        solid = true;
        changed.push(i);
      } else if (scratchFluids[i] !== old.fluids[i]) changed.push(i);
    }
    old.dirty.clear();
    old.types.set(scratchTypes);
    old.lastUsed = this.updateCount;
    if (changed.length > 0) {
      const had = old.fluidCount > 0;
      old.fluids.set(scratchFluids);
      old.fluidCount = countFluids(old.fluids);
      if (had && old.fluidCount === 0) this.fluidChunks--;
      else if (!had && old.fluidCount > 0) this.fluidChunks++;
    }
    if (!solid) {
      if (this.activeRefs.has(key2)) this.attach(old);
      if (changed.length > 0) this.wakeChanged(key2, old, changed);
      return true;
    }
    old.cells.set(scratchCells);
    const wasAttached = old.attached;
    if (wasAttached && old.body !== null) this.world.removeRigidBody(old.body, false);
    old.attached = false;
    old.body = this.buildBody(old);
    if (this.activeRefs.has(key2)) this.attach(old);
    this.wakeChanged(key2, old, changed);
    return true;
  }
  attach(c) {
    if (c.attached || c.dirty.size > 0) return;
    if (c.body !== null) this.world.addRigidBody(c.body);
    c.attached = true;
  }
  /** 摘下块的静态体，不叫醒接触着它的刚体 */
  detach(c) {
    if (!c.attached) return;
    if (c.body !== null) this.world.removeRigidBody(c.body, false);
    c.attached = false;
  }
  /** 块里的盒子做成一个静态体，放在块中心；没有盒子时返回 null */
  buildBody(c) {
    const boxes = boxScratch;
    boxes.length = 0;
    chunkBoxes(c.cells, this.table, c.ox, c.oy, c.oz, boxes);
    if (boxes.length === 0) return null;
    const config = new RigidBodyConfig();
    config.type = RigidBodyType.STATIC;
    const cx = c.ox + CHUNK_SIZE / 2, cy = c.oy + CHUNK_SIZE / 2, cz = c.oz + CHUNK_SIZE / 2;
    config.transform.px = cx;
    config.transform.py = cy;
    config.transform.pz = cz;
    const body = new RigidBody(config);
    const shapes = [];
    for (let i = 0; i < boxes.length; i++) {
      const b = boxes[i], hx = b.sizeX / 2, hy = b.sizeY / 2, hz = b.sizeZ / 2;
      const sc = new ShapeConfig(new BoxGeometry(hx, hy, hz));
      sc.transform.px = b.minX + hx - cx;
      sc.transform.py = b.minY + hy - cy;
      sc.transform.pz = b.minZ + hz - cz;
      const m = this.materials[b.material] ?? this.materials[0];
      sc.friction = m.friction;
      sc.restitution = m.restitution;
      sc.collisionMask = -1;
      const shape = new Shape(sc);
      shape.contactFilter = this.filterContact;
      shapes.push(shape);
    }
    body.addShapes(shapes);
    return body;
  }
  /** 点是否在挂在世界上的块的碰撞里。只查缓存，求解期间不读方块 */
  containsPoint(x, y, z) {
    const bx = Math.floor(x), by = Math.floor(y), bz = Math.floor(z);
    for (let dy = 0; dy <= 1; dy++) {
      const yy = by - dy;
      const c = this.chunks.get(chunkKey(Math.floor(bx / CHUNK_SIZE), Math.floor(yy / CHUNK_SIZE), Math.floor(bz / CHUNK_SIZE)));
      if (c === void 0 || !c.attached) continue;
      const id = shapeOf(c.cells[bx - c.ox + (bz - c.oz) * CHUNK_SIZE + (yy - c.oy) * 64]);
      if (id === SHAPE_NONE) continue;
      if (id === SHAPE_FULL_CUBE) {
        if (dy === 0) return true;
        continue;
      }
      const boxes = this.table.get(id);
      if (!boxes) continue;
      const lx = x - bx, ly = y - yy, lz = z - bz;
      for (const b of boxes) if (lx > b[0] && lx < b[3] && ly > b[1] && ly < b[4] && lz > b[2] && lz < b[5]) return true;
    }
    return false;
  }
  /** 静态体重建后，唤醒休眠范围与变化单元相交的请求者；等待地形的刚体保持冻结。 */
  wakeChanged(key2, c, changed) {
    for (const s of this.states.values()) {
      if (s.waiting || !s.keySet.has(key2) || !s.body.sleeping) continue;
      if (!bodyBounds(s.body, bounds)) continue;
      const m = this.margin;
      const x0 = Math.floor(bounds[0] - m), y0 = Math.floor(bounds[1] - m), z0 = Math.floor(bounds[2] - m);
      const x1 = Math.floor(bounds[3] + m), y1 = Math.floor(bounds[4] + m), z1 = Math.floor(bounds[5] + m);
      for (let n = 0; n < changed.length; n++) {
        const i = changed[n], x = c.ox + (i & 7), y = c.oy + (i >> 6), z = c.oz + (i >> 3 & 7);
        if (x < x0 || x > x1 || y < y0 || y > y1 || z < z0 || z > z1) continue;
        this.wakeWithNeighbors(s.body);
        break;
      }
    }
  }
  /** 唤醒刚体及包围盒重叠的休眠动态刚体，跳过锁定唤醒的对象。 */
  wakeWithNeighbors(body) {
    if (!body.sleeping) return;
    body.wakeUp();
    const queue = this.wakeQueue, a = this.wakeAabb;
    queue.length = 0;
    queue.push(body);
    for (let i = 0; i < queue.length; i++) {
      const b = queue[i];
      if (b.shapeList === null) continue;
      a.copyFrom(b.shapeList.aabb);
      for (let s = b.shapeList.next; s !== null; s = s.next) {
        const t = s.aabb;
        if (t.minX < a.minX) a.minX = t.minX;
        if (t.minY < a.minY) a.minY = t.minY;
        if (t.minZ < a.minZ) a.minZ = t.minZ;
        if (t.maxX > a.maxX) a.maxX = t.maxX;
        if (t.maxY > a.maxY) a.maxY = t.maxY;
        if (t.maxZ > a.maxZ) a.maxZ = t.maxZ;
      }
      this.world.aabbTest(a, this.wakeCallback);
    }
    queue.length = 0;
  }
  /**
   * 按动态刚体轮询已缓存且未失效的方块，跳过等待地形的刚体。
   * 每个刚体预算为 max(2, floor(总预算 / 刚体数))；范围外扩 1 格，启用流体时向下扩展 2 格。
   * 游标按 x、z、y 递进，发现变化时立即使对应缓存失效。
   */
  revalidate() {
    if (this.revalidateBlocksPerUpdate === 0) return;
    let n = 0;
    for (let b = this.world.rigidBodyList; b !== null; b = b.next) if (b.type === RigidBodyType.DYNAMIC && this.states.get(b)?.waiting !== true) n++;
    if (n === 0) return;
    const budget = Math.max(2, Math.floor(this.revalidateBlocksPerUpdate / n));
    const changed = changedScratch;
    for (let b = this.world.rigidBodyList; b !== null; b = b.next) {
      if (b.type !== RigidBodyType.DYNAMIC) continue;
      const s = this.states.get(b);
      if (s === void 0 || s.waiting || !bodyBounds(b, bounds)) continue;
      const x0 = Math.floor(bounds[0]) - 1, y0 = this.fluids ? Math.floor(bounds[1] - 1.5) : Math.floor(bounds[1]) - 1, z0 = Math.floor(bounds[2]) - 1;
      const sx = Math.floor(bounds[3]) + 2 - x0, sy = Math.floor(bounds[4]) + 2 - y0, sz = Math.floor(bounds[5]) + 2 - z0;
      const volume = sx * sy * sz;
      if (!(volume > 0) || !Number.isSafeInteger(volume)) continue;
      let cursor = s.cursor % volume;
      changed.length = 0;
      let c, last = -2;
      for (let k = 0; k < budget; k++) {
        const x = x0 + cursor % sx, z = z0 + Math.floor(cursor / sx) % sz, y = y0 + Math.floor(cursor / (sx * sz));
        cursor = (cursor + 1) % volume;
        const key2 = chunkKey(Math.floor(x / CHUNK_SIZE), Math.floor(y / CHUNK_SIZE), Math.floor(z / CHUNK_SIZE));
        if (key2 !== last) {
          c = this.chunks.get(key2);
          last = key2;
        }
        if (c === void 0 || c.dirty.size > 0) continue;
        const i = x - c.ox + (z - c.oz) * CHUNK_SIZE + (y - c.oy) * 64;
        if (this.source.changed(x, y, z, c.cells[i], c.types[i], c.fluids[i])) changed.push(x, y, z);
      }
      s.cursor = cursor;
      for (let k = 0; k < changed.length; k += 3) this.blocksChanged(changed[k], changed[k + 1], changed[k + 2], changed[k], changed[k + 1], changed[k + 2], true);
    }
  }
  /** 标记范围内的已缓存单元失效，并重新排队有引用的块；当前请求优先于预读请求。 */
  invalidate(key2, x0, y0, z0, x1, y1, z1) {
    const c = this.chunks.get(key2);
    if (c !== void 0) {
      const ax = Math.max(x0, c.ox), ay = Math.max(y0, c.oy), az = Math.max(z0, c.oz);
      const bx = Math.min(x1, c.ox + 7), by = Math.min(y1, c.oy + 7), bz = Math.min(z1, c.oz + 7);
      for (let y = ay; y <= by; y++) for (let z = az; z <= bz; z++) for (let x = ax; x <= bx; x++) c.dirty.add(x - c.ox + (z - c.oz) * CHUNK_SIZE + (y - c.oy) * 64);
    }
    this.normal.delete(key2);
    if (!this.refs.has(key2)) return;
    let current = false;
    for (const s of this.states.values()) if (s.keySet.has(key2)) {
      current = true;
      break;
    }
    this.enqueue(key2, current);
  }
  /** 删掉没有引用、unloadDelay 个 update 没用过的块 */
  prune() {
    for (const [key2, c] of this.chunks) {
      if (this.refs.has(key2) || this.updateCount - c.lastUsed <= this.unloadDelay) continue;
      this.detach(c);
      if (c.fluidCount > 0) this.fluidChunks--;
      this.chunks.delete(key2);
      if (this.lastChunk === c) this.lastChunk = null;
    }
  }
};
function countFluids(fluids) {
  let n = 0;
  for (let i = 0; i < CHUNK_CELLS; i++) if (fluids[i] !== 0) n++;
  return n;
}

// src/world/fluid-grid.ts
var SUB = 4;
var INSIDE_EPS = 1e-9;
var FLUID_GRID_MAX_CELLS = 4e4;
var SIZE_EPS = 1e-9;
var shapeAabb = new Aabb();
var probe = new SphereGeometry(0);
var probeTf = new Transform();
var cell = [0, 0, 0, 0, 0, 0, 0];
var FluidGrid = class {
  constructor() {
    /** 每次重建加 1 */
    this.version = 0;
    /** 格网最小角的局部坐标 */
    this.originX = 0;
    this.originY = 0;
    this.originZ = 0;
    /** 每根轴的格数 */
    this.sx = 0;
    this.sy = 0;
    this.sz = 0;
    /** 每格的编号：没占用是 0，占用的是格下标 + 1。格下标是 x + z·sx + y·sx·sz */
    this.cells = new Int32Array(0);
    /** 按编号保存占用体积、满格标志和局部包围盒；每个包围盒含 6 个分量，编号 0 保留。 */
    this.volume = new Float64Array(1);
    this.full = new Uint8Array(1);
    this.bounds = new Float64Array(6);
    /** 占用的格数和占用格的范围（格子坐标，max 不含） */
    this.count = 0;
    this.minX = 0;
    this.minY = 0;
    this.minZ = 0;
    this.maxX = 0;
    this.maxY = 0;
    this.maxZ = 0;
    /** 上一次重建时的形状和局部变换，用来发现形状变了（几何体是只读的，换几何体就是换形状） */
    this.shapes = [];
    this.poses = [];
  }
  /** 形状变了就重建。格网超过 FLUID_GRID_MAX_CELLS 格时是空的 */
  update(body) {
    if (this.version > 0 && this.same(body)) return;
    this.remember(body);
    this.version++;
    this.count = 0;
    this.minX = this.minY = this.minZ = this.maxX = this.maxY = this.maxZ = 0;
    let minX = Infinity, minY = Infinity, minZ = Infinity, maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
    for (let s = body.shapeList; s !== null; s = s.next) {
      s.geom.computeAabb(shapeAabb, s.localTransform);
      if (shapeAabb.minX < minX) minX = shapeAabb.minX;
      if (shapeAabb.minY < minY) minY = shapeAabb.minY;
      if (shapeAabb.minZ < minZ) minZ = shapeAabb.minZ;
      if (shapeAabb.maxX > maxX) maxX = shapeAabb.maxX;
      if (shapeAabb.maxY > maxY) maxY = shapeAabb.maxY;
      if (shapeAabb.maxZ > maxZ) maxZ = shapeAabb.maxZ;
    }
    const sx = cellsAlong(maxX - minX), sy = cellsAlong(maxY - minY), sz = cellsAlong(maxZ - minZ);
    if (body.shapeList === null || !(sx * sy * sz <= FLUID_GRID_MAX_CELLS)) {
      this.sx = this.sy = this.sz = 0;
      this.cells = new Int32Array(0);
      return;
    }
    this.originX = minX;
    this.originY = minY;
    this.originZ = minZ;
    this.sx = sx;
    this.sy = sy;
    this.sz = sz;
    const n = sx * sy * sz;
    this.cells = new Int32Array(n);
    this.volume = new Float64Array(n + 1);
    this.full = new Uint8Array(n + 1);
    this.bounds = new Float64Array((n + 1) * 6);
    for (let s = body.shapeList; s !== null; s = s.next) this.rasterize(s);
    let lx = Infinity, ly = Infinity, lz = Infinity, hx = -Infinity, hy = -Infinity, hz = -Infinity, count = 0;
    let idx = 0;
    for (let y = 0; y < sy; y++) {
      for (let z = 0; z < sz; z++) {
        for (let x = 0; x < sx; x++, idx++) {
          if (!(this.volume[idx + 1] > 0)) continue;
          this.cells[idx] = idx + 1;
          count++;
          if (x < lx) lx = x;
          if (y < ly) ly = y;
          if (z < lz) lz = z;
          if (x + 1 > hx) hx = x + 1;
          if (y + 1 > hy) hy = y + 1;
          if (z + 1 > hz) hz = z + 1;
        }
      }
    }
    this.count = count;
    if (count > 0) {
      this.minX = lx;
      this.minY = ly;
      this.minZ = lz;
      this.maxX = hx;
      this.maxY = hy;
      this.maxZ = hz;
    }
  }
  /** 格子 (x, y, z) 的编号，格网外是 0 */
  get(x, y, z) {
    if (x < 0 || y < 0 || z < 0 || x >= this.sx || y >= this.sy || z >= this.sz) return 0;
    return this.cells[x + this.sx * (z + this.sz * y)];
  }
  /** 格子 (x, y, z) 是否满格 */
  isFull(x, y, z) {
    const id = this.get(x, y, z);
    return id !== 0 && this.full[id] === 1;
  }
  same(body) {
    const shapes = this.shapes, poses = this.poses;
    let n = 0;
    for (let s = body.shapeList; s !== null; s = s.next, n++) {
      if (n >= shapes.length || shapes[n] !== s) return false;
      const t = s.localTransform, o = n * 12;
      if (poses[o] !== t.px || poses[o + 1] !== t.py || poses[o + 2] !== t.pz || poses[o + 3] !== t.r00 || poses[o + 4] !== t.r01 || poses[o + 5] !== t.r02 || poses[o + 6] !== t.r10 || poses[o + 7] !== t.r11 || poses[o + 8] !== t.r12 || poses[o + 9] !== t.r20 || poses[o + 10] !== t.r21 || poses[o + 11] !== t.r22) return false;
    }
    return n === shapes.length;
  }
  remember(body) {
    const shapes = this.shapes, poses = this.poses;
    shapes.length = 0;
    poses.length = 0;
    for (let s = body.shapeList; s !== null; s = s.next) {
      const t = s.localTransform;
      shapes.push(s);
      poses.push(t.px, t.py, t.pz, t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22);
    }
  }
  /** 将形状写入格网；同一格被多个形状占用时取最大占用体积。 */
  rasterize(s) {
    const t = s.localTransform, g = s.geom;
    const aligned = g instanceof BoxGeometry && t.r01 === 0 && t.r02 === 0 && t.r10 === 0 && t.r12 === 0 && t.r20 === 0 && t.r21 === 0;
    g.computeAabb(shapeAabb, t);
    const i0 = clampCell(Math.floor(shapeAabb.minX - this.originX), this.sx), i1 = clampCell(Math.ceil(shapeAabb.maxX - this.originX) - 1, this.sx);
    const j0 = clampCell(Math.floor(shapeAabb.minY - this.originY), this.sy), j1 = clampCell(Math.ceil(shapeAabb.maxY - this.originY) - 1, this.sy);
    const k0 = clampCell(Math.floor(shapeAabb.minZ - this.originZ), this.sz), k1 = clampCell(Math.ceil(shapeAabb.maxZ - this.originZ) - 1, this.sz);
    const b = this.bounds;
    for (let j = j0; j <= j1; j++) {
      for (let k = k0; k <= k1; k++) {
        for (let i = i0; i <= i1; i++) {
          const id = i + this.sx * (k + this.sz * j) + 1;
          if (this.full[id] === 1) continue;
          const cx = this.originX + i, cy = this.originY + j, cz = this.originZ + k;
          const hit = aligned ? boxCell(g, t, cx, cy, cz) : sampleCell(g, t, cx, cy, cz);
          if (!hit || cell[0] <= this.volume[id]) continue;
          const o = id * 6;
          if (cell[0] >= 1 - INSIDE_EPS) {
            this.full[id] = 1;
            this.volume[id] = 1;
            b[o] = 0;
            b[o + 1] = 0;
            b[o + 2] = 0;
            b[o + 3] = 1;
            b[o + 4] = 1;
            b[o + 5] = 1;
          } else {
            this.volume[id] = cell[0];
            b[o] = cell[1];
            b[o + 1] = cell[2];
            b[o + 2] = cell[3];
            b[o + 3] = cell[4];
            b[o + 4] = cell[5];
            b[o + 5] = cell[6];
          }
        }
      }
    }
  }
};
function cellsAlong(length) {
  const n = Math.ceil(length - SIZE_EPS);
  return n < 1 ? 1 : n;
}
function clampCell(i, n) {
  return i < 0 ? 0 : i >= n ? n - 1 : i;
}
function boxCell(g, t, cx, cy, cz) {
  const x0 = Math.max(t.px - g.hx, cx) - cx, x1 = Math.min(t.px + g.hx, cx + 1) - cx;
  const y0 = Math.max(t.py - g.hy, cy) - cy, y1 = Math.min(t.py + g.hy, cy + 1) - cy;
  const z0 = Math.max(t.pz - g.hz, cz) - cz, z1 = Math.min(t.pz + g.hz, cz + 1) - cz;
  if (!(x1 > x0 && y1 > y0 && z1 > z0)) return false;
  cell[0] = (x1 - x0) * (y1 - y0) * (z1 - z0);
  cell[1] = x0;
  cell[2] = y0;
  cell[3] = z0;
  cell[4] = x1;
  cell[5] = y1;
  cell[6] = z1;
  return true;
}
function sampleCell(g, t, cx, cy, cz) {
  let n = 0;
  let b0 = 1, b1 = 1, b2 = 1, b3 = 0, b4 = 0, b5 = 0;
  const step = 1 / SUB;
  for (let iz = 0; iz < SUB; iz++) {
    for (let iy = 0; iy < SUB; iy++) {
      for (let ix = 0; ix < SUB; ix++) {
        if (!inside(g, t, cx + (ix + 0.5) * step, cy + (iy + 0.5) * step, cz + (iz + 0.5) * step)) continue;
        n++;
        const x0 = ix * step, y0 = iy * step, z0 = iz * step;
        if (x0 < b0) b0 = x0;
        if (y0 < b1) b1 = y0;
        if (z0 < b2) b2 = z0;
        if (x0 + step > b3) b3 = x0 + step;
        if (y0 + step > b4) b4 = y0 + step;
        if (z0 + step > b5) b5 = z0 + step;
      }
    }
  }
  if (n === 0) return false;
  cell[0] = n / (SUB * SUB * SUB);
  cell[1] = b0;
  cell[2] = b1;
  cell[3] = b2;
  cell[4] = b3;
  cell[5] = b4;
  cell[6] = b5;
  return true;
}
function inside(g, t, x, y, z) {
  const dx = x - t.px, dy = y - t.py, dz = z - t.pz;
  const lx = t.r00 * dx + t.r10 * dy + t.r20 * dz;
  const ly = t.r01 * dx + t.r11 * dy + t.r21 * dz;
  const lz = t.r02 * dx + t.r12 * dy + t.r22 * dz;
  if (g instanceof BoxGeometry) {
    return Math.abs(lx) <= g.hx + INSIDE_EPS && Math.abs(ly) <= g.hy + INSIDE_EPS && Math.abs(lz) <= g.hz + INSIDE_EPS;
  }
  if (g instanceof SphereGeometry) {
    const r = g.radius + INSIDE_EPS;
    return lx * lx + ly * ly + lz * lz <= r * r;
  }
  if (g instanceof CapsuleGeometry) {
    const ey = ly < -g.halfHeight ? ly + g.halfHeight : ly > g.halfHeight ? ly - g.halfHeight : 0;
    const r = g.radius + INSIDE_EPS;
    return lx * lx + ey * ey + lz * lz <= r * r;
  }
  if (g instanceof CylinderGeometry) {
    if (Math.abs(ly) > g.halfHeight + INSIDE_EPS) return false;
    const r = g.radius + INSIDE_EPS;
    return lx * lx + lz * lz <= r * r;
  }
  if (g instanceof ConeGeometry) {
    if (Math.abs(ly) > g.halfHeight + INSIDE_EPS) return false;
    const r = g.radius * (g.halfHeight - ly) / (2 * g.halfHeight) + INSIDE_EPS;
    return lx * lx + lz * lz <= r * r;
  }
  if (g instanceof ConvexHullGeometry) {
    probeTf.px = x;
    probeTf.py = y;
    probeTf.pz = z;
    const gjk = GjkEpa.getInstance();
    gjk.computeDistance(probe, g, probeTf, t, null);
    return gjk.distance <= g.gjkMargin + INSIDE_EPS;
  }
  return false;
}

// src/world/fluid-forces.ts
var POCKET_ROTATION = 0.99985;
var POCKET_MIN_AGE = 10;
var FLOOD_FACTOR = 0.585;
var FluidForces = class {
  constructor(options = {}) {
    this.waterDensity = 1;
    this.lavaDensity = 3;
    this.drag = 0.5;
    this.linearDrag = 0.5;
    this.lavaViscosity = 6;
    this.waveDamping = 0.5;
    this.airPockets = true;
    this.floodRate = 1;
    this.currentSpeed = 1.4;
    /** 上一次 apply 的结果：浸没体积、气腔体积、腔里的水、浮力的不平衡程度 */
    this.lastSubmerged = 0;
    this.lastPocketVolume = 0;
    this.lastInside = 0;
    this.lastUnbalance = 0;
    /** 上次 apply 检测到的流体种类，第 k 位表示种类 k + 1。 */
    this.lastKinds = 0;
    this.pockets = /* @__PURE__ */ new WeakMap();
    /** 上一次 submerged、surfaceAt 查到的流体种类、液面高度、浸没部分的中心 */
    this.kind = 0;
    this.middle = 0;
    this.surface = 0;
    /** 上一次 flowAt 的水流速度 */
    this.flowX = 0;
    this.flowZ = 0;
    this.surfaces = /* @__PURE__ */ new Map();
    this.surfaceKinds = /* @__PURE__ */ new Map();
    this.flows = /* @__PURE__ */ new Map();
    this.offX = new Float64Array(0);
    this.offY = new Float64Array(0);
    this.offZ = new Float64Array(0);
    this.bottoms = new Float64Array(0);
    this.caps = new Float64Array(0);
    this.rooms = new Float64Array(0);
    this.outside = new Float64Array(0);
    this.kinds = new Uint8Array(0);
    if (options.waterDensity !== void 0) this.waterDensity = Math.max(0, options.waterDensity);
    if (options.lavaDensity !== void 0) this.lavaDensity = Math.max(0, options.lavaDensity);
    if (options.drag !== void 0) this.drag = Math.max(0, options.drag);
    if (options.linearDrag !== void 0) this.linearDrag = Math.max(0, options.linearDrag);
    if (options.lavaViscosity !== void 0) this.lavaViscosity = Math.max(0, options.lavaViscosity);
    if (options.waveDamping !== void 0) this.waveDamping = Math.max(0, options.waveDamping);
    if (options.airPockets !== void 0) this.airPockets = options.airPockets;
    if (options.floodRate !== void 0) this.floodRate = Math.max(0, options.floodRate);
    if (options.currentSpeed !== void 0) this.currentSpeed = Math.max(0, options.currentSpeed);
  }
  /**
   * 采集刚体在指定流体中的接触信息，供入水事件使用。
   * 依赖同一 tick 的 apply 缓存；无对应接触时返回 false。
   */
  collectEntry(body, terrain, kind, out) {
    const grid = body.grid, cells = grid.cells, bounds2 = grid.bounds, full2 = grid.full;
    const sx = grid.sx, sz = grid.sz, layer = sx * sz;
    const W = 0.5 * (Math.abs(body.r10) + Math.abs(body.r11) + Math.abs(body.r12));
    const ax = Math.abs(body.r10), ay = Math.abs(body.r11), az = Math.abs(body.r12);
    let count = 0, vy = Infinity, sumX = 0, sumY = 0, sumZ = 0;
    let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
    for (let i = 0; i < cells.length; i++) {
      const id = cells[i];
      if (id === 0) continue;
      const o = id * 6;
      const lx = i % sx + grid.originX + (bounds2[o] + bounds2[o + 3]) * 0.5;
      const ly = Math.floor(i / layer) + grid.originY + (bounds2[o + 1] + bounds2[o + 4]) * 0.5;
      const lz = Math.floor(i / sx) % sz + grid.originZ + (bounds2[o + 2] + bounds2[o + 5]) * 0.5;
      const rx = body.r00 * lx + body.r01 * ly + body.r02 * lz;
      const ry = body.r10 * lx + body.r11 * ly + body.r12 * lz;
      const rz = body.r20 * lx + body.r21 * ly + body.r22 * lz;
      const half = full2[id] === 1 ? W : 0.5 * (ax * (bounds2[o + 3] - bounds2[o]) + ay * (bounds2[o + 4] - bounds2[o + 1]) + az * (bounds2[o + 5] - bounds2[o + 2]));
      if (this.submerged(terrain, body.px + rx, body.py + ry, body.pz + rz, half > 0.01 ? half : 0.01) <= 0 || this.kind !== kind) continue;
      const v = body.vy + body.wz * rx - body.wx * rz;
      if (v < vy) vy = v;
      const x = body.px + rx, z = body.pz + rz;
      sumX += x;
      sumZ += z;
      sumY += this.surface === Infinity ? body.py + ry + half : this.surface;
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
      count++;
    }
    if (count === 0) return false;
    out.fastestVelocityY = vy;
    out.minX = minX;
    out.maxX = maxX;
    out.minZ = minZ;
    out.maxZ = maxZ;
    out.pointX = sumX / count;
    out.pointY = sumY / count + 0.02;
    out.pointZ = sumZ / count;
    return true;
  }
  /** 上次 apply 建立的格网是否包含气腔；气腔的液面查询向上扩展 64 格。 */
  hasPockets(body) {
    const f = this.pockets.get(body);
    return f !== void 0 && f.count > 0;
  }
  /** 气腔里的水（格³） */
  fluidInside(body) {
    const f = this.pockets.get(body);
    if (f === void 0) return 0;
    let x = 0;
    for (let l = 0; l < f.rooms; l++) x += f.water[l];
    return x;
  }
  /** 一段竖直线 [x − half, x + half] 浸在 (f, l) 这一格柱的流体里的比例，浸没部分的中心记在 middle */
  submerged(terrain, f, x, l, half) {
    const h = Math.floor(f), M = Math.floor(l), n = x - half, c = x + half;
    let a = 0, e = 0;
    this.kind = 0;
    for (let y = Math.floor(n); y < c; y++) {
      const i = terrain.fluidAt(h, y, M);
      if (i === 0) continue;
      const r = fluidKind(i), w = y + columnHeight(terrain, h, y, M, i), d = n > y ? n : y, g = c < w ? c : w;
      if (g <= d) continue;
      a += g - d;
      e += (g - d) * (d + g) * 0.5;
      if (r > this.kind) this.kind = r;
    }
    this.middle = a > 0 ? e / a : x;
    const F = a / (2 * half);
    this.surface = F >= 1 - 1e-9 ? 1 / 0 : n + a;
    return F > 1 ? 1 : F;
  }
  /** (f, l) 这一格柱的液面高度（世界坐标）。同一 tick 里查过的记下来 */
  surfaceAt(terrain, f, x, l) {
    const o = Math.floor(f), h = Math.floor(x), M = Math.floor(l), n = columnKey(o, h, M), c = this.surfaces.get(n);
    if (c !== void 0) {
      this.kind = this.surfaceKinds.get(n) ?? 0;
      return c;
    }
    let a = h, e = terrain.fluidAt(o, a, M);
    if (e === 0) {
      a--;
      e = terrain.fluidAt(o, a, M);
    }
    let F = -1 / 0;
    const y = e === 0 ? 0 : fluidKind(e);
    for (let i = 0; i < 64 && e !== 0; i++) {
      const r = terrain.fluidAt(o, a + 1, M);
      F = a + fluidHeight(e);
      if (r === 0 || fluidKind(r) !== y) break;
      a++;
      e = r;
      F = a + 1;
    }
    this.surfaces.set(n, F);
    this.surfaceKinds.set(n, y);
    this.kind = y;
    return F;
  }
  /** (f, x, l) 这一格的水流速度，结果在 flowX、flowZ。同一 tick 里查过的记下来 */
  flowAt(terrain, f, x, l) {
    this.flowX = 0;
    this.flowZ = 0;
    const o = this.currentSpeed;
    if (o <= 0) return;
    const h = columnKey(f, x, l), M = this.flows.get(h);
    if (M !== void 0) {
      this.flowX = (Math.floor(M / 16384) - 4096) / 4096 * o;
      this.flowZ = (M % 16384 - 4096) / 4096 * o;
      return;
    }
    const n = terrain.fluidAt(f, x, l);
    let c = 0, a = 0;
    if (n !== 0) {
      const F = fluidKind(n), y = columnHeight(terrain, f, x, l, n);
      for (let i = 0; i < 4; i++) {
        const r = i === 0 ? 1 : i === 1 ? -1 : 0, w = i === 2 ? 1 : i === 3 ? -1 : 0, d = terrain.fluidAt(f + r, x, l + w);
        let g = 0;
        if (d !== 0 && fluidKind(d) === F) g = y - columnHeight(terrain, f + r, x, l + w, d);
        else if (d === 0 && terrain.isTerrainEmpty(f + r, x, l + w)) {
          const Z = terrain.fluidAt(f + r, x - 1, l + w);
          if (Z !== 0 && fluidKind(Z) === F) g = y - (columnHeight(terrain, f + r, x - 1, l + w, Z) - 1);
        }
        c += r * g;
        a += w * g;
      }
    }
    const e = Math.sqrt(c * c + a * a);
    if (e > 1e-6) {
      c /= e;
      a /= e;
    } else c = a = 0;
    this.flows.set(h, (Math.round(c * 4096) + 4096) * 16384 + Math.round(a * 4096) + 4096);
    this.flowX = c * o;
    this.flowZ = a * o;
  }
  /**
   * 算一个刚体这个 tick 受的流体力，加到 fx..tz 上。附近没有流体、气腔里也没有水时返回 false。
   * dt 是一个 tick 的时长；gx、gy、gz 是世界的重力。
   */
  apply(body, terrain, gx, gy, gz, dt) {
    this.lastSubmerged = 0;
    this.lastPocketVolume = 0;
    this.lastInside = 0;
    this.lastUnbalance = 0;
    this.lastKinds = 0;
    body.submerged = 0;
    const c = terrain.hasFluidNear(Math.floor(body.minX), Math.floor(body.minY) - 1, Math.floor(body.minZ), Math.floor(body.maxX), Math.floor(body.maxY), Math.floor(body.maxZ));
    if (!c && this.fluidInside(body) <= 0) return false;
    this.surfaces.clear();
    this.surfaceKinds.clear();
    this.flows.clear();
    const grid = body.grid, F = body.px, y = body.py, i = body.pz, r = -grid.originX, w = -grid.originY, d = -grid.originZ;
    const g = body.vx, Z = body.vy, Y = body.vz, T = body.wx, rt = body.wy, lt = body.wz;
    const at = Math.abs(body.r10), k = Math.abs(body.r11), _ = Math.abs(body.r12), W = 0.5 * (at + k + _), P = 2 * W;
    const j = -gx * body.gravityScale, nt = -gy * body.gravityScale, B = -gz * body.gravityScale, J = Math.sqrt(Math.sqrt(j * j + nt * nt + B * B));
    let ot = 0, et = 0, K = 0, Q = 0, dtq = 0, mt = 0, $ = 0, O = 0, St = 0, kt = 0, Xt = 0, At = 0, zt = 0, vt = 0, Zt = 0, Ft = 0, Yt = 0;
    const Ct = this.waterDensity, Lt = this.lavaDensity, Nt = this.waveDamping * J, yt = grid.volume, ns = grid.bounds;
    const buoyancy = (v, R, G, X) => {
      const C = j * v, H = nt * v, it = B * v;
      ot += C;
      et += H;
      K += it;
      Q += G * it - X * H;
      dtq += X * C - R * it;
      mt += R * H - G * C;
    };
    const wave = (v, R, X) => {
      const C = Z + lt * R - T * X, H = -v * C;
      O += H;
      kt -= X * H;
      At += R * H;
      zt += v;
      vt += v * (R * R + X * X);
    };
    const U = this.airPockets && c ? this.pocketsOf(body, W) : this.pockets.get(body);
    if (U !== void 0) {
      const v = U.count;
      if (this.offX.length < v) {
        const s = Math.max(64, v * 2);
        this.offX = new Float64Array(s);
        this.offY = new Float64Array(s);
        this.offZ = new Float64Array(s);
        this.bottoms = new Float64Array(s);
        this.caps = new Float64Array(s);
        this.rooms = new Float64Array(s);
        this.outside = new Float64Array(s);
        this.kinds = new Uint8Array(s);
      }
      const R = U.cells, G = U.dips, X = U.fill, C = this.offX, H = this.offY, it = this.offZ;
      const D = this.bottoms, ft = this.caps, b = this.rooms, ht = this.outside, L = this.kinds;
      for (let s = 0; s < v; s++) {
        const S = R[s * 3], V = R[s * 3 + 1], A = R[s * 3 + 2];
        const m = S + 0.5 - r, u = V + 0.5 - w, q = A + 0.5 - d;
        const tt = body.r00 * m + body.r01 * u + body.r02 * q;
        const z = body.r10 * m + body.r11 * u + body.r12 * q;
        const p = body.r20 * m + body.r21 * u + body.r22 * q;
        C[s] = tt;
        H[s] = z;
        it[s] = p;
        const ct = y + z - W;
        D[s] = ct;
        const st = (G[s] + y - ct) / P;
        ft[s] = st <= 0 ? 0 : st >= 1 ? 1 : st;
        const gt = grid.get(S, V, A);
        b[s] = gt === 0 ? 1 : 1 - yt[gt];
        ht[s] = c ? this.surfaceAt(terrain, F + tt, y + z, i + p) : -1 / 0;
        L[s] = this.kind;
      }
      const below = (s, S, V) => {
        let A = 0;
        for (let m = s; m < S; m++) {
          const u = (V - D[m]) / P;
          if (u > 0) A += (u < ft[m] ? u : ft[m]) * b[m];
        }
        return A;
      };
      const level = (s, S, V) => {
        let A = 1 / 0, m = -1 / 0;
        for (let u = s; u < S; u++) {
          if (D[u] < A) A = D[u];
          if (D[u] + P > m) m = D[u] + P;
        }
        if (below(s, S, m) <= V) return m;
        for (let u = 0; u < 30; u++) {
          const q = (A + m) * 0.5;
          if (below(s, S, q) < V) A = q;
          else m = q;
        }
        return (A + m) * 0.5;
      };
      for (let s = 0; s < U.rooms; s++) {
        const S = U.roomStart[s], V = U.roomStart[s + 1];
        let A = -1 / 0;
        for (let z = S; z < V; z++) if (ht[z] > A) A = ht[z];
        const m = U.roomRim[s] + y;
        let u = U.water[s], q = u > 0 ? level(S, V, u) : -1 / 0;
        const tt = this.floodRate * FLOOD_FACTOR * J * U.roomEdge[s] * dt;
        if (A > m && q < A) {
          const z = A - (q > m ? q : m);
          u += tt * z * Math.sqrt(z);
          const p = below(S, V, A);
          if (u > p) u = p;
        } else if (q > m && q > A) {
          const z = A > m ? A : m, p = q - z;
          u -= tt * p * Math.sqrt(p);
          const ct = below(S, V, z);
          if (u < ct) u = ct;
        }
        if (u < 1e-6) u = 0;
        U.water[s] = u;
        q = u > 0 ? level(S, V, u) : -1 / 0;
        for (let z = S; z < V; z++) {
          const p = (q - D[z]) / P;
          X[z] = p <= 0 ? 0 : p < ft[z] ? p : ft[z];
        }
      }
      for (let s = 0; s < v; s++) {
        const S = C[s], A = it[s], m = D[s], u = m + P, q = ht[s], tt = L[s] === FLUID_LAVA ? Lt : Ct;
        const z = q > m ? (q < u ? q : u) - m : 0, p = X[s] * P;
        if (z > 0) buoyancy(tt * b[s] * z / P, S, m + z * 0.5 - y, A);
        if (p > 0) buoyancy(-tt * b[s] * p / P, S, m + p * 0.5 - y, A);
        if (z > p) Ft += b[s] * (z - p) / P;
        Yt += b[s] * p / P;
        if (q > m && q < u && p < z) wave(tt * Nt * b[s] / P, S, A);
      }
    }
    const is = this.drag, fs = this.linearDrag, cs = this.lavaViscosity;
    const rs = grid.sx, ls = grid.sy, as = grid.sz, ms = grid.cells;
    let Wt = 0;
    for (let v = 0; v < ls; v++) {
      for (let R = 0; R < as; R++) {
        for (let G = 0; G < rs; G++, Wt++) {
          const X = ms[Wt];
          if (X === 0 || !c) continue;
          const C = G, H = v, it = R, D = ns, o6 = X * 6;
          const ft = C + (D[o6] + D[o6 + 3]) * 0.5 - r, b = H + (D[o6 + 1] + D[o6 + 4]) * 0.5 - w, ht = it + (D[o6 + 2] + D[o6 + 5]) * 0.5 - d;
          const L = body.r00 * ft + body.r01 * b + body.r02 * ht;
          const E = body.r10 * ft + body.r11 * b + body.r12 * ht;
          const N = body.r20 * ft + body.r21 * b + body.r22 * ht;
          const s = yt[X] === 1 ? W : 0.5 * (at * (D[o6 + 3] - D[o6]) + k * (D[o6 + 4] - D[o6 + 1]) + _ * (D[o6 + 5] - D[o6 + 2]));
          const S = this.submerged(terrain, F + L, y + E, i + N, s > 0.01 ? s : 0.01);
          if (S <= 0) continue;
          const V = this.kind, A = this.surface, m = V === FLUID_LAVA ? Lt : Ct;
          this.lastKinds |= 1 << V - 1;
          Zt += S * yt[X];
          buoyancy(m * yt[X] * S, L, this.middle - y, N);
          if (A !== 1 / 0) wave(m * Nt * yt[X] / (2 * s), L, N);
          this.flowAt(terrain, Math.floor(F + L), Math.floor(y + E), Math.floor(i + N));
          const u = g + rt * N - lt * E - this.flowX, q = Z + lt * L - T * N, tt = Y + T * E - rt * L - this.flowZ;
          const z = m * (V === FLUID_LAVA ? cs : 1);
          for (let p = 0; p < 3; p++) {
            const ct = p === 0 ? body.r00 : p === 1 ? body.r01 : body.r02;
            const st = p === 0 ? body.r10 : p === 1 ? body.r11 : body.r12;
            const gt = p === 0 ? body.r20 : p === 1 ? body.r21 : body.r22;
            const xt = u * ct + q * st + tt * gt;
            if (xt > -1e-6 && xt < 1e-6) continue;
            const It = D[o6 + p + 3] - D[o6 + p], Vt = s - (st > 0 ? st : -st) * It * 0.5;
            let Mt = 0;
            for (let pt = -1; pt <= 1; pt += 2) {
              const Qt = C + (p === 0 ? pt : 0), $t = H + (p === 1 ? pt : 0), bt = it + (p === 2 ? pt : 0);
              if (grid.isFull(Qt, $t, bt) || U !== void 0 && U.index[Qt - U.x0 + U.nx * (bt - U.z0 + U.nz * ($t - U.y0))] >= 0) continue;
              if (A === 1 / 0) {
                Mt++;
                continue;
              }
              const ts = y + E + pt * st * It * 0.5;
              if (Vt < 1e-3) {
                if (A > ts) Mt++;
              } else {
                const qt = (A - ts + Vt) / (2 * Vt);
                Mt += qt <= 0 ? 0 : qt >= 1 ? 1 : qt;
              }
            }
            if (Mt <= 0) continue;
            const ys = It >= yt[X] ? yt[X] / It : 1, Kt = z * ys * Mt * 0.5 * (fs + is * (xt > 0 ? xt : -xt)), Rt = -Kt * xt;
            const Et = ct * Rt, Ot = st * Rt, Tt = gt * Rt;
            $ += Et;
            O += Ot;
            St += Tt;
            kt += E * Tt - N * Ot;
            Xt += N * Et - L * Tt;
            At += L * Ot - E * Et;
            zt += Kt;
            const Jt = L * ct + E * st + N * gt;
            vt += Kt * (L * L + E * E + N * N - Jt * Jt);
          }
        }
      }
    }
    this.lastPocketVolume = Ft;
    this.lastSubmerged = Zt + Ft;
    this.lastInside = Yt;
    body.submerged = this.lastSubmerged;
    if (Zt <= 0 && Ft <= 0 && Yt <= 0) return false;
    const Gt = body.mass * (j * j + nt * nt + B * B) ** 0.5;
    if (Gt > 0) {
      const v = ot - j * body.mass, R = et - nt * body.mass, G = K - B * body.mass;
      const X = Math.sqrt(v * v + R * R + G * G), C = Math.sqrt(Q * Q + dtq * dtq + mt * mt);
      this.lastUnbalance = (X > C ? X : C) / Gt;
    }
    const jt = body.invMass;
    if (zt * dt * jt > 0.9) {
      const v = 0.9 / (zt * dt * jt);
      $ *= v;
      O *= v;
      St *= v;
    }
    const Bt = body.ii00 + body.ii11 + body.ii22;
    if (vt * dt * Bt > 0.9) {
      const v = 0.9 / (vt * dt * Bt);
      kt *= v;
      Xt *= v;
      At *= v;
    }
    body.fx += ot + $;
    body.fy += et + O;
    body.fz += K + St;
    body.tx += Q + kt;
    body.ty += dtq + Xt;
    body.tz += mt + At;
    return true;
  }
  /** 刚体的气腔。形状没变、转动不超过约 2° 或距上次重建不到 10 tick 时用上次的结果 */
  pocketsOf(body, half) {
    const grid = body.grid, o = this.pockets.get(body);
    if (o !== void 0 && o.version === grid.version && (o.age++, Math.abs(o.qx * body.qx + o.qy * body.qy + o.qz * body.qz + o.qw * body.qw) >= POCKET_ROTATION || o.age < POCKET_MIN_AGE)) return o.count > 0 ? o : void 0;
    const h = grid.maxX - grid.minX + 2, M = grid.maxY - grid.minY + 2, n = grid.maxZ - grid.minZ + 2;
    if (grid.count === 0 || h * M * n > FLUID_GRID_MAX_CELLS) {
      this.pockets.delete(body);
      return void 0;
    }
    const c = buildPockets(body, half);
    if (o !== void 0) {
      for (let a = 0; a < c.rooms; a++) {
        for (let e = c.roomStart[a]; e < c.roomStart[a + 1]; e++) {
          const F = c.cells[e * 3], y = c.cells[e * 3 + 1], i = c.cells[e * 3 + 2];
          const r = F - o.x0, w = y - o.y0, d = i - o.z0;
          if (r < 0 || w < 0 || d < 0 || r >= o.nx || w >= o.ny || d >= o.nz) continue;
          const g = o.index[r + o.nx * (d + o.nz * w)];
          if (g < 0 || o.fill[g] <= 0) continue;
          const Z = grid.get(F, y, i);
          c.water[a] += o.fill[g] * (Z === 0 ? 1 : 1 - grid.volume[Z]);
          c.fill[e] = o.fill[g];
        }
      }
    }
    this.pockets.set(body, c);
    return c.count > 0 ? c : void 0;
  }
};
function columnKey(x, y, z) {
  return ((x & 1023) * 1024 + (z & 1023)) * 1024 + (y & 1023);
}
function columnHeight(terrain, x, y, z, code) {
  const o = terrain.fluidAt(x, y + 1, z);
  return o !== 0 && fluidKind(o) === fluidKind(code) ? 1 : fluidHeight(code);
}
function buildPockets(body, half) {
  const grid = body.grid;
  const x = grid.minX - 1, l = grid.minY - 1, o = grid.minZ - 1;
  const h = grid.maxX - grid.minX + 2, M = grid.maxY - grid.minY + 2, n = grid.maxZ - grid.minZ + 2, c = h * M * n;
  const i = new Float64Array(c), r = new Float64Array(c), w = new Uint8Array(c);
  let d = 0;
  for (let k = 0; k < M; k++) {
    for (let _ = 0; _ < n; _++) {
      for (let W = 0; W < h; W++, d++) {
        const P = W + x, ot = k + l, et = _ + o;
        const K = body.r10 * (P + 0.5 + grid.originX) + body.r11 * (ot + 0.5 + grid.originY) + body.r12 * (et + 0.5 + grid.originZ);
        i[d] = K - half;
        r[d] = -(K + half);
        w[d] = grid.isFull(P, ot, et) ? 0 : 1;
      }
    }
  }
  const g = flood(h, M, n, w, i), Z = flood(h, M, n, w, r), Y = new Uint8Array(c);
  let T = 0;
  for (let k = 0; k < c; k++) {
    if (w[k] === 1 && (g[k] > i[k] + 1e-9 || Z[k] > r[k] + 1e-9)) {
      Y[k] = 1;
      T++;
    }
  }
  const rt = new Int32Array(c);
  rt.fill(-1);
  const lt = new Int32Array(T), wt = [0], j = [], nt = [], B = h * n;
  let J = 0;
  for (let k = 0; k < c; k++) {
    if (Y[k] !== 1 || rt[k] >= 0) continue;
    const _ = J;
    let W = 1 / 0, P = 1 / 0, ot = -1 / 0;
    rt[k] = J;
    lt[J++] = k;
    for (let et = _; et < J; et++) {
      const K = lt[et];
      if (g[K] < W) W = g[K];
      const Q = Math.floor(K / B);
      if (Q < P) P = Q;
      if (Q > ot) ot = Q;
      const dtq = K % h, mt = Math.floor(K / h) % n;
      for (let $ = 0; $ < 6; $++) {
        let O;
        if ($ === 0) {
          if (dtq === 0) continue;
          O = K - 1;
        } else if ($ === 1) {
          if (dtq === h - 1) continue;
          O = K + 1;
        } else if ($ === 2) {
          if (mt === 0) continue;
          O = K - h;
        } else if ($ === 3) {
          if (mt === n - 1) continue;
          O = K + h;
        } else if ($ === 4) {
          if (Q === 0) continue;
          O = K - B;
        } else {
          if (Q === M - 1) continue;
          O = K + B;
        }
        if (Y[O] !== 1 || rt[O] >= 0) continue;
        rt[O] = J;
        lt[J++] = O;
      }
    }
    wt.push(J);
    j.push(W);
    nt.push(4 * Math.sqrt((J - _) / (ot - P + 1)));
  }
  const at = {
    version: grid.version,
    qx: body.qx,
    qy: body.qy,
    qz: body.qz,
    qw: body.qw,
    age: 0,
    x0: x,
    y0: l,
    z0: o,
    nx: h,
    ny: M,
    nz: n,
    index: rt,
    count: T,
    cells: new Int32Array(T * 3),
    dips: new Float64Array(T),
    fill: new Float64Array(T),
    rooms: j.length,
    roomStart: Int32Array.from(wt),
    roomRim: Float64Array.from(j),
    roomEdge: Float64Array.from(nt),
    water: new Float64Array(j.length)
  };
  for (let k = 0; k < T; k++) {
    const _ = lt[k];
    at.cells[k * 3] = _ % h + x;
    at.cells[k * 3 + 1] = Math.floor(_ / B) + l;
    at.cells[k * 3 + 2] = Math.floor(_ / h) % n + o;
    at.dips[k] = -Z[_];
  }
  return at;
}
function flood(sx, sy, sz, open, height) {
  const o = sx * sy * sz, h = new Float64Array(o);
  h.fill(1 / 0);
  const M = new Uint8Array(o);
  let n = new Int32Array(Math.max(64, o)), c = 0;
  const push = (i) => {
    if (c === n.length) {
      const d = new Int32Array(n.length * 2);
      d.set(n);
      n = d;
    }
    let r = c++;
    const w = h[i];
    while (r > 0) {
      const d = r - 1 >> 1;
      if (h[n[d]] <= w) break;
      n[r] = n[d];
      r = d;
    }
    n[r] = i;
  };
  const pop = () => {
    const i = n[0], r = n[--c], w = h[r];
    let d = 0;
    for (; ; ) {
      let g = d * 2 + 1;
      if (g >= c || (g + 1 < c && h[n[g + 1]] < h[n[g]] && g++, h[n[g]] >= w)) break;
      n[d] = n[g];
      d = g;
    }
    n[d] = r;
    return i;
  };
  let F = 0;
  for (let i = 0; i < sy; i++) {
    for (let r = 0; r < sz; r++) {
      for (let w = 0; w < sx; w++, F++) {
        if (w !== 0 && i !== 0 && r !== 0 && w !== sx - 1 && i !== sy - 1 && r !== sz - 1) continue;
        h[F] = height[F];
        push(F);
      }
    }
  }
  const y = sx * sz;
  while (c > 0) {
    const i = pop();
    if (M[i] === 1) continue;
    M[i] = 1;
    const r = h[i], w = i % sx, d = Math.floor(i / sx) % sz, g = Math.floor(i / y);
    for (let Z = 0; Z < 6; Z++) {
      let Y;
      if (Z === 0) {
        if (w === 0) continue;
        Y = i - 1;
      } else if (Z === 1) {
        if (w === sx - 1) continue;
        Y = i + 1;
      } else if (Z === 2) {
        if (d === 0) continue;
        Y = i - sx;
      } else if (Z === 3) {
        if (d === sz - 1) continue;
        Y = i + sx;
      } else if (Z === 4) {
        if (g === 0) continue;
        Y = i - y;
      } else {
        if (g === sy - 1) continue;
        Y = i + y;
      }
      if (M[Y] === 1 || open[Y] === 0) continue;
      const T = height[Y] > r ? height[Y] : r;
      if (T < h[Y]) {
        h[Y] = T;
        push(Y);
      }
    }
  }
  return h;
}

// src/mc/block-source.ts
var FULL_CUBE_BOX = [0, 0, 0, 1, 1, 1].join();
var NO_STATES2 = () => ({});
var FLUID_TYPES = /* @__PURE__ */ new Map([
  ["minecraft:water", FLUID_WATER],
  ["minecraft:flowing_water", FLUID_WATER],
  ["minecraft:bubble_column", FLUID_WATER],
  ["minecraft:lava", FLUID_LAVA],
  ["minecraft:flowing_lava", FLUID_LAVA]
]);
var BlockResolver = class {
  constructor(table, options = {}) {
    this.table = table;
    this.stateless = /* @__PURE__ */ new Map();
    this.custom = /* @__PURE__ */ new Map();
    this.customShapes = /* @__PURE__ */ new Map();
    this.nextShapeId = SHAPE_COUNT;
    /** 类型 → 令牌；令牌 → 类型、方块编号。0 号是空气 */
    this.tokens = /* @__PURE__ */ new Map([["minecraft:air", TYPE_AIR]]);
    this.tokenTypes = ["minecraft:air"];
    this.tokenIds = [SHAPE_NONE];
    this.registry = options.registry;
    this.collision = options.collision;
    this.unknownBlock = options.unknownBlock ?? "fullCube";
    this.fixedMaterial = options.fixedMaterial;
  }
  /** 有没有用户提供的 collision（有的话同一种方块的形状可能随状态变，不能按类型批量判断） */
  get hasCustomCollision() {
    return this.collision !== void 0;
  }
  /** 这个类型有没有登记碰撞盒（判断整块是否为空时，登记了碰撞盒的光源方块不能排除） */
  hasRegisteredCollision(typeId) {
    const entry = this.registry?.get(typeId);
    return entry !== void 0 && (entry.hasCollision || entry.terrainStateful);
  }
  /** 清除解析缓存，保留已登记形状和类型令牌。 */
  clearCache() {
    this.stateless.clear();
    this.custom.clear();
  }
  /**
   * 类型令牌：形状和材质只由类型决定的方块按类型编号（从 1 开始），其余为 TYPE_RESOLVE。空气是 TYPE_AIR。
   * 登记表里登记了碰撞盒、碰撞和材质不随状态变的方块，不受 collision 函数影响，也按类型编号
   */
  token(typeId) {
    const known = this.tokens.get(typeId);
    if (known !== void 0) return known;
    let t = TYPE_RESOLVE;
    const entry = this.registry?.get(typeId);
    let byType;
    if (entry !== void 0 && entry.terrainStateful) byType = false;
    else if (entry !== void 0 && entry.hasCollision) byType = true;
    else byType = this.collision === void 0 && isStateIndependent(typeId);
    if (byType) {
      t = this.tokenTypes.length;
      this.tokenTypes.push(typeId);
      this.tokenIds.push(this.resolve(typeId, NO_STATES2));
    }
    this.tokens.set(typeId, t);
    return t;
  }
  /** 令牌对应的类型和方块编号（令牌必须是 token 给出的、不小于 0 的值） */
  typeOf(token) {
    return this.tokenTypes[token];
  }
  idOf(token) {
    return this.tokenIds[token];
  }
  /** 返回方块编号。typeId 包含命名空间，仅在解析需要状态时调用 getStates。 */
  resolve(typeId, getStates2, context) {
    const entry = this.registry?.get(typeId);
    if (entry === void 0) return this.resolveUnregistered(typeId, getStates2, context);
    const m = entry.match(entry.terrainStateful ? getStates2() : NO_STATES2());
    if (m.boxes !== void 0) {
      const shape2 = this.shapeFromBoxes(m.boxes);
      return packBlock(shape2, this.fixedMaterial ?? (shape2 === SHAPE_NONE ? MATERIAL_DEFAULT : m.material));
    }
    const id = this.resolveUnregistered(typeId, getStates2, context);
    const shape = shapeOf(id);
    return packBlock(shape, this.fixedMaterial ?? (shape === SHAPE_NONE ? MATERIAL_DEFAULT : m.material));
  }
  resolveUnregistered(typeId, getStates2, context) {
    if (this.collision === void 0) {
      const cached2 = this.stateless.get(typeId);
      if (cached2 !== void 0) return cached2;
      if (isStateIndependent(typeId)) {
        const id2 = this.pack(resolveShapeId(typeId, {}), typeId);
        this.stateless.set(typeId, id2);
        return id2;
      }
      return this.builtin(typeId, getStates2(), context);
    }
    const states = getStates2();
    const key2 = `${typeId}|${JSON.stringify(states)}`;
    const cached = this.custom.get(key2);
    if (cached !== void 0) return cached === null ? this.builtin(typeId, states, context) : cached;
    const boxes = this.collision({ typeId, states });
    const id = boxes === void 0 ? null : this.pack(this.shapeFromBoxes(boxes.map((b) => [b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z])), typeId);
    this.custom.set(key2, id);
    return id === null ? this.builtin(typeId, states, context) : id;
  }
  builtin(typeId, states, context) {
    if (isStateShaped(typeId)) {
      const boxes = stateBoxes(typeId, states);
      if (boxes !== void 0) return this.pack(this.shapeFromBoxes(boxes), typeId);
    }
    if (context && needsBlockContext(typeId)) {
      const c = context(), boxes = contextualBoxes({ typeId, states, pistonExtended: c.pistonExtended }, c.read);
      if (boxes !== void 0) return this.pack(this.shapeFromBoxes(boxes), typeId);
    }
    return this.pack(resolveShapeId(typeId, states), typeId);
  }
  pack(shape, typeId) {
    if (shape === SHAPE_UNKNOWN) shape = this.unknownBlock === "fullCube" ? SHAPE_FULL_CUBE : SHAPE_NONE;
    const material = this.fixedMaterial ?? (shape === SHAPE_NONE ? MATERIAL_DEFAULT : resolveMaterialId(typeId));
    return packBlock(shape, material);
  }
  /** 用户给出的盒子 → 形状编号。相同的盒子共用一个编号 */
  shapeFromBoxes(boxes) {
    if (boxes.length === 0) return SHAPE_NONE;
    const key2 = boxes.map((b) => b.join()).join(";");
    if (key2 === FULL_CUBE_BOX) return SHAPE_FULL_CUBE;
    const known = this.customShapes.get(key2);
    if (known !== void 0) return known;
    const id = this.nextShapeId++;
    this.table.register(id, boxes);
    this.customShapes.set(key2, id);
    return id;
  }
};
function fluidCode(block, id) {
  const typeId = block.typeId;
  const kind = FLUID_TYPES.get(typeId);
  if (kind !== void 0) {
    const depth = typeId === "minecraft:bubble_column" ? 0 : block.permutation.getState("liquid_depth");
    return kind * 32 + fluidLevel(typeof depth === "number" ? depth : 0);
  }
  if (shapeOf(id) === SHAPE_FULL_CUBE || typeId === "minecraft:air") return FLUID_NONE;
  return block.isWaterlogged ? FLUID_WATER * 32 + fluidLevel(0) : FLUID_NONE;
}
function fluidLevel(depth) {
  const h = depth >= 8 ? 16 : Math.round((8 - Math.max(0, depth)) / 9 * 16);
  return h < 1 ? 1 : h > 16 ? 16 : h;
}
function mayHoldFluid(id, type, fluid) {
  return fluid !== FLUID_NONE || type !== TYPE_AIR && shapeOf(id) !== SHAPE_FULL_CUBE;
}
var DimensionBlockSource = class {
  constructor(dimension, host, resolver) {
    this.dimension = dimension;
    this.host = host;
    this.resolver = resolver;
    /** 区块未加载时是否当作空气（对应 unloadedChunkPolicy: "fall"） */
    this.unloadedAsAir = false;
    this.minY = Number.NaN;
    this.maxY = Number.NaN;
    this.empty = null;
    this.location = { x: 0, y: 0, z: 0 };
    this.groups = /* @__PURE__ */ new Map();
    this.direct = [];
    this.one = new Int32Array(1);
    this.oneType = new Int32Array(1);
    this.oneFluid = new Uint8Array(1);
  }
  /** 维度的高度范围 [minY, maxY]（两端都包含）。第一次用到时才读，读不到时不限制 */
  heightRange() {
    if (!Number.isNaN(this.minY)) return;
    try {
      const r = this.dimension.heightRange;
      this.minY = r.min;
      this.maxY = r.max - 1;
    } catch {
      this.minY = -Infinity;
      this.maxY = Infinity;
    }
  }
  /**
   * 空块过滤排除空气及受支持的 light_block_0～15。
   * 自定义 collision 可为任意方块提供碰撞，因此启用时仅排除空气。
   */
  emptyFilter() {
    if (this.empty !== null) return this.empty;
    const types = ["minecraft:air"];
    const exists = this.host.typeExists;
    if (exists !== void 0 && !this.resolver.hasCustomCollision) {
      for (let i = 0; i <= 15; i++) {
        const id = `minecraft:light_block_${i}`;
        if (this.resolver.hasRegisteredCollision(id)) continue;
        try {
          if (exists.call(this.host, id)) types.push(id);
        } catch {
        }
      }
    }
    this.empty = { excludeTypes: types };
    return this.empty;
  }
  readChunk(ox, oy, oz, cells, types, fluids) {
    this.heightRange();
    cells.fill(SHAPE_NONE);
    types.fill(TYPE_AIR);
    fluids.fill(FLUID_NONE);
    const y0 = Math.max(oy, this.minY), y1 = Math.min(oy + 7, this.maxY);
    if (y0 > y1) return true;
    try {
      const volume = this.host.createVolume(ox, y0, oz, ox + 7, y1, oz + 7);
      if (!this.dimension.containsBlock(volume, this.emptyFilter(), this.unloadedAsAir)) return true;
    } catch (e) {
      if (this.host.classifyError(e) === "unloaded") return this.unloadedAsAir;
    }
    for (let y = y0; y <= y1; y++) {
      for (let z = 0; z < 8; z++) {
        for (let x = 0; x < 8; x++) if (!this.readCell(ox + x, y, oz + z, cells, types, fluids, x + z * 8 + (y - oy) * 64)) return false;
      }
    }
    return true;
  }
  refreshCells(ox, oy, oz, cells, types, fluids, indices) {
    this.heightRange();
    const groups2 = this.groups, direct = this.direct;
    for (const g of groups2.values()) g.length = 0;
    direct.length = 0;
    const batch = !this.unloadedAsAir && this.host.createListVolume !== void 0;
    for (let n = 0; n < indices.length; n++) {
      const i = indices[n], y = oy + (i >> 6);
      if (y < this.minY || y > this.maxY) {
        cells[i] = SHAPE_NONE;
        types[i] = TYPE_AIR;
        fluids[i] = FLUID_NONE;
        continue;
      }
      const t = types[i];
      if (!batch || t < 0 || mayHoldFluid(cells[i], t, fluids[i])) {
        direct.push(i);
        continue;
      }
      let g = groups2.get(t);
      if (g === void 0) {
        g = [];
        groups2.set(t, g);
      }
      g.push(i);
    }
    for (const [t, g] of groups2) {
      if (g.length === 0) continue;
      if (g.length === 1) {
        direct.push(g[0]);
        continue;
      }
      try {
        const volume = this.groupVolume(ox, oy, oz, g);
        for (const p of this.dimension.getBlocks(volume, { excludeTypes: [this.resolver.typeOf(t)] }, false).getBlockLocationIterator()) {
          direct.push(p.x - ox + (p.z - oz) * 8 + (p.y - oy) * 64);
        }
      } catch {
        for (let n = 0; n < g.length; n++) direct.push(g[n]);
      }
    }
    for (let n = 0; n < direct.length; n++) {
      const i = direct[n];
      if (!this.readCell(ox + (i & 7), oy + (i >> 6), oz + (i >> 3 & 7), cells, types, fluids, i)) return false;
    }
    return true;
  }
  /** 一组格子的原生体积：正好填满包围盒时用 BlockVolume，否则用 ListBlockVolume */
  groupVolume(ox, oy, oz, g) {
    let x0 = 7, y0 = 7, z0 = 7, x1 = 0, y1 = 0, z1 = 0;
    for (let n = 0; n < g.length; n++) {
      const i = g[n], x = i & 7, y = i >> 6, z = i >> 3 & 7;
      if (x < x0) x0 = x;
      if (x > x1) x1 = x;
      if (y < y0) y0 = y;
      if (y > y1) y1 = y;
      if (z < z0) z0 = z;
      if (z > z1) z1 = z;
    }
    if ((x1 - x0 + 1) * (y1 - y0 + 1) * (z1 - z0 + 1) === g.length) return this.host.createVolume(ox + x0, oy + y0, oz + z0, ox + x1, oy + y1, oz + z1);
    return this.host.createListVolume(g.map((i) => ({ x: ox + (i & 7), y: oy + (i >> 6), z: oz + (i >> 3 & 7) })));
  }
  changed(x, y, z, id, type, fluid) {
    this.heightRange();
    if (y < this.minY || y > this.maxY) return id !== SHAPE_NONE || fluid !== FLUID_NONE;
    const p = this.location;
    p.x = x;
    p.y = y;
    p.z = z;
    let block;
    try {
      block = this.dimension.getBlock(p);
    } catch {
      return false;
    }
    if (block === void 0) return false;
    const t = this.resolver.token(block.typeId);
    let now2;
    if (t >= 0) {
      now2 = this.resolver.idOf(t);
      if (t !== type && now2 !== id) return true;
    } else {
      now2 = this.resolveBlock(block, x, y, z);
      if (now2 === SHAPE_UNLOADED) return false;
      if (now2 !== id) return true;
    }
    return fluidCode(block, now2) !== fluid;
  }
  /** 一格的方块编号（调试显示用）。区块没加载时返回 SHAPE_UNLOADED */
  getShapeId(x, y, z) {
    this.heightRange();
    if (y < this.minY || y > this.maxY) return SHAPE_NONE;
    return this.readCell(x, y, z, this.one, this.oneType, this.oneFluid, 0) ? this.one[0] : SHAPE_UNLOADED;
  }
  /** 一格的流体码（流体计算用）。区块没加载时返回 FLUID_NONE */
  fluidAt(x, y, z) {
    this.heightRange();
    if (y < this.minY || y > this.maxY) return FLUID_NONE;
    return this.readCell(x, y, z, this.one, this.oneType, this.oneFluid, 0) ? this.oneFluid[0] : FLUID_NONE;
  }
  /** 读一格写进 cells[i]、types[i]、fluids[i]。区块没加载时返回 false（unloadedAsAir 时当空气） */
  readCell(x, y, z, cells, types, fluids, i) {
    const p = this.location;
    p.x = x;
    p.y = y;
    p.z = z;
    let block;
    try {
      block = this.dimension.getBlock(p);
    } catch (e) {
      const kind = this.host.classifyError(e);
      if (kind === "outOfBounds") {
        cells[i] = SHAPE_NONE;
        types[i] = TYPE_AIR;
        fluids[i] = FLUID_NONE;
        return true;
      }
      if (kind !== "unloaded") throw e;
    }
    if (block !== void 0) {
      const t = this.resolver.token(block.typeId);
      const id = t >= 0 ? this.resolver.idOf(t) : this.resolveBlock(block, x, y, z);
      if (id !== SHAPE_UNLOADED) {
        cells[i] = id;
        types[i] = t;
        fluids[i] = fluidCode(block, id);
        return true;
      }
    }
    if (!this.unloadedAsAir) return false;
    cells[i] = SHAPE_NONE;
    types[i] = TYPE_AIR;
    fluids[i] = FLUID_NONE;
    return true;
  }
  /** 完整解析一格（依赖状态、邻居、自定义解析的方块）。邻居读不到（没加载、越界）当作不连接。区块没加载时返回 SHAPE_UNLOADED */
  resolveBlock(block, x, y, z) {
    try {
      return this.resolver.resolve(block.typeId, () => block.permutation.getAllStates(), () => ({
        pistonExtended: block.typeId === "minecraft:piston" || block.typeId === "minecraft:sticky_piston" ? ["Expanded", "Expanding"].includes(block.getComponent("minecraft:piston")?.state ?? "") : void 0,
        read: (dx, dy, dz) => {
          if (y + dy < this.minY || y + dy > this.maxY) return void 0;
          let other;
          try {
            other = this.dimension.getBlock({ x: x + dx, y: y + dy, z: z + dz });
          } catch (error) {
            const kind = this.host.classifyError(error);
            if (kind !== "outOfBounds" && kind !== "unloaded") throw error;
          }
          if (!other) return void 0;
          return { typeId: other.typeId, states: isStateIndependent(other.typeId) ? {} : other.permutation.getAllStates() };
        }
      }));
    } catch (error) {
      const kind = this.host.classifyError(error);
      if (kind === "unloaded") return SHAPE_UNLOADED;
      if (kind === "outOfBounds") return SHAPE_NONE;
      throw error;
    }
  }
};

// src/mc/block-events.ts
import { system, world } from "@minecraft/server";
var PISTON_DIRECTIONS = [
  { x: 0, y: -1, z: 0 },
  { x: 0, y: 1, z: 0 },
  { x: 0, y: 0, z: 1 },
  { x: 0, y: 0, z: -1 },
  { x: 1, y: 0, z: 0 },
  { x: -1, y: 0, z: 0 }
];
var PISTON_PUSH_LIMIT = 12;
function watchBlockEvents(dimensionId, onChanged) {
  const after = world.afterEvents;
  const unsubscribers = [];
  const pending = /* @__PURE__ */ new Set();
  const sameDimension = (e) => {
    const id = e.dimension?.id ?? e.block?.dimension?.id;
    return id === void 0 || id === dimensionId;
  };
  const guard = (handler) => (e) => {
    try {
      if (sameDimension(e)) handler(e);
    } catch {
    }
  };
  const add = (signal, handler) => {
    const cb = guard(handler);
    signal.subscribe(cb);
    unsubscribers.push(() => signal.unsubscribe(cb));
  };
  const around = (e) => {
    const l = e.block.location;
    onChanged(l.x - 1, l.y - 1, l.z - 1, l.x + 1, l.y + 1, l.z + 1, false);
  };
  const cell2 = (l) => onChanged(l.x, l.y, l.z, l.x, l.y, l.z, true);
  add(after.playerPlaceBlock, around);
  add(after.playerBreakBlock, around);
  add(after.blockExplode, around);
  add(after.playerInteractWithBlock, (e) => {
    if (e.isFirstEvent === true) around(e);
  });
  add(after.pressurePlatePush, (e) => cell2(e.block.location));
  add(after.pressurePlatePop, (e) => cell2(e.block.location));
  add(after.pistonActivate, (e) => {
    const p = e.block.location, d = PISTON_DIRECTIONS[e.block.permutation.getState("facing_direction")];
    if (d === void 0) return;
    const cells = [{ x: p.x, y: p.y, z: p.z }, { x: p.x + d.x, y: p.y + d.y, z: p.z + d.z }];
    if (e.isExpanding === true && e.dimension?.getBlock !== void 0) {
      try {
        for (let k = 2; k <= PISTON_PUSH_LIMIT + 1; k++) {
          const l = { x: p.x + d.x * k, y: p.y + d.y * k, z: p.z + d.z * k };
          const b = e.dimension.getBlock(l);
          if (b === void 0 || b.isAir || b.isLiquid) break;
          cells.push(l);
        }
      } catch {
      }
    }
    for (const a of e.piston.getAttachedBlocksLocations()) {
      cells.push({ x: a.x, y: a.y, z: a.z }, { x: a.x + d.x, y: a.y + d.y, z: a.z + d.z }, { x: a.x - d.x, y: a.y - d.y, z: a.z - d.z });
    }
    for (const l of cells) cell2(l);
    const id = system.runTimeout(() => {
      pending.delete(id);
      for (const l of cells) cell2(l);
    }, 1);
    pending.add(id);
  });
  add(after.explosion, (e) => {
    for (const b of e.getImpactedBlocks()) cell2(b.location);
  });
  return () => {
    for (const u of unsubscribers) {
      try {
        u();
      } catch {
      }
    }
    unsubscribers.length = 0;
    for (const id of pending) system.clearRun(id);
    pending.clear();
  };
}

// src/mc/explosions.ts
var MIN_RADIUS = 2.5;
var DEFAULT_RADIUS = 4.25;
var RADIUS_MARGIN = 1.25;
var MAX_RADIUS = 8;
var IMPULSE_MULTIPLIER = 2.78;
var OCCLUDED_SCALE = 0.35;
var FALLOFF_POWER = 0.68;
var UPWARD_BIAS = 0.4;
var MAX_EFFECTS_PER_BODY = 16;
var MAX_OCCLUSION_RAYS_PER_BODY = 4;
var MAX_LINEAR_VELOCITY_CHANGE = 12;
var MAX_ANGULAR_VELOCITY_CHANGE = 2.5;
var PUNCH_STRENGTH = 2.1;
var UPRIGHT_MULTIPLIER = 1.5;
var BACKEND_COMPENSATION = 1.5;
var CURVE_SCALE = 2;
var CURVE_EXPONENT = 0.5;
var CURVE_SLOPE = 0.8;
var ZERO_MASS_IMPULSE = 1.75;
var CURVE_OFFSET = CURVE_SLOPE / (CURVE_SCALE * CURVE_EXPONENT);
var CURVE_INVERSE = 1 / (CURVE_EXPONENT - 1);
var CURVE_BASE = Math.pow(CURVE_OFFSET, CURVE_INVERSE);
var CURVE_BIAS = Math.pow(CURVE_OFFSET, CURVE_EXPONENT * CURVE_INVERSE);
var CURVE_QUADRATIC = ZERO_MASS_IMPULSE + CURVE_SLOPE - 2;
var EPSILON = 1e-8;
function punchCurve(mass) {
  if (!Number.isFinite(mass) || mass <= 0) return 0;
  const m = mass - 1;
  if (mass < 1) return ((CURVE_QUADRATIC * m + CURVE_SLOPE - 1) * m + 1) * mass;
  return CURVE_SCALE * (Math.pow(m + CURVE_BASE, CURVE_EXPONENT) - CURVE_BIAS) + 1;
}
function explosionOf(source, impacted) {
  let center = source;
  if (center === void 0) {
    if (impacted.length === 0) return void 0;
    let x = 0, y = 0, z = 0;
    for (const b of impacted) {
      x += b.x + 0.5;
      y += b.y + 0.5;
      z += b.z + 0.5;
    }
    center = { x: x / impacted.length, y: y / impacted.length, z: z / impacted.length };
  }
  if (impacted.length === 0) return { center, radius: DEFAULT_RADIUS };
  let max = 0;
  for (const b of impacted) max = Math.max(max, (b.x + 0.5 - center.x) ** 2 + (b.y + 0.5 - center.y) ** 2 + (b.z + 0.5 - center.z) ** 2);
  return { center, radius: Math.min(MAX_RADIUS, Math.max(MIN_RADIUS, Math.sqrt(max) + RADIUS_MARGIN)) };
}
var point = new SphereGeometry(0);
var pointTransform = new Transform();
function closestOnShape(geom, tf, c, radius, out) {
  if (geom instanceof BoxGeometry) {
    const dx = c.x - tf.px, dy = c.y - tf.py, dz = c.z - tf.pz;
    const lx = Math.max(-geom.hx, Math.min(geom.hx, tf.r00 * dx + tf.r10 * dy + tf.r20 * dz));
    const ly = Math.max(-geom.hy, Math.min(geom.hy, tf.r01 * dx + tf.r11 * dy + tf.r21 * dz));
    const lz = Math.max(-geom.hz, Math.min(geom.hz, tf.r02 * dx + tf.r12 * dy + tf.r22 * dz));
    out.x = tf.px + tf.r00 * lx + tf.r01 * ly + tf.r02 * lz;
    out.y = tf.py + tf.r10 * lx + tf.r11 * ly + tf.r12 * lz;
    out.z = tf.pz + tf.r20 * lx + tf.r21 * ly + tf.r22 * lz;
  } else {
    const gjk = GjkEpa.getInstance();
    pointTransform.px = c.x;
    pointTransform.py = c.y;
    pointTransform.pz = c.z;
    if (gjk.computeDistance(point, geom, pointTransform, tf, null) !== GjkEpaResultState.SUCCEEDED) return void 0;
    if (gjk.distance <= geom.gjkMargin) {
      out.x = c.x;
      out.y = c.y;
      out.z = c.z;
    } else {
      const p = gjk.closestPoint2, s = geom.gjkMargin / gjk.distance;
      out.x = p.x + (c.x - p.x) * s;
      out.y = p.y + (c.y - p.y) * s;
      out.z = p.z + (c.z - p.z) * s;
    }
  }
  const d = Math.hypot(out.x - c.x, out.y - c.y, out.z - c.z);
  return d < radius ? d : void 0;
}
function worldOcclusion(dimension) {
  return (center, contact) => {
    const ox = contact.x - center.x, oy = contact.y - center.y, oz = contact.z - center.z;
    const distance = Math.hypot(ox, oy, oz);
    if (!Number.isFinite(distance) || distance <= 0.2) return false;
    try {
      return dimension.getBlockFromRay(center, { x: ox / distance, y: oy / distance, z: oz / distance }, {
        includeLiquidBlocks: false,
        includePassableBlocks: false,
        maxDistance: Math.max(0, distance - 0.15)
      }) !== void 0;
    } catch {
      return false;
    }
  };
}
function watchExplosions(signal, dimensionId, queue) {
  const cb = (e) => {
    try {
      if (e.dimension !== void 0 && e.dimension.id !== dimensionId) return;
      const impacted = [];
      for (const b of e.getImpactedBlocks()) {
        try {
          impacted.push({ x: b.location.x, y: b.location.y, z: b.location.z });
        } catch {
        }
      }
      let source;
      try {
        const l = e.source?.location;
        if (l !== void 0 && Number.isFinite(l.x) && Number.isFinite(l.y) && Number.isFinite(l.z)) source = { x: l.x, y: l.y, z: l.z };
      } catch {
      }
      const blast = explosionOf(source, impacted);
      if (blast !== void 0) queue.queue(blast.center, blast.radius);
    } catch {
    }
  };
  signal.subscribe(cb);
  return () => {
    try {
      signal.unsubscribe(cb);
    } catch {
    }
  };
}
var ExplosionQueue = class {
  constructor() {
    this.pending = /* @__PURE__ */ new Map();
  }
  get size() {
    return this.pending.size;
  }
  queue(center, radius) {
    if (!Number.isFinite(center.x) || !Number.isFinite(center.y) || !Number.isFinite(center.z) || !(radius > 0) || !Number.isFinite(radius)) return;
    const key2 = `${Math.round(center.x * 4)},${Math.round(center.y * 4)},${Math.round(center.z * 4)}|${Math.round(radius * 2)}`;
    const e = this.pending.get(key2);
    if (e === void 0) {
      this.pending.set(key2, { center: { x: center.x, y: center.y, z: center.z }, radius, count: 1 });
      return;
    }
    const n = e.count + 1;
    e.center = { x: (e.center.x * e.count + center.x) / n, y: (e.center.y * e.count + center.y) / n, z: (e.center.z * e.count + center.z) / n };
    e.count = n;
    e.radius = Math.max(e.radius, radius);
  }
  clear() {
    this.pending.clear();
  }
  apply(bodies, occluded) {
    if (this.pending.size === 0) return;
    const explosions = [...this.pending.values()];
    this.pending.clear();
    const contact = { x: 0, y: 0, z: 0 }, best = { x: 0, y: 0, z: 0 };
    for (const rb of bodies) {
      if (rb.type !== RigidBodyType.DYNAMIC || rb.shapeList === null) continue;
      let minX = Infinity, minY = Infinity, minZ = Infinity, maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
      for (let s = rb.shapeList; s !== null; s = s.next) {
        const a = s.aabb;
        minX = Math.min(minX, a.minX);
        minY = Math.min(minY, a.minY);
        minZ = Math.min(minZ, a.minZ);
        maxX = Math.max(maxX, a.maxX);
        maxY = Math.max(maxY, a.maxY);
        maxZ = Math.max(maxZ, a.maxZ);
      }
      const near = [];
      for (const e of explosions) {
        const c = e.center;
        const dx = c.x - Math.max(minX, Math.min(maxX, c.x)), dy = c.y - Math.max(minY, Math.min(maxY, c.y)), dz = c.z - Math.max(minZ, Math.min(maxZ, c.z));
        const d2 = dx * dx + dy * dy + dz * dz;
        if (d2 > e.radius * e.radius) continue;
        near.push({ e, score: d2 / (e.radius * e.radius) });
      }
      if (near.length === 0) continue;
      near.sort((a, b) => a.score - b.score);
      if (near.length > MAX_EFFECTS_PER_BODY) near.length = MAX_EFFECTS_PER_BODY;
      let lX = 0, lY = 0, lZ = 0, tX = 0, tY = 0, tZ = 0, contacted = false, rays = 0;
      const tf = rb.transform;
      const upright = Math.max(0, Math.min(1, tf.r11));
      for (const { e } of near) {
        let distance = Infinity;
        for (let s = rb.shapeList; s !== null; s = s.next) {
          const d = closestOnShape(s.geom, s.transform, e.center, e.radius, contact);
          if (d !== void 0 && d < distance) {
            distance = d;
            best.x = contact.x;
            best.y = contact.y;
            best.z = contact.z;
          }
        }
        if (distance === Infinity) continue;
        let occlusion = 1;
        if (rays < MAX_OCCLUSION_RAYS_PER_BODY && distance > 0.2 && occluded(e.center, best)) occlusion = OCCLUDED_SCALE;
        if (rays < MAX_OCCLUSION_RAYS_PER_BODY) rays++;
        let dX = best.x - e.center.x, dY = best.y - e.center.y, dZ = best.z - e.center.z;
        if (Math.hypot(dX, dY, dZ) < EPSILON) {
          dX = tf.px - e.center.x;
          dY = tf.py - e.center.y;
          dZ = tf.pz - e.center.z;
        }
        dY += UPWARD_BIAS;
        let len = Math.hypot(dX, dY, dZ);
        if (len < EPSILON) {
          dX = 0;
          dY = 1;
          dZ = 0;
          len = 1;
        }
        dX /= len;
        dY /= len;
        dZ /= len;
        const rX = best.x - tf.px, rY = best.y - tf.py, rZ = best.z - tf.pz;
        const cX = rY * dZ - rZ * dY, cY = rZ * dX - rX * dZ, cZ = rX * dY - rY * dX;
        const k = rb.invMass + cX * (rb.ii00 * cX + rb.ii01 * cY + rb.ii02 * cZ) + cY * (rb.ii10 * cX + rb.ii11 * cY + rb.ii12 * cZ) + cZ * (rb.ii20 * cX + rb.ii21 * cY + rb.ii22 * cZ);
        const punch = BACKEND_COMPENSATION * (1 + UPRIGHT_MULTIPLIER * upright) * PUNCH_STRENGTH * punchCurve(k > 0 ? 1 / k : 0);
        const magnitude = occlusion * Math.pow(Math.max(0, 1 - distance / e.radius), FALLOFF_POWER) * Math.min(2, Math.sqrt(Math.max(1, Math.floor(e.count)))) * punch * IMPULSE_MULTIPLIER;
        if (!(magnitude > 0)) continue;
        contacted = true;
        const iX = dX * magnitude, iY = dY * magnitude, iZ = dZ * magnitude;
        lX += iX;
        lY += iY;
        lZ += iZ;
        tX += rY * iZ - rZ * iY;
        tY += rZ * iX - rX * iZ;
        tZ += rX * iY - rY * iX;
      }
      if (!contacted) continue;
      const maxL = rb.mass * MAX_LINEAR_VELOCITY_CHANGE, l = Math.hypot(lX, lY, lZ);
      if (l > maxL) {
        const s = maxL / l;
        lX *= s;
        lY *= s;
        lZ *= s;
      }
      let inertia = Infinity;
      for (const v of [rb.li00, rb.li11, rb.li22]) if (v > 0 && v < inertia) inertia = v;
      const maxT = Number.isFinite(inertia) ? inertia * MAX_ANGULAR_VELOCITY_CHANGE : 0, t = Math.hypot(tX, tY, tZ);
      if (t > maxT) {
        const s = t > 0 ? maxT / t : 0;
        tX *= s;
        tY *= s;
        tZ *= s;
      }
      rb.applyLinearImpulse(lX, lY, lZ);
      rb.applyAngularImpulse(tX, tY, tZ);
    }
  }
};

// src/mc/floating.ts
function floatingClusters(points) {
  const groups2 = /* @__PURE__ */ new Map();
  for (const p of points) {
    if (p.scale <= 0) continue;
    let c = groups2.get(p.material);
    if (c === void 0) {
      c = { material: p.material, cx: 0, cy: 0, cz: 0, cov: new Float64Array(9), scale: 0 };
      groups2.set(p.material, c);
    }
    c.scale += p.scale;
    c.cx += p.x * p.scale;
    c.cy += p.y * p.scale;
    c.cz += p.z * p.scale;
  }
  for (const c of groups2.values()) {
    c.cx /= c.scale;
    c.cy /= c.scale;
    c.cz /= c.scale;
  }
  for (const p of points) {
    if (p.scale <= 0) continue;
    const c = groups2.get(p.material);
    const o0 = p.x - c.cx, o1 = p.y - c.cy, o2 = p.z - c.cz, w = p.scale / c.scale;
    const m = c.cov;
    m[0] += o0 * o0 * w;
    m[1] += o0 * o1 * w;
    m[2] += o0 * o2 * w;
    m[3] += o1 * o0 * w;
    m[4] += o1 * o1 * w;
    m[5] += o1 * o2 * w;
    m[6] += o2 * o0 * w;
    m[7] += o2 * o1 * w;
    m[8] += o2 * o2 * w;
  }
  for (const c of groups2.values()) {
    c.cov[0] += 1 / 6;
    c.cov[4] += 1 / 6;
    c.cov[8] += 1 / 6;
  }
  return [...groups2.values()];
}
var inv = new Float64Array(9);
var shiftedCov = new Float64Array(9);
var slowDrag = new Float64Array(9);
var fastDrag = new Float64Array(9);
var torqueM = new Float64Array(9);
function det(m) {
  return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
}
function invert(m, out) {
  const a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
  const A = e * i - f * h, B = f * g - d * i, C = d * h - e * g;
  const D = det(m);
  if (D === 0) {
    out.fill(0);
    return;
  }
  const s = 1 / D;
  out[0] = A * s;
  out[1] = (c * h - b * i) * s;
  out[2] = (b * f - c * e) * s;
  out[3] = B * s;
  out[4] = (a * i - c * g) * s;
  out[5] = (c * d - a * f) * s;
  out[6] = C * s;
  out[7] = (b * g - a * h) * s;
  out[8] = (a * e - b * d) * s;
}
function dragMatrix(gx, gy, gz, vertical, horizontal, scale, out) {
  const g2 = gx * gx + gy * gy + gz * gz;
  const k = g2 > 0 ? (horizontal - vertical) / g2 : 0;
  const g0 = gx, g1 = gy, g2v = gz;
  out[0] = scale * (k * g0 * g0 - horizontal);
  out[1] = scale * k * g0 * g1;
  out[2] = scale * k * g0 * g2v;
  out[3] = scale * k * g1 * g0;
  out[4] = scale * (k * g1 * g1 - horizontal);
  out[5] = scale * k * g1 * g2v;
  out[6] = scale * k * g2v * g0;
  out[7] = scale * k * g2v * g1;
  out[8] = scale * (k * g2v * g2v - horizontal);
}
function addDragTorque(x, y, out) {
  const tx = x[0] + x[4] + x[8], ty = y[0] + y[4] + y[8];
  let txy = 0;
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) txy += x[r * 3 + c] * y[c * 3 + r];
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      let v = -tx * y[r * 3 + c] - ty * x[r * 3 + c];
      for (let k = 0; k < 3; k++) v += x[r * 3 + k] * y[k * 3 + c] + y[r * 3 + k] * x[k * 3 + c];
      out[r * 3 + c] += v + (r === c ? tx * ty - txy : 0);
    }
  }
}
function applyFloating(rb, clusters, gx, gy, gz) {
  if (clusters.length === 0 || rb.sleeping) return;
  const t = rb.transform;
  const r00 = t.r00, r01 = t.r01, r02 = t.r02, r10 = t.r10, r11 = t.r11, r12 = t.r12, r20 = t.r20, r21 = t.r21, r22 = t.r22;
  const gs = rb.gravityScale;
  const lgx = (r00 * gx + r10 * gy + r20 * gz) * gs, lgy = (r01 * gx + r11 * gy + r21 * gz) * gs, lgz = (r02 * gx + r12 * gy + r22 * gz) * gs;
  const lvx = r00 * rb.vx + r10 * rb.vy + r20 * rb.vz, lvy = r01 * rb.vx + r11 * rb.vy + r21 * rb.vz, lvz = r02 * rb.vx + r12 * rb.vy + r22 * rb.vz;
  const wx = r00 * rb.wx + r10 * rb.wy + r20 * rb.wz, wy = r01 * rb.wx + r11 * rb.wy + r21 * rb.wz, wz = r02 * rb.wx + r12 * rb.wy + r22 * rb.wz;
  const w2 = wx * wx + wy * wy + wz * wz;
  const gLen = Math.sqrt(lgx * lgx + lgy * lgy + lgz * lgz);
  let fx = 0, fy = 0, fz = 0, tx = 0, ty = 0, tz = 0;
  let limited = 0, lcx = 0, lcy = 0, lcz = 0;
  for (let n = 0; n < clusters.length; n++) {
    const cl = clusters[n], m = cl.material;
    const cx = cl.cx, cy = cl.cy, cz = cl.cz;
    const lift = cl.scale * m.liftStrength;
    if (m.preventSelfLift) {
      limited += lift;
      lcx += cx * lift;
      lcy += cy * lift;
      lcz += cz * lift;
    } else {
      const px = -lgx * lift, py = -lgy * lift, pz = -lgz * lift;
      fx += px;
      fy += py;
      fz += pz;
      tx += cy * pz - cz * py;
      ty += cz * px - cx * pz;
      tz += cx * py - cy * px;
    }
    const speedScale = m.transitionSpeed === 0 ? 0 : 3 / (m.transitionSpeed * m.transitionSpeed);
    invert(cl.cov, inv);
    const ang = [wx, wy, wz];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) inv[r * 3 + c] += speedScale * ((r === c ? w2 : 0) - ang[r] * ang[c]);
    invert(inv, shiftedCov);
    const mvx = wy * cz - wz * cy + lvx, mvy = wz * cx - wx * cz + lvy, mvz = wx * cy - wy * cx + lvz;
    const s0 = speedScale * (wy * mvz - wz * mvy), s1 = speedScale * (wz * mvx - wx * mvz), s2 = speedScale * (wx * mvy - wy * mvx);
    const shx = shiftedCov[0] * s0 + shiftedCov[1] * s1 + shiftedCov[2] * s2;
    const shy = shiftedCov[3] * s0 + shiftedCov[4] * s1 + shiftedCov[5] * s2;
    const shz = shiftedCov[6] * s0 + shiftedCov[7] * s1 + shiftedCov[8] * s2;
    const ix = inv[0] * shx + inv[1] * shy + inv[2] * shz, iy = inv[3] * shx + inv[4] * shy + inv[5] * shz, iz = inv[6] * shx + inv[7] * shy + inv[8] * shz;
    const slowScale = speedScale === 0 ? 0 : Math.sqrt(det(shiftedCov) / det(cl.cov)) * Math.exp(-0.5 * (speedScale * (mvx * mvx + mvy * mvy + mvz * mvz) - (shx * ix + shy * iy + shz * iz)));
    const frictionScale = cl.scale * (m.scaleFrictionWithGravity ? gLen : 1);
    dragMatrix(lgx, lgy, lgz, m.slowVerticalFriction, m.slowHorizontalFriction, frictionScale * slowScale, slowDrag);
    dragMatrix(lgx, lgy, lgz, m.fastVerticalFriction, m.fastHorizontalFriction, frictionScale, fastDrag);
    const svx = wy * shz - wz * shy + mvx, svy = wz * shx - wx * shz + mvy, svz = wx * shy - wy * shx + mvz;
    const sfx = slowDrag[0] * svx + slowDrag[1] * svy + slowDrag[2] * svz;
    const sfy = slowDrag[3] * svx + slowDrag[4] * svy + slowDrag[5] * svz;
    const sfz = slowDrag[6] * svx + slowDrag[7] * svy + slowDrag[8] * svz;
    const ffx = fastDrag[0] * mvx + fastDrag[1] * mvy + fastDrag[2] * mvz + sfx;
    const ffy = fastDrag[3] * mvx + fastDrag[4] * mvy + fastDrag[5] * mvz + sfy;
    const ffz = fastDrag[6] * mvx + fastDrag[7] * mvy + fastDrag[8] * mvz + sfz;
    let qx = cy * ffz - cz * ffy, qy = cz * ffx - cx * ffz, qz = cx * ffy - cy * ffx;
    qx += shy * sfz - shz * sfy;
    qy += shz * sfx - shx * sfz;
    qz += shx * sfy - shy * sfx;
    torqueM.fill(0);
    addDragTorque(cl.cov, fastDrag, torqueM);
    addDragTorque(shiftedCov, slowDrag, torqueM);
    qx += torqueM[0] * wx + torqueM[1] * wy + torqueM[2] * wz;
    qy += torqueM[3] * wx + torqueM[4] * wy + torqueM[5] * wz;
    qz += torqueM[6] * wx + torqueM[7] * wy + torqueM[8] * wz;
    fx += ffx;
    fy += ffy;
    fz += ffz;
    tx += qx;
    ty += qy;
    tz += qz;
  }
  if (limited > 0 && gLen > 0) {
    const cx = lcx / limited, cy = lcy / limited, cz = lcz / limited;
    const ax = r00 * cx + r01 * cy + r02 * cz, ay = r10 * cx + r11 * cy + r12 * cz, az = r20 * cx + r21 * cy + r22 * cz;
    const gwx = r00 * lgx + r01 * lgy + r02 * lgz, gwy = r10 * lgx + r11 * lgy + r12 * lgz, gwz = r20 * lgx + r21 * lgy + r22 * lgz;
    const ux = gwx / gLen, uy = gwy / gLen, uz = gwz / gLen;
    const kx = ay * uz - az * uy, ky = az * ux - ax * uz, kz = ax * uy - ay * ux;
    const ikx = rb.ii00 * kx + rb.ii01 * ky + rb.ii02 * kz, iky = rb.ii10 * kx + rb.ii11 * ky + rb.ii12 * kz, ikz = rb.ii20 * kx + rb.ii21 * ky + rb.ii22 * kz;
    const effective = 1 / (rb.invMass + kx * ikx + ky * iky + kz * ikz);
    const amount = Math.min(limited, effective);
    const px = -lgx * amount, py = -lgy * amount, pz = -lgz * amount;
    fx += px;
    fy += py;
    fz += pz;
    tx += cy * pz - cz * py;
    ty += cz * px - cx * pz;
    tz += cx * py - cy * px;
  }
  rb.fx += r00 * fx + r01 * fy + r02 * fz;
  rb.fy += r10 * fx + r11 * fy + r12 * fz;
  rb.fz += r20 * fx + r21 * fy + r22 * fz;
  rb.tx += r00 * tx + r01 * ty + r02 * tz;
  rb.ty += r10 * tx + r11 * ty + r12 * tz;
  rb.tz += r20 * tx + r21 * ty + r22 * tz;
}

// src/mc/signal.ts
var SignalImpl = class {
  constructor() {
    this.callbacks = [];
  }
  /** 是否存在订阅者，用于跳过无订阅时的事件数据构造。 */
  get hasSubscribers() {
    return this.callbacks.length > 0;
  }
  subscribe(callback) {
    if (typeof callback !== "function") throw new Error("\u8BA2\u9605\u4E8B\u4EF6\u7684\u56DE\u8C03\u5FC5\u987B\u662F\u51FD\u6570");
    if (!this.callbacks.includes(callback)) this.callbacks.push(callback);
    return callback;
  }
  unsubscribe(callback) {
    const i = this.callbacks.indexOf(callback);
    if (i >= 0) this.callbacks.splice(i, 1);
  }
  clear() {
    this.callbacks.length = 0;
  }
  /** 依次调用所有订阅者。单个回调错误不阻断其余回调，全部调用结束后抛出首个错误。 */
  emit(event) {
    if (this.callbacks.length === 0) return;
    let failed = false;
    let firstError;
    for (const callback of this.callbacks.slice()) {
      try {
        callback(event);
      } catch (e) {
        if (!failed) {
          failed = true;
          firstError = e;
        }
      }
    }
    if (failed) throw firstError;
  }
};
function createAfterEvents() {
  return {
    bodyCollisionStart: new SignalImpl(),
    bodyCollisionEnd: new SignalImpl(),
    blockCollisionStart: new SignalImpl(),
    bodySleep: new SignalImpl(),
    bodyWake: new SignalImpl(),
    stepEnd: new SignalImpl(),
    jointBreak: new SignalImpl(),
    waterEntry: new SignalImpl(),
    lavaEntry: new SignalImpl(),
    blockBreak: new SignalImpl()
  };
}

// src/mc/compound-boxes.ts
var key = (x, y, z) => `${x},${y},${z}`;
function mergeBlockShapes(shapes) {
  if (shapes.length < 2) return shapes;
  const groups2 = /* @__PURE__ */ new Map();
  for (const shape of shapes) {
    const info = shape.userData;
    const g = shape.geom, t = shape.localTransform;
    if (!info || info.physics.fragileImpactSpeed !== void 0 || info.physics.floatingMaterial !== void 0 || !(g instanceof BoxGeometry) || g.hx !== 0.5 || g.hy !== 0.5 || g.hz !== 0.5 || t.r00 !== 1 || t.r11 !== 1 || t.r22 !== 1 || t.r01 !== 0 || t.r02 !== 0 || t.r10 !== 0 || t.r12 !== 0 || t.r20 !== 0 || t.r21 !== 0) continue;
    const { x, y, z } = info.block.location;
    if (!Number.isSafeInteger(x) || !Number.isSafeInteger(y) || !Number.isSafeInteger(z)) continue;
    const groupKey = `${shape.density},${shape.friction},${shape.restitution},${shape.collisionGroup},${shape.collisionMask}:${(t.px - x).toFixed(9)},${(t.py - y).toFixed(9)},${(t.pz - z).toFixed(9)}`;
    let group = groups2.get(groupKey);
    if (!group) {
      group = /* @__PURE__ */ new Map();
      groups2.set(groupKey, group);
    }
    const cellKey = key(x, y, z);
    if (group.has(cellKey)) continue;
    group.set(cellKey, { x, y, z, shape });
  }
  const replaced = /* @__PURE__ */ new Map();
  for (const group of groups2.values()) {
    if (group.size < 2) continue;
    const cells = [...group.values()].sort((a, b) => a.y - b.y || a.z - b.z || a.x - b.x);
    for (const cell2 of cells) {
      const { x, y, z, shape } = cell2;
      if (!group.has(key(x, y, z))) continue;
      let nx = 1, nz = 1, ny = 1;
      while (group.has(key(x + nx, y, z))) nx++;
      zScan: for (; ; ) {
        for (let dx = 0; dx < nx; dx++) if (!group.has(key(x + dx, y, z + nz))) break zScan;
        nz++;
      }
      yScan: for (; ; ) {
        for (let dz = 0; dz < nz; dz++) for (let dx = 0; dx < nx; dx++) if (!group.has(key(x + dx, y + ny, z + dz))) break yScan;
        ny++;
      }
      if (nx * ny * nz === 1) {
        group.delete(key(x, y, z));
        continue;
      }
      const config = new ShapeConfig(new BoxGeometry(nx / 2, ny / 2, nz / 2));
      config.transform.copyFrom(shape.localTransform);
      config.transform.px += (nx - 1) / 2;
      config.transform.py += (ny - 1) / 2;
      config.transform.pz += (nz - 1) / 2;
      config.density = shape.density;
      config.friction = shape.friction;
      config.restitution = shape.restitution;
      config.collisionGroup = shape.collisionGroup;
      config.collisionMask = shape.collisionMask;
      for (let dy = 0; dy < ny; dy++) for (let dz = 0; dz < nz; dz++) for (let dx = 0; dx < nx; dx++) {
        const k = key(x + dx, y + dy, z + dz);
        replaced.set(group.get(k).shape, null);
        group.delete(k);
      }
      replaced.set(shape, new Shape(config));
    }
  }
  if (replaced.size === 0) return shapes;
  const result = [];
  for (const shape of shapes) {
    const replacement = replaced.get(shape);
    if (replacement === null) continue;
    result.push(replacement ?? shape);
  }
  return result;
}

// src/mc/snapshot.ts
function copyJson(value, depth = 0, seen2 = /* @__PURE__ */ new Set()) {
  if (depth > 64) throw new Error("\u5B58\u6863\u6570\u636E\u5D4C\u5957\u8D85\u8FC7 64 \u5C42");
  if (value === null || typeof value === "boolean" || typeof value === "string") return value;
  if (typeof value === "number") {
    if (Number.isFinite(value)) return value;
    throw new Error(`\u5B58\u6863\u6570\u636E\u4E0D\u80FD\u5305\u542B ${value}\uFF08JSON \u65E0\u6CD5\u8868\u793A NaN \u548C\u65E0\u7A77\u5927\uFF09`);
  }
  if (typeof value !== "object" || value === null || seen2.has(value)) throw new Error("\u5B58\u6863\u6570\u636E\u5FC5\u987B\u662F\u65E0\u5FAA\u73AF\u7684 JSON \u503C");
  const proto = Object.getPrototypeOf(value);
  if (!Array.isArray(value) && proto !== Object.prototype && proto !== null) throw new Error("\u5B58\u6863\u4E0D\u63A5\u53D7\u5B9E\u4F53\u6216\u7C7B\u5B9E\u4F8B\uFF0C\u8BF7\u8F6C\u6362\u4E3A\u666E\u901A\u6570\u636E");
  seen2.add(value);
  let result;
  if (Array.isArray(value)) result = Array.from(value, (v) => copyJson(v, depth + 1, seen2));
  else result = Object.fromEntries(Object.entries(value).map(([k, v]) => [k, copyJson(v, depth + 1, seen2)]));
  seen2.delete(value);
  return result;
}
function shapeSnapshot(shape) {
  const t = shape.localTransform, g = shape.geom;
  const common = {
    offset: { x: t.px, y: t.py, z: t.pz },
    rotation: matrixToQuaternion(t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22),
    friction: shape.friction,
    restitution: shape.restitution,
    density: shape.density,
    collisionGroup: shape.collisionGroup,
    collisionMask: shape.collisionMask
  };
  const block = shape.userData?.block;
  if (block !== void 0) common.block = block.states === void 0 ? { location: { ...block.location }, typeId: block.typeId } : { location: { ...block.location }, typeId: block.typeId, states: { ...block.states } };
  if (g instanceof BoxGeometry) return { ...common, type: "box", size: { x: g.hx * 2, y: g.hy * 2, z: g.hz * 2 } };
  if (g instanceof SphereGeometry) return { ...common, type: "sphere", radius: g.radius };
  if (g instanceof CapsuleGeometry) return { ...common, type: "capsule", radius: g.radius, height: 2 * (g.halfHeight + g.radius) };
  if (g instanceof CylinderGeometry) return { ...common, type: "cylinder", radius: g.radius, height: 2 * g.halfHeight };
  if (g instanceof ConeGeometry) return { ...common, type: "cone", radius: g.radius, height: 2 * g.halfHeight };
  if (g instanceof ConvexHullGeometry) return { ...common, type: "convexHull", points: g.vertices.map((p) => ({ x: p.x, y: p.y, z: p.z })) };
  throw new Error("\u65E0\u6CD5\u4FDD\u5B58\u672A\u77E5\u7684\u5F62\u72B6\u7C7B\u578B");
}
var SnapshotShapeCache = class {
  constructor() {
    this.entries = /* @__PURE__ */ new WeakMap();
    this.texts = /* @__PURE__ */ new WeakMap();
  }
  capture(shape) {
    const entry = this.entries.get(shape), t = shape.localTransform, v = entry?.value, p = entry?.transform;
    if (v && p && p.px === t.px && p.py === t.py && p.pz === t.pz && p.r00 === t.r00 && p.r01 === t.r01 && p.r02 === t.r02 && p.r10 === t.r10 && p.r11 === t.r11 && p.r12 === t.r12 && p.r20 === t.r20 && p.r21 === t.r21 && p.r22 === t.r22 && v.friction === shape.friction && v.restitution === shape.restitution && v.density === shape.density && v.collisionGroup === shape.collisionGroup && v.collisionMask === shape.collisionMask) {
      const g = shape.geom;
      if (!(g instanceof ConvexHullGeometry) || v.type === "convexHull" && v.points.length === g.vertices.length && g.vertices.every((a, i) => a.x === v.points[i].x && a.y === v.points[i].y && a.z === v.points[i].z)) return v;
    }
    const value = shapeSnapshot(shape), transform = new Transform();
    shapeOptions(value);
    transform.copyFrom(t);
    this.entries.set(shape, { transform, value });
    this.texts.set(value, JSON.stringify(value));
    return value;
  }
  text(value) {
    return this.texts.get(value) ?? JSON.stringify(value);
  }
};
function captureBody(body, options, cache2) {
  const c = body.core;
  const shapes = [];
  if (body.snapshotShapes) {
    for (const s of body.snapshotShapes) shapes.push(cache2 ? cache2.capture(s) : shapeSnapshot(s));
  } else {
    for (let s = c.shapeList; s !== null; s = s.next) shapes.push(cache2 ? cache2.capture(s) : shapeSnapshot(s));
  }
  const out = {
    options: {
      type: body.type,
      shapes,
      location: body.location,
      rotation: body.rotation,
      linearVelocity: body.linearVelocity,
      angularVelocity: body.angularVelocity,
      linearDamping: body.linearDamping,
      angularDamping: body.angularDamping,
      gravityScale: body.gravityScale,
      allowSleep: c.autoSleep
    },
    // isSleeping 不算等地形：等地形的刚体恢复后照常运动（速度一直留着）
    sleeping: body.isSleeping,
    force: { x: c.fx, y: c.fy, z: c.fz },
    torque: { x: c.tx, y: c.ty, z: c.tz }
  };
  if (options.encodeUserData) out.data = copyJson(options.encodeUserData(body));
  return out;
}
function parts(c) {
  if (c instanceof SphericalJoint) return { springs: [c.sd], linear: [], angular: [] };
  if (c instanceof RevoluteJoint) return { springs: [c.sd], linear: [], angular: [c.lm] };
  if (c instanceof PrismaticJoint) return { springs: [c.sd], linear: [c.lm], angular: [] };
  if (c instanceof CylindricalJoint) return { springs: [c.translSd, c.rotSd], linear: [c.translLm], angular: [c.rotLm] };
  if (c instanceof UniversalJoint) return { springs: [c.sd1, c.sd2], linear: [], angular: [c.lm1, c.lm2] };
  if (c instanceof RagdollJoint) return { springs: [c.twistSd, c.swingSd], linear: [], angular: [c.twistLm] };
  if (c instanceof GenericJoint) return { springs: [...c.translSds, ...c.rotSds], linear: c.translLms, angular: c.rotLms };
  throw new Error("\u65E0\u6CD5\u4FDD\u5B58\u672A\u77E5\u7684\u5173\u8282\u7C7B\u578B");
}
function captureJoint(j, ids) {
  const c = j.core, p = parts(c);
  return {
    type: j.type,
    bodyA: ids.get(j.bodyA),
    bodyB: ids.get(j.bodyB),
    anchorA: { x: c.la1X, y: c.la1Y, z: c.la1Z },
    anchorB: { x: c.la2X, y: c.la2Y, z: c.la2Z },
    basisA: [c.lbX1X, c.lbX1Y, c.lbX1Z, c.lbY1X, c.lbY1Y, c.lbY1Z, c.lbZ1X, c.lbZ1Y, c.lbZ1Z],
    basisB: [c.lbX2X, c.lbX2Y, c.lbX2Z, c.lbY2X, c.lbY2Y, c.lbY2Z, c.lbZ2X, c.lbZ2Y, c.lbZ2Z],
    allowCollision: c.allowCollision,
    breakForce: c.breakForce,
    breakTorque: c.breakTorque,
    solver: c.solver instanceof DirectJointConstraintSolver ? "direct" : "iterative",
    springs: p.springs.map((s) => ({ frequency: s.frequency, dampingRatio: s.dampingRatio, useSymplecticEuler: s.useSymplecticEuler })),
    linear: p.linear.map((l) => ({ lower: l.lowerLimit, upper: l.upperLimit, speed: l.motorSpeed, maximum: l.motorForce })),
    angular: p.angular.map((l) => ({ lower: l.lowerLimit, upper: l.upperLimit, speed: l.motorSpeed, maximum: l.motorTorque })),
    swing: c instanceof RagdollJoint ? [c.maxSwingAngle1, c.maxSwingAngle2] : []
  };
}
function restoreContents(w, s, o, hasTerrain) {
  const bodies = s.bodies.map((b, i) => w.createBody({ ...b.options, userData: b.data === void 0 ? void 0 : o.decodeUserData ? o.decodeUserData(b.data, i) : b.data }));
  const x = { x: 1, y: 0, z: 0 }, y = { x: 0, y: 1, z: 0 };
  for (const j of s.joints) {
    const common = { bodyA: bodies[j.bodyA], bodyB: bodies[j.bodyB], anchor: { x: 0, y: 0, z: 0 }, allowCollision: j.allowCollision, breakForce: j.breakForce, breakTorque: j.breakTorque, solver: j.solver };
    let options;
    switch (j.type) {
      case "ball":
      case "generic":
        options = { ...common, type: j.type };
        break;
      case "hinge":
      case "slider":
      case "cylinder":
        options = { ...common, type: j.type, axis: x };
        break;
      case "universal":
        options = { ...common, type: j.type, axisA: x, axisB: y };
        break;
      case "ragdoll":
        options = { ...common, type: j.type, twistAxis: x, swingAxis: y };
        break;
    }
    const c = w.addJoint(options).core, p = parts(c);
    c.la1X = j.anchorA.x;
    c.la1Y = j.anchorA.y;
    c.la1Z = j.anchorA.z;
    c.la2X = j.anchorB.x;
    c.la2Y = j.anchorB.y;
    c.la2Z = j.anchorB.z;
    [c.lbX1X, c.lbX1Y, c.lbX1Z, c.lbY1X, c.lbY1Y, c.lbY1Z, c.lbZ1X, c.lbZ1Y, c.lbZ1Z] = j.basisA;
    [c.lbX2X, c.lbX2Y, c.lbX2Z, c.lbY2X, c.lbY2Y, c.lbY2Z, c.lbZ2X, c.lbZ2Y, c.lbZ2Z] = j.basisB;
    p.springs.forEach((v, i) => {
      const spring = j.springs[i];
      v.setSpring(spring.frequency, spring.dampingRatio).setSymplecticEuler(spring.useSymplecticEuler);
    });
    p.linear.forEach((v, i) => {
      const l = j.linear[i];
      v.setLimits(l.lower, l.upper).setMotor(l.speed, l.maximum);
    });
    p.angular.forEach((v, i) => {
      const l = j.angular[i];
      v.setLimits(l.lower, l.upper).setMotor(l.speed, l.maximum);
    });
    if (c instanceof RagdollJoint) [c.maxSwingAngle1, c.maxSwingAngle2] = j.swing;
    c.syncAnchors();
  }
  bodies.forEach((b, i) => {
    const state2 = s.bodies[i], c = b.core;
    c.fx = state2.force.x;
    c.fy = state2.force.y;
    c.fz = state2.force.z;
    c.tx = state2.torque.x;
    c.ty = state2.torque.y;
    c.tz = state2.torque.z;
    if (state2.sleeping && !hasTerrain) b.sleep();
  });
}
function requireValue(ok, name) {
  if (!ok) throw new Error(`\u7269\u7406\u5B58\u6863\u7684 ${name} \u4E0D\u5408\u6CD5`);
}
function number(v, name, min = -Infinity, integer = false) {
  requireValue(typeof v === "number" && Number.isFinite(v) && v >= min && (!integer || Number.isSafeInteger(v)), name);
}
function boolean(v, name) {
  requireValue(typeof v === "boolean", name);
}
function vector(v, name) {
  requireValue(v !== null && typeof v === "object", name);
  number(v.x, name);
  number(v.y, name);
  number(v.z, name);
}
function quaternion(v) {
  vector(v, "rotation");
  number(v.w, "rotation.w");
  const length = Math.hypot(v.x, v.y, v.z, v.w);
  requireValue(length > 0 && Number.isFinite(length), "rotation");
}
function positive2(v, name) {
  number(v, name, 0);
  requireValue(v > 0, name);
}
function shapeOptions(shape) {
  requireValue(shape !== null && typeof shape === "object", "shape");
  if (shape.offset !== void 0) vector(shape.offset, "offset");
  if (shape.rotation !== void 0) quaternion(shape.rotation);
  for (const k of ["friction", "restitution"]) if (shape[k] !== void 0) number(shape[k], k, 0);
  if (shape.density !== void 0) positive2(shape.density, "density");
  for (const k of ["collisionGroup", "collisionMask"]) if (shape[k] !== void 0) {
    number(shape[k], k, -2147483648, true);
    requireValue(shape[k] <= 4294967295, k);
  }
  if (shape.block !== void 0) {
    const b = shape.block;
    requireValue(b !== null && typeof b === "object" && typeof b.typeId === "string", "block");
    vector(b.location, "block.location");
    requireValue(Number.isInteger(b.location.x) && Number.isInteger(b.location.y) && Number.isInteger(b.location.z), "block.location");
    if (b.states !== void 0) {
      requireValue(b.states !== null && typeof b.states === "object" && !Array.isArray(b.states), "block.states");
      for (const v of Object.values(b.states)) requireValue(typeof v === "boolean" || typeof v === "string" || typeof v === "number" && Number.isFinite(v), "block.states");
    }
  }
  switch (shape.type) {
    case "box":
      vector(shape.size, "size");
      positive2(shape.size.x, "size.x");
      positive2(shape.size.y, "size.y");
      positive2(shape.size.z, "size.z");
      break;
    case "sphere":
      positive2(shape.radius, "radius");
      break;
    case "capsule":
      positive2(shape.radius, "radius");
      positive2(shape.height, "height");
      requireValue(shape.height >= shape.radius * 2, "capsule.height");
      break;
    case "cylinder":
    case "cone":
      positive2(shape.radius, "radius");
      positive2(shape.height, "height");
      break;
    case "convexHull":
      array(shape.points, "points");
      requireValue(shape.points.length >= 4, "points");
      shape.points.forEach((p) => vector(p, "point"));
      break;
    default:
      throw new Error("\u7269\u7406\u5B58\u6863\u5305\u542B\u672A\u77E5\u5F62\u72B6");
  }
}
function array(v, name, length) {
  requireValue(Array.isArray(v) && (length === void 0 || v.length === length), name);
}
function basis(v, name, ball) {
  array(v, name, 9);
  v.forEach((n) => number(n, name));
  if (ball) return;
  for (let a = 0; a < 3; a++) for (let b = a; b < 3; b++) {
    const dot = v[a * 3] * v[b * 3] + v[a * 3 + 1] * v[b * 3 + 1] + v[a * 3 + 2] * v[b * 3 + 2];
    requireValue(Math.abs(dot - (a === b ? 1 : 0)) < 1e-5, name);
  }
  const det2 = v[0] * (v[4] * v[8] - v[5] * v[7]) - v[1] * (v[3] * v[8] - v[5] * v[6]) + v[2] * (v[3] * v[7] - v[4] * v[6]);
  requireValue(det2 > 0, name);
}
var FLUID_FIELDS = ["waterDensity", "lavaDensity", "drag", "linearDrag", "lavaViscosity", "waveDamping", "airPockets", "floodRate", "currentSpeed"];
function readSnapshot(input) {
  return validateSnapshot(copyJson(input));
}
function validateSnapshot(s) {
  requireValue(s !== null && s.format === "cutlass-physics" && s.schema === 1, "\u683C\u5F0F\u6216\u7248\u672C");
  requireValue(typeof s.dimensionId === "string" && s.dimensionId.length > 0, "\u7EF4\u5EA6");
  const t = s.settings;
  requireValue(t !== null && typeof t === "object", "settings");
  vector(t.gravity, "gravity");
  number(t.substeps, "substeps", 1, true);
  if (t.voidSea !== void 0 && t.voidSea !== false) {
    requireValue(t.voidSea !== null && typeof t.voidSea === "object", "voidSea");
    number(t.voidSea.startY, "voidSea.startY");
    number(t.voidSea.depthGradient, "voidSea.depthGradient");
    number(t.voidSea.drag, "voidSea.drag", 0);
  }
  if (t.fluids !== void 0 && t.fluids !== false) {
    requireValue(t.fluids !== null && typeof t.fluids === "object", "fluids");
    const f = t.fluids;
    for (const [k, v] of Object.entries(f)) {
      if (!FLUID_FIELDS.includes(k)) throw new Error(`\u7269\u7406\u5B58\u6863\u5305\u542B\u672A\u77E5\u6D41\u4F53\u9009\u9879 ${k}`);
      if (k === "airPockets") boolean(v, "fluids.airPockets");
      else number(v, `fluids.${k}`, 0);
    }
  }
  if (t.explosions !== void 0) boolean(t.explosions, "explosions");
  if (t.breakFragileBlocks !== void 0) boolean(t.breakFragileBlocks, "breakFragileBlocks");
  if (t.timeBudgetMs !== null) {
    number(t.timeBudgetMs, "timeBudgetMs", 0);
    requireValue(t.timeBudgetMs > 0, "timeBudgetMs");
  }
  requireValue(t.stepMode === "auto" || t.stepMode === "manual", "stepMode");
  boolean(t.paused, "paused");
  boolean(t.customCollision, "customCollision");
  if (t.terrain !== null) {
    requireValue(typeof t.terrain === "object", "terrain");
    for (const [k, v] of Object.entries(t.terrain)) {
      const check = Object.prototype.hasOwnProperty.call(TERRAIN_FIELDS, k) ? TERRAIN_FIELDS[k] : void 0;
      if (check === void 0) throw new Error(`\u7269\u7406\u5B58\u6863\u5305\u542B\u672A\u77E5\u5730\u5F62\u9009\u9879 ${k}`);
      check(v, k);
    }
  } else requireValue(!t.customCollision, "customCollision");
  if (t.blocks !== void 0) {
    try {
      new BlockRegistry(t.blocks);
    } catch (error) {
      const wrapped = new Error(`\u7269\u7406\u5B58\u6863\u7684 blocks \u4E0D\u5408\u6CD5\uFF1A${error.message}`);
      wrapped.cause = error;
      throw wrapped;
    }
  }
  array(s.bodies, "bodies");
  array(s.joints, "joints");
  for (const b of s.bodies) {
    requireValue(b !== null && typeof b === "object" && b.options !== null && typeof b.options === "object", "body");
    const o = b.options;
    requireValue(o.type === "dynamic" || o.type === "static" || o.type === "kinematic", "body.type");
    vector(o.location, "location");
    quaternion(o.rotation);
    vector(o.linearVelocity, "linearVelocity");
    vector(o.angularVelocity, "angularVelocity");
    number(o.linearDamping, "linearDamping", 0);
    number(o.angularDamping, "angularDamping", 0);
    number(o.gravityScale, "gravityScale");
    boolean(o.allowSleep, "allowSleep");
    boolean(b.sleeping, "sleeping");
    vector(b.force, "force");
    vector(b.torque, "torque");
    array(o.shapes, "shapes");
    requireValue(o.shapes.length > 0, "shapes");
    for (const shape of o.shapes) shapeOptions(shape);
  }
  const counts = { ball: [1, 0, 0, 0], hinge: [1, 0, 1, 0], slider: [1, 1, 0, 0], cylinder: [2, 1, 1, 0], universal: [2, 0, 2, 0], ragdoll: [2, 0, 1, 2], generic: [6, 3, 3, 0] };
  for (const j of s.joints) {
    requireValue(j !== null && typeof j === "object" && Object.prototype.hasOwnProperty.call(counts, j.type), "joint.type");
    number(j.bodyA, "bodyA", 0, true);
    number(j.bodyB, "bodyB", 0, true);
    requireValue(j.bodyA < s.bodies.length && j.bodyB < s.bodies.length && j.bodyA !== j.bodyB, "\u5173\u8282\u521A\u4F53\u5F15\u7528");
    vector(j.anchorA, "anchorA");
    vector(j.anchorB, "anchorB");
    basis(j.basisA, "basisA", j.type === "ball");
    basis(j.basisB, "basisB", j.type === "ball");
    boolean(j.allowCollision, "allowCollision");
    number(j.breakForce, "breakForce");
    number(j.breakTorque, "breakTorque");
    requireValue(j.solver === "iterative" || j.solver === "direct", "solver");
    const n = counts[j.type];
    array(j.springs, "springs", n[0]);
    array(j.linear, "linear", n[1]);
    array(j.angular, "angular", n[2]);
    array(j.swing, "swing", n[3]);
    for (const p of j.springs) {
      requireValue(p !== null && typeof p === "object", "spring");
      number(p.frequency, "frequency");
      number(p.dampingRatio, "dampingRatio");
      boolean(p.useSymplecticEuler, "useSymplecticEuler");
    }
    for (const l of [...j.linear, ...j.angular]) {
      requireValue(l !== null && typeof l === "object", "limit");
      number(l.lower, "lower");
      number(l.upper, "upper");
      number(l.speed, "speed");
      number(l.maximum, "maximum");
    }
    j.swing.forEach((v) => number(v, "swing"));
  }
  return s;
}
var TERRAIN_FIELDS = {
  margin: (v, k) => number(v, k, 0),
  maxChunksPerTick: (v, k) => number(v, k, 1, true),
  unloadDelayTicks: (v, k) => number(v, k, 0, true),
  revalidateBlocksPerTick: (v, k) => number(v, k, 0, true),
  preloadTicks: (v, k) => number(v, k, 0, true),
  watchBlockEvents: (v, k) => boolean(v, k),
  unknownBlock: (v, k) => requireValue(v === "fullCube" || v === "none", k),
  unloadedChunkPolicy: (v, k) => requireValue(v === "freeze" || v === "fall", k),
  material: (v, k) => {
    requireValue(v !== null && typeof v === "object", k);
    const m = v;
    number(m.friction, k, 0);
    number(m.restitution, k, 0);
  }
};
function checkCapturedBodies(s) {
  for (const b of s.bodies) {
    const o = b.options, l = o.location, r = o.rotation, v = o.linearVelocity, w = o.angularVelocity, f = b.force, t = b.torque;
    if (!(Number.isFinite(l.x + l.y + l.z) && Number.isFinite(r.x + r.y + r.z + r.w) && Number.isFinite(v.x + v.y + v.z) && Number.isFinite(w.x + w.y + w.z) && Number.isFinite(f.x + f.y + f.z) && Number.isFinite(t.x + t.y + t.z))) {
      throw new Error("\u7269\u7406\u6A21\u62DF\u51FA\u73B0\u4E86 NaN \u6216\u65E0\u7A77\u5927\uFF08\u6A21\u62DF\u53D1\u6563\uFF09\uFF0C\u62D2\u7EDD\u4FDD\u5B58\uFF0C\u5DF2\u6709\u7684\u5B58\u6863\u4FDD\u6301\u4E0D\u53D8");
    }
  }
  return s;
}

// src/mc/world.ts
var PhysicsWorld = class _PhysicsWorld {
  /**
   * 创建一个物理世界。
   *
   * @param dimension
   * 物理世界对应的维度。地形碰撞从该维度读取方块。
   * @param options
   * 物理世界的选项。
   *
   * @example create-world.ts
   * {@includeCode ../../examples/getting-started/create-world.ts}
   */
  constructor(dimension, options = {}) {
    this.core = new World();
    this.bodySet = /* @__PURE__ */ new Set();
    this.jointSet = /* @__PURE__ */ new Set();
    /** 上一次通知过的睡眠状态，用来只在变化时发事件 */
    this.sleepState = /* @__PURE__ */ new Map();
    /** 见 revision */
    this.revisionValue = 0;
    this.events = createAfterEvents();
    this.slowSteps = 0;
    this.fastSteps = 0;
    this.collider = null;
    this.source = null;
    this.unwatchBlocks = null;
    /** 排队的爆炸，下一次推进前施加（见 explosions.ts） */
    this.explosionQueue = new ExplosionQueue();
    this.occlusion = null;
    this.unwatchExplosions = null;
    /** 流体计算。没有地形或 fluids 为 false 时为 null */
    this.fluids = null;
    /** 每个刚体的流体状态。第一次碰到流体时才建 */
    this.fluidBodies = /* @__PURE__ */ new Map();
    /** 带悬浮方块的刚体和它们的方块簇（见 floating.ts） */
    this.floatingBodies = /* @__PURE__ */ new Map();
    /** 带易碎方块的刚体（见 onBeginContact） */
    this.fragileBodies = /* @__PURE__ */ new Set();
    this.pendingBreaks = /* @__PURE__ */ new Set();
    this.brokenShapes = /* @__PURE__ */ new Set();
    this.runId = -1;
    /** checkChunks 轮到第几个刚体 */
    this.chunkCursor = 0;
    this.validFlag = true;
    this.totalSteps = 0;
    this.tickCount = 0;
    this.stepModeValue = "manual";
    this.pausedValue = false;
    /** 步进过程中产生的事件，步进结束后再依次触发，避免订阅者在内核遍历接触时增删刚体 */
    this.pending = [];
    this.contactListener = {
      beginContact: (c) => this.onBeginContact(c),
      endContact: (c) => this.onEndContact(c)
    };
    if (dimension === void 0 || dimension === null) throw new Error("\u521B\u5EFA\u7269\u7406\u4E16\u754C\u9700\u8981\u4E00\u4E2A\u7EF4\u5EA6");
    this.dimension = dimension;
    const g = options.gravity ?? DEFAULT_GRAVITY;
    finite32("gravity", g);
    this.gravityValue = { x: g.x, y: g.y, z: g.z };
    this.core.gravityX = g.x;
    this.core.gravityY = g.y;
    this.core.gravityZ = g.z;
    const sea = options.voidSea === void 0 ? { startY: VOID_SEA_START_Y } : options.voidSea;
    if (sea !== false) {
      const depthGradient = sea.depthGradient ?? 1, drag = sea.drag ?? 1;
      if (!Number.isFinite(sea.startY)) throw new Error(`voidSea.startY \u5FC5\u987B\u662F\u6709\u9650\u6570\uFF0C\u6536\u5230 ${sea.startY}`);
      if (!Number.isFinite(depthGradient)) throw new Error(`voidSea.depthGradient \u5FC5\u987B\u662F\u6709\u9650\u6570\uFF0C\u6536\u5230 ${depthGradient}`);
      if (!(drag >= 0) || !Number.isFinite(drag)) throw new Error(`voidSea.drag \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 0 \u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${drag}`);
      this.core.voidSea = { startY: sea.startY, depthGradient, drag };
    }
    const substeps = options.substeps ?? DEFAULT_SUBSTEPS;
    if (!Number.isInteger(substeps) || substeps < 1) throw new Error(`substeps \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 1 \u7684\u6574\u6570\uFF0C\u6536\u5230 ${substeps}`);
    this.timeBudget = checkBudget(options.timeBudgetMs ?? DEFAULT_TIME_BUDGET_MS);
    this.substepCount = substeps;
    this.activeSubsteps = substeps;
    this.core.numVelocityIterations = SOLVER_VELOCITY_ITERATIONS;
    this.core.softContacts = true;
    this.core.softIterations = 5;
    this.core.softRelaxIterations = 5;
    this.core.softImpactIterations = 10;
    this.core.softDampingRatio = 7;
    this.core.softTolerance = 1e-4;
    this.core.softAdaptiveIterations = true;
    this.core.softSubsteps = substeps;
    this.core.pairSpeculativeSpeed = Infinity;
    this.core.softContactSkin = 0;
    this.core.contactManager.maxRestitution = true;
    this.core.contactManager.listener = this.contactListener;
    this.registry = options.blocks === void 0 ? null : new BlockRegistry(options.blocks);
    this.fragileMinSpeed = Math.min(FRAGILE_MIN_SPEED, this.registry?.minFragileSpeed ?? Infinity);
    const t = options.terrain;
    this.customCollision = t?.collision !== void 0;
    this.terrainSettings = t === void 0 ? null : Object.fromEntries(Object.entries(t).filter(([k, v]) => k !== "collision" && v !== void 0));
    if (t?.material !== void 0) this.terrainSettings.material = { ...t.material };
    this.freezeOnUnloaded = (t?.unloadedChunkPolicy ?? "freeze") === "freeze";
    if (t !== void 0) {
      if (t.margin !== void 0 && (!(t.margin >= 0) || !Number.isFinite(t.margin))) throw new Error(`margin \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 0 \u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${t.margin}`);
      checkInteger("preloadTicks", t.preloadTicks, 0);
      checkInteger("maxChunksPerTick", t.maxChunksPerTick, 1);
      checkInteger("unloadDelayTicks", t.unloadDelayTicks, 0);
      checkInteger("revalidateBlocksPerTick", t.revalidateBlocksPerTick, 0);
      const table = createBlockShapeTable();
      const uniform = t.material;
      const resolver = new BlockResolver(table, {
        registry: this.registry ?? void 0,
        collision: t.collision,
        unknownBlock: t.unknownBlock,
        fixedMaterial: uniform === void 0 ? void 0 : 0
      });
      this.source = new DimensionBlockSource(dimension, HOST, resolver);
      this.source.unloadedAsAir = !this.freezeOnUnloaded;
      this.collider = new VoxelCollider(this.core, this.source, {
        table,
        materials: uniform === void 0 ? (this.registry?.materials ?? DEFAULT_MATERIALS).map((m) => ({ friction: terrainFriction(m), restitution: m.restitution })) : [{ friction: uniform.friction, restitution: uniform.restitution }],
        margin: t.margin,
        preloadTicks: t.preloadTicks,
        maxChunksPerUpdate: t.maxChunksPerTick,
        unloadDelay: t.unloadDelayTicks,
        revalidateBlocksPerUpdate: t.revalidateBlocksPerTick,
        suspend: this.freezeOnUnloaded,
        fluids: options.fluids !== false
      });
      if (t.watchBlockEvents ?? true) {
        const collider = this.collider;
        this.unwatchBlocks = watchBlockEvents(dimension.id, (x0, y0, z0, x1, y1, z1, cachedOnly) => collider.blocksChanged(x0, y0, z0, x1, y1, z1, cachedOnly));
      }
    }
    if (options.explosions ?? true) {
      this.occlusion = worldOcclusion(dimension);
      this.unwatchExplosions = watchExplosions(world2.afterEvents.explosion, dimension.id, this.explosionQueue);
    }
    if (options.fluids !== void 0 && options.fluids !== false) checkFluids(options.fluids);
    if (this.collider !== null && options.fluids !== false) this.fluids = new FluidForces({ waterDensity: DEFAULT_WATER_DENSITY, ...options.fluids });
    if (options.breakFragileBlocks !== void 0 && typeof options.breakFragileBlocks !== "boolean") {
      throw new Error(`breakFragileBlocks \u5FC5\u987B\u662F\u5E03\u5C14\u503C\uFF0C\u6536\u5230 ${String(options.breakFragileBlocks)}`);
    }
    this.breakFragile = options.breakFragileBlocks ?? true;
    this.stepMode = options.stepMode ?? "auto";
  }
  check() {
    if (!this.validFlag) throw new Error("\u7269\u7406\u4E16\u754C\u5DF2\u7ECF\u9500\u6BC1");
  }
  /** 方块的物理属性：内置数据，登记表覆盖 */
  blockPhysics(typeId, states) {
    return this.registry === null ? getBlockPhysics(typeId, states) : registeredPhysics(this.registry, typeId, states);
  }
  /**
   * 生成独立的刚体和关节快照，默认省略 userData。
   *
   * @remarks
   *
   * - 不包含地形缓存、事件订阅和求解缓存。
   * - 冻结状态保留速度，恢复时重新请求地形。
   * - 世界动态属性存储见 {@link PhysicsWorldStore}。
   *
   * @param options
   * 用 `encodeUserData` 把每个刚体的 userData 转成可存的 JSON 值；省略时不保存 userData。
   * @returns
   * 可以直接 `JSON.stringify` 的普通对象，和世界不共享任何对象。
   * @throws
   * 模拟发散（出现 NaN、无穷大）时，或 `encodeUserData` 返回了 JSON 不能表示的值时。
   */
  createSnapshot(options = {}) {
    return this.captureSnapshot(options);
  }
  /**
   * 可能影响存档的状态变更计数，供自动保存判断是否跳过捕获。
   * @internal
   */
  get revision() {
    return this.revisionValue;
  }
  /** @internal Body、Joint 改了会进存档的状态时调用 */
  markChanged() {
    this.revisionValue++;
  }
  /** @internal 持久化复用不变形状；仅供存储器使用，不能向外暴露共享描述。 */
  captureSnapshot(options, cache2) {
    this.check();
    const bodies = [...this.bodySet], ids = new Map(bodies.map((b, i) => [b, i]));
    return checkCapturedBodies({
      format: "cutlass-physics",
      schema: 1,
      dimensionId: this.dimension.id,
      settings: {
        gravity: this.gravity,
        voidSea: this.core.voidSea === null ? false : { ...this.core.voidSea },
        fluids: this.fluids === null ? false : { waterDensity: this.fluids.waterDensity, lavaDensity: this.fluids.lavaDensity, drag: this.fluids.drag, linearDrag: this.fluids.linearDrag, lavaViscosity: this.fluids.lavaViscosity, waveDamping: this.fluids.waveDamping, airPockets: this.fluids.airPockets, floodRate: this.fluids.floodRate, currentSpeed: this.fluids.currentSpeed },
        explosions: this.occlusion !== null,
        breakFragileBlocks: this.breakFragile,
        substeps: this.substepCount,
        timeBudgetMs: this.timeBudget === Infinity ? null : this.timeBudget,
        stepMode: this.stepModeValue,
        paused: this.pausedValue,
        terrain: copyJson(this.terrainSettings),
        customCollision: this.customCollision,
        ...this.registry === null ? {} : { blocks: this.registry.toJSON() }
      },
      bodies: bodies.map((b) => captureBody(b, options, cache2)),
      joints: [...this.jointSet].filter((j) => j.core.world !== null).map((j) => captureJoint(j, ids))
    });
  }
  /**
   * 校验快照并恢复为新世界。
   *
   * @remarks
   *
   * - 失败时释放新建资源。
   * - 地形重新读取，原休眠刚体在确认支撑后可再次休眠。
   * - 事件订阅不包含在快照中。
   *
   * @param dimension 恢复到的维度，必须与保存时相同。
   * @param input {@link PhysicsWorld.createSnapshot} 的结果或经过 JSON 往返后的对象。
   * @param options 覆盖已保存的世界设置。
   *
   * - 原世界使用 `terrain.collision` 时须重新提供。
   * - `decodeUserData` 用于恢复自定义数据。
   *
   * @returns 新的物理世界，推进方式与保存时一致。
   * @throws 快照格式或版本无效、数据损坏、维度不符或缺少 terrain.collision 时抛出错误。
   */
  static fromSnapshot(dimension, input, options = {}) {
    const snapshot = readSnapshot(input), s = snapshot.settings;
    if (snapshot.dimensionId !== dimension.id) throw new Error("\u7269\u7406\u5B58\u6863\u7684\u7EF4\u5EA6\u4E0E\u76EE\u6807\u7EF4\u5EA6\u4E0D\u540C");
    if (s.customCollision && options.terrain?.collision === void 0) throw new Error("\u6062\u590D\u6B64\u5B58\u6863\u9700\u8981\u91CD\u65B0\u63D0\u4F9B terrain.collision");
    const terrain = options.terrain === void 0 ? s.terrain ?? void 0 : { ...s.terrain, ...options.terrain };
    const w = new _PhysicsWorld(dimension, {
      gravity: options.gravity ?? s.gravity,
      voidSea: options.voidSea ?? s.voidSea,
      fluids: options.fluids ?? s.fluids,
      substeps: options.substeps ?? s.substeps,
      explosions: options.explosions ?? s.explosions,
      breakFragileBlocks: options.breakFragileBlocks ?? s.breakFragileBlocks,
      timeBudgetMs: options.timeBudgetMs ?? s.timeBudgetMs ?? Infinity,
      terrain,
      stepMode: "manual",
      blocks: options.blocks ?? s.blocks
    });
    try {
      restoreContents(w, snapshot, options, terrain !== void 0);
      for (const b of w.bodySet) if (b.core.sleeping) w.sleepState.set(b, true);
      w.paused = s.paused;
      w.stepMode = options.stepMode ?? s.stepMode;
      return w;
    } catch (error) {
      w.destroy();
      throw error;
    }
  }
  /**
   * 物理世界是否有效。
   *
   * @remarks
   * 调用 {@link PhysicsWorld.destroy} 后为 `false`，此后访问该世界会抛出错误。
   */
  get isValid() {
    return this.validFlag;
  }
  /**
   * 物理世界发出的事件。
   */
  get afterEvents() {
    return this.events;
  }
  /**
   * 世界的重力，单位为格/秒²。
   *
   * @remarks
   * 设置该值会唤醒所有刚体。
   */
  get gravity() {
    this.check();
    return { x: this.gravityValue.x, y: this.gravityValue.y, z: this.gravityValue.z };
  }
  set gravity(value) {
    this.check();
    finite32("gravity", value);
    this.gravityValue = { x: value.x, y: value.y, z: value.z };
    this.core.gravityX = value.x;
    this.core.gravityY = value.y;
    this.core.gravityZ = value.z;
    for (const b of this.bodySet) b.core.wakeUp();
    this.revisionValue++;
  }
  /**
   * 世界的时间推进方式。
   *
   * @remarks
   * 设为 `"manual"` 会停止自动推进。
   */
  get stepMode() {
    this.check();
    return this.stepModeValue;
  }
  set stepMode(value) {
    this.check();
    if (value !== "auto" && value !== "manual") throw new Error(`stepMode \u5FC5\u987B\u662F "auto" \u6216 "manual"\uFF0C\u6536\u5230 ${String(value)}`);
    if (value === this.stepModeValue) return;
    this.stepModeValue = value;
    this.revisionValue++;
    if (value === "auto") {
      this.runId = system2.runInterval(() => {
        if (this.validFlag && !this.pausedValue) this.step(TICK_SECONDS);
      }, 1);
    } else {
      system2.clearRun(this.runId);
      this.runId = -1;
    }
  }
  /**
   * 每次推进拆分的子步数。
   */
  get substeps() {
    this.check();
    return this.substepCount;
  }
  set substeps(value) {
    this.check();
    if (!Number.isInteger(value) || value < 1) throw new Error(`substeps \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 1 \u7684\u6574\u6570\uFF0C\u6536\u5230 ${value}`);
    this.substepCount = value;
    this.activeSubsteps = value;
    this.slowSteps = 0;
    this.fastSteps = 0;
    this.revisionValue++;
  }
  /**
   * 每次推进的目标求解耗时，单位为毫秒，用来增减子步。详见 {@link PhysicsWorldOptions.timeBudgetMs}。
   */
  get timeBudgetMs() {
    this.check();
    return this.timeBudget;
  }
  set timeBudgetMs(value) {
    this.check();
    this.timeBudget = checkBudget(value);
    this.revisionValue++;
  }
  /**
   * 世界中有效刚体的数量。
   */
  get bodyCount() {
    this.check();
    return this.bodySet.size;
  }
  /**
   * 世界中有效关节的数量。
   */
  get jointCount() {
    this.check();
    return this.jointSet.size;
  }
  /**
   * 世界中未休眠的刚体数量。
   */
  get awakeBodyCount() {
    this.check();
    let n = 0;
    for (const b of this.bodySet) if (b.core.type !== RigidBodyType.STATIC && (!b.core.sleeping || this.isWaiting(b))) n++;
    return n;
  }
  /**
   * 世界是否处于暂停状态。
   *
   * @remarks
   * 暂停时不会自动推进，但仍可以调用 {@link PhysicsWorld.step}。
   */
  get paused() {
    this.check();
    return this.pausedValue;
  }
  set paused(value) {
    this.check();
    this.pausedValue = !!value;
    this.revisionValue++;
  }
  /**
   * 推进物理世界。
   *
   * @remarks
   * 在 auto 模式下调用会额外推进一步。
   *
   * @param deltaTime
   * 推进的时间，单位为秒，必须大于 `0`，由各个子步平均分摊。默认值为 `1 / 20`，即一个 tick。
   * @returns
   * 这次推进中各活动组合使用的最大子步数。预算规则见 {@link PhysicsWorldOptions.substeps}。
   * @throws
   * `deltaTime` 不合法时；推进完成后有事件回调抛出错误时，抛出第一个错误（见 {@link Signal}）。
   *
   * @example manual-step.ts
   * {@includeCode ../../examples/getting-started/manual-step.ts}
   */
  step(deltaTime = TICK_SECONDS) {
    this.check();
    if (!(deltaTime > 0) || !Number.isFinite(deltaTime)) throw new Error(`deltaTime \u5FC5\u987B\u662F\u5927\u4E8E\u96F6\u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${deltaTime}`);
    const start = now();
    this.tickCount++;
    this.brokenShapes.clear();
    if (this.occlusion !== null && this.explosionQueue.size > 0) this.applyExplosions(this.occlusion);
    if (this.collider !== null && this.freezeOnUnloaded) this.checkChunks(this.collider);
    this.collider?.update(deltaTime);
    const solveStart = this.collider === null ? start : now();
    if (this.floatingBodies.size > 0) this.applyFloating();
    if (this.fluids !== null) this.applyFluids(this.fluids, this.collider, deltaTime);
    for (const b of this.bodySet) b.recordPrevious();
    this.core.softSubsteps = this.activeSubsteps;
    this.core.softSupportSubsteps = Math.min(this.substepCount, DEFAULT_SUBSTEPS);
    this.core.step(deltaTime);
    const done = this.core.lastMaxSubsteps;
    if (this.core.numIslands > 0) this.revisionValue++;
    this.totalSteps += done;
    this.detectBreaks();
    this.detectSleep();
    const end = now();
    this.adjustSubsteps(end - solveStart);
    try {
      this.flush();
    } finally {
      this.events.stepEnd.emit({ deltaTime, stepCount: this.totalSteps, elapsedMs: end - start });
    }
    return done;
  }
  /** 对活动且地形就绪的动态刚体施加悬浮升力和阻力。 */
  applyFloating() {
    const gx = this.core.gravityX, gy = this.core.gravityY, gz = this.core.gravityZ;
    for (const [b, clusters] of this.floatingBodies) {
      const rb = b.core;
      if (rb.type !== RigidBodyType.DYNAMIC || rb.sleeping || this.isWaiting(b)) continue;
      applyFloating(rb, clusters, gx, gy, gz);
    }
  }
  /** 计算活动且地形就绪的动态刚体的流体作用。受力不平衡且无接触的刚体保持活动。 */
  applyFluids(fluids, collider, dt) {
    if (!collider.hasAnyFluid()) {
      for (const [b, r] of this.fluidBodies) {
        b.core.restless = false;
        r.body.submerged = 0;
        r.inWater = false;
        r.inLava = false;
      }
      return;
    }
    const gx = this.core.gravityX, gy = this.core.gravityY, gz = this.core.gravityZ;
    const box = fluidBounds;
    for (const b of this.bodySet) {
      const rb = b.core;
      rb.restless = false;
      if (rb.type !== RigidBodyType.DYNAMIC || rb.sleeping || this.isWaiting(b)) continue;
      if (!bodyBounds(rb, box)) continue;
      let record = this.fluidBodies.get(b);
      if (!collider.hasFluidNear(Math.floor(box[0]), Math.floor(box[1]) - 1, Math.floor(box[2]), Math.floor(box[3]), Math.floor(box[4]), Math.floor(box[5])) && (record === void 0 || fluids.fluidInside(record.body) <= 0)) {
        if (record !== void 0) {
          record.body.submerged = 0;
          record.inWater = false;
          record.inLava = false;
        }
        continue;
      }
      if (record === void 0) {
        record = { body: newFluidBody(), inWater: false, inLava: false, lastLava: -Infinity };
        this.fluidBodies.set(b, record);
      }
      const body = record.body;
      body.grid.update(rb);
      const t = rb.transform;
      body.px = t.px;
      body.py = t.py;
      body.pz = t.pz;
      body.r00 = t.r00;
      body.r01 = t.r01;
      body.r02 = t.r02;
      body.r10 = t.r10;
      body.r11 = t.r11;
      body.r12 = t.r12;
      body.r20 = t.r20;
      body.r21 = t.r21;
      body.r22 = t.r22;
      const q = matrixToQuaternion(t.r00, t.r01, t.r02, t.r10, t.r11, t.r12, t.r20, t.r21, t.r22);
      body.qx = q.x;
      body.qy = q.y;
      body.qz = q.z;
      body.qw = q.w;
      body.vx = rb.vx;
      body.vy = rb.vy;
      body.vz = rb.vz;
      body.wx = rb.wx;
      body.wy = rb.wy;
      body.wz = rb.wz;
      body.mass = rb.mass;
      body.invMass = rb.invMass;
      body.ii00 = rb.ii00;
      body.ii11 = rb.ii11;
      body.ii22 = rb.ii22;
      body.gravityScale = rb.gravityScale;
      body.minX = box[0];
      body.minY = box[1];
      body.minZ = box[2];
      body.maxX = box[3];
      body.maxY = box[4];
      body.maxZ = box[5];
      body.fx = rb.fx;
      body.fy = rb.fy;
      body.fz = rb.fz;
      body.tx = rb.tx;
      body.ty = rb.ty;
      body.tz = rb.tz;
      const acted = fluids.apply(body, collider, gx, gy, gz, dt);
      rb.fx = body.fx;
      rb.fy = body.fy;
      rb.fz = body.fz;
      rb.tx = body.tx;
      rb.ty = body.ty;
      rb.tz = body.tz;
      if (acted && fluids.lastUnbalance > FLUID_RESTLESS) rb.restless = true;
      collider.setFluidReach(rb, fluids.airPockets && fluids.hasPockets(body));
      this.detectFluidEntry(b, record, fluids, collider);
    }
  }
  /**
   * 入水事件：上一次处理时不在水里、这次在，就发 waterEntry；岩浆要离开满 4 个 tick 才能再发 lavaEntry。
   * 没人订阅时只记状态，不收集接触点
   */
  detectFluidEntry(b, record, fluids, terrain) {
    const kinds = fluids.lastKinds;
    const inWater = (kinds & 1 << FLUID_WATER - 1) !== 0, inLava = (kinds & 1 << FLUID_LAVA - 1) !== 0;
    const water = inWater && !record.inWater, lava = inLava && !record.inLava && this.tickCount - record.lastLava > LAVA_ENTRY_REARM;
    record.inWater = inWater;
    record.inLava = inLava;
    if (inLava) record.lastLava = this.tickCount;
    if (water && this.events.waterEntry.hasSubscribers) this.queueFluidEntry(this.events.waterEntry, b, record.body, fluids, terrain, FLUID_WATER);
    if (lava && this.events.lavaEntry.hasSubscribers) this.queueFluidEntry(this.events.lavaEntry, b, record.body, fluids, terrain, FLUID_LAVA);
  }
  queueFluidEntry(signal, b, body, fluids, terrain, kind) {
    const e = fluidEntry;
    if (!fluids.collectEntry(body, terrain, kind, e)) return;
    const event = {
      body: b,
      fastestContactVelocityY: e.fastestVelocityY,
      point: { x: e.pointX, y: e.pointY, z: e.pointZ },
      minX: e.minX,
      maxX: e.maxX,
      minZ: e.minZ,
      maxZ: e.maxZ,
      bodySizeX: body.maxX - body.minX,
      bodySizeZ: body.maxZ - body.minZ
    };
    this.pending.push(() => signal.emit(event));
  }
  /** 施加已排队的爆炸冲量，跳过等待地形的刚体。 */
  applyExplosions(occlusion) {
    const cores = [];
    for (const b of this.bodySet) if (!this.isWaiting(b)) cores.push(b.core);
    this.explosionQueue.apply(cores, occlusion);
  }
  /**
   * 刚体最近的浸没体积，单位为格³。休眠期间保留最后一次计算结果。
   * @internal
   */
  submergedVolume(b) {
    return this.fluidBodies.get(b)?.body.submerged ?? 0;
  }
  /** @internal 刚体是否在等地形（见 TerrainOptions.unloadedChunkPolicy） */
  isWaiting(b) {
    return this.collider !== null && this.collider.isWaiting(b.core);
  }
  /** 轮询动态刚体所在区块。未加载时停用并释放地形请求，加载后恢复。 */
  checkChunks(collider) {
    const n = this.bodySet.size;
    if (n === 0 || typeof this.dimension.isChunkLoaded !== "function") return;
    const count = Math.min(CHUNK_CHECKS_PER_TICK, n);
    const first = this.chunkCursor % n;
    let i = 0, checked = 0;
    for (const b of this.bodySet) {
      const k = (i++ - first + n) % n;
      if (k >= count) continue;
      checked++;
      if (b.core.type !== RigidBodyType.DYNAMIC) continue;
      const t = b.core.transform;
      let loaded;
      try {
        loaded = this.dimension.isChunkLoaded({ x: t.px, y: t.py, z: t.pz });
      } catch {
        loaded = false;
      }
      if (loaded === collider.isParked(b.core)) {
        collider.setParked(b.core, !loaded);
        this.revisionValue++;
      }
      if (checked === count) break;
    }
    this.chunkCursor = first + count;
  }
  /** 连续 3 次求解超出预算就减 1 个子步，最少 2 个；连续 100 次不到预算的三分之一就加 1 个，最多回到 substeps */
  adjustSubsteps(ms) {
    const budget = this.timeBudget;
    if (ms > budget) {
      this.fastSteps = 0;
      if (++this.slowSteps >= 3 && this.activeSubsteps > MIN_SUBSTEPS) {
        this.activeSubsteps--;
        this.slowSteps = 0;
      }
    } else {
      this.slowSteps = 0;
      if (ms * 3 < budget && ++this.fastSteps >= 100 && this.activeSubsteps < this.substepCount) {
        this.activeSubsteps++;
        this.fastSteps = 0;
      }
    }
  }
  /** 内核在关节受力超过阈值时自己把关节摘掉；这里把对应的包装对象失效，并发出 jointBreak */
  detectBreaks() {
    if (this.jointSet.size === 0) return;
    let broken = null;
    for (const j of this.jointSet) {
      if (j.core.world === null) (broken ?? (broken = [])).push(j);
    }
    if (broken === null) return;
    const signal = this.events.jointBreak;
    for (const j of broken) {
      this.jointSet.delete(j);
      j.invalidate();
      if (signal.hasSubscribers) this.pending.push(() => signal.emit({ joint: j }));
    }
  }
  /** 睡眠状态的变化转成事件。等地形的刚体不算休眠 */
  detectSleep() {
    for (const b of this.bodySet) {
      if (this.isWaiting(b)) continue;
      const asleep = b.core.sleeping;
      const was = this.sleepState.get(b) ?? false;
      if (asleep === was) continue;
      this.sleepState.set(b, asleep);
      const signal = asleep ? this.events.bodySleep : this.events.bodyWake;
      if (signal.hasSubscribers) this.pending.push(() => signal.emit({ body: b }));
    }
  }
  /** 把步进过程中攒下的事件依次触发 */
  flush() {
    const list = this.pending;
    if (list.length === 0) return;
    this.pending = [];
    let failed = false;
    let firstError;
    for (const fn of list) {
      try {
        fn();
      } catch (e) {
        if (!failed) {
          failed = true;
          firstError = e;
        }
      }
    }
    if (failed) throw firstError;
  }
  /**
   * 创建一个刚体。
   *
   * @param options 刚体选项。
   * @returns 新创建的刚体。
   * @throws 选项无效时抛出错误，创建过程不保留部分结果。
   *
   * - 形状列表须非空，类型、尺寸、密度及 `block` 数据须有效。
   * - 位置、速度和 `gravityScale` 须为有限数，阻尼须非负。
   * - 形状错误包含该形状在 `shapes` 中的下标。
   *
   * @example drop-a-crate.ts
   * {@includeCode ../../examples/getting-started/drop-a-crate.ts}
   *
   * @example create-body-error.ts
   * {@includeCode ../../examples/world/create-body-error.ts}
   */
  createBody(options) {
    this.check();
    if (!Array.isArray(options.shapes) || options.shapes.length === 0) throw new Error("\u521A\u4F53\u81F3\u5C11\u9700\u8981\u4E00\u4E2A\u5F62\u72B6");
    const type = options.type ?? "dynamic";
    const t = type === "static" ? RigidBodyType.STATIC : type === "kinematic" ? RigidBodyType.KINEMATIC : type === "dynamic" ? RigidBodyType.DYNAMIC : -1;
    if (t < 0) throw new Error(`\u672A\u77E5\u7684\u521A\u4F53\u7C7B\u578B\uFF1A${String(type)}`);
    const config = new RigidBodyConfig();
    config.type = t;
    const loc = options.location;
    if (loc !== void 0) {
      finite32("location", loc);
      config.transform.px = loc.x;
      config.transform.py = loc.y;
      config.transform.pz = loc.z;
    }
    if (options.rotation !== void 0) {
      const m = quaternionToMatrix(options.rotation);
      const tf = config.transform;
      tf.r00 = m[0];
      tf.r01 = m[1];
      tf.r02 = m[2];
      tf.r10 = m[3];
      tf.r11 = m[4];
      tf.r12 = m[5];
      tf.r20 = m[6];
      tf.r21 = m[7];
      tf.r22 = m[8];
    }
    if (options.linearVelocity !== void 0) {
      finite32("linearVelocity", options.linearVelocity);
      config.vx = options.linearVelocity.x;
      config.vy = options.linearVelocity.y;
      config.vz = options.linearVelocity.z;
    }
    if (options.angularVelocity !== void 0) {
      finite32("angularVelocity", options.angularVelocity);
      config.wx = options.angularVelocity.x;
      config.wy = options.angularVelocity.y;
      config.wz = options.angularVelocity.z;
    }
    config.linearDamping = options.linearDamping === void 0 ? DEFAULT_DAMPING : nonNegative2("linearDamping", options.linearDamping);
    config.angularDamping = options.angularDamping === void 0 ? DEFAULT_DAMPING : nonNegative2("angularDamping", options.angularDamping);
    config.autoSleep = options.allowSleep ?? true;
    config.sleepingVelocityThreshold = SLEEP_LINEAR;
    config.sleepingAngularVelocityThreshold = SLEEP_ANGULAR;
    config.sleepingTimeThreshold = SLEEP_SECONDS;
    const shapes = options.shapes.map((o, i) => {
      try {
        return makeShape(withBodyFriction(o));
      } catch (e) {
        const error = new Error(`shapes[${i}]\uFF1A${e.message}`);
        error.cause = e;
        throw error;
      }
    });
    const core = new RigidBody(config);
    let floatingBlocks = null;
    let fragile = false;
    for (let i = 0; i < shapes.length; i++) {
      const given = options.shapes[i].block;
      if (given === void 0) continue;
      checkBodyBlock(given, i);
      const block = given.states === void 0 ? { location: { x: given.location.x, y: given.location.y, z: given.location.z }, typeId: given.typeId } : { location: { x: given.location.x, y: given.location.y, z: given.location.z }, typeId: given.typeId, states: { ...given.states } };
      const physics = this.blockPhysics(block.typeId, block.states ?? {});
      shapes[i].userData = { block, physics, index: i };
      if (physics.fragileImpactSpeed !== void 0) fragile = true;
      if (physics.floatingMaterial === void 0) continue;
      const s = shapes[i], t2 = s.localTransform, v = s.geom.volume, key2 = `${block.location.x},${block.location.y},${block.location.z}`;
      floatingBlocks ?? (floatingBlocks = /* @__PURE__ */ new Map());
      const e = floatingBlocks.get(key2);
      if (e === void 0) floatingBlocks.set(key2, { x: t2.px * v, y: t2.py * v, z: t2.pz * v, w: v, physics });
      else {
        e.x += t2.px * v;
        e.y += t2.py * v;
        e.z += t2.pz * v;
        e.w += v;
      }
    }
    const floating = [];
    if (floatingBlocks !== null) {
      for (const e of floatingBlocks.values()) {
        floating.push({ x: e.x / e.w, y: e.y / e.w, z: e.z / e.w, material: e.physics.floatingMaterial, scale: e.physics.floatingScale ?? 1 });
      }
    }
    if (options.gravityScale !== void 0) {
      if (!Number.isFinite(options.gravityScale)) throw new Error(`gravityScale \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 ${options.gravityScale}`);
      core.gravityScale = options.gravityScale;
    }
    const runtimeShapes = mergeBlockShapes(shapes);
    core.addShapes(runtimeShapes);
    const body = new Body(this, core, options.userData);
    if (runtimeShapes !== shapes) body.snapshotShapes = shapes;
    body.recordPrevious();
    this.core.addRigidBody(core);
    this.bodySet.add(body);
    if (floating.length > 0) this.floatingBodies.set(body, floatingClusters(floating));
    if (fragile) this.fragileBodies.add(body);
    this.revisionValue++;
    return body;
  }
  /** 内部使用：Body.remove 调用
   * @internal */
  removeBody(body) {
    if (!this.bodySet.has(body)) return;
    for (const j of [...this.jointSet]) if (j.bodyA === body || j.bodyB === body) this.removeJoint(j);
    this.bodySet.delete(body);
    this.sleepState.delete(body);
    this.fluidBodies.delete(body);
    this.floatingBodies.delete(body);
    this.fragileBodies.delete(body);
    if (body.core.world !== null) this.core.removeRigidBody(body.core);
    body.invalidate();
    this.revisionValue++;
  }
  /**
   * 创建一个关节，连接两个刚体。
   *
   * @param options
   * 关节的选项。
   * @returns
   * 新创建的关节。
   * @throws
   * 两个刚体不在同一个世界，或两者均不是动态刚体时抛出错误。
   */
  createJoint(options) {
    this.check();
    this.checkJointBodies(options);
    if (options.bodyA.core.type !== RigidBodyType.DYNAMIC && options.bodyB.core.type !== RigidBodyType.DYNAMIC) {
      throw new Error("\u5173\u8282\u8FDE\u63A5\u7684\u4E24\u4E2A\u521A\u4F53\u81F3\u5C11\u8981\u6709\u4E00\u4E2A\u662F\u52A8\u6001\u521A\u4F53");
    }
    return this.addJoint(options);
  }
  checkJointBodies(options) {
    const a = options.bodyA, b = options.bodyB;
    if (!(this.bodySet.has(a) && this.bodySet.has(b))) throw new Error("\u5173\u8282\u8FDE\u63A5\u7684\u4E24\u4E2A\u521A\u4F53\u90FD\u5FC5\u987B\u5728\u8FD9\u4E2A\u4E16\u754C\u91CC\uFF0C\u5E76\u4E14\u6709\u6548");
    if (a === b) throw new Error("\u5173\u8282\u4E0D\u80FD\u628A\u4E00\u4E2A\u521A\u4F53\u8FDE\u5230\u5B83\u81EA\u5DF1");
  }
  /**
   * 恢复关节，允许两端均为非动态刚体。
   * @internal
   */
  addJoint(options) {
    this.check();
    this.checkJointBodies(options);
    const a = options.bodyA, b = options.bodyB;
    const coreJoint = makeJoint(options, a.core, b.core);
    const joint = new Joint2(this, coreJoint, a, b, options.type);
    this.core.addJoint(coreJoint);
    this.jointSet.add(joint);
    this.revisionValue++;
    return joint;
  }
  /** 内部使用：Joint.remove 调用
   * @internal */
  removeJoint(joint) {
    if (!this.jointSet.has(joint)) return;
    this.jointSet.delete(joint);
    if (joint.core.world !== null) this.core.removeJoint(joint.core);
    joint.invalidate();
    this.revisionValue++;
  }
  /**
   * 遍历世界中所有有效的刚体。
   *
   * @remarks
   * 与 {@link PhysicsWorld.getBodies} 相比，不会创建数组。在回调中移除当前刚体是安全的。
   *
   * @param callback
   * 对每个刚体调用的函数。
   *
   * @example for-each-body.ts
   * {@includeCode ../../examples/display-sync/for-each-body.ts}
   */
  forEachBody(callback) {
    this.check();
    for (const b of this.bodySet) callback(b);
  }
  /**
   * 返回世界中所有有效刚体组成的新数组。
   */
  getBodies() {
    this.check();
    return [...this.bodySet];
  }
  /**
   * 执行射线检测，返回最近的命中。
   *
   * @param origin
   * 射线的起点。
   * @param direction
   * 射线的方向。无需归一化，不能为零向量。
   * @param options
   * 检测选项。
   * @returns
   * 最近的命中结果。没有命中时为 `undefined`。
   *
   * @example ray-cast.ts
   * {@includeCode ../../examples/world/ray-cast.ts}
   */
  rayCast(origin2, direction, options) {
    this.check();
    const hits = this.rayCastAll(origin2, direction, options);
    return hits[0];
  }
  /**
   * 执行射线检测，返回路径上的所有命中。
   *
   * @remarks
   * 结果按距离从近到远排列。
   *
   * @param origin
   * 射线的起点。
   * @param direction
   * 射线的方向。无需归一化，不能为零向量。
   * @param options
   * 检测选项。
   * @returns
   * 所有命中结果。
   */
  rayCastAll(origin2, direction, options) {
    this.check();
    finite32("origin", origin2);
    finite32("direction", direction);
    const len = Math.hypot(direction.x, direction.y, direction.z);
    if (!(len > 0)) throw new Error("direction \u4E0D\u80FD\u662F\u96F6\u5411\u91CF");
    const max = options?.maxDistance ?? 100;
    if (!(max > 0) || !Number.isFinite(max)) throw new Error(`maxDistance \u5FC5\u987B\u662F\u5927\u4E8E\u96F6\u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${max}`);
    const dx = direction.x / len, dy = direction.y / len, dz = direction.z / len;
    const mask = options?.collisionMask;
    const exclude = options?.excludeBodies;
    const includeBlocks = options?.includeBlocks ?? true, includeBodies = options?.includeBodies ?? true;
    const out = [];
    const callback = {
      process: (shape, hit) => {
        const rb = shape.rigidBody;
        const owner = rb.userData instanceof Body ? rb.userData : void 0;
        if (owner === void 0 && !includeBlocks) return;
        if (owner !== void 0 && !includeBodies) return;
        if (owner !== void 0 && exclude !== void 0 && exclude.includes(owner)) return;
        if (mask !== void 0 && (shape.collisionGroup & mask) === 0) return;
        const result = {
          point: { x: hit.positionX, y: hit.positionY, z: hit.positionZ },
          normal: { x: hit.normalX, y: hit.normalY, z: hit.normalZ },
          distance: hit.fraction * max
        };
        if (owner !== void 0) {
          result.body = owner;
        } else {
          result.blockLocation = {
            x: Math.floor(hit.positionX - hit.normalX * 1e-4),
            y: Math.floor(hit.positionY - hit.normalY * 1e-4),
            z: Math.floor(hit.positionZ - hit.normalZ * 1e-4)
          };
        }
        out.push(result);
      }
    };
    this.core.rayCast(origin2.x, origin2.y, origin2.z, origin2.x + dx * max, origin2.y + dy * max, origin2.z + dz * max, callback);
    out.sort((p, q) => p.distance - q.distance);
    return out;
  }
  /**
   * 返回包围盒与指定范围重叠的刚体。
   *
   * @remarks
   * 仅比较包围盒，因此结果是近似的。
   *
   * @param min
   * 范围的最小角。
   * @param max
   * 范围的最大角。
   * @param options
   * 查询选项。
   * @returns
   * 包围盒与该范围重叠的刚体。
   *
   * @example query-box.ts
   * {@includeCode ../../examples/world/query-box.ts}
   */
  queryBox(min, max, options) {
    this.check();
    finite32("min", min);
    finite32("max", max);
    if (options?.includeBodies === false) return [];
    const box = new Aabb();
    box.minX = Math.min(min.x, max.x);
    box.minY = Math.min(min.y, max.y);
    box.minZ = Math.min(min.z, max.z);
    box.maxX = Math.max(min.x, max.x);
    box.maxY = Math.max(min.y, max.y);
    box.maxZ = Math.max(min.z, max.z);
    const mask = options?.collisionMask;
    const found = /* @__PURE__ */ new Set();
    const callback = {
      process: (shape) => {
        const owner = shape.rigidBody.userData;
        if (!(owner instanceof Body)) return;
        if (mask !== void 0 && (shape.collisionGroup & mask) === 0) return;
        found.add(owner);
      }
    };
    this.core.aabbTest(box, callback);
    return [...found];
  }
  /**
   * 沿直线扫掠形状，返回最近的命中。
   *
   * @param shape
   * 要扫过的形状。仅支持凸形：`box`、`sphere`、`capsule`、`cylinder`、`cone` 和 `convexHull`。
   * @param location
   * 形状的起始位置。
   * @param translation
   * 位移向量，不是方向。
   * @param rotation
   * 形状的起始朝向。默认值为没有旋转。
   * @param options
   * 检测选项。
   * @returns
   * 命中结果。没有命中时为 `undefined`。
   */
  sweep(shape, location, translation, rotation, options) {
    this.check();
    finite32("location", location);
    finite32("translation", translation);
    const geometry = makeGeometry(shape);
    if (!isConvex(geometry)) throw new Error("sweep \u53EA\u652F\u6301\u51F8\u5F62\uFF1Abox\u3001sphere\u3001capsule\u3001cylinder\u3001cone\u3001convexHull");
    const begin = new Transform();
    begin.px = location.x;
    begin.py = location.y;
    begin.pz = location.z;
    if (rotation !== void 0) {
      const m = quaternionToMatrix(rotation);
      begin.r00 = m[0];
      begin.r01 = m[1];
      begin.r02 = m[2];
      begin.r10 = m[3];
      begin.r11 = m[4];
      begin.r12 = m[5];
      begin.r20 = m[6];
      begin.r21 = m[7];
      begin.r22 = m[8];
    }
    const distance = Math.hypot(translation.x, translation.y, translation.z);
    const mask = options?.collisionMask;
    const exclude = options?.excludeBodies;
    const includeBlocks = options?.includeBlocks ?? true, includeBodies = options?.includeBodies ?? true;
    const maxDistance = options?.maxDistance ?? Infinity;
    const best = new RayCastClosest();
    const filter = {
      process: (sh, hit) => {
        const owner2 = sh.rigidBody.userData instanceof Body ? sh.rigidBody.userData : void 0;
        if (owner2 === void 0 && !includeBlocks) return;
        if (owner2 !== void 0 && !includeBodies) return;
        if (owner2 !== void 0 && exclude !== void 0 && exclude.includes(owner2)) return;
        if (mask !== void 0 && (sh.collisionGroup & mask) === 0) return;
        best.process(sh, hit);
      }
    };
    this.core.convexCast(geometry, begin, translation.x, translation.y, translation.z, filter);
    if (!best.hit) return void 0;
    const d = best.fraction * distance;
    if (d > maxDistance) return void 0;
    const rb = best.shape.rigidBody;
    const owner = rb.userData instanceof Body ? rb.userData : void 0;
    const result = {
      point: { x: best.positionX, y: best.positionY, z: best.positionZ },
      // 反转 GJK 扫掠法线，使其朝向运动来向。
      normal: { x: -best.normalX + 0, y: -best.normalY + 0, z: -best.normalZ + 0 },
      distance: d
    };
    if (owner !== void 0) result.body = owner;
    else {
      result.blockLocation = {
        x: Math.floor(best.positionX + best.normalX * 1e-4),
        y: Math.floor(best.positionY + best.normalY * 1e-4),
        z: Math.floor(best.positionZ + best.normalZ * 1e-4)
      };
    }
    return result;
  }
  /**
   * 使指定范围的地形碰撞缓存失效。
   *
   * @remarks
   *
   * - 范围自动外扩 1 格，覆盖邻接形状的变化；未缓存区域在后续读取时获取当前数据。
   * - 未启用地形时无效果。
   *
   * @param min
   * 范围的最小方块坐标。
   * @param max
   * 范围的最大方块坐标，包含在内。
   *
   * @example invalidate-blocks.ts
   * {@includeCode ../../examples/terrain/invalidate-blocks.ts}
   */
  invalidateBlocks(min, max) {
    this.check();
    finite32("min", min);
    finite32("max", max);
    this.collider?.blocksChanged(
      Math.min(min.x, max.x),
      Math.min(min.y, max.y),
      Math.min(min.z, max.z),
      Math.max(min.x, max.x),
      Math.max(min.y, max.y),
      Math.max(min.z, max.z)
    );
  }
  /**
   * 销毁物理世界。
   *
   * @remarks
   *
   * - 停止自动推进，移除所有刚体和关节，并取消所有事件订阅。
   * - 世界及其创建的所有对象随即失效。
   * - 重复调用没有额外效果。
   */
  destroy() {
    if (!this.validFlag) return;
    this.validFlag = false;
    if (this.runId >= 0) {
      system2.clearRun(this.runId);
      this.runId = -1;
    }
    this.unwatchBlocks?.();
    this.unwatchBlocks = null;
    this.unwatchExplosions?.();
    this.unwatchExplosions = null;
    this.explosionQueue.clear();
    this.core.contactManager.listener = null;
    for (const j of this.jointSet) j.invalidate();
    for (const b of this.bodySet) b.invalidate();
    this.jointSet.clear();
    this.bodySet.clear();
    this.sleepState.clear();
    this.fluidBodies.clear();
    this.floatingBodies.clear();
    this.fragileBodies.clear();
    this.pendingBreaks.clear();
    this.brokenShapes.clear();
    this.collider?.dispose();
    for (const signal of Object.values(this.events)) signal.clear();
    this.pending = [];
  }
  // ---- 接触事件 ----
  onBeginContact(c) {
    const owner1 = c.b1.userData, owner2 = c.b2.userData;
    const b1 = owner1 instanceof Body ? owner1 : void 0, b2 = owner2 instanceof Body ? owner2 : void 0;
    const speed = c.contactConstraint.approachSpeed;
    if (speed >= this.fragileMinSpeed && c.manifold.numPoints > 0) this.checkFragile(c, b1, b2, speed);
    const m = c.manifold;
    let px = 0, py = 0, pz = 0;
    const n = m.numPoints;
    for (let i = 0; i < n; i++) {
      const p = m.points[i];
      px += (p.p1X + p.p2X) * 0.5;
      py += (p.p1Y + p.p2Y) * 0.5;
      pz += (p.p1Z + p.p2Z) * 0.5;
    }
    if (n > 0) {
      px /= n;
      py /= n;
      pz /= n;
    }
    const nx = m.nX, ny = m.nY, nz = m.nZ;
    const rel = c.contactConstraint.approachSpeed;
    if (b1 !== void 0 && b2 !== void 0) {
      const signal2 = this.events.bodyCollisionStart;
      if (!signal2.hasSubscribers) return;
      const event2 = { bodyA: b1, bodyB: b2, point: { x: px, y: py, z: pz }, normal: { x: -nx, y: -ny, z: -nz }, impactSpeed: rel };
      this.pending.push(() => signal2.emit(event2));
      return;
    }
    const body = b1 ?? b2;
    if (body === void 0) return;
    const signal = this.events.blockCollisionStart;
    if (!signal.hasSubscribers) return;
    const sign = b1 !== void 0 ? 1 : -1;
    const normal = { x: nx * sign, y: ny * sign, z: nz * sign };
    let bx = 0, by = 0, bz = 0;
    for (let i = 0; i < n; i++) {
      const p = m.points[i];
      if (b1 !== void 0) {
        bx += p.p2X;
        by += p.p2Y;
        bz += p.p2Z;
      } else {
        bx += p.p1X;
        by += p.p1Y;
        bz += p.p1Z;
      }
    }
    if (n > 0) {
      bx /= n;
      by /= n;
      bz /= n;
    }
    const event = {
      body,
      blockLocation: {
        x: Math.floor(bx - normal.x * 1e-3),
        y: Math.floor(by - normal.y * 1e-3),
        z: Math.floor(bz - normal.z * 1e-3)
      },
      point: { x: px, y: py, z: pz },
      normal,
      impactSpeed: rel
    };
    this.pending.push(() => signal.emit(event));
  }
  /**
   * 易碎方块：刚体上的方块形状撞得够快时发事件；刚体撞上世界里的易碎方块时发事件，并在下一 tick 破坏方块。
   * 世界方块按接触点沿法线前后几个位置找（接触点可能落在方块表面之外或者稍微陷进刚体里）
   */
  checkFragile(c, b1, b2, speed) {
    const m = c.manifold, n = m.numPoints;
    let px = 0, py = 0, pz = 0;
    for (let i = 0; i < n; i++) {
      const p = m.points[i];
      px += (p.p1X + p.p2X) * 0.5;
      py += (p.p1Y + p.p2Y) * 0.5;
      pz += (p.p1Z + p.p2Z) * 0.5;
    }
    px /= n;
    py /= n;
    pz /= n;
    const signal = this.events.blockBreak;
    for (let side = 0; side < 2; side++) {
      const body2 = side === 0 ? b1 : b2;
      if (body2 === void 0 || !this.fragileBodies.has(body2)) continue;
      const shape = side === 0 ? c.s1 : c.s2;
      const info = shape.userData;
      const threshold = info?.physics.fragileImpactSpeed;
      if (threshold === void 0 || speed < threshold) continue;
      if (!signal.hasSubscribers) continue;
      if (this.brokenShapes.has(shape)) continue;
      this.brokenShapes.add(shape);
      const l = info.block.location;
      const event = { body: body2, blockLocation: { x: l.x, y: l.y, z: l.z }, typeId: info.block.typeId, shapeIndex: info.index, impactSpeed: speed };
      this.pending.push(() => signal.emit(event));
    }
    const body = b1 !== void 0 && b2 === void 0 ? b1 : b2 !== void 0 && b1 === void 0 ? b2 : void 0;
    if (body === void 0) return;
    const sign = b1 !== void 0 ? 1 : -1;
    const nx = m.nX * sign, ny = m.nY * sign, nz = m.nZ * sign;
    let lastX = NaN, lastY = NaN, lastZ = NaN;
    for (let i = 0; i < FRAGILE_PROBES.length; i++) {
      const s = FRAGILE_PROBES[i];
      const x = Math.floor(px + nx * s), y = Math.floor(py + ny * s), z = Math.floor(pz + nz * s);
      if (x === lastX && y === lastY && z === lastZ) continue;
      lastX = x;
      lastY = y;
      lastZ = z;
      let block;
      try {
        block = this.dimension.getBlock({ x, y, z });
      } catch {
        continue;
      }
      if (block === void 0 || block.isAir || block.isLiquid) continue;
      const threshold = this.blockPhysics(block.typeId, getStates(block)).fragileImpactSpeed;
      if (threshold === void 0 || speed < threshold) continue;
      const key2 = `${x},${y},${z}`;
      if (this.pendingBreaks.has(key2)) return;
      this.pendingBreaks.add(key2);
      const typeId = block.typeId;
      if (signal.hasSubscribers) {
        const event = { body, blockLocation: { x, y, z }, typeId, impactSpeed: speed };
        this.pending.push(() => signal.emit(event));
      }
      if (this.breakFragile) this.queueBreak(x, y, z, typeId, key2);
      else this.pendingBreaks.delete(key2);
      return;
    }
  }
  /** 下一 tick 破坏仍为原类型的方块，并生成掉落物。 */
  queueBreak(x, y, z, typeId, key2) {
    const dimension = this.dimension;
    system2.run(() => {
      this.pendingBreaks.delete(key2);
      if (!this.validFlag) return;
      let block;
      try {
        block = dimension.getBlock({ x, y, z });
      } catch {
        return;
      }
      if (block === void 0 || block.typeId !== typeId) return;
      try {
        dimension.runCommand(`setblock ${x} ${y} ${z} minecraft:air destroy`);
      } catch {
        return;
      }
      this.collider?.blocksChanged(x - 1, y - 1, z - 1, x + 1, y + 1, z + 1);
    });
  }
  onEndContact(c) {
    const owner1 = c.b1.userData, owner2 = c.b2.userData;
    if (!(owner1 instanceof Body && owner2 instanceof Body)) return;
    const signal = this.events.bodyCollisionEnd;
    if (!signal.hasSubscribers) return;
    const event = { bodyA: owner1, bodyB: owner2 };
    this.pending.push(() => signal.emit(event));
  }
};
var FRAGILE_MIN_SPEED = 4;
var FRAGILE_PROBES = [0, -0.08, 0.08, -0.25, 0.25];
function getStates(block) {
  try {
    return block.permutation.getAllStates();
  } catch {
    return {};
  }
}
function checkBodyBlock(block, i) {
  if (block === null || typeof block !== "object" || typeof block.typeId !== "string") throw new Error(`shapes[${i}].block \u5FC5\u987B\u6709 typeId`);
  const l = block.location;
  if (l === null || typeof l !== "object" || !Number.isInteger(l.x) || !Number.isInteger(l.y) || !Number.isInteger(l.z)) {
    throw new Error(`shapes[${i}].block.location \u5FC5\u987B\u662F\u6574\u6570\u5750\u6807`);
  }
  if (block.states !== void 0 && (block.states === null || typeof block.states !== "object")) throw new Error(`shapes[${i}].block.states \u5FC5\u987B\u662F\u5BF9\u8C61`);
}
var TICK_SECONDS = 1 / 20;
var DEFAULT_GRAVITY = { x: 0, y: -11, z: 0 };
var DEFAULT_SUBSTEPS = 4;
var MIN_SUBSTEPS = 2;
var VOID_SEA_START_Y = -105;
var DEFAULT_TIME_BUDGET_MS = 12;
var DEFAULT_DAMPING = 0.09;
var DEFAULT_WATER_DENSITY = 10.5 / 11;
var SOLVER_VELOCITY_ITERATIONS = 8;
var CHUNK_CHECKS_PER_TICK = 4;
var FLUID_RESTLESS = 5e-3;
var LAVA_ENTRY_REARM = 4;
var fluidBounds = [0, 0, 0, 0, 0, 0];
var fluidEntry = { fastestVelocityY: 0, minX: 0, maxX: 0, minZ: 0, maxZ: 0, pointX: 0, pointY: 0, pointZ: 0 };
function newFluidBody() {
  return {
    grid: new FluidGrid(),
    px: 0,
    py: 0,
    pz: 0,
    r00: 1,
    r01: 0,
    r02: 0,
    r10: 0,
    r11: 1,
    r12: 0,
    r20: 0,
    r21: 0,
    r22: 1,
    qx: 0,
    qy: 0,
    qz: 0,
    qw: 1,
    vx: 0,
    vy: 0,
    vz: 0,
    wx: 0,
    wy: 0,
    wz: 0,
    mass: 0,
    invMass: 0,
    ii00: 0,
    ii11: 0,
    ii22: 0,
    gravityScale: 1,
    minX: 0,
    minY: 0,
    minZ: 0,
    maxX: 0,
    maxY: 0,
    maxZ: 0,
    fx: 0,
    fy: 0,
    fz: 0,
    tx: 0,
    ty: 0,
    tz: 0,
    submerged: 0
  };
}
var SLEEP_LINEAR = 0.06;
var SLEEP_ANGULAR = 0.08;
var SLEEP_SECONDS = 0.6;
function now() {
  return Date.now();
}
function checkBudget(v) {
  if (!(v > 0)) throw new Error(`timeBudgetMs \u5FC5\u987B\u5927\u4E8E\u96F6\uFF08Infinity \u8868\u793A\u4E0D\u9650\u5236\uFF09\uFF0C\u6536\u5230 ${v}`);
  return v;
}
function checkInteger(name, v, min) {
  if (v !== void 0 && (!Number.isSafeInteger(v) || v < min)) throw new Error(`${name} \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E ${min} \u7684\u6574\u6570\uFF0C\u6536\u5230 ${v}`);
}
function checkFluids(f) {
  if (f === null || typeof f !== "object") throw new Error("fluids \u5FC5\u987B\u662F\u5BF9\u8C61\u6216 false");
  for (const k of ["waterDensity", "lavaDensity", "drag", "linearDrag", "lavaViscosity", "waveDamping", "floodRate", "currentSpeed"]) {
    const v = f[k];
    if (v !== void 0 && (!(v >= 0) || !Number.isFinite(v))) throw new Error(`fluids.${k} \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 0 \u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${v}`);
  }
  if (f.airPockets !== void 0 && typeof f.airPockets !== "boolean") throw new Error(`fluids.airPockets \u5FC5\u987B\u662F\u5E03\u5C14\u503C\uFF0C\u6536\u5230 ${f.airPockets}`);
}
function finite32(name, v) {
  if (!Number.isFinite(v.x) || !Number.isFinite(v.y) || !Number.isFinite(v.z)) {
    throw new Error(`${name} \u5FC5\u987B\u662F\u6709\u9650\u7684\u6570\uFF0C\u6536\u5230 (${v.x}, ${v.y}, ${v.z})`);
  }
}
function nonNegative2(name, v) {
  if (!(v >= 0) || !Number.isFinite(v)) throw new Error(`${name} \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E\u96F6\u7684\u6709\u9650\u6570\uFF0C\u6536\u5230 ${v}`);
  return v;
}
function withBodyFriction(o) {
  if (o.friction !== void 0) return o;
  return { ...o, friction: materialFriction(DEFAULT_MATERIALS[0]) };
}
var HOST = {
  createListVolume: (locations) => new ListBlockVolume(locations),
  createVolume: (x0, y0, z0, x1, y1, z1) => new BlockVolume({ x: x0, y: y0, z: z0 }, { x: x1, y: y1, z: z1 }),
  classifyError: (e) => {
    if (e instanceof UnloadedChunksError || e instanceof LocationInUnloadedChunkError) return "unloaded";
    if (e instanceof LocationOutOfWorldBoundariesError) return "outOfBounds";
    return "other";
  },
  typeExists: (typeId) => BlockTypes.get(typeId) !== void 0
};

// src/mc/persistence.ts
import { system as system3, world as world4 } from "@minecraft/server";

// src/mc/property-store.ts
import { world as world3 } from "@minecraft/server";
var DEFAULT_CHUNK_SIZE = 3e4;
var JsonPropertyStore = class {
  constructor(prefix, chunkSize = DEFAULT_CHUNK_SIZE, target = world3, owned = false) {
    this.prefix = prefix;
    this.chunkSize = chunkSize;
    this.target = target;
    this.loaded = false;
    this.manifestLoaded = false;
    /** 上次读 `_active` 时它是否存在；owned 时 hasStoredValue 直接用它 */
    this.stored = false;
    this.nextGeneration = 1;
    this.owned = owned;
  }
  /** `_active` 是否存在（格式不对的也算） */
  get hasStoredValue() {
    if (this.owned && this.manifestLoaded) return this.stored;
    return this.target.getDynamicProperty(this.manifestKey()) !== void 0;
  }
  load(raw = false, owned = false, fresh = false) {
    this.owned || (this.owned = owned);
    if (fresh) {
      this.loaded = this.manifestLoaded = false;
      this.lastJson = void 0;
    }
    try {
      const manifest = this.getActiveManifest();
      if (this.owned && this.loaded) return this.lastJson === void 0 ? void 0 : raw ? this.lastJson : JSON.parse(this.lastJson);
      if (!manifest) {
        this.lastJson = void 0;
        this.loaded = true;
        return void 0;
      }
      let json = manifest.json ?? "";
      if (manifest.generation) {
        for (let index = 0; index < manifest.chunkCount; index++) {
          const chunk = this.target.getDynamicProperty(this.chunkKey(manifest.generation, index));
          if (typeof chunk !== "string") return void 0;
          json += chunk;
        }
        if (json.length !== manifest.byteCount) return void 0;
      }
      const value = raw ? json : JSON.parse(json);
      this.lastJson = json;
      this.loaded = true;
      return value;
    } catch {
      return void 0;
    }
  }
  /**
   * 保存 JSON 值；内容相同时返回 unchanged 并跳过写入。
   * 写入失败时抛出原始错误，保留旧活动记录和内存状态。
   */
  saveWithResult(value) {
    const json = JSON.stringify(value);
    if (json === this.lastJson) return "unchanged";
    const previous = this.owned && this.manifestLoaded ? this.activeManifest : this.getActiveManifest();
    let manifest, manifestValue;
    if (json.length * 3 + 1 <= this.chunkSize) {
      manifest = { json };
      manifestValue = `=${json}`;
    } else {
      const chunks = splitIntoChunks(json, this.chunkSize);
      let generation = this.createGeneration();
      if (generation === previous?.generation) generation = this.createGeneration();
      for (let index = 0; index < chunks.length; index++) {
        this.target.setDynamicProperty(this.chunkKey(generation, index), chunks[index]);
      }
      manifest = {
        byteCount: json.length,
        chunkCount: chunks.length,
        generation
      };
      manifestValue = serializeManifest(manifest);
    }
    this.target.setDynamicProperty(this.manifestKey(), manifestValue);
    this.lastJson = json;
    this.manifestValue = manifestValue;
    this.activeManifest = manifest;
    this.manifestLoaded = this.loaded = this.stored = true;
    if (previous?.generation) {
      this.tryDeleteGeneration(previous.generation, previous.chunkCount);
    }
    return "saved";
  }
  /** 删除记录（`_active` 和当前一代的分片）。删 `_active` 出错时返回 false，记录保持原样 */
  clear() {
    let manifest;
    try {
      manifest = this.owned && this.manifestLoaded ? this.activeManifest : this.getActiveManifest();
      if (this.owned && !this.stored) return true;
      this.target.setDynamicProperty(this.manifestKey());
      this.lastJson = void 0;
      this.manifestValue = void 0;
      this.activeManifest = void 0;
      this.manifestLoaded = this.loaded = true;
      this.stored = false;
    } catch {
      return false;
    }
    if (manifest?.generation) this.tryDeleteGeneration(manifest.generation, manifest.chunkCount);
    return true;
  }
  /** 尽量删掉已经被取代的一代分片 */
  tryDeleteGeneration(generation, chunkCount) {
    try {
      for (let index = 0; index < chunkCount; index++) {
        this.target.setDynamicProperty(this.chunkKey(generation, index));
      }
    } catch {
    }
  }
  createGeneration() {
    return `${Date.now().toString(36)}_${(this.nextGeneration++).toString(36)}`;
  }
  getActiveManifest() {
    const raw = this.target.getDynamicProperty(this.manifestKey());
    if (this.owned && this.manifestLoaded && raw === this.manifestValue) return this.activeManifest;
    this.loaded = false;
    this.lastJson = void 0;
    this.manifestValue = raw;
    const manifest = typeof raw === "string" ? parseManifest(raw) : void 0;
    this.activeManifest = manifest;
    this.manifestLoaded = true;
    this.stored = raw !== void 0;
    return manifest;
  }
  manifestKey() {
    return `${this.prefix}_active`;
  }
  chunkKey(generation, index) {
    return `${this.prefix}_g_${generation}_${index}`;
  }
};
function serializeManifest(manifest) {
  return [
    manifest.generation,
    manifest.chunkCount,
    manifest.byteCount
  ].join("|");
}
function parseManifest(value) {
  if (value[0] === "=") return { json: value.slice(1) };
  const [generation, chunks, bytes, extra] = value.split("|");
  const chunkCount = Number.parseInt(chunks ?? "", 10);
  const byteCount = Number.parseInt(bytes ?? "", 10);
  if (!generation || extra !== void 0 || !Number.isFinite(chunkCount) || !Number.isFinite(byteCount) || chunkCount < 1 || byteCount < 0) return void 0;
  return { byteCount, chunkCount, generation };
}
function splitIntoChunks(value, size) {
  const chunks = [];
  for (let index = 0; index < value.length; index += size) {
    chunks.push(value.slice(index, index + size));
  }
  return chunks;
}

// src/mc/persistence.ts
var owners = /* @__PURE__ */ new WeakMap();
var PhysicsWorldStore = class {
  constructor(key2, target = world4) {
    this.key = key2;
    this.target = target;
    this.shapeCache = new SnapshotShapeCache();
    if (!/^[A-Za-z0-9_.:-]{1,80}$/.test(key2)) throw new Error("\u5B58\u6863\u540D\u9700\u4E3A 1\uFF5E80 \u4E2A\u5B57\u6BCD\u3001\u6570\u5B57\u3001\u4E0B\u5212\u7EBF\u3001\u70B9\u3001\u5192\u53F7\u6216\u6A2A\u7EBF");
    this.prefix = `cutlass:store:${key2.length}:${key2}:`;
    this.store = new JsonPropertyStore(this.prefix, void 0, target);
  }
  /**
   * 读取并校验存档。
   *
   * @remarks
   *
   * - 没有存档、记录缺片或 JSON 损坏时返回 `undefined`；`open` 会创建空世界，下次保存覆盖原记录。
   * - 记录格式、版本未知或内容不合法时抛出错误，保存不会覆盖原记录。
   * - 每次读取所有分片。
   */
  read() {
    const value = this.store.load();
    if (value === void 0) {
      this.dimension = null;
      return void 0;
    }
    const snapshot = readSnapshot(value);
    this.dimension = snapshot.dimensionId;
    return snapshot;
  }
  /**
   * 保存完整快照。
   *
   * @remarks
   *
   * - 内容相同时返回 `false` 并跳过写入。
   * - 有活动会话时仅允许会话保存。
   */
  write(snapshot) {
    this.checkOwner(void 0);
    return this.commit(readSnapshot(snapshot), false);
  }
  /**
   * 捕获并保存当前状态。
   *
   * @remarks
   *
   * - 内容相同时跳过写入。
   * - 写入前校验已有记录的维度和版本。
   * - 有活动会话时仅允许会话保存。
   */
  save(physics, options = {}) {
    return this.saveAs(void 0, physics, options);
  }
  /** @internal 会话用自己的身份保存；owner 为 undefined 表示不经过会话的直接保存 */
  saveAs(owner, physics, options) {
    this.checkOwner(owner);
    const start = Date.now(), snapshot = physics.captureSnapshot(options, this.shapeCache);
    const changed = this.commit(snapshot, owner !== void 0);
    return { changed, elapsedMs: Date.now() - start, bodyCount: snapshot.bodies.length, jointCount: snapshot.joints.length };
  }
  /** 活动会话独占同一存档的写入。 */
  checkOwner(owner) {
    const holder = owners.get(this.target)?.get(this.key);
    if (holder !== void 0 && holder !== owner) throw new Error("\u6B64\u7269\u7406\u5B58\u6863\u6709\u6253\u5F00\u7684\u4F1A\u8BDD\uFF0C\u8BF7\u901A\u8FC7\u4F1A\u8BDD\u4FDD\u5B58\uFF0C\u6216\u5148\u5173\u95ED\u4F1A\u8BDD");
  }
  /** 写入一份快照。session 为 true 表示由打开着的会话保存，可以信任记下的维度 */
  commit(snapshot, session) {
    if (!session || this.dimension === void 0) this.read();
    if (typeof this.dimension === "string" && this.dimension !== snapshot.dimensionId) {
      throw new Error(`\u6B64\u7269\u7406\u5B58\u6863\u5C5E\u4E8E\u7EF4\u5EA6 ${this.dimension}\uFF0C\u4E0D\u80FD\u4FDD\u5B58 ${snapshot.dimensionId} \u7684\u4E16\u754C\uFF1B\u6BCF\u4E2A\u7EF4\u5EA6\u8BF7\u7528\u4E0D\u540C\u7684\u5B58\u6863\u540D`);
    }
    const changed = this.store.saveWithResult(snapshot) === "saved";
    this.dimension = snapshot.dimensionId;
    return changed;
  }
  /**
   * 删除活动记录后清理分片。
   *
   * @remarks
   *
   * - 清理失败后可重新调用。
   * - 有活动会话时抛出错误。
   */
  delete() {
    if (owners.get(this.target)?.has(this.key)) throw new Error("\u8BF7\u5148\u5173\u95ED\u6B64\u5B58\u6863\u7684\u6301\u4E45\u5316\u4F1A\u8BDD");
    this.store.clear();
    for (const id of this.target.getDynamicPropertyIds()) if (id.startsWith(this.prefix)) this.target.setDynamicProperty(id, void 0);
  }
  /**
   * 打开持久化世界。
   *
   * @remarks
   *
   * - 默认加载已有记录，并每 100 tick 自动保存。
   * - 同一后端及存档名仅允许一个活动会话。
   * - 自动保存默认包含正常退出保存。
   */
  open(dimension, options = {}) {
    let active = owners.get(this.target);
    if (active === void 0) {
      active = /* @__PURE__ */ new Map();
      owners.set(this.target, active);
    }
    if (active.has(this.key)) throw new Error("\u6B64\u7269\u7406\u5B58\u6863\u5DF2\u6709\u6253\u5F00\u7684\u4F1A\u8BDD");
    const interval = options.autoSaveIntervalTicks ?? 100;
    if (!Number.isSafeInteger(interval) || interval < 0) throw new Error("autoSaveIntervalTicks \u5FC5\u987B\u662F\u4E0D\u5C0F\u4E8E 0 \u7684\u6574\u6570");
    const pending = {};
    active.set(this.key, pending);
    this.dimension = void 0;
    try {
      const snapshot = options.autoLoad === false ? void 0 : this.read();
      const physics = snapshot === void 0 ? new PhysicsWorld(dimension, options) : PhysicsWorld.fromSnapshot(dimension, snapshot, options);
      const session = new PersistentPhysicsWorld(this, physics, dimension, options, interval, snapshot !== void 0, () => active.delete(this.key));
      active.set(this.key, session);
      return session;
    } catch (error) {
      if (active.get(this.key) === pending) active.delete(this.key);
      throw error;
    }
  }
};
var PersistentPhysicsWorld = class {
  /** @internal 通过 PhysicsWorldStore.open 创建。loaded 表示 current 刚从存档恢复，和存档一致 */
  constructor(store, current, dimension, options, interval, loaded, release) {
    this.store = store;
    this.current = current;
    this.dimension = dimension;
    this.options = options;
    this.release = release;
    this.closed = false;
    this.shutdown = () => {
      if (this.closed || !this.current.isValid) return;
      try {
        this.save();
      } catch (error) {
        if (this.options.onError) this.options.onError(error);
        else throw error;
      }
    };
    this.watchingShutdown = false;
    this.savedRevision = loaded ? current.revision : -1;
    try {
      if (options.saveOnShutdown ?? interval > 0) {
        system3.beforeEvents.shutdown.subscribe(this.shutdown);
        this.watchingShutdown = true;
      }
      let elapsed = 0;
      if (interval > 0) this.timer = system3.runInterval(() => {
        if (!this.current.isValid) {
          this.close(false);
          return;
        }
        if (++elapsed < interval) return;
        elapsed = 0;
        try {
          if (this.unchanged()) {
            this.options.onSave?.({ changed: false, elapsedMs: 0, bodyCount: this.current.bodyCount, jointCount: this.current.jointCount });
            return;
          }
          this.save();
        } catch (error) {
          if (this.options.onError) this.options.onError(error);
          else throw error;
        }
      }, 1);
    } catch (error) {
      if (this.watchingShutdown) system3.beforeEvents.shutdown.unsubscribe(this.shutdown);
      current.destroy();
      throw error;
    }
  }
  /** 当前物理世界。reload 成功后替换为新对象，旧显示绑定和事件订阅不继承。 */
  get world() {
    this.check();
    return this.current;
  }
  check() {
    if (this.closed) throw new Error("\u6301\u4E45\u5316\u4F1A\u8BDD\u5DF2\u7ECF\u5173\u95ED");
  }
  /** 世界状态未变化时可跳过自动保存。指定 encodeUserData 时不使用此判断。 */
  unchanged() {
    return this.current.revision === this.savedRevision && this.options.encodeUserData === void 0;
  }
  /** 立即保存；未变化时不写动态属性。 */
  save() {
    this.check();
    const revision = this.current.revision;
    const result = this.store.saveAs(this, this.current, this.options);
    this.savedRevision = revision;
    this.options.onSave?.(result);
    return result.changed;
  }
  /** 以最近保存的状态替换世界。新世界创建成功后销毁旧世界，旧 Body、Joint 和事件订阅随之失效。 */
  reload() {
    this.check();
    const snapshot = this.store.read();
    if (snapshot === void 0) throw new Error("\u6B64\u5B58\u6863\u5C1A\u672A\u4FDD\u5B58\uFF0C\u6216\u8005\u8BB0\u5F55\u5DF2\u635F\u574F\uFF08\u7F3A\u7247\u3001JSON \u65E0\u6CD5\u89E3\u6790\uFF09");
    const next = PhysicsWorld.fromSnapshot(this.dimension, snapshot, this.options);
    this.current.destroy();
    this.current = next;
    this.savedRevision = next.revision;
    return next;
  }
  /**
   * 保存后停止定时器并销毁世界。
   *
   * @remarks
   *
   * - 保存失败时保留会话。
   * - `save=false` 放弃未保存变更。
   * - 退出回调中的退订失败不影响其余清理。
   */
  close(save = true) {
    if (this.closed) return;
    if (save && this.current.isValid) this.save();
    if (this.timer !== void 0) system3.clearRun(this.timer);
    if (this.watchingShutdown) {
      try {
        system3.beforeEvents.shutdown.unsubscribe(this.shutdown);
      } catch {
      }
    }
    this.current.destroy();
    this.closed = true;
    this.release();
  }
};
export {
  Body,
  Drag,
  Joint2 as Joint,
  PersistentPhysicsWorld,
  PhysicsWorld,
  PhysicsWorldStore,
  VERSION,
  blockShapes,
  eulerToQuaternion,
  getBlockPhysics,
  quaternionToEuler
};
