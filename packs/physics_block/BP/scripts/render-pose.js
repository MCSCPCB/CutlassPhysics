// 同一结构共用姿态采样、插值时钟和显示锚点。
import { system } from "@minecraft/server";
const squaredDistance = (a, b) => (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2;
const vectorsEqual = (a, b) => a.x === b.x && a.y === b.y && a.z === b.z;
const RENDER_INPUT_REFRESH_TICKS = 40;
const INITIAL_RENDER_INPUT_REFRESH_TICKS = [2, 4, 8, 12, 20];
const RENDER_INPUT_ANIMATION = "animation.cutlass.fancy.input";
const sessionTag = `${Date.now().toString(36)}_${Math.random().toString(36).slice(2)}`;
let poseId = 0;
const WORLD_ANCHOR_DISTANCE = 14;
const WORLD_ANCHOR_LARGE_DISTANCE = 28;
const WORLD_ANCHOR_CHECK_TICKS = 20;
const WORLD_ANCHOR_LIGHT_TICKS = 80;
const WORLD_ANCHOR_OFFSETS = [
  [0, 0, 0],
  [0, 1, 0],
  [1, 0, 0],
  [-1, 0, 0],
  [0, 0, 1],
  [0, 0, -1],
  [0, -1, 0],
  [0, 2, 0],
  [0, 3, 0]
];
class RenderPose {
  constructor(body, anchor, dimension) {
    this.body = body;
    this.anchor = anchor;
    this.dimension = dimension;
    const position = body.localPointToWorld(anchor);
    const selected = dimension ? selectWorldRenderAnchor(dimension, position) : position;
    if (!selected) throw new Error("附近没有可用的显示锚点。");
    this.#worldAnchor = { ...selected };
  }
  body;
  anchor;
  dimension;
  #tick = -1;
  #current;
  #input = {};
  #serial = 0;
  #sleeping = false;
  #worldAnchor;
  #anchorRevision = 0;
  #anchorTick = -1;
  #anchorCheckTick = system.currentTick;
  #lightCheckTick = system.currentTick;
  #anchorSearchTick = -WORLD_ANCHOR_CHECK_TICKS;
  #models = new Map();
  #tag = `cutlass_pose_${sessionTag}_${poseId++}`;
  #modelRevision = 0;
  #publishedModels = -1;
  #retryAt = 0;
  #refreshTicks = new Set();
  #publishedTick = -1;
  #publishedSerial = -1;
  #publishedGeneration = -1;
  #renderSampleTick = -1;
  #anchorCommits = [];
  get input() {
    return this.#input;
  }
  get worldAnchor() {
    return { ...this.#worldAnchor };
  }
  get anchorRevision() {
    return this.#anchorRevision;
  }
  get isPublished() {
    return this.#publishedSerial === this.#input.pose_serial && this.#publishedGeneration === this.#anchorRevision && this.#publishedModels === this.#modelRevision;
  }
  sampleForRender(force = false, position) {
    if (this.#renderSampleTick === system.currentTick && !force && this.#sleeping === (this.body.isSleeping === true)) {
      return this.#input;
    }
    this.#renderSampleTick = system.currentTick;
    return this.sample(true, position);
  }
  addModel(entity) {
    for (const tag of entity.getTags()) if (tag.startsWith("cutlass_pose_")) entity.removeTag(tag);
    entity.addTag(`${this.#tag}_${this.#anchorRevision}`);
    this.#models.set(entity.id, { entity, generation: this.#anchorRevision });
    this.#modelRevision++;
    this.#retryAt = 0;
  }
  removeModel(entity) {
    this.#models.delete(entity.id);
    this.#modelRevision++;
  }
  refreshAfterSpawn(isValid) {
    for (const delay of INITIAL_RENDER_INPUT_REFRESH_TICKS) {
      const tick = system.currentTick + delay;
      if (this.#refreshTicks.has(tick)) continue;
      this.#refreshTicks.add(tick);
      system.runTimeout(() => {
        this.#refreshTicks.delete(tick);
        if (isValid()) this.publish(true);
      }, delay);
    }
  }
  // 持续控制器向后来进入视野的客户端提供姿态。
  publish(force = false) {
    const input = this.sample();
    const unchanged = this.#publishedSerial === input.pose_serial && this.#publishedGeneration === this.#anchorRevision && this.#publishedModels === this.#modelRevision;
    if (unchanged && this.#publishedTick === system.currentTick) return 0;
    if (!force && unchanged && system.currentTick - this.#publishedTick < RENDER_INPUT_REFRESH_TICKS) return 0;
    if (this.#models.size === 0 || system.currentTick < this.#retryAt) return 0;
    try {
      const result = this.dimension.runCommand(`playanimation @e[type=cutlass:physics_block,tag=${this.#tag}_${this.#anchorRevision}] ${RENDER_INPUT_ANIMATION} none 0 "${renderInputExpression(input)}" cutlass_pose`);
      if (result.successCount === 0) { this.#retryAt = system.currentTick + 20; return 0; }
    } catch {
      this.#retryAt = system.currentTick + 20;
      return 0;
    }
    this.#publishedTick = system.currentTick;
    this.#publishedSerial = input.pose_serial;
    this.#publishedGeneration = this.#anchorRevision;
    this.#publishedModels = this.#modelRevision;
    return 1;
  }
  // 隐藏一 tick 后发布姿态并显示。
  deferAnchorCommit(reveal) {
    this.#anchorCommits.push({ reveal });
    if (this.#anchorCommits.length > 1) return;
    const commit = () => {
      const commits = this.#anchorCommits;
      this.#anchorCommits = [];
      for (const entry of commits) {
        if (entry.reveal() === false) this.#anchorCommits.push(entry);
      }
      if (this.#anchorCommits.length) system.runTimeout(commit, 1);
    };
    system.runTimeout(commit, 1);
  }
  // 每 tick 统一更新结构的显示锚点。
  updateWorldAnchor(position) {
    if (!this.dimension || this.#anchorTick === system.currentTick) return;
    this.#anchorTick = system.currentTick;
    position ??= this.body.localPointToWorld(this.anchor);
    const distance2 = squaredDistance(position, this.#worldAnchor);
    const sleeping = this.body.isSleeping === true;
    const reach = WORLD_ANCHOR_DISTANCE + (WORLD_ANCHOR_LARGE_DISTANCE - WORLD_ANCHOR_DISTANCE) * Math.max(0, Math.min(1, (this.#models.size - 48) / 112));
    let velocity2;
    let replace = distance2 > reach ** 2;
    if (replace) {
      velocity2 = this.body.velocity;
      replace = !velocity2 || (position.x - this.#worldAnchor.x) * velocity2.x + (position.y - this.#worldAnchor.y) * velocity2.y + (position.z - this.#worldAnchor.z) * velocity2.z >= 0 || distance2 > (reach * 3) ** 2;
    }
    if (system.currentTick - this.#anchorCheckTick >= WORLD_ANCHOR_CHECK_TICKS) {
      this.#anchorCheckTick = system.currentTick;
      replace ||= !isWorldRenderAnchorClear(this.dimension, this.#worldAnchor);
    }
    let checkLight = false;
    if (sleeping && system.currentTick - this.#lightCheckTick >= WORLD_ANCHOR_LIGHT_TICKS) {
      this.#lightCheckTick = system.currentTick;
      checkLight = isWorldRenderAnchorClear(this.dimension, this.#worldAnchor) && this.dimension.getLightLevel(this.#worldAnchor) < 12;
    }
    if (!replace && !checkLight || system.currentTick - this.#anchorSearchTick < WORLD_ANCHOR_CHECK_TICKS) return;
    this.#anchorSearchTick = system.currentTick;
    this.#lightCheckTick = system.currentTick;
    if (replace && !sleeping && !velocity2) velocity2 = this.body.velocity;
    const lead = replace && !sleeping && velocity2 ? Math.min(1, reach * 0.6 / (Math.hypot(velocity2.x, velocity2.y, velocity2.z) || 1)) : 0;
    let target = position;
    if (lead > 0 && velocity2) {
      const steps = Math.ceil(Math.hypot(velocity2.x, velocity2.y, velocity2.z) * lead * 2);
      for (let step = 1; step <= steps; step++) {
        const fraction = lead * step / steps;
        const candidate = {
          x: position.x + velocity2.x * fraction,
          y: position.y + velocity2.y * fraction,
          z: position.z + velocity2.z * fraction
        };
        if (!isWorldRenderAnchorClear(this.dimension, candidate)) break;
        target = candidate;
      }
    }
    const next = selectWorldRenderAnchor(this.dimension, target);
    if (!next) return;
    if (vectorsEqual(next, this.#worldAnchor)) return;
    if (!replace && this.dimension.getLightLevel(next) < this.dimension.getLightLevel(this.#worldAnchor) + 4) return;
    const shift = { x: next.x - this.#worldAnchor.x, y: next.y - this.#worldAnchor.y, z: next.z - this.#worldAnchor.z };
    for (const key of ["x", "y", "z"]) {
      if (this.#current) this.#input[`pose_c${key}`] -= shift[key];
    }
    const translate = (sample) => sample && {
      ...sample,
      x: sample.x - shift.x,
      y: sample.y - shift.y,
      z: sample.z - shift.z
    };
    this.#current = translate(this.#current);
    this.#worldAnchor = next;
    this.#anchorRevision++;
  }
  sample(force = false, position) {
    const sameTick = this.#tick === system.currentTick;
    const sleeping = this.body.isSleeping === true;
    if (sameTick && !force && this.#sleeping === sleeping) {
      return this.#input;
    }
    position ??= this.body.localPointToWorld(this.anchor);
    const quaternion = this.body.getRenderQuaternion();
    const sample = {
      n: system.currentTick % 1e6,
      x: position.x - this.#worldAnchor.x,
      y: position.y - this.#worldAnchor.y,
      z: position.z - this.#worldAnchor.z,
      qx: quaternion.x,
      qy: quaternion.y,
      qz: quaternion.z,
      qw: quaternion.w
    };
    const current = this.#current;
    const unchanged = current && current.x === sample.x && current.y === sample.y && current.z === sample.z && current.qx === sample.qx && current.qy === sample.qy && current.qz === sample.qz && current.qw === sample.qw;
    if (sameTick && unchanged && sleeping === this.#sleeping) return this.#input;
    const moving = !sleeping && !unchanged ? 1 : 0;
    if (unchanged && this.#input.pose_moving === moving && (sameTick || !moving)) {
      this.#sleeping = sleeping;
      return this.#input;
    }
    const interval = this.#input.pose_moving === 1 ? Math.max(1, system.currentTick - this.#tick) : 1;
    this.#current = sample;
    this.#tick = system.currentTick;
    this.#sleeping = sleeping;
    this.#serial = (this.#serial + 1) % 1e6;
    this.#input = {
      pose_moving: moving,
      pose_serial: this.#serial,
      pose_dt: interval,
      pose_cn: sample.n,
      pose_cx: sample.x,
      pose_cy: sample.y,
      pose_cz: sample.z,
      pose_cqx: sample.qx,
      pose_cqy: sample.qy,
      pose_cqz: sample.qz,
      pose_cqw: sample.qw
    };
    return this.#input;
  }
}
function isWorldRenderAnchorClear(dimension, position) {
  const { min, max } = dimension.heightRange;
  if (position.y < min || position.y >= max || !dimension.isChunkLoaded(position)) return false;
  const block = dimension.getBlock(position);
  return block?.isAir === true || block?.isLiquid === true;
}
function selectWorldRenderAnchor(dimension, position) {
  let selected;
  let light2 = -1;
  for (const [x, y, z] of WORLD_ANCHOR_OFFSETS) {
    const candidate = {
      x: Math.floor(position.x) + 0.5 + x,
      y: Math.floor(position.y) + 0.5 + y,
      z: Math.floor(position.z) + 0.5 + z
    };
    if (!isWorldRenderAnchorClear(dimension, candidate)) continue;
    const candidateLight = dimension.getLightLevel(candidate);
    if (!selected || candidateLight >= light2 + 3) {
      selected = candidate;
      light2 = candidateLight;
    }
    if (light2 >= 13) break;
  }
  if (selected) return selected;
  for (let radius = 2; radius <= WORLD_ANCHOR_LARGE_DISTANCE; radius++) {
    for (const [x, y, z] of [[0, 1, 0], [1, 0, 0], [-1, 0, 0], [0, 0, 1], [0, 0, -1], [0, -1, 0]]) {
      const candidate = {
        x: Math.floor(position.x) + 0.5 + x * radius,
        y: Math.floor(position.y) + 0.5 + y * radius,
        z: Math.floor(position.z) + 0.5 + z * radius
      };
      if (isWorldRenderAnchorClear(dimension, candidate)) return candidate;
    }
  }
  return void 0;
}
function playSubLevelRenderInput(entity, values, controller) {
  entity.playAnimation(RENDER_INPUT_ANIMATION, {
    ...controller ? { controller } : {},
    stopExpression: renderInputExpression(values)
  });
}
function renderInputExpression(values) {
  if (values.pose_serial !== void 0) {
    return `v.pose_moving=${values.pose_moving};v.pose_serial=${values.pose_serial};v.pose_dt=${values.pose_dt};v.pose_cn=${values.pose_cn};v.pose_cx=${values.pose_cx};v.pose_cy=${values.pose_cy};v.pose_cz=${values.pose_cz};v.pose_cqx=${values.pose_cqx};v.pose_cqy=${values.pose_cqy};v.pose_cqz=${values.pose_cqz};v.pose_cqw=${values.pose_cqw};${values.pose_attached === void 0 ? "" : `v.pose_attached=${values.pose_attached};`}return !${Math.random()};`;
  }
  let expression = "";
  for (const name in values) expression += `v.${name}=${values[name]};`;
  return `${expression}return !${Math.random()};`;
}
export {
  INITIAL_RENDER_INPUT_REFRESH_TICKS,
  RENDER_INPUT_ANIMATION,
  RENDER_INPUT_REFRESH_TICKS,
  RenderPose,
  playSubLevelRenderInput
};
