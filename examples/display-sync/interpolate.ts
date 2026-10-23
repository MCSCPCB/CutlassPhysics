import { PhysicsWorld, type Body, type Vector3 } from "cutlass-physics";
import { world } from "@minecraft/server";

const physics = new PhysicsWorld(world.getDimension("overworld"));

/** 在上一步和这一步之间插值，t 是 0 到 1 的比例 */
function lerp(a: Vector3, b: Vector3, t: number): Vector3 {
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, z: a.z + (b.z - a.z) * t };
}

export function displayLocation(body: Body, t: number): Vector3 {
  return lerp(body.previousLocation, body.location, t);
}

physics.afterEvents.stepEnd.subscribe(() => {
  // 显示位置由相邻两步的位置插值得到。
});
