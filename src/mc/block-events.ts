// 将原版方块事件转换为地形失效通知。
// 放置、破坏和首次交互更新所有被引用的块；压力板、活塞和爆炸仅更新已缓存块。
// 活塞在当前 tick 和下一 tick 各更新一次，以覆盖方块移动过程。
// 未产生事件的变化由地形复查或显式失效通知处理。
import { system, world } from "@minecraft/server";

/** 方块坐标闭区间可能变了。cachedOnly 为 true 时只处理已经缓存的块 */
export type BlockChangeHandler = (minX: number, minY: number, minZ: number, maxX: number, maxY: number, maxZ: number, cachedOnly: boolean) => void;

interface Vec { x: number; y: number; z: number }

interface BlockEventLike {
  block?: { location: Vec; dimension?: { id: string }; permutation?: { getState(name: string): unknown } };
  dimension?: { id: string; getBlock?(location: Vec): { isAir: boolean; isLiquid: boolean } | undefined };
  isFirstEvent?: boolean;
  isExpanding?: boolean;
  piston?: { getAttachedBlocksLocations(): Vec[] };
  getImpactedBlocks?: () => { location: Vec }[];
}

/** 活塞 facing_direction 0~5 对应的推出方向 */
const PISTON_DIRECTIONS: readonly Vec[] = [
  { x: 0, y: -1, z: 0 }, { x: 0, y: 1, z: 0 }, { x: 0, y: 0, z: 1 }, { x: 0, y: 0, z: -1 }, { x: 1, y: 0, z: 0 }, { x: -1, y: 0, z: 0 },
];
/** 活塞最多推 12 个方块，沿推的方向从第 2 格查到第 13 格 */
const PISTON_PUSH_LIMIT = 12;

/** 订阅方块相关事件，只处理指定维度的。返回取消订阅的函数 */
export function watchBlockEvents(dimensionId: string, onChanged: BlockChangeHandler): () => void {
  const after = world.afterEvents;
  const unsubscribers: (() => void)[] = [];
  const pending = new Set<number>();

  const sameDimension = (e: BlockEventLike): boolean => {
    const id = e.dimension?.id ?? e.block?.dimension?.id;
    return id === undefined || id === dimensionId;
  };

  // 事件对象可能已失效；单次读取失败不影响其他事件。
  const guard = (handler: (e: BlockEventLike) => void) => (e: BlockEventLike): void => {
    try {
      if (sameDimension(e)) handler(e);
    } catch {
      // 读取失败的变化由地形复查或显式失效通知处理。
    }
  };

  const add = (signal: { subscribe(cb: never): unknown; unsubscribe(cb: never): void }, handler: (e: BlockEventLike) => void): void => {
    const cb = guard(handler) as never;
    signal.subscribe(cb);
    unsubscribers.push(() => signal.unsubscribe(cb));
  };

  const around = (e: BlockEventLike): void => {
    const l = e.block!.location;
    onChanged(l.x - 1, l.y - 1, l.z - 1, l.x + 1, l.y + 1, l.z + 1, false);
  };
  const cell = (l: Vec): void => onChanged(l.x, l.y, l.z, l.x, l.y, l.z, true);

  add(after.playerPlaceBlock, around);
  add(after.playerBreakBlock, around);
  add(after.blockExplode, around);
  add(after.playerInteractWithBlock, (e) => { if (e.isFirstEvent === true) around(e); });
  add(after.pressurePlatePush, (e) => cell(e.block!.location));
  add(after.pressurePlatePop, (e) => cell(e.block!.location));

  add(after.pistonActivate, (e) => {
    const p = e.block!.location, d = PISTON_DIRECTIONS[e.block!.permutation!.getState("facing_direction") as number];
    if (d === undefined) return;
    const cells: Vec[] = [{ x: p.x, y: p.y, z: p.z }, { x: p.x + d.x, y: p.y + d.y, z: p.z + d.z }];
    if (e.isExpanding === true && e.dimension?.getBlock !== undefined) {
      try {
        for (let k = 2; k <= PISTON_PUSH_LIMIT + 1; k++) {
          const l = { x: p.x + d.x * k, y: p.y + d.y * k, z: p.z + d.z * k };
          const b = e.dimension.getBlock(l);
          if (b === undefined || b.isAir || b.isLiquid) break;
          cells.push(l);
        }
      } catch {
        // 未加载位置终止扫描，已扫描范围仍产生通知。
      }
    }
    for (const a of e.piston!.getAttachedBlocksLocations()) {
      cells.push({ x: a.x, y: a.y, z: a.z }, { x: a.x + d.x, y: a.y + d.y, z: a.z + d.z }, { x: a.x - d.x, y: a.y - d.y, z: a.z - d.z });
    }
    for (const l of cells) cell(l);
    const id = system.runTimeout(() => {
      pending.delete(id);
      for (const l of cells) cell(l);
    }, 1);
    pending.add(id);
  });

  add(after.explosion, (e) => {
    for (const b of e.getImpactedBlocks!()) cell(b.location);
  });

  return () => {
    for (const u of unsubscribers) {
      // 退出回调禁止退订；失败后继续清理其他资源。
      try { u(); } catch { /* 见上 */ }
    }
    unsubscribers.length = 0;
    for (const id of pending) system.clearRun(id);
    pending.clear();
  };
}
