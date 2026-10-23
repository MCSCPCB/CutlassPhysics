import type { Dimension } from "@minecraft/server";
import { PhysicsWorldStore, type PersistentPhysicsWorld } from "cutlass-physics";

// 世界就绪后可打开存档；存档名包含维度标识。
// 记录归属行为包 UUID；默认每 100 tick 自动保存，并在正常退出时保存。
export function openSavedPhysics(dimension: Dimension): PersistentPhysicsWorld {
  const store = new PhysicsWorldStore(`myaddon:${dimension.id}`);
  return store.open(dimension, {
    terrain: {},
    // 0 禁用自动保存；未变化的世界跳过写入。
    autoSaveIntervalTicks: 100,
    // 退出保存默认随自动保存启用，也可独立设置。
    saveOnShutdown: true,
    // 默认不保存 userData；此处仅保存业务标识。
    encodeUserData: (body) => typeof body.userData === "string" ? body.userData : null,
  });
}

// 已有存档恢复后无需重复初始化刚体。
// const session = openSavedPhysics(world.getDimension("overworld"));
// session.world.createBody({ shapes: [...], userData: "crate-001" });

// save 同步保存，内容未变化时跳过写入。
// session.save();

// reload 成功后，旧世界及其 Body、Joint 引用失效。
// 新世界的事件订阅和显示绑定独立于旧世界。
// const restored = session.reload();
// restored.forEachBody((body) => bindDisplay(body, body.userData));

// close 默认先保存再销毁；保存失败时保留会话。
// session.close();
// close(false) 放弃未保存的变更，保留已有存档。
// store.delete() 仅在会话关闭后可删除记录。
