// 世界动态属性存储与自动保存。没有导入副作用，由接入方指定存档名后启动。
import { system, world, type Dimension } from "@minecraft/server";
import { JsonPropertyStore } from "./property-store.js";
import { readSnapshot, SnapshotShapeCache, type PhysicsSnapshot, type RestoreOptions, type SnapshotOptions } from "./snapshot.js";
import { PhysicsWorld } from "./world.js";

/** 可替换的动态属性后端，默认使用 MC 世界。 */
export type PhysicsStorageTarget = Pick<typeof world, "getDynamicProperty" | "setDynamicProperty" | "getDynamicPropertyIds">;

/** 自动持久化世界的选项。 */
export interface PersistenceOptions extends RestoreOptions, SnapshotOptions {
  /** 默认 100 tick（正常速度下 5 秒）。0 关闭自动保存，仍可显式调用 save。 */
  autoSaveIntervalTicks?: number;
  /**
   * 是否在正常退出世界时同步补存。
   *
   * @remarks
   *
   * - 默认随自动保存启用；手动模式可单独设为 `true`。
   * - 异常终止不触发保存。
   */
  saveOnShutdown?: boolean;
  /**
   * 是否加载已有记录，默认为 true。
   * false 从空世界开始，下次保存替换记录；未知版本或非法内容仍会阻止覆盖。
   */
  autoLoad?: boolean;
  /**
   * 自动保存或正常退出时补存出错的回调。
   *
   * @remarks
   *
   * - 参数为写入时抛出的原始错误。
   * - 省略时将错误写入 ContentLog。
   * - 自动保存失败后在下一个保存间隔重试。
   */
  onError?: (error: unknown) => void;
  /** 每次显式或自动保存成功后调用，也包含跳过写入的结果。 */
  onSave?: (result: PersistenceSaveResult) => void;
}

/** 一次成功保存的统计，耗时包含快照和动态属性读写。 */
export interface PersistenceSaveResult {
  /** 是否写入了动态属性。内容和存档完全相同（或自动保存发现世界没有变化）时为 false */
  changed: boolean;
  /** 这次保存花的时间（毫秒），包括捕获、序列化和写入；自动保存发现世界没有变化而跳过时为 0 */
  elapsedMs: number;
  /** 保存的刚体数 */
  bodyCount: number;
  /** 保存的关节数 */
  jointCount: number;
}

/** 各后端上打开着的会话：存档名 → 会话。会话打开期间，只有它自己能写这个存档 */
const owners = new WeakMap<PhysicsStorageTarget, Map<string, object>>();

/**
 * 当前行为包的命名物理存档，通过世界动态属性保存。
 *
 * @remarks
 *
 * - 记录归属行为包 UUID。
 * - 大记录分片写入后切换 `_active`，写入失败时保留旧记录。
 * - 提交顺序仅覆盖脚本 API 调用，不保证进程崩溃时的磁盘事务性。
 */
export class PhysicsWorldStore {
  private readonly prefix: string;
  private readonly store: JsonPropertyStore;
  private readonly shapeCache = new SnapshotShapeCache();
  /** 已有记录的维度：null 表示无有效记录，undefined 表示尚未确认。仅在会话独占写入期间缓存。 */
  private dimension: string | null | undefined;

  constructor(
    /** 存档名 */
    readonly key: string,
    private readonly target: PhysicsStorageTarget = world,
  ) {
    if (!/^[A-Za-z0-9_.:-]{1,80}$/.test(key)) throw new Error("存档名需为 1～80 个字母、数字、下划线、点、冒号或横线");
    // 前缀编码名称长度，防止存档名称与分片编号冲突。
    this.prefix = `cutlass:store:${key.length}:${key}:`;
    this.store = new JsonPropertyStore(this.prefix, undefined, target);
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
  read(): PhysicsSnapshot | undefined {
    const value = this.store.load();
    if (value === undefined) {
      this.dimension = null;
      return undefined;
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
  write(snapshot: PhysicsSnapshot): boolean {
    this.checkOwner(undefined);
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
  save(physics: PhysicsWorld, options: SnapshotOptions = {}): PersistenceSaveResult {
    return this.saveAs(undefined, physics, options);
  }

  /** @internal 会话用自己的身份保存；owner 为 undefined 表示不经过会话的直接保存 */
  saveAs(owner: object | undefined, physics: PhysicsWorld, options: SnapshotOptions): PersistenceSaveResult {
    this.checkOwner(owner);
    const start = Date.now(), snapshot = physics.captureSnapshot(options, this.shapeCache);
    const changed = this.commit(snapshot, owner !== undefined);
    return { changed, elapsedMs: Date.now() - start, bodyCount: snapshot.bodies.length, jointCount: snapshot.joints.length };
  }

  /** 活动会话独占同一存档的写入。 */
  private checkOwner(owner: object | undefined): void {
    const holder = owners.get(this.target)?.get(this.key);
    if (holder !== undefined && holder !== owner) throw new Error("此物理存档有打开的会话，请通过会话保存，或先关闭会话");
  }

  /** 写入一份快照。session 为 true 表示由打开着的会话保存，可以信任记下的维度 */
  private commit(snapshot: PhysicsSnapshot, session: boolean): boolean {
    // 会话之外，别的实例可能刚写过这个存档：重新读一次，同时刷新存储里“上次的内容”，相同内容才能正确跳过
    if (!session || this.dimension === undefined) this.read();
    // 存档名跨维度共享，写入前校验维度一致性。
    if (typeof this.dimension === "string" && this.dimension !== snapshot.dimensionId) {
      throw new Error(`此物理存档属于维度 ${this.dimension}，不能保存 ${snapshot.dimensionId} 的世界；每个维度请用不同的存档名`);
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
  delete(): void {
    if (owners.get(this.target)?.has(this.key)) throw new Error("请先关闭此存档的持久化会话");
    // 清除存储缓存；活动记录删除失败时由后续分片清理重试并报告错误。
    this.store.clear();
    for (const id of this.target.getDynamicPropertyIds()) if (id.startsWith(this.prefix)) this.target.setDynamicProperty(id, undefined);
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
  open(dimension: Dimension, options: PersistenceOptions = {}): PersistentPhysicsWorld {
    let active = owners.get(this.target);
    if (active === undefined) { active = new Map(); owners.set(this.target, active); }
    if (active.has(this.key)) throw new Error("此物理存档已有打开的会话");
    const interval = options.autoSaveIntervalTicks ?? 100;
    if (!Number.isSafeInteger(interval) || interval < 0) throw new Error("autoSaveIntervalTicks 必须是不小于 0 的整数");
    // 会话对象创建之前先占住存档名；会话建好后换成它自己
    const pending = {};
    active.set(this.key, pending);
    // 会话之前记下的维度可能已经过时（别的实例在这之间写过）：会话的第一次读或写重新确认
    this.dimension = undefined;
    try {
      const snapshot = options.autoLoad === false ? undefined : this.read();
      const physics = snapshot === undefined ? new PhysicsWorld(dimension, options) : PhysicsWorld.fromSnapshot(dimension, snapshot, options);
      const session = new PersistentPhysicsWorld(this, physics, dimension, options, interval, snapshot !== undefined, () => active!.delete(this.key));
      active.set(this.key, session);
      return session;
    } catch (error) { if (active.get(this.key) === pending) active.delete(this.key); throw error; }
  }
}

/** 管理世界与保存定时器的生命周期。close 默认先保存，失败则保留当前世界供重试。 */
export class PersistentPhysicsWorld {
  private timer: number | undefined;
  private closed = false;
  private readonly shutdown = (): void => {
    if (this.closed || !this.current.isValid) return;
    // 退出回调处于受限执行环境：只捕获内存物理状态并写动态属性，不销毁世界或调度后续任务。
    try { this.save(); }
    catch (error) {
      if (this.options.onError) this.options.onError(error);
      else throw error; // MC 记录回调错误；已有已提交的存档仍可读取。
    }
  };
  private watchingShutdown = false;

  /** 最近成功保存时的世界 revision，-1 表示尚未与存档同步。 */
  private savedRevision: number;

  /** @internal 通过 PhysicsWorldStore.open 创建。loaded 表示 current 刚从存档恢复，和存档一致 */
  constructor(
    private readonly store: PhysicsWorldStore, private current: PhysicsWorld, private readonly dimension: Dimension,
    private readonly options: PersistenceOptions, interval: number, loaded: boolean, private readonly release: () => void,
  ) {
    this.savedRevision = loaded ? current.revision : -1;
    try {
      if (options.saveOnShutdown ?? interval > 0) {
        system.beforeEvents.shutdown.subscribe(this.shutdown);
        this.watchingShutdown = true;
      }
      let elapsed = 0;
      if (interval > 0) this.timer = system.runInterval(() => {
        if (!this.current.isValid) { this.close(false); return; }
        if (++elapsed < interval) return;
        elapsed = 0;
        try {
          if (this.unchanged()) {
            this.options.onSave?.({ changed: false, elapsedMs: 0, bodyCount: this.current.bodyCount, jointCount: this.current.jointCount });
            return;
          }
          this.save();
        }
        catch (error) {
          if (this.options.onError) this.options.onError(error);
          else throw error; // MC 会把未捕获的回调错误写入 ContentLog；下个间隔仍会重试。
        }
      }, 1);
    } catch (error) {
      if (this.watchingShutdown) system.beforeEvents.shutdown.unsubscribe(this.shutdown);
      current.destroy(); throw error;
    }
  }

  /** 当前物理世界。reload 成功后替换为新对象，旧显示绑定和事件订阅不继承。 */
  get world(): PhysicsWorld { this.check(); return this.current; }
  private check(): void { if (this.closed) throw new Error("持久化会话已经关闭"); }

  /** 世界状态未变化时可跳过自动保存。指定 encodeUserData 时不使用此判断。 */
  private unchanged(): boolean {
    return this.current.revision === this.savedRevision && this.options.encodeUserData === undefined;
  }

  /** 立即保存；未变化时不写动态属性。 */
  save(): boolean {
    this.check();
    const revision = this.current.revision;
    const result = this.store.saveAs(this, this.current, this.options);
    this.savedRevision = revision;
    this.options.onSave?.(result);
    return result.changed;
  }

  /** 以最近保存的状态替换世界。新世界创建成功后销毁旧世界，旧 Body、Joint 和事件订阅随之失效。 */
  reload(): PhysicsWorld {
    this.check();
    const snapshot = this.store.read();
    if (snapshot === undefined) throw new Error("此存档尚未保存，或者记录已损坏（缺片、JSON 无法解析）");
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
  close(save = true): void {
    if (this.closed) return;
    if (save && this.current.isValid) this.save();
    if (this.timer !== undefined) system.clearRun(this.timer);
    if (this.watchingShutdown) {
      try { system.beforeEvents.shutdown.unsubscribe(this.shutdown); } catch { /* 退出回调里不允许退订，脚本即将结束，忽略 */ }
    }
    this.current.destroy();
    this.closed = true;
    this.release();
  }
}
