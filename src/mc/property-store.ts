// 存在世界动态属性里的一条 JSON 记录：按代次分片，由 `_active` 一次提交。
import { world } from "@minecraft/server";

const DEFAULT_CHUNK_SIZE = 30000;

/** 世界动态属性的存储接口。 */
export interface DynamicPropertyTarget {
  getDynamicProperty(identifier: string): boolean | number | string | object | undefined;
  setDynamicProperty(identifier: string, value?: boolean | number | string | object): void;
}

/** 保存结果：已写入或内容未变化。写入失败直接抛错。 */
export type DynamicPropertySaveResult = "saved" | "unchanged";

/** `_active` 的内容：小记录是 json；大记录是代次、片数和总长度（UTF-16 单位） */
interface ActiveManifest {
  byteCount?: number;
  chunkCount?: number;
  generation?: string;
  json?: string;
}

/**
 * 按代次分片的 JSON 记录，以 <prefix>_active 为提交点。
 * 小记录直接写入活动属性；大记录先写分片，再切换活动属性。缺片或解析失败返回无记录。
 * owned 模式下，活动属性未变化时复用 JSON 缓存。
 */
export class JsonPropertyStore {
  private lastJson: string | undefined;
  private owned: boolean;
  private loaded = false;
  private manifestLoaded = false;
  private manifestValue: unknown;
  private activeManifest: ActiveManifest | undefined;
  /** 上次读 `_active` 时它是否存在；owned 时 hasStoredValue 直接用它 */
  private stored = false;
  private nextGeneration = 1;

  constructor(readonly prefix: string, readonly chunkSize = DEFAULT_CHUNK_SIZE, readonly target: DynamicPropertyTarget = world, owned = false) {
    this.owned = owned;
  }

  /** `_active` 是否存在（格式不对的也算） */
  get hasStoredValue(): boolean {
    if (this.owned && this.manifestLoaded) return this.stored;
    return this.target.getDynamicProperty(this.manifestKey()) !== undefined;
  }

  /**
   * 读出记录。没有记录、缺片、长度不对、JSON 解析失败（以及读取时抛错）都返回 undefined。
   * raw 为 true 时返回 JSON 原文；owned 为 true 时从此按 owned 处理；fresh 为 true 时丢掉缓存，重新读所有分片。
   */
  load<T>(raw?: false, owned?: boolean, fresh?: boolean): T | undefined;
  load(raw: true, owned?: boolean, fresh?: boolean): string | undefined;
  load<T>(raw: boolean, owned?: boolean, fresh?: boolean): T | string | undefined;
  load<T>(raw = false, owned = false, fresh = false): T | string | undefined {
    this.owned ||= owned;
    if (fresh) {
      this.loaded = this.manifestLoaded = false;
      this.lastJson = undefined;
    }
    try {
      const manifest = this.getActiveManifest();
      if (this.owned && this.loaded) return this.lastJson === undefined ? undefined : raw ? this.lastJson : JSON.parse(this.lastJson);
      if (!manifest) {
        this.lastJson = undefined;
        this.loaded = true;
        return undefined;
      }
      let json = manifest.json ?? "";
      if (manifest.generation) {
        for (let index = 0; index < manifest.chunkCount!; index++) {
          const chunk = this.target.getDynamicProperty(this.chunkKey(manifest.generation, index));
          if (typeof chunk !== "string") return undefined;
          json += chunk;
        }
        if (json.length !== manifest.byteCount) return undefined;
      }
      const value = raw ? json : JSON.parse(json);
      this.lastJson = json;
      this.loaded = true;
      return value;
    } catch {
      return undefined;
    }
  }

  /**
   * 保存 JSON 值；内容相同时返回 unchanged 并跳过写入。
   * 写入失败时抛出原始错误，保留旧活动记录和内存状态。
   */
  saveWithResult<T>(value: T): DynamicPropertySaveResult {
    const json = JSON.stringify(value);
    if (json === this.lastJson) return "unchanged";
    const previous = this.owned && this.manifestLoaded ? this.activeManifest : this.getActiveManifest();
    let manifest: ActiveManifest, manifestValue: string;
    // 一个 UTF-16 单位转成 UTF-8 最多 3 个字节：放得进一片的小记录直接存在 _active 里，一次写入；大记录走分片
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
        generation,
      };
      manifestValue = serializeManifest(manifest);
    }
    this.target.setDynamicProperty(this.manifestKey(), manifestValue);
    this.lastJson = json;
    this.manifestValue = manifestValue;
    this.activeManifest = manifest;
    this.manifestLoaded = this.loaded = this.stored = true;
    if (previous?.generation) {
      this.tryDeleteGeneration(previous.generation, previous.chunkCount!);
    }
    return "saved";
  }

  /** 删除记录（`_active` 和当前一代的分片）。删 `_active` 出错时返回 false，记录保持原样 */
  clear(): boolean {
    let manifest: ActiveManifest | undefined;
    try {
      manifest = this.owned && this.manifestLoaded ? this.activeManifest : this.getActiveManifest();
      if (this.owned && !this.stored) return true;
      this.target.setDynamicProperty(this.manifestKey());
      this.lastJson = undefined;
      this.manifestValue = undefined;
      this.activeManifest = undefined;
      this.manifestLoaded = this.loaded = true;
      this.stored = false;
    } catch {
      return false;
    }
    if (manifest?.generation) this.tryDeleteGeneration(manifest.generation, manifest.chunkCount!);
    return true;
  }

  /** 尽量删掉已经被取代的一代分片 */
  private tryDeleteGeneration(generation: string, chunkCount: number): void {
    try {
      for (let index = 0; index < chunkCount; index++) {
        this.target.setDynamicProperty(this.chunkKey(generation, index));
      }
    } catch {
      // 删不掉的旧分片不影响已经提交的 _active
    }
  }

  private createGeneration(): string {
    return `${Date.now().toString(36)}_${(this.nextGeneration++).toString(36)}`;
  }

  private getActiveManifest(): ActiveManifest | undefined {
    const raw = this.target.getDynamicProperty(this.manifestKey());
    if (this.owned && this.manifestLoaded && raw === this.manifestValue) return this.activeManifest;
    this.loaded = false;
    this.lastJson = undefined;
    this.manifestValue = raw;
    const manifest = typeof raw === "string" ? parseManifest(raw) : undefined;
    this.activeManifest = manifest;
    this.manifestLoaded = true;
    this.stored = raw !== undefined;
    return manifest;
  }

  private manifestKey(): string {
    return `${this.prefix}_active`;
  }

  private chunkKey(generation: string, index: number): string {
    return `${this.prefix}_g_${generation}_${index}`;
  }
}

function serializeManifest(manifest: ActiveManifest): string {
  return [
    manifest.generation,
    manifest.chunkCount,
    manifest.byteCount,
  ].join("|");
}

function parseManifest(value: string): ActiveManifest | undefined {
  if (value[0] === "=") return { json: value.slice(1) };
  const [generation, chunks, bytes, extra] = value.split("|");
  const chunkCount = Number.parseInt(chunks ?? "", 10);
  const byteCount = Number.parseInt(bytes ?? "", 10);
  if (!generation || extra !== undefined || !Number.isFinite(chunkCount) || !Number.isFinite(byteCount) || chunkCount < 1 || byteCount < 0) return undefined;
  return { byteCount, chunkCount, generation };
}

// 输入为非空 JSON 字符串，分片大小为正整数。
// 按 UTF-16 单位切，代理对可能被切到相邻两片里
function splitIntoChunks(value: string, size: number): string[] {
  const chunks: string[] = [];
  for (let index = 0; index < value.length; index += size) {
    chunks.push(value.slice(index, index + size));
  }
  return chunks;
}
