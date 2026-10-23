// 事件信号的实现。公开的类型见 events.ts；这里的内容不对外导出。
import type {
  BlockBreakEvent, BlockCollisionStartEvent, BodyCollisionEndEvent, BodyCollisionStartEvent, BodySleepEvent, FluidEntryEvent, JointBreakEvent, Signal, StepEndEvent,
} from "./events.js";

/** 订阅者列表。触发时先复制一份，订阅者在回调里订阅或取消订阅不影响本次触发 */
export class SignalImpl<T> implements Signal<T> {
  private readonly callbacks: ((event: T) => void)[] = [];

  /** 是否存在订阅者，用于跳过无订阅时的事件数据构造。 */
  get hasSubscribers(): boolean {
    return this.callbacks.length > 0;
  }

  subscribe(callback: (event: T) => void): (event: T) => void {
    if (typeof callback !== "function") throw new Error("订阅事件的回调必须是函数");
    if (!this.callbacks.includes(callback)) this.callbacks.push(callback);
    return callback;
  }

  unsubscribe(callback: (event: T) => void): void {
    const i = this.callbacks.indexOf(callback);
    if (i >= 0) this.callbacks.splice(i, 1);
  }

  clear(): void {
    this.callbacks.length = 0;
  }

  /** 依次调用所有订阅者。单个回调错误不阻断其余回调，全部调用结束后抛出首个错误。 */
  emit(event: T): void {
    if (this.callbacks.length === 0) return;
    let failed = false;
    let firstError: unknown;
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
}

/** 物理世界的全部事件信号 */
export function createAfterEvents() {
  return {
    bodyCollisionStart: new SignalImpl<BodyCollisionStartEvent>(),
    bodyCollisionEnd: new SignalImpl<BodyCollisionEndEvent>(),
    blockCollisionStart: new SignalImpl<BlockCollisionStartEvent>(),
    bodySleep: new SignalImpl<BodySleepEvent>(),
    bodyWake: new SignalImpl<BodySleepEvent>(),
    stepEnd: new SignalImpl<StepEndEvent>(),
    jointBreak: new SignalImpl<JointBreakEvent>(),
    waterEntry: new SignalImpl<FluidEntryEvent>(),
    lavaEntry: new SignalImpl<FluidEntryEvent>(),
    blockBreak: new SignalImpl<BlockBreakEvent>(),
  };
}
