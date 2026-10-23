# CutlassPhysics

一个用于 Minecraft 基岩版 Script API 的 3D 刚体物理库

[API 文档](https://mcscpcb.github.io/CutlassPhysics/)

## 特性

- **方块碰撞** — 支持楼梯、栅栏、门等方块的碰撞检测与响应
- **刚体形状** — 支持盒子、球、胶囊、圆柱、圆锥，以及凸包和复合形状
- **物理属性** — 可配置密度、摩擦、弹性等物理参数
- **关节约束** — 支持铰链、滑块、布娃娃等关节，可配置弹簧、马达和断开阈值
- **流体交互** — 模拟水和岩浆的浮力、阻力与流动作用
- **存档管理** — 保存和恢复刚体与关节的状态

## 如何使用

```bash
npm install cutlass-physics
```

```ts
import { world } from "@minecraft/server";
import { PhysicsWorld } from "cutlass-physics";

const physics = new PhysicsWorld(world.getDimension("overworld"));

const crate = physics.createBody({
  shapes: [{ type: "box", size: { x: 1, y: 1, z: 1 } }],
  location: { x: 0, y: 80, z: 0 },
});

physics.afterEvents.stepEnd.subscribe(() => {
  console.warn(crate.location.y);
});
```

## 兼容性

使用正式版 `@minecraft/server 2.10.0` 模块

## 许可证

[MIT](LICENSE)
