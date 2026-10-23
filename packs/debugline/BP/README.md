# 基于 Debugline 的物理场景演示

需要开启实验玩法。

```mcfunction
/cutlass
/cutlass stack
/cutlass clear
```

同一维度的场景累积保留，后续命令添加新场景，已有刚体继续参与模拟。

`clear` 清除当前维度的全部场景及其存档。

| 场景 | 内容 |
| --- | --- |
| `drop` | 盒子、球与胶囊自由落体 |
| `stack` | 盒子堆叠 |
| `wall` | 球撞击墙面 |
| `bounce` | 弹跳 |
| `chain` | 关节链 |
| `hinge` | 铰链 |
| `slope` | 斜面摩擦 |
| `shapes` | 圆柱、圆锥、凸包与复合形状 |
| `platform` | 运动平台 |
| `swarm` | 压力场景，同时产生60个刚体 |
| `beam` | 旋转长梁 |
| `blocks` | 不同材质的刚体 |
| `fragile` | 高速刚体与世界方块碰撞产生破碎 |
| `water` | 不同密度的刚体 |
