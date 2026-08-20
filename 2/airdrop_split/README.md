# 空投仿真 MATLAB 拆分版说明

本目录由 `../airdrop_simulation.m` 拆分而来，原始文件未修改。拆分目标是让迁移时可以逐阶段阅读和改写，每个 `.m` 文件只保留一个顶层函数。

## 文件结构

| 文件 | 职责 |
|---|---|
| `airdrop_simulation.m` | 顶层编排：初始化参数、串联各阶段、拼接 `time_all/data_all` |
| `stage_moving.m` | 出舱滑动阶段 |
| `stage_revote.m` | 出舱翻转阶段 |
| `jiansu_straight.m` | 减速伞拉直阶段 |
| `jiansu_chongqi.m` | 减速伞充气阶段 |
| `zhusan_straight.m` | 主伞拉直阶段 |
| `zhusan_chongqi.m` | 主伞充气阶段 |
| `sim_wj.m` | 主伞充气结束后的稳降阶段 |
| `ode45_stage1.m` | 出舱滑动 ODE 右端 |
| `ode45_stage2.m` | 出舱翻转 ODE 右端 |
| `ode45_stage3.m` | 减速伞拉直 ODE 右端 |
| `ode45_stage4.m` | 减速伞充气 ODE 右端 |
| `ode45_stage5.m` | 主伞拉直 ODE 右端 |
| `ode45_stage6.m` | 主伞充气 ODE 右端 |
| `dynsys.m` | 稳降阶段 10 状态动力学方程 |

## 调用顺序

```text
airdrop_simulation
├─ stage_moving       -> ode45_stage1
├─ stage_revote       -> ode45_stage2
├─ jiansu_straight    -> ode45_stage3
├─ jiansu_chongqi     -> ode45_stage4
├─ zhusan_straight    -> ode45_stage5
├─ zhusan_chongqi     -> ode45_stage6
└─ sim_wj             -> dynsys
```

## 迁移阅读顺序

1. 先读 `airdrop_simulation.m` 前 130 行，理解输入向量如何映射为全局结构体。
2. 再按调用顺序逐个迁移阶段函数。每个阶段函数负责设置初值、调用 `ode45`、查找阶段终止点、整理阶段输出。
3. 每迁移一个阶段，再迁移对应的 `ode45_stage*.m`。ODE 函数通常只依赖全局结构体和当前状态 `y`。
4. 最后迁移 `sim_wj.m` 和 `dynsys.m`，因为稳降阶段包含体轴系到空间系转换，变量含义和坐标轴重映射更容易出错。
5. 回到 `airdrop_simulation.m` 末尾，迁移各阶段输出拼接逻辑，保持最终 `data_all` 的 9 列顺序不变。

## 关键全局变量

| 全局变量 | 作用 |
|---|---|
| `drop_envi` | 环境参数：海拔、重力、空气密度、投放高度、飞机速度、风速、温度 |
| `Vehicle` | 空降车质量、惯量、几何尺寸、质心、牵引点、摩擦和阻力系数 |
| `pullchute` | 牵引伞参数 |
| `decechute` | 减速伞参数 |
| `mainchute` | 主伞参数 |
| `sim` | 仿真总时长和采样步长 |
| `data_stage_3` | 减速伞拉直终止状态，供 `ode45_stage4` 使用 |
| `data_stage_55` | 主伞拉直相关状态，供 `ode45_stage6` 使用 |
| `t21` | 主伞第二阶段充气时长，由 `zhusan_chongqi` 写入、`ode45_stage6` 读取 |

迁移到 Python 时建议把这些全局变量改为一个上下文对象，例如 `SimulationContext`，再显式传入阶段函数和 ODE 右端函数。

## 输出列定义

`data_all` 实际返回 9 列：

| 列 | 含义 |
|---:|---|
| 1 | X 向水平位移 |
| 2 | Y 向垂向位移/高度历程 |
| 3 | Z 向侧向位移 |
| 4 | X 向水平速度 |
| 5 | Y 向垂直速度 |
| 6 | Z 向侧向速度 |
| 7 | 俯仰角 |
| 8 | 侧倾角 |
| 9 | 牵引绳/开伞过载 |

源码开头旧注释提到 13 列和 `time_max/data_max`，但当前函数实际只返回 `time_all` 和 9 列 `data_all`。迁移时应先保持实际行为。

## 需要重点核查

- `decechute.aera`、`mainchute.ChuteAera` 是源码拼写，迁移时可保留兼容字段名，也可统一为 `area` 并在输入映射层处理。
- `ode45_stage4` 是减速伞充气方程，但内部读取 `mainchute.k_ks`。这与主流程中准备的 `decechute.ks` 不完全一致，建议先按原行为迁移，再确认物理参数来源。
- `zhusan_straight` 中 `data_out(:,9) = FL(n,1)` 会把终止点拉直力填充整列，可能是原代码问题。迁移时若要严格对齐旧结果，需要保留；若要修正，需要单独做回归对比。
- 多个阶段通过后处理循环寻找终止点，没有找到时可能导致未初始化变量。迁移语言中应显式抛错，方便定位输入或模型问题。
- MATLAB 里同时使用 `sin/cos` 和 `sind/cosd`。前者是弧度，后者是角度；稳降阶段尤其要核对单位。
