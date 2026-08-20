# MATLAB 空投仿真代码迁移分析

源文件：`2/airdrop_simulation.m`

该文件定义了一个顶层函数 `airdrop_simulation` 和一组本地阶段函数，用于计算空降车从牵引出舱、减速伞打开、主伞打开到稳降过程的时间历程。代码以 MATLAB 的 `ode45` 作为核心积分器，阶段之间通过上一阶段终止状态作为下一阶段初值串联。

> 注意：源码注释编码为 GB2312/GBK。按 UTF-8 直接读取会乱码，迁移前建议先统一转为 UTF-8。

## 已生成的拆分版代码

拆分版位于：`2/airdrop_split/`

该目录保留原 MATLAB 行为，不修改原始 `2/airdrop_simulation.m`。每个原本的本地函数已拆成单独 `.m` 文件，并在函数签名后补充了迁移友好的说明注释。建议迁移时以 `2/airdrop_split/README.md` 作为入口，按调用顺序逐个阅读和改写：

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

拆分版文件使用 UTF-8 编码，便于后续在 Python、C++ 或其他语言迁移时直接查看中文注释。

## 顶层接口

```matlab
function [time_all,data_all] = airdrop_simulation(kte_p,vel_p,qys_p,jss_p,zhs_p,sim_p)
```

### 输入参数

所有输入都是定长数值数组，函数内部再拆解到全局结构体中。

#### `kte_p`：空投环境参数，长度 9

| 下标 | 字段 | 含义 | 单位/说明 |
|---:|---|---|---|
| 1 | `Land_hight` | 着陆场海拔 | m |
| 2 | `drophight` | 空投高度 | m |
| 3 | `AirplaneVelo` | 运输机航速 | km/h，后续多处除以 3.6 转为 m/s |
| 4 | `Cargo_angle` | 货舱与水平面夹角 | deg |
| 5 | `LoadPosi_x` | 空降车货舱内 X 装载坐标 | 代码取 `abs(kte_p(5))` |
| 6 | `LoadPosi_y` | 空降车货舱内 Y 装载坐标 | m |
| 7 | `wind_x` | 水平风速 | m/s |
| 8 | `wind_y` | 侧向风速 | m/s |
| 9 | `T` | 地面温度 | degC |

函数还根据海拔和温度计算：

- `Grav`：重力加速度。
- `Earth_R`：地球半径，固定为 `6370000`。
- `AirDensity`：空气密度，使用指数大气模型。

#### `vel_p`：空降车参数，长度 19

| 下标 | 字段 | 含义 | 单位/说明 |
|---:|---|---|---|
| 1 | `dropmass` | 空投总质量 | t，力学方程中通常乘 `1000` 转 kg |
| 2 | `Izz` | Z 向转动惯量 |  |
| 3 | `Ixx` | X 向转动惯量 | 当前代码基本未使用 |
| 4 | `hull_cmx` | 质心 X 坐标 | 代码赋值为 `-vel_p(4)` |
| 5 | `hull_cmy` | 质心 Y 坐标 |  |
| 6 | `hull_L` | 车体长度 | m |
| 7 | `hull_W` | 车体宽度 | m |
| 8 | `hull_H` | 车体高度 | m |
| 9 | `cmpull_y` | 吊钩与重心的垂直距离 | m |
| 10 | `cmpull_H` | 吊钩与吊点的垂直距离 | m |
| 11 | `cmpull_L` | 吊钩与重心的水平距离 | m |
| 12 | `pull_x` | 牵引点 X 坐标 | 代码赋值为 `-vel_p(12)` |
| 13 | `pull_y` | 牵引点 Y 坐标 |  |
| 14 | `track_fdx` | 履带前端 X 坐标 |  |
| 15 | `track_fdy` | 履带前端 Y 坐标 |  |
| 16 | `track_bdx` | 履带后端 X 坐标 |  |
| 17 | `track_bdy` | 履带后端 Y 坐标 |  |
| 18 | `resist_hull` | 车体空气阻力系数 |  |
| 19 | `mu` | 车体与货舱地板滑动摩擦系数 |  |

#### `qys_p`：牵引伞参数，长度 4

| 下标 | 字段 | 含义 |
|---:|---|---|
| 1 | `mass` | 牵引伞质量 |
| 2 | `resist` | 牵引伞阻力系数 |
| 3 | `area` | 牵引伞特征面积 |
| 4 | `RopeLength` | 牵引伞伞绳长度 |

#### `jss_p`：减速伞参数，长度 11

| 下标 | 字段 | 含义 |
|---:|---|---|
| 1 | `mass` | 减速伞伞衣质量 |
| 2 | `resist` | 减速伞阻力系数 |
| 3 | `aera` | 减速伞等效面积，源码拼写为 `aera` |
| 4 | `ChuteWidth` | 伞边宽度 |
| 5 | `ChuteSideMass` | 伞边质量 |
| 6 | `RopeNum` | 伞绳数量 |
| 7 | `RopeLength` | 伞绳长度 |
| 8 | `RopeMass` | 伞绳质量 |
| 9 | `kf` | 附加质量系数 |
| 10 | `ks` | 充气系数 |
| 11 | `ksn` | 开伞充气指数，当前主流程未直接使用 |

#### `zhs_p`：主伞参数，长度 19

| 下标 | 字段 | 含义 |
|---:|---|---|
| 1 | `ChuteAera` | 主伞特征面积，源码拼写为 `Aera` |
| 2 | `ChuteNum` | 主伞数量 |
| 3 | `ChuteMass` | 主伞伞衣质量 |
| 4 | `SideWidth` | 主伞伞边宽度 |
| 5 | `SideMass` | 主伞伞边质量 |
| 6 | `resist_ca` | 主伞法向阻力系数 |
| 7 | `resist_cn` | 主伞周向阻力系数 |
| 8 | `resist_cm` | 主伞径向阻力系数 |
| 9 | `ChuteCoverResist` | 伞衣套阻力系数 |
| 10 | `PulloutResist` | 已拉出物体阻力系数 |
| 11 | `PulloutForce` | 拉出阻力 |
| 12 | `RopeNum` | 主伞伞绳数量 |
| 13 | `RopeMass` | 主伞伞绳质量 |
| 14 | `RopeLength` | 主伞伞绳长度 |
| 15 | `kf` | 主伞附加质量系数 |
| 16 | `khalf` | 主伞收口比 |
| 17 | `k_ks` | 主伞第一次充气参数 |
| 18 | `n_ks` | 第二次充气参数 |
| 19 | `StepOpen` | 两次充气间隔时间 |

#### `sim_p`：仿真控制参数，长度 2

| 下标 | 字段 | 含义 |
|---:|---|---|
| 1 | `time` | 仿真时间上限 |
| 2 | `step` | 固定采样步长 |

## 输出参数

### `time_all`

列向量，表示完整仿真时间轴。前半段按固定步长 `sim.step` 拼接出舱、拉直、充气过程；稳降段使用 `ode45` 自适应时间并加上前半段最大时间作为偏移。

### `data_all`

源码实际返回 9 列：

| 列 | 含义 | 单位 |
|---:|---|---|
| 1 | `Xx_all`，X 向水平位移 | m |
| 2 | `Yy_all`，Y 向垂向位移/高度历程 | m |
| 3 | `Zz_all`，Z 向侧向位移 | m |
| 4 | `Vx_all`，X 向水平速度 | m/s |
| 5 | `Vy_all`，Y 向垂直速度 | m/s |
| 6 | `Vz_all`，Z 向侧向速度 | m/s |
| 7 | `theta_all`，俯仰角 | deg |
| 8 | `beta_all`，侧倾角 | deg |
| 9 | `Force_all`，牵引绳/开伞过载 | g |

源码开头注释里提到 `data_all` 有 13 列，并提到 `time_max/data_max`，但当前函数签名和实际拼接结果只返回 `time_all`、`data_all`，且 `data_all` 只有 9 列。Python 迁移时应以实际代码行为为准，或先确认业务是否仍需要旧版 13 列输出。

## 全局状态

代码大量依赖 MATLAB `global`：

- `drop_envi`：环境参数。
- `Vehicle`：空降车参数。
- `pullchute`：牵引伞参数。
- `decechute`：减速伞参数。
- `mainchute`：主伞参数。
- `sim`：仿真时间和步长。
- `data_stage_3`：减速伞拉直终止状态，供 `ode45_stage4` 使用。
- `data_stage_55`：主伞拉直前/相关状态，供 `ode45_stage6` 使用。
- `t21`：主伞二次充气时间，先在 `zhusan_chongqi` 中计算，再由 `ode45_stage6` 使用。

迁移到 Python 时建议用 `dataclass` 或普通类封装上下文，避免全局变量。例如将所有结构体放入 `SimulationContext`，并把上下文显式传给 ODE 右端函数。

## 主流程结构

顶层函数按以下阶段串行计算：

1. `stage_moving`：出舱滑动阶段。
2. `stage_revote`：出舱翻转阶段。
3. `jiansu_straight`：减速伞系统拉直阶段。
4. `jiansu_chongqi`：减速伞充气阶段。
5. `zhusan_straight`：主伞系统拉直阶段。
6. `zhusan_chongqi`：主伞充气阶段。
7. `sim_wj`：稳降阶段，含风场影响和三维位移/速度转换。
8. 顶层拼接各阶段输出，形成 `time_all` 和 `data_all`。

调用关系如下：

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

## 阶段函数说明

### `stage_moving(para_pdong)`

功能：模拟空降车在货舱内受牵引伞拉力后的滑动出舱过程。

实际主要依赖全局 `Vehicle/drop_envi/sim`，传入的 `para_pdong` 基本未使用。

ODE 状态量：

| 状态 | 含义 |
|---:|---|
| `y(1)` | X 向位移 |
| `y(2)` | X 向速度 |
| `y(3)` | Y 向位移 |
| `y(4)` | Y 向速度 |

终止条件：循环查找第一个满足 `data_1(i,1) + drop_envi.LoadPosi_x <= 0` 的点，认为滑动阶段结束。

输出：

- `time_pd`：滑动阶段终止时间。
- `data_pd`：终止状态 `[x, vx, y, vy]`。
- `time_1/data_1`：完整积分时间和状态。

### `stage_revote(data_stage_1, para_fanzh)`

功能：模拟车辆后续绕履带/支点翻转出舱过程。

ODE 状态量：

| 状态 | 含义 |
|---:|---|
| `y(1)` | 相对 X 向位移 |
| `y(2)` | X 向速度 |
| `y(3)` | Y 向位移 |
| `y(4)` | Y 向速度 |
| `y(5)` | 俯仰角 |
| `y(6)` | 俯仰角速度 |

终止条件：计算后端质心相关坐标 `x_cm_back`，第一个 `x_cm_back <= 0` 的点为结束点。

输出 `data_fz` 共 6 列/项：

`[X位移, X速度, Y位移, Y速度, 俯仰角, 俯仰角速度]`

### `jiansu_straight(data_stage_2, js_strait_para)`

功能：模拟减速伞从开始拉出到伞系统拉直的过程。核心包括牵引伞、伞衣套、已拉出物体阻力、伞绳/伞衣分段质量释放，以及车辆俯仰角积分。

ODE 状态量：

| 状态 | 含义 |
|---:|---|
| `y(1)` | 车辆 X 向位移 `Xd` |
| `y(2)` | 车辆 Y 向位移 `Yd` |
| `y(3)` | 轨迹角/姿态角 |
| `y(4)` | 降落伞速度 `Vys` |
| `y(5)` | 空投车辆速度 `Vw` |
| `y(6)` | 伞绳拉出长度 |

终止条件：伞绳拉出长度 `Lx >= Lxt`，其中 `Lxt = 减速伞绳长 + 伞边宽度 + 伞衣名义直径/2`。

输出 `data_jslz` 共 9 项：

`[Xd, Yd, 轨迹角, Vys, Vw, 拉出长度, 俯仰角, 俯仰角速度, 拉直力FL]`

### `jiansu_chongqi(data_stage_3, js_chuqi_para)`

功能：模拟减速伞充气过程。充气时间按

```text
t1 = k_ks * sqrt(CAjst) / VL1
```

估算，其中 `CAjst = 减速伞阻力系数 * 减速伞面积`，`VL1` 来自上一阶段车辆速度。

ODE 状态量：

| 状态 | 含义 |
|---:|---|
| `y(1)` | 车辆 X 向位移 |
| `y(2)` | 车辆 Y 向位移 |
| `y(3)` | 轨迹角 |
| `y(4)` | 车辆速度 |

终止条件：`time_4 >= t1`。

输出 `data_jscq` 共 7 项：

`[Xd, Yd, 轨迹角, Vw, 俯仰角, 俯仰角速度, 开伞力Fk]`

### `zhusan_straight(data_stage_2, ms_strait_para, data_jslz)`

功能：模拟主伞系统拉直过程。该阶段会把减速伞系统长度纳入总拉出长度，质量释放模型分为减速伞绳、减速伞伞边、减速伞伞衣、主伞绳、主伞伞边、主伞伞衣多个区间。

ODE 状态量与 `jiansu_straight` 类似：

| 状态 | 含义 |
|---:|---|
| `y(1)` | 车辆 X 向位移 |
| `y(2)` | 车辆 Y 向位移 |
| `y(3)` | 轨迹角 |
| `y(4)` | 降落伞速度 |
| `y(5)` | 空投车辆速度 |
| `y(6)` | 伞系统拉出长度 |

终止条件：`Lx >= Lxt`，这里的 `Lxt = 主伞系统长度 + 减速伞系统长度`。

输出 `data_mslz` 共 9 项：

`[Xd, Yd, 轨迹角, Vys, Vw, 拉出长度, 俯仰角, 俯仰角速度, 拉直力FL]`

### `zhusan_chongqi(data_stage_4, ms_chuqi_para)`

功能：模拟主伞两阶段充气过程。

充气时间计算：

```text
t1 = k_ks * sqrt(khalf * CAms) / VL1
t2 = t1 + step_open
t21 = n_ks * (sqrt(CAms) - sqrt(khalf * CAms)) / VL2
t3 = t2 + t21
```

其中 `t21` 被写入全局变量，在 `ode45_stage6` 内用于主伞二次充气面积增长。

ODE 状态量：

| 状态 | 含义 |
|---:|---|
| `y(1)` | 车辆 X 向位移 |
| `y(2)` | 车辆 Y 向位移 |
| `y(3)` | 轨迹角 |
| `y(4)` | 车辆速度 |

终止条件：源码先积分到 `sim_time`，再通过 `diff(Fk)` 从 `8/step` 之后寻找第一个 `abs(diff(Fk)) <= 1` 的点作为主伞充气稳定结束时间。

输出 `data_mscq`：

`[Xd, Yd, 轨迹角, Vw, Fk, 俯仰角, 俯仰角速度]`

### `sim_wj(data_mscq, wj_para, data_66)`

功能：模拟主伞打开后的稳降过程，加入水平风和侧向风，并将体轴系状态转换到空间坐标系。

输入：

- `data_mscq`：主伞充气结束状态。
- `wj_para = [wind_x, wind_y, wind_z]`，顶层传入 `wind_z = 0`。
- `data_66`：主伞充气阶段速度/轨迹角历程，用最后一点初始化稳降速度分量。

内部状态 `Y0` 共 10 项：

```text
[beta, theta, u, w, x_body, vx_body, che_vx, che_vy, che_x, che_y]
```

`dynsys` 根据姿态角、风速、伞群气动阻力、车辆阻力和摆动方程计算状态导数，同时返回一个轴向气动载荷 `ax_body_1`。

稳降积分时间固定为 `[0, 200]`，循环中保留高度未超过投放高度条件的数据：

```matlab
if -weyi(3,1) + Z <= params.H
    ...
end
```

输出：

- `time_wj`：稳降阶段时间。
- `data_wj`：稳降内部变量，至少包含侧倾角、俯仰角、角速度、体轴位移/速度、过载等。
- `Xx/Yy/Zz`：稳降段空间位移。
- `Vx/Vy/Vz`：稳降段空间速度。

## ODE 方程对应关系

| ODE 函数 | 阶段 | 状态维度 | 主要物理量 |
|---|---|---:|---|
| `ode45_stage1` | 出舱滑动 | 4 | 牵引伞阻力、货舱法向力、摩擦、重力 |
| `ode45_stage2` | 出舱翻转 | 6 | 牵引伞阻力、法向力、摩擦、绕支点转动 |
| `ode45_stage3` | 减速伞拉直 | 6 | 牵引伞/伞衣套/车体阻力、已拉出质量、拉出力 |
| `ode45_stage4` | 减速伞充气 | 4 | 减速伞面积增长、附加质量、车体阻力 |
| `ode45_stage5` | 主伞拉直 | 6 | 减速伞和主伞复合拉出质量、主伞绳/伞衣分段释放 |
| `ode45_stage6` | 主伞充气 | 4 | 主伞二阶段面积增长、减速伞阻力、附加质量 |
| `dynsys` | 稳降 | 10 | 风场、姿态摆动、伞群三向阻力、体轴/空间坐标转换 |

## 数据拼接逻辑

顶层函数将出舱、减速伞、主伞阶段的数据按列统一后拼接：

- `KjVehicle_Dx`：由出舱 X 位移取负，再拼接各伞阶段 X 位移。
- `KjVehicle_Dy`：拼接出舱 Y 位移和各伞阶段 Y 位移。
- `KjVehicle_Velo_x/y`：使用速度和轨迹角分解为水平、垂向速度。
- `Vehicle_Chute_Angle`：出舱阶段补零，伞阶段使用轨迹角。
- `Vehicle_Pitch_Angle`、`Vehicle_Pitch_Anglew`：拼接俯仰角和角速度。
- `Chute_Rope_Force`：出舱阶段补零，伞阶段使用拉直力或开伞力。

随后再拼接稳降段：

```text
Vx_all = [开伞前X速度; 稳降X速度]
Vy_all = [开伞前Y速度; 稳降Z方向速度]
Vz_all = [开伞前0;      稳降Y方向速度]

Xx_all = [开伞前X位移; 稳降X位移]
Yy_all = [drop_hight - 开伞前Y位移; drop_hight - 稳降Z位移]
Zz_all = [开伞前0; 稳降Y位移]
```

这里存在坐标轴重映射：稳降函数内部的 `Y/Z` 与最终 `data_all` 的垂向/侧向列并不是一一同名直传，迁移时要按最终拼接公式保持一致。

## Python 迁移建议

1. 用 `numpy.ndarray` 替代 MATLAB 数组，注意 MATLAB 是 1-based 索引，Python 是 0-based。
2. 用 `scipy.integrate.solve_ivp` 替代 `ode45`。如果需要固定输出步长，使用 `t_eval=np.arange(0, sim_time + step, step)`。
3. MATLAB 三角函数混用弧度制和角度制：
   - `sin/cos/tan/atan` 使用弧度。
   - `sind/cosd` 使用角度。
   - `data_all` 中俯仰角、侧倾角最终为角度。
4. 不要直接照搬 `global`。建议定义：

```python
@dataclass
class SimulationContext:
    env: DropEnvironment
    vehicle: Vehicle
    pullchute: PullChute
    decechute: DeceChute
    mainchute: MainChute
    sim: SimConfig
```

5. 对每个阶段函数保持“输入上一阶段终止状态，返回终止状态和阶段历程”的接口，这样最接近当前 MATLAB 结构。
6. 对阶段终止条件建议用两层实现：
   - 先按 MATLAB 行为用 `t_eval` 后处理查找终止点，便于对齐旧结果。
   - 后续再考虑用 `solve_ivp` 的 event 函数精确终止。
7. 迁移测试应固定一组源码注释里的默认参数，比较：
   - `time_all` 长度和最后时间。
   - `data_all` 每列最大/最小值。
   - 各阶段终止时间。
   - 关键列误差容忍，例如 `rtol=1e-5` 或按业务接受范围设定。

## 需要重点核查的源码问题

- 顶层注释与实际输出不一致：注释提到 13 列和 `data_max/time_max`，实际只有 9 列且未返回最大值。
- `para_pdong` 在 `stage_moving` 中没有实际使用，函数改为直接读全局变量。
- `decechute.aera`、`mainchute.ChuteAera` 存在拼写错误，迁移时可以保留兼容字段名，也可以统一为 `area` 后加映射层。
- 多处变量名和单位注释混乱，例如部分距离注释写 `mm`，但默认值看起来按 m 使用。
- `zhusan_straight` 中 `data_out(:,9) = FL(n,1)` 会把终止点拉直力填满整列，而不是 `FL(1:n)`；这可能是 bug，迁移前应确认业务期望。
- `ode45_stage4` 使用 `k_ks = mainchute.k_ks`，但该阶段是减速伞充气，主流程参数中传入的是 `decechute.ks`。全局函数内又读了 `mainchute.k_ks`，存在参数来源不一致。
- `ode45_stage6` 中二次充气 `Dsm` 公式和面积增长公式不完全对应，`Dsm = 2.5 * ...` 看起来像经验系数或遗留写法，需要确认。
- 多个阶段通过循环后处理终止，如果没有找到终止点，变量 `n/nn/stop_time_i` 可能未初始化或为 0，Python 迁移时应显式处理异常。

## 建议的 Python 模块拆分

```text
airdrop/
├─ models.py          # dataclass 参数结构
├─ atmosphere.py      # 空气密度、重力、风场和单位转换
├─ stages.py          # stage_moving / stage_revote / parachute stages
├─ ode.py             # 各阶段右端函数
├─ steady_descent.py  # sim_wj / dynsys
└─ simulation.py      # airdrop_simulation 顶层编排和输出拼接
```

这种拆分能在保持 MATLAB 阶段边界的同时，减少全局状态和数组下标耦合，便于逐阶段做数值回归测试。
