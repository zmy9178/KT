# 前后端 Excel 交互流程设计

## 1. 目标

前端通过目录

```text
C:\Users\Tree\AppData\Roaming\KTData
```

与 Python 仿真程序交换 Excel 文件，不使用 HTTP 接口传递仿真参数和结果。

整个流程分为两个阶段：

1. 出舱阶段：读取第一阶段 Excel 中的 `vel_p`、`qys_p`、`sim_p`，输出六列出舱过程数据。
2. 后续空降阶段：读取第二阶段 Excel 中的 `kte_p`、`jss_p`、`zhs_p`，并与第一阶段数据合并后输出整个流程的九列数据。

本文档只描述数据协议和处理流程，暂不修改仿真代码。

## 2. 当前实际文件

当前目录中已经存在以下文件：

```text
C:\Users\Tree\AppData\Roaming\KTData\airdrop_input_first.xlsx
C:\Users\Tree\AppData\Roaming\KTData\airdrop_input_second.xlsx
C:\Users\Tree\AppData\Roaming\KTData\airdrop_output_first.xlsx
C:\Users\Tree\AppData\Roaming\KTData\airdrop_output_second.xlsx
```

当前 Excel 文件使用 `Sheet1`，参数名放在第一列，同一行后面依次存放参数值。
现在两个输入文件的职责已经调整为：

```text
airdrop_input_first.xlsx  -> vel_p, qys_p, sim_p
airdrop_input_second.xlsx -> kte_p, jss_p, zhs_p
```

## 3. 第一阶段：出舱阶段

### 3.1 输入文件

文件：

```text
C:\Users\Tree\AppData\Roaming\KTData\airdrop_input_first.xlsx
```

输入内容：

| 行标识 | 参数 |
|---|---|
| `vel_p` | 19 个参数 |
| `qys_p` | 4 个参数 |
| `sim_p` | 2 个参数 |

当前实际布局示例：

```text
vel_p, vel_p[0], vel_p[1], ..., vel_p[18]
qys_p, qys_p[0], qys_p[1], ..., qys_p[3]
sim_p, sim_p[0], sim_p[1]
```

参数数组含义沿用 MATLAB/Python 当前定义：

```text
vel_p = [车辆质量, 转动惯量, 车体惯量, 车体质心/几何参数,
         拉点位置, 履带接触点, 阻力系数和摩擦系数]

qys_p = [牵引伞质量, 牵引伞阻力系数, 牵引伞特征面积, 牵引伞伞绳长度]

sim_p = [仿真总时间, 输出时间步长]
```

### 3.2 第一阶段计算范围

第一阶段只计算出舱过程，包括：

1. 车辆在货舱内滑移；
2. 车辆绕接触位置转动并完成出舱。

对应现有 Python 结构中的：

```text
stage_moving()
stage_revote()
```

第一阶段输出应为从仿真开始到车辆完成出舱的时间序列，不包含减速伞和主伞阶段。

### 3.3 第一阶段输出文件

文件：

```text
C:\Users\Tree\AppData\Roaming\KTData\airdrop_output_first.xlsx
```

第一阶段输出六列，建议固定为：

| 列号 | 列名 | 物理意义 | 单位 |
|---:|---|---|---|
| 1 | `x_position` | 车辆 x 方向位移 | m |
| 2 | `y_position` | 车辆 y 方向位移 | m |
| 3 | `x_velocity` | 车辆 x 方向速度 | m/s |
| 4 | `y_velocity` | 车辆 y 方向速度 | m/s |
| 5 | `pitch_angle` | 车辆俯仰角 | deg |
| 6 | `roll_angle` | 车辆侧倾角 | deg |

建议第一行写列名，从第二行开始写数据：

```text
x_position,y_position,x_velocity,y_velocity,pitch_angle,roll_angle
...
```

第一阶段不需要在输出中重复写参数名，也不需要把内部状态变量如角速度、法向力写入前端结果表。

## 4. 第二阶段：后续空降阶段

### 4.1 输入文件

文件：

```text
C:\Users\Tree\AppData\Roaming\KTData\airdrop_input_second.xlsx
```

输入内容：

| 行标识 | 参数 |
|---|---|
| `vel_p` | 19 个参数 |
| `jss_p` | 11 个参数 |
| `zhs_p` | 19 个参数 |

当前实际布局示例：

```text
kte_p, kte_p[0], kte_p[1], ..., kte_p[8]
jss_p, jss_p[0], jss_p[1], ..., jss_p[10]
zhs_p, zhs_p[0], zhs_p[1], ..., zhs_p[18]
```

参数数组含义沿用当前 MATLAB/Python 参数映射：

```text
kte_p = [着陆场海拔, 投放高度, 运输机速度, 货舱角度,
         装载位置x, 装载位置y, x方向风速, y方向风速, 地面温度]

jss_p = 减速伞质量、阻力参数、面积、伞绳参数和展开参数

zhs_p = 主伞面积、数量、质量、阻力参数、伞绳参数和充气参数
```

### 4.2 第二阶段如何获得完整参数

第二阶段并不是只使用 `airdrop_input_second.xlsx` 的内容。

Python 处理第二阶段时，应组合两个输入文件：

```text
第一输入文件：
    vel_p、qys_p、sim_p

第二输入文件：
    kte_p、jss_p、zhs_p
```

组合后形成完整仿真调用：

```text
run_full_simulation(kte_p, vel_p, qys_p, jss_p, zhs_p, sim_p)
```

第二阶段输出“整个流程”的数据，而不是只输出减速伞和主伞阶段。因此完整输出中应包含：

1. 出舱阶段；
2. 减速伞整理和充气阶段；
3. 主伞整理和充气阶段；
4. 如启用稳降计算，还包括稳降阶段。

为了保证第一阶段输出和第二阶段完整输出的物理状态连续，第二阶段应根据组合后的全部参数重新计算整个流程，而不是把第一阶段 Excel 中的最后一行作为新的初始条件直接拼接。

## 5. 第二阶段输出格式

文件：

```text
C:\Users\Tree\AppData\Roaming\KTData\airdrop_output_second.xlsx
```

第二阶段输出九列，按用户描述建议固定为：

| 列号 | 列名 | 物理意义 | 单位 |
|---:|---|---|---|
| 1 | `x_position` | x 方向位移 | m |
| 2 | `y_position` | y 方向位移 | m |
| 3 | `z_position` | z 方向位移 | m |
| 4 | `x_velocity` | x 方向速度 | m/s |
| 5 | `y_velocity` | y 方向速度 | m/s |
| 6 | `z_velocity` | z 方向速度 | m/s |
| 7 | `pitch_angle` | 俯仰角 | deg |
| 8 | `roll_angle` | 侧倾角 | deg |
| 9 | `yaw_angle` | 偏航角 | deg |

建议第一行写列名：

```text
x_position,y_position,z_position,x_velocity,y_velocity,z_velocity,pitch_angle,roll_angle,yaw_angle
...
```

## 6. 与现有 `data_all.xlsx` 的差异

当前参考文件：

```text
C:\Users\Tree\Desktop\2\2\伞降输出\data_all.xlsx
```

实际是九列、无表头。按照当前旧版 MATLAB/Python 代码，其列顺序更接近：

```text
x_position,
height,
lateral_displacement,
x_speed,
vertical_speed,
lateral_speed,
pitch_angle,
roll_angle,
overload
```

这与当前新接口要求不同：

```text
x_position,
y_position,
z_position,
x_velocity,
y_velocity,
z_velocity,
pitch_angle,
roll_angle,
yaw_angle
```

因此第二阶段输出不能简单地把旧 `data_all` 原样改名。特别是：

1. `height` 是否改为 `z_position`，必须统一坐标正方向；
2. `overload` 不能直接作为 `yaw_angle`；
3. 当前模型是否真实计算偏航角，需要单独确认；
4. 如果当前模型只有二维运动，则缺少真实的第三个姿态角。

## 7. 当前模型与目标接口之间的必要约定

### 7.1 第一阶段只输入三组参数的问题

现有出舱动力学仍然同时使用 `kte_p` 和 `vel_p`。

因此“第一阶段输入文件只包含 `kte_p`、`qys_p`、`sim_p`”只有在以下条件成立时才能计算：

```text
第一阶段使用后端固定的 kte_p 默认参数或固定环境参数配置。
```

否则，第一阶段仍然必须接收 `kte_p`。

本设计按当前 Excel 布局采用拆分输入：

```text
第一阶段：前端提交 vel_p、qys_p、sim_p。
第二阶段：前端提交 kte_p、jss_p、zhs_p。
后端将两份文件合并后重新计算完整流程。
```

第一阶段输出因此是“基于第一阶段文件中的车辆参数和第二阶段文件中的环境参数配置的出舱结果”。

### 7.2 第一阶段侧倾角

现有出舱阶段的动力学状态主要是二维平动和绕单一轴的转动，没有独立的侧倾角状态。

因此第一阶段的 `roll_angle` 目前只能有以下两种处理方式：

1. 暂时输出 `0`，表示当前模型未计算侧倾；
2. 扩展出舱动力学，增加侧倾角和侧倾角速度状态。

在没有完成三维姿态动力学之前，不能把某个二维角度复制成侧倾角。

### 7.3 第二阶段三个姿态角

如果第二阶段必须输出真实的：

```text
pitch_angle、roll_angle、yaw_angle
```

则仿真模型必须明确计算三个独立姿态自由度及其角速度。

如果当前模型仍是二维或仅包含俯仰/侧倾两个角度，则第三个角度应：

1. 明确标记为未计算；
2. 或暂时固定为 `0`；
3. 不能把旧数据中的 `overload` 当作第三个角度。

## 8. 文件处理流程

### 8.1 第一阶段

```text
1. 前端写入 airdrop_input_first.xlsx。
2. Python 读取 Sheet1 中的 vel_p、qys_p、sim_p。
3. Python 等待或读取第二阶段文件中的 kte_p。
4. Python 运行出舱阶段。
5. Python 将六列结果写入 airdrop_output_first.xlsx。
```

### 8.2 第二阶段

```text
1. 前端写入或更新 airdrop_input_second.xlsx。
2. Python 同时读取 input_first.xlsx 和 input_second.xlsx。
3. Python 组合 kte_p、vel_p、qys_p、sim_p、jss_p、zhs_p。
4. Python 重新计算完整空投流程。
5. Python 将九列完整结果写入 airdrop_output_second.xlsx。
```

### 8.3 Excel 写入安全要求

为避免前端读取到写入一半的文件，Python 写输出时建议采用临时文件替换：

```text
airdrop_output_first.tmp.xlsx
    -> 写入完成
    -> 替换为 airdrop_output_first.xlsx
```

第二阶段同理。

前端写输入文件时也应先写临时文件，写完后再替换正式输入文件。这样 Python 不会在 Excel 尚未写完时读取到半个文件。

## 9. 建议的状态判定

当前只使用固定文件名时，Python 需要避免重复处理同一个输入文件。建议至少记录：

```text
输入文件最后修改时间
输入文件大小
输入文件内容哈希
```

只有输入文件发生变化并且能够正常打开时，才启动对应阶段。

如果后续需要更稳定的前后端协同，可以增加状态文件：

```text
airdrop_status.json
```

状态至少包含：

```text
stage: first / second
status: idle / running / success / error
message: 错误或完成信息
updated_at: 更新时间
```

## 10. 当前理解的最终协议

```text
第一阶段输入：
    airdrop_input_first.xlsx
    vel_p、qys_p、sim_p

第一阶段输出：
    airdrop_output_first.xlsx
    x位移、y位移、x速度、y速度、俯仰角、侧倾角

第二阶段输入：
    airdrop_input_first.xlsx
    airdrop_input_second.xlsx
    kte_p、vel_p、qys_p、sim_p、jss_p、zhs_p

第二阶段输出：
    airdrop_output_second.xlsx
    x/y/z位移、x/y/z速度、俯仰角/侧倾角/偏航角
```


