# Python 离线迁移指南

本迁移面向没有网络、但已安装 Python 及依赖库的电脑。当前代码只使用本地库：

- `numpy`
- `scipy`
- `openpyxl`，仅旧 Excel 测试需要
- Python 标准库 `dataclasses`、`unittest` 等

## 当前包结构

```text
airdrop_py/
├─ __init__.py              # 对外导出入口
├─ defaults.py              # MATLAB 注释中的默认参数
├─ models.py                # dataclass 参数结构和 build_context
├─ numerics.py              # MATLAB 风格时间网格和 solve_ivp 包装
├─ exit_stages.py           # stage_moving / stage_revote / ode45_stage1/2
├─ decechute_stages.py      # jiansu_straight / jiansu_chongqi / ode45_stage3/4
└─ simulation.py            # 分阶段运行入口
```

旧文件 `airdrop_simulation_partial.py` 保留为第一批迁移记录，新迁移建议从 `airdrop_py/` 开始。

## 运行测试

在 `C:\Users\Tree\Desktop\2\2` 目录执行：

```powershell
py -3.13 -m unittest discover -s . -p "test_*.py"
```

如果本机没有 `伞降输出/time_all.xlsx` 和 `伞降输出/data_all.xlsx`，旧 Excel 参考输出测试会自动跳过；不影响 Python 阶段函数的基础测试。

## 当前已迁移阶段

### 第一批：出舱阶段

对应 MATLAB：

- `stage_moving.m`
- `stage_revote.m`
- `ode45_stage1.m`
- `ode45_stage2.m`

Python 文件：

- `airdrop_py/exit_stages.py`

### 第二批：减速伞阶段

对应 MATLAB：

- `jiansu_straight.m`
- `jiansu_chongqi.m`
- `ode45_stage3.m`
- `ode45_stage4.m`

Python 文件：

- `airdrop_py/decechute_stages.py`

## 核心迁移方式

MATLAB 使用 `global`：

```matlab
global Vehicle drop_envi mainchute decechute
```

Python 使用显式上下文：

```python
ctx.vehicle
ctx.drop_envi
ctx.mainchute
ctx.decechute
```

固定参数放在 `AirdropContext` 中。阶段中间结果通过函数参数传递，例如：

```python
time_jslz, data_jslz, time_3, data_3, data_33 = jiansu_straight(data_fz, ctx)
time_jscq, data_jscq, time_4, data_4, data_44 = jiansu_chongqi(data_jslz, ctx)
```

这种结构比全局变量更适合离线逐步测试和排错。

## 当前可用入口

只运行出舱阶段：

```python
from airdrop_py import (
    DEFAULT_KTE_P, DEFAULT_VEL_P, DEFAULT_QYS_P,
    DEFAULT_JSS_P, DEFAULT_ZHS_P, DEFAULT_SIM_P,
    run_exit_stages,
)

result = run_exit_stages(
    DEFAULT_KTE_P,
    DEFAULT_VEL_P,
    DEFAULT_QYS_P,
    DEFAULT_JSS_P,
    DEFAULT_ZHS_P,
    DEFAULT_SIM_P,
)
```

运行到减速伞充气结束：

```python
from airdrop_py import (
    DEFAULT_KTE_P, DEFAULT_VEL_P, DEFAULT_QYS_P,
    DEFAULT_JSS_P, DEFAULT_ZHS_P, DEFAULT_SIM_P,
    run_decechute_stages,
)

result = run_decechute_stages(
    DEFAULT_KTE_P,
    DEFAULT_VEL_P,
    DEFAULT_QYS_P,
    DEFAULT_JSS_P,
    DEFAULT_ZHS_P,
    DEFAULT_SIM_P,
)

print(result["time_jslz"], result["data_jslz"])
print(result["time_jscq"], result["data_jscq"])
```

## 下一批建议

下一阶段迁移主伞拉直和主伞充气：

- MATLAB `zhusan_straight.m` -> Python `mainchute_stages.py`
- MATLAB `ode45_stage5.m` -> Python `ode45_stage5`
- MATLAB `zhusan_chongqi.m` -> Python `zhusan_chongqi`
- MATLAB `ode45_stage6.m` -> Python `ode45_stage6`

建议先保持 MATLAB 原行为，包括已知疑点，再用测试记录这些行为。等完整链路打通后，再考虑修正物理模型或变量命名。
