# Python Migration Progress

## Batch 1: Exit Stages

Implemented in `airdrop_simulation_partial.py`:

- MATLAB input-vector to Python dataclass mapping.
- Atmospheric pressure, density, and gravity initialization.
- `ode45_stage1` and `stage_moving`.
- `ode45_stage2` and `stage_revote`.
- MATLAB-style fixed output time grids with SciPy `solve_ivp(method="RK45")`.
- Explicit errors when a MATLAB post-processing stop condition is not reached.
- Input length and finite-value validation.
- Regression tests that read `伞降输入/伞降计算输入.xlsx` and compare the
  supplied output schema and initial state against
  `伞降输出/time_all.xlsx` and `伞降输出/data_all.xlsx`.

## Reference Data Status

The input workbook is used by the automated tests. The output workbook has the
expected 9-column schema and matches the input-derived initial state. However,
its early dynamic values do not match the current MATLAB source:

- At `t=1.861 s`, the supplied output horizontal speed is about `75.80 m/s`.
- The current MATLAB equations, as implemented in the first Python batch,
  produce about `82.56 m/s`.

The difference starts immediately after the initial state, so it is not an ODE
tolerance issue. The full numerical regression test is intentionally skipped
until the workbook's MATLAB source revision or generation settings are
confirmed.

Run the first batch:

```powershell
py -3.13 .\2\airdrop_simulation_partial.py
py -3.13 -m unittest discover -s .\2 -p "test_*.py"
```

## Planned Batches

1. Main-chute straightening and inflation: `ode45_stage5`, `zhusan_straight`,
   `ode45_stage6`, and `zhusan_chongqi`.
2. Steady descent, output assembly, and comparison against the provided Excel
   result files.

The MATLAB source remains unchanged.

## Batch 2: Decechute Stages

Implemented in the new package directory `airdrop_py/`:

- `models.py`: explicit dataclass context replacing MATLAB `global` structs.
- `numerics.py`: MATLAB-style fixed time grid and `solve_ivp` wrapper.
- `exit_stages.py`: package version of the already migrated exit stages.
- `decechute_stages.py`: `ode45_stage3`, `jiansu_straight`, `ode45_stage4`,
  and `jiansu_chongqi`.
- `simulation.py`: `run_exit_stages` and `run_decechute_stages`.
- `test_airdrop_py_decechute.py`: offline tests for context mapping, exit stage
  API, decechute stage shapes, finite values, and stop conditions.

The old Excel-reference tests now skip output-reference assertions when
`伞降输出/time_all.xlsx` and `伞降输出/data_all.xlsx` are not available.

Run all offline tests:

```powershell
py -3.13 -m unittest discover -s . -p "test_*.py"
```
