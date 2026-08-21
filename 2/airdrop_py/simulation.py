from __future__ import annotations

import numpy as np

from .decechute_stages import jiansu_chongqi, jiansu_straight
from .exit_stages import stage_moving, stage_revote
from .mainchute_stages import zhusan_chongqi, zhusan_straight
from .models import build_context
from .steady_descent import sim_wj


def run_exit_stages(
    kte_p: list[float] | np.ndarray,
    vel_p: list[float] | np.ndarray,
    qys_p: list[float] | np.ndarray,
    jss_p: list[float] | np.ndarray,
    zhs_p: list[float] | np.ndarray,
    sim_p: list[float] | np.ndarray,
) -> dict[str, np.ndarray | float]:
    """Run the migrated exit stages through vehicle rotation."""

    ctx = build_context(kte_p, vel_p, qys_p, jss_p, zhs_p, sim_p)
    time_pd, data_pd, time_1, data_1 = stage_moving(ctx)
    time_fz, data_fz, time_2, data_2 = stage_revote(data_pd, ctx)
    return {
        "ctx": ctx,
        "time_pd": time_pd,
        "data_pd": data_pd,
        "time_1": time_1,
        "data_1": data_1,
        "time_fz": time_fz,
        "data_fz": data_fz,
        "time_2": time_2,
        "data_2": data_2,
    }


def run_decechute_stages(
    kte_p: list[float] | np.ndarray,
    vel_p: list[float] | np.ndarray,
    qys_p: list[float] | np.ndarray,
    jss_p: list[float] | np.ndarray,
    zhs_p: list[float] | np.ndarray,
    sim_p: list[float] | np.ndarray,
) -> dict[str, np.ndarray | float]:
    """Run exit stages plus deceleration parachute straightening and inflation."""

    result = run_exit_stages(kte_p, vel_p, qys_p, jss_p, zhs_p, sim_p)
    ctx = result["ctx"]
    time_jslz, data_jslz, time_3, data_3, data_33 = jiansu_straight(result["data_fz"], ctx)
    time_jscq, data_jscq, time_4, data_4, data_44 = jiansu_chongqi(data_jslz, ctx)
    result.update(
        {
            "time_jslz": time_jslz,
            "data_jslz": data_jslz,
            "time_3": time_3,
            "data_3": data_3,
            "data_33": data_33,
            "time_jscq": time_jscq,
            "data_jscq": data_jscq,
            "time_4": time_4,
            "data_4": data_4,
            "data_44": data_44,
        }
    )
    return result


def run_mainchute_stages(
    kte_p: list[float] | np.ndarray,
    vel_p: list[float] | np.ndarray,
    qys_p: list[float] | np.ndarray,
    jss_p: list[float] | np.ndarray,
    zhs_p: list[float] | np.ndarray,
    sim_p: list[float] | np.ndarray,
) -> dict[str, np.ndarray | float]:
    """Run the exit, deceleration-parachute, and main-parachute stages."""

    result = run_decechute_stages(kte_p, vel_p, qys_p, jss_p, zhs_p, sim_p)
    ctx = result["ctx"]
    time_mslz, data_mslz, time_5, data_5, data_55 = zhusan_straight(
        result["data_jscq"],
        result["data_jslz"],
        ctx,
    )
    time_mscq, data_mscq, time_6, data_6, data_66 = zhusan_chongqi(
        data_mslz,
        ctx,
        result["data_jslz"],
    )
    result.update(
        {
            "time_mslz": time_mslz,
            "data_mslz": data_mslz,
            "time_5": time_5,
            "data_5": data_5,
            "data_55": data_55,
            "time_mscq": time_mscq,
            "data_mscq": data_mscq,
            "time_6": time_6,
            "data_6": data_6,
            "data_66": data_66,
        }
    )
    return result


def _take_stage_prefix(
    time_values: np.ndarray,
    data_values: np.ndarray,
    stop_time: float,
) -> np.ndarray:
    """Return rows through a MATLAB post-processing stop time."""

    stop_index = int(np.searchsorted(time_values, stop_time, side="left"))
    stop_index = min(max(stop_index, 0), len(data_values) - 1)
    return data_values[: stop_index + 1]


def run_full_simulation(
    kte_p: list[float] | np.ndarray,
    vel_p: list[float] | np.ndarray,
    qys_p: list[float] | np.ndarray,
    jss_p: list[float] | np.ndarray,
    zhs_p: list[float] | np.ndarray,
    sim_p: list[float] | np.ndarray,
    *,
    include_steady_descent: bool = True,
    steady_duration: float = 200.0,
) -> dict[str, np.ndarray | float]:
    """Run all seven MATLAB stages and assemble the final nine-column output."""

    result = run_mainchute_stages(
        kte_p,
        vel_p,
        qys_p,
        jss_p,
        zhs_p,
        sim_p,
    )
    ctx = result["ctx"]
    sim = ctx.sim

    exit_stage_1 = _take_stage_prefix(
        result["time_1"],
        result["data_1"],
        result["time_pd"],
    )
    exit_stage_2 = _take_stage_prefix(
        result["time_2"],
        result["data_2"],
        result["time_fz"],
    )
    data_cc_1 = np.zeros((len(exit_stage_1), 6), dtype=float)
    data_cc_1[:, :4] = exit_stage_1
    data_cc = np.vstack((data_cc_1, exit_stage_2))

    data_3 = result["data_3"]
    data_4 = result["data_4"]
    data_5 = result["data_5"]
    data_6 = result["data_6"]
    data_33 = result["data_33"]
    data_44 = result["data_44"]
    data_55 = result["data_55"]
    data_66 = result["data_66"]

    def velocity_components(data: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        angle_rad = data[:, 1]
        #统一变量单位为弧度制，参考2/伞降稳降连续过渡改进方案.md改动一个人意见
        return data[:, 0] * np.cos(angle_rad), data[:, 0] * np.sin(angle_rad)

    v3_x, v3_y = velocity_components(data_33)
    v4_x, v4_y = velocity_components(data_44)
    v5_x, v5_y = velocity_components(data_55[:, :2])
    v6_x, v6_y = velocity_components(data_66)

    vehicle_x = np.concatenate(
        (
            -data_cc[:, 0],
            data_3[:, 0],
            data_4[:, 0],
            data_5[:, 0],
            data_6[:, 0],
        )
    )
    vehicle_y = np.concatenate(
        (
            data_cc[:, 2],
            data_3[:, 1],
            data_4[:, 1],
            data_5[:, 1],
            data_6[:, 1],
        )
    )
    velocity_x = np.concatenate(
        (
            ctx.drop_envi.airplane_velo / 3.6 + data_cc[:, 1],
            v3_x,
            v4_x,
            v5_x,
            v6_x,
        )
    )
    velocity_y = np.concatenate(
        (
            data_cc[:, 3],
            v3_y,
            v4_y,
            v5_y,
            v6_y,
        )
    )
    chute_angle = np.concatenate(
        (
            np.zeros(len(data_cc), dtype=float),
            data_3[:, 2],
            data_4[:, 2],
            data_5[:, 2],
            data_6[:, 2],
        )
    )
    pitch_angle = np.concatenate(
        (
            data_cc[:, 4],
            data_3[:, 6],
            data_4[:, 4],
            data_5[:, 6],
            data_6[:, 5],
        )
    )
    pitch_rate = np.concatenate(
        (
            data_cc[:, 5],
            data_3[:, 7],
            data_4[:, 5],
            data_5[:, 7],
            data_6[:, 6],
        )
    )
    rope_force = np.concatenate(
        (
            np.zeros(len(data_cc), dtype=float),
            data_3[:, 8],
            data_4[:, 6],
            data_5[:, 8],
            data_6[:, 4],
        )
    )
    vehicle_speed = np.concatenate(
        (
            ctx.drop_envi.airplane_velo / 3.6 + data_cc[:, 1],
            data_33[:, 0],
            data_44[:, 0],
            data_55[:, 0],
            data_66[:, 0],
        )
    )

    time_pre_steady = np.arange(len(vehicle_x), dtype=float) * sim.step
    # Match MATLAB's final data_all order:
    # X, height, lateral displacement, X speed, vertical speed,
    # lateral speed, pitch angle, roll angle, overload.
    data_pre_steady = np.column_stack(
        (
            vehicle_x,
            ctx.drop_envi.drophight - vehicle_y,
            np.zeros(len(vehicle_x), dtype=float),
            velocity_x,
            velocity_y,
            np.zeros(len(vehicle_x), dtype=float),
            np.rad2deg(pitch_angle),
            np.zeros(len(vehicle_x), dtype=float),
            rope_force / (9.8 * ctx.vehicle.dropmass * 1000.0),
        )
    )

    if include_steady_descent:
        wind = np.array(
            [ctx.drop_envi.wind_x, ctx.drop_envi.wind_y, 0.0],
            dtype=float,
        )
        (
            time_wj,
            data_wj,
            x_wj,
            y_wj,
            z_wj,
            vx_wj,
            vy_wj,
            vz_wj,
        ) = sim_wj(
            result["data_mscq"],
            wind,
            data_66,
            ctx,
            duration=steady_duration,
        )




        #稳降阶段的列顺序与 data_pre_steady 保持一致
        #避免伞降阶段和稳降阶段在交接点重复一行

        force_wj = (
            np.abs(data_wj[:, 6] / ctx.vehicle.dropmass / 1000.0)
            if len(data_wj)
            else np.empty(0, dtype=float)
        )
        #计算稳降阶段载荷

        #稳降函数内部坐标名和最终 CSV 坐标名不是完全一致，需要在这里重新映射
        steady_data = np.column_stack(
            (     
                x_wj,
                ctx.drop_envi.drophight - z_wj, #Y位置离地高度
                y_wj,                           #Z位置侧向位移
                vx_wj,
                vz_wj,                          #Y速度垂直速度
                vy_wj,                          #Z速度侧向速度
                90.0 - data_wj[:, 1],
                data_wj[:, 0],
                force_wj,
            )
        )


        #替换原始稳降阶段数据，避免重复一行，
        if len(time_wj) and np.isclose(time_wj[0], 0.0):
            time_wj_output = time_wj[1:]
            steady_data_output = steady_data[1:]
        else:
            time_wj_output = time_wj
            steady_data_output = steady_data

        #拼接时间和数据，形成完整的时间序列和数据序列
        time_all = np.concatenate(
            (time_pre_steady, time_wj_output + time_pre_steady[-1])
        )
        data_all = np.vstack(
            (
                data_pre_steady,
                steady_data_output,
            )
        )





        result.update(
            {
                "time_wj": time_wj,
                "data_wj": data_wj,
                "x_wj": x_wj,
                "y_wj": y_wj,
                "z_wj": z_wj,
                "vx_wj": vx_wj,
                "vy_wj": vy_wj,
                "vz_wj": vz_wj,
            }
        )
    else:
        time_all = time_pre_steady
        data_all = data_pre_steady

    result["time_pre_steady"] = time_pre_steady
    result["data_pre_steady"] = data_pre_steady
    result["time_all"] = time_all
    result["data_all"] = data_all
    return result
