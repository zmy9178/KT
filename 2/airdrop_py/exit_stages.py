from __future__ import annotations

from math import atan, cos, pi, sin, sqrt, tan

import numpy as np

from .models import AirdropContext
from .numerics import matlab_time_grid, solve_on_matlab_grid


# 前两个出舱阶段的阻尼系数，单位可理解为 1/s。
# 目前不考虑出仓阻尼系数
STAGE1_TRANSLATION_DAMPING_RATIO = 0.00
STAGE2_TRANSLATION_DAMPING_RATIO = 0.00
STAGE2_ANGULAR_DAMPING_RATIO = 0.00


def ode45_stage1(_t: float, y: np.ndarray, ctx: AirdropContext) -> np.ndarray:
    """MATLAB ``ode45_stage1``: vehicle sliding inside the cargo bay."""

    drop_envi = ctx.drop_envi
    vehicle = ctx.vehicle
    pullchute = ctx.pullchute
    
    chute_k = pullchute.resist * pullchute.area     #降落伞阻力系数
    velocity = drop_envi.airplane_velo / 3.6        #空投运输机速度转化km/h
    alfa0 = drop_envi.cargo_angle * pi / 180.0      #运输机货舱与水平面夹角转换：角度->弧度
    mass = vehicle.dropmass * 1000.0                #空投车质量转换：kg->g
    grav1 = drop_envi.grav * (1.0 - drop_envi.drophight / drop_envi.earth_r)  
    gravity_force = vehicle.dropmass * grav1         
    d_temp = 6.5e-3                                  #大气温度湿度系数
    t0 = 273.0
    h = drop_envi.drophight
    rho = drop_envi.air_density * (1.0 - d_temp * h / t0) ** 4.26

    dy = np.zeros(4, dtype=float)
    dy[0] = y[1]
    dy[2] = y[3]
    q = 0.5 * chute_k * rho * (velocity - y[1]) ** 2
    n_force = gravity_force * cos(alfa0) - q * sin(alfa0)
    translation_damping = STAGE1_TRANSLATION_DAMPING_RATIO * mass
    damping_x = translation_damping * y[1]
    damping_y = translation_damping * y[3]
    dy[1] = (-q + vehicle.mu * n_force * cos(alfa0) - n_force * sin(alfa0) - damping_x) / mass
    dy[3] = (gravity_force - n_force * cos(alfa0) - vehicle.mu * n_force * sin(alfa0) - damping_y) / mass
    return dy


def stage_moving(ctx: AirdropContext) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    """Run the exit sliding stage and return its MATLAB-compatible outputs."""

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    sim = ctx.sim

    hm = vehicle.hull_cmy - vehicle.track_fdy
    alfa = drop_envi.cargo_angle * pi / 180.0
    t_grid = matlab_time_grid(0.0, sim.time / 4.0, sim.step)
    initial = np.zeros(4, dtype=float)
    initial[0] = 0.0
    initial[1] = 0.0
    initial[2] = -1.0 * (vehicle.track_fdx + vehicle.hull_cmx) * sin(alfa) - hm * cos(alfa)
    initial[3] = 0.0

    time_1, data_1 = solve_on_matlab_grid(lambda t, y: ode45_stage1(t, y, ctx), t_grid, initial)
    hit = np.nonzero(data_1[:, 0] + drop_envi.load_posi_x <= 0.0)[0]
    if hit.size == 0:
        raise RuntimeError("stage_moving did not reach the MATLAB stop condition")
    idx = int(hit[0])
    return float(time_1[idx]), data_1[idx, :4].copy(), time_1, data_1


def ode45_stage2(_t: float, y: np.ndarray, ctx: AirdropContext) -> np.ndarray:
    """MATLAB ``ode45_stage2``: vehicle rotation while exiting the bay."""

    vehicle = ctx.vehicle
    pullchute = ctx.pullchute
    drop_envi = ctx.drop_envi

    drop_mass = vehicle.dropmass
    grav1 = drop_envi.grav * (1.0 - drop_envi.drophight / drop_envi.earth_r)
    hm = vehicle.hull_cmy - vehicle.track_fdy
    hp = vehicle.pull_y - vehicle.track_fdy
    lp = -vehicle.pull_x + vehicle.hull_cmx
    chute_k = pullchute.resist * pullchute.area
    mass = drop_mass * 1000.0
    l0 = sqrt((hm - hp) ** 2 + lp**2)
    gama = atan((hm - hp) / lp)
    gravity_force = grav1 * mass
    velos0 = drop_envi.airplane_velo / 3.6
    d_temp = 6.5e-3
    t0 = 273.0
    rho = drop_envi.air_density * (1.0 - d_temp * drop_envi.drophight / t0) ** 4.26

    dy = np.zeros(6, dtype=float)
    dy[0] = y[1]
    dy[2] = y[3]
    dy[4] = y[5]
    q = 0.5 * chute_k * rho * (velos0 - y[1]) ** 2
    n_force = gravity_force * cos(y[4]) - q * sin(y[4])
    lop = (abs(y[0]) - hm * tan(y[4])) / cos(y[4])
    translation_damping = STAGE2_TRANSLATION_DAMPING_RATIO * mass
    angular_damping = STAGE2_ANGULAR_DAMPING_RATIO * vehicle.izz
    damping_x = translation_damping * y[1]
    damping_y = translation_damping * y[3]
    damping_pitch = angular_damping * y[5]
    dy[1] = (-q - n_force * sin(y[4]) + vehicle.mu * n_force * cos(y[4]) - damping_x) / mass
    dy[3] = (gravity_force - n_force * cos(y[4]) - vehicle.mu * n_force * sin(y[4]) - damping_y) / mass
    dy[5] = (
        vehicle.mu * n_force * hm
        + n_force * lop
        - q * l0 * sin(y[4] + gama)
        - damping_pitch
    ) / vehicle.izz
    return dy


def stage_revote(
    data_stage_1: np.ndarray,
    ctx: AirdropContext,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    sim = ctx.sim

    hm = vehicle.hull_cmy - vehicle.track_fdy
    lt = -vehicle.track_fdx + vehicle.track_bdx
    lp = -vehicle.track_fdx + vehicle.hull_cmx
    alfa = drop_envi.cargo_angle * pi / 180.0

    initial = np.zeros(6, dtype=float)
    initial[0] = 0.0
    initial[1] = data_stage_1[1] #继承阶段一x速度
    initial[2] = data_stage_1[2] #继承阶段一y位置
    initial[3] = data_stage_1[3] #继承阶段一y速度
    initial[4] = alfa
    initial[5] = 0.0

    t_grid = matlab_time_grid(0.0, sim.time / 5.0, sim.step)

    gama = atan(hm / (lt - lp))
    po = sqrt(hm**2 + (lt - lp) ** 2)

    def rear_contact_event(_t: float, y: np.ndarray) -> float:
        return float(y[0] + po * cos(y[4] - gama))

    rear_contact_event.terminal = True
    rear_contact_event.direction = -1

    result = solve_on_matlab_grid(
        lambda t, y: ode45_stage2(t, y, ctx),
        t_grid,
        initial,
        events=rear_contact_event,
        return_result=True,
    )

    if len(result.t_events)== 0 or len(result.t_events[0])==0:
        raise RuntimeError("stage_revote did not reach the MATLAB stop condition")

    time_fz = float(result.t_events[0][0])
    events_state = result.y_events[0][0].copy()

    time_2 = result.t.copy()
    data_2 = result.y.T.copy()

    # t_eval 只返回固定网格点，事件时刻通常不在固定网格上。
    if time_2.size == 0 or not np.isclose(time_2[-1], time_fz):
        time_2 = np.append(time_2, time_fz)
        data_2 = np.vstack((data_2, events_state))

    data_fz = events_state.copy()

    #阶段二的 X 位移从 0 重新计数，因此转换为累计相对位移。

    data_fz[0] += data_stage_1[0]

    # data_fz[1] 不再加 data_stage_1[1]：
    # 阶段二 initial[1] 已经继承了阶段一 X 速度。
    return time_fz, data_fz, time_2, data_2