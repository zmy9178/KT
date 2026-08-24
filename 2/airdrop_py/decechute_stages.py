from __future__ import annotations

from math import atan, cos, pi, sin, sqrt

import numpy as np

from .models import AirdropContext
from .numerics import matlab_time_grid, solve_on_matlab_grid

def _decechute_released_mass(
    deployed_length : float,
    ctx : AirdropContext,
) -> tuple[float,float]:
    #返回已拉出质量和单位展开长度对应的质量

    decechute = ctx.decechute

    rope_length = decechute.rope_length
    band_length = decechute.chute_width
    canopy_diameter = 2.0 * sqrt(decechute.aera / pi)
    canopy_length = canopy_diameter /2.0
    total_length = rope_length + band_length + canopy_length

    length = float(np.clip(deployed_length, 0.0, total_length))

    if length <= rope_length:
        mass_per_length = decechute.rope_mass / rope_length
        released_mass = mass_per_length * length
        return released_mass, mass_per_length

    if length <= rope_length + band_length:
        band_deployed = length - rope_length
        mass_per_length = decechute.chute_side_mass / band_length
        released_mass = (
            decechute.rope_mass
            + mass_per_length * band_deployed
        )
        return released_mass, mass_per_length

    canopy_deployed = length - rope_length - band_length
    progress = np.clip(canopy_deployed / canopy_length, 0.0, 1.0)

    remaining_canopy_mass = (
        decechute.mass
        - decechute.rope_mass
        - decechute.chute_side_mass
    )

    # 2p - p^2：起点和终点质量连续，终点导数自然降为零。
    released_canopy_mass = remaining_canopy_mass * (
        2.0 * progress - progress**2
    )

    released_mass = (
        decechute.rope_mass
        + decechute.chute_side_mass
        + released_canopy_mass
    )

    mass_per_length = (
        remaining_canopy_mass
        * 2.0
        * (1.0 - progress)
        / canopy_length
    )

    return released_mass, mass_per_length



def ode45_stage3(_t: float, y: np.ndarray, ctx: AirdropContext) -> np.ndarray:
    """MATLAB ``ode45_stage3``: deceleration parachute straightening."""

    vehicle = ctx.vehicle
    pullchute = ctx.pullchute
    drop_envi = ctx.drop_envi
    mainchute = ctx.mainchute
    decechute = ctx.decechute

    rho = drop_envi.air_density
    g = drop_envi.grav
    ca_ys = pullchute.resist * pullchute.area
    ca_d = mainchute.chute_cover_resist
    ca_w = vehicle.resist_hull
    ca_e = mainchute.pullout_resist
    mw = vehicle.dropmass * 1000.0
    mys = pullchute.mass
    msh1 = decechute.rope_mass / decechute.rope_length
    msy1 = decechute.chute_side_mass / decechute.chute_width
    b = decechute.chute_width
    msh = decechute.rope_mass
    msy = decechute.mass - decechute.rope_mass
    lsh = decechute.rope_length
    do = sqrt(decechute.aera / pi) * 2.0
    lxt = decechute.rope_length + decechute.chute_width + do / 2.0
    fsh = mainchute.pullout_force

    qys = 0.5 * rho * ca_ys * y[3] ** 2
    qd = 0.5 * rho * ca_d * y[3] ** 2
    qw = 0.5 * rho * ca_w * vehicle.hull_l * vehicle.hull_w * y[4] ** 2
    qe = 0.5 * rho * ca_e * y[3] ** 2

    released_mass, mass_per_length = _decechute_released_mass(
        y[5],
        ctx,
    )

    remaining_mass = decechute.mass - released_mass
    relative_velocity = max(y[4]-y[3], 0.0)
    f1 = fsh + mass_per_length * relative_velocity**2

    dy = np.zeros(6, dtype=float)
    dy[0] = y[4] * cos(y[2])
    dy[1] = y[4] * sin(y[2])
    dy[2] = g * cos(y[2]) / y[4]
    dy[3] = g * sin(y[2]) - (qys + qd - f1) / (mys + released_mass)
    dy[4] = g * sin(y[2]) - (qw + qe + f1) / (mw + remaining_mass)
    dy[5] = relative_velocity      #车辆速度-伞包速度=拉出长度增长率
    return dy


def jiansu_straight(
    data_stage_2: np.ndarray,
    ctx: AirdropContext,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Run deceleration parachute straightening and post-process MATLAB outputs."""

    vehicle = ctx.vehicle
    pullchute = ctx.pullchute
    drop_envi = ctx.drop_envi
    mainchute = ctx.mainchute
    decechute = ctx.decechute
    sim = ctx.sim

    cmy = vehicle.hull_cmx
    jiansusan_diam = sqrt(decechute.aera / pi) * 2.0



     #修复减速伞整理阶段的同类跳变
    initial = np.zeros(6, dtype=float)
    initial[0] = -1.0 * data_stage_2[0]
    initial[1] = -1.0 * data_stage_2[2]
    #继承进入本阶段前的实际速度向量
    incoming_vx = drop_envi.airplane_velo / 3.6 + data_stage_2[1]
    incoming_vy = data_stage_2[3]


    initial[2] = np.arctan2(incoming_vy, incoming_vx)
    initial[3] = np.hypot(incoming_vx, incoming_vy)
    initial[4] = initial[3]
    initial[5] = 0.0







    lsh = decechute.rope_length
    lxt = decechute.rope_length + jiansusan_diam / 2.0 + decechute.chute_width
    fsh = mainchute.pullout_force
    b = decechute.chute_width
    b1 = vehicle.hull_cmy - vehicle.track_fdy
    a1 = vehicle.hull_cmx
    msh1 = decechute.rope_mass / decechute.rope_length
    msy1 = decechute.chute_side_mass / decechute.chute_width
    ca_w = vehicle.resist_hull

    t_grid = matlab_time_grid(0.0, sim.time, sim.step)

    def straightening_complete_event(
        _t: float,
        y: np.ndarray,
    ) -> float:
        # y[5] 是当前已拉出长度；达到 lxt 时整理阶段结束。
        return float(y[5] - lxt)

    # 找到事件后立即停止积分。
    straightening_complete_event.terminal = True

    # 只检测 y[5] 从小于 lxt 增长到大于 lxt 的穿越。
    straightening_complete_event.direction = 1

    solution = solve_on_matlab_grid(
        lambda t, y: ode45_stage3(t, y, ctx),
        t_grid,
        initial,
        events=straightening_complete_event,
        return_result=True,
    )

    if (
        len(solution.t_events) == 0
        or solution.t_events[0].size == 0
    ):
        raise RuntimeError(
            "jiansu_straight did not reach the straightening-complete event"
        )

    # 求解器计算出的准确事件时刻和状态。
    time_jslz = float(solution.t_events[0][0])
    event_state = solution.y_events[0][0].copy()

    # t_eval 返回事件发生前的固定网格点。
    time_3 = solution.t.copy()
    data_3 = solution.y.T.copy()

    # 事件一般发生在两个固定输出时刻之间，因此手动补入准确事件点。
    if (
        time_3.size == 0
        or not np.isclose(
            time_3[-1],
            time_jslz,
            rtol=0.0,
            atol=1.0e-12,
        )
    ):
        time_3 = np.append(time_3, time_jslz)
        data_3 = np.vstack((data_3, event_state))

    # 事件点现在是该阶段最后一个数据点。
    nn = len(time_3) - 1

    vr = data_3[:, 4] - data_3[:, 3]
    m = len(time_3)
    ang_vehicle = np.zeros(nn + 1, dtype=float)
    angw_vehicle = np.zeros(nn + 1, dtype=float)
    ang_vehicle[0] = data_stage_2[4]
    angw_vehicle[0] = data_stage_2[5]
    gama = atan(vehicle.cmpull_y / vehicle.cmpull_l)
    lat = vehicle.cmpull_h / sin(gama)
    vmx = data_3[:, 4] * np.cos(data_3[:, 2])
    dmi = np.zeros(m, dtype=float)
    fl = np.zeros(m, dtype=float)
    msy = decechute.mass - decechute.rope_mass
    do = sqrt(decechute.aera / pi) * 2.0

    for i in range(m):
        if data_3[i, 5] <= lsh:
            dmi[i] = msh1
        elif data_3[i, 5] <= lsh + b:
            dmi[i] = msy1
        elif data_3[i, 5] <= lxt:
            dmi[i] = 8.0 * (msy - msy1 * b) * (lxt - data_3[i, 5]) / do**2
        fl[i] = dmi[i] * vr[i] ** 2 + fsh

    for i in range(1, nn + 1):
        dt = time_3[i] - time_3[i - 1]
        sx = (vehicle.hull_h * cos(ang_vehicle[i - 1]) + vehicle.hull_l * sin(ang_vehicle[i - 1])) * vehicle.hull_w
        shb = (b1 * cos(ang_vehicle[i - 1]) + a1 * sin(ang_vehicle[i - 1])) * vehicle.hull_w
        sht = sx - shb
        qwb = 0.5 * drop_envi.air_density * ca_w * shb**2 * vmx[i - 1] ** 2 / (4.0 * vehicle.hull_w)
        qwt = 0.5 * drop_envi.air_density * ca_w * sht**2 * vmx[i - 1] ** 2 / (4.0 * vehicle.hull_w)
        qfl = fl[i - 1] * lat * sin(ang_vehicle[i - 1] + data_3[i - 1, 2] - pi / 2.0)
        if ang_vehicle[i - 1] + data_3[i - 1, 2] == pi / 2.0:
            tw = qwt - qwb
        else:
            tw = qwt - qwb - qfl
        acc_angle = tw / vehicle.izz
        angw_vehicle[i] = angw_vehicle[i - 1] + acc_angle * dt
        ang_vehicle[i] = ang_vehicle[i - 1] + angw_vehicle[i - 1] * dt

    data_jslz = np.zeros(9, dtype=float)
    data_jslz[:6] = data_3[nn, :6]
    data_jslz[6] = ang_vehicle[nn]
    data_jslz[7] = angw_vehicle[nn]
    data_jslz[8] = fl[nn]

    data_out = np.zeros((nn + 1, 9), dtype=float)
    data_out[:, :6] = data_3[: nn + 1, :6]
    data_out[:, 6] = ang_vehicle
    data_out[:, 7] = angw_vehicle
    data_out[:, 8] = fl[: nn + 1]

    data_out_2 = np.zeros((nn + 1, 2), dtype=float)
    data_out_2[:, 0] = data_3[: nn + 1, 4]
    data_out_2[:, 1] = data_3[: nn + 1, 2]
    return time_jslz, data_jslz, time_3, data_out, data_out_2


def ode45_stage4(_t: float, y: np.ndarray, ctx: AirdropContext, data_stage_3: np.ndarray) -> np.ndarray:
    """MATLAB ``ode45_stage4``: deceleration parachute inflation."""

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    mainchute = ctx.mainchute
    decechute = ctx.decechute

    ca_jst = decechute.resist * decechute.aera
    # Match first.m/ode45_stage4: this ODE uses the main-parachute k_ks.
    # The outer jiansu_chongqi post-processing still uses decechute.ks.
    t1 = mainchute.k_ks * sqrt(ca_jst) / data_stage_3[4]
    d_temp = 6.5e-3
    t0 = 273.0
    h = drop_envi.drophight - y[1]
    rho = drop_envi.air_density * (1.0 - d_temp * h / t0) ** 4.26
    g = drop_envi.grav
    jiansusan_num = mainchute.chute_num
    yita = 1.05 - 0.05 * jiansusan_num
    ca_w = vehicle.resist_hull
    mw = vehicle.dropmass * 1000.0
    mjs = decechute.mass

    sq = (vehicle.hull_h * abs(-pi / 2.0 + y[2])) + vehicle.hull_l * cos(-pi / 2.0 + y[2]) * vehicle.hull_w
    if _t <= t1:
        sm = decechute.aera * _t / t1
        dsm = decechute.aera / t1
    else:
        sm = decechute.aera
        dsm = 0.0

    ca_js = decechute.resist * yita * sm * jiansusan_num
    qjs = 0.5 * rho * ca_js * y[3] ** 2
    qwq = 0.5 * rho * ca_w * sq * y[3] ** 2
    msf = rho * decechute.kf * ca_js ** 1.5
    dmsf = 1.5 * rho * decechute.kf * ca_js**0.5 * dsm if ca_js > 0.0 else 0.0
    m_act = mw + mjs
    m_all = m_act + msf

    dy = np.zeros(4, dtype=float)
    dy[0] = y[3] * cos(y[2])
    dy[1] = y[3] * sin(y[2])
    dy[2] = g * cos(y[2]) / y[3]
    dy[3] = g * sin(y[2]) * m_act / m_all - (qjs + qwq) / m_all - y[3] * dmsf / m_all
    return dy


def jiansu_chongqi(
    data_stage_3: np.ndarray,
    ctx: AirdropContext,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Run deceleration parachute inflation and post-process MATLAB outputs."""

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    mainchute = ctx.mainchute
    decechute = ctx.decechute
    sim = ctx.sim

    d_temp = 6.5e-3
    h = drop_envi.drophight - data_stage_3[1]
    t0 = 273.0
    rho = drop_envi.air_density * (1.0 - d_temp * h / t0) ** 4.26
    jiansusan_num = mainchute.chute_num
    yita = 1.05 - 0.05 * jiansusan_num
    mjs = decechute.mass
    g = drop_envi.grav
    ca_js = decechute.resist * yita * decechute.aera * jiansusan_num
    msf = rho * decechute.kf * ca_js**1.5
    ms_all = mjs + msf
    vl1 = data_stage_3[4]
    ca_jst = decechute.resist * decechute.aera
    t1 = decechute.ks * sqrt(ca_jst) / vl1

    initial = np.zeros(4, dtype=float)
    initial[0] = data_stage_3[0]
    initial[1] = data_stage_3[1]
    initial[2] = data_stage_3[2]
    initial[3] = data_stage_3[4]

    t_grid = matlab_time_grid(0.0, sim.time, sim.step)
    time_4, data_4 = solve_on_matlab_grid(lambda t, y: ode45_stage4(t, y, ctx, data_stage_3), t_grid, initial)

    hit = np.nonzero(time_4 >= t1)[0]
    if hit.size == 0:
        raise RuntimeError("jiansu_chongqi did not reach the MATLAB stop condition")
    n = int(hit[0])
    time_jscq = float(t1)

    dv = np.zeros(n + 1, dtype=float)
    sm = np.zeros(n + 1, dtype=float)
    ca_ms = np.zeros(n + 1, dtype=float)
    fk = np.zeros(n + 1, dtype=float)
    fk[0] = data_stage_3[8]
    dsm = decechute.aera / t1
    for i in range(1, n + 1):
        sm[i] = decechute.aera * time_4[i] / t1
        ca_ms[i] = decechute.resist * yita * sm[i] * jiansusan_num
        dv[i] = (data_4[i - 1, 3] - data_4[i, 3]) / (time_4[i] - time_4[i - 1])
        fk[i] = (
            ms_all * dv[i]
            + 0.5 * rho * data_4[i, 3] ** 2 * ca_ms[i]
            + data_4[i, 3] * dsm
            + mjs * g * sin(data_4[i, 2])
        )

    ang_vehicle = np.zeros(n + 1, dtype=float)
    angw_vehicle = np.zeros(n + 1, dtype=float)
    ang_vehicle[0] = data_stage_3[6]
    angw_vehicle[0] = data_stage_3[7]
    gama = atan(vehicle.cmpull_y / vehicle.cmpull_l)
    lat = vehicle.cmpull_h / sin(gama)
    vmx = data_4[:, 3] * np.cos(data_4[:, 2])
    b1 = vehicle.hull_cmy - vehicle.track_fdy
    a1 = vehicle.hull_cmx
    for i in range(1, n + 1):
        dt = time_4[i] - time_4[i - 1]
        sx = (vehicle.hull_h * cos(ang_vehicle[i - 1]) + vehicle.hull_l * sin(ang_vehicle[i - 1])) * vehicle.hull_w
        shb = (b1 * cos(ang_vehicle[i - 1]) + a1 * sin(ang_vehicle[i - 1])) * vehicle.hull_w
        sht = sx - shb
        qwb = 0.5 * drop_envi.air_density * vehicle.resist_hull * shb**2 * vmx[i - 1] ** 2 / (4.0 * vehicle.hull_w)
        qwt = 0.5 * drop_envi.air_density * vehicle.resist_hull * sht**2 * vmx[i - 1] ** 2 / (4.0 * vehicle.hull_w)
        qfl = fk[i - 1] * lat * sin(ang_vehicle[i - 1] + data_4[i - 1, 2] - gama - pi / 2.0)
        if ang_vehicle[i - 1] + data_4[i - 1, 2] == pi / 2.0:
            tw = qwt - qwb
        else:
            tw = qwt - qwb - qfl
        acc_angle = tw / vehicle.izz
        angw_vehicle[i] = angw_vehicle[i - 1] + acc_angle * dt
        ang_vehicle[i] = ang_vehicle[i - 1] + angw_vehicle[i - 1] * dt

    data_jscq = np.zeros(7, dtype=float)
    data_jscq[0] = data_4[n, 0]
    data_jscq[1] = data_4[n, 1]
    data_jscq[2] = data_4[n, 2]
    data_jscq[3] = data_4[n, 3]
    data_jscq[4] = ang_vehicle[n]
    data_jscq[5] = angw_vehicle[n]
    data_jscq[6] = fk[n]

    data_out = np.zeros((n + 1, 7), dtype=float)
    data_out[:, :4] = data_4[: n + 1, :]
    data_out[:, 4] = ang_vehicle
    data_out[:, 5] = angw_vehicle
    data_out[:, 6] = fk

    data_out_2 = np.zeros((n + 1, 2), dtype=float)
    data_out_2[:, 0] = data_4[: n + 1, 3]
    data_out_2[:, 1] = data_4[: n + 1, 2]
    return time_jscq, data_jscq, time_4, data_out, data_out_2
