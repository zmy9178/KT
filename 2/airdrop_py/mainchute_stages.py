from __future__ import annotations

from math import atan, cos, pi, sin, sqrt

import numpy as np

from .models import AirdropContext
from .numerics import matlab_time_grid, solve_on_matlab_grid


#阻尼
STAGE5_RELATIVE_DAMPING = 0.15
STAGE5_ANGULAR_DAMPING = 0.5
STAGE6_SPEED_DAMPING = 0.1
STAGE6_ANGULAR_DAMPING = 0.5


def _nonzero(value: float, minimum: float = 1.0e-9) -> float:
    """Keep ODE denominators finite when a test state has zero speed."""

    if abs(value) >= minimum:
        return value
    return minimum if value >= 0.0 else -minimum


def mainchute_inflation_state(
    t: float,
    t1: float,
    step_open: float,
    t21: float,
    area: float,
    khalf: float,
) -> tuple[float, float]:
    """Return effective main-chute area and area growth rate."""

    if t <= 0.0:
        return 0.0, 0.0

    half_area = area * khalf
    t2 = t1 + step_open
    t3 = t2 + t21

    if t <= t1:
        return half_area * t / t1, half_area / t1
    if t <= t2:
        return half_area, 0.0
    if t <= t3 and t21 > 0.0:
        ratio = (t - t2) / t21
        sm = half_area + area * (1.0 - khalf) * ratio**2
        dsm = 2.5 * area * (1.0 - khalf) * ratio
        return sm, dsm
    return area, 0.0


def _mainchute_lengths(ctx: AirdropContext) -> tuple[float, float, float]:
    """Return main-chute diameter, deployed-system length, and decelerator length."""

    mainchute = ctx.mainchute
    decechute = ctx.decechute
    main_diameter = 2.0 * sqrt(mainchute.chute_aera / pi)
    # zhusan_straight.m reuses the decelerator rope length as length_rope.
    main_length = decechute.rope_length + mainchute.side_width + main_diameter / 2.0
    dece_diameter = 2.0 * sqrt(decechute.aera / pi)
    dece_length = decechute.rope_length + decechute.chute_width + dece_diameter / 2.0
    return main_diameter, main_length, dece_length


def ode45_stage5(_t: float, y: np.ndarray, ctx: AirdropContext) -> np.ndarray:
    """MATLAB ``ode45_stage5``: main-parachute system straightening."""

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    pullchute = ctx.pullchute
    decechute = ctx.decechute
    mainchute = ctx.mainchute

    rho = drop_envi.air_density
    g = drop_envi.grav
    ca_ys = pullchute.resist * decechute.aera
    ca_js = decechute.resist * decechute.aera * (
        1.05 - 0.05 * mainchute.chute_num
    ) * mainchute.chute_num
    ca_d = mainchute.chute_cover_resist
    ca_w = vehicle.resist_hull
    ca_e = mainchute.pullout_resist

    mw = vehicle.dropmass * 1000.0
    mjs = decechute.mass
    mjsh = decechute.rope_mass
    ljsh = decechute.rope_length
    djs = 2.0 * sqrt(decechute.aera / pi)
    mjsy1 = decechute.chute_side_mass / decechute.chute_width
    ljsb = decechute.chute_width
    mjsy = mjs - mjsh - ljsb * mjsy1

    mys = pullchute.mass
    msh1 = mainchute.rope_mass / mainchute.rope_length
    # ode45_stage5.m uses the main-parachute side density and width here.
    msy1 = mainchute.side_mass / mainchute.side_width
    b = mainchute.side_width
    msh = mainchute.rope_mass
    msy = mainchute.chute_mass
    lsh = mainchute.rope_length
    do = 2.0 * sqrt(mainchute.chute_aera / pi)
    ljs = ljsh + ljsb + djs / 2.0
    lxt = mainchute.side_width + mainchute.rope_length + do / 2.0 + ljs
    fsh = mainchute.pullout_force

    qys = 0.5 * rho * ca_ys * y[3] ** 2
    qd = 0.5 * rho * ca_d * y[3] ** 2
    qjs = 0.5 * rho * ca_js * y[4] ** 2
    qw = 0.5 * rho * ca_w * vehicle.hull_l * vehicle.hull_w * y[4] ** 2
    qe = 0.5 * rho * ca_e * y[3] ** 2

    length = y[5]
    if length <= ljsh:
        me = mjsh / ljsh * length
        dmi = mjsh / ljsh
    elif length <= ljsh + ljsb:
        me = mjsh + mjsy1 * (length - ljsh)
        dmi = mjsy1
    elif length <= ljs:
        me = mjsh + ljsb * mjsy1 + mjsy * (
            1.0 - 4.0 * ((ljs - length) / djs) ** 2
        )
        dmi = 8.0 * mjsy * (ljs - length) / djs**2
    elif length <= ljs + lsh:
        me = mjs + msh1 * (length - ljs)
        dmi = msh1
    elif length <= ljs + lsh + b:
        me = mjs + msh1 * lsh + msy1 * (length - ljs - lsh)
        dmi = msy1
    elif length <= lxt:
        me = mjs + msh1 * lsh + msy1 * b + (msy - msy1 * b) * (
            1.0 - 4.0 * ((lxt - length) / do) ** 2
        )
        dmi = 8.0 * (msy - msy1 * b) * (lxt - length) / do**2
    else:
        me = 0.0
        dmi = 0.001

    mv = mjs + msh + msy - me
    relative_velocity = y[4] - y[3]
    deployment_rate = max(relative_velocity, 0.0)

    fl = dmi * deployment_rate**2 + fsh
    relative_damping = STAGE5_RELATIVE_DAMPING * relative_velocity

    dy = np.zeros(6, dtype=float)
    dy[0] = y[4] * cos(y[2])
    dy[1] = y[4] * sin(y[2])
    dy[2] = g * cos(y[2]) / y[4]
    dy[3] = g * sin(y[2]) - (qys + qd - fsh) / (mys + mv) + relative_damping
    if length < ljs:
        dy[4] = g * sin(y[2]) - (qw + qe + fl) / (mw + me) - relative_damping
    else:
        dy[4] = g * sin(y[2]) - (qw + qe + qjs + fl) / (mw + me) - relative_damping
    dy[5] = deployment_rate
    return dy


def zhusan_straight(
    data_stage_2: np.ndarray,
    data_jslz: np.ndarray,
    ctx: AirdropContext,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Run main-parachute straightening and return MATLAB-compatible outputs."""

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    pullchute = ctx.pullchute
    decechute = ctx.decechute
    mainchute = ctx.mainchute
    sim = ctx.sim

    _, main_length, dece_length = _mainchute_lengths(ctx)


    # data_stage_2[2] 是上一阶段的轨迹角，单位为弧度。
    # 直接继承上一阶段速度状态，避免阶段交接时速度方向瞬间跳变。继承的是弧度，没有重新用几何位置计算一个新的 trajectory_angle。
    trajectory_angle = float(data_stage_2[2])
    initial_speed = float(data_stage_2[3])

    initial = np.zeros(6, dtype=float)
    initial[0] = data_stage_2[0]
    initial[1] = data_stage_2[1]
    initial[2] = trajectory_angle
    initial[3] = initial_speed
    initial[4] = initial_speed
    initial[5] = data_jslz[5]

    lxt = main_length + dece_length
    t_grid = matlab_time_grid(0.0, sim.time, sim.step)

    def main_straightening_complete_event(
        _t: float,
        y: np.ndarray,
    ) -> float:
        # y[5] 是当前已经拉出的伞系统长度。
        return float(y[5] - lxt)

    # 达到主伞整理长度后立即结束本阶段。
    main_straightening_complete_event.terminal = True
    main_straightening_complete_event.direction = 1

    solution = solve_on_matlab_grid(
        lambda t, y: ode45_stage5(t, y, ctx),
        t_grid,
        initial,
        events=main_straightening_complete_event,
        return_result=True,
    )

    if (
        len(solution.t_events) == 0
        or solution.t_events[0].size == 0
    ):
        raise RuntimeError(
            "zhusan_straight did not reach the "
            "main-straightening-complete event"
        )

    time_mslz = float(solution.t_events[0][0])
    event_state = solution.y_events[0][0].copy()

    time_5 = solution.t.copy()
    data_5 = solution.y.T.copy()

    # t_eval 通常不包含非固定网格上的准确事件点。
    if (
        time_5.size == 0
        or not np.isclose(
            time_5[-1],
            time_mslz,
            rtol=0.0,
            atol=1.0e-12,
        )
    ):
        time_5 = np.append(time_5, time_mslz)
        data_5 = np.vstack((data_5, event_state))

    # 追加事件点后，最后一行就是本阶段终止状态。
    n = len(time_5) - 1


    angle_vehicle = np.zeros(n + 1, dtype=float)
    angular_velocity = np.zeros(n + 1, dtype=float)
    angle_vehicle[0] = data_stage_2[4]
    angular_velocity[0] = data_stage_2[5]

    gama = atan(vehicle.cmpull_y / vehicle.cmpull_l)
    lat = vehicle.cmpull_h / sin(gama)
    vmx = data_5[:, 4] * np.cos(data_5[:, 2])

    ljs = dece_length
    ljsh = decechute.rope_length
    ljsb = decechute.chute_width
    djs = 2.0 * sqrt(decechute.aera / pi)
    mjsy1 = decechute.chute_side_mass / decechute.chute_width
    mjsy = decechute.mass - decechute.rope_mass - ljsb * mjsy1
    # Match zhusan_straight.m post-processing.  Here Msh1 is the
    # deceleration-parachute rope density, even though ode45_stage5 uses
    # the main-parachute rope density for its own mass-release model.
    msh1 = decechute.rope_mass / decechute.rope_length
    # zhusan_straight.m post-processing uses the deceleration-parachute
    # side density and width for these released-mass segments.
    msy1 = decechute.chute_side_mass / decechute.chute_width
    b = decechute.chute_width
    msy = mainchute.chute_mass
    do = 2.0 * sqrt(mainchute.chute_aera / pi)
    lsh = decechute.rope_length
    fsh = mainchute.pullout_force
    vr = data_5[: n + 1, 4] - data_5[: n + 1, 3]

    fl = np.zeros(n + 1, dtype=float)
    dmi = 0.0
    for i, length in enumerate(data_5[: n + 1, 5]):
        if length <= ljsh:
            dmi = decechute.rope_mass / ljsh
        elif length <= ljsh + ljsb:
            dmi = mjsy1
        elif length <= ljs:
            dmi = 8.0 * mjsy * (ljs - length) / djs**2
        elif length <= ljs + lsh:
            dmi = decechute.rope_mass / decechute.rope_length
        elif length <= ljs + lsh + b:
            dmi = (
                decechute.chute_side_mass
                / decechute.chute_width
            )
        elif length <= lxt:
            dmi = 8.0 * (msy - msy1 * b) * (lxt - length) / do**2
        # MATLAB has no ``else`` branch here, so Dmi retains its
        # previous value after the terminal length is crossed.
        fl[i] = dmi * vr[i] ** 2 + fsh

    for i in range(1, n + 1):
        dt = time_5[i] - time_5[i - 1]
        sx = (
            vehicle.hull_h * cos(angle_vehicle[i - 1])
            + vehicle.hull_l * sin(angle_vehicle[i - 1])
        ) * vehicle.hull_w
        shb = (
            (vehicle.hull_cmy - vehicle.track_fdy) * cos(angle_vehicle[i - 1])
            + vehicle.hull_cmx * sin(angle_vehicle[i - 1])
        ) * vehicle.hull_w
        sht = sx - shb
        qwb = (
            0.5
            * drop_envi.air_density
            * vehicle.resist_hull
            * shb**2
            * vmx[i - 1] ** 2
            / (4.0 * vehicle.hull_w)
        )
        qwt = (
            0.5
            * drop_envi.air_density
            * vehicle.resist_hull
            * sht**2
            * vmx[i - 1] ** 2
            / (4.0 * vehicle.hull_w)
        )
        qfl = fl[i - 1] * lat * sin(
            angle_vehicle[i - 1] + data_5[i - 1, 2] - pi / 2.0
        )
        tw = qwt - qwb if angle_vehicle[i - 1] + data_5[i - 1, 2] - gama == pi / 2.0 else qwt - qwb - qfl
        tw -= vehicle.izz * STAGE5_ANGULAR_DAMPING * angular_velocity[i - 1]
        angular_acceleration = tw / vehicle.izz
        angular_velocity[i] = angular_velocity[i - 1] + angular_acceleration * dt
        angle_vehicle[i] = angle_vehicle[i - 1] + angular_velocity[i - 1] * dt

    data_mslz = np.zeros(9, dtype=float)
    data_mslz[:6] = data_5[n, :6]
    data_mslz[6] = angle_vehicle[n]
    data_mslz[7] = angular_velocity[n]
    data_mslz[8] = fl[n]

    data_out = np.zeros((n + 1, 9), dtype=float)
    data_out[:, :6] = data_5[: n + 1, :]
    data_out[:, 6] = angle_vehicle
    data_out[:, 7] = angular_velocity
    # Preserve MATLAB zhusan_straight.m, which fills the whole output
    # column with the terminal FL value rather than the full FL history.
    data_out[:, 8] = fl[n]

    data_out_2 = np.zeros((n + 1, 3), dtype=float)
    data_out_2[:, 0] = data_5[: n + 1, 4]
    data_out_2[:, 1] = data_5[: n + 1, 2]
    data_out_2[:, 2] = angle_vehicle
    return float(time_5[n]), data_mslz, time_5, data_out, data_out_2


def ode45_stage6(
    t: float,
    y: np.ndarray,
    ctx: AirdropContext,
    data_stage_55: np.ndarray,
    t21: float,
) -> np.ndarray:
    """MATLAB ``ode45_stage6``: main-parachute inflation."""

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    decechute = ctx.decechute
    mainchute = ctx.mainchute

    d_temp = 6.5e-3
    t0 = 273.0
    rho = drop_envi.air_density * (
        1.0 - d_temp * (drop_envi.drophight - y[1]) / t0
    ) ** 4.26
    g = drop_envi.grav
    number_mainchute = mainchute.chute_num
    yita = 1.05 - 0.05 * number_mainchute
    mw = vehicle.dropmass * 1000.0
    mms = mainchute.chute_mass
    mjs = decechute.mass

    sq = (
        vehicle.hull_h * abs(-pi / 2.0 + y[2])
        + vehicle.hull_l * cos(-pi / 2.0 + y[2]) * vehicle.hull_w
    )
    ca_mst = mainchute.resist_ca * mainchute.chute_aera
    velocity_start = float(data_stage_55[4])
    # [first.m 源码迁移]
    # first.m 第 1391 行明确写的是：
    #     k_ks = mainchute.k_ks;
    # 这里直接按源码使用主伞参数。
    t1 = mainchute.k_ks * sqrt(mainchute.khalf * ca_mst) / velocity_start
    effective_t = max(t - mainchute.deploy_delay, 0.0)
    sm, dsm = mainchute_inflation_state(
        effective_t,
        t1,
        mainchute.step_open,
        t21,
        mainchute.chute_aera,
        mainchute.khalf,
    )

    ca_ms = (
        mainchute.resist_ca
        * yita
        * sm
        * number_mainchute
    )
    ca_js = (
        decechute.resist
        * (1.05 - 0.05 * number_mainchute)
        * decechute.aera
        * number_mainchute
    )
    qjs = 0.5 * rho * ca_js * y[3] ** 2
    qms = 0.5 * rho * ca_ms * y[3] ** 2
    qwq = 0.5 * rho * vehicle.resist_hull * sq * y[3] ** 2
    msf = rho * mainchute.kf * ca_ms**1.5
    dmsf = 1.5 * rho * mainchute.kf * ca_ms**0.5 * dsm
    m_act = mw + mjs + mms
    m_all = m_act + msf
    dy = np.zeros(4, dtype=float)
    dy[0] = y[3] * cos(y[2])
    dy[1] = y[3] * sin(y[2])
    dy[2] = g * cos(y[2]) / y[3]
    dy[3] = (
        g * sin(y[2]) * m_act / m_all
        - (qms + qwq + qjs) / m_all
        - y[3] * dmsf / m_all
        - STAGE6_SPEED_DAMPING * (y[3] - velocity_start)
    )
    return dy


def zhusan_chongqi(
    data_stage_4: np.ndarray,
    ctx: AirdropContext,
    data_stage_55: np.ndarray | None = None,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Run main-parachute inflation and return MATLAB-compatible outputs."""

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    pullchute = ctx.pullchute
    decechute = ctx.decechute
    mainchute = ctx.mainchute
    sim = ctx.sim
    stage_55 = data_stage_4 if data_stage_55 is None else data_stage_55

    initial = np.array(
        [
            data_stage_4[0],
            data_stage_4[1],
            data_stage_4[2],
            data_stage_4[4],
        ],
        dtype=float,
    )
    ca_mst = mainchute.resist_ca * mainchute.chute_aera

    # [first.m 源码迁移]
    # first.m 主程序把 k_ks 定义为 decechute.ks，并将其作为
    # ms_chuqi_para(23) 传入 zhusan_chongqi；函数第 436 行重新取出
    # 该参数，第 474 行直接用于外层后处理的 t1。
    t1 = mainchute.k_ks * sqrt(mainchute.khalf * ca_mst) / initial[3]
    t2 = t1 + mainchute.step_open

    t_grid = matlab_time_grid(0.0, sim.time, sim.step)
    velocity_start = float(data_stage_4[4])
    second_open_time = mainchute.deploy_delay + t2
    first_second_open = np.nonzero(t_grid > second_open_time)[0]
    if first_second_open.size == 0:
        raise RuntimeError("zhusan_chongqi did not reach the second inflation stage")
    velocity_index = max(int(first_second_open[0]) - 1, 0)
    velocity_second = velocity_start
    # [first.m 源码迁移]
    # MATLAB 先执行 ode45(fun, simtime, initial_data)，此时 t21 尚未在
    # zhusan_chongqi 中赋值。ode45_stage6 因而在 t > t2 后走 Sm=aera_mainchute
    # 的 else 分支。随后 MATLAB 才从这次 data_6 计算 VL2 和 t21；不会用
    # 新 t21 重新积分 data_6。
    time_6, data_6 = solve_on_matlab_grid(
        lambda t, y: ode45_stage6(t, y, ctx, stage_55, 0.0),
        t_grid,
        initial,
    )
    if data_6.shape[0] > 0:
        velocity_second = float(data_6[velocity_index, 3])

    t21 = mainchute.n_ks * (
        sqrt(ca_mst) - sqrt(mainchute.khalf * ca_mst)
    ) / velocity_second

    yita = 1.05 - 0.05 * mainchute.chute_num
    rho = drop_envi.air_density * (
        1.0 - 6.5e-3 * (drop_envi.drophight - data_stage_4[1]) / 273.0
    ) ** 4.26
    m_act = vehicle.dropmass * 1000.0 + pullchute.mass + mainchute.chute_mass
    m_all = m_act
    n_total = len(time_6)
    fk = np.zeros(n_total, dtype=float)
    dsm_values = np.zeros(n_total, dtype=float)
    ca_values = np.zeros(n_total, dtype=float)

    t3 = t2 + t21
    inflation_complete_time = mainchute.deploy_delay + t3
    if n_total == 0 or time_6[-1] < inflation_complete_time:
        raise RuntimeError(
            "zhusan_chongqi simulation time is too short for main-chute "
            f"inflation; need at least {inflation_complete_time:.6f} s"
        )
    for i in range(1, n_total):
        effective_t = max(time_6[i] - mainchute.deploy_delay, 0.0)
        sm, dsm = mainchute_inflation_state(
            effective_t,
            t1,
            mainchute.step_open,
            t21,
            mainchute.chute_aera,
            mainchute.khalf,
        )
        ca_values[i] = mainchute.resist_ca * yita * sm
        dsm_values[i] = dsm
        dt = time_6[i] - time_6[i - 1]
        dv = (data_6[i - 1, 3] - data_6[i, 3]) / dt
        fk[i] = (
            m_all * dv
            + 0.5 * rho * data_6[i, 3] ** 2 * ca_values[i]
            + data_6[i, 3] * dsm
            + (pullchute.mass + mainchute.chute_mass)
            * drop_envi.grav
            * sin(data_6[i, 2])
            + m_all * drop_envi.grav
        )

    angle_vehicle = np.zeros(n_total, dtype=float)
    angular_velocity = np.zeros(n_total, dtype=float)
    angle_vehicle[0] = data_stage_4[6]
    angular_velocity[0] = data_stage_4[7]
    gama = atan(vehicle.cmpull_y / vehicle.cmpull_l)
    lat = vehicle.cmpull_h / sin(gama)
    vmx = data_6[:, 3] * angle_vehicle
    b1 = vehicle.hull_cmy - vehicle.track_fdy
    a1 = vehicle.hull_cmx

    for i in range(1, n_total):
        dt = time_6[i] - time_6[i - 1]
        sx = (
            vehicle.hull_h * cos(angle_vehicle[i - 1])
            + vehicle.hull_l * sin(angle_vehicle[i - 1])
        ) * vehicle.hull_w
        shb = (
            b1 * cos(angle_vehicle[i - 1])
            + a1 * sin(angle_vehicle[i - 1])
        ) * vehicle.hull_w
        sht = sx - shb
        qwb = (
            0.5
            * drop_envi.air_density
            * vehicle.resist_hull
            * shb**2
            * vmx[i - 1] ** 2
            / (4.0 * vehicle.hull_w)
        )
        qwt = (
            0.5
            * drop_envi.air_density
            * vehicle.resist_hull
            * sht**2
            * vmx[i - 1] ** 2
            / (4.0 * vehicle.hull_w)
        )
        qfl = (
            fk[i - 1]
            * lat
            * sin(angle_vehicle[i - 1] + data_6[i - 1, 2] - pi / 2.0)
            + 5000.0 * angular_velocity[i - 1] * lat
        )

        tw = qwt - qwb if angle_vehicle[i - 1] + data_6[i - 1, 2] == pi / 2.0 else qwt - qwb - qfl
        tw -= vehicle.izz * STAGE6_ANGULAR_DAMPING * angular_velocity[i - 1]

        angular_acceleration = tw / vehicle.izz
        angular_velocity[i] = angular_velocity[i - 1] + angular_acceleration * dt
        angle_vehicle[i] = angle_vehicle[i - 1] + angular_velocity[i - 1] * dt

    # Search only after the nominal stable window and full inflation.
    start = max(
        int(8.0 / sim.step) - 1,
        int(np.searchsorted(time_6, inflation_complete_time, side="left")),
        0,
    )
    changes = np.abs(np.diff(fk))
    stable = np.nonzero(changes[start:] <= 1.0)[0]
    stop_index = int(start + stable[0]) if stable.size else n_total - 1

    data_mscq = np.array(
        [
            data_6[stop_index, 0],
            data_6[stop_index, 1],
            data_6[stop_index, 2],
            data_6[stop_index, 3],
            fk[stop_index],
            angle_vehicle[stop_index],
            angular_velocity[stop_index],
        ],
        dtype=float,
    )
    data_out = np.column_stack(
        (
            data_6[: stop_index + 1],
            fk[: stop_index + 1],
            angle_vehicle[: stop_index + 1],
            angular_velocity[: stop_index + 1],
        )
    )
    data_out_2 = np.column_stack(
        (data_6[: stop_index + 1, 3], data_6[: stop_index + 1, 2])
    )
    # MATLAB returns the post-processed prefix as ``time`` and ``data``;
    # the full ODE history remains available through ``data_66``.
    return (
        float(time_6[stop_index]),
        data_mscq,
        time_6[: stop_index + 1],
        data_out,
        data_out_2,
    )
