from __future__ import annotations

from math import atan, cos, pi, sin, sqrt

import numpy as np

from .models import AirdropContext
from .numerics import matlab_time_grid, solve_on_matlab_grid


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

    if y[5] <= lsh:
        me = msh1 * y[5]
        dmi = msh1
    elif y[5] <= lsh + b:
        # Keep the MATLAB expression exactly, including its unshifted y(6).
        me = msh1 * lsh + msy1 * y[5]
        dmi = msy1
    elif y[5] <= lxt:
        me = msh1 * lsh + msy1 * b + (msy - msy1 * b) * (1.0 - 4.0 * ((lxt - y[5]) / do) ** 2)
        dmi = 8.0 * (msy - msy1 * b) * (lxt - y[5]) / do**2
    else:
        me = 0.0
        dmi = 0.01

    mv = msh + msy - me
    relative_velocity = y[4] - y[3]
    fl = dmi * relative_velocity**2 + fsh

    dy = np.zeros(6, dtype=float)
    dy[0] = y[4] * cos(y[2])
    dy[1] = y[4] * sin(y[2])
    dy[2] = g * cos(y[2]) / y[4]
    dy[3] = g * sin(y[2]) - (qys + qd - fsh) / (mys + mv)
    dy[4] = g * sin(y[2]) - (qw + qe + fl) / (mw + me)
    dy[5] = sqrt((fl - fsh) / dmi)
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
    nume = data_stage_2[2] - (vehicle.track_fdy - cmy)
    deno = vehicle.cmpull_h + pullchute.rope_length

    initial = np.zeros(6, dtype=float)
    initial[0] = -1.0 * data_stage_2[0]
    initial[1] = -1.0 * data_stage_2[2]
    initial[2] = atan(nume / deno)
    lvx = drop_envi.airplane_velo / 3.6 + data_stage_2[1]
    lvy = data_stage_2[3]
    initial[3] = lvx * cos(initial[2]) + lvy * sin(initial[2])
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
    time_3, data_3 = solve_on_matlab_grid(lambda t, y: ode45_stage3(t, y, ctx), t_grid, initial)

    hit = np.nonzero(data_3[:, 5] >= lxt)[0]
    if hit.size == 0:
        raise RuntimeError("jiansu_straight did not reach the MATLAB stop condition")
    nn = int(hit[0])
    time_jslz = float(time_3[nn])

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
