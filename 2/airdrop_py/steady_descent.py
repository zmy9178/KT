from __future__ import annotations

from dataclasses import dataclass
from math import atan, cos, pi, sin

import numpy as np
from scipy.integrate import solve_ivp

from .models import AirdropContext
from .numerics import MatlabRK45


@dataclass(frozen=True)
class SteadyDescentParams:
    """Parameters used by the MATLAB ``dynsys`` steady-descent model."""

    height: float
    m: float
    vehicle_mass: float
    rope_length: float
    ca: float
    ca1: float
    ca2: float
    ca3: float
    inertia: float
    gravity: float
    ca22: float
    ca33: float
    mainchute_area: float
    wind_x: float
    wind_y: float
    wind_z: float
    z0: float


def _rotation_matrix(theta_deg: float, beta_deg: float) -> np.ndarray:
    """Return the body-to-space matrix used by the MATLAB source."""

    theta = np.deg2rad(theta_deg)
    beta = np.deg2rad(beta_deg)
    return np.array(
        [
            [
                -cos(theta) * cos(beta),
                -sin(beta),
                -sin(theta) * cos(beta),
            ],
            [
                -cos(theta) * sin(beta),
                cos(beta),
                -sin(theta) * sin(beta),
            ],
            [-sin(theta), 0.0, cos(theta)],
        ],
        dtype=float,
    )


def dynsys(
    _t: float,
    state: np.ndarray,
    params: SteadyDescentParams,
    ctx: AirdropContext,
) -> tuple[np.ndarray, float]:
    """Translate MATLAB ``dynsys`` and return derivatives plus axial force."""

    state = np.asarray(state, dtype=float)
    beta, theta, u, w = state[:4]
    x_body, vx_body = state[4:6]
    che_vx, che_vy = state[6:8]

    rotation = _rotation_matrix(theta, beta)
    drop_envi = ctx.drop_envi
    density_air = drop_envi.air_density
    d_temp = 6.5e-3
    t0 = drop_envi.temperature + 273.0

    displacement = rotation.T @ np.array([x_body, 0.0, 0.0])
    if -displacement[2] + params.z0 <= params.height - drop_envi.land_hight:
        h = params.height - (-displacement[2] + params.z0)
    else:
        h = drop_envi.land_hight
    rho = density_air * (1.0 - d_temp * h / t0) ** 4.26

    total = params.m + params.vehicle_mass
    wind_body = rotation @ np.array(
        [params.wind_x, params.wind_y, params.wind_z],
        dtype=float,
    )
    wind_x_body, wind_y_body, wind_z_body = wind_body
    vehicle_velocity = rotation.T @ np.array([vx_body, 0.0, 0.0])

    relative_x = -vehicle_velocity[2] - wind_x_body
    relative_y = che_vy - wind_y_body
    relative_z = che_vx + wind_z_body
    velocity_wm = w * (params.m / total) * params.rope_length + wind_z_body
    velocity_wm_small = w * (params.vehicle_mass / total) * params.rope_length - wind_z_body
    velocity_um = u * (params.m / total) * params.rope_length + wind_y_body
    velocity_um_small = u * (params.vehicle_mass / total) * params.rope_length - wind_y_body

    def signed_square(value: float) -> float:
        return value * abs(value)

    delta_x_sq = signed_square(relative_x)
    delta_y_sq = signed_square(relative_y)
    delta_z_sq = signed_square(relative_z)
    delta_wm_sq = signed_square(velocity_wm)
    delta_wm_small_sq = signed_square(velocity_wm_small)
    delta_um_sq = signed_square(velocity_um)
    delta_um_small_sq = signed_square(velocity_um_small)

    du_dt = (
        -0.5
        * rho
        * delta_um_sq
        * params.ca2
        * (params.m / total)
        * params.rope_length
        - 0.5
        * rho
        * delta_um_small_sq
        * (params.vehicle_mass / total)
        * params.rope_length
        * params.ca22
    ) / params.inertia
    dbeta_dt = u
    dw_dt = (
        params.vehicle_mass
        * params.gravity
        * cos(np.deg2rad(theta))
        * (params.m / total)
        * params.rope_length
        - 0.5
        * rho
        * delta_wm_small_sq
        * (params.vehicle_mass / total)
        * params.rope_length
        * params.ca33
    ) / params.inertia
    dtheta_dt = w if abs(dw_dt) > 0.005 else 0.0

    ax_body = (
        total * params.gravity * sin(np.deg2rad(theta))
        - 0.5 * rho * delta_x_sq * params.ca
        - 0.5 * rho * delta_x_sq * params.ca1
    ) / total
    ay_body = (
        -0.5 * rho * delta_y_sq * params.ca22
        - 0.5 * rho * delta_y_sq * params.ca2
    ) / total
    az_body = (
        -0.5 * rho * delta_z_sq * params.ca33
        - 0.5 * rho * delta_z_sq * params.ca3
    ) / total
    axial_force = -0.5 * rho * delta_x_sq * params.mainchute_area

    derivatives = np.zeros(10, dtype=float)
    derivatives[0] = dbeta_dt
    derivatives[1] = dtheta_dt
    derivatives[2] = du_dt
    derivatives[3] = dw_dt
    derivatives[4] = vx_body
    derivatives[5] = ax_body
    derivatives[6] = az_body
    derivatives[7] = ay_body
    derivatives[8] = che_vx
    derivatives[9] = che_vy
    return derivatives, float(axial_force)


def sim_wj(
    data_mscq: np.ndarray,
    wind: np.ndarray,
    data_66: np.ndarray,
    ctx: AirdropContext,
    *,
    duration: float = 200.0,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """Run the MATLAB steady-descent stage and convert body to space axes."""

    vehicle = ctx.vehicle
    drop_envi = ctx.drop_envi
    mainchute = ctx.mainchute
    pullchute = ctx.pullchute
    decechute = ctx.decechute

    wind = np.asarray(wind, dtype=float).reshape(3)
    data_mscq = np.asarray(data_mscq, dtype=float).reshape(-1)
    data_66 = np.asarray(data_66, dtype=float)

    theta0 = np.rad2deg(data_mscq[2])
    beta0 = 0.0
    u0 = 0.0
    w0 = data_mscq[6]
    x0 = 0.0
    vx0 = data_mscq[3]

    main_count = mainchute.chute_num
    main_area = mainchute.chute_aera
    rope_length = mainchute.rope_length
    parachute_mass = (
        mainchute.rope_mass + mainchute.side_mass + mainchute.chute_mass
    ) * main_count
    vehicle_mass = (
        vehicle.dropmass * 1000.0
        - parachute_mass
        - pullchute.mass
        - (
            decechute.chute_side_mass
            + decechute.mass
            + decechute.rope_mass * decechute.rope_num
        )
    )
    total = parachute_mass + vehicle_mass
    inertia = parachute_mass * vehicle_mass * rope_length**2 / total
    efficiency = 1.05 - main_count * 0.05

    ca = main_area * mainchute.resist_ca * efficiency * main_count
    ca22 = main_area * mainchute.resist_cn * efficiency * main_count
    ca33 = main_area * mainchute.resist_cm * efficiency * main_count
    ca3 = 0.6 * vehicle.hull_l * vehicle.hull_w
    ca2 = 0.6 * vehicle.hull_l * vehicle.hull_h
    ca1 = 0.4 * vehicle.hull_w * vehicle.hull_h

    params = SteadyDescentParams(
        height=drop_envi.drophight,
        m=parachute_mass,
        vehicle_mass=vehicle_mass,
        rope_length=rope_length,
        ca=ca,
        ca1=ca1,
        ca2=ca2,
        ca3=ca3,
        inertia=inertia,
        gravity=drop_envi.grav,
        ca22=ca22,
        ca33=ca33,
        mainchute_area=main_area,
        wind_x=wind[0],
        wind_y=wind[1],
        wind_z=wind[2],
        z0=data_mscq[1],
    )

    if data_66.size:
        last_speed = data_66[-1, 0]
        last_angle = np.deg2rad(data_66[-1, 1])
        initial_horizontal = last_speed * cos(last_angle)
    else:
        initial_horizontal = vx0

    initial = np.array(
        [beta0, theta0, u0, w0, x0, vx0, initial_horizontal, 0.0, 0.0, 0.0],
        dtype=float,
    )
    result = solve_ivp(
        lambda t, state: dynsys(t, state, params, ctx)[0],
        (0.0, duration),
        initial,
        method=MatlabRK45,
        rtol=1.0e-3,
        atol=1.0e-6,
        # MATLAB ode45 default: MaxStep = 0.1 * (tfinal - t0).
        max_step=0.1 * abs(float(duration)),
    )
    if not result.success:
        raise RuntimeError(result.message)

    time_values: list[float] = []
    data_values: list[list[float]] = []
    x_values: list[float] = []
    y_values: list[float] = []
    z_values: list[float] = []
    vx_values: list[float] = []
    vy_values: list[float] = []
    vz_values: list[float] = []

    for time, state in zip(result.t, result.y.T):
        beta = state[0]
        theta = state[1]
        rotation = _rotation_matrix(theta, beta)
        body_position = rotation.T @ np.array([state[4], 0.0, 0.0])
        body_velocity = rotation.T @ np.array([state[5], 0.0, 0.0])
        altitude = -body_position[2] + data_mscq[1]
        if altitude > params.height:
            continue
        _, axial_force = dynsys(time, state, params, ctx)
        side_angle = atan(theta / beta) if beta != 0.0 else pi / 2.0
        time_values.append(float(time))
        data_values.append(
            [
                state[0],
                state[1],
                state[2],
                state[3],
                state[4],
                state[5],
                axial_force / drop_envi.grav,
                side_angle,
            ]
        )
        vx_values.append(float(state[6]))
        vy_values.append(float(state[7]))
        vz_values.append(float(-body_velocity[2]))
        x_values.append(float(state[8] + data_mscq[0]))
        y_values.append(float(state[9]))
        z_values.append(float(altitude))

    return (
        np.asarray(time_values, dtype=float),
        np.asarray(data_values, dtype=float).reshape(-1, 8),
        np.asarray(x_values, dtype=float),
        np.asarray(y_values, dtype=float),
        np.asarray(z_values, dtype=float),
        np.asarray(vx_values, dtype=float),
        np.asarray(vy_values, dtype=float),
        np.asarray(vz_values, dtype=float),
    )
