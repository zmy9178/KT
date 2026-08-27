from __future__ import annotations

from dataclasses import dataclass
from math import exp

import numpy as np


@dataclass(frozen=True)
class DropEnvironment:
    """Environment and release-condition parameters from MATLAB ``drop_envi``."""

    land_hight: float
    grav: float
    earth_r: float
    air_density: float
    drophight: float
    airplane_velo: float
    cargo_angle: float
    load_posi_x: float
    load_posi_y: float
    wind_x: float
    wind_y: float
    temperature: float


@dataclass(frozen=True)
class Vehicle:
    """Air-droppable vehicle parameters from MATLAB ``Vehicle``."""

    dropmass: float
    izz: float
    ixx: float
    hull_cmx: float
    hull_cmy: float
    hull_l: float
    hull_w: float
    hull_h: float
    cmpull_y: float
    cmpull_h: float
    cmpull_l: float
    pull_x: float
    pull_y: float
    track_fdx: float
    track_fdy: float
    track_bdx: float
    track_bdy: float
    resist_hull: float
    mu: float


@dataclass(frozen=True)
class PullChute:
    """Extraction parachute parameters from MATLAB ``pullchute``."""

    mass: float
    resist: float
    area: float
    rope_length: float


@dataclass(frozen=True)
class DeceChute:
    """Deceleration parachute parameters from MATLAB ``decechute``."""

    mass: float
    resist: float
    aera: float
    chute_width: float
    chute_side_mass: float
    rope_num: float
    rope_length: float
    rope_mass: float
    kf: float
    ks: float
    ksn: float


@dataclass(frozen=True)
class MainChute:
    """Main parachute parameters from MATLAB ``mainchute``."""

    chute_aera: float
    chute_num: float
    chute_mass: float
    side_width: float
    side_mass: float
    resist_ca: float
    resist_cn: float
    resist_cm: float
    chute_cover_resist: float
    pullout_resist: float
    pullout_force: float
    rope_num: float
    rope_mass: float
    rope_length: float
    kf: float
    khalf: float
    k_ks: float
    n_ks: float
    step_open: float
    deploy_delay: float = 2.0


@dataclass(frozen=True)
class SimulationConfig:
    """Simulation time limit and fixed MATLAB-style output step."""

    time: float
    step: float


@dataclass(frozen=True)
class AirdropContext:
    """Explicit replacement for the MATLAB global parameter structs."""

    drop_envi: DropEnvironment
    vehicle: Vehicle
    pullchute: PullChute
    decechute: DeceChute
    mainchute: MainChute
    sim: SimulationConfig


def as_parameter_vector(
    values: list[float] | np.ndarray,
    *,
    name: str,
    expected_length: int | tuple[int, ...],
) -> np.ndarray:
    """Convert one MATLAB input vector to a validated 1-D float array."""

    vector = np.asarray(values, dtype=float).reshape(-1)
    expected_lengths = (
        (expected_length,)
        if isinstance(expected_length, int)
        else tuple(expected_length)
    )
    if vector.size not in expected_lengths:
        expected_text = " or ".join(str(length) for length in expected_lengths)
        raise ValueError(f"{name} must contain {expected_text} values; got {vector.size}")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain only finite numeric values")
    return vector


def build_context(
    kte_p: list[float] | np.ndarray,
    vel_p: list[float] | np.ndarray,
    qys_p: list[float] | np.ndarray,
    jss_p: list[float] | np.ndarray,
    zhs_p: list[float] | np.ndarray,
    sim_p: list[float] | np.ndarray,
) -> AirdropContext:
    """Build the explicit Python context from MATLAB-compatible input vectors."""

    kte_p = as_parameter_vector(kte_p, name="kte_p", expected_length=9)
    vel_p = as_parameter_vector(vel_p, name="vel_p", expected_length=19)
    qys_p = as_parameter_vector(qys_p, name="qys_p", expected_length=4)
    jss_p = as_parameter_vector(jss_p, name="jss_p", expected_length=11)
    zhs_p = as_parameter_vector(zhs_p, name="zhs_p", expected_length=(19, 20))
    sim_p = as_parameter_vector(sim_p, name="sim_p", expected_length=2)
    if sim_p[0] <= 0.0 or sim_p[1] <= 0.0:
        raise ValueError("sim_p time and step must both be positive")
    deploy_delay = float(zhs_p[19]) if zhs_p.size == 20 else 0.0
    if zhs_p[16] <= 0.0:
        raise ValueError("mainchute k_ks must be positive")
    if zhs_p[17] <= 0.0:
        raise ValueError("mainchute n_ks must be positive")
    if zhs_p[18] < 0.0:
        raise ValueError("mainchute step_open must be non-negative")
    if deploy_delay < 0.0:
        raise ValueError("mainchute deploy_delay must be non-negative")

    land_hight = kte_p[0]
    earth_mass = 5.97e24
    earth_radius = 6_370_000.0
    big_g = 6.67e-11
    rs = 287.0
    p0 = 101_325.0
    pressure = p0 * exp((-1.0 * land_hight / 7924.0))
    temperature_k = 273.15 + kte_p[8]
    air_density = pressure / (rs * temperature_k)
    grav = (big_g * earth_mass) / (earth_radius + land_hight) ** 2

    return AirdropContext(
        drop_envi=DropEnvironment(
            land_hight=land_hight,
            grav=grav,
            earth_r=earth_radius,
            air_density=air_density,
            drophight=kte_p[1],
            airplane_velo=kte_p[2],
            cargo_angle=kte_p[3],
            load_posi_x=abs(kte_p[4]),
            load_posi_y=kte_p[5],
            wind_x=kte_p[6],
            wind_y=kte_p[7],
            temperature=kte_p[8],
        ),
        vehicle=Vehicle(
            dropmass=vel_p[0],
            izz=vel_p[1],
            ixx=vel_p[2],
            hull_cmx=-1.0 * vel_p[3],
            hull_cmy=vel_p[4],
            hull_l=vel_p[5],
            hull_w=vel_p[6],
            hull_h=vel_p[7],
            cmpull_y=vel_p[8],
            cmpull_h=vel_p[9],
            cmpull_l=vel_p[10],
            pull_x=-1.0 * vel_p[11],
            pull_y=vel_p[12],
            track_fdx=vel_p[13],
            track_fdy=vel_p[14],
            track_bdx=vel_p[15],
            track_bdy=vel_p[16],
            resist_hull=vel_p[17],
            mu=vel_p[18],
        ),
        pullchute=PullChute(
            mass=qys_p[0],
            resist=qys_p[1],
            area=qys_p[2],
            rope_length=qys_p[3],
        ),
        decechute=DeceChute(
            mass=jss_p[0],
            resist=jss_p[1],
            aera=jss_p[2],
            chute_width=jss_p[3],
            chute_side_mass=jss_p[4],
            rope_num=jss_p[5],
            rope_length=jss_p[6],
            rope_mass=jss_p[7],
            kf=jss_p[8],
            ks=jss_p[9],
            ksn=jss_p[10],
        ),
        mainchute=MainChute(
            chute_aera=zhs_p[0],
            chute_num=zhs_p[1],
            chute_mass=zhs_p[2],
            side_width=zhs_p[3],
            side_mass=zhs_p[4],
            resist_ca=zhs_p[5],
            resist_cn=zhs_p[6],
            resist_cm=zhs_p[7],
            chute_cover_resist=zhs_p[8],
            pullout_resist=zhs_p[9],
            pullout_force=zhs_p[10],
            rope_num=zhs_p[11],
            rope_mass=zhs_p[12],
            rope_length=zhs_p[13],
            kf=zhs_p[14],
            khalf=zhs_p[15],
            k_ks=zhs_p[16],
            n_ks=zhs_p[17],
            step_open=zhs_p[18],
            deploy_delay=deploy_delay,
        ),
        sim=SimulationConfig(time=sim_p[0], step=sim_p[1]),
    )
