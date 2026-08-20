"""Default parameter vectors copied from the MATLAB source comments."""

from __future__ import annotations

import re
from collections.abc import Callable

import numpy as np

DEFAULT_KTE_P = [0, 800, 320, 0, 5.75, -1.24, 6, 7, 25]

DEFAULT_VEL_P = [
    15.2,
    4.96e4,
    2.64e4,
    -3.279,
    -0.407,
    5.6,
    3.2,
    2.2,
    0.506,
    4,
    1.614,
    -0.34,
    -0.698,
    -1.209,
    -4.9,
    -5.75,
    -4.9,
    0.83,
    0.05,
]

DEFAULT_QYS_P = [20.2, 0.8, 14, 5.55]

DEFAULT_JSS_P = [10, 0.8, 6.8, 0.01, 2, 8, 22, 4, 1, 8, 0.9]

DEFAULT_ZHS_P = [
    760,
    9,
    58.767,
    0.026,
    6.618,
    1,
    0.8,
    0.8,
    0.8,
    0.6,
    200,
    56,
    24.734,
    80.425,
    1,
    0.16,
    23.5,
    4.3,
    5,
]

DEFAULT_SIM_P = [50, 0.001]


def _parse_vector_text(text: str, *, expected_length: int, name: str) -> list[float]:
    """Parse one comma/space separated parameter vector from user input."""

    parts = [part for part in re.split(r"[\s,，]+", text.strip()) if part]
    values = [float(part) for part in parts]
    if len(values) != expected_length:
        raise ValueError(f"{name} must contain {expected_length} values; got {len(values)}")
    if not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must contain only finite numeric values")
    return values


def prompt_vector(
    name: str,
    default: list[float],
    *,
    input_func: Callable[[str], str] = input,
) -> list[float]:
    """Prompt for one parameter vector and fall back to the default on Enter."""

    default_text = ", ".join(str(value) for value in default)
    print(f"{name} [{len(default)} values]")
    print(f"  default: {default_text}")
    while True:
        raw = input_func("  enter values, or press Enter to keep default: ").strip()
        if not raw:
            return list(default)
        try:
            return _parse_vector_text(raw, expected_length=len(default), name=name)
        except ValueError as exc:
            print(f"  invalid input: {exc}")


def prompt_all_defaults(
    *,
    input_func: Callable[[str], str] = input,
) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float]]:
    """Prompt for all simulation vectors while keeping defaults available."""

    return (
        prompt_vector("kte_p", DEFAULT_KTE_P, input_func=input_func),
        prompt_vector("vel_p", DEFAULT_VEL_P, input_func=input_func),
        prompt_vector("qys_p", DEFAULT_QYS_P, input_func=input_func),
        prompt_vector("jss_p", DEFAULT_JSS_P, input_func=input_func),
        prompt_vector("zhs_p", DEFAULT_ZHS_P, input_func=input_func),
        prompt_vector("sim_p", DEFAULT_SIM_P, input_func=input_func),
    )
