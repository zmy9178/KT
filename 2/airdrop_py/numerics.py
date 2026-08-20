from __future__ import annotations

from collections.abc import Callable

import numpy as np
from scipy.integrate import solve_ivp
from scipy.integrate._ivp.rk import RK45, rk_step


class MatlabRK45(RK45):
    """Dormand-Prince solver with MATLAB-style componentwise error control.

    SciPy's RK45 uses an RMS error norm. MATLAB's default ``ode45`` controls
    each state component against its own ``AbsTol + RelTol * scale`` and uses
    the largest normalized component. The distinction matters at the
    discontinuous stage stop conditions used by this model.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if kwargs.get("first_step") is None:
            self.h_abs = self._select_matlab_initial_step()

    def _select_matlab_initial_step(self) -> float:
        """Use the componentwise initial-step estimate used by ode45."""

        interval_length = abs(self.t_bound - self.t)
        if interval_length == 0.0:
            return 0.0

        scale = np.maximum(self.atol, self.rtol * np.abs(self.y))
        d0 = float(np.max(np.abs(self.y / scale)))
        d1 = float(np.max(np.abs(self.f / scale)))
        if d0 < 1.0e-5 or d1 < 1.0e-5:
            h0 = 1.0e-6
        else:
            h0 = 0.01 * d0 / d1
        h0 = min(h0, interval_length)

        y1 = self.y + h0 * self.direction * self.f
        f1 = self.fun(self.t + h0 * self.direction, y1)
        d2 = float(np.max(np.abs((f1 - self.f) / scale))) / h0
        if d1 <= 1.0e-15 and d2 <= 1.0e-15:
            h1 = max(1.0e-6, h0 * 1.0e-3)
        else:
            h1 = (0.01 / max(d1, d2)) ** (1.0 / 5.0)
        return min(100.0 * h0, h1, interval_length, self.max_step)

    def _step_impl(self):
        """Apply MATLAB-like error scaling and step growth limits."""

        t = self.t
        y = self.y
        min_step = 10.0 * np.abs(
            np.nextafter(t, self.direction * np.inf) - t
        )
        h_abs = min(max(self.h_abs, min_step), self.max_step)
        step_rejected = False

        while True:
            if h_abs < min_step:
                return False, self.TOO_SMALL_STEP

            h = h_abs * self.direction
            t_new = t + h
            if self.direction * (t_new - self.t_bound) > 0:
                t_new = self.t_bound
            h = t_new - t
            h_abs = np.abs(h)

            y_new, f_new = rk_step(
                self.fun,
                t,
                y,
                self.f,
                h,
                self.A,
                self.B,
                self.C,
                self.K,
            )
            scale = np.maximum(
                self.atol,
                self.rtol * np.maximum(np.abs(y), np.abs(y_new)),
            )
            error_norm = self._estimate_error_norm(self.K, h, scale)

            if error_norm < 1.0:
                if error_norm == 0.0:
                    factor = 5.0
                else:
                    factor = min(
                        5.0,
                        0.8 * error_norm**self.error_exponent,
                    )
                if step_rejected:
                    factor = min(1.0, factor)
                h_abs *= factor
                break

            h_abs *= max(
                0.1,
                0.8 * error_norm**self.error_exponent,
            )
            step_rejected = True

        self.h_previous = h
        self.y_old = y
        self.t = t_new
        self.y = y_new
        self.h_abs = h_abs
        self.f = f_new
        return True, None

    def _estimate_error_norm(
        self,
        K: np.ndarray,
        h: float,
        scale: np.ndarray,
    ) -> float:
        normalized_error = self._estimate_error(K, h) / scale
        return float(np.max(np.abs(normalized_error)))


def matlab_time_grid(start: float, stop: float, step: float) -> np.ndarray:
    """Return the same fixed output grid style as MATLAB ``start:step:stop``."""

    count = int(np.floor((stop - start) / step + 1e-12)) + 1
    return start + np.arange(count, dtype=float) * step


def solve_on_matlab_grid(
    fun: Callable[[float, np.ndarray], np.ndarray],
    t_grid: np.ndarray,
    initial: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Solve an ODE and return data as ``(time, rows_by_state)`` like MATLAB."""

    first_step = (
        abs(float(t_grid[1]) - float(t_grid[0]))
        if len(t_grid) > 1
        else None
    )
    result = solve_ivp(
        fun,
        (float(t_grid[0]), float(t_grid[-1])),
        np.asarray(initial, dtype=float),
        method=MatlabRK45,
        t_eval=t_grid,
        # MATLAB receives the full vector ``0:step:sim_time`` as tspan.
        # Its first integration step is constrained by the first requested
        # output interval; passing only the two endpoints changes the
        # adaptive trajectory at discontinuous stage boundaries.
        first_step=first_step,
        rtol=1e-3,
        atol=1e-6,
        # MATLAB ode45 defaults MaxStep to 10% of the integration interval.
        # Leaving this infinite lets SciPy take a much larger step than MATLAB
        # and changes the interpolated values on the fixed output grid.
        max_step=0.1 * abs(float(t_grid[-1]) - float(t_grid[0])),
    )
    if not result.success:
        raise RuntimeError(result.message)
    return result.t, result.y.T
