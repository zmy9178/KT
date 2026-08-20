import unittest
from pathlib import Path

import numpy as np
from openpyxl import load_workbook

from airdrop_py import (
    DEFAULT_JSS_P,
    DEFAULT_KTE_P,
    DEFAULT_QYS_P,
    DEFAULT_SIM_P,
    DEFAULT_VEL_P,
    DEFAULT_ZHS_P,
    build_context,
    run_full_simulation,
    run_mainchute_stages,
)
from airdrop_py.mainchute_stages import ode45_stage5, ode45_stage6
from airdrop_py.steady_descent import dynsys, sim_wj


PROJECT_DIR = Path(__file__).resolve().parent
REFERENCE_TIME_PATH = PROJECT_DIR / "伞降输出" / "time_all.xlsx"
REFERENCE_DATA_PATH = PROJECT_DIR / "伞降输出" / "data_all.xlsx"


def load_reference_output() -> tuple[np.ndarray, np.ndarray]:
    """Read MATLAB reference time/data workbooks."""

    time_workbook = load_workbook(
        REFERENCE_TIME_PATH,
        data_only=True,
        read_only=True,
    )
    reference_time = np.asarray(
        [
            float(row[0])
            for row in time_workbook.active.iter_rows(
                min_row=1,
                max_col=1,
                values_only=True,
            )
            if row[0] is not None
        ],
        dtype=float,
    )

    data_workbook = load_workbook(
        REFERENCE_DATA_PATH,
        data_only=True,
        read_only=True,
    )
    reference_data = np.asarray(
        [
            [float(value) for value in row[:9]]
            for row in data_workbook.active.iter_rows(
                min_row=1,
                max_col=9,
                values_only=True,
            )
            if row[0] is not None
        ],
        dtype=float,
    )
    return reference_time, reference_data


def load_reference_data() -> np.ndarray:
    """Read only the MATLAB data workbook for data-only regression tests."""

    data_workbook = load_workbook(
        REFERENCE_DATA_PATH,
        data_only=True,
        read_only=True,
    )
    return np.asarray(
        [
            [float(value) for value in row[:9]]
            for row in data_workbook.active.iter_rows(
                min_row=1,
                max_col=9,
                values_only=True,
            )
            if row[0] is not None
        ],
        dtype=float,
    )


class MainChuteStageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.ctx = build_context(
            DEFAULT_KTE_P,
            DEFAULT_VEL_P,
            DEFAULT_QYS_P,
            DEFAULT_JSS_P,
            DEFAULT_ZHS_P,
            DEFAULT_SIM_P,
        )
        cls.result = run_mainchute_stages(
            DEFAULT_KTE_P,
            DEFAULT_VEL_P,
            DEFAULT_QYS_P,
            DEFAULT_JSS_P,
            DEFAULT_ZHS_P,
            DEFAULT_SIM_P,
        )

    def test_stage5_rhs_shape_and_kinematic_derivatives(self) -> None:
        state = np.array(
            [0.0, 0.0, 0.05, 70.0, 70.0, 22.0],
            dtype=float,
        )
        derivative = ode45_stage5(0.0, state, self.ctx)

        self.assertEqual(derivative.shape, (6,))
        self.assertTrue(np.isfinite(derivative).all())
        self.assertAlmostEqual(derivative[0], state[4] * np.cos(state[2]))
        self.assertAlmostEqual(derivative[1], state[4] * np.sin(state[2]))

    def test_mainchute_stages_return_expected_shapes(self) -> None:
        result = self.result

        self.assertEqual(result["data_mslz"].shape, (9,))
        self.assertEqual(result["data_mscq"].shape, (7,))
        self.assertEqual(result["data_5"].shape[1], 9)
        self.assertEqual(result["data_6"].shape[1], 7)
        self.assertEqual(result["data_55"].shape[1], 3)
        self.assertEqual(result["data_66"].shape[1], 2)
        self.assertTrue(np.isfinite(result["data_5"]).all())
        self.assertTrue(np.isfinite(result["data_6"]).all())

        main_diameter = 2.0 * np.sqrt(
            self.ctx.mainchute.chute_aera / np.pi
        )
        main_length = (
            self.ctx.mainchute.side_width
            + self.ctx.decechute.rope_length
            + main_diameter / 2.0
        )
        dece_diameter = 2.0 * np.sqrt(
            self.ctx.decechute.aera / np.pi
        )
        dece_length = (
            self.ctx.decechute.rope_length
            + self.ctx.decechute.chute_width
            + dece_diameter / 2.0
        )
        self.assertGreaterEqual(
            result["data_mslz"][5],
            result["data_jslz"][5],
        )
        self.assertGreaterEqual(
            result["data_mslz"][5],
            main_length + dece_length,
        )

    def test_stage6_rhs_shape(self) -> None:
        result = self.result
        state = np.array(
            [
                result["data_mslz"][0],
                result["data_mslz"][1],
                result["data_mslz"][2],
                result["data_mslz"][4],
            ],
            dtype=float,
        )
        derivative = ode45_stage6(
            0.1,
            state,
            self.ctx,
            result["data_mslz"],
            1.0,
        )

        self.assertEqual(derivative.shape, (4,))
        self.assertTrue(np.isfinite(derivative).all())

    def test_steady_descent_rhs_and_short_run(self) -> None:
        result = self.result
        wind = np.array(
            [
                self.ctx.drop_envi.wind_x,
                self.ctx.drop_envi.wind_y,
                0.0,
            ],
            dtype=float,
        )
        time_wj, data_wj, *_ = sim_wj(
            result["data_mscq"],
            wind,
            result["data_66"],
            self.ctx,
            duration=1.0,
        )

        self.assertEqual(data_wj.shape[1], 8)
        self.assertEqual(time_wj.shape[0], data_wj.shape[0])
        self.assertTrue(np.isfinite(data_wj).all())

        # dynsys uses ten state variables, including the auxiliary space
        # position and velocity states.
        state = np.zeros(10, dtype=float)
        state[1] = result["data_mscq"][2] * 180.0 / np.pi
        state[3] = result["data_mscq"][6]
        derivative, axial_force = dynsys(
            0.0,
            state,
            self._steady_params(),
            self.ctx,
        )
        self.assertEqual(derivative.shape, (10,))
        self.assertTrue(np.isfinite(derivative).all())
        self.assertTrue(np.isfinite(axial_force))

    def _steady_params(self):
        # sim_wj is the public constructor for this model. Reuse its
        # implementation output indirectly by checking the public result.
        # This helper is replaced below by a direct smoke test of sim_wj.
        from airdrop_py.steady_descent import SteadyDescentParams

        return SteadyDescentParams(
            height=self.ctx.drop_envi.drophight,
            m=1.0,
            vehicle_mass=1.0,
            rope_length=1.0,
            ca=1.0,
            ca1=1.0,
            ca2=1.0,
            ca3=1.0,
            inertia=1.0,
            gravity=self.ctx.drop_envi.grav,
            ca22=1.0,
            ca33=1.0,
            mainchute_area=1.0,
            wind_x=0.0,
            wind_y=0.0,
            wind_z=0.0,
            z0=0.0,
        )

    def test_full_pipeline_assembles_nine_columns(self) -> None:
        result = run_full_simulation(
            DEFAULT_KTE_P,
            DEFAULT_VEL_P,
            DEFAULT_QYS_P,
            DEFAULT_JSS_P,
            DEFAULT_ZHS_P,
            DEFAULT_SIM_P,
            include_steady_descent=False,
        )

        self.assertEqual(result["data_all"].shape[1], 9)
        self.assertEqual(
            result["time_all"].shape[0],
            result["data_all"].shape[0],
        )
        self.assertTrue(np.isfinite(result["data_all"]).all())

    @unittest.skip(
        "Replaced by test_debug_stages_regression.py, which uses debug_stages.mat."
    )
    def test_data_output_matches_matlab_reference(self) -> None:
        """Compare all nine data columns with the MATLAB reference workbook."""

        if not REFERENCE_DATA_PATH.exists():
            self.skipTest("伞降输出/data_all.xlsx 不存在")

        result = run_full_simulation(
            DEFAULT_KTE_P,
            DEFAULT_VEL_P,
            DEFAULT_QYS_P,
            DEFAULT_JSS_P,
            DEFAULT_ZHS_P,
            DEFAULT_SIM_P,
        )
        reference_data = load_reference_data()
        python_data = result["data_all"]

        self.assertEqual(reference_data.shape[1], 9)
        self.assertEqual(
            python_data.shape,
            reference_data.shape,
            "Python 与 MATLAB 的 data_all 行列数不一致",
        )
        np.testing.assert_allclose(
            python_data,
            reference_data,
            rtol=2.0e-3,
            atol=2.0e-3,
        )


if __name__ == "__main__":
    unittest.main()
