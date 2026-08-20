import unittest
from pathlib import Path

import numpy as np
from scipy.io import loadmat

from airdrop_py import run_full_simulation, run_mainchute_stages


PROJECT_DIR = Path(__file__).resolve().parent
REFERENCE_PATH = PROJECT_DIR.parent / "debug_stages_full.mat"
REGRESSION_RTOL = 1.0e-6
REGRESSION_ATOL = 1.0e-8


def load_debug_reference() -> dict[str, np.ndarray]:
    """Load one internally consistent MATLAB run snapshot."""

    values = loadmat(REFERENCE_PATH)
    names = [
        "kte_p",
        "vel_p",
        "qys_p",
        "jss_p",
        "zhs_p",
        "sim_p",
        "data_1",
        "time_2",
        "data_2",
        "data_3",
        "data_4",
        "data_5",
        "data_6",
        "data_all",
        "data_pd",
        "data_fz",
    ]
    return {name: np.asarray(values[name], dtype=float).squeeze() for name in names}


class DebugStagesRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if not REFERENCE_PATH.exists():
            raise unittest.SkipTest(f"MATLAB reference not found: {REFERENCE_PATH}")

        cls.reference = load_debug_reference()
        inputs = [
            cls.reference[name]
            for name in ("kte_p", "vel_p", "qys_p", "jss_p", "zhs_p", "sim_p")
        ]
        cls.stages = run_mainchute_stages(
            *inputs,
        )
        cls.full = run_full_simulation(
            *inputs,
        )

    def test_reference_contains_expected_full_output_shape(self) -> None:
        self.assertEqual(self.reference["data_all"].ndim, 2)
        self.assertEqual(self.reference["data_all"].shape[1], 9)

    def test_stage_shapes_match_matlab_debug_snapshot(self) -> None:
        for name in (
            "data_1",
            "data_2",
            "data_3",
            "data_4",
            "data_5",
            "data_6",
        ):
            actual = self.stages[name]
            self.assertEqual(
                np.asarray(actual).shape,
                self.reference[name].shape,
                f"{name} shape differs from debug_stages_full.mat",
            )

    def test_stage_values_match_matlab_debug_snapshot(self) -> None:
        for name in (
            "data_1",
            "data_2",
            "data_3",
            "data_4",
            "data_5",
            "data_6",
        ):
            actual = self.stages[name]
            expected = self.reference[name]
            self.assertEqual(actual.shape, expected.shape, name)
            np.testing.assert_allclose(
                actual,
                expected,
                rtol=REGRESSION_RTOL,
                atol=REGRESSION_ATOL,
                err_msg=f"{name} values differ from debug_stages_full.mat",
            )

    def test_full_data_all_matches_matlab_debug_snapshot(self) -> None:
        actual = self.full["data_all"]
        expected = self.reference["data_all"]
        self.assertEqual(actual.shape, expected.shape)
        np.testing.assert_allclose(
            actual,
            expected,
            rtol=REGRESSION_RTOL,
            atol=REGRESSION_ATOL,
        )

    def test_stage_3_boundary_is_continuous(self) -> None:
        data_fz = self.reference["data_fz"]
        data_3 = self.reference["data_3"]
        self.assertAlmostEqual(data_3[0, 0], -data_fz[0], places=12)
        self.assertAlmostEqual(data_3[0, 1], -data_fz[2], places=12)


if __name__ == "__main__":
    unittest.main()
