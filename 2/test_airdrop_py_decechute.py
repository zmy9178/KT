import unittest

import numpy as np

from airdrop_py import (
    DEFAULT_JSS_P,
    DEFAULT_KTE_P,
    DEFAULT_QYS_P,
    DEFAULT_SIM_P,
    DEFAULT_VEL_P,
    DEFAULT_ZHS_P,
    build_context,
    run_decechute_stages,
    run_exit_stages,
)


class AirdropPackageTests(unittest.TestCase):
    def test_context_mapping_preserves_matlab_sign_conventions(self) -> None:
        ctx = build_context(
            DEFAULT_KTE_P,
            DEFAULT_VEL_P,
            DEFAULT_QYS_P,
            DEFAULT_JSS_P,
            DEFAULT_ZHS_P,
            DEFAULT_SIM_P,
        )

        self.assertEqual(ctx.drop_envi.load_posi_x, abs(DEFAULT_KTE_P[4]))
        self.assertEqual(ctx.vehicle.hull_cmx, -DEFAULT_VEL_P[3])
        self.assertEqual(ctx.vehicle.pull_x, -DEFAULT_VEL_P[11])
        self.assertEqual(ctx.decechute.aera, DEFAULT_JSS_P[2])
        self.assertEqual(ctx.mainchute.chute_aera, DEFAULT_ZHS_P[0])

    def test_exit_stage_package_api_runs(self) -> None:
        result = run_exit_stages(
            DEFAULT_KTE_P,
            DEFAULT_VEL_P,
            DEFAULT_QYS_P,
            DEFAULT_JSS_P,
            DEFAULT_ZHS_P,
            DEFAULT_SIM_P,
        )

        self.assertEqual(result["data_pd"].shape, (4,))
        self.assertEqual(result["data_fz"].shape, (6,))
        self.assertEqual(result["data_1"].shape[1], 4)
        self.assertEqual(result["data_2"].shape[1], 6)
        self.assertTrue(np.isfinite(result["data_1"]).all())
        self.assertTrue(np.isfinite(result["data_2"]).all())

    def test_decechute_stages_run_and_reach_stop_conditions(self) -> None:
        result = run_decechute_stages(
            DEFAULT_KTE_P,
            DEFAULT_VEL_P,
            DEFAULT_QYS_P,
            DEFAULT_JSS_P,
            DEFAULT_ZHS_P,
            DEFAULT_SIM_P,
        )
        ctx = result["ctx"]

        self.assertEqual(result["data_jslz"].shape, (9,))
        self.assertEqual(result["data_jscq"].shape, (7,))
        self.assertEqual(result["data_3"].shape[1], 9)
        self.assertEqual(result["data_4"].shape[1], 7)
        self.assertEqual(result["data_33"].shape[1], 2)
        self.assertEqual(result["data_44"].shape[1], 2)
        self.assertTrue(np.isfinite(result["data_3"]).all())
        self.assertTrue(np.isfinite(result["data_4"]).all())

        dece_diam = 2.0 * np.sqrt(ctx.decechute.aera / np.pi)
        dece_total_length = ctx.decechute.rope_length + ctx.decechute.chute_width + dece_diam / 2.0
        self.assertGreaterEqual(result["data_jslz"][5], dece_total_length)

        ca_jst = ctx.decechute.resist * ctx.decechute.aera
        expected_t1 = ctx.decechute.ks * np.sqrt(ca_jst) / result["data_jslz"][4]
        self.assertAlmostEqual(result["time_jscq"], expected_t1, places=12)


if __name__ == "__main__":
    unittest.main()
