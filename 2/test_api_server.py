import unittest

from api_server import run_simulation_request
from airdrop_py import (
    DEFAULT_JSS_P,
    DEFAULT_KTE_P,
    DEFAULT_QYS_P,
    DEFAULT_SIM_P,
    DEFAULT_VEL_P,
    DEFAULT_ZHS_P,
)


class ApiServerTests(unittest.TestCase):
    def test_request_uses_defaults_and_returns_json_ready_data(self) -> None:
        result = run_simulation_request({"include_steady_descent": False})

        self.assertTrue(result["success"])
        self.assertEqual(len(result["columns"]), 9)
        self.assertGreater(len(result["time_all"]), 0)
        self.assertEqual(len(result["time_all"]), len(result["data_all"]))
        self.assertEqual(len(result["data_all"][0]), 9)
        self.assertIn("time_mscq", result["stage_times"])

    def test_request_accepts_explicit_vectors(self) -> None:
        result = run_simulation_request(
            {
                "kte_p": list(DEFAULT_KTE_P),
                "vel_p": list(DEFAULT_VEL_P),
                "qys_p": list(DEFAULT_QYS_P),
                "jss_p": list(DEFAULT_JSS_P),
                "zhs_p": list(DEFAULT_ZHS_P),
                "sim_p": list(DEFAULT_SIM_P),
                "include_steady_descent": False,
            }
        )
        self.assertTrue(result["success"])

    def test_request_rejects_wrong_vector_length(self) -> None:
        with self.assertRaisesRegex(ValueError, "kte_p must contain 9 values"):
            run_simulation_request({"kte_p": [1, 2], "include_steady_descent": False})


if __name__ == "__main__":
    unittest.main()
