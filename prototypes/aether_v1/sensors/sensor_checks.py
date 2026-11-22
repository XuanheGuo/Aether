"""Native sensor, seeded-noise, and independent-file-replay regression tests."""

import tempfile
import unittest
from pathlib import Path

import numpy as np
from sensors.experiment import CASES, record, run_case
from sensors.replay import read_recording, replay


class SensorChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cases = {case: run_case(case) for case in CASES}

    def test_native_matches_independent_reference(self):
        for case, recording in self.cases.items():
            with self.subTest(case=case):
                np.testing.assert_allclose(recording.stamps, np.linspace(0, 2, 1001),
                                           rtol=0, atol=1e-10)
                self.assertTrue(np.all(np.isfinite(recording.kinematics)))
                np.testing.assert_allclose(recording.raw, recording.truth, rtol=0, atol=1e-9)
                np.testing.assert_array_equal(recording.raw, recording.measured)

    def test_rest_gravity_and_free_fall(self):
        rest = self.cases["rest"].raw
        fall = self.cases["fall"].raw
        np.testing.assert_allclose(np.linalg.norm(rest[:, :3], axis=1), 9.81, rtol=0, atol=1e-12)
        self.assertTrue(np.all(abs(rest[0, :3]) > 0.2))
        np.testing.assert_allclose(rest[:, 3:], 0, rtol=0, atol=1e-12)
        np.testing.assert_allclose(fall[:, :6], 0, rtol=0, atol=1e-12)
        self.assertGreater(np.linalg.norm(fall[-1, 6:] - fall[0, 6:]), 19.6)

    def test_rotation_excites_all_gyroscope_axes_and_lever_arm(self):
        raw = self.cases["tumble"].raw
        self.assertTrue(np.all(np.max(np.abs(raw[:, 3:6]), axis=0) > 0.2))
        self.assertTrue(np.all(np.ptp(raw[:, 3:6], axis=0) > 0.02))
        self.assertGreater(np.max(np.linalg.norm(raw[:, :3], axis=1)), 0.025)

    def test_seeded_noise_preserves_native_stream(self):
        noise_std = (0.08, 0.004, 0.015)
        first = run_case("tumble", noise_std=noise_std, seed=31)
        again = run_case("tumble", noise_std=noise_std, seed=31)
        other = run_case("tumble", noise_std=noise_std, seed=32)
        np.testing.assert_array_equal(first.raw, self.cases["tumble"].raw)
        np.testing.assert_array_equal(first.raw, other.raw)
        np.testing.assert_array_equal(first.measured, again.measured)
        self.assertFalse(np.array_equal(first.measured, other.measured))
        np.testing.assert_allclose(np.std(first.measured - first.raw, axis=0),
                                   np.repeat(noise_std, 3), rtol=0.12, atol=0)

    def test_file_replay_and_timestamp_mismatch(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            recording = folder / "fall"
            original = self.cases["fall"]
            record(recording, original)
            _, actual_stamps, (actual_truth, actual_raw, measured) = read_recording(recording)
            np.testing.assert_array_equal(actual_stamps, original.stamps)
            np.testing.assert_array_equal(actual_truth, original.truth)
            np.testing.assert_array_equal(actual_raw, original.raw)
            np.testing.assert_array_equal(measured, original.raw)
            result = replay(recording, folder / "errors", start=0.02, end=0.1, plots=False)
            self.assertEqual(result["samples"], 41)
            self.assertLess(result["channels"]["accel_z"]["measurement_minus_truth"]["rmse"], 1e-9)
            csv = recording / "raw.csv"
            lines = csv.read_text().splitlines()
            _, *values = lines[2].split(",")
            lines[2] = ",".join(["0.0021", *values])
            csv.write_text("\n".join(lines) + "\n")
            with self.assertRaisesRegex(ValueError, "Timestamp mismatch"):
                read_recording(recording)


if __name__ == "__main__":
    unittest.main()
