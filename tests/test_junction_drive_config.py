"""Pure tests: calibration values persist and feed junction alignment."""
import tempfile
import unittest
from pathlib import Path

import junction_drive_config as config


class JunctionDriveConfigTests(unittest.TestCase):
    def test_round_trip_pwm_90_and_half_second(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "drive.json"
            config.save(0.5, 90, path)
            self.assertEqual(config.load(path), (0.5, 90))

    def test_missing_file_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                config.load(Path(folder) / "missing.json")

    def test_invalid_values_are_rejected(self):
        for seconds, pwm in ((0,90), (3,90), (0.5,0), (0.5,256),
                             ("nan",90), (0.5,90.5)):
            with self.subTest(seconds=seconds, pwm=pwm):
                with self.assertRaises(ValueError):
                    config.validate(seconds, pwm)


if __name__ == "__main__":
    unittest.main()
