import sys
import types
import unittest
from unittest.mock import Mock, patch

import button_arm
from arm_ik import forward_kinematics


class ButtonPathTests(unittest.TestCase):
    def test_horizontal_path_and_preflight(self):
        with patch.dict(sys.modules, {'telemetrix': types.SimpleNamespace(telemetrix=Mock())}):
            import arm_web as web
            from roboarm import RoboArm
        web.arm = RoboArm(Mock())
        web.arm.write(**web.START_ANGLES)
        saved = web.get_current_angles()
        paths = button_arm.plan_button_paths(web, 350, 0, -40)
        self.assertEqual(web.get_current_angles(), saved)
        for path, target in zip(paths, [(250,0,100),(350,0,100),(350,0,-40)]):
            xyz = forward_kinematics(**path[-1])
            for actual, expected in zip(xyz, target):
                self.assertAlmostEqual(actual, expected, delta=0.2)
            # The first path begins by aligning the initial tilted home pose.
            # Every point in both pressing segments must remain horizontal.
            if path is not paths[0]:
                for pose in path:
                    pitch = (170 + pose['shoulder']-180 -120
                             -(pose['elbow']-180)-50
                             +(pose['wrist']-20)*90/124)
                    self.assertAlmostEqual(pitch, 0, delta=0.03)
        with self.assertRaises(ValueError):
            button_arm.plan_button_paths(web, 900, 0, -40)
        self.assertEqual(web.get_current_angles(), saved)
