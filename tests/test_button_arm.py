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
        self.assertEqual(len(paths), 2)
        for path, target in zip(paths, [(250,0,100),(350,0,-40)]):
            xyz = forward_kinematics(**path[-1])
            for actual, expected in zip(xyz, target):
                self.assertAlmostEqual(actual, expected, delta=0.2)
            # The first path begins by aligning the initial tilted home pose.
            # Every point in the direct pressing transition stays horizontal.
            if path is not paths[0]:
                for pose in path:
                    pitch = (170 + pose['shoulder']-180 -120
                             -(pose['elbow']-180)-50
                             +(pose['wrist']-20)*90/124)
                    self.assertAlmostEqual(pitch, 0, delta=0.03)
        points = [forward_kinematics(**pose) for pose in paths[1]]
        self.assertLess(points[0][2], 100)
        self.assertGreater(points[0][0], 250)
        self.assertTrue(all(b[2] <= a[2] + 0.05 for a, b in zip(points, points[1:])))
        self.assertTrue(all(not (abs(px-350) < 0.5 and abs(pz-100) < 0.5)
                            for px, py, pz in points))
        with self.assertRaises(ValueError):
            button_arm.plan_button_paths(web, 900, 0, -40)
        self.assertEqual(web.get_current_angles(), saved)
