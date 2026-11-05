import sys
import types
import unittest
from unittest.mock import Mock, patch

import button_arm
from arm_ik import forward_kinematics


class ButtonPathTests(unittest.TestCase):
    def test_arm_web_and_camera_use_the_same_start_pose(self):
        with patch.dict(sys.modules, {
            'telemetrix': types.SimpleNamespace(telemetrix=Mock())
        }):
            import arm_web
            from arm_camera_button import CFG
        expected = (90, 170, 180, 0, 180)
        self.assertEqual(
            tuple(arm_web.START_ANGLES[name] for name in (
                'base', 'shoulder', 'elbow', 'wrist', 'gripper'
            )),
            expected,
        )
        self.assertEqual(CFG.OBSERVATION_ANGLES, expected)

    def test_shared_arm_attaches_servos_only_once(self):
        from roboarm import get_shared_arm

        board = Mock()
        first, newly_attached = get_shared_arm(board)
        second, attached_again = get_shared_arm(board)
        self.assertTrue(newly_attached)
        self.assertFalse(attached_again)
        self.assertIs(first, second)
        self.assertEqual(board.set_pin_mode_servo.call_count, 5)

    def test_horizontal_path_and_preflight(self):
        with patch.dict(sys.modules, {'telemetrix': types.SimpleNamespace(telemetrix=Mock())}):
            import arm_web as web
            from roboarm import RoboArm
        web.arm = RoboArm(Mock())
        web.arm.write(**web.START_ANGLES)
        saved = web.get_current_angles()
        paths = button_arm.plan_button_paths(web, 350, 0, -50)
        self.assertEqual(web.get_current_angles(), saved)
        self.assertEqual(len(paths), 2)
        for path, target in zip(paths, [(250,0,100),(350,0,-50)]):
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
            button_arm.plan_button_paths(web, 900, 0, -50)
        self.assertEqual(web.get_current_angles(), saved)

    def test_tested_minus_50_contact_is_reachable_from_new_start(self):
        with patch.dict(sys.modules, {
            'telemetrix': types.SimpleNamespace(telemetrix=Mock())
        }):
            import arm_web as web
            from roboarm import RoboArm
        web.arm = RoboArm(Mock())
        web.arm.write(**web.START_ANGLES)
        paths = button_arm.plan_button_paths(web, 350, 0, -50)
        xyz = forward_kinematics(**paths[-1][-1])
        for actual, expected in zip(xyz, (350, 0, -50)):
            self.assertAlmostEqual(actual, expected, delta=0.2)

    def test_press_cycle_returns_to_base_pose_after_observation(self):
        with patch.dict(sys.modules, {
            'telemetrix': types.SimpleNamespace(telemetrix=Mock())
        }):
            import arm_web as web
            from roboarm import RoboArm
        web.arm = RoboArm(Mock())
        web.arm.write(**web.START_ANGLES)
        web.arm.write(base=95, shoulder=165, elbow=178, wrist=2)
        with patch.dict(sys.modules, {'arm_web': web}), \
             patch.object(web.time, 'sleep'):
                result = web.execute_button_cycle(
                    {'x': 350, 'y': 0, 'z': -50},
                    hold_seconds=0.2,
                )
        for joint in web.IK_JOINTS:
            self.assertAlmostEqual(
                result['returned_angles'][joint],
                web.START_ANGLES[joint],
            )
