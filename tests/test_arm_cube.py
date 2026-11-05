import sys
import types
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from arm_cube import (
    CUBE_CFG, CubeController, CubeDetector, TARGET_FORWARD_CLEARANCE_MM,
)


class CubeTests(unittest.TestCase):
    def test_running_task_blocks_buttons_and_duplicate_start_is_ignored(self):
        arm = Mock(base_url='http://localhost:8080')
        controller = CubeController(Mock(), arm)
        controller.mode = 'AUTO'
        controller.state = 'OPEN_GRIPPER'
        controller.thread = Mock()
        controller.thread.is_alive.return_value = True
        status = controller.telemetry()
        self.assertTrue(status['task_running'])
        self.assertFalse(status['auto_ready'])
        self.assertFalse(controller.start('AUTO'))
        arm._request.assert_not_called()

    def test_open_gripper_pause_counts_down_and_can_be_stopped(self):
        controller = CubeController(Mock(), Mock(base_url='http://localhost:8080'))
        countdown = []
        def wait(_):
            countdown.append(controller.status['stage_wait_seconds'])
            return False
        with patch.object(controller.stop_event, 'wait', side_effect=wait):
            controller.wait_for_gripper()
        self.assertEqual(countdown, [5, 4, 3, 2, 1])
        self.assertNotIn('stage_wait_seconds', controller.status)
        with patch.object(controller.stop_event, 'wait', return_value=True):
            with self.assertRaises(InterruptedError):
                controller.wait_for_gripper()
        self.assertNotIn('stage_wait_seconds', controller.status)

    def test_daniel_detector_finds_green_cube_on_cluttered_background(self):
        frame = np.full((240, 320, 3), (70, 70, 70), dtype=np.uint8)
        cv2.rectangle(frame, (10, 10), (65, 70), (0, 0, 220), -1)
        cv2.rectangle(frame, (135, 75), (205, 145), (10, 190, 20), -1)
        detector = CubeDetector(CUBE_CFG)
        objects = detector.candidates(frame)
        self.assertEqual(len(objects), 1)
        self.assertEqual(objects[0]['color'], 'G')
        self.assertEqual(objects[0]['center'], (170, 110))

    def test_camera_keeps_cube_centered_until_height_correction(self):
        controller = CubeController(Mock(), Mock(base_url='http://localhost:8080'))
        self.assertEqual(controller._control_center({
            'center': (320, 300), 'w': 100, 'h': 100,
        }), (320, 300))

    def test_image_growth_estimates_camera_clearance(self):
        samples = [(x, 30000 / (200 - x)) for x in (0, 8, 16, 24)]
        self.assertAlmostEqual(
            CubeController.camera_distance_from_scale(samples), 176,
            delta=0.1,
        )
        self.assertEqual(TARGET_FORWARD_CLEARANCE_MM, 70)

    def test_camera_approach_uses_short_steps_until_gripper_clearance(self):
        arm = Mock(base_url='http://localhost:8080')
        arm.status.return_value = {'tool_pitch': 0}
        controller = CubeController(Mock(), arm)
        moved = [0.0]
        def target():
            height = 30000 / (200 - moved[0])
            return {'center': (320, 240),
                    'w': height, 'h': height}
        def step(method, path, delta, timeout):
            self.assertEqual(path, '/api/cube-step')
            moved[0] += delta['dx']
            return {'ok': True}
        controller.fresh_target = Mock(side_effect=target)
        arm._request.side_effect = step
        controller.approach_by_camera()
        self.assertAlmostEqual(moved[0], 200 - TARGET_FORWARD_CLEARANCE_MM,
                               delta=3)
        self.assertGreater(arm._request.call_count, 10)

    def test_camera_height_offset_is_corrected_with_relative_steps(self):
        arm = Mock(base_url='http://localhost:8080')
        arm.status.return_value = {'tool_pitch': 0}
        controller = CubeController(Mock(), arm)
        controller.correct_camera_height()
        steps = [call.args[2]['dz'] for call in arm._request.call_args_list]
        self.assertEqual(steps, [8, 8, 8, 6])
        self.assertEqual(controller.status['height_correction_mm'], 30)

    def test_height_offset_rotates_with_tilted_gripper(self):
        arm = Mock(base_url='http://localhost:8080')
        arm.status.return_value = {'tool_pitch': -30, 'angles': {'base': 90}}
        controller = CubeController(Mock(), arm)
        controller.correct_camera_height()
        deltas = [call.args[2] for call in arm._request.call_args_list]
        self.assertAlmostEqual(sum(d['dx'] for d in deltas), 15, delta=0.01)
        self.assertAlmostEqual(sum(d['dz'] for d in deltas), 25.981, delta=0.01)
        self.assertTrue(all(d['dy'] == 0 for d in deltas))

    def test_cube_step_is_relative_and_does_not_command_gripper(self):
        with patch.dict(sys.modules, {
            'telemetrix': types.SimpleNamespace(telemetrix=Mock()),
        }):
            import arm_web as web
            from roboarm import RoboArm
        web.arm = RoboArm(Mock())
        web.arm.write(**web.START_ANGLES)
        with patch.object(web.time, 'sleep'), patch.object(
            web.arm, 'write', wraps=web.arm.write
        ) as write:
            result = web.execute_cube_step({'dx': 8, 'dy': 0, 'dz': 0})
        self.assertAlmostEqual(result['delta']['dx'], 8)
        self.assertGreater(write.call_count, 0)
        self.assertTrue(all('gripper' not in call.kwargs
                            for call in write.call_args_list))

    def test_first_probe_is_reachable_from_reported_joint_limit_pose(self):
        with patch.dict(sys.modules, {
            'telemetrix': types.SimpleNamespace(telemetrix=Mock()),
        }):
            import arm_web as web
            from roboarm import RoboArm
        web.arm = RoboArm(Mock())
        web.arm.write(base=92, shoulder=155, elbow=180, wrist=0, gripper=0)
        before = web.get_current_xyz()
        forward, _ = CubeController.tool_axes({
            'tool_pitch': web.get_current_tool_pitch(),
            'angles': web.get_current_angles(),
        })
        delta = {key: axis * 2 for key, axis in
                 zip(('dx', 'dy', 'dz'), forward)}
        with patch.object(web.time, 'sleep'):
            result = web.execute_cube_step(delta)
        self.assertGreater(result['xyz'][0], before[0])
        self.assertEqual(result['angles']['gripper'], 0)

    def test_cube_is_selected_automatically_from_camera(self):
        frame = np.full((240, 320, 3), (70, 70, 70), dtype=np.uint8)
        cv2.rectangle(frame, (135, 75), (205, 145), (10, 190, 20), -1)
        camera = Mock()
        camera.read.return_value = frame
        controller = CubeController(camera, Mock(base_url=None))
        _, target = controller.observe()
        self.assertIsNotNone(target)
        self.assertTrue(controller.button.selected)
        self.assertEqual(target['color'], 'G')

    def test_auto_task_opens_then_closes_gripper(self):
        arm = Mock(base_url='http://localhost:8080')
        arm.status.return_value = {
            'xyz': CUBE_CFG.REFERENCE_OBSERVATION_XYZ,
            'angles': {'base': CUBE_CFG.REFERENCE_OBSERVATION_ANGLES[0]},
            'tool_pitch': CUBE_CFG.REFERENCE_TOOL_PITCH_DEG,
        }
        arm._request.return_value = {'angles': {'gripper': 180}}
        controller = CubeController(Mock(), arm)
        controller.mode = 'AUTO'
        controller.observe = Mock()
        controller.fresh_target = Mock(return_value={'center': (320, 300), 'w': 100, 'h': 100})
        controller.calibrate = Mock()
        controller._align = Mock()
        controller.approach_by_camera = Mock()
        controller.correct_camera_height = Mock()
        with patch.object(controller.stop_event, 'wait', return_value=False):
            controller._run()
        controller.calibrate.assert_called_once()
        controller._align.assert_called_once()
        controller.approach_by_camera.assert_called_once()
        controller.correct_camera_height.assert_called_once()
        self.assertEqual(controller.state, 'GRASP_COMMANDED')
        self.assertEqual(
            [call.args[:3] for call in arm._request.call_args_list],
            [('POST', '/api/servo', {'gripper': 0}),
             ('POST', '/api/servo', {'gripper': 180})],
        )


if __name__ == '__main__':
    unittest.main()
