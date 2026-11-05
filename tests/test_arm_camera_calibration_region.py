import unittest
import threading
from unittest.mock import Mock

from arm_camera_button import ArmController, ButtonTaskController, CFG


class CalibrationRegionTests(unittest.TestCase):
    def controller(self, coordinates):
        controller = object.__new__(ButtonTaskController)
        controller.config = CFG
        controller.servo = Mock(coordinates=coordinates)
        return controller

    def test_base_shoulder_calibration_allows_expected_pitch_change(self):
        controller = self.controller('base_shoulder')
        reference = {
            'angles': {'base': 90, 'shoulder': 160},
            'xyz': [140, 0, 20],
            'tool_pitch': -35,
        }
        pose = {
            'angles': {'base': 98, 'shoulder': 178},
            'xyz': [125, 20, 55],
            'tool_pitch': -17,
        }
        self.assertTrue(
            controller._pose_within_calibration_region(pose, reference)
        )

    def test_joint_calibration_still_has_bounded_range(self):
        controller = self.controller('base_shoulder')
        reference = {
            'angles': {'base': 90, 'shoulder': 160},
            'xyz': [140, 0, 20],
            'tool_pitch': -35,
        }
        pose = {
            'angles': {'base': 90, 'shoulder': 186},
            'xyz': [120, 0, 70],
            'tool_pitch': -9,
        }
        self.assertFalse(
            controller._pose_within_calibration_region(pose, reference)
        )

    def test_local_alignment_envelope_is_bounded_at_60_mm(self):
        arm = ArmController(None)
        arm.local_origin = (0, 0, 0)
        arm._check_workspace((0, 0, 59.9))
        with self.assertRaisesRegex(RuntimeError, '60 mm'):
            arm._check_workspace((0, 0, 60.1))

    def test_alignment_correction_is_faster_but_bounded(self):
        controller = self.controller('base_shoulder')
        self.assertEqual(controller._bounded_joint_correction(0.2), 0)
        self.assertEqual(controller._bounded_joint_correction(1.8), 2)
        self.assertEqual(controller._bounded_joint_correction(-20), -3)

    def test_reference_image_reconstructs_tested_contact_point(self):
        controller = self.controller('base_shoulder')
        cfg = controller.config
        pose = {
            'xyz': cfg.REFERENCE_OBSERVATION_XYZ,
            'angles': {
                'base': cfg.REFERENCE_OBSERVATION_ANGLES[0],
            },
            'tool_pitch': cfg.REFERENCE_TOOL_PITCH_DEG,
        }
        target = {
            'w': cfg.REFERENCE_BUTTON_DIAMETER_PX,
            'h': cfg.REFERENCE_BUTTON_DIAMETER_PX,
        }
        xyz, direction, distance = controller._adaptive_contact_target(
            target, pose
        )
        for actual, expected in zip(xyz, cfg.REFERENCE_CONTACT_XYZ):
            self.assertAlmostEqual(actual, expected, places=5)
        deeper, _, _ = controller._adaptive_contact_target(
            target, pose, cfg.PRESS_RETRY_STEP_MM
        )
        self.assertAlmostEqual(
            ((sum((deeper[i] - xyz[i])**2 for i in range(3))) ** 0.5),
            cfg.PRESS_RETRY_STEP_MM,
            places=5,
        )

    def test_press_once_without_visible_indicator(self):
        controller = self.controller('base_shoulder')
        cfg = controller.config
        controller.stop_event = threading.Event()
        controller.status = {}
        controller.state = 'ALIGNED'
        controller.arm = Mock()
        controller.arm.status.return_value = {
            'xyz': cfg.REFERENCE_OBSERVATION_XYZ,
            'angles': {
                'base': cfg.REFERENCE_OBSERVATION_ANGLES[0],
            },
            'tool_pitch': cfg.REFERENCE_TOOL_PITCH_DEG,
        }
        controller.fresh_target = Mock(return_value={
            'w': cfg.REFERENCE_BUTTON_DIAMETER_PX,
            'h': cfg.REFERENCE_BUTTON_DIAMETER_PX,
        })
        controller.arm.press_button.return_value = {'returned_angles': {}}
        controller._confirm_indicator = Mock()

        controller._adaptive_press()
        controller.arm.press_button.assert_called_once()
        controller._confirm_indicator.assert_not_called()
        self.assertEqual(controller.status['press_attempt'], 1)

    def test_full_is_available_before_indicator_selection(self):
        controller = ButtonTaskController(
            Mock(), Mock(base_url='http://localhost:8080'),
            Mock(selected=True), Mock(roi=None), Mock(calibrated=True),
        )
        controller.status['indicator'] = 'UNKNOWN'
        self.assertTrue(controller.telemetry()['full_ready'])
        self.assertFalse(controller.telemetry()['verify_ready'])
        controller.state = 'PRESSED'
        controller.indicator.roi = (1, 2, 10, 10)
        self.assertTrue(controller.telemetry()['verify_ready'])

    def test_lamp_verification_runs_after_press_without_arm_motion(self):
        controller = ButtonTaskController(
            Mock(), Mock(base_url='http://localhost:8080'),
            Mock(selected=True), Mock(roi=(1, 2, 10, 10)),
            Mock(calibrated=True),
        )
        controller.state = 'PRESSED'
        controller.mode = 'VERIFY'
        controller._confirm_indicator = Mock(return_value='GREEN')
        controller._run()
        self.assertEqual(controller.state, 'SUCCESS')
        controller.arm.press_button.assert_not_called()


if __name__ == '__main__':
    unittest.main()
