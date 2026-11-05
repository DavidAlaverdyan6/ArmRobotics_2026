import io
import json
import subprocess
import unittest
from unittest.mock import Mock, patch

import run_button_system as launcher


class Response:
    status = 200

    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self):
        return json.dumps(self.payload).encode()


class LauncherTests(unittest.TestCase):
    def test_arm_is_ready_requires_connected_status(self):
        with patch.object(launcher, "urlopen", return_value=Response({"connected": True})):
            self.assertTrue(launcher.arm_is_ready("http://127.0.0.1:8080"))
        with patch.object(launcher, "urlopen", return_value=Response({"connected": False})):
            self.assertFalse(launcher.arm_is_ready("http://127.0.0.1:8080"))

    def test_arm_is_ready_handles_connection_failure(self):
        with patch.object(launcher, "urlopen", side_effect=OSError("refused")):
            self.assertFalse(launcher.arm_is_ready("http://127.0.0.1:8080"))

    def test_existing_controller_without_press_capability_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError, 'старый arm_web'):
            launcher.require_press_capability({'connected': True})
        with self.assertRaisesRegex(RuntimeError, 'старый arm_web'):
            launcher.require_press_capability({
                'connected': True,
                'capabilities': ['button-press-v1'],
            })
        launcher.require_press_capability({
            'connected': True,
            'capabilities': ['button-press-v2'],
        })

    def test_wait_for_arm_reports_early_exit(self):
        process = Mock()
        process.poll.return_value = 3
        with self.assertRaisesRegex(RuntimeError, "кодом 3"):
            launcher.wait_for_arm(process, launcher.DEFAULT_ARM_URL, 1)

    def test_stop_child_escalates_after_timeout(self):
        process = Mock()
        process.poll.return_value = None
        process.wait.side_effect = [
            subprocess.TimeoutExpired("arm", 8),
            subprocess.TimeoutExpired("arm", 3),
            0,
        ]
        with patch("sys.stdout", new=io.StringIO()):
            launcher.stop_child(process, "test")
        process.send_signal.assert_called_once()
        process.terminate.assert_called_once()
        process.kill.assert_called_once()


if __name__ == "__main__":
    unittest.main()
