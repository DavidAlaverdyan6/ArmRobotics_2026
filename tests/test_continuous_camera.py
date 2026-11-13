"""Camera startup and capture failures, using a fake device and real threads."""
import queue
import sys
import threading
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

from color_marker_detector import ContinuousCamera


class FakeCamera:
    def __init__(self):
        self.frames = queue.Queue()
        self.capture_entered = threading.Event()
        self.closed = False

    def create_preview_configuration(self, **kwargs):
        return kwargs

    def configure(self, config):
        pass

    def start(self):
        pass

    def capture_array(self):
        self.capture_entered.set()
        value = self.frames.get(timeout=2)
        if isinstance(value, Exception):
            raise value
        return value

    def stop(self):
        self.frames.put(np.zeros((4, 4, 3), dtype=np.uint8))

    def close(self):
        self.closed = True


class ContinuousCameraTests(unittest.TestCase):
    def setUp(self):
        self.device = FakeCamera()
        self.stream = ContinuousCamera(port=0)
        self.stream.STARTUP_FRAME_TIMEOUT = 1.0
        self.preview = Mock(has_viewers=False)
        self.patches = [
            patch.dict(sys.modules, {
                'picamera2': types.SimpleNamespace(Picamera2=lambda: self.device),
            }),
            patch('color_marker_detector.ColorPreviewServer', return_value=self.preview),
        ]
        for item in self.patches:
            item.start()
            self.addCleanup(item.stop)
        self.addCleanup(self.stream.stop)

    def test_start_waits_for_frame_and_healthy_checks_do_not_reconnect(self):
        finished = threading.Event()
        errors = []

        def start():
            try:
                self.stream.start()
            except Exception as exc:
                errors.append(exc)
            finally:
                finished.set()

        starter = threading.Thread(target=start)
        starter.start()
        try:
            self.assertTrue(self.device.capture_entered.wait(1))
            self.assertFalse(finished.is_set())
            self.device.frames.put(np.zeros((4, 4, 3), dtype=np.uint8))
            self.assertTrue(finished.wait(1))
            self.assertEqual(errors, [])
            for _ in range(10):
                self.stream.raise_if_failed()
            self.assertEqual(self.stream.sequence, 1)
            self.assertFalse(self.device.closed)
            self.preview.start.assert_called_once()
        finally:
            self.stream.stop()
            starter.join(timeout=2)

    def test_timeout_uses_instance_budget_and_closes_device(self):
        self.stream.STARTUP_FRAME_TIMEOUT = 0.02
        with self.assertRaisesRegex(TimeoutError, '0.02 seconds'):
            self.stream.start()
        self.assertTrue(self.device.closed)
        self.assertFalse(self.stream._thread.is_alive())

    def test_start_propagates_capture_error_and_closes_device(self):
        failure = OSError('sensor disconnected')
        self.device.frames.put(failure)
        with self.assertRaisesRegex(RuntimeError, 'sensor disconnected') as result:
            self.stream.start()
        self.assertIs(result.exception.__cause__, failure)
        self.assertTrue(self.device.closed)

    def test_later_failure_is_reported_and_closed_stream_is_not_healthy(self):
        self.device.frames.put(np.zeros((4, 4, 3), dtype=np.uint8))
        self.stream.start()
        self.stream.raise_if_failed()
        self.device.frames.put(OSError('capture failed later'))
        with self.stream._condition:
            self.assertTrue(self.stream._condition.wait_for(
                lambda: self.stream._error is not None, timeout=1))
        with self.assertRaisesRegex(RuntimeError, 'capture failed later'):
            self.stream.raise_if_failed()
        closed = ContinuousCamera()
        closed.stop()
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            closed.raise_if_failed()


if __name__ == '__main__':
    unittest.main()
