"""Unexpected OneWire packets cannot kill the Telemetrix reporter thread."""
import unittest
from unittest.mock import Mock, patch

from telemetrix_guard import install_unexpected_onewire_guard


class FakeTelemetrix:
    def __init__(self):
        self.onewire_callback = None
        self.dispatch = {14: self._onewire_report}

    def _onewire_report(self, report):
        self.onewire_callback(report)


class OneWireGuardTests(unittest.TestCase):
    def setUp(self):
        install_unexpected_onewire_guard(FakeTelemetrix)

    def test_unexpected_packet_does_not_raise_or_kill_dispatch(self):
        board = FakeTelemetrix()
        with patch("builtins.print") as log:
            board.dispatch[14]([1, 2, 3])
            board.dispatch[14]([1, 2, 3])
        self.assertEqual(board._postintegral_unexpected_onewire, 2)
        self.assertTrue(log.called)

    def test_actual_onewire_callback_still_works(self):
        board = FakeTelemetrix()
        callback = Mock()
        board.onewire_callback = callback
        board.dispatch[14]([29, 42])
        callback.assert_called_once_with([29, 42])

    def test_reinstall_is_idempotent(self):
        initial = FakeTelemetrix._onewire_report
        install_unexpected_onewire_guard(FakeTelemetrix)
        self.assertIs(FakeTelemetrix._onewire_report, initial)


if __name__ == "__main__":
    unittest.main()
