"""Offline tests for transient status and durable terminal events."""
import io
import unittest
from unittest.mock import patch
import terminal_status as terminal


class FakeTTY(io.StringIO):
    def isatty(self):
        return True


class TerminalTests(unittest.TestCase):
    def tearDown(self):
        terminal._active = False
        terminal._last_plain = 0.0

    def test_status_refreshes_same_line_without_newline(self):
        output = FakeTTY()
        with patch.object(terminal.sys, "stdout", output):
            terminal.status_line("LINE pos=0.1")
            terminal.status_line("LINE pos=0.2")
        self.assertNotIn("\n", output.getvalue())
        self.assertIn("\r\033[2KLINE pos=0.2", output.getvalue())

    def test_event_clears_status_and_keeps_event_in_history(self):
        output = FakeTTY()
        with patch.object(terminal.sys, "stdout", output):
            terminal.status_line("LINE pos=0.1")
            terminal.event_print("\nROUTE: R90")
        self.assertTrue(output.getvalue().endswith("ROUTE: R90\n"))

    def test_status_is_shortened_to_prevent_wrap(self):
        output = FakeTTY()
        with patch.object(terminal.sys, "stdout", output), patch.object(
            terminal.shutil, "get_terminal_size", return_value=type("S", (), {"columns": 30})()
        ):
            terminal.status_line("X" * 100)
        self.assertIn("X" * 29, output.getvalue())
        self.assertNotIn("X" * 30, output.getvalue())


if __name__ == "__main__":
    unittest.main()
