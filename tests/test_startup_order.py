"""Static regression tests for startup/reconnect ADC order (no hardware)."""
import ast
import unittest
from pathlib import Path

MAIN = Path(__file__).resolve().parents[1] / "main.py"


class StartupOrderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tree = ast.parse(MAIN.read_text(encoding="utf-8"))

    def actions_in(self, function_name):
        node = next(
            x for x in self.tree.body
            if isinstance(x, (ast.FunctionDef, ast.AsyncFunctionDef))
            and x.name == function_name
        )
        return [
            (x.lineno, ast.unparse(x.func))
            for x in ast.walk(node)
            if isinstance(x, ast.Call)
            and isinstance(x.func, (ast.Name, ast.Attribute))
        ]

    def test_adc_initialized_before_motor_and_arm_in_main(self):
        actions = self.actions_in("main")
        def pos(name):
            return min(line for line, fn in actions if fn == name)
        self.assertLess(pos("line_tracker.initialize"), pos("motors.initialize"))
        self.assertLess(pos("line_tracker.wait_for_callbacks"), pos("motors.initialize"))
        self.assertLess(pos("motors.initialize"), pos("activate_arm_home"))

    def test_adc_initialized_before_motor_and_arm_in_recovery(self):
        actions = self.actions_in("recover_stm32_connection")
        def pos(name):
            return min(line for line, fn in actions if fn == name)
        self.assertLess(pos("line_tracker.initialize"), pos("motors.initialize"))
        self.assertLess(pos("line_tracker.wait_for_callbacks"), pos("motors.initialize"))
        self.assertLess(pos("motors.initialize"), pos("activate_arm_home"))


if __name__ == "__main__":
    unittest.main()
