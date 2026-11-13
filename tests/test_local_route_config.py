"""Local route storage must survive repository default changes."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import local_route_config as route_config


class RouteStorageTests(unittest.TestCase):
    def test_local_route_created_once_and_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "route.json"
            with patch.object(route_config, "ROUTE_FILE", path):
                self.assertEqual(route_config.load_local_route(["R", "Q"]), ["R", "Q"])
                path.write_text(json.dumps(["L", "Q"]), encoding="utf-8")
                self.assertEqual(route_config.load_local_route(["R", "R", "Q"]), ["L", "Q"])

    def test_invalid_local_route_is_not_silently_replaced(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "route.json"
            path.write_text('{"route":["L"]}', encoding="utf-8")
            with patch.object(route_config, "ROUTE_FILE", path):
                with self.assertRaises(ValueError):
                    route_config.load_local_route(["R", "Q"])


if __name__ == "__main__":
    unittest.main()
