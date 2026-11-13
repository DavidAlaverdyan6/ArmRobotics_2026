"""Per-robot route stored in route.json, excluded from Git updates."""

import json
from pathlib import Path

ROUTE_FILE = Path(__file__).with_name("route.json")


def load_local_route(default_route):
    """Use a local route; on first launch create one from the repo defaults."""
    if not ROUTE_FILE.exists():
        # Exclusive create avoids overwriting a route created by another process.
        try:
            with ROUTE_FILE.open("x", encoding="utf-8") as output:
                json.dump(list(default_route), output, ensure_ascii=False, indent=2)
                output.write("\n")
        except FileExistsError:
            pass

    with ROUTE_FILE.open("r", encoding="utf-8") as source:
        route = json.load(source)
    if not isinstance(route, list):
        raise ValueError(f"{ROUTE_FILE}: route must be a JSON list of commands")
    return route
