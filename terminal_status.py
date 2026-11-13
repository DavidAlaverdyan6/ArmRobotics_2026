"""Readable terminal output: one transient sensor row and durable event history."""

import builtins
import shutil
import sys
import threading
import time

_lock = threading.RLock()
_active = False
_last_plain = 0.0
_PLAIN_INTERVAL = 5.0


def _interactive():
    return bool(getattr(sys.stdout, "isatty", lambda: False)())


def _clear_status():
    global _active
    if _active and _interactive():
        builtins.print("\r\033[2K", end="", flush=True)
    _active = False


def event_print(*args, **kwargs):
    """Print an event on its own permanent line, without damaging status."""
    with _lock:
        _clear_status()
        if args and isinstance(args[0], str):
            # Old task messages used leading newlines to break a live status.
            args = (args[0].lstrip("\n"), *args[1:])
        builtins.print(*args, **kwargs)


def status_line(message):
    """Replace the current TTY status line; never grow scrollback on refresh."""
    global _active, _last_plain
    with _lock:
        message = " ".join(str(message).splitlines()).strip()
        if _interactive():
            width = max(20, shutil.get_terminal_size(fallback=(80, 24)).columns)
            # Reserve one terminal column to avoid automatic line wrapping.
            message = message[: width - 1]
            builtins.print("\r\033[2K" + message, end="", flush=True)
            _active = True
        elif time.monotonic() - _last_plain >= _PLAIN_INTERVAL:
            # Logs redirected to a file cannot replace lines; throttle instead.
            builtins.print(message, flush=True)
            _last_plain = time.monotonic()
