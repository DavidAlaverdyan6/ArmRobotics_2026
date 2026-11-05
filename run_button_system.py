#!/usr/bin/env python3
"""Start the arm controller and a camera UI as one supervised system."""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parent
DEFAULT_ARM_URL = "http://127.0.0.1:8080"


def arm_status(base_url: str, timeout: float = 0.4) -> dict | None:
    """Read arm_web status; return None when no valid response is available."""
    try:
        with urlopen(base_url.rstrip("/") + "/api/status", timeout=timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
            if response.status != 200 or not isinstance(payload, dict):
                return None
        return payload
    except (HTTPError, URLError, TimeoutError, OSError, ValueError, json.JSONDecodeError):
        return None


def arm_is_ready(base_url: str, timeout: float = 0.4) -> bool:
    status = arm_status(base_url, timeout)
    return status is not None and status.get("connected") is True


def require_press_capability(status: dict) -> None:
    if "button-press-v2" not in status.get("capabilities", []):
        raise RuntimeError(
            "На порту 8080 работает старый arm_web без возврата в базовое положение. "
            "Остановите старый arm_web через Ctrl+C и запустите систему снова."
        )


def wait_for_arm(process: subprocess.Popen, base_url: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(f"arm_web завершился с кодом {return_code}")
        if arm_is_ready(base_url):
            return
        time.sleep(0.2)
    raise RuntimeError(
        f"arm_web не стал доступен по адресу {base_url} за {timeout:g} с"
    )


def stop_child(process: subprocess.Popen | None, name: str) -> None:
    if process is None or process.poll() is not None:
        return
    print(f"Останавливаю {name}...", flush=True)
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=8)
        return
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-url", default=DEFAULT_ARM_URL)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--camera-script", choices=("arm_camera_button.py", "arm_cube.py"),
                        default="arm_camera_button.py", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)

    arm_process = None
    camera_process = None
    owns_arm = False
    try:
        existing_arm = arm_status(args.arm_url)
        if existing_arm is not None and existing_arm.get("connected") is True:
            require_press_capability(existing_arm)
            print(f"Использую уже запущенный arm_web: {args.arm_url}", flush=True)
        else:
            if args.arm_url.rstrip("/") != DEFAULT_ARM_URL:
                parser.error(
                    "автозапуск поддерживается только для локального "
                    f"{DEFAULT_ARM_URL}; запустите удалённый arm_web отдельно"
                )
            print("Запускаю контроллер руки...", flush=True)
            arm_process = subprocess.Popen(
                [sys.executable, "-u", str(ROOT / "arm_web.py")], cwd=ROOT
            )
            owns_arm = True
            wait_for_arm(arm_process, args.arm_url, args.startup_timeout)
            require_press_capability(arm_status(args.arm_url) or {})

        print("Контроллер руки готов. Запускаю камеру...", flush=True)
        camera_process = subprocess.Popen(
            [
                sys.executable,
                "-u",
                str(ROOT / args.camera_script),
                "--arm-url",
                args.arm_url,
                "--host",
                args.host,
                "--port",
                str(args.port),
            ],
            cwd=ROOT,
        )
        print(f"Откройте http://127.0.0.1:{args.port}", flush=True)
        return camera_process.wait()
    except KeyboardInterrupt:
        return 0
    except Exception as exc:
        print(f"Ошибка запуска: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        stop_child(camera_process, "камеру")
        if owns_arm:
            stop_child(arm_process, "контроллер руки")


if __name__ == "__main__":
    raise SystemExit(main())
