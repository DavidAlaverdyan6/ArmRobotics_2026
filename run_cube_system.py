#!/usr/bin/env python3
"""Launch arm_web and the separate green-cube camera interface."""

import sys

from run_button_system import main


if __name__ == '__main__':
    raise SystemExit(main(['--camera-script', 'arm_cube.py', *sys.argv[1:]]))
