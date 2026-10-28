from telemetrix import telemetrix
from roboarm import RoboArm
from arm_web import PORT

JOINTS = ("base", "shoulder", "elbow", "wrist")
STEP = 2
board = None

try:
    input("Stop main_qar2.py and arm_web.py, then press Enter: ")

    board = telemetrix.Telemetrix(com_port=PORT)
    arm = RoboArm(board)

    while True:
        name = input(
            "\nJoint (base/shoulder/elbow/wrist/quit): "
        ).strip().lower()

        if name == "quit":
            break

        if name not in JOINTS:
            print("Unknown joint")
            continue

        current = int(input("Current estimated angle (0-180): "))

        if not 0 <= current <= 180:
            print("Invalid angle")
            continue

        joint = getattr(arm, name)

        while True:
            command = input(
                f"{name}={current}; + / - / back: "
            ).strip().lower()

            if command == "back":
                break

            if command == "+":
                target = min(180, current + STEP)
            elif command == "-":
                target = max(0, current - STEP)
            else:
                print("Use +, - or back")
                continue

            if input(f"Send angle {target}? (yes/no): ").lower() != "yes":
                continue

            joint.write(target)
            current = target

finally:
    if board is not None:
        board.shutdown()

    print("Test stopped")
