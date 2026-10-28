
from telemetrix.telemetrix import Telemetrix
from roboarm import RoboArm
from arm_web import PORT

JOINTS = ("base", "shoulder", "elbow", "wrist")
STEP = 2

board = None
arm = None


def read_angle(prompt):
    try:
        angle = int(input(prompt))
    except ValueError:
        print("Enter an integer.")
        return None

    if not 0 <= angle <= 180:
        print("Angle must be between 0 and 180.")
        return None

    return angle


try:
    print("Stop main_qar2.py and arm_web.py before starting.")
    input("Press Enter when the robot controller is free...")

    board = Telemetrix(com_port=PORT)
    arm = RoboArm(board)

    print("\nServo calibration test")
    print("No servo moves automatically.")
    print("Commands: base, shoulder, elbow, wrist, quit")

    while True:
        name = input("\nJoint or quit: ").strip().lower()

        if name == "quit":
            break

        if name not in JOINTS:
            print("Unknown joint.")
            continue

        joint = getattr(arm, name)
        current = read_angle("Current commanded angle (0-180): ")

        if current is None:
            continue

        print(f"Selected: {name}. No movement yet.")
        print("Commands: +, -, set, back")

        while True:
            command = input(
                f"{name}, commanded angle {current}: "
            ).strip().lower()

            if command == "back":
                break

            if command == "+":
                target = min(180, current + STEP)
            elif command == "-":
                target = max(0, current - STEP)
            elif command == "set":
                target = read_angle("Target angle (0-180): ")
                if target is None:
                    continue
            else:
                print("Use +, -, set or back.")
                continue

            if target == current:
                print("Angle unchanged.")
                continue

            confirm = input(
                f"Send {name} command {target}? Type YES to confirm: "
            ).strip()

            if confirm != "YES":
                print("Cancelled.")
                continue

            try:
                joint.write(target)
                current = target
                print(f"{name} command sent: {current}")
            except Exception as error:
                print("Command error:", error)
                break

except KeyboardInterrupt:
    print("\nInterrupted by user.")

except Exception as error:
    print("Test error:", error)

finally:
    if board is not None:
        try:
            board.shutdown()
        except Exception as error:
            print("Controller shutdown error:", error)

    print("Test stopped.")
