#!/usr/bin/env python3
"""ArmRobotics: differential2 line following + junction commands + button task."""

import time

import smbus
from telemetrix import telemetrix

import line_tracker
import motors
from sensors.vl53l0x_sensor import VL53L0XSensor


# Previously tested differential2 line-following speeds.
SERIAL_PORT = "/dev/serial0"
BASE_SPEED = 100
MAX_SPEED = 170
MIN_SPEED = -80
MANEUVER_SPEED = 100
MANEUVER_MAX_SPEED = 170
KP = 30.0
TARGET_POSITION = 0.08
LOOP_DELAY = 0.04

# Junction recognition is separate from line tracking.
BLACK_ON = 0.85
BLACK_OFF = 0.65
JUNCTION_CENTER_MIN = 0.55
JUNCTION_CONFIRM = 2
JUNCTION_CLEAR = 4
JUNCTION_COOLDOWN = 0.55

# After any gyro turn, differential2 centers the robot again.
REACQUIRE_TIMEOUT = 2.0
REACQUIRE_TOLERANCE = 0.65
REACQUIRE_CONFIRM = 4

I2C_BUS = 1
MPU_ADDR = 0x68
GYRO_Z_REG = 0x47
GYRO_SCALE = 131.0
GYRO_TIMEOUT = 5.0
TURN_ANGLE = 80.0
TURN_PWM_FAST = 255
TURN_PWM_SLOW = 90

BUTTON_DISTANCE_CM = 10.0
# Button tip target (mm, arm_web coordinate system). Arm moves in two stages.
BUTTON_X_MM = 250.0
BUTTON_Y_MM = 0.0
BUTTON_Z_MM = -40.0

# Parking: 9 s red blink, then 2 s solid green; total 11 s before R.
PARK_SECONDS = 11.0
PARK_RED_SECONDS = 9.0
PARK_BLINK_HALF_PERIOD = 0.5
BUTTON_MAX_TIME = 12.0
BUTTON_CONFIRM = 3
BUTTON_CLEAR_TIMEOUT = 3.0

RETURN_SPEED = 150
RETURN_SLOW_SPEED = 95
RETURN_GYRO_KP = 2.0
RETURN_MAX_DELTA = 85
RETURN_MAX_TIME = 15.0
RETURN_LINE_LOST = 0.4
RETURN_MAX_ANGLE = 30.0
RETURN_RECOVERY_ANGLE = 12.0
RETURN_RECOVERY_SAFE = 6.0
RETURN_RECOVERY_TIMEOUT = 1.5

RETURN_FORWARD_TIME = 0.3
BUTTON_FINAL_RIGHT_ANGLE = 90.0

# Automatic route: one entry for each detected junction, in order.
# L=left, R=right, S=straight, B=press button, P=parking, Q=stop.
# Example: ROUTE = ["L", "S", "B", "R", "P", "Q"]
# The commented entries below are placeholders: edit them before launching.
ROUTE = [
    # "L",
    # "S",
    # "B",
    # "R",
    # "P",
    # "Q",
]
ALLOWED_ROUTE_COMMANDS = {"L", "R", "S", "B", "P", "Q"}


def clamp(value, low, high):
    return max(low, min(high, value))


def stop():
    motors.motor_stop()


def differential_drive(state, speed=BASE_SPEED, max_speed=MAX_SPEED):
    """The differential2 P-controller, optionally with a maneuver speed cap."""
    if state is None or state["position"] is None:
        stop()
        return False

    position = state["position"]
    error = position - TARGET_POSITION
    correction = KP * error

    left_speed = clamp(speed + correction, MIN_SPEED, max_speed)
    right_speed = clamp(speed - correction, MIN_SPEED, max_speed)

    motors.set_speeds(round(left_speed), round(right_speed))
    return True


def differential_reverse(state, speed, heading_correction=0.0):
    """Same calibrated line position and P correction, adapted for reverse."""
    if state is None or state["position"] is None:
        stop()
        return False

    position = state["position"]
    error = position - TARGET_POSITION
    line_correction = KP * error
    correction = clamp(
        line_correction + heading_correction,
        -RETURN_MAX_DELTA,
        RETURN_MAX_DELTA,
    )

    # Keep both wheels in reverse; a large correction only changes their ratio.
    left_speed = clamp(-speed + correction, -255, -15)
    right_speed = clamp(-speed - correction, -255, -15)
    motors.set_speeds(round(left_speed), round(right_speed))
    return True


class JunctionDetector:
    """Only branch sensing. It never calculates line position or steers motors."""

    def __init__(self):
        self.left_black = False
        self.right_black = False

    @staticmethod
    def hysteresis(previous, value):
        return value > BLACK_OFF if previous else value >= BLACK_ON

    def read(self, state):
        values = state["normalized"]
        if len(values) != 8:
            raise ValueError("Expected eight line sensor values")

        self.left_black = self.hysteresis(self.left_black, values[0])
        self.right_black = self.hysteresis(self.right_black, values[7])

        # An outer sensor alone means the robot is off-center, NOT a junction.
        center_present = any(
            values[index] >= JUNCTION_CENTER_MIN
            for index in (2, 3, 4, 5)
        )
        candidate = (
            state["position"] is not None
            and center_present
            and (self.left_black or self.right_black)
        )

        return self.left_black, self.right_black, candidate


def gyro_raw(bus):
    hi = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG)
    lo = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG + 1)
    value = (hi << 8) | lo
    return value - 65536 if value >= 32768 else value


def gyro_rate(bus, offset):
    return (gyro_raw(bus) - offset) / GYRO_SCALE


def gyro_setup(bus):
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)
    bus.write_byte_data(MPU_ADDR, 0x1B, 0)
    time.sleep(0.2)


def gyro_calibrate(bus):
    stop()
    print("\nGyroscope calibration: keep the robot still.")
    values = []
    for _ in range(300):
        values.append(gyro_raw(bus))
        time.sleep(0.005)
    return sum(values) / len(values)


class Heading:
    def __init__(self, bus, offset, left_positive):
        self.bus = bus
        self.offset = offset
        self.sign = left_positive
        self.angle = 0.0
        self.last = time.monotonic()

    def read(self):
        now = time.monotonic()
        rate = gyro_rate(self.bus, self.offset)
        self.angle += self.sign * rate * max(0.0, now - self.last)
        self.last = now
        return self.angle


def pivot(bus, offset, direction, angle_target=TURN_ANGLE):
    """Gyro-controlled pivot. Return success and LEFT-turn gyro polarity."""
    if direction not in ("L", "R"):
        raise ValueError("Turn direction must be L or R")

    left = direction == "L"
    motors.set_speeds(
        -TURN_PWM_SLOW if left else TURN_PWM_FAST,
        TURN_PWM_FAST if left else -TURN_PWM_SLOW,
    )

    start = previous = time.monotonic()
    angle = signed = 0.0

    try:
        while angle < angle_target:
            now = time.monotonic()
            rate = gyro_rate(bus, offset)
            dt = max(0.0, now - previous)
            previous = now

            angle += abs(rate) * dt
            signed += rate * dt

            if now - start >= GYRO_TIMEOUT:
                print("\nTurn timeout.")
                return False, None

            time.sleep(0.005)

        sign = (
            (1 if signed > 0 else -1)
            if left and abs(signed) > 20
            else None
        )
        return True, sign

    finally:
        stop()
        time.sleep(0.2)


def reacquire_line(calibration):
    """Use differential2 driving after EVERY turn, including edge-only line."""
    deadline = time.monotonic() + REACQUIRE_TIMEOUT
    stable = 0

    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            position = state["position"]

            if position is None:
                stop()
                stable = 0
            else:
                differential_drive(
                    state, speed=MANEUVER_SPEED, max_speed=MANEUVER_MAX_SPEED
                )
                if abs(position - TARGET_POSITION) <= REACQUIRE_TOLERANCE:
                    stable += 1
                    if stable >= REACQUIRE_CONFIRM:
                        return True
                else:
                    stable = 0

            time.sleep(LOOP_DELAY)

        print("\nDifferential2: could not re-center after turn.")
        return False

    finally:
        stop()


def follow_for(calibration, seconds):
    """Timed forward movement: unchanged differential2 line-following."""
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            state = line_tracker.update(calibration)
            if state["position"] is None:
                stop()
                print("\nLine missing during timed forward movement.")
                return False
            differential_drive(
                state, speed=MANEUVER_SPEED, max_speed=MANEUVER_MAX_SPEED
            )
            time.sleep(LOOP_DELAY)
        return True
    finally:
        stop()
        time.sleep(0.15)


def button_approach(calibration, detector, sensor):
    """Follow line to 10 cm from the button and pause instead of pressing."""
    start = time.monotonic()
    cleared_count = 0
    left_initial_junction = False
    close_count = 0
    missing_since = None

    try:
        while time.monotonic() - start < BUTTON_MAX_TIME:
            state = line_tracker.update(calibration)
            left, right, _ = detector.read(state)

            if not left and not right:
                cleared_count += 1
            else:
                cleared_count = 0

            if cleared_count >= JUNCTION_CLEAR:
                left_initial_junction = True

            if (
                not left_initial_junction
                and time.monotonic() - start > BUTTON_CLEAR_TIMEOUT
            ):
                print("\nB: initial junction did not clear.")
                return False

            distance_cm = sensor.read_distance_mm() / 10.0
            if (
                left_initial_junction
                and 0 < distance_cm <= BUTTON_DISTANCE_CM
            ):
                close_count += 1
            else:
                close_count = 0

            if close_count >= BUTTON_CONFIRM:
                stop()
                print(f"\nB: button at {distance_cm:.1f} cm. Stopped for arm task.")
                return True

            if state["position"] is None:
                stop()
                if missing_since is None:
                    missing_since = time.monotonic()
                elif time.monotonic() - missing_since >= 0.5:
                    print("\nB: line lost on approach.")
                    return False
            else:
                missing_since = None
                differential_drive(
                    state, speed=MANEUVER_SPEED,
                    max_speed=MANEUVER_MAX_SPEED
                )

            time.sleep(LOOP_DELAY)

        print("\nB: button approach timeout.")
        return False

    finally:
        stop()


def button_return(calibration, detector, bus, offset, left_sign):
    """Reverse on the differential2 sensor position, with gyro protection."""
    heading = Heading(bus, offset, left_sign)
    start = time.monotonic()
    last_line = start
    cleared_count = 0
    candidate_since = None
    recovery_since = None

    try:
        while time.monotonic() - start < RETURN_MAX_TIME:
            state = line_tracker.update(calibration)
            left, right, _ = detector.read(state)
            now = time.monotonic()
            angle = heading.read()

            if not left and not right:
                cleared_count += 1

            # Preserve the original B return-junction rule.
            if cleared_count >= JUNCTION_CLEAR and left and right:
                print("\nB: return junction found (both outer sensors).")
                return True

            if abs(angle) >= RETURN_MAX_ANGLE:
                print("\nB: reverse gyro angle limit reached.")
                return False

            if left != right:
                if candidate_since is None:
                    candidate_since = now
            elif not left and not right:
                candidate_since = None

            if abs(angle) >= RETURN_RECOVERY_ANGLE:
                if recovery_since is None:
                    recovery_since = now
            elif abs(angle) <= RETURN_RECOVERY_SAFE:
                recovery_since = None

            if (
                recovery_since is not None
                and now - recovery_since >= RETURN_RECOVERY_TIMEOUT
            ):
                print("\nB: reverse heading recovery timeout.")
                return False

            if state["position"] is not None:
                last_line = now
                speed = (
                    RETURN_SLOW_SPEED
                    if candidate_since is not None or recovery_since is not None
                    else RETURN_SPEED
                )
                differential_reverse(
                    state, speed, RETURN_GYRO_KP * angle
                )
            else:
                stop()
                if now - last_line >= RETURN_LINE_LOST:
                    print("\nB: line lost during reverse.")
                    return False

            print(
                f'\rB BACK pos={state["position"]} angle={angle:+.1f} '
                f'S1={int(left)} S8={int(right)}',
                end="", flush=True,
            )
            time.sleep(LOOP_DELAY)

        print("\nB: reverse timeout.")
        return False

    finally:
        stop()


def button_task(calibration, detector, sensor, bus, offset, board):
    print("\nB: turn LEFT.")
    ok, left_sign = pivot(bus, offset, "L")
    if not ok or left_sign is None:
        print("\nB: initial turn/gyro check failed.")
        return False

    if not reacquire_line(calibration):
        return False

    if not button_approach(calibration, detector, sensor):
        return False

    # The drive motors stay stopped until the arm maneuver finishes.
    if not button_arm_action(board):
        return False

    if not button_return(calibration, detector, bus, offset, left_sign):
        return False

    print("\nB: move forward for 0.3 seconds.")
    if not follow_for(calibration, RETURN_FORWARD_TIME):
        return False

    print("\nB: turn RIGHT by 90 degrees.")
    ok, _ = pivot(
        bus, offset, "R", angle_target=BUTTON_FINAL_RIGHT_ANGLE
    )
    return ok and reacquire_line(calibration)


def button_arm_action(board):
    """Move arm to the tested button XYZ, then return home before reversing."""
    from button_arm import move_to_xyz_and_home

    stop()
    print(
        f"\nB: moving arm automatically to X={BUTTON_X_MM:g}, "
        f"Y={BUTTON_Y_MM:g}, Z={BUTTON_Z_MM:g} mm."
    )
    try:
        ok = move_to_xyz_and_home(
            board, BUTTON_X_MM, BUTTON_Y_MM, BUTTON_Z_MM
        )
        if ok:
            print("B: arm returned to home. Reversing to junction.")
        return ok
    except Exception as exc:
        print(f"\nB: arm movement failed: {exc}")
        print("B: drive stopped. Check arm position before continuing.")
        return False
    finally:
        stop()


def parking_task():
    """Stop for 11 s: blink RED for 9 s, GREEN for 2 s, then turn R."""
    stop()
    rgb = None
    try:
        # The existing led.RGBLED handles the robot's active-low RGB pins.
        import led
        import robot_pin_config as pins

        # Physical red/blue LED channels are swapped on this robot:
        # previous RED command lit BLUE. Swap only here, not global pins.
        rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
        start = time.monotonic()
        red_end = start + PARK_RED_SECONDS
        park_end = start + PARK_SECONDS

        print("\nP: robot stopped. Red blinking for 9 seconds.")
        while time.monotonic() < red_end:
            rgb.red()
            time.sleep(
                max(0.0, min(PARK_BLINK_HALF_PERIOD,
                             red_end - time.monotonic()))
            )
            rgb.off()
            time.sleep(
                max(0.0, min(PARK_BLINK_HALF_PERIOD,
                             red_end - time.monotonic()))
            )

        print("\nP: green ON for remaining 2 seconds.")
        rgb.green()
        time.sleep(max(0.0, park_end - time.monotonic()))
        print("\nP: parking complete; turning RIGHT.")
        return True

    except Exception as exc:
        print(f"\nP: RGB parking error: {exc}")
        print("P: not turning until the light problem is resolved.")
        return False

    finally:
        if rgb is not None:
            try:
                rgb.off()
                rgb.close()
            except Exception:
                pass
        stop()


def load_route():
    """Read the user-edited ROUTE list before starting the robot."""
    if not ROUTE:
        raise ValueError(
            "ROUTE is empty. Add your junction commands to the ROUTE list "
            "in main_ggs_interactive.py before starting."
        )

    route = []
    for index, command in enumerate(ROUTE, 1):
        if not isinstance(command, str):
            raise ValueError(f"ROUTE entry {index} is not a string.")
        cmd = command.strip().upper()
        if cmd not in ALLOWED_ROUTE_COMMANDS:
            raise ValueError(
                f"Invalid ROUTE entry #{index}: {command!r}. "
                "Allowed: L, R, S, B, P, Q."
            )
        if cmd == "Q" and index != len(ROUTE):
            raise ValueError("Q must be the last ROUTE entry.")
        route.append(cmd)
    return route


def main():
    board = bus = sensor = None
    motors_on = tracker_on = False

    try:
        route = load_route()
        print(f"Automatic ROUTE loaded: {route}")
        # SAME connection and initialization sequence as differential2.py.
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)

        # Match the proven timing in show_line_position.py before ADC setup.
        time.sleep(2.0)

        motors.initialize(board)
        motors_on = True
        line_tracker.initialize(board)
        tracker_on = True

        # Let all asynchronous analog input subscriptions begin reporting.
        time.sleep(1.0)

        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("No calibration found; run python calibrate.py.")
            return

        # Telemetrix analog input callbacks arrive asynchronously.
        # Do not confuse the eight initial zero placeholders with real ADC data.
        # On this robot all eight channels normally report nonzero ADC values.
        print("Waiting for first readings from all eight line sensors...")
        ready_deadline = time.monotonic() + 6.0
        ready_count = 0
        last_report = 0.0
        while time.monotonic() < ready_deadline:
            startup_state = line_tracker.update(calibration)
            raw = startup_state["raw"]
            missing = [i + 1 for i, value in enumerate(raw) if value == 0]
            now = time.monotonic()
            if not missing:
                ready_count += 1
                if ready_count >= 3:
                    break
            else:
                ready_count = 0
            if now - last_report >= 1.0:
                print(f"ADC startup: raw={raw}; missing S={missing}")
                last_report = now
            time.sleep(LOOP_DELAY)
        if ready_count < 3:
            print("ERROR: some analog channels have no data. Motors remain stopped.")
            print(f"Latest RAW values: {startup_state['raw']}")
            print(f"Latest normalized values: {startup_state['normalized']}")
            print("Run python show_line_position.py to inspect the sensor board.")
            return
        print(f"All eight ADC channels ready: {startup_state['raw']}")
        print(f"Initial line position: {startup_state['position']}")

        detector = JunctionDetector()
        print(
            f"Differential2 ready: target {TARGET_POSITION}, "
            f"base PWM {BASE_SPEED}, max PWM {MAX_SPEED}. "
            "Automatic route active."
        )

        route_index = 0
        junction_count = 0
        candidate_count = 0
        clear_count = 0
        latched = False
        ignore_until = 0.0

        while True:
            # EXACT same reading as the working standalone differential2.py.
            state = line_tracker.update(calibration)
            position = state["position"]
            left, right, candidate = detector.read(state)
            now = time.monotonic()

            if not left and not right:
                clear_count += 1
            else:
                clear_count = 0

            if clear_count >= JUNCTION_CLEAR:
                latched = False

            if (
                not latched
                and now >= ignore_until
                and candidate
            ):
                candidate_count += 1
            else:
                candidate_count = 0

            if candidate_count >= JUNCTION_CONFIRM:
                stop()
                latched = True
                candidate_count = 0
                junction_count += 1

                if route_index >= len(route):
                    print("\nROUTE finished. Robot stopped.")
                    break
                cmd = route[route_index]
                route_index += 1
                print(
                    f"\nJUNCTION #{junction_count}: S1={int(left)} "
                    f"S8={int(right)}; ROUTE {route_index}/{len(route)} -> {cmd}"
                )
                if cmd == "Q":
                    print("ROUTE: stop command.")
                    break

                if cmd in ("L", "R", "B", "P"):
                    # Gyro stays unloaded until a maneuver requires it.
                    if bus is None:
                        bus = smbus.SMBus(I2C_BUS)
                        gyro_setup(bus)
                    offset = gyro_calibrate(bus)

                    if cmd == "B":
                        if sensor is None:
                            sensor = VL53L0XSensor()
                            sensor.initialize()
                        ok = button_task(
                            calibration, detector, sensor, bus, offset, board
                        )
                    elif cmd == "P":
                        ok = parking_task()
                        if ok:
                            # Same 80-degree R turn and differential reacquire.
                            ok, _ = pivot(bus, offset, "R")
                            ok = ok and reacquire_line(calibration)
                    else:
                        ok, _ = pivot(bus, offset, cmd)
                        ok = ok and reacquire_line(calibration)
                else:
                    ok = follow_for(calibration, 0.5)

                if not ok:
                    print("\nManeuver failed. Robot stopped.")
                    break
                if route_index >= len(route):
                    stop()
                    print("\nAll ROUTE commands completed. Robot stopped.")
                    break

                ignore_until = time.monotonic() + JUNCTION_COOLDOWN
                continue

            # Only the tested differential2.py decides how to follow the line.
            if position is None:
                stop()
                print(
                    '\rNO LINE: stopped; waiting for line sensors. '
                    f'raw={state["raw"]}',
                    end="", flush=True,
                )
            else:
                differential_drive(state)
                print(
                    f"\rLINE pos={position:+.3f} "
                    f"S1={int(left)} S8={int(right)}",
                    end="", flush=True,
                )

            time.sleep(LOOP_DELAY)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as exc:
        print(f"\nERROR: {exc}")
    finally:
        if motors_on:
            try:
                stop()
            except Exception:
                pass
        if sensor is not None:
            try:
                sensor.close()
            except Exception:
                pass
        if tracker_on:
            try:
                line_tracker.shutdown()
            except Exception:
                pass
        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass
        print("\nRobot stopped.")


if __name__ == "__main__":
    main()
    