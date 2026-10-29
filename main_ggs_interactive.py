#!/usr/bin/env python3
"""Interactive line follower for ArmRobotics_2026 (Raspberry Pi + Telemetrix)."""
import time
import smbus
from telemetrix import telemetrix
import line_tracker
import motors
from sensors.vl53l0x_sensor import VL53L0XSensor

SERIAL_PORT = '/dev/serial0'
LOOP_DELAY = 0.04
DIFF_KP = 30.0
DIFF_TARGET = 0.08
DIFF_BASE = 115
DIFF_MIN = -80
DIFF_MAX = 185
REACQUIRE_TIME = 1.2
REACQUIRE_SPEED = 90
BLACK_ON = 0.85
BLACK_OFF = 0.65
LINE_MIN = 0.20
APPROACH_SPEED = 125
SLOW_SPEED = 95

CANDIDATE_TIMEOUT = 0.35
JUNCTION_CONFIRM = 2
JUNCTION_CLEAR = 4
JUNCTION_COOLDOWN = 0.55
LINE_LOST_TIMEOUT = 0.35

I2C_BUS = 1
MPU_ADDR = 0x68
GYRO_Z_REG = 0x47
GYRO_SCALE = 131.0
GYRO_TIMEOUT = 5.0
TURN_ANGLE = 80.0
TURN_PWM_FAST = 255
TURN_PWM_SLOW = 90

BUTTON_DISTANCE_CM = 10.0
BUTTON_WAIT = 1.0
BUTTON_MAX_TIME = 12.0
BUTTON_CONFIRM = 3
BUTTON_CLEAR_TIMEOUT = 3.0
RETURN_SPEED = 155
RETURN_SLOW_SPEED = 95
RETURN_LINE_KP = 30.0
RETURN_GYRO_KP = 2.0
RETURN_MAX_DELTA = 85
RETURN_SIGN = 1
RETURN_GYRO_SIGN = 1
RETURN_MAX_TIME = 15.0
RETURN_LINE_LOST = 0.4
RETURN_MAX_ANGLE = 30.0
RETURN_RECOVERY_ANGLE = 12.0
RETURN_RECOVERY_SAFE = 6.0
RETURN_RECOVERY_TIMEOUT = 1.5
RETURN_FORWARD_TIME = 0.3
RETURN_FORWARD_SPEED = 115


def clamp(n, lo, hi):
    return max(lo, min(hi, n))


def send(left, right):
    motors.set_speeds(int(clamp(round(left), -255, 255)),
                      int(clamp(round(right), -255, 255)))


def stop():
    motors.motor_stop()


class Vision:
    def __init__(self):
        self.dark = [False] * 8
        self.position = None

    def sample(self, calibration):
        state = line_tracker.update(calibration)
        vals = state['normalized']
        self.position = differential_position(state)
        if len(vals) != 8:
            raise ValueError('Expected 8 sensor values')
        for i, v in enumerate(vals):
            if self.dark[i]:
                self.dark[i] = v > BLACK_OFF
            else:
                self.dark[i] = v >= BLACK_ON
        return vals, tuple(self.dark)


def differential_position(state):
    """Reuse exact eight-sensor position from line_tracker.update(), as differential2.py does."""
    return state['position']


def differential_forward(pos, speed=DIFF_BASE):
    error = pos - DIFF_TARGET
    correction = DIFF_KP * error
    send(clamp(speed+correction, DIFF_MIN, DIFF_MAX),
         clamp(speed-correction, DIFF_MIN, DIFF_MAX))


def reacquire_line(calibration, vision):
    # Active differential centering immediately after a turn.
    # It may recover even when only an outermost sensor sees black.
    deadline = time.monotonic() + REACQUIRE_TIME
    stable = 0
    try:
        while time.monotonic() < deadline:
            vals, _ = vision.sample(calibration)
            pos = vision.position
            if pos is None:
                stop()
                stable = 0
            else:
                differential_forward(pos, REACQUIRE_SPEED)
                stable = stable+1 if abs(pos-DIFF_TARGET) < 0.65 else 0
                if stable >= 4:
                    return True
            time.sleep(LOOP_DELAY)
        print('Differential: could not center on line after turn')
        return False
    finally:
        stop()


def edges(bits):
    return bits[0], bits[7]


def gyro_raw(bus):
    hi = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG)
    lo = bus.read_byte_data(MPU_ADDR, GYRO_Z_REG+1)
    val = (hi << 8) | lo
    return val-65536 if val >= 32768 else val


def gyro_rate(bus, offset):
    return (gyro_raw(bus)-offset)/GYRO_SCALE


def gyro_setup(bus):
    bus.write_byte_data(MPU_ADDR, 0x6B, 0)
    bus.write_byte_data(MPU_ADDR, 0x1B, 0)
    time.sleep(0.2)


def gyro_calibrate(bus):
    print('Gyro calibration: do not move the robot.')
    vals = []
    for _ in range(300):
        vals.append(gyro_raw(bus))
        time.sleep(0.005)
    return sum(vals)/len(vals)


class Heading:
    def __init__(self, bus, offset, left_positive):
        self.bus = bus
        self.offset = offset
        self.sign = left_positive
        self.angle = 0.0
        self.last = time.monotonic()

    def read(self):
        rate = gyro_rate(self.bus, self.offset)
        now = time.monotonic()
        self.angle += self.sign * rate * max(0, now-self.last)
        self.last = now
        return self.angle


def pivot(bus, offset, direction, angle_target=TURN_ANGLE):
    """Return (success, polarity if measured using a LEFT turn)."""
    left = direction == 'L'
    if direction not in ('L','R'):
        raise ValueError('Direction must be L or R')
    send(-TURN_PWM_SLOW if left else TURN_PWM_FAST,
         TURN_PWM_FAST if left else -TURN_PWM_SLOW)
    start = prev = time.monotonic()
    angle = signed = 0.0
    try:
        while angle < angle_target:
            rate = gyro_rate(bus, offset)
            now = time.monotonic()
            dt = max(0, now-prev)
            prev = now
            angle += abs(rate)*dt
            signed += rate*dt
            if now-start >= GYRO_TIMEOUT:
                print('Turn timeout')
                return False, None
            time.sleep(0.005)
        polarity = (1 if signed > 0 else -1) if left and abs(signed)>20 else None
        return True, polarity
    finally:
        stop()
        time.sleep(0.2)


def follow_for(calibration, vision, seconds, speed):
    start = time.monotonic()
    try:
        while time.monotonic()-start < seconds:
            vals, _ = vision.sample(calibration)
            pos = vision.position
            if pos is None:
                print('Line lost during timed forward movement')
                return False
            differential_forward(pos, speed)
            time.sleep(LOOP_DELAY)
        return True
    finally:
        stop()
        time.sleep(0.15)


def button_approach(calibration, vision, sensor):
    start = time.monotonic()
    clear = 0
    left_junction = False
    close = 0
    try:
        while time.monotonic()-start < BUTTON_MAX_TIME:
            vals,bits = vision.sample(calibration)
            pos = vision.position
            left,right = edges(bits)
            clear = clear+1 if not (left or right) else 0
            if clear >= JUNCTION_CLEAR:
                left_junction = True
            if not left_junction and time.monotonic()-start > BUTTON_CLEAR_TIMEOUT:
                print('Button branch: never cleared initial junction')
                return False
            dist = sensor.read_distance_mm()/10.0
            close = close+1 if left_junction and 0 < dist <= BUTTON_DISTANCE_CM else 0
            if close >= BUTTON_CONFIRM:
                stop()
                print(f'Button object at {dist:.1f} cm. Wait 1 second.')
                time.sleep(BUTTON_WAIT)
                return True
            if pos is None:
                print('Button approach: line lost')
                return False
            differential_forward(pos, APPROACH_SPEED)
            time.sleep(LOOP_DELAY)
        print('Button approach timeout')
        return False
    finally:
        stop()


def button_return(calibration, vision, bus, offset, left_sign):
    heading = Heading(bus, offset, left_sign)
    start = time.monotonic()
    recovery_since = None
    candidate_since = None
    last_line = start
    cleared = 0
    confirmed = 0
    try:
        while time.monotonic()-start < RETURN_MAX_TIME:
            vals,bits = vision.sample(calibration)
            pos = vision.position
            l,r = edges(bits)
            now = time.monotonic()
            angle = heading.read()
            if not l and not r:
                cleared += 1
            if cleared >= JUNCTION_CLEAR and l and r:
                confirmed += 1
            else:
                confirmed = 0
            # Detect junction before line-loss check. One double-edge sample
            # is accepted to avoid overshoot at 150 PWM.
            if confirmed >= 1:
                print('\nReturn junction: BOTH S1 and S8 black')
                return True
            if abs(angle) >= RETURN_MAX_ANGLE:
                print('\nReturn gyro limit exceeded')
                return False
            if l != r:
                candidate_since = now if candidate_since is None else candidate_since
            elif not l and not r:
                candidate_since = None
            if abs(angle) >= RETURN_RECOVERY_ANGLE:
                recovery_since = now if recovery_since is None else recovery_since
            elif abs(angle) <= RETURN_RECOVERY_SAFE:
                recovery_since = None
            if recovery_since is not None and now-recovery_since > RETURN_RECOVERY_TIMEOUT:
                print('\nReturn heading recovery timeout')
                return False
            if pos is not None:
                last_line = now
                err = pos-DIFF_TARGET
                line_corr = RETURN_LINE_KP * err * RETURN_SIGN
                gyro_corr = RETURN_GYRO_KP * angle * RETURN_GYRO_SIGN
                corr = clamp(line_corr+gyro_corr, -RETURN_MAX_DELTA, RETURN_MAX_DELTA)
                speed = RETURN_SLOW_SPEED if (candidate_since is not None or recovery_since is not None) else RETURN_SPEED
                send(clamp(-speed+corr,-255,-15),clamp(-speed-corr,-255,-15))
            else:
                stop()
                # Do not continue driving blind; check for a double-edge signal
                # for a short time before final line-lost failure.
                if now-last_line > RETURN_LINE_LOST:
                    print('\nReturn: LINE LOST before confirmed junction')
                    return False
            if candidate_since is not None and now-candidate_since > CANDIDATE_TIMEOUT:
                candidate_since = None
            print(f'\rBACK pos={pos if pos is not None else "---"} angle={angle:+.1f} '
                  f'S1={int(l)} S8={int(r)}',end='',flush=True)
            time.sleep(LOOP_DELAY)
        print('\nReturn timeout')
        return False
    finally:
        stop()


def button_task(calibration, vision, sensor, bus, offset):
    print('B: left turn')
    ok, left_sign = pivot(bus, offset, 'L')
    if not ok or left_sign is None or not reacquire_line(calibration, vision):
        print('B: left turn/gyro verification failed')
        return False
    if not button_approach(calibration, vision, sensor):
        return False
    if not button_return(calibration, vision, bus, offset, left_sign):
        return False
    print('B: forward adjustment for 0.3 seconds')
    if not follow_for(calibration, vision, RETURN_FORWARD_TIME, RETURN_FORWARD_SPEED):
        return False
    print('B: right turn')
    ok, _ = pivot(bus, offset, 'R', angle_target=90.0)
    return ok and reacquire_line(calibration, vision)


def ask_command(count, bits):
    stop()
    print(f'\nJUNCTION #{count}: S1={int(bits[0])} S8={int(bits[7])}')
    print('Robot STOPPED. Commands: L=left, R=right, S=straight, B=button, Q=quit')
    while True:
        cmd = input('Command [L/R/S/B/Q]: ').strip().upper()
        if cmd in ('L','R','S','B','Q'):
            return cmd
        print('Enter L, R, S, B, or Q.')


def main():
    bus = board = sensor = None
    tracker_on = motors_on = False
    count = 0
    try:
        bus = smbus.SMBus(I2C_BUS)
        gyro_setup(bus)
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        line_tracker.initialize(board)
        tracker_on = True
        calibration = line_tracker.load_calibration()
        if calibration is None:
            print('Calibration missing. Run python calibrate.py first.')
            return
        motors.initialize(board)
        motors_on = True
        stop()
        vision = Vision()
        sensor = VL53L0XSensor()
        sensor.initialize()
        print(f'VL53L0X initially {sensor.read_distance_mm()/10:.1f} cm')
        gyro_offset = gyro_calibrate(bus)
        time.sleep(0.5)
        print('Checking sensor readiness before motors start...')
        ready_frames = 0
        ready_deadline = time.monotonic() + 4.0
        while time.monotonic() < ready_deadline:
            v, _ = vision.sample(calibration)
            if vision.position is not None:
                ready_frames += 1
                if ready_frames >= 5:
                    break
            else:
                ready_frames = 0
            time.sleep(LOOP_DELAY)
        if ready_frames < 5:
            print('START CANCELLED: no stable line. Motors remain stopped.')
            return
        print('Robot ready: differential2 controller with target 0.08; prompts at junctions.')
        latched = False
        clear_count = junction_count = 0
        candidate_since = None
        ignore_until = 0.0
        last_line = time.monotonic()
        while True:
            vals,bits = vision.sample(calibration)
            pos = vision.position
            left,right = edges(bits)
            now = time.monotonic()
            clear_count = clear_count+1 if not(left or right) else 0
            if clear_count >= JUNCTION_CLEAR:
                latched = False
            # Junction check stays independent from the eight-sensor centering controller.
            candidate = (left or right) and pos is not None
            if not latched and now >= ignore_until:
                if candidate:
                    candidate_since = now if candidate_since is None else candidate_since
                    junction_count += 1
                else:
                    candidate_since = None
                    junction_count = 0
                if junction_count >= JUNCTION_CONFIRM:
                    stop()
                    latched = True
                    junction_count = 0
                    count += 1
                    cmd = ask_command(count, bits)
                    if cmd == 'Q':
                        break
                    if cmd == 'B':
                        ok = button_task(calibration, vision, sensor, bus, gyro_offset)
                    elif cmd in ('L','R'):
                        ok, _ = pivot(bus, gyro_offset, cmd)
                        ok = ok and reacquire_line(calibration, vision)
                    else:
                        ok = follow_for(calibration, vision, 0.5, 125)
                    if not ok:
                        print('Maneuver failed; stopping.')
                        break
                    # Ensure old crossing is cleared; remain latched until then.
                    ignore_until = time.monotonic()+JUNCTION_COOLDOWN
                    candidate_since = None
                    last_line = time.monotonic()
                    continue
            if pos is None:
                stop()
                if now-last_line >= LINE_LOST_TIMEOUT:
                    print('\nMain line lost. Robot stopped.')
                    break
            else:
                last_line = now
                slow = candidate and not latched and now >= ignore_until
                differential_forward(pos, SLOW_SPEED if slow else DIFF_BASE)
            if candidate_since is not None and now-candidate_since > CANDIDATE_TIMEOUT:
                candidate_since = None
            print(f'\rpos={pos if pos is not None else "---"} S1={int(left)} S8={int(right)} next=ASK',
                  end='',flush=True)
            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print('\nInterrupted; robot stopped.')
    except Exception as exc:
        print(f'\nERROR: {exc}')
    finally:
        if motors_on:
            try: stop()
            except Exception: pass
        if sensor is not None:
            try: sensor.close()
            except Exception: pass
        if tracker_on:
            try: line_tracker.shutdown()
            except Exception: pass
        if board is not None:
            try: board.shutdown()
            except Exception: pass
        if bus is not None:
            try: bus.close()
            except Exception: pass
        print('\nRobot stopped.')


if __name__ == '__main__':
    main()