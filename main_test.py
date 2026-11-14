#!/usr/bin/env python3
"""Основной маршрут ArmRobotics: навигация и задания робота."""
import math
import time
import smbus
from telemetrix import telemetrix
import line_tracker
import motors
from stm32_reset import STM32_RESET_GPIO, reset_stm32

# BUTTON использует двухсекундное следование по линии; VL53L0X не нужен.
PROGRAM_BUILD = "cube-fire-2026-11-10.1"

# Ранее проверенные скорости контроллера differential2.
SERIAL_PORT = "/dev/serial0"
BASE_SPEED = 220
MAX_SPEED = 255
MIN_SPEED = 0
MANEUVER_SPEED = 100
MANEUVER_MAX_SPEED = 170
KP = 100.0
KI = 0.0  # Существующий LineTrackingEngine использует PD; интегральный член равен нулю.
KD = 5.0
MAX_CORRECTION = 255  # Справочное значение; ограничение PWM выполняет LineTrackingEngine.
TARGET_POSITION = line_tracker.CENTER_POSITION
LOOP_DELAY = 0.04
# Восстановление пропущенного перекрёстка.

# Пороги интенсивности линии для позиционирования возле кнопки.
START_CENTER_BLACK_MIN = 0.30
START_OUTER_BLACK_MAX = 0.70

# Распознавание перекрёстков отдельно от управления линией.
BLACK_ON = 0.85
BLACK_OFF = 0.65
JUNCTION_SIDE_SUPPORT_ON = 0.55
JUNCTION_SIDE_SUPPORT_OFF = 0.35
JUNCTION_CENTER_MIN = 0.50
JUNCTION_CONFIRM = 2
JUNCTION_CLEAR = 3
JUNCTION_REARM_CLEAR_FRAMES = 3
POST_TURN_JUNCTION_IGNORE_SECONDS = 0.0
POST_JUNCTION_ACCEL_SECONDS = 0.5
STRAIGHT_MAX_SPEED = 255
STRAIGHT_ACCEL_RATE = 180.0
STRAIGHT_DECEL_RATE = 300.0
STRAIGHT_CONFIRM_SECONDS = 0.25
STRAIGHT_POSITION_TOLERANCE = 0.35
STRAIGHT_DERROR_LIMIT = 8.0

# После поворота по гироскопу робот возвращается к управлению differential2.
REACQUIRE_TIMEOUT = 1.2
REACQUIRE_TOLERANCE = 0.25
REACQUIRE_CONFIRM = 3
REACQUIRE_SPEED = 80
REACQUIRE_MAX_SPEED = 170
REACQUIRE_STEER_GAIN = 1.6
I2C_BUS = 1
MPU_ADDR = 0x68
GYRO_Z_REG = 0x47
GYRO_SCALE = 131.0
GYRO_TIMEOUT = 5.0
GYRO_CALIBRATION_SAMPLES = 20
GYRO_CALIBRATION_SETTLE_SECONDS = 0.08
GYRO_CALIBRATION_SAMPLE_DELAY = 0.001
GYRO_DIRECTION_SETTLE_SECONDS = 0.03
GYRO_DIRECTION_MIN_RATE = 8.0
GYRO_DIRECTION_CONFIRM_SAMPLES = 3
GYRO_REVERSE_CONFIRM_SECONDS = 0.05
PIVOT_SETTLE_SECONDS = 0.1
TURN_ANGLE = 75.0
TURN_90_ANGLE = 90.0
TURN_PWM_FAST = 255
TURN_PWM_SLOW = 90

# Настройки предварительного выравнивания из main_old: PWM и проезд 0,5 с.

# Эти параметры независимы от BASE_SPEED и MANEUVER_SPEED.
TURN_90_FORWARD_SPEED = 80
TURN_90_FORWARD_MAX_SPEED = 120
TURN_90_FORWARD_TIME = 0.5
SLOW_90_PWM_FAST = 110
SLOW_90_REVERSE_SECONDS = 0.15
SLOW_90_REVERSE_SPEED = 80
TURN_TOLERANCE_DEG = 8.0
TURN_CORRECTION_START_DEG = 4.0
BUTTON_BUMPER_START_CM = 20.0
BUTTON_BUMPER_START_TOLERANCE_CM = 1.0
BUTTON_BUMPER_RAM_SECONDS = 1.0
BUTTON_BUMPER_APPROACH_TIMEOUT = 18.0
BUTTON_BUMPER_SPEED = 255
BUTTON_BUMPER_STATUS_SECONDS = 0.25

# Зарядка: красный мигает 9 с, затем зелёный горит 2 с; всего 11 с перед R.
CHARGE_SECONDS = 11.0
CHARGE_RED_SECONDS = 9.0
CHARGE_BLINK_HALF_PERIOD = 0.5

# Геометрия следования по линии задним ходом (размеры в миллиметрах).

# Линейный датчик находится на 70 мм впереди колёсной оси, поэтому при

# движении назад он становится задним и требует другого регулятора.
RETURN_SPEED = 130
RETURN_SLOW_SPEED = 115
REVERSE_MIN_PWM = 80
REVERSE_MAX_PWM = 150
REVERSE_TRACKER_OFFSET_MM = 70.0
REVERSE_WHEELBASE_MM = 226.0
REVERSE_SENSOR_SPAN_MM = 60.0
REVERSE_POSITION_MM_PER_UNIT = REVERSE_SENSOR_SPAN_MM / 7.0

# Отдельный PD-регулятор и наблюдатель гироскопа для заднего хода; настройка на роботе.
REVERSE_LATERAL_KP = 2.6          # PWM на мм ошибки, оценённой у оси.
REVERSE_LATERAL_KD = 0.055        # PWM на (мм/с); подавляет колебания.
REVERSE_HEADING_KP = 0.70         # PWM на градус ошибки курса MPU.
REVERSE_YAW_RATE_KD = 0.12        # PWM на (градус/с); подавляет рыскание.
REVERSE_REFERENCE_FOLLOW = 2.5    # 1/с: скорость подстройки опорного курса на поворотах.
REVERSE_REFERENCE_WINDOW_MM = 7.0
REVERSE_MAX_BRAKE_PWM = 70
REVERSE_PWM_SLEW_PER_LOOP = 7
REVERSE_LINE_LOSS_TIMEOUT = 0.8
BUTTON_REVERSE_LINE_LOSS_TIMEOUT = 5.0
REVERSE_GYRO_SPEED = 90

# Установить -1 только если тест подтвердит обратное подключение

# левой и правой сторон моторов.
REVERSE_STEER_SIGN = 1
RETURN_MAX_TIME = 24.0
RETURN_FORWARD_TIME = 0.5
BUTTON_RETURN_JUNCTION_WINDOW = 0.40
BUTTON_RETURN_JUNCTION_CONFIRM = 2
BUTTON_RETURN_JUNCTION_SLOW_SPEED = 100
CUBE_LOST_FINAL_RIGHT_ANGLE = 80.0
FIRE_APPROACH_SPEED = 255
FIRE_APPROACH_MAX_SPEED = 255
FIRE_APPROACH_TIMEOUT = 12.0
FIRE_FINAL_ADVANCE_SECONDS = 0.3
FIRE_LINE_RECOVERY_SPEED = 80
FIRE_LINE_RECOVERY_TIMEOUT = 5.0
FIRE_LINE_RECOVERY_CONFIRM = 2
FIRE_LINE_BLACKNESS_MIN = 0.80
FIRE_LINE_END_CONFIRM = 3
FIRE_RETURN_MAX_TIME = 40.0

# CUBE1: рука сохраняет указанную позицию XYZ с открытым захватом.
CUBE1_STAGE_X_MM = 250.0
CUBE1_STAGE_Y_MM = 0.0
CUBE1_STAGE_Z_MM = 100.0
CUBE1_X_MM = 200.0
CUBE1_Y_MM = 0.0
CUBE1_Z_MM = -180.0
CUBE1_TOOL_PITCH_DEG = -90.0
CUBE1_PIT_ALL_BLACK_MIN = 0.85
CUBE1_PIT_ALL_WHITE_MAX = 0.15  # Все восемь датчиков видят белое.
CUBE1_PIT_CONFIRM_FRAMES = 4
CUBE1_EMERGENCY_TIMEOUT = 8.0
CUBE1_BLINK_HALF_PERIOD = 0.25
CUBE1_DROP_CONFIRM_FRAMES = 3
CUBE1_DROP_TIMEOUT = 20.0
CUBE1_APPROACH_SPEED = 255
CUBE1_APPROACH_MAX_SPEED = 255
CUBE1_SENSOR_CONFIRM = 3
CUBE1_LINE_LOSS_CONFIRM = 5
CUBE1_EMERGENCY_REVERSE_CONFIRM = 3
CUBE1_EMERGENCY_REVERSE_SPEED = 80
# После возврата CUBE1 на исходный перекрёсток выполнить проезд перед R90.
CUBE1_RETURN_FORWARD_SECONDS = 0.5
CUBE1_RETURN_FORWARD_PWM = 80

# COLOR: остановиться, найти цветной маркер и включить соответствующий RGB.
COLOR_MARKER_TIMEOUT = 10.0
COLOR_MARKER_HOLD_SECONDS = 1.0
COLOR_MARKER_MIN_AREA = 700
COLOR_MARKER_STABLE_FRAMES = 3
COLOR_ARM_ANGLES = {
    "base": 90.0,
    "shoulder": 160.0,
    "elbow": 180.0,
    "wrist": 0.0,
    "gripper": 0.0,
}
COLOR_LED_HOLD = None
# BUTTON_BOUNCE: обычный PD; меняется только базовый PWM.
BUTTON_BOUNCE_DRIVE_PWM = 80
BUTTON_BOUNCE_MIN_OBJECT_AREA = 140
BUTTON_BOUNCE_CONFIRM_FRAMES = 3
BUTTON_BOUNCE_TARGET_X = (0.35, 0.65)
BUTTON_BOUNCE_TARGET_Y = (0.65, 0.90)
BUTTON_BOUNCE_PRESS = (0.0, 250.0, -20.0)
BUTTON_BOUNCE_REVERSE_LINE_TIMEOUT = 5.0

# Автоматический маршрут: по одной команде на каждый найденный перекрёсток.

# L/R/S и L90/R90 задают движение; именованные команды запускают задания.

# CHARGING, FIREFIGHTER, CUBE1 и COLOR — задания; Q завершает маршрут.

ROUTE = [
    "BUTTON",
    "L",
    "CUBE1",
    "L",
    "L",
    "R",
    "CHARGING",
    "COLOR",
    "L90",
    "R",
    "R",
    "L90",
    "FIREFIGHTER",
    "L",
    "R90",
    "BUTTON_BOUNCE",
    "R90",
    "L",
    "R",
    "R",
    "S",
    "L",
    "L",
    "S",
    "Q",

]
ALLOWED_ROUTE_COMMANDS = {
    "L", "R", "L90", "R90", "L90SLOW", "R90SLOW", "S", "SLOW",
    "BUTTON", "BUTTON_BOUNCE", "CHARGING", "FIREFIGHTER", "CUBE1", "COLOR", "Q",
}
line_follower = line_tracker.LineTrackingEngine(
    base_speed=BASE_SPEED,
    max_speed=MAX_SPEED,
    min_speed=MIN_SPEED,
    kp=KP,
    kd=KD,
    target_position=TARGET_POSITION,
)


def stop():
    line_follower.stop()


def differential_drive(state, speed=BASE_SPEED, max_speed=MAX_SPEED):
    """Совместимый интерфейс для отдельного контроллера следования по линии."""
    return line_follower.drive(state, speed=speed, max_speed=max_speed)


def post_junction_drive_limits(elapsed):
    """Плавно увеличивает базовый PWM и максимальный PWM колёс после манёвра."""
    fraction = max(0.0, min(1.0, elapsed / POST_JUNCTION_ACCEL_SECONDS))
    start_speed = min(MANEUVER_SPEED, BASE_SPEED)
    start_max = min(MANEUVER_MAX_SPEED, MAX_SPEED)
    speed = start_speed + (BASE_SPEED - start_speed) * fraction
    max_speed = start_max + (MAX_SPEED - start_max) * fraction
    return speed, max_speed


def update_straight_speed(state, left, right, candidate, controller, now):
    """Ускоряет робот только на устойчивой прямой без ответвлений."""
    position = state.get("position") if state else None
    previous_time = controller["previous_time"]
    dt = 0.0 if previous_time is None else max(0.0, now - previous_time)
    controller["previous_time"] = now
    if position is None:
        controller["current_speed"] = BASE_SPEED
        controller["previous_error"] = None
        controller["straight_since"] = None
        return BASE_SPEED, MAX_SPEED
    error = position - TARGET_POSITION
    previous_error = controller["previous_error"]
    derivative = 0.0
    if previous_error is not None and dt > 0.0:
        derivative = (error - previous_error) / dt
    controller["previous_error"] = error
    stable = (
        not left and not right and not candidate
        and abs(error) <= STRAIGHT_POSITION_TOLERANCE
        and abs(derivative) <= STRAIGHT_DERROR_LIMIT
    )
    if stable:
        if controller["straight_since"] is None:
            controller["straight_since"] = now
        if now - controller["straight_since"] >= STRAIGHT_CONFIRM_SECONDS:
            controller["current_speed"] = min(
                STRAIGHT_MAX_SPEED,
                controller["current_speed"] + STRAIGHT_ACCEL_RATE * dt,
            )
    else:
        controller["straight_since"] = None
        controller["current_speed"] = max(
            BASE_SPEED,
            controller["current_speed"] - STRAIGHT_DECEL_RATE * dt,
        )
    return controller["current_speed"], min(
        STRAIGHT_MAX_SPEED,
        max(MANEUVER_MAX_SPEED, controller["current_speed"]),
    )


def differential_reverse(state, speed, controller=None):
    """Запускает отдельный контроллер заднего хода (совместимость с тестами)."""
    if controller is not None:
        return controller.drive(state, speed)
    return line_follower.reverse(state, speed)


class JunctionDetector:
    """Определяет только ответвления; не рассчитывает положение линии и не управляет моторами."""

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
        left_edge = self.hysteresis(self.left_black, values[0])
        right_edge = self.hysteresis(self.right_black, values[7])
        left_support = values[1] >= (
            JUNCTION_SIDE_SUPPORT_OFF
            if self.left_black else JUNCTION_SIDE_SUPPORT_ON
        )
        right_support = values[6] >= (
            JUNCTION_SIDE_SUPPORT_OFF
            if self.right_black else JUNCTION_SIDE_SUPPORT_ON
        )
        self.left_black = left_edge and left_support
        self.right_black = right_edge and right_support
        side_pattern = self.left_black or self.right_black

        # Срабатывание одного крайнего датчика означает смещение, а не перекрёсток.
        center_present = any(
            values[index] >= JUNCTION_CENTER_MIN
            for index in (2, 3, 4, 5)
        )
        candidate = (
            state["position"] is not None
            and center_present
            and side_pattern
        )
        return self.left_black, self.right_black, candidate


class JunctionRearmGate:
    """Разрешает новый перекрёсток только после устойчивого центрирования."""

    def __init__(self):
        self.armed = True
        self.clear_count = 0

    def mark_junction(self):
        self.armed = False
        self.clear_count = 0

    def update(self, left, right, position):
        if self.armed:
            return
        centered_clear = (
            not left
            and not right
            and position is not None
            and abs(position - TARGET_POSITION) <= STRAIGHT_POSITION_TOLERANCE
        )
        if not centered_clear:
            self.clear_count = 0
            return
        self.clear_count += 1
        if self.clear_count >= JUNCTION_REARM_CLEAR_FRAMES:
            self.armed = True
            self.clear_count = 0


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


def gyro_calibrate(bus, settle_seconds=GYRO_CALIBRATION_SETTLE_SECONDS):
    stop()
    time.sleep(settle_seconds)
    values = []
    for _ in range(GYRO_CALIBRATION_SAMPLES):
        values.append(gyro_raw(bus))
        time.sleep(GYRO_CALIBRATION_SAMPLE_DELAY)
    return sum(values) / len(values)


class Heading:

    def __init__(self, bus, offset, left_positive):
        self.bus = bus
        self.offset = offset
        self.sign = left_positive
        self.angle = 0.0
        self.last = time.monotonic()
        self.last_rate = 0.0

    def read(self):
        now = time.monotonic()
        rate = self.sign * gyro_rate(self.bus, self.offset)
        self.angle += rate * max(0.0, now - self.last)
        self.last = now
        self.last_rate = rate
        return self.angle


def _wrap_degrees(angle):
    """Приводит разницу углов к диапазону [-180, 180) градусов."""
    return (angle + 180.0) % 360.0 - 180.0


class ReverseLineController:
    """Контроллер движения назад с учётом положения датчиков относительно оси.
    Датчики находятся на 70 мм впереди оси, а при заднем ходе оказываются позади.
    Поэтому обычный PD-регулятор запаздывает и вызывает колебания.
    Гироскоп компенсирует поворот корпуса; на поворотах трассы опорный курс
    постепенно подстраивается, когда боковая ошибка небольшая."""

    def __init__(self, heading, line_loss_timeout=REVERSE_LINE_LOSS_TIMEOUT):
        self.heading = heading
        self.line_loss_timeout = line_loss_timeout
        self.reference_heading = None
        self.previous_axle_error = None
        self.previous_time = None
        self.previous_pwm = None
        self.line_lost_since = None
        self.last_angle = 0.0
        self.last_axle_error = None
        self.mode = "LINE"

    @staticmethod

    def _slew(previous, target):
        if previous is None:
            return target
        step = REVERSE_PWM_SLEW_PER_LOOP
        return max(previous - step, min(previous + step, target))

    def _wheel_command(self, base_pwm, steering):
        """При движении назад подтормаживает только одно колесо и ограничивает изменение PWM."""
        base = max(REVERSE_MIN_PWM, min(REVERSE_MAX_PWM, round(base_pwm)))
        brake = min(REVERSE_MAX_BRAKE_PWM, abs(steering))
        left = right = -base

        # Положительная коррекция тормозит правое колесо. При движении назад

        # реакция корпуса обратная движению вперёд; знак коррекции

        # задаётся калибровкой оборудования, а не перестановкой значений PWM.
        if steering * REVERSE_STEER_SIGN >= 0:
            right = -max(REVERSE_MIN_PWM, base - round(brake))
        else:
            left = -max(REVERSE_MIN_PWM, base - round(brake))
        old_left, old_right = self.previous_pwm or (None, None)
        left = self._slew(old_left, left)
        right = self._slew(old_right, right)
        self.previous_pwm = (left, right)
        motors.set_speeds(left, right)
        return True

    def drive(self, state, speed):
        now = time.monotonic()
        angle = self.heading.read()
        rate = self.heading.last_rate
        self.last_angle = angle
        position = state.get("position") if state else None
        if position is None:

            # Краткое движение только по гироскопу перекрывает пропуски ADC,

            # движется назад по гироскопу до обнаружения перекрёстка датчиком S8.
            if self.line_lost_since is None:
                self.line_lost_since = now
            if (
                self.line_loss_timeout is not None
                and now - self.line_lost_since > self.line_loss_timeout
            ):
                return False
            self.mode = "GYRO"
            heading_error = (
                0.0 if self.reference_heading is None else
                _wrap_degrees(angle - self.reference_heading)
            )
            steering = (
                REVERSE_HEADING_KP * heading_error
                + REVERSE_YAW_RATE_KD * rate
            )
            return self._wheel_command(REVERSE_GYRO_SPEED, steering)
        self.line_lost_since = None
        sensor_error_mm = position * REVERSE_POSITION_MM_PER_UNIT
        if self.reference_heading is None:
            self.reference_heading = angle
        heading_error = _wrap_degrees(angle - self.reference_heading)

        # Датчик расположен в x=+70 мм от оси колёс.

        # Вычитаем составляющую курса для оценки ошибки у оси.
        axle_error = sensor_error_mm - (
            REVERSE_TRACKER_OFFSET_MM
            * math.sin(math.radians(heading_error))
        )
        self.last_axle_error = axle_error
        dt = 0.0 if self.previous_time is None else now - self.previous_time
        derivative = 0.0
        if (self.previous_axle_error is not None
                and 0.005 <= dt <= 0.25):
            derivative = (axle_error - self.previous_axle_error) / dt
        self.previous_axle_error = axle_error
        self.previous_time = now

        # На изгибе трассы опорный курс должен следовать направлению линии,

        # но только при небольшом боковом отклонении.
        if abs(sensor_error_mm) <= REVERSE_REFERENCE_WINDOW_MM and dt > 0:
            follow = min(1.0, REVERSE_REFERENCE_FOLLOW * dt)
            self.reference_heading += heading_error * follow
            heading_error = _wrap_degrees(angle - self.reference_heading)
        steering = (
            REVERSE_LATERAL_KP * axle_error
            + REVERSE_LATERAL_KD * derivative
            + REVERSE_HEADING_KP * heading_error
            + REVERSE_YAW_RATE_KD * rate
        )

        # При большом отклонении снижаем скорость, сохраняя запас

        # для дифференциального торможения и предел заднего PWM 150.
        base = max(
            RETURN_SLOW_SPEED,
            speed - min(25.0, abs(axle_error) * 0.8),
        )
        self.mode = "LINE"
        return self._wheel_command(base, steering)


def pivot(bus, offset, direction, angle_target=TURN_ANGLE):
    """Исходный поворот L/R, также применяемый в заданиях B/C/F."""
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

        # Возвращаем полярность гироскопа для реального поворота влево.

        # При правом повороте знак обратный; поэтому на заднем ходе

        # одна и та же коррекция курса применима к обоим направлениям.
        left_sign = None
        if abs(signed) > 20:
            observed_sign = 1 if signed > 0 else -1
            left_sign = observed_sign if left else -observed_sign
        return True, left_sign
    finally:
        stop()
        time.sleep(PIVOT_SETTLE_SECONDS)


def pivot_90(bus, offset, direction, angle_target=TURN_90_ANGLE, *, slow=False):
    """Поворот с проверенным алгоритмом коррекции до 90 градусов."""
    if direction not in ("L", "R"):
        raise ValueError("Turn direction must be L or R")
    stop()
    left = direction == "L"
    start = previous = time.monotonic()
    angle = signed = 0.0
    gyro_sign = None
    candidate_sign = None
    direction_samples = 0
    reverse_since = None
    last_rate = 0.0
    lead = 5.6425 if left else 6.125
    try:
        while True:
            now = time.monotonic()
            rate = gyro_rate(bus, offset)
            dt = max(0.0, min(now - previous, 0.1))
            previous = now
            if not math.isfinite(rate) or abs(rate) > 1000:
                raise RuntimeError("GYRO_TURN: invalid angular rate")

            # Игнорируем измерения до начала движения и во время инерции.

            # Определяем полярность только после устойчивой реакции моторов.
            if gyro_sign is None:
                if (now - start >= GYRO_DIRECTION_SETTLE_SECONDS
                        and abs(rate) >= GYRO_DIRECTION_MIN_RATE):
                    observed_sign = 1 if rate > 0 else -1
                    direction_samples = (
                        direction_samples + 1
                        if observed_sign == candidate_sign else 1
                    )
                    candidate_sign = observed_sign
                    if direction_samples >= GYRO_DIRECTION_CONFIRM_SAMPLES:
                        gyro_sign = candidate_sign
                else:
                    direction_samples = 0
                    candidate_sign = None
            if gyro_sign is not None:
                directed = rate * gyro_sign
                if directed < -GYRO_DIRECTION_MIN_RATE:
                    if reverse_since is None:
                        reverse_since = now
                    elif now - reverse_since >= GYRO_REVERSE_CONFIRM_SECONDS:
                        raise RuntimeError("GYRO_TURN: gyro direction reversed")
                else:
                    reverse_since = None
                angle = max(0.0, angle + directed * dt)
                signed += rate * dt
            last_rate = abs(rate)
            stop_angle = angle_target - max(
                lead, min(12.0, last_rate * 0.04)
            )
            if angle >= stop_angle:
                break
            if now - start >= GYRO_TIMEOUT or (now - start > 0.7 and angle < 2):
                print(f"\nGYRO_TURN timeout/no motion at {angle:.1f} degrees")
                return False, None
            pwm = (SLOW_90_PWM_FAST if slow else
                   (TURN_PWM_SLOW if angle >= angle_target - 25 else TURN_PWM_FAST))
            motors.set_speeds(-pwm if left else pwm, pwm if left else -pwm)
            time.sleep(0.005)
        stop()
        settle_until = time.monotonic() + 0.05
        while time.monotonic() < settle_until:
            now = time.monotonic()
            signed += gyro_rate(bus, offset) * max(
                0.0, min(now - previous, 0.1)
            )
            previous = now
            time.sleep(0.005)
        final_angle = abs(signed)
        if (angle_target >= 45
                and TURN_CORRECTION_START_DEG < angle_target - final_angle <= 20):
            correction_deadline = time.monotonic() + 0.2
            while time.monotonic() < correction_deadline:
                now = time.monotonic()
                signed += gyro_rate(bus, offset) * max(
                    0.0, min(now - previous, 0.1)
                )
                previous = now
                if abs(signed) >= angle_target - lead:
                    break
                correction_pwm = 70 if not slow else min(70, SLOW_90_PWM_FAST)
                motors.set_speeds(
                    -correction_pwm if left else correction_pwm,
                    correction_pwm if left else -correction_pwm,
                )
                time.sleep(0.005)
            stop()
            final_angle = abs(signed)
        tolerance = min(TURN_TOLERANCE_DEG, angle_target * 0.2)
        print(
            f"\nGYRO_TURN {direction}: target={angle_target:.1f}, "
            f"gyro={final_angle:.1f}, error={final_angle-angle_target:+.1f}"
        )
        left_sign = None
        if abs(signed) > 5:
            observed_sign = 1 if signed > 0 else -1
            left_sign = observed_sign if left else -observed_sign
        return abs(final_angle - angle_target) <= tolerance, left_sign
    finally:
        stop()
        time.sleep(0.02)


def aggressive_reacquire_drive(state):
    """Резко корректирует положение на небольшой скорости до центрирования датчиков."""
    position = state.get("position") if state else None
    if position is None:
        line_follower.drive_straight(
            speed=REACQUIRE_SPEED,
            max_speed=REACQUIRE_MAX_SPEED,
        )
        return False
    correction = (
        line_follower.steering_correction(position) * REACQUIRE_STEER_GAIN
    )
    left = max(
        MIN_SPEED,
        min(REACQUIRE_MAX_SPEED, REACQUIRE_SPEED + correction),
    )
    right = max(
        MIN_SPEED,
        min(REACQUIRE_MAX_SPEED, REACQUIRE_SPEED - correction),
    )
    motors.set_speeds(round(left), round(right))
    return True


def reacquire_line(calibration, recover_connection=None):
    """После поворота восстанавливает линию с использованием differential2."""
    deadline = time.monotonic() + REACQUIRE_TIMEOUT
    stable = 0
    try:
        while time.monotonic() < deadline:
            if not line_tracker.all_channels_fresh():
                stop()
                if recover_connection is None or not recover_connection("reacquire"):
                    print("\nREACQUIRE: ADC stale; stopping.")
                    return False
                deadline = time.monotonic() + REACQUIRE_TIMEOUT
                stable = 0
                continue
            state = line_tracker.update(calibration)
            position = state["position"]
            if not aggressive_reacquire_drive(state):
                stable = 0
            else:
                if abs(position - TARGET_POSITION) <= REACQUIRE_TOLERANCE:
                    stable += 1
                    if stable >= REACQUIRE_CONFIRM:
                        print("\nREACQUIRE: line centered.")
                        return True
                else:
                    stable = 0
            print(
                f"\rREACQUIRE pos={position} N=["
                + " ".join(
                    f"{value:.2f}" for value in state["normalized"]
                )
                + "]",
                end="",
                flush=True,
            )
            time.sleep(LOOP_DELAY)
        print("\nREACQUIRE timeout: line is not centered; stopping maneuver.")
        return False
    finally:
        stop()


def follow_for(calibration, seconds, *, speed=MANEUVER_SPEED,
               max_speed=MANEUVER_MAX_SPEED, recover_connection=None):
    """Движение вперёд заданное время с сохранением регулятора differential2."""
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            if not line_tracker.all_channels_fresh():
                stop()
                remaining = max(0.0, deadline - time.monotonic())
                if recover_connection is None or not recover_connection("forward"):
                    print("\nFORWARD: ADC stale; stopping.")
                    return False
                deadline = time.monotonic() + remaining
                continue
            state = line_tracker.update(calibration)
            if state["position"] is None:
                line_follower.drive_straight(
                    speed=speed,
                    max_speed=max_speed,
                )
            else:
                differential_drive(
                    state, speed=speed, max_speed=max_speed
                )
            time.sleep(LOOP_DELAY)
        return True
    finally:
        stop()
        time.sleep(0.15)


def activate_arm_home(board):
    """Один раз включает сервоприводы и устанавливает руку в безопасное начальное положение."""
    from arm_web import START_ANGLES
    from roboarm import get_shared_arm
    arm, _ = get_shared_arm(board)
    arm.write(**START_ANGLES)
    print("ARM: active in base pose (90, 170, 180, 0, 170).")


def button_bumper_approach(calibration, sensor):
    """Нажимает кнопку бампером корпуса без движения руки (устаревший вариант)."""
    deadline = time.monotonic() + BUTTON_BUMPER_APPROACH_TIMEOUT
    ram_started = False
    ram_deadline = None
    last_status = 0.0
    try:
        while time.monotonic() < deadline:
            distance_cm = sensor.read_distance_mm() / 10.0
            if not math.isfinite(distance_cm):
                print("\nBUTTON: invalid distance reading; stopping.")
                return False
            state = line_tracker.update(calibration)
            if not ram_started:

                # Полный таран начинается с расстояния 20 ± 1 см (старый вариант).
                if distance_cm <= (
                    BUTTON_BUMPER_START_CM + BUTTON_BUMPER_START_TOLERANCE_CM
                ):
                    ram_started = True
                    ram_deadline = (
                        time.monotonic() + BUTTON_BUMPER_RAM_SECONDS
                    )
                    print(
                        f"\nBUTTON: {distance_cm:.1f} cm; "
                        f"starting {BUTTON_BUMPER_RAM_SECONDS:g}-second "
                        "full-speed bumper press."
                    )
                elif state["position"] is None:
                    stop()
                    time.sleep(LOOP_DELAY)
                    continue
                else:
                    differential_drive(
                        state,
                        speed=MANEUVER_SPEED,
                        max_speed=MANEUVER_MAX_SPEED,
                    )
                    time.sleep(LOOP_DELAY)
                    continue
            if time.monotonic() >= ram_deadline:
                stop()
                print(
                    f"\nBUTTON: bumper run complete; final distance "
                    f"{distance_cm:.1f} cm. Returning to junction."
                )
                return True
            now = time.monotonic()
            if now - last_status >= BUTTON_BUMPER_STATUS_SECONDS:
                print(
                    f"\rBUTTON RAM distance={distance_cm:.1f} cm",
                    end="",
                    flush=True,
                )
                last_status = now
            if state["position"] is None:

                # Во время ограниченного тарана можно продолжать прямо при потере линии.
                line_follower.drive_straight(
                    speed=BUTTON_BUMPER_SPEED,
                    max_speed=BUTTON_BUMPER_SPEED,
                )
            else:
                differential_drive(
                    state,
                    speed=BUTTON_BUMPER_SPEED,
                    max_speed=BUTTON_BUMPER_SPEED,
                )
            time.sleep(LOOP_DELAY)
        print("\nBUTTON: bumper approach timeout.")
        return False
    finally:
        stop()


def button_return(
    calibration,
    detector,
    bus,
    offset,
    left_sign,
    label="BUTTON",
    accept_s8_only=False,
    accept_any_junction=False,
    max_return_time=RETURN_MAX_TIME,
    line_loss_timeout=REVERSE_LINE_LOSS_TIMEOUT,
    recover_connection=None,
):
    """Возврат назад с геометрически корректным регулятором."""
    heading = Heading(bus, offset, left_sign)
    reverse_controller = ReverseLineController(
        heading, line_loss_timeout=line_loss_timeout
    )
    start = time.monotonic()
    cleared_count = 0
    outer_confirm_count = 0
    candidate_since = None
    button_first_edge_at = None
    button_seen_left = False
    button_seen_right = False
    button_confirm = 0
    try:
        while (
            max_return_time is None
            or time.monotonic() - start < max_return_time
        ):
            if not line_tracker.all_channels_fresh():
                stop()
                print(f"\n{label}: ADC stopped updating during reverse.")
                if recover_connection is None or not recover_connection(f"{label} reverse"):
                    return False
                # STM32 reset: новая серия данных, курс и поиск перекрёстка.
                heading = Heading(bus, offset, left_sign)
                reverse_controller = ReverseLineController(
                    heading, line_loss_timeout=line_loss_timeout
                )
                start = time.monotonic()
                cleared_count = 0
                outer_confirm_count = 0
                candidate_since = None
                button_first_edge_at = None
                button_seen_left = button_seen_right = False
                button_confirm = 0
                continue
            state = line_tracker.update(calibration)
            left, right, candidate = detector.read(state)
            now = time.monotonic()
            if not left and not right:
                cleared_count += 1
            if accept_any_junction:
                outer_confirm_count = (
                    outer_confirm_count + 1
                    if (
                        cleared_count >= JUNCTION_CLEAR
                        and (candidate or left or right)
                    )
                    else 0
                )
                if outer_confirm_count >= JUNCTION_CONFIRM:
                    print(f"\n{label}: return junction found.")
                    return True
            elif accept_s8_only:
                outer_confirm_count = (
                    outer_confirm_count + 1
                    if cleared_count >= JUNCTION_CLEAR and right
                    else 0
                )
                if outer_confirm_count >= JUNCTION_CONFIRM:
                    print(
                        f"\n{label}: return junction found by S8 "
                        "(S1 is ignored for this task)."
                    )
                    return True

            # BUTTON: запоминаем срабатывания краёв в коротком окне.
            # Не требуем одновременной чёрной линии на S1 и S8.
            elif label == "BUTTON":
                if cleared_count >= JUNCTION_CLEAR:
                    if button_first_edge_at is not None and (
                        now - button_first_edge_at > BUTTON_RETURN_JUNCTION_WINDOW
                        or (not left and not right)
                    ):
                        button_first_edge_at = None
                        button_seen_left = button_seen_right = False
                        button_confirm = 0
                    if left or right:
                        if button_first_edge_at is None:
                            button_first_edge_at = now
                        button_seen_left |= left
                        button_seen_right |= right
                        if button_seen_left and button_seen_right:
                            button_confirm += 1
                            if button_confirm >= BUTTON_RETURN_JUNCTION_CONFIRM:
                                print(f"\n{label}: return junction found (S1/S8 confirmed).")
                                return True
                        else:
                            button_confirm = 0
                else:
                    button_first_edge_at = None
                    button_seen_left = button_seen_right = False
                    button_confirm = 0
            # Остальные задания сохраняют исходное правило обоих краёв.
            elif cleared_count >= JUNCTION_CLEAR and left and right:
                print(f"\n{label}: return junction found (both outer sensors).")
                return True

            # При возврате CUBE1 датчик S1 не видит ответвление.

            # Не используем его для распознавания перекрёстка и выбора скорости.
            candidate_visible = right if accept_s8_only else left != right
            if label == "BUTTON" and button_first_edge_at is not None:
                candidate_visible = True
            if candidate_visible:
                if candidate_since is None:
                    candidate_since = now
            elif accept_s8_only or (not left and not right):
                candidate_since = None
            speed = RETURN_SLOW_SPEED if candidate_since is not None else RETURN_SPEED
            if label == "BUTTON" and button_first_edge_at is not None:
                speed = BUTTON_RETURN_JUNCTION_SLOW_SPEED
            if not differential_reverse(state, speed, reverse_controller):
                print(f"\n{label}: line lost too long while reversing.")
                return False
            print(
                f'\r{label} BACK {reverse_controller.mode} '
                f'pos={state["position"]} axle={reverse_controller.last_axle_error} '
                f'angle={reverse_controller.last_angle:+.1f} '
                f'S1={int(left)} S8={int(right)}',
                end="", flush=True,
            )
            time.sleep(LOOP_DELAY)
        print(f"\n{label}: reverse timeout.")
        return False
    finally:
        stop()


def button_task(
    calibration, detector, sensor, bus, offset, board, recover_connection=None,
):
    """Проверенные повороты и двухсекундное движение по линии."""
    print("\nBUTTON: turn LEFT.")
    ok, left_sign = pivot(bus, offset, "L")
    if not ok or left_sign is None or not reacquire_line(
        calibration, recover_connection=recover_connection
    ):
        return False
    print("\nBUTTON: follow line for 2 seconds.")
    if not follow_for(calibration, 2.0, recover_connection=recover_connection):
        return False
    if not button_return(
        calibration, detector, bus, offset, left_sign,
        line_loss_timeout=BUTTON_REVERSE_LINE_LOSS_TIMEOUT,
        recover_connection=recover_connection,
    ):
        return False
    if not follow_for(
        calibration, RETURN_FORWARD_TIME, recover_connection=recover_connection
    ):
        return False
    print("\nBUTTON: final R90.")
    ok, _ = pivot_90(bus, offset, "R", angle_target=TURN_90_ANGLE)
    return ok and reacquire_line(
        calibration, recover_connection=recover_connection
    )


def cube1_arm_action(board):
    """Две заданные позиции руки без дополнительного центрирования корпуса."""
    from roboarm import get_shared_arm
    from button_arm import _execute_plan
    import arm_web
    stop()
    arm, _ = get_shared_arm(board)
    arm_web.arm = arm
    try:
        # Предварительное положение с автоматическим углом кисти и открытым захватом.
        arm.write(gripper=0)
        first, free_pitch = arm_web.plan_auto_path({
            "x": CUBE1_STAGE_X_MM, "y": CUBE1_STAGE_Y_MM,
            "z": CUBE1_STAGE_Z_MM, "orientation_mode": "auto",
        })
        print(f"CUBE1: stage 1 (250,0,100), free pitch {free_pitch:.1f}, gripper open.")
        _execute_plan(arm, first, arm_web.STEP_DELAY)
        arm.write(gripper=0)

        # Конечная позиция фиксирует абсолютный угол захвата -90 градусов.
        second = arm_web.plan_target_pitch_path({
            "x": CUBE1_X_MM, "y": CUBE1_Y_MM, "z": CUBE1_Z_MM,
            "tool_pitch": CUBE1_TOOL_PITCH_DEG,
        })
        print("CUBE1: stage 2 (200,0,-180), pitch -90, gripper open.")
        _execute_plan(arm, second, arm_web.STEP_DELAY)
        arm.write(gripper=0)
        return True
    except Exception as exc:
        print(f"CUBE1: arm motion failed: {exc}")
        stop()
        return False


def cube1_fold_arm(board, reason="task endpoint reached"):
    """Поднимает и складывает руку после завершения CUBE1."""
    from arm_web import START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY
    from button_arm import _return_home
    from roboarm import get_shared_arm
    stop()
    print(f"\nCUBE1: {reason}; raising and folding arm home.")
    try:
        arm, newly_attached = get_shared_arm(board)
        if newly_attached:
            arm.write(**START_ANGLES)
        else:
            _return_home(
                arm, START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY
            )
        # _return_home также применяет начальный угол захвата.
        print("CUBE1: arm is home; ready for the final right turn.")
        return True
    except Exception as exc:
        print(f"CUBE1: could not fold arm home: {exc}")
        return False
    finally:
        stop()


def cube1_create_indicator():
    """Создаёт RGB-индикатор с учётом перепутанных красного и синего проводов."""
    return create_robot_rgb_indicator()


def create_robot_rgb_indicator():
    """Подключает физический RGB-индикатор с учётом измеренной распиновки."""
    import led
    import robot_pin_config as pins
    rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
    # gpiozero уже выключает индикатор при создании; повторно выключаем его,
    # чтобы новое задание не унаследовало цвет предыдущего.
    rgb.off()
    return rgb


class Cube1Signal:
    """Неблокирующий индикатор состояния RGB без фоновых записей GPIO."""
    def __init__(self, led):
        self.led = led
        self.mode = "off"
        self.last_visible = None

    def set_mode(self, mode):
        if mode not in ("off", "green_blink", "green", "red_blink"):
            raise ValueError(mode)
        if mode != self.mode:
            self.mode = mode
            self.last_visible = None
        self.tick()

    def tick(self, now=None):
        if now is None:
            now = time.monotonic()
        show = self.mode == "green" or (
            self.mode.endswith("_blink")
            and int(now / CUBE1_BLINK_HALF_PERIOD) % 2 == 0
        )
        color = "green" if self.mode.startswith("green") else "red"
        state = (show, color if show else None)
        if state != self.last_visible:
            if show:
                getattr(self.led, color)()
            else:
                self.led.off()
            self.last_visible = state

    def close(self):
        self.led.off()
        self.led.close()


def cube1_transport(calibration, detector, cube_sensor, indicator,
                    force_emergency=False, recover_ir=None):
    """Поиск и перенос кубика CUBE1 с ограниченным аварийным сценарием.
    «drop» — кубик исчез после второго перекрёстка.
    «drop_line_lost» — после него пропала линия.
    «emergency» — подтверждена яма в аварийном режиме.
    Полностью чёрный перекрёсток нельзя принимать за яму."""
    deadline = time.monotonic() + CUBE1_DROP_TIMEOUT
    phase = "search"
    clear_count = 0
    junction_armed = False
    crossing_count = 0
    second_crossing_seen = False
    second_crossing_cleared = False
    post_cross_clear_count = 0
    present_count = 0
    missing_count = 0
    line_loss_count = 0
    pit_count = 0
    pit_kind = None
    emergency_deadline = None
    last_status = 0.0

    def enter_emergency(reason, now):
        nonlocal phase, emergency_deadline
        if phase != "emergency":
            print(f"\\nCUBE1 EMERGENCY: {reason}; RED blink until pit after second crossing.")
            phase = "emergency"
            indicator.set_mode("red_blink")
            # Оставляем время добраться до второго перекрёстка перед поиском ямы.
            emergency_deadline = now + CUBE1_DROP_TIMEOUT
    if force_emergency:
        enter_emergency("PB0 has no fresh readings", time.monotonic())
    ir_recovery_attempted = False
    try:
        while time.monotonic() < deadline or phase == "emergency":
            now = time.monotonic()
            if phase == "emergency" and now >= emergency_deadline:
                print("\\nCUBE1: emergency timeout before confirmed pit; STOP.")
                return None
            state = line_tracker.update(calibration)
            values = state.get("normalized", ())
            if not line_tracker.all_channels_fresh() or len(values) != 8:
                stop()
                print("\\nCUBE1: ADC stale; restarting Telemetrix.")
                if ir_recovery_attempted or recover_ir is None or not recover_ir():
                    print("\\nCUBE1: cannot recover ADC; STOP.")
                    return None
                ir_recovery_attempted = True
                # После перезапуска STM32 положение захвата неизвестно.
                # Переключаемся на аварийный режим, не заявляя, что кубик ещё в руке.
                cube_sensor = None
                enter_emergency("ADC reset; cube grip lost", time.monotonic())
                continue
            indicator.tick(now)
            left, right, candidate = detector.read(state)
            if not left and not right:
                clear_count += 1
            else:
                clear_count = 0
            if clear_count >= JUNCTION_CLEAR:
                junction_armed = True
            at_junction = junction_armed and candidate
            crossing_count = crossing_count + 1 if at_junction else 0
            if crossing_count >= JUNCTION_CONFIRM and not second_crossing_seen:
                second_crossing_seen = True
                post_cross_clear_count = 0
                print("\\nCUBE1: second crossing reached.")
                if phase == "search":
                    enter_emergency("cube NOT found before second crossing", now)
                elif phase == "carry":
                    phase = "past_junction"
                    missing_count = 0
                    indicator.set_mode("green")
                    print("\\nCUBE1: GREEN solid, keep moving until cube disappears.")

            # Не распознаём яму непосредственно на перекрёстке.
            # Сначала подтверждаем, что второй перекрёсток закончился,
            # и только затем ищем все белые или все чёрные датчики.
            if second_crossing_seen and not second_crossing_cleared:
                if (
                    not candidate and not left and not right
                    and not all(v >= CUBE1_PIT_ALL_BLACK_MIN for v in values)
                    and not all(v <= CUBE1_PIT_ALL_WHITE_MAX for v in values)
                ):
                    post_cross_clear_count += 1
                    if post_cross_clear_count >= JUNCTION_CLEAR:
                        second_crossing_cleared = True
                        if phase == "emergency":
                            emergency_deadline = now + CUBE1_EMERGENCY_TIMEOUT
                        pit_count = 0
                        pit_kind = None
                        print("\\nCUBE1: second crossing cleared; PIT detection armed.")
                else:
                    post_cross_clear_count = 0
            cube_visible = cube_sensor.detected() if cube_sensor is not None else None
            if cube_visible is None and phase != "emergency":
                if not ir_recovery_attempted and recover_ir is not None:
                    ir_recovery_attempted = True
                    print("\nCUBE1: IR reports stale; reconnecting STM32 once.")
                    if not recover_ir():
                        print("\nCUBE1: STM32 recovery failed; stopping safely.")
                        return None
                # После сброса STM32 рука возвращается домой, поэтому обычный
                # перенос кубика ненадёжен; используем аварийный поиск ямы.
                enter_emergency("IR input unavailable", now)
            if cube_visible is None:
                cube_visible = False
            if phase == "search" and not second_crossing_seen:
                if cube_visible:
                    present_count += 1
                    if present_count >= CUBE1_SENSOR_CONFIRM:
                        phase = "carry"
                        missing_count = 0
                        indicator.set_mode("green_blink")
                        print("\\nCUBE1: cube found; GREEN blinking.")
                else:
                    present_count = 0
            elif phase == "carry" and not second_crossing_seen:
                if cube_visible:
                    missing_count = 0
                else:
                    missing_count += 1
                    if missing_count >= CUBE1_DROP_CONFIRM_FRAMES:
                        enter_emergency("cube lost BEFORE second crossing", now)
            elif phase == "past_junction":
                if not cube_visible:
                    missing_count += 1
                    if missing_count >= CUBE1_DROP_CONFIRM_FRAMES:
                        print("\\nCUBE1: cube dropped AFTER second crossing.")
                        return (
                            "drop_line_lost"
                            if state.get("position") is None
                            else "drop"
                        )
                else:
                    missing_count = 0
            if phase == "emergency" and second_crossing_cleared:
                black = all(value >= CUBE1_PIT_ALL_BLACK_MIN for value in values)
                white = all(value <= CUBE1_PIT_ALL_WHITE_MAX for value in values)
                kind = "black" if black else ("white" if white else None)
                pit_count = pit_count + 1 if kind and kind == pit_kind else (1 if kind else 0)
                pit_kind = kind
                if pit_count >= CUBE1_PIT_CONFIRM_FRAMES:
                    print(f"\\nCUBE1: all 8 {kind} AFTER crossing; reversing to find line.")
                    return "emergency"
            if state.get("position") is None:
                # Подтверждённая потеря линии после второго перекрёстка
                # может означать успешный сброс. На время проверки останавливаемся,
                # чтобы не проехать вслепую за край трассы.
                if (
                    phase == "past_junction" and second_crossing_seen
                    and not candidate and not left and not right
                ):
                    line_loss_count += 1
                    stop()
                    if line_loss_count >= CUBE1_LINE_LOSS_CONFIRM:
                        print(
                            "\\nCUBE1: line lost after second crossing; "
                            "counting cube as dropped. Reverse to find line."
                        )
                        return "drop_line_lost"
                    time.sleep(LOOP_DELAY)
                    continue
                if phase != "emergency":
                    print("\\nCUBE1: line lost before drop; STOP.")
                    return None
                line_follower.drive_straight(
                    speed=MANEUVER_SPEED,
                    max_speed=MANEUVER_MAX_SPEED,
                )
            else:
                line_loss_count = 0
                differential_drive(state)
            if now - last_status >= 0.5:
                print(f"CUBE1 phase={phase} cube={int(cube_visible)} "
                      f"crossing_passed={int(second_crossing_cleared)}")
                last_status = now
            time.sleep(LOOP_DELAY)
        print("\\nCUBE1: search/transport timeout; STOP.")
        return None
    finally:
        stop()


def cube1_emergency_reverse_to_line(
    calibration, recover_connection=None,
):
    """Движется назад до обнаружения линии после аварии или сброса кубика."""
    seen = 0
    try:
        while True:
            if not line_tracker.all_channels_fresh():
                stop()
                if recover_connection is None or not recover_connection(
                    "emergency reverse", require_line=False
                ):
                    return False
                seen = 0
                continue
            state = line_tracker.update(calibration)
            if state.get("position") is not None:
                seen += 1
                if seen >= CUBE1_EMERGENCY_REVERSE_CONFIRM:
                    print("\nCUBE1: line recovered; reversing to starting junction.")
                    return True
            else:
                seen = 0
            line_follower.reverse_straight(CUBE1_EMERGENCY_REVERSE_SPEED)
            time.sleep(LOOP_DELAY)
    finally:
        stop()


def cube1_final_turn_approach():
    """После возвращения к перекрёстку едет 0,5 с на PWM 80 перед R90.
    Во время этого короткого откалиброванного проезда поиск края линии и PD не используются."""
    if not line_tracker.all_channels_fresh():
        print("\nCUBE1: ADC stale before final forward pulse; stopping.")
        stop()
        return False
    try:
        print(
            f"\nCUBE1: return junction found; forward "
            f"{CUBE1_RETURN_FORWARD_SECONDS:.3f}s PWM {CUBE1_RETURN_FORWARD_PWM}."
        )
        motors.set_speeds(CUBE1_RETURN_FORWARD_PWM, CUBE1_RETURN_FORWARD_PWM)
        time.sleep(CUBE1_RETURN_FORWARD_SECONDS)
        return True
    finally:
        stop()


def cube1_task(
    calibration, detector, bus, offset, board, board_state=None,
    recover_connection=None,
):
    """Ищет и переносит кубик с помощью оптического датчика."""
    from firefighter import Firefighter
    indicator = None
    stop()

    def recover_cube(stage, require_line=True):
        nonlocal board
        stop()
        try:
            if recover_connection is None:
                new_board = recover_stm32_connection(
                    board, calibration, require_line=require_line
                )
            else:
                new_board = recover_connection(
                    f"CUBE1 {stage}", require_line=require_line
                )
            if new_board is None:
                return None
            board = new_board
            if board_state is not None:
                board_state["board"] = board
            return board
        except Exception as exc:
            print(f"\\nCUBE1: reconnect failed at {stage}: {exc}")
            return None

    try:
        print("\nCUBE1: execute R90.")
        if not follow_for(
            calibration,
            TURN_90_FORWARD_TIME,
            speed=TURN_90_FORWARD_SPEED,
            max_speed=TURN_90_FORWARD_MAX_SPEED,
            recover_connection=recover_cube,
        ):
            return False
        ok, left_sign = pivot_90(
            bus, offset, "R", angle_target=TURN_90_ANGLE
        )
        if not ok or left_sign is None:
            print("CUBE1: initial turn/gyro check failed.")
            return False
        if not reacquire_line(
            calibration, recover_connection=recover_cube
        ):
            return False

        # Во время поворота рука остаётся сложенной;

        # захват опускается только после выравнивания по линии.
        if not cube1_arm_action(board):
            return False
        if not line_tracker.all_channels_fresh():
            if recover_cube("after arm positioning") is None:
                return False
            if not cube1_arm_action(board):
                return False
        cube_sensor = Firefighter(board)
        cube_sensor.initialize()
        sensor_ready = True
        # Это новая подписка: достаточно первого свежего значения PB0.
        # Ожидание последующего отчёта может привести к тайм-ауту,
        # если аналоговое значение не изменяется.
        try:
            cube_sensor.wait_for_sample()
        except TimeoutError:
            stop()
            print("\nCUBE1: PB0 reported no fresh ADC data; resubscribing once.")
            # Не подписываем PB0 повторно на той же связи: сначала
            # проверяем, появятся ли отчёты у существующей подписки.
            try:
                cube_sensor.wait_for_sample()
            except TimeoutError:
                stop()
                print(
                    "\nCUBE1: PB0 still has no reports after resubscription. "
                    "Check PB0 sensor wiring / Telemetrix connection; "
                    "motors remain stopped."
                )
                sensor_ready = False

        def recover_ir():
            return recover_cube("sensor or ADC failure") is not None
        indicator = Cube1Signal(cube1_create_indicator())
        status = cube1_transport(
            calibration, detector, cube_sensor if sensor_ready else None,
            indicator, force_emergency=not sensor_ready, recover_ir=recover_ir,
        )
        if status not in ("drop", "drop_line_lost", "emergency"):
            return False

        # Перед обратным ходом складываем руку — кубик уже не удерживается.
        if not cube1_fold_arm(board, reason=status):
            return False
        if status in ("emergency", "drop_line_lost"):
            if not cube1_emergency_reverse_to_line(
                calibration, recover_connection=recover_cube
            ):
                return False
        # Возвращаемся по линии к тому же перекрёстку.
        # Используем геометрический контроллер заднего хода и гироскоп.
        if not button_return(
            calibration, detector, bus, offset, left_sign,
            label="CUBE1", accept_s8_only=True,
            max_return_time=None, line_loss_timeout=None,
            recover_connection=recover_cube,
        ):
            return False
        if not line_tracker.all_channels_fresh():
            if recover_cube("before final R90") is None:
                return False
        if not cube1_final_turn_approach():
            return False
        print("\nCUBE1: final R90 using main_test.py's existing pivot.")
        ok, _ = pivot_90(
            bus, offset, "R", angle_target=TURN_90_ANGLE
        )
        return ok and reacquire_line(
            calibration, recover_connection=recover_cube
        )
    finally:
        stop()
        if indicator is not None:
            indicator.close()


def firefighter_reverse_until_line(calibration, recover_connection=None):
    """Аварийный медленный задний ход для восстановления линии при тушении."""
    stop()
    print(
        "\nF: line lost during flame search; emergency stop. "
        "Reversing slowly to find it."
    )
    confirmed = 0
    gray_reported = False
    deadline = time.monotonic() + FIRE_LINE_RECOVERY_TIMEOUT
    try:
        while time.monotonic() < deadline:
            if not line_tracker.all_channels_fresh():
                stop()
                print("F: ADC stale during emergency line search; reconnecting.")
                if recover_connection is None or not recover_connection("reverse line"):
                    return False
                deadline = time.monotonic() + FIRE_LINE_RECOVERY_TIMEOUT
                confirmed = 0
                continue
            state = line_tracker.update(calibration)
            values = state.get("normalized", [])
            raw = state.get("raw", [])
            strong_sensors = [
                index for index, value in enumerate(values)
                if value >= FIRE_LINE_BLACKNESS_MIN
            ]
            if calibration is not None and len(raw) == len(values) == 8:
                strong_sensors = [
                    index for index in strong_sensors
                    if abs(raw[index] - calibration["black"][index])
                    < abs(raw[index] - calibration["gray"][index])
                ]
                if not strong_sensors and not gray_reported and any(
                    abs(raw[index] - calibration["gray"][index])
                    < abs(raw[index] - calibration["black"][index])
                    and abs(raw[index] - calibration["gray"][index])
                    < abs(raw[index] - calibration["white"][index])
                    for index in range(8)
                ):
                    print("\nF: gray line passed while reversing.")
                    gray_reported = True
            black_line = (
                state.get("position") is not None
                and len(values) == 8
                and bool(strong_sensors)
            )
            if black_line:
                confirmed += 1
                if confirmed >= FIRE_LINE_RECOVERY_CONFIRM:
                    stop()
                    print(
                        "\nF: line found; switching to the normal "
                        "line-following return."
                    )
                    return True
            else:
                confirmed = 0
            line_follower.reverse_straight(FIRE_LINE_RECOVERY_SPEED)
            time.sleep(LOOP_DELAY)
        print("F: line recovery exceeded safe time; stopped.")
        return False
    finally:
        stop()


def firefighter_task(
    calibration, detector, bus, offset, board, camera_stream=None,
    recover_connection=None,
):
    """Тушит найденное пламя; при отсутствии сигнала тушит в конце линии."""
    from firefighter import (
        Firefighter,
        firefighting_pose,
    )
    from roboarm import get_shared_arm
    from arm_web import START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY
    from button_arm import _return_home
    fire = Firefighter(board)
    indicator = None
    stop()

    def reconnect_fire(stage, restage=False, require_line=True):
        nonlocal board, fire, arm
        if recover_connection is None:
            return False
        new_board = recover_connection(
            f"FIREFIGHTER {stage}", require_line=require_line
        )
        if new_board is None:
            return False
        board = new_board
        fire = Firefighter(board)
        fire.initialize()
        # Ссылку на руку обновляем на любом этапе, даже после тушения.
        arm, _ = get_shared_arm(board)
        if restage:
            arm.open_gripper()
            _return_home(arm, pose, AUTO_MAX_STEP_DEG, STEP_DELAY)
        return line_tracker.all_channels_fresh()

    try:
        pose = {**START_ANGLES, **firefighting_pose()}
        fire.initialize()
        try:
            indicator = create_robot_rgb_indicator()
        except Exception as exc:
            print(f"F: RGB unavailable ({exc}); continuing without indication.")
        # Перед входным поворотом сохраняем проверенный проезд 0,5 с.
        print("\nF: advance 0.5s along line before initial L90.")
        if not follow_for(
            calibration, TURN_90_FORWARD_TIME,
            speed=TURN_90_FORWARD_SPEED,
            max_speed=TURN_90_FORWARD_MAX_SPEED,
            recover_connection=lambda stage: reconnect_fire(stage),
        ):
            return False
        print("\nF: initial shared L90.")
        ok, left_sign = pivot_90(bus, offset, "L", angle_target=TURN_90_ANGLE)
        if not ok or left_sign is None:
            return False
        if not line_tracker.all_channels_fresh():
            if not reconnect_fire("after initial L90"):
                return False

        # До окончания позиционирования руки не движемся вперёд.
        stop()
        arm, newly_attached = get_shared_arm(board)
        if newly_attached:
            arm.write(**START_ANGLES)
        else:
            _return_home(arm, START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY)
        time.sleep(0.6)
        # Открываем захват перед позой тушения, чтобы
        # он не мешал струе воды.
        arm.open_gripper()
        _return_home(arm, pose, AUTO_MAX_STEP_DEG, STEP_DELAY)
        time.sleep(0.3)
        if not line_tracker.all_channels_fresh():
            if not reconnect_fire("after arm positioning", restage=True):
                return False

        # При подтверждении пламени тушим сразу; резерв — конец линии.
        deadline = time.monotonic() + FIRE_APPROACH_TIMEOUT
        line_end_fallback = False
        line_end_count = 0
        while time.monotonic() < deadline:
            if not line_tracker.all_channels_fresh():
                stop()
                remaining = max(0.0, deadline - time.monotonic())
                if not reconnect_fire("approach", restage=True):
                    print("\nF: reconnect failed during approach.")
                    return False
                deadline = time.monotonic() + remaining
                line_end_count = 0
                continue
            state = line_tracker.update(calibration)
            flame_detected = fire.detected()
            if flame_detected is True:
                stop()
                print("\nF: flame detected; GREEN indicator, water ON.")
                if indicator is not None:
                    indicator.green()
                if camera_stream is not None:
                    camera_stream.save_latest_task_frame("FIREFIGHTER: flame detected")
                fire.spray()
                try:
                    fire.wait_for_sample(after=time.monotonic())
                    if fire.detected() is True:
                        print("\nF: flame remains; second and final spray.")
                        fire.spray()
                except TimeoutError:
                    print("\nF: flame status unavailable after spray; no second spray.")
                break
            if state["position"] is None:
                stop()
                line_end_count += 1
                if line_end_count < FIRE_LINE_END_CONFIRM:
                    time.sleep(LOOP_DELAY)
                    continue
                line_end_fallback = True
                print("\nF: no flame found at end of line; RED indicator, water ON.")
                if indicator is not None:
                    indicator.red()
                if camera_stream is not None:
                    camera_stream.save_latest_task_frame(
                        "FIREFIGHTER: fallback spray at line end"
                    )
                fire.spray()
                if not firefighter_reverse_until_line(
                    calibration,
                    recover_connection=lambda stage: reconnect_fire(
                        stage, require_line=False
                    ),
                ):
                    return False
                break
            line_end_count = 0
            differential_drive(
                state, speed=FIRE_APPROACH_SPEED,
                max_speed=FIRE_APPROACH_MAX_SPEED,
            )
            time.sleep(LOOP_DELAY)
        else:
            print("\nF: flame approach timeout; stopped.")
            return False
        if line_end_fallback:
            print("\nF: fallback spray complete; returning along the line.")
        stop()
        _return_home(arm, START_ANGLES, AUTO_MAX_STEP_DEG, STEP_DELAY)
        if not button_return(
            calibration,
            detector,
            bus,
            offset,
            left_sign,
            label="FIREFIGHTER",
            accept_any_junction=True,
            line_loss_timeout=BUTTON_REVERSE_LINE_LOSS_TIMEOUT,
            recover_connection=lambda stage: reconnect_fire(stage),
        ):
            return False
        print("\nF: advance 0.5s along line before final R90.")
        if not follow_for(
            calibration, TURN_90_FORWARD_TIME,
            speed=TURN_90_FORWARD_SPEED,
            max_speed=TURN_90_FORWARD_MAX_SPEED,
            recover_connection=lambda stage: reconnect_fire(stage),
        ):
            return False
        print("\nF: final shared R90.")
        ok, _ = pivot_90(bus, offset, "R", angle_target=TURN_90_ANGLE)
        return ok and reacquire_line(
            calibration,
            recover_connection=lambda stage: reconnect_fire(stage),
        )
    finally:
        try:
            stop()
            fire.off()
        finally:
            if indicator is not None:
                try:
                    indicator.off()
                    indicator.close()
                except Exception as exc:
                    print(f"F: RGB shutdown error: {exc}")


def charging_task():
    """Сохраняет 11-секундную остановку зарядки даже при отказе RGB-индикатора.
    Ошибка индикации не прерывает маршрут. Моторы стоят до конца зарядки;
    последующий поворот по-прежнему проверяется."""
    stop()
    rgb = None
    try:
        try:
            rgb = create_robot_rgb_indicator()
        except Exception as exc:
            print(f"\nC: RGB unavailable ({exc}); continuing timed charge without LED.")
        start = time.monotonic()
        red_end = start + CHARGE_RED_SECONDS
        park_end = start + CHARGE_SECONDS

        def indicate(method):
            nonlocal rgb
            if rgb is None:
                return
            try:
                getattr(rgb, method)()
            except Exception as exc:
                print(f"\nC: RGB indicator failed ({exc}); charging timer continues.")
                try:
                    rgb.close()
                except Exception:
                    pass
                rgb = None
        print("\nC: charging pause; red blink for 9 seconds if LED available.")
        while time.monotonic() < red_end:
            indicate("red")
            time.sleep(max(0.0, min(
                CHARGE_BLINK_HALF_PERIOD, red_end - time.monotonic()
            )))
            indicate("off")
            time.sleep(max(0.0, min(
                CHARGE_BLINK_HALF_PERIOD, red_end - time.monotonic()
            )))
        print("\nC: remaining 2 seconds; green LED if available.")
        indicate("green")
        time.sleep(max(0.0, park_end - time.monotonic()))
        print("\nC: charging pause complete; proceeding to validated right turn.")
        return True
    finally:
        if rgb is not None:
            try:
                rgb.off()
                rgb.close()
            except Exception:
                pass
        stop()


def create_color_rgb_indicator():
    """Создаёт RGB-индикатор с измеренным обменом красного и синего каналов."""
    import led
    import robot_pin_config as pins
    rgb = led.RGBLED(pins.LED_BLUE, pins.LED_GREEN, pins.LED_RED)
    rgb.off()
    return rgb


def color_marker_task(calibration, board, bus, offset, camera_stream=None):
    """Задание COLOR с проверенным поворотом R90 из этого файла.
    После поворота рука занимает заданное положение, камера распознаёт цвет,
    а индикатор сохраняет его до следующего перекрёстка."""
    from color_marker_detector import run_color_marker_task
    from roboarm import get_shared_arm
    global COLOR_LED_HOLD
    stop()
    marker = None
    try:
        print("\nCOLOR: execute R90 using this file's existing turn logic.")
        if not follow_for(
            calibration, TURN_90_FORWARD_TIME,
            speed=TURN_90_FORWARD_SPEED,
            max_speed=TURN_90_FORWARD_MAX_SPEED,
        ):
            return False
        ok, _ = pivot_90(bus, offset, "R", angle_target=TURN_90_ANGLE)
        if not ok or not reacquire_line(calibration):
            return False
        arm, _ = get_shared_arm(board)
        print("COLOR: arm base=90 shoulder=160 elbow=180 wrist=0 gripper=0.")
        arm.write(**COLOR_ARM_ANGLES)
        if COLOR_LED_HOLD is not None:
            try:
                COLOR_LED_HOLD.off()
                COLOR_LED_HOLD.close()
            except Exception:
                pass
            COLOR_LED_HOLD = None
        COLOR_LED_HOLD = create_color_rgb_indicator()

        def light_confirmed_color(match):
            getattr(COLOR_LED_HOLD, match["color"])()
            print(
                f"COLOR: {match['color'].upper()} indicator ON; "
                f"holding for {COLOR_MARKER_HOLD_SECONDS:g}s."
            )
        marker = run_color_marker_task(
            timeout_s=COLOR_MARKER_TIMEOUT,
            hold_seconds=COLOR_MARKER_HOLD_SECONDS,
            min_object_area=COLOR_MARKER_MIN_AREA,
            stable_frames=COLOR_MARKER_STABLE_FRAMES,
            control_led=False,
            confirmation_callback=light_confirmed_color,
            preview_port=None if camera_stream is not None else 8000,
            camera_stream=camera_stream,
        )
        if marker is None:
            print("COLOR: marker not found within 10 seconds; continue route.")
        return True
    except Exception as exc:
        print(f"\nCOLOR: camera/LED error: {exc}")
        return False
    finally:
        if marker is None and COLOR_LED_HOLD is not None:
            try:
                COLOR_LED_HOLD.off()
                COLOR_LED_HOLD.close()
            except Exception:
                pass
            COLOR_LED_HOLD = None
        stop()


def button_bounce_prepare_arm(board):
    """Сначала BASE 180, затем SHOULDER 130; захват закрыт."""
    from roboarm import get_shared_arm
    stop()
    arm, _ = get_shared_arm(board)
    arm.write(gripper=180)
    arm.write(base=180)
    time.sleep(0.3)
    arm.write(shoulder=130)
    time.sleep(0.3)
    return arm


def button_bounce_press_and_home(board):
    """Поднять плечо, нажать по IK с автоматической кистью, сложить руку."""
    import arm_web
    from button_arm import _execute_plan, _return_home
    from roboarm import get_shared_arm
    stop()
    arm, _ = get_shared_arm(board)
    arm.write(gripper=180)
    arm.write(shoulder=180)
    time.sleep(0.3)
    arm_web.arm = arm
    x, y, z = BUTTON_BOUNCE_PRESS
    path, pitch = arm_web.plan_auto_path({
        "x": x, "y": y, "z": z, "orientation_mode": "auto",
    })
    print(f"BUTTON_BOUNCE: press ({x:g}, {y:g}, {z:g}), auto wrist {pitch:.1f}.")
    _execute_plan(arm, path, arm_web.STEP_DELAY)
    arm.write(gripper=180)
    _return_home(
        arm, arm_web.START_ANGLES, arm_web.AUTO_MAX_STEP_DEG,
        arm_web.STEP_DELAY,
    )
    return True


def button_bounce_find_color(calibration, camera_stream, recover=None):
    """Следовать по линии с PWM 130/255 до цветного объекта в нижней центральной зоне."""
    import cv2
    from color_marker_detector import ColorMarkerDetector
    if camera_stream is None:
        print("BUTTON_BOUNCE: camera unavailable.")
        return False
    camera = camera_stream
    detector = ColorMarkerDetector(
        min_object_area=BUTTON_BOUNCE_MIN_OBJECT_AREA,
        stable_frames=BUTTON_BOUNCE_CONFIRM_FRAMES,
        allowed_colors={"red", "green", "blue"},
    )
    sequence = camera.sequence
    locked_color = None
    centered_frames = 0
    last_frame_at = time.monotonic()
    line_follower.reset_steering()
    camera.start_task("BUTTON_BOUNCE: searching red/green/blue")
    try:
        while True:
            if not line_tracker.all_channels_fresh():
                stop()
                if recover is None or not recover():
                    return False
                sequence = camera.sequence
                last_frame_at = time.monotonic()
                detector.reset()
                locked_color = None
                centered_frames = 0
                continue
            state = line_tracker.update(calibration)
            if state["position"] is None:
                stop()
                print("\\nBUTTON_BOUNCE: line lost.")
                return False
            captured = camera.get_frame(after_sequence=sequence, timeout=0.01)
            if captured is not None:
                sequence, frame = captured
                last_frame_at = time.monotonic()
                objects, annotated, confirmed_color = detector.process_frame(frame)
                if locked_color is None and confirmed_color in ("red", "green", "blue"):
                    locked_color = confirmed_color
                    print(f"\\nBUTTON_BOUNCE: locked {locked_color}.")
                height, width = frame.shape[:2]
                x1 = int(width * BUTTON_BOUNCE_TARGET_X[0])
                x2 = int(width * BUTTON_BOUNCE_TARGET_X[1])
                y1 = int(height * BUTTON_BOUNCE_TARGET_Y[0])
                y2 = int(height * BUTTON_BOUNCE_TARGET_Y[1])
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 255, 255), 2)
                target = next((
                    obj for obj in objects
                    if obj["color"] == locked_color
                    and obj["x"] < x2
                    and obj["x"] + obj["w"] > x1
                    and obj["y"] < y2
                    and obj["y"] + obj["h"] > y1
                ), None)
                centered_frames = 1 if target else 0
                camera.publish_task_frame(
                    annotated,
                    f"BUTTON_BOUNCE: {locked_color or 'searching'} "
                    f"target {centered_frames}/{BUTTON_BOUNCE_CONFIRM_FRAMES}",
                )
                if target is not None:
                    stop()
                    print(f"\\nBUTTON_BOUNCE: {locked_color} in lower center.")
                    return True
            elif time.monotonic() - last_frame_at > 0.5:
                stop()
                print("\\nBUTTON_BOUNCE: camera frames stopped.")
                return False
            differential_drive(
                state, speed=BUTTON_BOUNCE_DRIVE_PWM, max_speed=MAX_SPEED
            )
            time.sleep(LOOP_DELAY)
    finally:
        stop()
        line_follower.reset_steering()
        camera.end_task()


def button_bounce_task(
    calibration, detector, bus, offset, board, camera_stream=None,
    recover_connection=None,
):
    """R, подготовка руки, цвет в нижней части кадра, нажатие, возврат, L90."""
    stop()
    try:
        print("\\nBUTTON_BOUNCE: ordinary R.")
        ok, turn_sign = pivot(bus, offset, "R")
        if not ok or turn_sign is None or not reacquire_line(
            calibration, recover_connection=recover_connection
        ):
            return False
        stop()
        button_bounce_prepare_arm(board)
        if not button_bounce_find_color(
            calibration, camera_stream,
            recover=(lambda: recover_connection("BUTTON_BOUNCE color search")
                     is not None) if recover_connection else None,
        ):
            return False
        if not button_bounce_press_and_home(board):
            return False
        if not button_return(
            calibration, detector, bus, offset, turn_sign,
            label="BUTTON_BOUNCE",
            line_loss_timeout=BUTTON_BOUNCE_REVERSE_LINE_TIMEOUT,
            recover_connection=recover_connection,
        ):
            return False
        if not follow_for(
            calibration, RETURN_FORWARD_TIME,
            recover_connection=recover_connection,
        ):
            return False
        print("\\nBUTTON_BOUNCE: final L90.")
        ok, _ = pivot_90(bus, offset, "L", angle_target=TURN_90_ANGLE)
        return ok and reacquire_line(
            calibration, recover_connection=recover_connection
        )
    except Exception as exc:
        print(f"\\nBUTTON_BOUNCE: {exc}")
        return False
    finally:
        stop()


def reverse_for_short_turn(seconds, speed):
    """Короткий задний ход после медленного поворота, как в main.py."""
    deadline = time.monotonic() + seconds
    try:
        while time.monotonic() < deadline:
            line_follower.reverse_straight(speed)
            time.sleep(LOOP_DELAY)
    finally:
        stop()
    return True


def load_route():
    """Проверяет маршрут ROUTE, заданный пользователем."""
    if not ROUTE:
        raise ValueError(
            "ROUTE is empty. Add your junction commands to the ROUTE list "
            "in main_test.py before starting main_test.py."
        )
    route = []
    for index, command in enumerate(ROUTE, 1):
        if not isinstance(command, str):
            raise ValueError(f"ROUTE entry {index} is not a string.")
        cmd = command.strip().upper()
        if cmd not in ALLOWED_ROUTE_COMMANDS:
            raise ValueError(
                f"Invalid ROUTE entry #{index}: {command!r}. "
                "Allowed: L, R, L90, R90, L90SLOW, R90SLOW, S, SLOW, "
                "BUTTON, CHARGING, FIREFIGHTER, CUBE1, COLOR, BUTTON_BOUNCE, Q."
            )
        if cmd == "Q" and index != len(ROUTE):
            raise ValueError("Q must be the last ROUTE entry.")
        route.append(cmd)
    return route


def recover_stm32_connection(old_board, calibration, require_line=True):
    """Повторно подключает STM32 и заново подписывается на ADC после потери данных.
    Вызывается только после остановки моторов. Если восстановление не удалось,
    движение прекращается; старое подключение не используется."""
    try:
        stop()
    except Exception:
        pass
    line_tracker.shutdown()
    try:
        old_board.shutdown()
    except Exception as exc:
        print(f"STM32 RECOVERY: old connection close error: {exc}")
    reset_stm32()
    time.sleep(3.0)  # Пауза для загрузки STM32 после сброса.
    board = None
    try:
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        # При восстановлении сохраняем порядок подключения из differential2.py.
        line_tracker.initialize(board)
        if not line_tracker.wait_for_callbacks(timeout=6.0):
            raise RuntimeError("Not all eight ADC callbacks returned")
        deadline = time.monotonic() + 2.0
        while not line_tracker.all_channels_fresh():
            if time.monotonic() >= deadline:
                raise RuntimeError("ADC remained stale after reconnect")
            time.sleep(LOOP_DELAY)
        if require_line and line_tracker.update(calibration).get("position") is None:
            raise RuntimeError("No track line after reconnect; stop safely")
        motors.initialize(board)
        motors.motor_stop()
        # Сбрасываем общие объекты сервоприводов STM32 перед подключением.
        import roboarm
        roboarm._shared_board = None
        roboarm._shared_arm = None
        activate_arm_home(board)
        motors.motor_stop()
        print("STM32 RECOVERY: all eight ADC channels fresh; outputs restored.")
        return board
    except Exception:
        try:
            motors.motor_stop()
        except Exception:
            pass
        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass
        raise


def main():
    global COLOR_LED_HOLD
    board = bus = sensor = None
    camera_stream = None
    offset_90 = None
    motors_on = tracker_on = False
    adc_stale_since = None
    stm32_recovery_count = 0
    STM32_RECOVERY_LIMIT = 2
    try:
        route = load_route()
        print(f"MAIN_TEST BUILD: {PROGRAM_BUILD}")
        print(f"Automatic ROUTE loaded: {route}")
        print(
            f"Resetting STM32 through Raspberry Pi BCM{STM32_RESET_GPIO}..."
        )
        reset_stm32()

        # Сначала подключаем все ADC, как в проверенных ADC.py и differential2.py.
        board = telemetrix.Telemetrix(com_port=SERIAL_PORT)
        line_tracker.initialize(board)
        tracker_on = True
        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("No calibration found; run python calibrate.py.")
            return
        print("Waiting for all eight ADC callbacks...")
        if not line_tracker.wait_for_callbacks(timeout=6.0):
            received = line_tracker.callback_status()
            missing = [i + 1 for i, ready in enumerate(received) if not ready]
            startup_state = line_tracker.update(calibration)
            print(
                "ERROR: ADC callbacks missing for "
                f"S={missing}; motors remain stopped."
            )
            print(f"Latest RAW values: {startup_state['raw']}")
            return
        startup_state = line_tracker.update(calibration)
        print(f"All eight ADC channels ready: {startup_state['raw']}")
        print(f"Initial line position: {startup_state['position']}")
        if not line_tracker.all_channels_fresh():
            print("ERROR: ADC samples are stale after startup; motors not enabled.")
            return
        # Только после свежих данных всех 8 каналов включаем выходы моторов и руку.
        motors.initialize(board)
        motors_on = True
        stop()
        activate_arm_home(board)

        # Одна камера для всех заданий, без повторного запуска Picamera2.
        try:
            from color_marker_detector import ContinuousCamera
            camera_stream = ContinuousCamera(port=8000)
            camera_stream.start()
        except Exception as exc:
            print(f"CAMERA: unavailable ({exc}).")
            if camera_stream is not None:
                camera_stream.stop()
            camera_stream = None
        print("Starting route; line tracker will self-center the robot.")
        detector = JunctionDetector()

        def recover_during_task(stage, require_line=True):
            nonlocal board, offset_90, stm32_recovery_count, adc_stale_since
            stop()
            if stm32_recovery_count >= STM32_RECOVERY_LIMIT:
                print(f"\nSTM32 RECOVERY: limit reached at {stage}; stopping.")
                return None
            stm32_recovery_count += 1
            print(
                f"\nSTM32 RECOVERY: {stage}; restarting Telemetrix "
                f"({stm32_recovery_count}/{STM32_RECOVERY_LIMIT})."
            )
            try:
                new_board = recover_stm32_connection(
                    board, calibration, require_line=require_line
                )
            except Exception as exc:
                print(f"STM32 RECOVERY: failed at {stage}: {exc}")
                return None
            board = new_board
            offset_90 = None
            adc_stale_since = None
            # У старого экземпляра определителя могли остаться сработавшие S1/S8.
            detector.left_black = False
            detector.right_black = False
            # Сам ROUTE и выполненный поворот не сбрасываем.
            print(f"STM32 RECOVERY: resumed task at {stage}.")
            return board

        print(
            f"Differential2 ready: target {TARGET_POSITION}, "
            f"base PWM {BASE_SPEED}, max PWM {MAX_SPEED}. "
            "Automatic route active."
        )
        route_index = 0
        junction_count = 0
        candidate_count = 0
        junction_gate = JunctionRearmGate()
        junction_ignore_until = 0.0
        acceleration_started = None
        slow_until_next_junction = False
        straight_speed = {
            "current_speed": BASE_SPEED,
            "previous_error": None,
            "previous_time": None,
            "straight_since": None,
        }
        while True:

            # Не используем устаревшие данные ADC для управления моторами.
            state = line_tracker.update(calibration)
            if not line_tracker.all_channels_fresh():
                stop()
                if adc_stale_since is None:
                    adc_stale_since = time.monotonic()
                    print("\nADC stale; motors stopped pending recovery.")
                if time.monotonic() - adc_stale_since >= 0.75:
                    if stm32_recovery_count >= STM32_RECOVERY_LIMIT:
                        print("\nADC stale after 2 STM32 recovery attempts.")
                        break
                    stm32_recovery_count += 1
                    try:
                        board = recover_stm32_connection(board, calibration)
                    except Exception as exc:
                        print(f"STM32 recovery failed: {exc}")
                        break
                    offset_90 = None
                    detector = JunctionDetector()
                    junction_gate = JunctionRearmGate()
                    candidate_count = 0
                    adc_stale_since = None
                time.sleep(LOOP_DELAY)
                continue
            adc_stale_since = None
            position = state["position"]
            left, right, candidate = detector.read(state)
            now = time.monotonic()
            junction_gate.update(left, right, position)
            if (now >= junction_ignore_until
                    and junction_gate.armed and candidate):
                candidate_count += 1
            else:
                candidate_count = 0
            if candidate_count >= JUNCTION_CONFIRM:
                stop()
                slow_until_next_junction = False
                junction_gate.mark_junction()
                candidate_count = 0
                junction_count += 1

                # Подтверждённый цвет COLOR горит до следующего перекрёстка.
                if COLOR_LED_HOLD is not None:
                    try:
                        COLOR_LED_HOLD.off()
                        COLOR_LED_HOLD.close()
                    except Exception:
                        pass
                    COLOR_LED_HOLD = None
                    print("\nCOLOR: next junction reached; indicator OFF.")
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
                if cmd == "COLOR":
                    if bus is None:
                        bus = smbus.SMBus(I2C_BUS)
                        gyro_setup(bus)
                    if offset_90 is None:
                        offset_90 = gyro_calibrate(bus, settle_seconds=0.05)
                    ok = color_marker_task(calibration, board, bus, offset_90, camera_stream)
                elif cmd == "SLOW":
                    # Как в main.py: не задерживаем обнаружение следующего перекрёстка.
                    ok = True
                elif cmd in (
                    "L", "R", "L90", "R90", "L90SLOW", "R90SLOW", "BUTTON", "BUTTON_BOUNCE", "CHARGING",
                    "FIREFIGHTER", "CUBE1",
                ):

                    # Гироскоп включаем только при необходимости поворота.
                    if bus is None:
                        bus = smbus.SMBus(I2C_BUS)
                        gyro_setup(bus)
                    if cmd in ("L90", "R90", "L90SLOW", "R90SLOW"):
                        if offset_90 is None:
                            offset_90 = gyro_calibrate(bus, settle_seconds=0.05)
                        offset = offset_90
                    else:
                        offset = gyro_calibrate(bus)
                    if cmd == "BUTTON":

                        # BUTTON едет 2 секунды по линии, без датчика расстояния.
                        ok = button_task(
                            calibration, detector, sensor, bus, offset, board,
                            recover_connection=recover_during_task,
                        )
                    elif cmd == "BUTTON_BOUNCE":
                        ok = button_bounce_task(
                            calibration, detector, bus, offset, board, camera_stream,
                            recover_connection=recover_during_task,
                        )
                    elif cmd == "FIREFIGHTER":
                        ok = firefighter_task(
                            calibration, detector, bus, offset, board, camera_stream,
                            recover_connection=recover_during_task,
                        )
                    elif cmd == "CUBE1":
                        board_state = {"board": board}
                        ok = cube1_task(
                            calibration, detector, bus, offset, board,
                            board_state=board_state,
                            recover_connection=recover_during_task,
                        )
                        board = board_state["board"]
                    elif cmd == "CHARGING":
                        ok = charging_task()
                        if ok:

                            # Тот же поворот вправо и последующее центрирование, что в старом алгоритме.
                            ok, _ = pivot(bus, offset, "R")
                            ok = ok and reacquire_line(calibration)
                    elif cmd in ("L90", "R90", "L90SLOW", "R90SLOW"):
                        ok = follow_for(
                            calibration, TURN_90_FORWARD_TIME,
                            speed=TURN_90_FORWARD_SPEED,
                            max_speed=TURN_90_FORWARD_MAX_SPEED,
                            recover_connection=recover_during_task,
                        )
                        if ok:
                            ok, _ = pivot_90(
                                bus,
                                offset,
                                cmd[0],
                                angle_target=TURN_90_ANGLE,
                                slow=False,
                            )
                        ok = ok and reacquire_line(
                            calibration, recover_connection=recover_during_task
                        )
                    else:
                        # Обычные L/R выполняются без проезда вперёд.
                        ok, _ = pivot(bus, offset, cmd)
                        ok = ok and reacquire_line(
                            calibration, recover_connection=recover_during_task
                        )
                else:
                    ok = follow_for(calibration, 0.5)
                if not ok:
                    print("\nManeuver failed. Robot stopped.")
                    break
                slow_until_next_junction = cmd in ("SLOW", "L90SLOW", "R90SLOW")
                acceleration_started = None if slow_until_next_junction else time.monotonic()
                straight_speed["current_speed"] = MANEUVER_SPEED
                straight_speed["previous_error"] = None
                straight_speed["previous_time"] = None
                straight_speed["straight_since"] = None
                if cmd in (
                    "L", "R", "L90", "R90", "L90SLOW", "R90SLOW",
                    "BUTTON", "CHARGING", "FIREFIGHTER", "CUBE1", "COLOR", "BUTTON_BOUNCE",
                ):
                    junction_ignore_until = (
                        time.monotonic() + POST_TURN_JUNCTION_IGNORE_SECONDS
                    )
                    candidate_count = 0
                if slow_until_next_junction:
                    # Без блокировки обнаружения после SLOW-поворота.
                    junction_ignore_until = 0.0
                    junction_gate = JunctionRearmGate()
                    candidate_count = 0
                if route_index >= len(route):
                    stop()
                    print("\nAll ROUTE commands completed. Robot stopped.")
                    break
                continue

            # Только проверенный differential2.py управляет движением по линии.
            if position is None:
                stop()
                if acceleration_started is not None:
                    acceleration_started = time.monotonic()
                print(
                    '\rNO LINE: paused; route resumes when line returns. '
                    f'raw={state["raw"]}',
                    end="", flush=True,
                )
            else:
                if slow_until_next_junction:
                    differential_drive(state, speed=150, max_speed=150)
                elif acceleration_started is None:
                    speed, max_speed = update_straight_speed(
                        state, left, right, candidate,
                        straight_speed, time.monotonic(),
                    )
                    differential_drive(state, speed=speed, max_speed=max_speed)
                else:
                    elapsed = time.monotonic() - acceleration_started
                    speed, max_speed = post_junction_drive_limits(elapsed)
                    differential_drive(state, speed=speed, max_speed=max_speed)
                    if elapsed >= POST_JUNCTION_ACCEL_SECONDS:
                        acceleration_started = None
                        straight_speed["current_speed"] = speed
                        straight_speed["previous_error"] = None
                        straight_speed["previous_time"] = None
                        straight_speed["straight_since"] = None
                print(
                    f"\rLINE pos={position:+.3f} "
                    f"S1={int(left)} S8={int(right)} "
                    "N=["
                    + " ".join(
                        f"{value:.2f}" for value in state["normalized"]
                    )
                    + "]",
                    end="", flush=True,
                )
            time.sleep(LOOP_DELAY)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as exc:
        print(f"\nERROR: {exc}")
    finally:
        if camera_stream is not None:
            try:
                camera_stream.stop()
            except Exception:
                pass
        if COLOR_LED_HOLD is not None:
            try:
                COLOR_LED_HOLD.off()
                COLOR_LED_HOLD.close()
            except Exception:
                pass
            COLOR_LED_HOLD = None
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
            except (Exception, KeyboardInterrupt):
                pass
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass
        print("\nRobot stopped.")
if __name__ == "__main__":
    main()
