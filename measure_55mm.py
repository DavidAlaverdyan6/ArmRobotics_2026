#!/usr/bin/env python3
"""Three identical measured forward pulses, with repeatable PWM/timing profile.

Run twice with different commanded durations at the same PWM to fit a pulse model.
Measurements are recorded; only two distinct durations produce a profile.
"""
import json
import math
import statistics
import time
from pathlib import Path

TRIALS = 3
TARGET_MM = 55.0
MIN_PWM = 70
MAX_PWM = 140
MAX_SECONDS = 2.0
MEASUREMENTS = Path(__file__).with_name('motion_measurements.json')
PROFILE = Path(__file__).with_name('motion_calibration.json')


def number(prompt, *, low=None, high=None, integer=False):
    while True:
        try:
            value = float(input(prompt).strip().replace(',', '.'))
        except ValueError:
            print('Введите число.')
            continue
        if not math.isfinite(value) or (low is not None and value < low) or (high is not None and value > high) or (integer and not value.is_integer()):
            print('Некорректное значение.')
            continue
        return int(value) if integer else value


def fit_profiles(records, pwm):
    """Line fit d = velocity*t + intercept from averages at distinct times."""
    rows = [(float(r['seconds']), statistics.mean(r['distances_mm'])) for r in records if r['pwm'] == pwm and len(r['distances_mm']) == TRIALS]
    xs = [x for x, y in rows]
    ys = [y for x, y in rows]
    if len(set(xs)) < 2:
        return None
    mean_x, mean_y = statistics.mean(xs), statistics.mean(ys)
    variance = sum((x - mean_x)**2 for x in xs)
    slope = sum((x-mean_x)*(y-mean_y) for x,y in rows)/variance
    intercept = mean_y-slope*mean_x
    if not (math.isfinite(slope) and math.isfinite(intercept) and slope > 0):
        return None
    time_for_55 = (TARGET_MM-intercept)/slope
    if not 0 < time_for_55 <= MAX_SECONDS:
        return None
    return slope, intercept, time_for_55


def write_json(path, data):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def main():
    print('Три одинаковых заезда с измерением полного пути после остановки.')
    seconds = number('Время движения в секундах (0–2): ', low=.001, high=MAX_SECONDS)
    pwm = number('PWM для обоих моторов (70–140): ', low=MIN_PWM, high=MAX_PWM, integer=True)
    from telemetrix import telemetrix
    import motors
    from stm32_reset import reset_stm32
    board = None
    measured = []
    try:
        reset_stm32()
        board = telemetrix.Telemetrix(com_port='/dev/serial0')
        motors.initialize(board)
        motors.motor_stop()
        for trial in range(1, TRIALS+1):
            input(f'\nПопытка {trial}/3. Верните ОСЬ на стартовую отметку; освободите путь; Enter — поехать: ')
            start = time.monotonic()
            try:
                motors.set_speeds(pwm, pwm)
                remaining = seconds-(time.monotonic()-start)
                if remaining > 0:
                    time.sleep(remaining)
            finally:
                motors.motor_stop()
            print('Дождитесь полной физической остановки, прежде чем измерять.')
            measured.append(number('Сколько проехал (мм)? ', low=0))
    finally:
        if board is not None:
            try:
                motors.motor_stop()
            finally:
                board.shutdown()

    print(f'\nРасстояния: {measured}')
    print(f'Среднее: {statistics.mean(measured):.2f} мм; разброс: {max(measured)-min(measured):.2f} мм')
    records = json.loads(MEASUREMENTS.read_text(encoding='utf-8')) if MEASUREMENTS.exists() else []
    records.append({'pwm':pwm,'seconds':seconds,'distances_mm':measured})
    write_json(MEASUREMENTS, records)
    fit = fit_profiles(records, pwm)
    if not fit:
        print('Результаты сохранены. Для калибровки повторите программу с ДРУГИМ временем, тем же PWM.')
        print('Автоматический поворот не изменён.')
        return
    speed, intercept, predicted_time = fit
    print(f'Модель: путь = {speed:.3f} * время {intercept:+.3f} мм')
    print(f'Расчёт времени для 55 мм: {predicted_time:.4f} с')
    print('Это только предварительная модель: измерьте отдельный контрольный заезд на 55 мм.')
    if pwm not in (80,100,120,140):
        print('Профиль L90/R90 не сохранён: поддерживаются PWM 80, 100, 120, 140.')
        return
    approve = input('Сохранить ПРЕДВАРИТЕЛЬНЫЙ профиль для этого PWM? [да/нет]: ').strip().lower()
    if approve not in ('да','д','yes','y'):
        print('Профиль не менялся.')
        return
    profile = json.loads(PROFILE.read_text(encoding='utf-8')) if PROFILE.exists() else {}
    for key,value in (('forward_mm_s',speed),('stop_mm',max(0.0,intercept)),('launch_deficit_mm',max(0.0,-intercept))):
        profile.setdefault(key,{})[str(pwm)] = round(value,6)
    write_json(PROFILE, profile)
    print('Сохранено:',PROFILE)
    print('ВАЖНО: подтвердите 55 мм реальным проездом перед запуском автоматической трассы.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nПрерывание; движение остановлено в блоке finally.')
