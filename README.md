# ArmRobotics 2026

Исходники робота: https://github.com/DavidAlaverdyan6/ArmRobotics_2026

## Окружение Raspberry Pi OS / Python 3.11

```bash
sudo apt update
sudo apt install --no-install-recommends python3-venv python3-dev build-essential python3-picamera2 python3-libcamera python3-rpi-lgpio python3-pyqt5 python3-opengl i2c-tools
cd ~/ArmRobotics
python3 -m venv --system-site-packages venv
source venv/bin/activate
python -m pip install -r requirements.txt
```

`--system-site-packages` открывает доступ к системным Picamera2, libcamera,
Qt и GPIO. `board` и `busio` устанавливаются вместе с Adafruit Blinka;
отдельный пакет с именем `board` не нужен. NumPy ограничен версией 1.x
для совместимости с системными расширениями камеры.

Для запуска скриптов из вложенных папок зарегистрируйте корень проекта:

```bash
python -c 'from pathlib import Path; import sysconfig; (Path(sysconfig.get_path("purelib")) / "armrobotics.pth").write_text(str(Path.cwd()) + "\n")'
```

В подготовленном здесь `venv` библиотеки и этот путь уже установлены.

## Интерфейсы нового Raspberry Pi

Запустите на самом Raspberry Pi, где доступна запись в `/boot/firmware`:

```bash
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_serial_hw 0
```

После изменения загрузочных настроек перезагрузите Raspberry Pi.
Команды включают I2C и UART и отключают системную консоль на UART,
используемом роботом. Эти настройки уже применены. I2C доступен как `/dev/i2c-1`; для появления
`/dev/serial0` требуется перезагрузка. Резервные копии прежних настроек:
`/boot/firmware/config.txt.armrobotics.bak` и
`/boot/firmware/cmdline.txt.armrobotics.bak`.

Пользователь должен входить в группы `dialout`, `i2c`, `gpio`, `video`,
`render`. У `admin` эти группы уже есть. Камера OV5647 обнаруживается libcamera.
После нового входа проверьте `id` и наличие `/dev/serial0`, `/dev/i2c-1`.
Графические примеры камеры требуют рабочего стола.

## Запуск существующих скриптов

```bash
cd ~/ArmRobotics
source venv/bin/activate
python main_ggs_interactive.py
```

Для веб-интерфейса манипулятора: `python arm_web.py`, затем откройте
`http://<адрес-Raspberry-Pi>:8080`. Запускайте один сценарий управления
контроллером одновременно. Сценарии могут сразу двигать моторы и сервоприводы.

Код ожидает калибровку в `~/ArmRobotics/linetracker/calibration.json`.
Прошивка контроллера находится в `Telemetrix4Arduino-master/`.
Установка Python-библиотеки Telemetrix не прошивает STM32.

## Особые файлы старых экспериментов

В текущей версии репозитория отсутствуют:

- специальный драйвер `VL53L0X` с транспортом Telemetrix для
  `Test_scripts/stmvl53_test.py`; соседний `vl53l0x.py` — отдельный
  диагностический скрипт для I2C Raspberry Pi;
- `/usr/share/libcamera/ipa/rpi/vc4/ov5647_dw9714.json`, используемый
  `Test_scripts/camera_af.py`. Нужен профиль от поставщика камеры/объектива.

Основной сценарий использует установленный драйвер Adafruit VL53L0X.
Исходные Python-скрипты оставлены без изменений.
