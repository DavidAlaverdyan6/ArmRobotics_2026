import smbus
import time

bus = smbus.SMBus(1)
MPU6050 = 0x68

# Выводим MPU-6050 из сна
bus.write_byte_data(MPU6050, 0x6B, 0)


def read_word(reg):
    high = bus.read_byte_data(MPU6050, reg)
    low = bus.read_byte_data(MPU6050, reg + 1)

    value = (high << 8) | low

    if value >= 32768:
        value -= 65536

    return value


# -----------------------------
# КАЛИБРОВКА
# -----------------------------

print("Не двигайте MPU-6050...")

samples = 500

gx_offset = 0
gy_offset = 0
gz_offset = 0

for i in range(samples):
    gx_offset += read_word(0x43)
    gy_offset += read_word(0x45)
    gz_offset += read_word(0x47)

    time.sleep(0.002)

gx_offset /= samples
gy_offset /= samples
gz_offset /= samples

print("Калибровка закончена.")
print("Можно двигать датчик.")

# Начальные углы
angle_x = 0.0
angle_y = 0.0
angle_z = 0.0

last_time = time.time()


while True:

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Считываем гироскоп
    gx = (read_word(0x43) - gx_offset) / 131.0
    gy = (read_word(0x45) - gy_offset) / 131.0
    gz = (read_word(0x47) - gz_offset) / 131.0

    # Интегрируем скорость вращения
    angle_x += gx * dt
    angle_y += gy * dt
    angle_z += gz * dt

    print(
        f"X: {angle_x:7.2f}° | "
        f"Y: {angle_y:7.2f}° | "
        f"Z: {angle_z:7.2f}°"
    )

    time.sleep(0.01)
