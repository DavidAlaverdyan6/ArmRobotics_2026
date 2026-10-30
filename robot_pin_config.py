# robot_pins.py


from pins import (
    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8,
    PA12, PB0, PB5, PB6, PB7, PB8, PB9, PB12, PB13, PB14, PB15,
    PC13, PC15,
)

# ── Line tracker: 8-channel analog sensor array ──
line_l1 = PA0
line_l2 = PA1
line_l3 = PA2
line_l4 = PA3

line_r1 = PA4
line_r2 = PA5
line_r3 = PA6
line_r4 = PA7

LINE_SENSORS = (
    line_l1, line_l2, line_l3, line_l4,
    line_r1, line_r2, line_r3, line_r4,
)

# ── Motor driver ──
# Provisional mapping: reconcile with the actual wiring.
motorl1 = PB6
motorl2 = PB7
motorr1 = PB9
motorr2 = PB8

LEFT_MOTOR = (motorl1, motorl2)
RIGHT_MOTOR = (motorr1, motorr2)

# ── Servos ──
servopan = PB12
servotilt = PB13
servo3 = PB14
servo4 = PB15
servo_gripper = PA8

SERVOS = (servopan, servotilt, servo3, servo4, servo_gripper)

# ── Other hardware ──
fire_sensor = PB0
pump = PA12

LED_RED = 17
LED_GREEN = 27
LED_BLUE = 22