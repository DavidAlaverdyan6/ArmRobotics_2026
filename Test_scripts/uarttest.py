from telemetrix import telemetrix
from stm32_reset import reset_stm32

reset_stm32()
board = telemetrix.Telemetrix()


print("STM32 connected!")

board.shutdown()
