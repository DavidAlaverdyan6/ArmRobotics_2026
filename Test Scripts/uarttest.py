from telemetrix import telemetrix

board = telemetrix.Telemetrix()


print("STM32 connected!")

board.shutdown()