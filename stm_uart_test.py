from telemetrix import telemetrix

board = telemetrix.Telemetrix(
    com_port="/dev/serial0"
)

print("STM32 connected!")

board.shutdown()