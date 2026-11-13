# CUBE1 state and recoverable STM32 dropout (2026-11-13)

Main entry point: `python3 main.py`.

## CUBE1

- At the initial crossing: calibrated movement to its trailing edge, then gyro
  R90. **Do not recenter the chassis after that turn.**
- With drive stopped, open gripper, move to (250,0,100) with a freely chosen
  wrist/tool pitch, then to (200,0,-180) with tool pitch locked at -90 degrees.
- Follow the line at the already-configured PWM 255. Three IR-positive reads
  confirm the cube; the green LED blinks as the robot transports it.
- At the second confirmed crossing, green stays solid until 3 consecutive
  IR-negative reads indicate the cube has dropped. Then stop and reverse.
- If no cube has been confirmed before the second crossing, or the cube is
  lost before it, blink red immediately. Continue past the **second crossing**.
  Pit detection is armed only after the second crossing clears for a number
  of consecutive samples. Four consecutive all-eight-black readings then
  cause a reverse-to-line and reverse-to-junction, followed by final R90.
- Arm folds home before reversing. If hardware reports disappear, a time
  budget expires, or recovery fails, motors stop; no infinite advance.
- A short region of solid black **before** the second crossing cannot
  trigger the pit branch.

## ADC dropout

Only sustained `ADC STALE` in the regular route loop can trigger an STM32
reset and Telemetrix/ADC reinitialization, at most twice. The route index
does not return to the beginning. If the robot cannot confirm fresh ADC
and a visible line after reconnect, it stops. No automatic continuation
of an interrupted arm task is attempted. A `TypeError: NoneType...`
is a Python exception, not proof that STM32 should be reset.

## Camera

Browse `http://ROBOT_IP:8000/` for the two-panel `Robot cameras` interface.
The standalone legacy `Pi Camera` page is **not** this interface and may
occupy the port. During COLOR the right panel streams annotated detections;
outside COLOR the status says the task is inactive. At the end of
`main.py`, the camera and HTTP server are closed in `finally`.
