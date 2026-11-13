# Navigation testing for Post Integral

The route entry point is `python3 main.py`. Calibrate via
`python3 calibrate.py` and record the measured forward pulse used after
a junction's trailing edge; `junction_drive.json` stores its time and PWM.
`python3 measure_55mm.py` remains available for three repeated physical runs.

1. Test with the wheels raised and an emergency power cutoff nearby.
2. Confirm the eight line-sensor channels, WHITE/GRAY/BLACK samples,
   and that the saved pulse settings are reported correctly.
3. Start on a marked 30 mm crossbar, verify trailing-edge detection, and
   measure axle position after the timed pulse.
4. Check L90/R90 and BUTTON, CUBE1, FIREFIGHTER and COLOR independently.
5. Check reverse returns, camera and arm behavior before a full route run.

The 55 mm ideal geometry excludes coasting and edge sensor latency.
Real-run acceptance tests are required; no hardware testing is implied.
