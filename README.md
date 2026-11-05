                                  # ArmRobotics_2026
# Setup guide:
  1. Clone the repo
  2. Create virtual enviroment (venv) inside the folder [python -m venv venv] 
  3. Activate the enviroment [source venv/bin/activate]

## Camera-guided button control

Start the complete arm and camera system with one command:

    source venv/bin/activate
    python run_button_system.py

The launcher starts the arm controller first, waits for it to be ready, and
then starts the camera UI at http://127.0.0.1:8090. Stop both with Ctrl+C.
Starting arm_camera_button.py directly also switches to this supervised
launcher automatically when the local arm server is not running.

For an adaptive physical button press, select the red button, calibrate, align,
and use Full Test. The reference contact is X=350, Y=0, Z=-50 with a 60 mm
button. Full Test presses once and returns the arm to its base angles
(90, 170, 180, 0, 180). Then select "индикатор", draw a rectangle around the
lamp, and click "Проверить лампу". Success requires four consecutive green
frames. A red or unknown result does not trigger another press automatically.

## Green cube grasp

Stop the button camera server first, then launch the separate cube interface:

    python run_cube_system.py

Place the robot next to the green 5 × 5 × 5 cm cube on the floor and open port
8090. The launcher starts `arm_web.py` on port 8080 when needed. Start the
camera, then click "Найти и захватить кубик". The camera automatically selects
a green cube, calibrates local arm motion, and centers it in the image. The arm
commands gripper 0° (open) and advances in steps of at most 8 mm while checking
how the cube grows in the image. At the midpoint between the open and closed
tip offsets (60 and 80 mm), it corrects the 30 mm camera height offset and
commands gripper 180° (closed). The camera need not receive a cube XYZ.
Wheels do not move in this experiment. The physical gripper may need manual
help. STOP cancels remaining servo commands. The image scale approach and
camera orientation still need a physical check before relying on the grasp.
The camera and gripper offsets rotate together with the wrist; approach and
height correction follow the current tool pitch and base yaw.
