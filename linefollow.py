# linefollow.py
#
# Main line-following behavior.
#
# Hardware intentionally NOT included yet.
#
# Camera:
#     camera.py
#
# Vision:
#     vision.py
#
# Motors can later be added as:
#     motors.py
#
# No Telemetrix.
# No STM32.

import time

from camera import camera
from vision import LineTracker


# ============================================================
# CONFIGURATION
# ============================================================

DEAD_ZONE = 25

LOOP_DELAY = 0.02


# ============================================================
# LINE FOLLOWER
# ============================================================

class LineFollower:

    def __init__(self):

        self.tracker = LineTracker()

        self.running = False


    # ========================================================
    # PROCESS
    # ========================================================

    def process(
        self,
        frame
    ):

        result = self.tracker.process(
            frame
        )

        error = result.get(
            "error"
        )

        # ----------------------------------------------------
        # LINE LOST
        # ----------------------------------------------------

        if error is None:

            state = "LOST"

        # ----------------------------------------------------
        # CENTER
        # ----------------------------------------------------

        elif abs(error) <= DEAD_ZONE:

            state = "FORWARD"

        # ----------------------------------------------------
        # LEFT
        # ----------------------------------------------------

        elif error < 0:

            state = "LEFT"

        # ----------------------------------------------------
        # RIGHT
        # ----------------------------------------------------

        else:

            state = "RIGHT"


        result["state"] = state

        return result


    # ========================================================
    # RUN
    # ========================================================

    def run(self):

        self.running = True

        print()
        print("========================================")
        print("VISION LINE FOLLOWER")
        print("========================================")
        print()
        print(
            "Camera: camera.py"
        )
        print(
            "Vision: vision.py"
        )
        print(
            "Motors: DISABLED"
        )
        print()
        print(
            "Camera preview:"
        )
        print(
            "http://<PI-IP>:8000"
        )
        print()
        print(
            "Press CTRL+C to stop."
        )
        print()


        while self.running:

            frame = camera.get_frame()


            # ------------------------------------------------
            # No frame yet.
            # ------------------------------------------------

            if frame is None:

                time.sleep(
                    0.01
                )

                continue


            # ------------------------------------------------
            # Vision.
            # ------------------------------------------------

            result = self.process(
                frame
            )


            # ------------------------------------------------
            # Send processed image
            # to camera server.
            # ------------------------------------------------

            camera.set_processed_frame(
                result["display"]
            )


            # ------------------------------------------------
            # State.
            # ------------------------------------------------

            state = result[
                "state"
            ]


            # ------------------------------------------------
            # Crossing.
            # ------------------------------------------------

            crossing = result.get(
                "confirmed_crossing"
            )


            if crossing is not None:

                print()
                print(
                    "========================================"
                )

                print(
                    "CROSSING DETECTED:",
                    crossing
                )

                print(
                    "========================================"
                )

                print()


            # ------------------------------------------------
            # Debug.
            # ------------------------------------------------

            center = result.get(
                "line_center"
            )

            error = result.get(
                "error"
            )


            if center is None:

                print(
                    "LINE LOST"
                )

            else:

                print(
                    f"center={center:6.1f} "
                    f"error={error:7.1f} "
                    f"state={state}"
                )


            time.sleep(
                LOOP_DELAY
            )


        print(
            "Line follower stopped."
        )


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    follower = LineFollower()

    try:

        camera.start()

        follower.run()

    except KeyboardInterrupt:

        pass

    finally:

        follower.running = False

        camera.stop()

