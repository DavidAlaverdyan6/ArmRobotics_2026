#!/usr/bin/env python3
"""10-second moving line-tracker ADC rate test; independent of route code."""
import time
from telemetrix import telemetrix
import line_tracker
import motors
from navigation_core import FreshFrames
from stm32_reset import reset_stm32
from differential2 import LinePID, PORT, BASE_SPEED, MAX_SPEED

DURATION = 10.0
POLL_INTERVAL = 0.005
CONTROL_INTERVAL = 0.04
STARTUP_TIMEOUT = 6.0
MAX_SAMPLE_AGE = 0.25
MAX_NO_LINE_SECONDS = 0.12

def main():
    board = None
    sensors_ready = False
    motors_ready = False
    started = None
    controller = LinePID()
    freshness = FreshFrames()
    counts = [0] * 8
    gaps = [[] for _ in range(8)]
    last_times = [None] * 8
    last_ids = None
    loops = 0
    control_updates = 0
    stale_snapshots = 0
    lost_since = None
    next_control = 0.0
    next_report = 0.0
    reason = "10 seconds completed"
    try:
        reset_stm32()
        board = telemetrix.Telemetrix(com_port=PORT)
        line_tracker.initialize(board)
        sensors_ready = True
        calibration = line_tracker.load_calibration()
        if calibration is None:
            print("No calibration. Run python calibrate.py first.")
            return
        if not line_tracker.wait_for_callbacks(timeout=STARTUP_TIMEOUT):
            print("Missing ADC callbacks:", line_tracker.callback_status())
            return
        if not line_tracker.all_channels_fresh(max_age=MAX_SAMPLE_AGE):
            print("ADC samples not fresh. Motors remain off.")
            return
        initial = line_tracker.update(calibration)
        if initial["position"] is None:
            print("No line at start. Motors remain off.")
            return
        motors.initialize(board)
        motors_ready = True
        motors.motor_stop()
        motors.enable_guard(lambda: line_tracker.all_channels_fresh(max_age=MAX_SAMPLE_AGE))
        print("ADC TEST: base PWM=255, max PWM=255, PID from differential2.")
        print("10-second run. Keep the track clear. Ctrl+C stops immediately.")
        time.sleep(1.0)
        if not line_tracker.all_channels_fresh(max_age=MAX_SAMPLE_AGE):
            print("ADC stale before start.")
            return
        state = line_tracker.update(calibration)
        if state["position"] is None:
            print("Line lost before start.")
            return
        last_ids = state["sample_ids"][:]
        last_times = state["sample_times"][:]
        started = time.monotonic()
        next_control = started
        next_report = started + 1.0
        motors.arm()
        while True:
            now = time.monotonic()
            if now - started >= DURATION:
                break
            state = line_tracker.update(calibration)
            loops += 1
            for i, new_id in enumerate(state["sample_ids"]):
                delta = new_id - last_ids[i]
                if delta > 0:
                    counts[i] += delta
                    # A timestamp only records the latest callback. Gaps are
                    # exact only when one callback arrived between polls.
                    if delta == 1 and last_times[i] is not None:
                        gaps[i].append(state["sample_times"][i] - last_times[i])
                    last_times[i] = state["sample_times"][i]
                    last_ids[i] = new_id
            if not line_tracker.all_channels_fresh(max_age=MAX_SAMPLE_AGE):
                reason = "ADC stale"
                break
            if state["position"] is None:
                if lost_since is None:
                    lost_since = now
                    motors.motor_stop()
                    controller.reset()
                if now - lost_since >= MAX_NO_LINE_SECONDS:
                    reason = "Line lost"
                    break
            else:
                lost_since = None
                if now >= next_control:
                    if freshness.accept(state, now):
                        command = controller.command(state["position"])
                        if command is not None:
                            motors.set_speeds(command[0], command[1])
                            control_updates += 1
                    else:
                        stale_snapshots += 1
                    next_control = now + CONTROL_INTERVAL
            if now >= next_report:
                elapsed = now - started
                rates = " ".join(f"S{i+1}={counts[i]/elapsed:.1f}" for i in range(8))
                print(f"{elapsed:.1f}s ADC Hz: {rates}; control={control_updates/elapsed:.1f}Hz")
                next_report += 1.0
            time.sleep(POLL_INTERVAL)
    except KeyboardInterrupt:
        reason = "Ctrl+C"
    except Exception as exc:
        reason = f"Error: {exc}"
    finally:
        if motors_ready:
            try:
                motors.disarm()
            except Exception as exc:
                print("Motor stop error:", exc)
        if started is not None:
            elapsed = max(time.monotonic() - started, 0.001)
            print(f"\nSTOP: {reason}; elapsed={elapsed:.2f}s")
            print(f"Polling: {loops/elapsed:.1f} Hz; PID updates: {control_updates/elapsed:.1f} Hz")
            print(f"Repeated PID snapshots: {stale_snapshots}")
            for i in range(8):
                maximum = max(gaps[i]) * 1000 if gaps[i] else None
                gap_text = f"{maximum:.1f} ms" if maximum is not None else "n/a"
                print(f"S{i+1}: {counts[i]/elapsed:.1f} Hz, callbacks={counts[i]}, max observed gap={gap_text}")
            print("Note: max observed gaps exclude intervals with multiple callbacks per poll.")
        if board is not None:
            try:
                if sensors_ready:
                    line_tracker.shutdown()
            finally:
                board.shutdown()

if __name__ == "__main__":
    main()
