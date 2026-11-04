"""Eye-in-hand visual servo experiment for the ArmRobotics button.

This module is deliberately standalone.  It never opens a Telemetrix board or
moves the arm at import time.  For real motion use ``--arm-url`` and point it
at the already running arm_web.py server (one Telemetrix owner only).
"""
from __future__ import annotations

import argparse
import errno
import json
import logging
import math
import signal
import threading
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional
from urllib.error import URLError
from urllib.request import Request, urlopen

try:
    import cv2
    import numpy as np
except ImportError:  # Raspberry Pi dependencies are optional for dry import/tests.
    cv2 = None
    np = None

# Import the existing detector by path: ``2_bar_follow`` is not a valid Python
# dotted package component, and the detector itself has no hardware side effects.
try:
    import importlib.util
    _spec = importlib.util.spec_from_file_location(
        "armrobotics_color_detector", str(Path(__file__).parent / "Archive/2_bar_follow/Color_Detector_Daniel.py")
    )
    _detector_module = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_detector_module)
    ColorDetector = _detector_module.ColorDetector
except Exception:
    ColorDetector = None

from arm_ik import IKError, inverse_kinematics, forward_kinematics

LOG = logging.getLogger("arm_camera_button")


@dataclass
class Config:
    CAMERA_WIDTH: int = 640
    CAMERA_HEIGHT: int = 480
    TARGET_PIXEL_U: float = 320.0
    TARGET_PIXEL_V: float = 240.0
    MIN_RED_AREA: int = 700
    CENTER_TOLERANCE_PX: float = 12.0
    MAX_XYZ_STEP_MM: float = 8.0
    MOVEMENT_SETTLE_TIME: float = 0.35
    MAX_APPROACH_TIME: float = 20.0
    MAX_PRESS_DEPTH_MM: Optional[float] = None  # NOT_CALIBRATED: must be measured.
    BUTTON_TARGET_SIZE_MM: Optional[float] = 60.0  # User-measured diameter.
    SCENE_SENSOR_DISTANCE_MM: Optional[float] = 218.0  # Chassis-mounted sensor; not camera depth.
    CAMERA_TO_PRESS_POINT_OFFSET: Optional[tuple] = None  # NOT_CALIBRATED.
    MAX_ATTEMPTS: int = 2
    GREEN_CONFIRM_FRAMES: int = 4
    INDICATOR_CHECK_TIMEOUT: float = 8.0
    CAMERA_TIMEOUT: float = 5.0
    SEARCH_TIMEOUT: float = 20.0
    ALIGN_TIMEOUT: float = 60.0
    MAX_TARGET_LOSS_FRAMES: int = 3
    SAFE_WORKSPACE: Optional[dict] = None  # NOT_CALIBRATED until checked by operator.
    APPROACH_AXIS: Optional[tuple] = None  # NOT_CALIBRATED in eye-in-hand setup.
    HOME_XYZ: Optional[tuple] = None  # NOT_CALIBRATED: verify a collision-free home pose.
    CALIBRATION_MOVE_MM: float = 5.0
    CAMERA_ROTATION_DEG: int = 180
    MAX_SERVO_STEP_DEG: int = 1
    SERVO_STEP_INTERVAL: float = 0.15
    MAX_CALIBRATION_DRIFT_MM: float = 15.0
    OBSERVATION_ANGLES: tuple = (90, 170, 180, 0, 180)
    MAX_OBSERVATION_PROBE_DEG: int = 6
    MIN_PROBE_RESPONSE_PX: float = 10.0


CFG = Config()


class CameraManager:
    def __init__(self, config: Config = CFG):
        self.config, self.camera, self.last_frame_time = config, None, 0.0
        self.lock = threading.RLock()
        self.closing = threading.Event()

    def start(self):
        with self.lock:
            self._start()

    def _start(self):
        if self.closing.is_set():
            raise RuntimeError('Camera is shutting down')
        if self.camera is not None:
            return
        try:
            from picamera2 import Picamera2
        except ImportError as exc:
            raise RuntimeError("PiCamera2 is unavailable; use --dry-run with a camera backend") from exc
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration(
            main={"size": (self.config.CAMERA_WIDTH, self.config.CAMERA_HEIGHT), "format": "RGB888"},
            buffer_count=3, queue=False,
        ))
        self.camera.start()
        time.sleep(0.5)

    def read(self):
        with self.lock:
            self._start()
            request = self.camera.capture_request(flush=True)
            try:
                frame = request.make_array("main")
            finally:
                request.release()
            # The camera is mounted upside down and cannot be mechanically
            # rotated.  This pipeline delivers frames in the BGR order expected
            # by OpenCV/ColorDetector, so do not swap color channels here.
            if cv2 is not None and self.config.CAMERA_ROTATION_DEG == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            self.last_frame_time = time.monotonic()
        return frame

    def close(self):
        self.closing.set()
        with self.lock:
            if self.camera is not None:
                try:
                    self.camera.stop()
                finally:
                    self.camera.close()
                    self.camera = None


class ButtonDetector:
    """Uses the project's ColorDetector and adds target locking/tracking."""
    def __init__(self, config: Config = CFG):
        if ColorDetector is None:
            raise RuntimeError("ColorDetector could not be loaded")
        self.config, self.detector = config, ColorDetector(min_object_area=config.MIN_RED_AREA)
        self.locked_center, self.locked_size, self.lost_frames = None, None, 0
        self.confirmation_frames = 0
        self.pending_center = None
        self.target_id = 0

    def candidates(self, frame):
        # Preserve the project's classifier, but find contours in its RED mask
        # alone: adjacent blue/brown regions must not merge into the button.
        color_map = self.detector.classify_colors(frame)
        mask = np.where(color_map == self.detector.RED, 255, 0).astype(np.uint8)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        saturated = (((hsv[:, :, 0] <= 8) | (hsv[:, :, 0] >= 172)) &
                     (hsv[:, :, 1] >= 90) & (hsv[:, :, 2] >= 55))
        mask[~saturated] = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        height, width = frame.shape[:2]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.config.MIN_RED_AREA:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            perimeter = cv2.arcLength(contour, True)
            if not (0.6 <= w / h <= 1.7) or perimeter <= 0:
                continue
            if 4 * math.pi * area / perimeter**2 < 0.6:
                continue
            if x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1:
                continue  # cropped objects cannot provide a reliable centre/depth
            pad = max(6, min(w, h)//5)
            x0, y0, x1, y1 = max(0, x-pad), max(0, y-pad), min(width, x+w+pad), min(height, y+h+pad)
            ring = np.ones((y1-y0, x1-x0), dtype=bool)
            ring[y-y0:y-y0+h, x-x0:x-x0+w] = False
            surround = hsv[y0:y1, x0:x1]
            white = (surround[:, :, 1] < 70) & (surround[:, :, 2] > 100)
            if not ring.any() or white[ring].mean() < 0.3:
                continue
            moments = cv2.moments(contour)
            center = (round(moments['m10']/moments['m00']), round(moments['m01']/moments['m00']))
            objects.append(dict(color="R", x=x, y=y, w=w, h=h, center=center, area=round(area)))
        return objects

    def detect(self, frame):
        objects = red = self.candidates(frame)
        if not red:
            self.lost_frames += 1
            return None, objects
        if self.locked_center is None:
            if len(red) != 1:
                self.confirmation_frames = 0
                return None, objects  # multiple buttons require an explicit click
            chosen = red[0]
            stable = self.pending_center is not None and math.dist(chosen['center'], self.pending_center) < 10
            self.confirmation_frames = self.confirmation_frames + 1 if stable else 1
            self.pending_center = chosen['center']
            if self.confirmation_frames < 3:
                return None, objects
            self.target_id += 1
        else:
            chosen = min(red, key=lambda o: math.dist(o["center"], self.locked_center))
            ratio = (chosen['w'] * chosen['h']) / (self.locked_size[0] * self.locked_size[1])
            if math.dist(chosen["center"], self.locked_center) > min(50, max(chosen["w"], chosen["h"]) * 0.4) or not 0.6 < ratio < 1.6:
                self.lost_frames += 1
                return None, objects
        self.locked_center, self.locked_size, self.lost_frames = chosen["center"], (chosen["w"], chosen["h"]), 0
        chosen['target_id'] = self.target_id
        return chosen, objects

    @staticmethod
    def _is_saturated_red(frame, obj):
        """Reject brown/wood areas that pass the legacy ratio classifier."""
        if cv2 is None or np is None:
            return True
        x, y, w, h = (obj[k] for k in ("x", "y", "w", "h"))
        roi = frame[max(0, y):y+h, max(0, x):x+w]
        if roi.size == 0:
            return False
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        red = ((hue <= 8) | (hue >= 172)) & (hsv[:, :, 1] >= 90) & (hsv[:, :, 2] >= 55)
        return float(red.mean()) >= 0.30

    def reset_lock(self):
        self.locked_center = self.locked_size = None
        self.lost_frames = self.confirmation_frames = 0
        self.pending_center = None


class IndicatorDetector:
    def __init__(self, config: Config = CFG):
        self.config, self.roi = config, None

    def set_roi(self, x, y, w, h):
        self.roi = tuple(map(int, (x, y, w, h)))

    def state(self, frame):
        if self.roi is None:
            return "UNKNOWN"
        x, y, w, h = self.roi
        crop = frame[max(0, y):y+h, max(0, x):x+w]
        if crop.size == 0:
            return "UNKNOWN"
        b, g, r = crop.reshape(-1, 3).mean(axis=0)
        if max(r, g, b) < 55:
            return "UNKNOWN"
        if r > g * 1.25 and r > b * 1.25:
            return "RED"
        if g > r * 1.15 and g > b * 1.15:
            return "GREEN"
        return "UNKNOWN"


class ArmController:
    """HTTP adapter for the existing arm_web server; no direct STM32 connection."""
    def __init__(self, base_url: Optional[str] = None, config=CFG):
        self.base_url, self.motion_lock = base_url.rstrip("/") if base_url else None, threading.Lock()
        self.config = config
        self.stop_event = threading.Event()
        self.local_origin = None

    def status(self):
        if not self.base_url:
            return {"connected": False, "xyz": [0.0, 0.0, 0.0], "tool_pitch": 0.0}
        return self._request("GET", "/api/status")

    def move_xyz(self, xyz, pitch=0.0, dry_run=False):
        x, y, z = map(float, xyz)
        if not all(math.isfinite(v) for v in (x, y, z)):
            raise ValueError("non-finite XYZ")
        solution = inverse_kinematics(x, y, z, tool_pitch=pitch)
        current = self.status()
        if not current.get('connected'):
            raise RuntimeError('STM32 controller is not connected')
        if math.dist(current['xyz'], (x, y, z)) > self.config.MAX_XYZ_STEP_MM + 0.01:
            raise RuntimeError('XYZ step exceeds configured maximum')
        self._check_workspace((x, y, z))
        if dry_run:
            return {"dry_run": True, "xyz": [x, y, z]}
        if not self.base_url:
            raise RuntimeError("No arm_web URL configured; motion is disabled")
        return self.move_angles({n: round(getattr(solution, n)) for n in
                                 ('base', 'shoulder', 'elbow', 'wrist')})

    def _check_workspace(self, xyz):
        if self.config.SAFE_WORKSPACE is not None:
            for i, name in enumerate(('x', 'y', 'z')):
                low, high = self.config.SAFE_WORKSPACE[name]
                if not low <= xyz[i] <= high:
                    raise RuntimeError('Outside configured workspace')
        elif self.local_origin is None or math.dist(xyz, self.local_origin) > 20:
            raise RuntimeError('Outside local 20 mm observation trial envelope')

    def move_angles(self, target):
        """Integer servo microsteps through arm_web's existing short endpoint.

        Stop cancels subsequent writes; a write already accepted by STM32 cannot
        be recalled.  There is no physical joint-position feedback.
        """
        with self.motion_lock:
            current = self.status()
            previous = current['angles'].copy()
            final = {**previous, **target}
            if any(not math.isfinite(float(a)) or not 0 <= float(a) <= 180 for a in final.values()):
                raise RuntimeError('Invalid servo angle')
            if max(abs(final[n]-previous[n]) for n in final) > 4:
                raise RuntimeError('Local observation move exceeds 4 degrees')
            steps = int(max(abs(final[n]-previous[n]) for n in final))
            path = []
            last_xyz = current['xyz']
            for index in range(1, steps+1):
                pose = {n: round(previous[n] + (final[n]-previous[n])*index/steps) for n in final}
                xyz = forward_kinematics(*(pose[n] for n in ('base', 'shoulder', 'elbow', 'wrist')))
                self._check_workspace(xyz)
                if math.dist(xyz, last_xyz) > self.config.MAX_XYZ_STEP_MM:
                    raise RuntimeError('Servo microstep exceeds XYZ limit')
                path.append(pose)
                last_xyz = xyz
            expected = previous
            for pose in path:
                if self.stop_event.is_set():
                    raise InterruptedError('EMERGENCY STOP')
                actual = self.status()['angles']
                if any(abs(actual[n]-expected[n]) > 0.1 for n in expected):
                    raise RuntimeError('Arm was changed by another client; stopping')
                commands = {n: pose[n] for n in pose if pose[n] != expected[n]}
                if commands:
                    self._request('POST', '/api/servo', commands)
                expected = pose
                if self.stop_event.wait(self.config.SERVO_STEP_INTERVAL):
                    raise InterruptedError('EMERGENCY STOP')
            if self.stop_event.wait(self.config.MOVEMENT_SETTLE_TIME):
                raise InterruptedError('EMERGENCY STOP')
            return self.status()

    def _request(self, method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = Request(self.base_url + path, data=data, method=method,
                      headers={"Content-Type": "application/json"})
        try:
            with urlopen(req, timeout=5) as response:
                return json.loads(response.read().decode())
        except (URLError, TimeoutError) as exc:
            raise RuntimeError(f"arm_web unavailable: {exc}") from exc


class VisualServoController:
    """Pixel error -> XYZ using a locally measured 2x2 image Jacobian."""
    def __init__(self, config: Config = CFG):
        self.config, self.jacobian = config, None
        self.filtered_error = None
        self.reference = None
        self.coordinates = 'yz'

    @property
    def calibrated(self):
        return self.jacobian is not None

    def set_jacobian(self, matrix):
        if len(matrix) != 2 or any(len(row) != 2 for row in matrix):
            raise ValueError("Jacobian must be 2x2")
        self.jacobian = tuple(tuple(float(v) for v in row) for row in matrix)
        if not all(math.isfinite(v) for row in self.jacobian for v in row):
            self.jacobian = None
            raise ValueError('Jacobian contains non-finite values')
        if np is not None and np.linalg.cond(self.jacobian) > 30:
            self.jacobian = None
            raise ValueError('Jacobian is singular or poorly conditioned')

    def calibrate_from_samples(self, samples):
        """Estimate and store ``image_error = J * [dY, dZ]``.

        ``samples`` contains two independent ``(before, after, (dY, dZ))``
        observations made after short, settled moves.  The caller is expected
        to verify the scene stayed locked during both observations.
        """
        if len(samples) != 2:
            raise ValueError("two independent calibration samples are required")
        motion = []; image = []
        for before, after, move in samples:
            dy, dz = map(float, move)
            if abs(dy) + abs(dz) < 1e-9:
                raise ValueError("calibration motion must be non-zero")
            motion.append((dy, dz)); image.append((after[0] - before[0], after[1] - before[1]))
        det = motion[0][0] * motion[1][1] - motion[0][1] * motion[1][0]
        if abs(det) < 1e-6:
            raise ValueError("calibration moves must be independent")
        # Samples are rows: motion @ J.T = image.  Do not transpose
        # the off-diagonal terms; coupled camera axes need the complete matrix.
        inv = ((motion[1][1] / det, -motion[0][1] / det), (-motion[1][0] / det, motion[0][0] / det))
        self.set_jacobian(tuple(tuple(sum(inv[col][k] * image[k][row] for k in range(2))
                                     for col in range(2)) for row in range(2)))
        return self.jacobian

    def update(self, center):
        error = (center[0] - self.config.TARGET_PIXEL_U, center[1] - self.config.TARGET_PIXEL_V)
        if self.filtered_error is None:
            self.filtered_error = error
        else:
            self.filtered_error = tuple(.65*a + .35*b for a, b in zip(self.filtered_error, error))
        return self.filtered_error

    def correction(self, center):
        error = self.update(center)
        if not self.calibrated:
            raise RuntimeError("Visual servo Jacobian is NOT_CALIBRATED")
        if max(abs(v) for v in error) <= self.config.CENTER_TOLERANCE_PX:
            return (0.0, 0.0, 0.0)
        # Invert the measured mapping image_error = J * [local_y, local_z].
        a, b = self.jacobian[0]; c, d = self.jacobian[1]
        det = a*d - b*c
        if abs(det) < 1e-9:
            raise RuntimeError("Visual servo Jacobian is singular")
        dy = (d*error[0] - b*error[1]) / det
        dz = (-c*error[0] + a*error[1]) / det
        scale = min(0.5, 4.0 / max(math.hypot(dy, dz), 1e-9))
        return (0.0, -dy*scale, -dz*scale)


class ButtonTaskController:
    STATES = ("IDLE", "SEARCH", "ALIGN", "APPROACH", "PRESS", "RETREAT", "VERIFY", "HOME", "SUCCESS", "FAILED", "STOP")
    def __init__(self, camera, arm, button, indicator, servo, config=CFG):
        self.camera, self.arm, self.button, self.indicator, self.servo, self.config = camera, arm, button, indicator, servo, config
        self.state, self.mode, self.attempt, self.stop_event = "IDLE", "", 0, threading.Event()
        self.thread, self.status = None, {"state": "IDLE", "indicator": "UNKNOWN", "button": None, "error_px": None}
        self.observation_lock = threading.RLock()
        self.arm.stop_event = self.stop_event
        self.lifecycle_lock = threading.Lock()
        self.closing = threading.Event()

    def observe(self):
        with self.observation_lock:
            frame = self.camera.read()
            target, candidates = self.button.detect(frame)
            error = None if target is None else [target['center'][0]-self.config.TARGET_PIXEL_U,
                                                 target['center'][1]-self.config.TARGET_PIXEL_V]
            self.status.update(button=target, error_px=error, candidates=candidates,
                               frame_time=time.monotonic(), indicator=self.indicator.state(frame))
            return frame, target

    def telemetry(self):
        return {**self.status, 'state': self.state, 'attempt': self.attempt,
                'calibrated': self.servo.calibrated,
                'jacobian': self.servo.jacobian,
                'full_ready': False,
                'full_blocker': 'Distance, contact offset and safe approach/retreat route require calibration'}

    def fresh_target(self):
        samples = []
        for _ in range(6):
            if self.stop_event.is_set():
                raise InterruptedError('EMERGENCY STOP')
            _, target = self.observe()
            if target:
                samples.append(target)
                if len(samples) == 3:
                    centers = np.array([t['center'] for t in samples])
                    if np.max(np.ptp(centers, axis=0)) > 8:
                        raise RuntimeError('Button is not stable across fresh frames')
                    result = dict(samples[-1])
                    result['center'] = tuple(np.median(centers, axis=0).tolist())
                    return result
            elif self.button.locked_center is not None:
                raise RuntimeError('Target lost: further motion cancelled')
        raise RuntimeError('No confirmed red round button on white background')

    def calibrate(self):
        """Measure two image directions using bounded monotonic probes.

        The wrist is held fixed because this robot's small wrist servo has
        intermittent response.  Do not assume an exact return to commanded
        angles: finish calibration at the newly observed position instead.
        """
        self.state = 'CALIBRATE'
        self.status['message'] = 'Measuring base and shoulder response from fresh images; wrist held fixed'
        self.servo.jacobian = None
        origin = self.arm.status()
        self.arm.local_origin = origin['xyz']
        self.fresh_target()
        samples = []
        for joint, direction in (('base', 1), ('shoulder', -1)):
            before = self.fresh_target()
            probe_origin = self.arm.status()
            response = 0
            for step in range(1, self.config.MAX_OBSERVATION_PROBE_DEG+1):
                self.arm.move_angles({joint: probe_origin['angles'][joint]+direction*step})
                after = self.fresh_target()
                response = math.dist(after['center'], before['center'])
                if step >= 2 and response >= self.config.MIN_PROBE_RESPONSE_PX:
                    break
            if response < self.config.MIN_PROBE_RESPONSE_PX:
                raise RuntimeError(f'{joint}: no reliable visual response within bounded probe; motion stopped')
            observed_pose = self.arm.status()
            commanded = [observed_pose['angles'][name]-probe_origin['angles'][name] for name in ('base','shoulder')]
            samples.append((before['center'], after['center'], commanded))
            self.status['calibration_samples'] = list(samples)
            LOG.info('CALIBRATION %s: commanded=%s image=%s -> %s', joint, commanded, before['center'], after['center'])
        self.status['calibration_samples'] = samples
        self.servo.calibrate_from_samples(samples)
        self.servo.coordinates = 'base_shoulder'
        self.servo.reference = self.arm.status()
        self.arm.local_origin = self.servo.reference['xyz']
        self.servo.filtered_error = None
        self.status.update(calibration_pose=self.servo.reference, message='Camera direction calibration complete; approach remains uncalibrated')
        LOG.info('CALIBRATION COMPLETE (px/degree): %s', self.servo.jacobian)

    def _calibrate_xyz(self):
        self.state = 'CALIBRATE'
        self.servo.jacobian = None
        origin = self.arm.status()
        self.arm.local_origin = origin['xyz']
        self.fresh_target()
        # The operator's observation pose puts elbow/wrist on model limits.
        # Reserve two degrees of local travel, checking the button after each
        # one-degree write.  This is part of the explicitly started calibration.
        if origin['angles']['elbow'] == 180 and origin['angles']['wrist'] == 0:
            for elbow, wrist in ((179, 1), (178, 2)):
                self.arm.move_angles({'elbow': elbow, 'wrist': wrist})
                self.fresh_target()
            origin = self.arm.status()
            self.arm.local_origin = origin['xyz']
        initial = self.fresh_target()
        samples = []
        for dy, dz in ((4.0, 0.0), (0.0, 4.0)):
            before = self.fresh_target()
            xyz = origin['xyz']
            self.arm.move_xyz((xyz[0], xyz[1]+dy, xyz[2]+dz), origin['tool_pitch'])
            moved = self.arm.status()
            after = self.fresh_target()
            measured = (moved['xyz'][1]-xyz[1], moved['xyz'][2]-xyz[2])
            if abs(moved['xyz'][0]-xyz[0]) > 2:
                raise RuntimeError('Calibration has excessive X coupling after servo quantisation')
            samples.append((before['center'], after['center'], measured))
            LOG.info('CALIBRATION: actual YZ=%s image=%s -> %s', measured, before['center'], after['center'])
            self.arm.move_angles(origin['angles'])
            restored = self.fresh_target()
            if math.dist(restored['center'], initial['center']) > 8:
                raise RuntimeError('Calibration return did not reproduce the button position')
        self.servo.calibrate_from_samples(samples)
        self.servo.reference = self.arm.status()
        self.servo.filtered_error = None
        self.status.update(calibration_samples=samples, calibration_pose=self.servo.reference,
                           message='Local visual calibration complete')
        LOG.info('CALIBRATION COMPLETE: %s', self.servo.jacobian)

    def start(self, mode):
        with self.lifecycle_lock:
            if self.closing.is_set():
                raise RuntimeError('Server is shutting down')
            self._start_task(mode)

    def _start_task(self, mode):
        if self.thread and self.thread.is_alive():
            raise RuntimeError("task already running")
        if self.arm.motion_lock.locked():
            raise RuntimeError('Arm command is already running')
        if mode not in ('DRY_RUN', 'CALIBRATE', 'ALIGNMENT', 'FULL'):
            raise ValueError('Unknown test mode')
        if mode == 'FULL':
            raise RuntimeError(self.telemetry()['full_blocker'])
        if mode == 'ALIGNMENT' and not self.servo.calibrated:
            raise RuntimeError('Run Calibrate before Alignment Test')
        self.mode, self.stop_event, self.thread = mode, threading.Event(), threading.Thread(target=self._run, daemon=True)
        self.arm.stop_event = self.stop_event
        self.servo.filtered_error = None
        self.status.pop('last_error', None)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.state = "STOP"
        self.status['message'] = 'EMERGENCY STOP: further commands cancelled; an accepted servo write cannot be recalled'

    def close(self):
        with self.lifecycle_lock:
            self.closing.set()
            self.stop()
        if self.thread:
            self.thread.join(timeout=6)
            if self.thread.is_alive():
                LOG.warning('Control thread is still finishing an in-flight operation; no further moves are allowed')

    def _run(self):
        try:
            if self.mode == 'CALIBRATE':
                self.calibrate()
                self.state = 'CALIBRATED'
                return
            for attempt in range(1, self.config.MAX_ATTEMPTS + 1):
                self.attempt = attempt; self.state = "SEARCH"
                deadline = time.monotonic() + self.config.SEARCH_TIMEOUT
                target = None
                while time.monotonic() < deadline and not self.stop_event.is_set():
                    _, target = self.observe()
                    if target: break
                if not target: raise RuntimeError("button not found")
                if self.stop_event.is_set(): raise InterruptedError('EMERGENCY STOP')
                LOG.info("BUTTON DETECTED / TARGET LOCKED: %s", target["center"])
                if self.mode == "DRY_RUN":
                    self.state = "ALIGN"
                    self.status["button"] = target
                    self._align(target, dry_run=True)
                    self.state = "DRY_RUN_COMPLETE"
                    self.status["message"] = "DRY RUN COMPLETE: no arm motion was sent"
                    return
                self.state = "ALIGN"; self._align(target, dry_run=False)
                if self.mode == "ALIGNMENT":
                    self.state = "ALIGNED"
                    self.status["message"] = "ALIGNMENT TEST COMPLETE"
                    return
                initial_indicator = self._verify_initial_indicator()
                if initial_indicator != "RED":
                    raise RuntimeError("indicator is not confirmed RED before press")
                self.state = "APPROACH"; self._approach()
                self.state = "PRESS"; self._press()
                self.state = "RETREAT"; self._retreat()
                self.state = "VERIFY"
                result = self._verify()
                if result == "GREEN":
                    self.state = "HOME"
                    if self.config.HOME_XYZ is not None:
                        self.arm.move_xyz(self.config.HOME_XYZ, 0, self.mode != "FULL")
                    else:
                        LOG.warning("SUCCESS, but HOME_XYZ is NOT_CALIBRATED; arm remains in retreat pose")
                    self.state = "SUCCESS"; LOG.info("SUCCESS"); return
                if result != "RED":
                    raise RuntimeError("indicator state UNKNOWN; blind retry prohibited")
                if attempt < self.config.MAX_ATTEMPTS: LOG.warning("RETRYING")
            self.state = "FAILED"
        except Exception as exc:
            LOG.error("FAILED: %s", exc); self.state = "STOP" if self.stop_event.is_set() else "FAILED"
            self.status['message'] = str(exc)
            self.status['last_error'] = str(exc)
        finally:
            self.status.update(state=self.state, attempt=self.attempt)

    def _align(self, target, dry_run=False):
        LOG.info('ALIGNMENT STARTED')
        deadline = time.monotonic() + self.config.ALIGN_TIMEOUT
        previous_error = None
        for iteration in range(20):
            if time.monotonic() > deadline:
                break
            target = self.fresh_target()
            error = [target['center'][0]-self.config.TARGET_PIXEL_U, target['center'][1]-self.config.TARGET_PIXEL_V]
            self.status.update(button=target, error_px=error, iteration=iteration)
            if max(abs(v) for v in error) <= self.config.CENTER_TOLERANCE_PX:
                LOG.info('ALIGNMENT COMPLETE: error=%s', error)
                return
            if not self.servo.calibrated:
                if dry_run:
                    self.status['proposed_xyz'] = None
                    return
                raise RuntimeError('Jacobian NOT_CALIBRATED; alignment has not completed')
            pose = self.arm.status()
            ref = self.servo.reference
            if ref is None or math.dist(pose['xyz'], ref['xyz']) > self.config.MAX_CALIBRATION_DRIFT_MM or abs(pose['tool_pitch']-ref['tool_pitch']) > 5:
                raise RuntimeError('Camera moved beyond local calibration; run Calibrate again')
            magnitude = math.hypot(*error)
            if previous_error is not None and magnitude > previous_error + 5:
                raise RuntimeError('Image error increased after correction; recalibration required')
            previous_error = magnitude
            if self.servo.coordinates in ('base_shoulder', 'base_wrist'):
                active_error = [e if abs(e) > self.config.CENTER_TOLERANCE_PX else 0 for e in error]
                change = -0.5 * np.linalg.solve(np.array(self.servo.jacobian), active_error)
                next_angles = pose['angles'].copy()
                elevation_joint = 'wrist' if self.servo.coordinates == 'base_wrist' else 'shoulder'
                for joint, correction in zip(('base',elevation_joint),change):
                    next_angles[joint] += int(np.sign(correction)) if abs(correction) >= .35 else 0
                next_xyz = forward_kinematics(*(next_angles[n] for n in ('base','shoulder','elbow','wrist')))
                self.status.update(proposed_xyz=next_xyz, proposed_angles=next_angles)
                LOG.info('ALIGNMENT ERROR: %s proposed_angles=%s',error,next_angles)
                if dry_run: return
                if next_angles == pose['angles']:
                    raise RuntimeError('Correction is below servo angular resolution')
                self.arm.move_angles(next_angles)
                continue
            delta = self.servo.correction(target['center'])
            nxt = [a+b for a, b in zip(pose['xyz'], delta)]
            self.status['proposed_xyz'] = nxt
            LOG.info('ALIGNMENT ERROR: center=%s error=%s XYZ=%s next=%s iteration=%s', target['center'], error, pose['xyz'], nxt, iteration)
            if dry_run:
                return
            if self.stop_event.is_set(): raise InterruptedError('EMERGENCY STOP')
            moved = self.arm.move_xyz(nxt, pose['tool_pitch'])
            if math.dist(moved['xyz'], pose['xyz']) < 0.2:
                raise RuntimeError('Requested correction is smaller than servo resolution')
        raise RuntimeError("alignment timeout")

    def _approach(self):
        if self.config.APPROACH_AXIS is None: raise RuntimeError("approach axis NOT_CALIBRATED")
        # The operator-calibrated vector is bounded by MAX_PRESS_DEPTH_MM.
        axis = self.config.APPROACH_AXIS; depth = min(self.config.MAX_PRESS_DEPTH_MM, self.config.MAX_XYZ_STEP_MM)
        cur = self.arm.status().get("xyz", [0, 0, 0]); self.arm.move_xyz([cur[i] + axis[i]*depth for i in range(3)], 0, self.mode != "FULL")

    def _press(self):
        depth = self.config.MAX_PRESS_DEPTH_MM
        if depth is None: raise RuntimeError("press depth NOT_CALIBRATED")
        axis = self.config.APPROACH_AXIS; cur = self.arm.status().get("xyz", [0, 0, 0])
        self.arm.move_xyz([cur[i] + axis[i]*depth for i in range(3)], 0, self.mode != "FULL")

    def _retreat(self):
        if self.config.APPROACH_AXIS is None: raise RuntimeError("retreat axis NOT_CALIBRATED")
        cur = self.arm.status().get("xyz", [0, 0, 0]); axis = self.config.APPROACH_AXIS
        self.arm.move_xyz([cur[i] - axis[i]*self.config.MAX_PRESS_DEPTH_MM for i in range(3)], 0, self.mode != "FULL")

    def _verify(self):
        deadline, greens = time.monotonic() + self.config.INDICATOR_CHECK_TIMEOUT, 0
        while time.monotonic() < deadline and not self.stop_event.is_set():
            state = self.indicator.state(self.camera.read()); self.status["indicator"] = state
            greens = greens + 1 if state == "GREEN" else 0
            if greens >= self.config.GREEN_CONFIRM_FRAMES: LOG.info("GREEN CONFIRMED"); return "GREEN"
        return "RED" if self.status["indicator"] == "RED" else "UNKNOWN"

    def _verify_initial_indicator(self):
        state = self.indicator.state(self.camera.read())
        self.status["indicator"] = state
        return state


HTML = """<!doctype html><html lang=ru><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1"><title>Наведение на кнопку</title>
<style>body{background:#17202b;color:#eef3fa;font:16px system-ui;max-width:950px;margin:auto;padding:20px}
img{max-width:100%;height:auto;cursor:crosshair}button,select{font:inherit;padding:12px;margin:5px;border:0;border-radius:6px}
button:disabled{opacity:.4}#stop{background:#e84444;color:white}#msg{white-space:pre-wrap;padding:14px;background:#263447}
pre{overflow:auto}</style><h1>Наведение на красную кнопку</h1>
<p>1. Включи камеру. 2. Проверь рамку кнопки. 3. Калибровка. 4. Центрирование.</p>
<button onclick="v.src='/video'">Start Camera / Камера</button>
<button onclick="go('DRY_RUN')">Dry Run / Только проверить</button>
<button onclick="go('CALIBRATE')">Calibrate / Калибровка с движением</button>
<button onclick="go('ALIGNMENT')">Alignment Test / Центрировать</button>
<button disabled title="Нужны расстояние до контакта и проверенная траектория">Full Test / Нажать</button>
<button id=stop onclick="post('/api/stop')">STOP</button>
<p>Калибровка двигает руку на несколько мм. STOP отменяет следующие команды;
уже отправленный шаг сервопривода завершится.</p>
<label>Выбрать в кадре: <select id=selection><option value=button>кнопку — щелчком</option>
<option value=indicator>индикатор — выделить прямоугольник</option></select></label>
<div><img id=v width=640 height=480 alt="Нажмите Камера"></div><div id=msg>Ожидание</div>
<details><summary>Подробные данные</summary><pre id=s></pre></details>
<script>
async function post(url,data={}){try{let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});let j=await r.json();if(!r.ok)msg.textContent=j.error;return j}catch(e){msg.textContent=e.message}}
function go(mode){post('/api/start',{mode})}
function point(e){let b=v.getBoundingClientRect();return {x:Math.round((e.clientX-b.left)*640/b.width),y:Math.round((e.clientY-b.top)*480/b.height)}}
let start=null;v.ondragstart=e=>e.preventDefault();v.onpointerdown=e=>{start=point(e)};
v.onpointerup=e=>{let p=point(e);if(selection.value==='button')post('/api/select',p);else if(start)post('/api/indicator-roi',{x:Math.min(start.x,p.x),y:Math.min(start.y,p.y),w:Math.abs(p.x-start.x),h:Math.abs(p.y-start.y)});start=null};
setInterval(async()=>{try{let j=await(await fetch('/api/status',{cache:'no-store'})).json();s.textContent=JSON.stringify(j,null,2);
msg.textContent='Этап: '+j.state+'\\nОшибка (пиксели): '+JSON.stringify(j.error_px)+'\\n'+(j.message||'')+'\\n'+(j.arm_error||'')}catch(e){msg.textContent='Нет связи с сервером'}},700);
</script></html>"""


def make_server(controller, host="0.0.0.0", port=8090):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, obj, code=200):
            raw = json.dumps(obj, default=str).encode(); self.send_response(code); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(raw))); self.end_headers(); self.wfile.write(raw)
        def do_GET(self):
            if self.path == "/":
                raw = HTML.encode(); self.send_response(200); self.send_header("Content-Type", "text/html"); self.send_header("Content-Length", str(len(raw))); self.end_headers(); self.wfile.write(raw)
            elif self.path == "/video":
                if cv2 is None:
                    self.send_json({"error": "OpenCV unavailable"}, 503); return
                self.send_response(200); self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame"); self.end_headers()
                try:
                    while not self.server.closing.is_set():
                        frame, target = controller.observe()
                        if target:
                            x, y, w, h = (target[k] for k in ("x", "y", "w", "h"))
                            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
                            cv2.circle(frame, tuple(target["center"]), 5, (0, 255, 255), -1)
                        cv2.drawMarker(frame, (int(CFG.TARGET_PIXEL_U), int(CFG.TARGET_PIXEL_V)), (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
                        if controller.indicator.roi:
                            x,y,w,h = controller.indicator.roi
                            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                        ok, encoded = cv2.imencode(".jpg", frame)
                        if not ok: continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + encoded.tobytes() + b"\r\n")
                        self.wfile.flush(); time.sleep(0.08)
                except (BrokenPipeError, ConnectionResetError, RuntimeError):
                    pass
            elif self.path == '/frame.jpg':
                try:
                    frame, _ = controller.observe()
                    ok, data = cv2.imencode('.jpg', frame)
                    if not ok: raise RuntimeError('JPEG encoding failed')
                    self.send_response(200); self.send_header('Content-Type','image/jpeg')
                    self.send_header('Cache-Control','no-store'); self.end_headers(); self.wfile.write(data.tobytes())
                except Exception as exc: self.send_json({'error':str(exc)},503)
            elif self.path == "/api/status":
                payload = controller.telemetry()
                try: payload['arm'] = controller.arm.status()
                except Exception as exc: payload['arm_error'] = str(exc)
                self.send_json(payload)
            else: self.send_json({"error": "not found"}, 404)
        def do_POST(self):
            try:
                length = int(self.headers.get('Content-Length',0))
                if not 0 <= length <= 4096: raise ValueError('Request is too large')
                body = json.loads(self.rfile.read(length) or b'{}')
                if self.path == "/api/start": controller.start(body.get("mode", "DRY_RUN")); self.send_json({"ok": True})
                elif self.path == "/api/stop": controller.stop(); self.send_json({"ok": True})
                elif self.path == "/api/calibrate":
                    # The endpoint accepts values recorded by the operator after
                    # two small, settled, independent moves.  It never invents
                    # a calibration from image coordinates.
                    if controller.thread and controller.thread.is_alive(): raise ValueError('Task is running')
                    if "samples" in body:
                        controller.servo.calibrate_from_samples(body["samples"])
                    elif "jacobian" in body:
                        controller.servo.set_jacobian(body["jacobian"])
                    else:
                        controller.start('CALIBRATE')
                    self.send_json({"ok": True, "jacobian": controller.servo.jacobian})
                elif self.path == "/api/indicator-roi":
                    if controller.thread and controller.thread.is_alive(): raise ValueError('Task is running')
                    if min(body['w'],body['h']) < 4: raise ValueError('Indicator rectangle is too small')
                    controller.indicator.set_roi(body["x"], body["y"], body["w"], body["h"]); self.send_json({"ok": True})
                elif self.path == '/api/select':
                    if controller.thread and controller.thread.is_alive(): raise ValueError('Task is running')
                    with controller.observation_lock:
                        frame = controller.camera.read()
                        candidates = controller.button.candidates(frame)
                        clicked = [o for o in candidates if o['x'] <= body['x'] <= o['x']+o['w'] and o['y'] <= body['y'] <= o['y']+o['h']]
                        if len(clicked) != 1: raise ValueError('Click inside one red round button on white background')
                        obj = clicked[0]
                        controller.button.locked_center = obj['center']; controller.button.locked_size=(obj['w'],obj['h'])
                        controller.button.target_id += 1
                    self.send_json({'ok':True})
                else: self.send_json({"error": "not found"}, 404)
            except Exception as exc: self.send_json({"error": str(exc)}, 400)
        def log_message(self, *args): pass
    server = ThreadingHTTPServer((host, port), Handler)
    server.closing = threading.Event()
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-url", default="http://127.0.0.1:8080",
                        help="existing arm_web URL (default: http://127.0.0.1:8080)")
    parser.add_argument("--camera-only", action="store_true", help="disable arm control for this run")
    parser.add_argument("--host", default="0.0.0.0"); parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args(); logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    camera = CameraManager(); controller = ButtonTaskController(camera, ArmController(None if args.camera_only else args.arm_url), ButtonDetector(), IndicatorDetector(), VisualServoController())
    try:
        server = make_server(controller, args.host, args.port)
    except OSError as exc:
        if exc.errno != errno.EADDRINUSE:
            raise
        camera.close()
        display_host = '127.0.0.1' if args.host in ('0.0.0.0', '::') else args.host
        parser.exit(1, f'Порт {args.port} уже занят. Возможно, arm_camera_button.py уже запущен.\n'
                       f'Откройте http://{display_host}:{args.port} в браузере.\n'
                       'Если сервер уже работает, используйте его — второй экземпляр не нужен.\n')
    def request_exit(signum, frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, request_exit)
    try:
        LOG.info("Сервер запущен: http://127.0.0.1:%d", args.port)
        LOG.info("Для остановки нажмите Ctrl+C в этом терминале. Для повторного запуска выполните ту же команду.")
        LOG.info("При запуске рука не двигается. Управление рукой: %s", 'выключено' if args.camera_only else args.arm_url)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        LOG.info("Останавливаю сервер и освобождаю камеру...")
    finally:
        server.closing.set()
        try:
            controller.close()
        finally:
            server.server_close()
            try:
                camera.close()
            finally:
                signal.signal(signal.SIGTERM, previous_sigterm)
        LOG.info("Сервер остановлен. Порт %d и камера освобождены — можно запускать снова.", args.port)


if __name__ == "__main__": main()
