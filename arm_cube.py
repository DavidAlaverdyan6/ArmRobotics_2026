#!/usr/bin/env python3
"""Separate camera guided grasp of a green 50 mm cube on the floor.

The robot is placed next to the cube by the operator. Camera calibration and
centering reuse the button experiment's local visual servo. Gripper commands
are sent even when the mechanical gripper needs physical assistance.
"""

import argparse
import errno
import json
import logging
import math
import os
import signal
import sys
import threading
import time
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from arm_camera_button import (
    CFG, ArmController, ButtonDetector, ButtonTaskController, CameraManager,
    ColorDetector, VisualServoController, cv2, np,
)


LOG = logging.getLogger('arm_cube')
CUBE_SIZE_MM = 50.0
CAMERA_ABOVE_GRIP_MM = 30.0
OPEN_TIP_AHEAD_MM = 60.0
CLOSED_TIP_AHEAD_MM = 80.0
TARGET_FORWARD_CLEARANCE_MM = (OPEN_TIP_AHEAD_MM + CLOSED_TIP_AHEAD_MM) / 2
MAX_APPROACH_TRAVEL_MM = 160.0
CUBE_CFG = replace(CFG, BUTTON_TARGET_SIZE_MM=CUBE_SIZE_MM)


class CubeDetector(ButtonDetector):
    """Daniel's color classification restricted to green objects."""

    def candidates(self, frame):
        color_map = self.detector.classify_colors(frame)
        green_map = np.where(
            color_map == self.detector.GREEN,
            self.detector.GREEN, 0,
        ).astype(np.uint8)
        return [obj for obj in self.detector.find_objects(green_map)
                if obj['area'] >= 1200 and
                0.5 <= obj['w'] / max(1, obj['h']) <= 2.0 and
                (self.selected or (obj['w'] < frame.shape[1] * 0.7 and
                                   obj['h'] < frame.shape[0] * 0.7))]


class CubeController(ButtonTaskController):
    def __init__(self, camera, arm):
        super().__init__(camera, arm, CubeDetector(CUBE_CFG), None,
                         VisualServoController(CUBE_CFG), CUBE_CFG)

    def observe(self):
        with self.observation_lock:
            frame = self.camera.read()
            if not self.button.selected:
                candidates = self.button.candidates(frame)
                if candidates:
                    self.button.select(max(candidates, key=lambda obj: obj['area']))
            target, candidates = self.button.detect(frame)
            control = None if target is None else self._control_center(target)
            error = None if control is None else [
                control[0] - self.config.TARGET_PIXEL_U,
                control[1] - self.config.TARGET_PIXEL_V,
            ]
            self.status.update(button=target, candidates=candidates,
                               error_px=error, frame_time=time.monotonic())
            return frame, target

    def fresh_target(self):
        # Servos can still be settling when arm_web acknowledges the command.
        # Wait for a stable sliding window instead of rejecting the first one.
        deadline = time.monotonic() + 3.0
        samples = []
        missed = 0
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                raise InterruptedError('STOP')
            _, target = self.observe()
            if target is None:
                samples.clear()
                missed += 1
                if missed > 5:
                    raise RuntimeError('Зелёный кубик потерян; движение остановлено')
            else:
                missed = 0
                samples.append(dict(target))
                samples = samples[-3:]
                if len(samples) == 3:
                    centers = np.array([sample['center'] for sample in samples])
                    sizes = np.array([[sample['w'], sample['h']] for sample in samples])
                    if (np.max(np.ptp(centers, axis=0)) <= 8 and
                            np.max(np.ptp(sizes, axis=0)) <= 5):
                        result = dict(samples[-1])
                        result['center'] = tuple(np.median(centers, axis=0).tolist())
                        result['w'], result['h'] = np.median(sizes, axis=0).tolist()
                        return result
            if self.stop_event.wait(0.10):
                raise InterruptedError('STOP')
        raise RuntimeError('Изображение кубика не стабилизировалось после движения')

    def _control_center(self, target):
        # Keep the cube visible near image center throughout approach. The
        # camera-to-gripper height correction is made after visual approach.
        return target['center']

    @staticmethod
    def tool_axes(pose):
        """Camera forward/up axes, assuming its lens is parallel to the tool."""
        pitch = math.radians(float(pose['tool_pitch']))
        yaw = math.radians(float(pose.get('angles', {}).get('base', 90)) - 90)
        forward = (math.cos(pitch)*math.cos(yaw),
                   math.cos(pitch)*math.sin(yaw), math.sin(pitch))
        up = (-math.sin(pitch)*math.cos(yaw),
              -math.sin(pitch)*math.sin(yaw), math.cos(pitch))
        return forward, up

    @staticmethod
    def camera_position(pose):
        forward, up = CubeController.tool_axes(pose)
        return tuple(float(pose['xyz'][i]) - CLOSED_TIP_AHEAD_MM*forward[i]
                     + CAMERA_ABOVE_GRIP_MM*up[i] for i in range(3))

    def recenter_approach(self, target):
        """Use the wrist to keep the target visible as the arm extends."""
        for _ in range(12):
            error = (target['center'][0]-self.config.TARGET_PIXEL_U,
                     target['center'][1]-self.config.TARGET_PIXEL_V)
            if max(abs(value) for value in error) <= 20:
                return target
            if self.stop_event.is_set():
                raise InterruptedError('STOP')
            pose = self.arm.status()
            commands = {}
            if abs(error[0]) > 20:
                commands['base'] = pose['angles']['base'] + (-1 if error[0] > 0 else 1)
            if abs(error[1]) > 20:
                commands['wrist'] = pose['angles']['wrist'] + (-1 if error[1] > 0 else 1)
            if any(not 0 <= angle <= 180 for angle in commands.values()):
                raise RuntimeError('Запястье достигло предела при наведении на кубик')
            self.arm._request('POST', '/api/servo', commands)
            if self.stop_event.wait(0.4):
                raise InterruptedError('STOP')
            target = self.fresh_target()
        raise RuntimeError('Не удалось удержать кубик в центре при подходе')

    def telemetry(self):
        blockers = [] if self.arm.base_url else ['управление рукой отключено']
        running = bool(self.thread and self.thread.is_alive())
        if running:
            blockers.append('Задача выполняется; дождитесь завершения или нажмите STOP')
        return {**self.status, 'state': self.state,
                'task_running': running,
                'calibrated': self.servo.calibrated,
                'auto_ready': not blockers,
                'auto_blocker': '; '.join(blockers)}

    def start(self, mode):
        with self.lifecycle_lock:
            if self.closing.is_set():
                raise RuntimeError('Сервер выключается')
            if self.thread and self.thread.is_alive():
                if self.mode == mode:
                    return False
                raise RuntimeError('Задача уже выполняется')
            if self.arm.motion_lock.locked():
                raise RuntimeError('Рука уже движется')
            if mode not in ('DRY_RUN', 'AUTO'):
                raise ValueError('Неизвестный режим')
            if mode == 'AUTO' and not self.telemetry()['auto_ready']:
                raise RuntimeError(self.telemetry()['auto_blocker'])
            self.mode = mode
            self.stop_event = threading.Event()
            self.arm.stop_event = self.stop_event
            self.servo.filtered_error = None
            for key in ('last_error', 'action_error', 'message'):
                self.status.pop(key, None)
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return True

    def set_stage(self, state, message):
        self.state = state
        self.status['message'] = message
        self.status.pop('action_error', None)
        LOG.info('%s: %s', state, message)

    def wait_for_gripper(self, seconds=5):
        try:
            for remaining in range(seconds, 0, -1):
                self.status.update(
                    stage_wait_seconds=remaining,
                    message=f'Гриппер: 0° (открыт). До подхода руки: {remaining} с',
                )
                if self.stop_event.wait(1):
                    raise InterruptedError('STOP')
        finally:
            self.status.pop('stage_wait_seconds', None)

    @staticmethod
    def camera_distance_from_scale(samples):
        """Estimate only remaining camera clearance from image growth.

        Under a fixed camera pitch, 1/face_pixels is linear in forward travel.
        This avoids selecting a world XYZ for the cube.
        """
        if len(samples) < 3 or samples[-1][0] - samples[0][0] < 6:
            return None
        xs = [float(sample[0]) for sample in samples]
        ys = [1.0 / float(sample[1]) for sample in samples]
        if samples[-1][1] - samples[0][1] < 3:
            return None
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        denominator = sum((x-mean_x)**2 for x in xs)
        if denominator <= 0:
            return None
        slope = sum((x-mean_x)*(y-mean_y) for x, y in zip(xs, ys)) / denominator
        if slope >= -1e-7:
            return None
        residual = sum((y - (mean_y + slope*(x-mean_x)))**2
                       for x, y in zip(xs, ys))
        total = sum((y-mean_y)**2 for y in ys)
        if total <= 0 or residual / total > 0.2:
            return None
        distance = ys[-1] / -slope
        return distance if 30 <= distance <= 600 else None

    def approach_by_camera(self):
        self.set_stage('APPROACH', 'Подхожу к кубику короткими шагами по изображению')
        travel = 0.0
        samples = []
        last_distance = None
        measured_at_travel = 0.0
        observed_travel = 0.0
        previous_camera = None
        previous_forward = None
        for _ in range(40):
            if self.stop_event.is_set():
                raise InterruptedError('STOP')
            target = self.recenter_approach(self.fresh_target())
            pose = self.arm.status()
            forward, _ = self.tool_axes(pose)
            camera = self.camera_position(pose)
            if previous_camera is not None:
                observed_travel += sum((camera[i]-previous_camera[i])*previous_forward[i]
                                       for i in range(3))
            previous_camera, previous_forward = camera, forward
            face_px = float(target['w'])
            samples.append((observed_travel, face_px))
            measured = self.camera_distance_from_scale(samples[-8:])
            if measured is not None:
                last_distance = measured
                measured_at_travel = observed_travel
            if last_distance is None:
                if travel >= 12:
                    raise RuntimeError('Кубик не увеличивается достаточно для безопасного подхода')
                step_mm = 2.0
            else:
                pitch = math.radians(float(self.arm.status()['tool_pitch']))
                forward = math.cos(pitch)
                if forward < 0.5:
                    raise RuntimeError('Наклон камеры слишком велик для захвата')
                desired_distance = TARGET_FORWARD_CLEARANCE_MM
                remaining = last_distance - (observed_travel - measured_at_travel) - desired_distance
                if remaining <= 2:
                    self.status['camera_clearance_mm'] = round(
                        last_distance - (observed_travel - measured_at_travel), 1)
                    return
                step_mm = min(8.0, remaining)
            if travel + step_mm > MAX_APPROACH_TRAVEL_MM:
                raise RuntimeError('Достигнут предел движения к кубику')
            pose = self.arm.status()
            forward, _ = self.tool_axes(pose)
            delta = {key: round(step_mm * axis, 3)
                     for key, axis in zip(('dx', 'dy', 'dz'), forward)}
            self.arm._request('POST', '/api/cube-step', delta, timeout=12)
            if self.stop_event.wait(0.35):
                raise InterruptedError('STOP')
            travel += step_mm
            self.status.update(approach_travel_mm=round(travel, 1),
                               camera_travel_mm=round(observed_travel, 1),
                               camera_clearance_mm=(None if last_distance is None
                                                    else round(last_distance, 1)))
        raise RuntimeError('Подход к кубику превысил число шагов')

    def correct_camera_height(self):
        """Place the tip at cube center using measured camera/gripper offsets."""
        _, up = self.tool_axes(self.arm.status())
        # Camera and finger offset rotates with the wrist. Move the whole tool
        # toward its local up axis to bring the lower finger plane to the cube.
        self.status['height_correction_mm'] = CAMERA_ABOVE_GRIP_MM
        remaining = CAMERA_ABOVE_GRIP_MM
        while abs(remaining) > 0.5:
            if self.stop_event.is_set():
                raise InterruptedError('STOP')
            step = min(8.0, remaining)
            self.arm._request('POST', '/api/cube-step',
                              {key: round(step * axis, 3)
                               for key, axis in zip(('dx', 'dy', 'dz'), up)},
                              timeout=12)
            remaining -= step

    def _run(self):
        try:
            self.set_stage('SEARCH', 'Ищу зелёный кубик')
            for _ in range(3):
                self.observe()
            target = self.fresh_target()
            if self.mode == 'DRY_RUN':
                self.state = 'DRY_RUN_COMPLETE'
                self.status['message'] = 'Зелёный кубик найден; движения не было'
                return
            self.calibrate()
            target = self.fresh_target()
            self.set_stage('ALIGN', 'Калибровка завершена. Навожу руку на кубик')
            self._align(target, dry_run=False)
            if self.stop_event.is_set():
                raise InterruptedError('STOP')
            self.set_stage('OPEN_GRIPPER', 'Отправляю грипперу команду 0° (открыть)')
            self.arm._request('POST', '/api/servo', {'gripper': 0})
            self.wait_for_gripper()
            self.approach_by_camera()
            self.set_stage('HEIGHT_CORRECTION', 'Совмещаю пальцы с кубиком с учётом положения камеры')
            self.correct_camera_height()
            if self.stop_event.is_set():
                raise InterruptedError('STOP')
            self.set_stage('CLOSE_GRIPPER', 'Отправляю грипперу команду 180° (закрыть)')
            self.arm._request('POST', '/api/servo', {'gripper': 180})
            self.state = 'GRASP_COMMANDED'
            self.status['message'] = (
                'Рука подошла к кубику; грипперу отправлена команда закрыться.'
            )
        except Exception as exc:
            LOG.exception('Cube task failed')
            self.state = 'STOP' if self.stop_event.is_set() else 'FAILED'
            self.status['message'] = str(exc)
            self.status['last_error'] = str(exc)
        finally:
            self.status['state'] = self.state


HTML = """<!doctype html><html lang=ru><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Зелёный кубик</title>
<style>body{background:#17202b;color:#eef3fa;font:16px system-ui;max-width:950px;margin:auto;padding:20px}
img{max-width:100%;height:auto;cursor:crosshair}button{font:inherit;padding:12px;margin:5px;border:0;border-radius:6px}
button:disabled{opacity:.4}#stop{background:#e84444;color:white}#msg{white-space:pre-wrap;padding:14px;background:#263447}pre{overflow:auto}</style>
<h1>Наведение на зелёный кубик 5 × 5 × 5 см</h1>
<p>Поставь робота рядом с кубиком и включи камеру. Программа сама найдёт
зелёный кубик, откалибрует наведение, откроет гриппер (0°), подведёт руку
к кубику и закроет гриппер (180°). Колёса не двигаются.</p>
<button onclick="v.src='/video'">Камера</button>
<button id=check onclick="go('DRY_RUN')">Проверить цель</button>
<button id=auto disabled onclick="go('AUTO')">Найти и захватить кубик</button>
<button id=stop onclick="post('/api/stop')">STOP</button>
<div id=readiness></div><img id=v width=640 height=480 alt="Нажмите Камера">
<div id=msg>Ожидание</div><details><summary>Подробные данные</summary><pre id=s></pre></details>
<script>
const autoButton=document.getElementById('auto');
const checkButton=document.getElementById('check');
let requestPending=false;
async function post(url,data={}){try{let r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});let j=await r.json();if(!r.ok)msg.textContent=j.error;return j}catch(e){msg.textContent=e.message}}
async function go(mode){if(requestPending)return;requestPending=true;autoButton.disabled=true;checkButton.disabled=true;try{await post('/api/start',{mode})}finally{requestPending=false}}
setInterval(async()=>{try{let j=await(await fetch('/api/status',{cache:'no-store'})).json();s.textContent=JSON.stringify(j,null,2);
autoButton.disabled=requestPending||!j.auto_ready;checkButton.disabled=requestPending||j.task_running;
readiness.textContent=j.task_running?'Задача выполняется. STOP остановит следующие движения.':(j.auto_ready?'Автоматический захват доступен.':j.auto_blocker);
msg.textContent='Этап: '+j.state+'\\nОшибка (пиксели): '+JSON.stringify(j.error_px)+'\\n'+(j.action_error||'')+'\\n'+(j.message||'')+'\\n'+(j.arm_error||'')
}catch(e){msg.textContent='Нет связи с сервером'}},700);
</script></html>"""


def make_server(controller, host='0.0.0.0', port=8090):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, payload, code=200):
            raw = json.dumps(payload, default=str).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path == '/':
                raw = HTML.encode()
                self.send_response(200)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            elif self.path == '/video':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    while not controller.closing.is_set():
                        frame, target = controller.observe()
                        for obj in controller.status['candidates']:
                            x, y, w, h = (obj[key] for key in ('x', 'y', 'w', 'h'))
                            color = (0, 255, 0) if target is not None and obj is target else (0, 255, 255)
                            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                        ok, encoded = cv2.imencode('.jpg', frame)
                        if ok:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + encoded.tobytes() + b'\r\n')
                            self.wfile.flush()
                        time.sleep(0.08)
                except (BrokenPipeError, ConnectionResetError, RuntimeError):
                    pass
            elif self.path == '/frame.jpg':
                try:
                    frame, _ = controller.observe()
                    ok, encoded = cv2.imencode('.jpg', frame)
                    if not ok:
                        raise RuntimeError('JPEG encoding failed')
                    raw = encoded.tobytes()
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(raw)))
                    self.end_headers()
                    self.wfile.write(raw)
                except Exception as exc:
                    self.send_json({'error': str(exc)}, 503)
            elif self.path == '/api/status':
                payload = controller.telemetry()
                try:
                    payload['arm'] = controller.arm.status()
                except Exception as exc:
                    payload['arm_error'] = str(exc)
                self.send_json(payload)
            else:
                self.send_json({'error': 'not found'}, 404)

        def do_POST(self):
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 <= length <= 4096:
                    raise ValueError('Запрос слишком большой')
                body = json.loads(self.rfile.read(length) or b'{}')
                if self.path == '/api/start':
                    started = controller.start(body.get('mode', 'DRY_RUN'))
                    self.send_json({'ok': True, 'already_running': not started})
                elif self.path == '/api/stop':
                    controller.stop()
                    self.send_json({'ok': True})
                else:
                    self.send_json({'error': 'not found'}, 404)
            except Exception as exc:
                controller.status['action_error'] = str(exc)
                self.send_json({'error': str(exc)}, 400)

        def log_message(self, *args):
            pass

    return ThreadingHTTPServer((host, port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm-url', default='http://127.0.0.1:8080')
    parser.add_argument('--camera-only', action='store_true')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8090)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    if cv2 is None or np is None or ColorDetector is None:
        parser.error('OpenCV и детектор Daniel недоступны')
    if not args.camera_only:
        from run_button_system import arm_status
        status = arm_status(args.arm_url)
        if status is None or status.get('connected') is not True:
            os.execv(sys.executable, [
                sys.executable,
                str(Path(__file__).resolve().parent / 'run_cube_system.py'),
                '--arm-url', args.arm_url, '--host', args.host,
                '--port', str(args.port),
            ])
        if 'cube-step-v2' not in status.get('capabilities', []):
            parser.error('На порту 8080 старый arm_web. Остановите его и запустите run_cube_system.py')
    camera = CameraManager(CUBE_CFG)
    controller = CubeController(camera, ArmController(
        None if args.camera_only else args.arm_url, CUBE_CFG))
    try:
        server = make_server(controller, args.host, args.port)
    except OSError as exc:
        if exc.errno != errno.EADDRINUSE:
            raise
        parser.error(f'Порт {args.port} занят. Остановите предыдущий сервер камеры.')
    def exit_signal(signum, frame):
        raise KeyboardInterrupt
    previous_sigterm = signal.signal(signal.SIGTERM, exit_signal)
    try:
        LOG.info('Кубик: http://127.0.0.1:%d', args.port)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        controller.close()
        server.server_close()
        camera.close()
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    main()
