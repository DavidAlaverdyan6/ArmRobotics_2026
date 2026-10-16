
#!/usr/bin/env python3

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from telemetrix import telemetrix
from roboarm import RoboArm

HOST = "0.0.0.0"
PORT = 8080

SERVO_NAMES = ["base", "shoulder", "elbow", "wrist", "gripper"]
MIN_ANGLE = 0
MAX_ANGLE = 180

board = None
arm = None
lock = threading.Lock()

# Current commanded angles; these are not measured positions.
angles = {
    "base": 90,
    "shoulder": 90,
    "elbow": 90,
    "wrist": 90,
    "gripper": 90,
}

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RoboArm Joint Control</title>
<style>
body {
  margin: 0; padding: 20px; background: #11151c; color: #e8edf5;
  font: 16px system-ui, sans-serif;
}
main { max-width: 700px; margin: auto; }
.panel {
  background: #1c2430; border: 1px solid #344154;
  padding: 18px; border-radius: 14px; margin: 14px 0;
}
.joint { margin: 23px 0; }
.heading { display:flex; justify-content:space-between; align-items:center; }
.angle { color:#69b5ff; font-size:20px; font-weight:bold; }
input[type=range] { width:100%; accent-color:#69b5ff; }
button {
  border:0; border-radius:9px; padding:12px 16px; margin:4px;
  background:#33445b; color:white; font-size:15px; cursor:pointer;
}
button:disabled { opacity:.5; }
.primary { background:#2385e6; }
.warning { background:#853e43; }
.muted { color:#9eacc0; font-size:13px; }
#status { padding:12px; background:#11151c; border-radius:8px; overflow-wrap:anywhere; }
</style>
</head>
<body>
<main>
<h1>RoboArm Joint Control</h1>
<p class="muted">Direct servo commands · Raspberry Pi + STM32</p>

<div class="panel">
  <h2>Individual joints</h2>
  <p class="muted">Move one joint at a time. Start with small movements and watch for mechanical interference.</p>
  <div id="joints"></div>
  <button class="primary" onclick="sendAll()">Apply all angles</button>
  <button onclick="resetSliders()">Reset sliders to 90°</button>
</div>

<div class="panel">
  <h2>Controller</h2>
  <div id="status">Connecting...</div>
  <p class="muted">Displayed angles are commanded values, not feedback from position sensors.</p>
</div>
</main>

<script>
const names = [
  ["base", "Base"],
  ["shoulder", "Shoulder"],
  ["elbow", "Elbow"],
  ["wrist", "Wrist"],
  ["gripper", "Gripper"]
];

const root = document.getElementById("joints");
const statusBox = document.getElementById("status");

for (const [id, label] of names) {
  const section = document.createElement("div");
  section.className = "joint";
  section.innerHTML = `
    <div class="heading">
      <label for="${id}">${label}</label>
      <span class="angle"><span id="${id}value">90</span>°</span>
    </div>
    <input id="${id}" type="range" min="0" max="180" value="90" step="1">
    <div class="heading muted"><span>0°</span><span>180°</span></div>
    <button onclick="sendOne('${id}')">Move ${label}</button>`;
  root.appendChild(section);

  document.getElementById(id).addEventListener("input", e => {
    document.getElementById(id + "value").textContent = e.target.value;
  });
}

async function post(data) {
  const buttons = [...document.querySelectorAll("button")];
  buttons.forEach(b => b.disabled = true);
  statusBox.textContent = "Sending command...";

  try {
    const response = await fetch("/api/servo", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(data)
    });
    const result = await response.json();
    if (!response.ok || !result.ok) throw new Error(result.error);
    statusBox.textContent = result.message;
  } catch (e) {
    statusBox.textContent = "Error: " + (e.message || e);
  } finally {
    buttons.forEach(b => b.disabled = false);
  }
}

function sendOne(name) {
  post({[name]: Number(document.getElementById(name).value)});
}

function sendAll() {
  const data = {};
  for (const [id] of names) {
    data[id] = Number(document.getElementById(id).value);
  }
  post(data);
}

function resetSliders() {
  for (const [id] of names) {
    document.getElementById(id).value = 90;
    document.getElementById(id + "value").textContent = "90";
  }
}

fetch("/api/status")
  .then(r => r.json())
  .then(() => statusBox.textContent = "Web controller is ready.")
  .catch(() => statusBox.textContent = "Cannot contact controller.");
</script>
</body>
</html>
"""


def command_servo(name, angle):
    """Send a direct angle command through the existing RoboArm API."""
    angle = int(angle)
    if not 0 <= angle <= 180:
        raise ValueError(f"{name}: angle must be between 0 and 180")

    # RoboArm exposes Servo objects as attributes.
    servo = getattr(arm, name)
    servo.write(angle)
    angles[name] = angle


class Handler(BaseHTTPRequestHandler):
    def send_json(self, status, data):
        body = json.dumps(data).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            body = HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/api/status":
            self.send_json(200, {"ok": True})
        else:
            self.send_json(404, {"ok": False, "error": "Not found"})

    def do_POST(self):
        if self.path != "/api/servo":
            self.send_json(404, {"ok": False, "error": "Not found"})
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 4096:
                raise ValueError("Invalid request size")
            data = json.loads(self.rfile.read(length))

            if not isinstance(data, dict) or not data:
                raise ValueError("No servo angles supplied")
            if any(name not in SERVO_NAMES for name in data):
                raise ValueError("Unknown servo name")

            with lock:
                for name, value in data.items():
                    if isinstance(value, bool) or not isinstance(value, (int, float)):
                        raise ValueError(f"{name}: angle must be numeric")
                    if not 0 <= value <= 180:
                        raise ValueError(f"{name}: angle must be between 0 and 180")

                # Apply commands in the order received.
                for name, value in data.items():
                    command_servo(name, value)

            self.send_json(200, {
                "ok": True,
                "message": "Command sent: " + ", ".join(
                    f"{name}={int(value)}°" for name, value in data.items()
                )
            })

        except Exception as exc:
            self.send_json(400, {"ok": False, "error": str(exc)})

    def log_message(self, fmt, *args):
        print("[HTTP]", fmt % args)


def main():
    global board, arm

    print("Connecting to STM32...")
    board = telemetrix.Telemetrix(com_port="/dev/serial0")
    arm = RoboArm(board)

    # Deliberately do not home or move the robot automatically on startup.
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Open http://127.0.0.1:{PORT} through an SSH tunnel.")
    print("Press Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        board.shutdown()
        print("Controller disconnected.")


if __name__ == "__main__":
    main()
