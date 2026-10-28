
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from telemetrix import telemetrix
from roboarm import RoboArm
from arm_ik import (
    inverse_kinematics,
    forward_kinematics,
    IKError,
    HOME_BASE,
    HOME_SHOULDER,
    HOME_ELBOW,
    HOME_WRIST,
    HOME_GRIPPER,
)

PORT = "/dev/serial0"
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080

START_ANGLES = {
    "base": HOME_BASE,
    "shoulder": HOME_SHOULDER,
    "elbow": HOME_ELBOW,
    "wrist": HOME_WRIST,
    "gripper": HOME_GRIPPER,
}

JOINT_NAMES = ("base", "shoulder", "elbow", "wrist", "gripper")
IK_JOINTS = ("base", "shoulder", "elbow", "wrist")

MOVE_STEPS = 30
STEP_DELAY = 0.03

board = None
arm = None
arm_lock = threading.Lock()
moving = False


HTML = r"""
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ArmRobotics 2026</title>
<style>
* { box-sizing: border-box; }
body {
    margin: 0;
    padding: 20px;
    background: #11151c;
    color: #e8edf5;
    font-family: system-ui, sans-serif;
}
h1 { margin-top: 0; }
h2 { font-size: 1.1rem; }
.layout {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
    gap: 16px;
    max-width: 1100px;
    margin: auto;
}
.panel {
    background: #1b222d;
    border: 1px solid #303b4b;
    border-radius: 12px;
    padding: 18px;
}
.control { margin: 18px 0; }
.control label {
    display: flex;
    justify-content: space-between;
    gap: 12px;
    margin-bottom: 8px;
}
input[type="range"] {
    width: 100%;
    accent-color: #62a5ff;
}
button {
    padding: 10px 14px;
    border: 0;
    border-radius: 7px;
    background: #3478d4;
    color: white;
    cursor: pointer;
    font-weight: 600;
}
button:disabled { opacity: 0.5; cursor: not-allowed; }
.secondary { background: #394658; }
.buttons { display: flex; flex-wrap: wrap; gap: 8px; }
.status {
    padding: 10px;
    border-radius: 7px;
    background: #11151c;
    color: #a9c7ef;
    overflow-wrap: anywhere;
}
.muted { color: #aab5c5; font-size: 0.9rem; }
</style>
</head>
<body>
<div class="layout">
<header style="grid-column:1/-1">
    <h1>ArmRobotics 2026</h1>
    <p class="muted">
        XYZ coordinates are millimeters relative to the home gripper position.
        Move to target commands a straight Cartesian path.
    </p>
</header>

<section class="panel">
    <h2>Cartesian control</h2>

    <div class="control">
        <label for="x"><span>X — forward/back</span><strong id="xValue">0</strong></label>
        <input id="x" type="range" min="-100" max="100" value="0" step="1">
    </div>

    <div class="control">
        <label for="y"><span>Y — left/right</span><strong id="yValue">0</strong></label>
        <input id="y" type="range" min="-100" max="100" value="0" step="1">
    </div>

    <div class="control">
        <label for="z"><span>Z — up/down</span><strong id="zValue">0</strong></label>
        <input id="z" type="range" min="-200" max="0" value="0" step="1">
    </div>

    <div class="control">
        <label for="pitch"><span>Tool pitch</span><strong id="pitchValue">0</strong></label>
        <input id="pitch" type="range" min="-90" max="90" value="0" step="1">
    </div>

    <label>
        <input id="elbowUp" type="checkbox">
        Elbow-up solution
    </label>

    <div class="buttons">
        <button id="move">Move to target</button>
        <button id="zero" class="secondary">Set XYZ to zero</button>
    </div>

    <p id="ikStatus" class="status">Ready.</p>
</section>

<section class="panel">
    <h2>Manual joint control</h2>
    <p class="muted">
        Manual controls move joints directly. They do not follow a Cartesian path.
    </p>
    <div id="jointControls"></div>
    <p id="jointStatus" class="status">Waiting for connection.</p>
</section>
</div>

<script>
const names = ["base", "shoulder", "elbow", "wrist", "gripper"];

const angles = {
    base: 95,
    shoulder: 180,
    elbow: 180,
    wrist: 20,
    gripper: 180
};

let debounce = null;
let requestId = 0;

async function postJSON(path, data) {
    const response = await fetch(path, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(data)
    });

    const result = await response.json();
    if (!response.ok) {
        throw new Error(result.error || "Request failed");
    }
    return result;
}

function setBusy(busy) {
    document.getElementById("move").disabled = busy;
    document.getElementById("zero").disabled = busy;
}

function buildJointControls() {
    const parent = document.getElementById("jointControls");

    for (const name of names) {
        const wrapper = document.createElement("div");
        wrapper.className = "control";

        const label = document.createElement("label");
        label.htmlFor = "joint-" + name;

        const title = document.createElement("span");
        title.textContent = name.charAt(0).toUpperCase() + name.slice(1);

        const value = document.createElement("strong");
        value.id = "value-" + name;
        value.textContent = angles[name] + "°";

        label.append(title, value);

        const slider = document.createElement("input");
        slider.type = "range";
        slider.id = "joint-" + name;
        slider.min = 0;
        slider.max = 180;
        slider.step = 1;
        slider.value = angles[name];

        slider.addEventListener("input", () => {
            angles[name] = Number(slider.value);
            value.textContent = angles[name] + "°";

            clearTimeout(debounce);
            debounce = setTimeout(async () => {
                const id = ++requestId;
                try {
                    await postJSON("/api/servo", {[name]: angles[name]});
                    if (id === requestId) {
                        document.getElementById("jointStatus").textContent =
                            name + " commanded to " + angles[name] + "°.";
                    }
                } catch (error) {
                    document.getElementById("jointStatus").textContent =
                        "Error: " + error.message;
                }
            }, 100);
        });

        wrapper.append(label, slider);
        parent.appendChild(wrapper);
    }
}

for (const id of ["x", "y", "z", "pitch"]) {
    document.getElementById(id).addEventListener("input", () => {
        document.getElementById(id + "Value").textContent =
            document.getElementById(id).value;
    });
}

document.getElementById("zero").addEventListener("click", () => {
    for (const id of ["x", "y", "z"]) {
        document.getElementById(id).value = 0;
        document.getElementById(id + "Value").textContent = "0";
    }
});

document.getElementById("move").addEventListener("click", async () => {
    const status = document.getElementById("ikStatus");
    const target = {
        x: Number(document.getElementById("x").value),
        y: Number(document.getElementById("y").value),
        z: Number(document.getElementById("z").value),
        tool_pitch: Number(document.getElementById("pitch").value),
        elbow_up: document.getElementById("elbowUp").checked
    };

    setBusy(true);
    status.textContent = "Planning and checking the path...";

    try {
        const result = await postJSON("/api/ik", target);

        for (const name of ["base", "shoulder", "elbow", "wrist"]) {
            angles[name] = result.angles[name];
            document.getElementById("joint-" + name).value = angles[name];
            document.getElementById("value-" + name).textContent =
                angles[name] + "°";
        }

        status.textContent =
            "Movement complete. Final angles: " +
            JSON.stringify(result.angles);
    } catch (error) {
        status.textContent = "Movement failed: " + error.message;
    } finally {
        setBusy(false);
    }
});

buildJointControls();
</script>
</body>
</html>
"""


def get_current_xyz():
    return forward_kinematics(
        arm.base.angle,
        arm.shoulder.angle,
        arm.elbow.angle,
    )


def get_current_angles():
    return {
        name: float(getattr(arm, name).angle)
        for name in JOINT_NAMES
    }


def plan_cartesian_path(target):
    start_x, start_y, start_z = get_current_xyz()

    target_x = float(target["x"])
    target_y = float(target["y"])
    target_z = float(target["z"])
    pitch = float(target.get("tool_pitch", 0))
    elbow_up = bool(target.get("elbow_up", False))

    path = []

    for index in range(1, MOVE_STEPS + 1):
        t = index / MOVE_STEPS

        x = start_x + (target_x - start_x) * t
        y = start_y + (target_y - start_y) * t
        z = start_z + (target_z - start_z) * t

        solution = inverse_kinematics(
            x,
            y,
            z,
            tool_pitch=pitch,
            elbow_up=elbow_up,
        )

        path.append({
            "base": solution.base,
            "shoulder": solution.shoulder,
            "elbow": solution.elbow,
            "wrist": solution.wrist,
        })

    return path


def execute_path(path):
    global moving

    with arm_lock:
        moving = True

        try:
            for angles in path:
                arm.write(**angles)

                for name, angle in angles.items():
                    getattr(arm, name).angle = int(angle)

                time.sleep(STEP_DELAY)
        finally:
            moving = False


class Handler(BaseHTTPRequestHandler):
    def send_json(self, status, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path

        if path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/api/status":
            if arm is None:
                self.send_json(503, {"connected": False})
            else:
                with arm_lock:
                    self.send_json(200, {
                        "connected": True,
                        "angles": get_current_angles(),
                        "xyz": get_current_xyz(),
                    })
            return

        self.send_json(404, {"error": "Not found"})

    def do_POST(self):
        global moving

        path = urlparse(self.path).path

        try:
            length = int(self.headers.get("Content-Length", "0"))

            if length <= 0 or length > 16384:
                self.send_json(400, {"error": "Invalid request body size."})
                return

            data = json.loads(self.rfile.read(length).decode("utf-8"))

            if not isinstance(data, dict):
                self.send_json(400, {"error": "Expected a JSON object."})
                return

            if arm is None:
                self.send_json(503, {"error": "Robot is not connected."})
                return

            if path == "/api/servo":
                commands = {}

                for name, angle in data.items():
                    if name not in JOINT_NAMES:
                        self.send_json(400, {"error": f"Unknown joint: {name}"})
                        return

                    angle = float(angle)

                    if not 0 <= angle <= 180:
                        self.send_json(
                            400,
                            {"error": f"{name} angle must be between 0 and 180."}
                        )
                        return

                    commands[name] = int(angle)

                with arm_lock:
                    if moving:
                        self.send_json(
                            409,
                            {"error": "Cartesian movement is in progress."}
                        )
                        return

                    arm.write(**commands)

                self.send_json(200, {"ok": True, "angles": commands})
                return

            if path == "/api/ik":
                with arm_lock:
                    if moving:
                        self.send_json(
                            409,
                            {"error": "Another movement is already in progress."}
                        )
                        return

                    path_angles = plan_cartesian_path(data)

                execute_path(path_angles)

                final_angles = path_angles[-1] if path_angles else {}

                self.send_json(200, {
                    "ok": True,
                    "angles": final_angles,
                    "target": {
                        "x": float(data["x"]),
                        "y": float(data["y"]),
                        "z": float(data["z"]),
                    },
                })
                return

            self.send_json(404, {"error": "Not found"})

        except (KeyError, TypeError, ValueError, IKError) as error:
            self.send_json(400, {"error": str(error)})

        except Exception as error:
            print("Request error:", repr(error))
            self.send_json(500, {"error": "Internal server error."})

    def log_message(self, format_string, *args):
        print("[web]", format_string % args)


def main():
    global board, arm

    try:
        print("Connecting to controller...")
        board = telemetrix.Telemetrix(com_port=PORT)

        print("Initializing arm...")
        arm = RoboArm(board)

        print("Sending startup positions:", START_ANGLES)
        with arm_lock:
            arm.write(**START_ANGLES)

        print("Startup positions sent.")

        server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), Handler)

        print(f"Arm web interface available on port {WEB_PORT}.")
        print("Stop with Ctrl+C.")

        try:
            server.serve_forever()
        finally:
            server.server_close()

    except KeyboardInterrupt:
        print("\nStopping web server.")

    except Exception as error:
        print("Startup error:", repr(error))
        raise

    finally:
        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
