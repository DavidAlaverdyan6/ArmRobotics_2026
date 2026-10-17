import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from telemetrix import telemetrix

from roboarm import RoboArm
from arm_ik import inverse_kinematics, forward_kinematics, IKError


PORT = "/dev/serial0"
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080

board = None
arm = None

# Prevent simultaneous web requests from issuing conflicting servo commands.
arm_lock = threading.Lock()


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

    h1 { margin-top: 0; font-size: 1.6rem; }
    h2 { font-size: 1.1rem; margin-top: 0; }

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
        min-width: 0;
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

    input[type="number"] {
        width: 90px;
        background: #11151c;
        color: white;
        border: 1px solid #46536a;
        border-radius: 6px;
        padding: 6px;
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

    button.secondary { background: #394658; }

    .buttons {
        display: flex;
        flex-wrap: wrap;
        gap: 8px;
        margin-top: 16px;
    }

    .status {
        padding: 10px;
        border-radius: 7px;
        background: #11151c;
        color: #a9c7ef;
        overflow-wrap: anywhere;
    }

    .warning { color: #ffcf83; }
    .muted { color: #aab5c5; font-size: 0.9rem; }

    #preview {
        width: 100%;
        height: auto;
        background: #11151c;
        border-radius: 8px;
    }

    .checkbox {
        display: flex;
        align-items: center;
        gap: 8px;
        margin-top: 14px;
    }
</style>
</head>

<body>
<div class="layout">
    <header style="grid-column: 1 / -1">
        <h1>ArmRobotics 2026</h1>
        <p class="muted">
            Live joint control and inverse kinematics.
            Keep clear of the arm while testing.
        </p>
    </header>

    <section class="panel">
        <h2>Cartesian control (mm)</h2>

        <p class="muted">
            X and Y are measured from the base axis.
            Z is measured from the shoulder pivot, not the table.
        </p>

        <div class="control">
            <label for="x">
                <span>X</span><strong id="xValue">120</strong>
            </label>
            <input id="x" type="range" min="-240" max="240"
                   value="120" step="1">
        </div>

        <div class="control">
            <label for="y">
                <span>Y</span><strong id="yValue">0</strong>
            </label>
            <input id="y" type="range" min="-240" max="240"
                   value="0" step="1">
        </div>

        <div class="control">
            <label for="z">
                <span>Z</span><strong id="zValue">120</strong>
            </label>
            <input id="z" type="range" min="-240" max="240"
                   value="120" step="1">
        </div>

        <div class="control">
            <label for="pitch">
                <span>Tool pitch (degrees)</span>
                <strong id="pitchValue">0</strong>
            </label>
            <input id="pitch" type="range" min="-90" max="90"
                   value="0" step="1">
        </div>

        <label class="checkbox">
            <input id="elbowUp" type="checkbox">
            Elbow-up solution
        </label>

        <div class="buttons">
            <button id="moveCartesian">Move to target</button>
        </div>

        <p class="muted">
            Cartesian controls move the physical arm when you press
            the button. Joint sliders below move the arm immediately.
        </p>

        <div id="ikStatus" class="status" role="status">
            Ready. No movement commanded yet.
        </div>
    </section>

    <section class="panel">
        <h2>Joint control</h2>
        <p class="muted">
            Servo angles are commanded positions, not measured feedback.
        </p>

        <div id="jointControls"></div>

        <div class="buttons">
            <button id="refreshPreview" class="secondary">
                Refresh preview
            </button>
        </div>

        <p id="jointStatus" class="status" role="status">
            Waiting for connection.
        </p>
    </section>

    <section class="panel">
        <h2>Arm preview</h2>
        <svg id="preview" viewBox="0 0 360 320"
             role="img" aria-label="Simplified arm pose preview">
            <line x1="20" y1="270" x2="340" y2="270"
                  stroke="#526176" stroke-width="1"/>
            <circle cx="180" cy="270" r="8" fill="#62a5ff"/>
            <text x="180" y="300" text-anchor="middle"
                  fill="#aab5c5" font-size="12">
                Simplified side-view model
            </text>
            <polyline id="armShape" points="180,270 180,190 180,110"
                      fill="none" stroke="#62a5ff"
                      stroke-width="8" stroke-linecap="round"
                      stroke-linejoin="round"/>
            <circle id="shoulderPoint" cx="180" cy="190" r="7"
                    fill="#e8edf5"/>
            <circle id="elbowPoint" cx="180" cy="110" r="7"
                    fill="#e8edf5"/>
            <circle id="toolPoint" cx="180" cy="110" r="5"
                    fill="#ffcf83"/>
        </svg>
        <p class="muted">
            Preview is a simplified side view based on the commanded
            base, shoulder and elbow angles. It is not sensor feedback.
        </p>
    </section>
</div>

<script>
const jointNames = ["base", "shoulder", "elbow", "wrist", "gripper"];
const jointAngles = {
    base: 90,
    shoulder: 90,
    elbow: 90,
    wrist: 90,
    gripper: 90
};

const jointControls = document.getElementById("jointControls");
let requestTimer = null;
let requestSequence = 0;

function makeJointControls() {
    for (const name of jointNames) {
        const container = document.createElement("div");
        container.className = "control";

        const label = document.createElement("label");
        label.htmlFor = "joint-" + name;

        const title = document.createElement("span");
        title.textContent = name.charAt(0).toUpperCase() + name.slice(1);

        const value = document.createElement("strong");
        value.id = "jointValue-" + name;
        value.textContent = jointAngles[name] + "°";

        label.append(title, value);

        const slider = document.createElement("input");
        slider.type = "range";
        slider.id = "joint-" + name;
        slider.min = 0;
        slider.max = 180;
        slider.step = 1;
        slider.value = jointAngles[name];

        slider.addEventListener("input", () => {
            const angle = Number(slider.value);
            jointAngles[name] = angle;
            value.textContent = angle + "°";
            drawPreview();

            // Coalesce rapid slider events into the latest command.
            clearTimeout(requestTimer);
            requestTimer = setTimeout(
                () => sendJoint(name, angle), 100
            );
        });

        container.append(label, slider);
        jointControls.appendChild(container);
    }
}

async function postJSON(path, payload) {
    const response = await fetch(path, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(payload)
    });

    const result = await response.json();

    if (!response.ok) {
        throw new Error(result.error || "Request failed");
    }

    return result;
}

async function sendJoint(name, angle) {
    const sequence = ++requestSequence;

    try {
        await postJSON("/api/servo", {[name]: angle});

        if (sequence === requestSequence) {
            document.getElementById("jointStatus").textContent =
                name + " commanded to " + angle + "°.";
        }
    } catch (error) {
        if (sequence === requestSequence) {
            document.getElementById("jointStatus").textContent =
                "Error: " + error.message;
        }
    }
}

function readCartesianTarget() {
    return {
        x: Number(document.getElementById("x").value),
        y: Number(document.getElementById("y").value),
        z: Number(document.getElementById("z").value),
        tool_pitch: Number(document.getElementById("pitch").value),
        elbow_up: document.getElementById("elbowUp").checked
    };
}

for (const id of ["x", "y", "z", "pitch"]) {
    const slider = document.getElementById(id);
    slider.addEventListener("input", () => {
        document.getElementById(id + "Value").textContent = slider.value;
    });
}

document.getElementById("moveCartesian").addEventListener("click", async () => {
    const status = document.getElementById("ikStatus");
    status.textContent = "Calculating and commanding movement...";

    try {
        const result = await postJSON("/api/ik", readCartesianTarget());

        for (const name of ["base", "shoulder", "elbow", "wrist"]) {
            jointAngles[name] = result.angles[name];
            document.getElementById("joint-" + name).value =
                result.angles[name];
            document.getElementById("jointValue-" + name).textContent =
                result.angles[name] + "°";
        }

        drawPreview();

        status.textContent =
            "Commanded angles: " + JSON.stringify(result.angles) +
            ". Modelled XYZ: " + JSON.stringify(result.modelled_xyz);
    } catch (error) {
        status.textContent = "IK error: " + error.message;
    }
});

function drawPreview() {
    // A simplified planar preview using the two 120 mm links.
    const l1 = 120;
    const l2 = 120;

    const shoulder = Number(jointAngles.shoulder);
    const elbow = Number(jointAngles.elbow);

    // This drawing convention is only a rough visualization.
    // It may need adjustment after physical servo calibration.
    const q1 = (180 - shoulder) * Math.PI / 180;
    const q2 = (elbow - 90) * Math.PI / 180;

    const originX = 180;
    const originY = 270;
    const scale = 0.55;

    const x1 = originX + l1 * Math.cos(q1) * scale;
    const y1 = originY - l1 * Math.sin(q1) * scale;

    const x2 = x1 + l2 * Math.cos(q1 + q2) * scale;
    const y2 = y1 - l2 * Math.sin(q1 + q2) * scale;

    document.getElementById("armShape").setAttribute(
        "points",
        `${originX},${originY} ${x1},${y1} ${x2},${y2}`
    );

    for (const [id, x, y] of [
        ["shoulderPoint", x1, y1],
        ["elbowPoint", x2, y2],
        ["toolPoint", x2, y2]
    ]) {
        const point = document.getElementById(id);
        point.setAttribute("cx", x);
        point.setAttribute("cy", y);
    }
}

document.getElementById("refreshPreview").addEventListener(
    "click", drawPreview
);

makeJointControls();
drawPreview();
</script>
</body>
</html>
"""


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
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/api/status":
            with arm_lock:
                angles = {
                    name: getattr(arm, name).angle
                    for name in jointNamesForServer()
                }
            self.send_json(200, {"connected": True, "angles": angles})
            return

        self.send_json(404, {"error": "Not found"})

    def do_POST(self):
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

            if path == "/api/servo":
                allowed = {"base", "shoulder", "elbow", "wrist", "gripper"}
                commands = {}

                for name, angle in data.items():
                    if name not in allowed:
                        self.send_json(400, {"error": f"Unknown joint: {name}"})
                        return

                    angle = float(angle)
                    if not 0 <= angle <= 180:
                        self.send_json(
                            400,
                            {"error": f"{name} angle must be between 0 and 180."}
                        )
                        return

                    commands[name] = angle

                if not commands:
                    self.send_json(400, {"error": "No joint commands supplied."})
                    return

                with arm_lock:
                    arm.write(**commands)

                self.send_json(200, {"ok": True, "angles": commands})
                return

            if path == "/api/ik":
                x = float(data["x"])
                y = float(data["y"])
                z = float(data["z"])
                pitch = float(data.get("tool_pitch", 0))
                elbow_up = bool(data.get("elbow_up", False))

                solution = inverse_kinematics(
                    x, y, z,
                    tool_pitch=pitch,
                    elbow_up=elbow_up
                )

                angles = {
                    "base": solution.base,
                    "shoulder": solution.shoulder,
                    "elbow": solution.elbow,
                    "wrist": solution.wrist,
                }

                # Validate all IK calculations before commanding any joint.
                with arm_lock:
                    arm.write(**angles)

                model_x, model_y, model_z = forward_kinematics(
                    solution.base,
                    solution.shoulder,
                    solution.elbow
                )

                self.send_json(200, {
                    "ok": True,
                    "angles": angles,
                    "target": {"x": x, "y": y, "z": z},
                    "modelled_xyz": {
                        "x": round(model_x, 1),
                        "y": round(model_y, 1),
                        "z": round(model_z, 1),
                    }
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


def jointNamesForServer():
    return ("base", "shoulder", "elbow", "wrist", "gripper")


def main():
    global board, arm

    print("Connecting to STM32 via Telemetrix...")
    board = telemetrix.Telemetrix(com_port=PORT)

    try:
        arm = RoboArm(board)

        server = ThreadingHTTPServer((WEB_HOST, WEB_PORT), Handler)
        print(f"Arm web interface: http://<raspberry-pi-ip>:{WEB_PORT}")
        print("Stop with Ctrl+C.")

        try:
            server.serve_forever()
        finally:
            server.server_close()

    except KeyboardInterrupt:
        print("\nStopping web server.")

    finally:
        if board is not None:
            try:
                board.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
