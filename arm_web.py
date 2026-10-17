
#!/usr/bin/env python3

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from telemetrix import telemetrix

from roboarm import RoboArm
from arm_ik import inverse_kinematics, forward_kinematics, IKError


# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

PORT = "/dev/serial0"
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080

# Synchronized movement settings.
# Lower MAX_JOINT_STEP_DEG or increase STEP_DELAY for slower motion.
MAX_JOINT_STEP_DEG = 1.0
STEP_DELAY = 0.04

board = None
arm = None

# Only one physical movement can run at a time.
motion_lock = threading.Lock()

# Protect individual Telemetrix command/state accesses.
arm_lock = threading.Lock()


# ------------------------------------------------------------
# Web interface
# ------------------------------------------------------------

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
    background: #10151d;
    color: #e8edf5;
    font-family: system-ui, sans-serif;
}

main {
    max-width: 1200px;
    margin: auto;
}

h1 { margin-top: 0; font-size: 1.7rem; }
h2 { margin-top: 0; font-size: 1.1rem; }

.layout {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(290px, 1fr));
    gap: 16px;
}

.panel {
    min-width: 0;
    padding: 18px;
    border: 1px solid #344155;
    border-radius: 14px;
    background: #1a222e;
}

.wide { grid-column: 1 / -1; }

.muted {
    color: #9eacbf;
    font-size: 0.9rem;
    line-height: 1.5;
}

.control { margin: 20px 0; }

.control label {
    display: flex;
    justify-content: space-between;
    gap: 12px;
    margin-bottom: 8px;
}

input[type="range"] {
    width: 100%;
    accent-color: #60a5fa;
    cursor: pointer;
}

input[type="checkbox"] {
    accent-color: #60a5fa;
}

button {
    padding: 11px 15px;
    border: 0;
    border-radius: 8px;
    background: #3478d4;
    color: white;
    font-weight: 650;
    cursor: pointer;
}

button:disabled {
    opacity: 0.55;
    cursor: wait;
}

button.secondary { background: #3b4658; }

.buttons {
    display: flex;
    flex-wrap: wrap;
    gap: 10px;
    margin-top: 16px;
}

.status {
    padding: 12px;
    border-radius: 8px;
    background: #10151d;
    color: #b8d6ff;
    overflow-wrap: anywhere;
    line-height: 1.5;
}

#armGraphic {
    display: block;
    width: 100%;
    height: auto;
    border-radius: 10px;
    background: #10151d;
}

.axis { stroke: #536174; stroke-width: 1; }

.arm-link {
    stroke: #60a5fa;
    stroke-width: 13;
    stroke-linecap: round;
}

.joint {
    fill: #e5efff;
    stroke: #60a5fa;
    stroke-width: 3;
}

.tool {
    stroke: #fbbf24;
    stroke-width: 5;
    stroke-linecap: round;
}

@media (max-width: 500px) {
    body { padding: 10px; }
    .panel { padding: 13px; }
}
</style>
</head>

<body>
<main>
    <h1>ArmRobotics 2026</h1>
    <p class="muted">
        Joint and Cartesian control. The graphic is a modelled pose,
        not physical position feedback. Keep clear of the robot.
    </p>

    <div class="layout">

        <section class="panel wide">
            <h2>Arm graphic</h2>

            <svg id="armGraphic" viewBox="0 0 700 390"
                 role="img" aria-label="Robot arm pose preview">

                <defs>
                    <pattern id="grid" width="25" height="25"
                             patternUnits="userSpaceOnUse">
                        <path d="M25 0H0V25"
                              fill="none"
                              stroke="#283342"
                              stroke-width="1"/>
                    </pattern>
                </defs>

                <rect width="700" height="390" fill="url(#grid)"/>

                <line x1="35" y1="320" x2="665" y2="320"
                      class="axis"/>

                <line x1="350" y1="25" x2="350" y2="350"
                      class="axis" stroke-dasharray="5 6"/>

                <path id="link1" class="arm-link"
                      d="M350 320 L350 200"/>

                <path id="link2" class="arm-link"
                      d="M350 200 L470 130"/>

                <path id="wristLink" class="tool"
                      d="M470 130 L510 130"/>

                <path id="gripperLeft" class="tool"
                      d="M510 130 L530 115"/>

                <path id="gripperRight" class="tool"
                      d="M510 130 L530 145"/>

                <circle cx="350" cy="320" r="19"
                        fill="#24364b"
                        stroke="#60a5fa"
                        stroke-width="4"/>

                <circle id="shoulderPoint" cx="350" cy="200"
                        r="11" class="joint"/>

                <circle id="elbowPoint" cx="470" cy="130"
                        r="11" class="joint"/>

                <circle id="wristPoint" cx="510" cy="130"
                        r="8" fill="#fbbf24"/>

                <g fill="#cbd5e1" font-size="14">
                    <text x="375" y="345">Base</text>
                    <text id="shoulderLabel" x="365" y="194">
                        Shoulder
                    </text>
                    <text id="elbowLabel" x="480" y="115">
                        Elbow
                    </text>
                    <text id="wristLabel" x="520" y="105">
                        Wrist
                    </text>
                    <text x="20" y="25" fill="#94a3b8">
                        Simplified side-view preview
                    </text>
                </g>
            </svg>

            <p class="muted">
                Both arm links are modelled as 120 mm. The drawing
                is approximate and may need calibration to match
                the physical servo mounting directions.
            </p>
        </section>

        <section class="panel">
            <h2>Joint sliders</h2>

            <p class="muted">
                Drag a slider to command that servo. The slider
                displays commanded angles, not measured feedback.
            </p>

            <div id="jointControls"></div>

            <p id="jointStatus" class="status" role="status">
                Ready. No command sent.
            </p>
        </section>

        <section class="panel">
            <h2>Cartesian target</h2>

            <p class="muted">
                X and Y are relative to the base axis.
                Z is relative to the shoulder pivot.
                Units are millimetres.
            </p>

            <div class="control">
                <label for="x">
                    <span>X</span>
                    <strong id="xValue">180 mm</strong>
                </label>
                <input id="x" type="range"
                       min="-230" max="230" value="180" step="1">
            </div>

            <div class="control">
                <label for="y">
                    <span>Y</span>
                    <strong id="yValue">0 mm</strong>
                </label>
                <input id="y" type="range"
                       min="-230" max="230" value="0" step="1">
            </div>

            <div class="control">
                <label for="z">
                    <span>Z</span>
                    <strong id="zValue">100 mm</strong>
                </label>
                <input id="z" type="range"
                       min="-230" max="230" value="100" step="1">
            </div>

            <div class="control">
                <label for="pitch">
                    <span>Tool pitch</span>
                    <strong id="pitchValue">0°</strong>
                </label>
                <input id="pitch" type="range"
                       min="-90" max="90" value="0" step="1">
            </div>

            <label>
                <input id="elbowUp" type="checkbox">
                Elbow-up solution
            </label>

            <div class="buttons">
                <button id="moveCartesian">Move to XYZ target</button>
            </div>

            <p id="ikStatus" class="status" role="status">
                Target not sent.
            </p>
        </section>

    </div>
</main>

<script>
"use strict";

const names = [
    "base", "shoulder", "elbow", "wrist", "gripper"
];

// These are UI estimates until fetched from the server.
// Initial page load does not issue servo commands.
const angles = {
    base: 90,
    shoulder: 90,
    elbow: 90,
    wrist: 90,
    gripper: 90
};

const $ = id => document.getElementById(id);

let sliderTimers = {};
let requestChain = Promise.resolve();
let cartesianBusy = false;


function createJointSliders() {
    const root = $("jointControls");

    for (const name of names) {
        const container = document.createElement("div");
        container.className = "control";

        const label = document.createElement("label");
        label.htmlFor = "joint-" + name;

        const title = document.createElement("span");
        title.textContent =
            name.charAt(0).toUpperCase() + name.slice(1);

        const value = document.createElement("strong");
        value.id = "value-" + name;
        value.textContent = angles[name] + "°";

        label.append(title, value);

        const slider = document.createElement("input");
        slider.type = "range";
        slider.id = "joint-" + name;
        slider.min = "0";
        slider.max = "180";
        slider.step = "1";
        slider.value = angles[name];

        slider.addEventListener("input", () => {
            const angle = Number(slider.value);

            angles[name] = angle;
            value.textContent = angle + "°";
            drawArm();

            clearTimeout(sliderTimers[name]);

            // Coalesce slider events so we don't flood Telemetrix.
            sliderTimers[name] = setTimeout(() => {
                requestChain = requestChain
                    .then(() => sendJoint(name, angle))
                    .catch(() => {});
            }, 120);
        });

        container.append(label, slider);
        root.appendChild(container);
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
    try {
        await postJSON("/api/servo", {[name]: angle});

        $("jointStatus").textContent =
            name + " commanded to " + angle + "°.";
    } catch (error) {
        $("jointStatus").textContent =
            "Command failed: " + error.message;
    }
}


function updateAngleControls(values) {
    for (const name of names) {
        if (!Number.isFinite(Number(values[name]))) {
            continue;
        }

        angles[name] = Number(values[name]);

        const slider = $("joint-" + name);
        const label = $("value-" + name);

        if (slider) slider.value = angles[name];
        if (label) label.textContent = angles[name] + "°";
    }

    drawArm();
}


for (const id of ["x", "y", "z", "pitch"]) {
    $(id).addEventListener("input", () => {
        const suffix = id === "pitch" ? "°" : " mm";
        $(id + "Value").textContent = $(id).value + suffix;
    });
}


$("moveCartesian").addEventListener("click", async () => {
    if (cartesianBusy) return;

    cartesianBusy = true;

    const button = $("moveCartesian");
    button.disabled = true;
    $("ikStatus").textContent = "Calculating and moving...";

    try {
        const result = await postJSON("/api/ik", {
            x: Number($("x").value),
            y: Number($("y").value),
            z: Number($("z").value),
            tool_pitch: Number($("pitch").value),
            elbow_up: $("elbowUp").checked
        });

        updateAngleControls(result.angles);

        $("ikStatus").textContent =
            "Movement complete. Final angles: " +
            JSON.stringify(result.angles);
    } catch (error) {
        $("ikStatus").textContent =
            "Movement failed: " + error.message;
    } finally {
        cartesianBusy = false;
        button.disabled = false;
    }
});


function drawArm() {
    // Simplified planar projection of the two 120 mm links.
    // This is a visualization, not a kinematic calibration model.

    const link1 = 120;
    const link2 = 120;
    const scale = 1.05;

    const originX = 350;
    const originY = 320;

    const q1 = (180 - angles.shoulder) * Math.PI / 180;
    const q2 = (angles.elbow - 90) * Math.PI / 180;

    const x1 = originX + link1 * Math.cos(q1) * scale;
    const y1 = originY - link1 * Math.sin(q1) * scale;

    const x2 = x1 + link2 * Math.cos(q1 + q2) * scale;
    const y2 = y1 - link2 * Math.sin(q1 + q2) * scale;

    const toolAngle =
        q1 + q2 + (90 - angles.wrist) * Math.PI / 180;

    const toolLength = 42;

    const x3 = x2 + toolLength * Math.cos(toolAngle);
    const y3 = y2 - toolLength * Math.sin(toolAngle);

    $("link1").setAttribute(
        "d", `M${originX} ${originY} L${x1} ${y1}`
    );

    $("link2").setAttribute(
        "d", `M${x1} ${y1} L${x2} ${y2}`
    );

    $("wristLink").setAttribute(
        "d", `M${x2} ${y2} L${x3} ${y3}`
    );

    const gripAngle = (angles.gripper - 90) * Math.PI / 180;
    const half = 12;

    const gx = half * Math.cos(gripAngle);
    const gy = half * Math.sin(gripAngle);

    $("gripperLeft").setAttribute(
        "d", `M${x3} ${y3} L${x3 + gx - gy} ${y3 - gy - gx}`
    );

    $("gripperRight").setAttribute(
        "d", `M${x3} ${y3} L${x3 + gx + gy} ${y3 - gy + gx}`
    );

    for (const [id, x, y] of [
        ["shoulderPoint", x1, y1],
        ["elbowPoint", x2, y2],
        ["wristPoint", x3, y3]
    ]) {
        $(id).setAttribute("cx", x);
        $(id).setAttribute("cy", y);
    }

    $("shoulderLabel").setAttribute("x", x1 + 12);
    $("shoulderLabel").setAttribute("y", y1 - 12);

    $("elbowLabel").setAttribute("x", x2 + 12);
    $("elbowLabel").setAttribute("y", y2 - 12);

    $("wristLabel").setAttribute("x", x3 + 10);
    $("wristLabel").setAttribute("y", y3 - 10);
}


async function refreshStatus() {
    try {
        const response = await fetch("/api/status", {
            cache: "no-store"
        });

        if (!response.ok) return;

        const result = await response.json();

        if (result.angles) {
            // Reading status never sends commands to the servos.
            updateAngleControls(result.angles);
        }
    } catch (_) {
        $("jointStatus").textContent =
            "Cannot refresh status. Check the Pi connection.";
    }
}


createJointSliders();
drawArm();
refreshStatus();
</script>
</body>
</html>
"""


# ------------------------------------------------------------
# Motion helpers
# ------------------------------------------------------------

def joint_names():
    return ("base", "shoulder", "elbow", "wrist", "gripper")


def current_angles():
    """Read the software's last commanded angles."""
    with arm_lock:
        return {
            name: float(getattr(arm, name).angle)
            for name in joint_names()
        }


def synchronized_joint_move(targets):
    """
    Move several joints together using a shared smoothstep profile.

    Each update commands all selected joints during the same control
    cycle. Hobby servos perform their own internal position control;
    this function ramps the commanded targets rather than controlling
    their internal motor current or speed.
    """
    start = current_angles()

    for name, target in targets.items():
        target = float(target)

        if not math.isfinite(target) or not 0 <= target <= 180:
            raise ValueError(
                f"{name} target must be between 0 and 180 degrees."
            )

    maximum_delta = max(
        (abs(float(targets[name]) - start[name])
         for name in targets),
        default=0.0
    )

    if maximum_delta == 0:
        return

    # Smoothstep's maximum slope is 1.5. This step count limits
    # the largest commanded change to approximately MAX_JOINT_STEP_DEG.
    steps = max(
        1,
        math.ceil(
            1.5 * maximum_delta / MAX_JOINT_STEP_DEG
        )
    )

    for index in range(1, steps + 1):
        t = index / steps
        smooth = t * t * (3.0 - 2.0 * t)

        commands = {
            name: round(
                start[name]
                + (float(targets[name]) - start[name]) * smooth
            )
            for name in targets
        }

        # Keep the multi-joint command batch together.
        with arm_lock:
            arm.write(**commands)

        time.sleep(STEP_DELAY)


# ------------------------------------------------------------
# HTTP request handler
# ------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):

    def send_json(self, status, payload):
        body = json.dumps(payload).encode("utf-8")

        self.send_response(status)
        self.send_header(
            "Content-Type", "application/json; charset=utf-8"
        )
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlparse(self.path).path

        if path in ("/", "/index.html"):
            body = HTML.encode("utf-8")

            self.send_response(200)
            self.send_header(
                "Content-Type", "text/html; charset=utf-8"
            )
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/api/status":
            try:
                self.send_json(200, {
                    "connected": board is not None,
                    "busy": motion_lock.locked(),
                    "angles": current_angles()
                })
            except Exception as error:
                self.send_json(500, {"error": str(error)})
            return

        self.send_json(404, {"error": "Not found."})

    def do_POST(self):
        path = urlparse(self.path).path

        try:
            length = int(self.headers.get("Content-Length", "0"))

            if length <= 0 or length > 16384:
                self.send_json(
                    400, {"error": "Invalid request body size."}
                )
                return

            data = json.loads(
                self.rfile.read(length).decode("utf-8")
            )

            if not isinstance(data, dict):
                self.send_json(
                    400, {"error": "Expected a JSON object."}
                )
                return

            # ------------------------------------------------
            # Individual joint command
            # ------------------------------------------------
            if path == "/api/servo":
                allowed = set(joint_names())
                commands = {}

                for name, value in data.items():
                    if name not in allowed:
                        self.send_json(
                            400, {"error": f"Unknown joint: {name}"}
                        )
                        return

                    angle = float(value)

                    if (
                        not math.isfinite(angle)
                        or not 0 <= angle <= 180
                    ):
                        self.send_json(
                            400,
                            {
                                "error":
                                f"{name} must be between 0 and 180."
                            }
                        )
                        return

                    commands[name] = angle

                if not commands:
                    self.send_json(
                        400, {"error": "No joint command supplied."}
                    )
                    return

                # Reject a conflicting command rather than waiting
                # behind an active Cartesian movement.
                if not motion_lock.acquire(blocking=False):
                    self.send_json(
                        409,
                        {
                            "error":
                            "Another movement is in progress."
                        }
                    )
                    return

                try:
                    with arm_lock:
                        arm.write(**commands)
                finally:
                    motion_lock.release()

                self.send_json(200, {
                    "ok": True,
                    "angles": current_angles()
                })
                return

            # ------------------------------------------------
            # Cartesian inverse kinematics + synchronized move
            # ------------------------------------------------
            if path == "/api/ik":
                x = float(data["x"])
                y = float(data["y"])
                z = float(data["z"])
                pitch = float(data.get("tool_pitch", 0))
                elbow_up = bool(data.get("elbow_up", False))

                # Validate the complete target before moving anything.
                solution = inverse_kinematics(
                    x, y, z,
                    tool_pitch=pitch,
                    elbow_up=elbow_up
                )

                targets = {
                    "base": solution.base,
                    "shoulder": solution.shoulder,
                    "elbow": solution.elbow,
                    "wrist": solution.wrist
                }

                if not motion_lock.acquire(blocking=False):
                    self.send_json(
                        409,
                        {
                            "error":
                            "Another movement is in progress."
                        }
                    )
                    return

                try:
                    # All four joints use one shared ramp.
                    synchronized_joint_move(targets)

                    model_x, model_y, model_z = forward_kinematics(
                        solution.base,
                        solution.shoulder,
                        solution.elbow
                    )

                    self.send_json(200, {
                        "ok": True,
                        "angles": current_angles(),
                        "target": {
                            "x": x,
                            "y": y,
                            "z": z
                        },
                        "modelled_xyz": {
                            "x": round(model_x, 1),
                            "y": round(model_y, 1),
                            "z": round(model_z, 1)
                        }
                    })

                finally:
                    # Always release the lock, even if a servo command
                    # or the response handler raises an exception.
                    motion_lock.release()

                return

            self.send_json(404, {"error": "Not found."})

        except (KeyError, TypeError, ValueError, IKError) as error:
            self.send_json(400, {"error": str(error)})

        except Exception as error:
            print("Request error:", repr(error))

            try:
                self.send_json(
                    500, {"error": "Internal server error."}
                )
            except Exception:
                pass

    def log_message(self, format_string, *args):
        print("[web]", format_string % args)


# ------------------------------------------------------------
# Startup and shutdown
# ------------------------------------------------------------

def main():
    global board, arm

    server = None

    try:
        print("Connecting to STM32 via Telemetrix...")
        board = telemetrix.Telemetrix(com_port=PORT)

        arm = RoboArm(board)

        server = ThreadingHTTPServer(
            (WEB_HOST, WEB_PORT),
            Handler
        )

        print(
            f"Arm web interface: http://<raspberry-pi-ip>:{WEB_PORT}"
        )
        print("No servo movement is commanded at startup.")
        print("Press Ctrl+C to stop the server.")

        server.serve_forever()

    except KeyboardInterrupt:
        print("\nStopping web server.")

    finally:
        if server is not None:
            server.server_close()

        # Do not command all servos to 90 degrees during shutdown.
        if board is not None:
            try:
                board.shutdown()
            except Exception as error:
                print("Board shutdown warning:", error)

        print("Web server stopped.")


if __name__ == "__main__":
    main()
