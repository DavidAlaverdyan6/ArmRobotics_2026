
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

# Protect Telemetrix writes and shared controller state.
arm_lock = threading.Lock()
target_lock = threading.Lock()
latest_cartesian_target = None
target_generation = 0
cartesian_active = False
controller_stop = threading.Event()
controller_thread = None


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

input[type="number"] {
    width: 76px;
    padding: 8px;
    border: 1px solid #536174;
    border-radius: 6px;
    background: #10151d;
    color: #e8edf5;
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

            <p class="muted">
                The arm follows the latest target while you drag.
                Set a reference pose below before using Cartesian control.
            </p>

            <p id="ikStatus" class="status" role="status">
                Waiting for reference pose.
            </p>

            <p id="xyzStatus" class="status" role="status">
                Modelled XYZ: unavailable
            </p>
        </section>

        <section class="panel wide">
            <h2>Reference pose</h2>
            <p class="muted">
                Enter the angles that match the arm's current physical pose,
                then record them. This does not move any servo. Hobby servos
                do not report actual position, so estimate the angles from
                the physical arm. Cartesian control stays disabled until set.
            </p>
            <div id="referenceControls" class="buttons"></div>
            <div class="buttons">
                <button id="setReference">Record reference pose (no movement)</button>
            </div>
            <p id="referenceStatus" class="status" role="status">
                Reference pose not set.
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
let cartesianTimer = null;
let targetRequestVersion = 0;
let userDraggingJoint = false;


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
            userDraggingJoint = true;
            const angle = Number(slider.value);

            angles[name] = angle;
            value.textContent = angle + "°";
            drawArm();

            clearTimeout(sliderTimers[name]);

            // Manual joint control takes precedence over Cartesian targets.
            sliderTimers[name] = setTimeout(() => sendJoint(name, angle), 120);
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
            name + " commanded to " + angle + "°. Cartesian target cancelled.";
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


async function sendCartesianTarget() {
    const version = ++targetRequestVersion;
    const payload = {
        x: Number($("x").value),
        y: Number($("y").value),
        z: Number($("z").value),
        tool_pitch: Number($("pitch").value),
        elbow_up: $("elbowUp").checked
    };

    $("ikStatus").textContent = "Updating live target...";
    try {
        await postJSON("/api/ik", payload);
        if (version === targetRequestVersion) {
            $("ikStatus").textContent =
                "Following latest XYZ target (" + payload.x + ", " +
                payload.y + ", " + payload.z + ") mm.";
        }
    } catch (error) {
        if (version === targetRequestVersion) {
            $("ikStatus").textContent = "Target rejected: " + error.message;
        }
    }
}

function scheduleCartesianTarget() {
    clearTimeout(cartesianTimer);
    cartesianTimer = setTimeout(sendCartesianTarget, 100);
}

for (const id of ["x", "y", "z", "pitch"]) {
    $(id).addEventListener("input", () => {
        const suffix = id === "pitch" ? "°" : " mm";
        $(id + "Value").textContent = $(id).value + suffix;
        scheduleCartesianTarget();
    });
}
$("elbowUp").addEventListener("change", scheduleCartesianTarget);

function createReferenceControls() {
    const root = $("referenceControls");
    for (const name of names) {
        const label = document.createElement("label");
        label.style.display = "flex";
        label.style.alignItems = "center";
        label.style.gap = "8px";
        label.textContent = name.charAt(0).toUpperCase() + name.slice(1);

        const input = document.createElement("input");
        input.type = "number";
        input.min = "0";
        input.max = "180";
        input.step = "1";
        input.value = "90";
        input.id = "reference-" + name;
        input.setAttribute("aria-label", name + " reference angle");
        label.appendChild(input);
        root.appendChild(label);
    }
}

$("setReference").addEventListener("click", async () => {
    const button = $("setReference");
    const values = {};
    for (const name of names) {
        const value = Number($("reference-" + name).value);
        if (!Number.isFinite(value) || value < 0 || value > 180) {
            $("referenceStatus").textContent =
                "Every reference angle must be between 0° and 180°.";
            return;
        }
        values[name] = value;
    }

    button.disabled = true;
    try {
        const result = await postJSON("/api/reference", values);
        updateAngleControls(result.angles);
        $("referenceStatus").textContent =
            "Reference recorded. No servo was moved; Cartesian control is enabled.";
        $("ikStatus").textContent = "Ready. Drag an XYZ slider to move.";
    } catch (error) {
        $("referenceStatus").textContent =
            "Could not set reference: " + error.message;
    } finally {
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

        if (result.angles && !userDraggingJoint) {
            // Reading status never sends commands to the servos.
            updateAngleControls(result.angles);
        }

        $("referenceStatus").textContent = result.referenced
            ? "Reference pose is set."
            : "Reference pose not set.";

        if (result.modelled_xyz) {
            const p = result.modelled_xyz;
            $("xyzStatus").textContent = "Modelled XYZ: X " + p.x +
                " mm, Y " + p.y + " mm, Z " + p.z +
                " mm (not physical feedback).";
        } else {
            $("xyzStatus").textContent = "Modelled XYZ: unavailable until referenced.";
        }
    } catch (_) {
        $("jointStatus").textContent =
            "Cannot refresh status. Check the Pi connection.";
    }
}


createJointSliders();
createReferenceControls();
drawArm();
refreshStatus();

document.addEventListener("pointerup", () => { userDraggingJoint = false; });
document.addEventListener("keyup", event => {
    if (event.target && event.target.type === "range") userDraggingJoint = false;
});
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
    """Read last commanded/reference angles; None means unreferenced."""
    with arm_lock:
        return {
            name: (
                None if getattr(arm, name).angle is None
                else float(getattr(arm, name).angle)
            )
            for name in joint_names()
        }


def controller_loop():
    """Continuously move toward the newest Cartesian target."""
    global cartesian_active, latest_cartesian_target

    while not controller_stop.is_set():
        with target_lock:
            target = latest_cartesian_target
            generation = target_generation

        if target is None or not arm.is_referenced():
            with target_lock:
                cartesian_active = False
            controller_stop.wait(0.03)
            continue

        try:
            solution = inverse_kinematics(
                target["x"], target["y"], target["z"],
                tool_pitch=target["tool_pitch"],
                elbow_up=target["elbow_up"],
            )
            desired = {
                "base": solution.base,
                "shoulder": solution.shoulder,
                "elbow": solution.elbow,
                "wrist": solution.wrist,
            }

            angles = current_angles()
            commands = {}
            for name, goal in desired.items():
                current = angles[name]
                delta = goal - current
                commands[name] = (
                    goal if abs(delta) <= MAX_JOINT_STEP_DEG
                    else current + math.copysign(MAX_JOINT_STEP_DEG, delta)
                )

            # Do not issue an obsolete step if the target changed
            # while inverse kinematics was being calculated.
            with target_lock:
                if generation != target_generation:
                    continue
                cartesian_active = any(
                    abs(commands[name] - desired[name]) > 1e-6
                    for name in desired
                )
                # Serialize target changes with physical writes.
                with arm_lock:
                    arm.write(**commands)

            controller_stop.wait(STEP_DELAY)

        except Exception as error:
            print("[motion controller] Target rejected:", repr(error))
            with target_lock:
                if generation == target_generation:
                    latest_cartesian_target = None
                    cartesian_active = False
            controller_stop.wait(0.05)

    with target_lock:
        cartesian_active = False


def set_cartesian_target(target):
    """Replace the target atomically; the controller always follows newest."""
    global latest_cartesian_target, target_generation
    with target_lock:
        latest_cartesian_target = target
        target_generation += 1


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
                    "busy": cartesian_active,
                    "referenced": arm.is_referenced(),
                    "angles": current_angles(),
                    "modelled_xyz": (
                        None if not arm.is_referenced()
                        else dict(zip(
                            ("x", "y", "z"),
                            [round(v, 1) for v in forward_kinematics(
                                current_angles()["base"],
                                current_angles()["shoulder"],
                                current_angles()["elbow"]
                            )]
                        ))
                    )
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

                # Manual joint control takes precedence over any
                # queued Cartesian target.
                set_cartesian_target(None)
                with arm_lock:
                    arm.write(**commands)

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

                if not arm.is_referenced():
                    self.send_json(
                        409,
                        {"error": "Set the reference pose before Cartesian control."}
                    )
                    return

                # Validate the complete target before accepting it.
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

                if not arm.is_referenced():
                    self.send_json(
                        409,
                        {"error": "Set the reference pose before Cartesian control."}
                    )
                    return

                target = {
                    "x": x, "y": y, "z": z,
                    "tool_pitch": pitch,
                    "elbow_up": elbow_up,
                }
                set_cartesian_target(target)

                self.send_json(202, {
                    "ok": True,
                    "queued": True,
                    "target": {"x": x, "y": y, "z": z}
                })
                return

            if path == "/api/reference":
                values = {}
                for name in joint_names():
                    angle = float(data[name])
                    if not math.isfinite(angle) or not 0 <= angle <= 180:
                        self.send_json(400, {"error": f"{name} must be between 0 and 180."})
                        return
                    values[name] = angle

                set_cartesian_target(None)
                with arm_lock:
                    arm.set_reference(**values)

                self.send_json(200, {
                    "ok": True,
                    "referenced": True,
                    "angles": current_angles()
                })
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

        controller_thread = threading.Thread(
            target=controller_loop,
            name="cartesian-motion-controller",
            daemon=True,
        )
        controller_thread.start()

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
        controller_stop.set()
        if controller_thread is not None:
            controller_thread.join(timeout=2.0)

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
