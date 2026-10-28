
import json
import math
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
AUTO_MAX_STEP_DEG = 2.0

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
.visual-panel { grid-column: 1 / -1; }
.views { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 12px; }
.view { background: #11151c; border: 1px solid #303b4b; border-radius: 8px; padding: 8px; min-width: 0; }
.view h3 { margin: 4px 4px 8px; font-size: 0.95rem; }
.view canvas { display: block; width: 100%; height: 250px; }
.legend { display: flex; flex-wrap: wrap; gap: 12px; margin-top: 8px; font-size: 0.85rem; color: #aab5c5; }
.legend span::before { content: ""; display: inline-block; width: 10px; height: 10px; border-radius: 50%; margin-right: 5px; background: var(--dot); }

</style>
</head>
<body>
<div class="layout">
<header style="grid-column:1/-1">
    <h1>ArmRobotics 2026</h1>
    <p class="muted">
        Cartesian XYZ is the closed-gripper tip in millimeters relative to the base model origin.
        Move to target interpolates XYZ and tool pitch. Joint calibration is approximate.
    </p>
</header>

<section class="panel">
    <h2>Cartesian control</h2>

    <div class="control">
        <label for="x"><span>X — forward/back</span><strong id="xValue">0</strong></label>
        <input id="x" type="range" min="-415" max="415" value="0" step="1">
    </div>

    <div class="control">
        <label for="y"><span>Y — left/right</span><strong id="yValue">0</strong></label>
        <input id="y" type="range" min="-415" max="415" value="0" step="1">
    </div>

    <div class="control">
        <label for="z"><span>Z — up/down</span><strong id="zValue">0</strong></label>
        <input id="z" type="range" min="-415" max="415" value="0" step="1">
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
        <button id="zero" class="secondary">Use current XYZ</button>
    </div>

    <p id="ikStatus" class="status">Ready.</p>
</section>

<section class="panel">
    <h2>Automatic XYZ destination</h2>
    <p class="muted">Enter the closed-gripper tip position in mm. The arm chooses the wrist angle and a joint-space route. The tip may follow a curved path.</p>
    <div class="control"><label for="autoX">X (mm)</label><input id="autoX" type="number" min="-415" max="415" step="1" value="0"></div>
    <div class="control"><label for="autoY">Y (mm)</label><input id="autoY" type="number" min="-415" max="415" step="1" value="0"></div>
    <div class="control"><label for="autoZ">Z (mm)</label><input id="autoZ" type="number" min="-415" max="415" step="1" value="0"></div>
    <div class="buttons">
        <button id="autoMove">Move automatically</button>
        <button id="autoCurrent" class="secondary">Fill current XYZ</button>
    </div>
    <p id="autoStatus" class="status">Ready. Automatic mode does not check collisions.</p>
</section>

<section class="panel">
    <h2>Manual joint control</h2>
    <p class="muted">
        Manual controls move joints directly. They do not follow a Cartesian path.
    </p>
    <div id="jointControls"></div>
    <p id="jointStatus" class="status">Waiting for connection.</p>
</section>

<section class="panel visual-panel">
    <h2>Arm position — kinematic model</h2>
    <p class="muted">
        The diagrams update from the angles commanded by Arm Web. They show the model, not sensor feedback.
        The photographed home pose is approximated. Link lengths are 120 mm, 120 mm, and 175 mm; joint offsets are estimates. Not suitable for collision-safety checks.
    </p>
    <div class="views">
        <div class="view"><h3>Y–X · Top view</h3><canvas id="viewXY"></canvas></div>
        <div class="view"><h3>X–Z · Side view</h3><canvas id="viewXZ"></canvas></div>
        <div class="view"><h3>Y–Z · Front view</h3><canvas id="viewYZ"></canvas></div>
    </div>
    <div class="legend">
        <span style="--dot:#62a5ff">Base / shoulder</span>
        <span style="--dot:#65d6a6">Link 1</span>
        <span style="--dot:#ffbf69">Link 2</span>
        <span style="--dot:#b79cff">Wrist / tool</span>\n        <span style="--dot:#ff6b6b">End point</span>
    </div>
    <p id="modelStatus" class="status">Waiting for angle data…</p>
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
    document.getElementById("autoMove").disabled = busy;
    document.getElementById("autoCurrent").disabled = busy;
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

const coordinateIds = ["x", "y", "z", "pitch"];
let validTarget = null;
let validationSerial = 0;
let validationTimer = null;
let validating = false;

function readTarget() {
    return {
        x: Number(document.getElementById("x").value),
        y: Number(document.getElementById("y").value),
        z: Number(document.getElementById("z").value),
        tool_pitch: Number(document.getElementById("pitch").value),
        elbow_up: document.getElementById("elbowUp").checked
    };
}

function displayTarget() {
    for (const id of coordinateIds) {
        document.getElementById(id + "Value").textContent =
            document.getElementById(id).value;
    }
}

function restoreValidTarget() {
    if (!validTarget) return;
    for (const id of ["x", "y", "z"]) {
        document.getElementById(id).value = validTarget[id];
    }
    document.getElementById("pitch").value = validTarget.tool_pitch;
    document.getElementById("elbowUp").checked = validTarget.elbow_up;
    displayTarget();
}

async function validateTarget() {
    const serial = ++validationSerial;
    const target = readTarget();
    validating = true;
    setBusy(true);
    document.getElementById("ikStatus").textContent = "Checking reachability...";
    try {
        await postJSON("/api/ik/check", target);
        if (serial !== validationSerial) return;
        validTarget = target;
        document.getElementById("ikStatus").textContent = "Reachable target.";
    } catch (error) {
        if (serial !== validationSerial) return;
        restoreValidTarget();
        document.getElementById("ikStatus").textContent =
            "Unreachable position. Reverted: " + error.message;
    } finally {
        if (serial === validationSerial) {
            validating = false;
            setBusy(false);
        }
    }
}

function scheduleValidation() {
    displayTarget();
    clearTimeout(validationTimer);
    validating = true;
    setBusy(true);
    validationTimer = setTimeout(validateTarget, 160);
}

for (const id of coordinateIds) {
    document.getElementById(id).addEventListener("input", scheduleValidation);
}
document.getElementById("elbowUp").addEventListener("change", scheduleValidation);

document.getElementById("zero").addEventListener("click", async () => {
    clearTimeout(validationTimer);
    ++validationSerial;
    validating = true;
    setBusy(true);
    try {
        const response = await fetch("/api/status", {cache: "no-store"});
        const data = await response.json();
        if (!data.xyz || !data.angles) throw new Error("No position data");
        for (const [i, id] of ["x", "y", "z"].entries()) {
            document.getElementById(id).value = Math.round(data.xyz[i]);
        }
        const a = data.angles;
        const q2 = 170 + (a.shoulder - 180) - 120 - (a.elbow - 180);
        document.getElementById("pitch").value =
            Math.round(q2 - 50 + (a.wrist - 20) * 90 / 124);
        displayTarget();
        validTarget = readTarget();
        document.getElementById("ikStatus").textContent =
            "Current XYZ loaded. Adjust sliders to check new targets.";
    } catch (error) {
        restoreValidTarget();
        document.getElementById("ikStatus").textContent = error.message;
    } finally {
        validating = false;
        setBusy(false);
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

    if (validating || !validTarget) {
        status.textContent = "Wait for coordinate validation or load current XYZ.";
        return;
    }
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

async function fillAutoCurrent() {
    try {
        const response = await fetch("/api/status", {cache: "no-store"});
        if (!response.ok) throw new Error("Cannot read robot position");
        const data = await response.json();
        if (!data.xyz) throw new Error("No position data");
        ["autoX", "autoY", "autoZ"].forEach((id, i) => {
            document.getElementById(id).value = Math.round(data.xyz[i]);
        });
        document.getElementById("autoStatus").textContent = "Current position loaded.";
    } catch (error) {
        document.getElementById("autoStatus").textContent = error.message;
    }
}
document.getElementById("autoCurrent").addEventListener("click", fillAutoCurrent);
document.getElementById("autoMove").addEventListener("click", async () => {
    const status = document.getElementById("autoStatus");
    const xyz = ["autoX", "autoY", "autoZ"].map(id =>
        Number(document.getElementById(id).value));
    if (xyz.some(v => !Number.isFinite(v)) ||
        ["autoX", "autoY", "autoZ"].some(id => document.getElementById(id).value.trim() === "")) {
        status.textContent = "Enter valid X, Y and Z.";
        return;
    }
    setBusy(true);
    status.textContent = "Searching for reachable arm pose...";
    try {
        const result = await postJSON("/api/auto", {
            x: xyz[0], y: xyz[1], z: xyz[2]
        });
        for (const name of ["base", "shoulder", "elbow", "wrist"]) {
            angles[name] = result.angles[name];
            document.getElementById("joint-" + name).value = angles[name];
            document.getElementById("value-" + name).textContent = angles[name] + "°";
        }
        status.textContent = "Reached target in model. Tool pitch: " +
            result.pitch.toFixed(1) + "°. Check actual position.";
        fillAutoCurrent();
    } catch (error) {
        status.textContent = "Automatic move failed: " + error.message;
    } finally {
        setBusy(false);
    }
});

buildJointControls();
document.getElementById("move").disabled = true;
document.getElementById("zero").click();

const LINK1_MM = 120;
const LINK2_MM = 120;
let lastModelAngles = {...angles};

// Approximate geometry inferred from the photographed home pose.
// X points toward the gripper/front (down in top view), Y toward the right in front view, Z upward.
// These offsets affect the drawing only, not motor commands or Cartesian IK.
const HOME_SHOULDER_DEG = 170;
const HOME_ELBOW_BEND_DEG = -120;
const WRIST_LENGTH_MM = 175;
const HOME_WRIST_PITCH_DEG = 0;
const WRIST_SCALE = 90 / 124;

function armPoints(a) {
    const yaw = (a.base - 95) * Math.PI / 180;
    const shoulder = (HOME_SHOULDER_DEG + (a.shoulder - 180)) * Math.PI / 180;
    const elbow = shoulder + (HOME_ELBOW_BEND_DEG - (a.elbow - 180)) * Math.PI / 180;
    const wrist = elbow + (HOME_WRIST_PITCH_DEG - (HOME_SHOULDER_DEG + HOME_ELBOW_BEND_DEG) + (a.wrist - 20) * WRIST_SCALE) * Math.PI / 180;

    const planar = [
        {r: 0, z: 0},
        {r: LINK1_MM * Math.cos(shoulder), z: LINK1_MM * Math.sin(shoulder)}
    ];
    planar.push({
        r: planar[1].r + LINK2_MM * Math.cos(elbow),
        z: planar[1].z + LINK2_MM * Math.sin(elbow)
    });
    planar.push({
        r: planar[2].r + WRIST_LENGTH_MM * Math.cos(wrist),
        z: planar[2].z + WRIST_LENGTH_MM * Math.sin(wrist)
    });
    return planar.map(p => ({
        x: p.r * Math.cos(yaw),
        y: p.r * Math.sin(yaw),
        z: p.z
    }));
}

function drawProjection(canvasId, points, horizontal, vertical, horizontalLabel, verticalLabel, verticalDown = false) {
    const canvas = document.getElementById(canvasId);
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(rect.width * dpr));
    canvas.height = Math.max(1, Math.round(rect.height * dpr));
    const ctx = canvas.getContext("2d");
    ctx.scale(dpr, dpr);
    const w = rect.width, h = rect.height;
    ctx.clearRect(0, 0, w, h);

    const values = points.map(p => [p[horizontal], p[vertical]]);
    const extent = LINK1_MM + LINK2_MM + WRIST_LENGTH_MM + 25;
    const scale = Math.min((w - 54) / (2 * extent), (h - 54) / (2 * extent));
    const ox = w / 2, oy = h / 2;
    const px = v => ox + v * scale;
    const py = v => oy + (verticalDown ? v : -v) * scale;

    ctx.strokeStyle = "#303b4b";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(18, oy); ctx.lineTo(w - 18, oy);
    ctx.moveTo(ox, 18); ctx.lineTo(ox, h - 18);
    ctx.stroke();

    ctx.fillStyle = "#aab5c5";
    ctx.font = "12px system-ui";
    ctx.fillText(horizontalLabel, w - 32, oy - 8);
    ctx.fillText(verticalLabel, ox + 8, 16);
    ctx.fillText("0", ox + 5, oy + 14);

    const segmentColors = ["#65d6a6", "#ffbf69", "#b79cff"];
    ctx.lineWidth = 5;
    ctx.lineCap = "round";
    for (let i = 0; i < values.length - 1; i++) {
        ctx.strokeStyle = segmentColors[i];
        ctx.beginPath();
        ctx.moveTo(px(values[i][0]), py(values[i][1]));
        ctx.lineTo(px(values[i + 1][0]), py(values[i + 1][1]));
        ctx.stroke();
    }

    const colors = ["#62a5ff", "#65d6a6", "#ffbf69", "#ff6b6b"];
    points.forEach((p, i) => {
        ctx.beginPath();
        ctx.fillStyle = colors[i];
        ctx.arc(px(p[horizontal]), py(p[vertical]), i === points.length - 1 ? 6 : 5, 0, Math.PI * 2);
        ctx.fill();
    });
}

function renderArmModel(a) {
    const points = armPoints(a);
    drawProjection("viewXY", points, "y", "x", "Y", "X", true);
    drawProjection("viewXZ", points, "x", "z", "X", "Z");
    drawProjection("viewYZ", points, "y", "z", "Y", "Z");
    document.getElementById("modelStatus").textContent =
        "Base " + a.base + "° · Shoulder " + a.shoulder +
        "° · Elbow " + a.elbow + "° · Endpoint relative to base: X " +
        points[3].x.toFixed(1) + " mm, Y " + points[3].y.toFixed(1) +
        " mm, Z " + points[3].z.toFixed(1) + " mm.";
}

async function refreshModel() {
    try {
        const response = await fetch("/api/status", {cache: "no-store"});
        if (!response.ok) return;
        const result = await response.json();
        if (result.connected && result.angles) {
            lastModelAngles = {...lastModelAngles, ...result.angles};
            renderArmModel(lastModelAngles);
        }
    } catch (_) {}
}

renderArmModel(lastModelAngles);
refreshModel();
setInterval(refreshModel, 500);
window.addEventListener("resize", () => renderArmModel(lastModelAngles));
</script>
</body>
</html>
"""


def get_current_xyz():
    return forward_kinematics(
        arm.base.angle,
        arm.shoulder.angle,
        arm.elbow.angle,
        arm.wrist.angle,
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
    current_q2 = 170.0 + (arm.shoulder.angle - 180.0) - 120.0 - (arm.elbow.angle - 180.0)
    start_pitch = current_q2 - 50.0 + (arm.wrist.angle - 20.0) * (90.0 / 124.0)
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
            tool_pitch=start_pitch + (pitch - start_pitch) * t,
            elbow_up=elbow_up,
        )

        path.append({
            "base": solution.base,
            "shoulder": solution.shoulder,
            "elbow": solution.elbow,
            "wrist": solution.wrist,
        })

    return path


def plan_auto_path(target):
    """Find a reachable tool pitch and interpolate servo angles to the pose.

    This deliberately does not guarantee straight-line Cartesian travel.
    It cannot detect collisions or mechanical interference.
    """
    x, y, z = (float(target[k]) for k in ("x", "y", "z"))
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise IKError("XYZ must be finite.")
    if math.sqrt(x*x + y*y + z*z) > 415.01:
        raise IKError("Target is beyond the 415 mm maximum theoretical reach.")

    current = get_current_angles()
    q2 = 170.0 + (current["shoulder"] - 180.0) - 120.0 - (current["elbow"] - 180.0)
    current_pitch = q2 - 50.0 + (current["wrist"] - 20.0) * 90.0 / 124.0

    # First try orientations near the current tool direction to avoid
    # unnecessarily large wrist and shoulder movements.
    pitches = sorted(
        [float(p) for p in range(-180, 181, 3)] + [current_pitch],
        key=lambda p: abs(p - current_pitch)
    )
    candidates = []
    for pitch in pitches:
        for elbow_up in (False, True):
            try:
                solution = inverse_kinematics(
                    x, y, z, tool_pitch=pitch, elbow_up=elbow_up
                )
            except IKError:
                continue
            joints = {
                name: getattr(solution, name)
                for name in IK_JOINTS
            }
            delta = [abs(joints[n] - current[n]) for n in IK_JOINTS]
            score = sum(d*d for d in delta) + 0.03 * (pitch-current_pitch)**2
            candidates.append((score, joints, pitch))

    if not candidates:
        raise IKError(
            "No reachable joint configuration for this XYZ within servo limits."
        )
    _, final, pitch = min(candidates, key=lambda item: item[0])
    steps = max(1, math.ceil(max(
        abs(final[n] - current[n]) for n in IK_JOINTS
    ) / AUTO_MAX_STEP_DEG))
    path = []
    for index in range(1, steps + 1):
        t = index / steps
        # Smooth start and finish.
        t = t*t*(3 - 2*t)
        path.append({
            n: round(current[n] + (final[n] - current[n]) * t, 2)
            for n in IK_JOINTS
        })
    return path, pitch


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

            if path == "/api/auto":
                # Reserve the arm while planning and executing to prevent
                # simultaneous commands from another browser request.
                with arm_lock:
                    if moving:
                        self.send_json(409, {"error": "Arm is already moving."})
                        return
                    path_angles, chosen_pitch = plan_auto_path(data)
                    moving = True
                try:
                    for step in path_angles:
                        with arm_lock:
                            arm.write(**step)
                        time.sleep(STEP_DELAY)
                finally:
                    with arm_lock:
                        moving = False
                self.send_json(200, {
                    "ok": True,
                    "angles": path_angles[-1],
                    "pitch": chosen_pitch,
                })
                return

            if path == "/api/ik/check":
                with arm_lock:
                    if moving:
                        self.send_json(409, {"error": "Arm is moving."})
                        return
                    path_angles = plan_cartesian_path(data)
                self.send_json(200, {
                    "ok": True,
                    "angles": path_angles[-1] if path_angles else {},
                })
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