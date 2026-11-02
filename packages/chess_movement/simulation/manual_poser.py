# ============================================================
# manual_poser.py
# Purpose: Spawn the full chess scene and pose the robot by hand with sliders.
#
# Run from: My Code/Complete Chess/Chess Movement/
#     python simulation/manual_poser.py            (viewer + slider panel)
#     python simulation/manual_poser.py --no-viewer   (panel only, headless)
#
# Opens the Genesis viewer (drag to orbit) plus a joint-slider panel at
# http://127.0.0.1:8014 — open that URL in any browser. Every slider is one
# joint, ranged to its URDF limits; the arm follows the sliders live through
# the same PD position servos the movement system uses. Buttons snap to the
# movement system's known poses (zero / CANONICAL / UP).
# ============================================================
import argparse
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import numpy as np
import genesis as gs

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))  # Chess Movement/
from simulation.OLD_genesis_setup import GenesisSetup

PORT = 8014

# Slider grouping / ordering for the panel (anything unlisted lands in "other").
GROUPS = [
    ("torso", ["chestturn", "waistlean", "abdominalcrunch", "neckturn", "headnod"]),
    ("right arm", ["r_shoulder_x", "r_shoulder_y", "r_elbow_x",
                   "r_wrist_x", "r_wrist_z", "r_magnet"]),
    ("left arm", ["l_shoulder_x", "l_shoulder_y", "l_elbow_x",
                  "l_wrist_x", "l_wrist_z", "l_magnet"]),
    ("legs", ["hiprotate_left", "thighlift_left", "knee_left", "ankle_left",
              "hiplift_right", "hiprotate_right", "thighlift_right", "knee_right",
              "ankle_right"]),
]

# Pose presets copied from OLD_movement_system.py (right arm, r_shoulder_x..r_wrist_x).
RIGHT_ARM = ["r_shoulder_x", "r_shoulder_y", "r_elbow_x",
             "r_wrist_z", "r_wrist_x"]
POSES = {
    "canonical": dict(zip(RIGHT_ARM, [1.0, 0.0, 1.0, 0.0, 0.5])),   # CANONICAL_ARM
    "up": dict(zip(RIGHT_ARM, [0.0, 1.5, 1.0, 0.0, 0.0])),          # UP_POSTURE
}

_HTML = """<!doctype html><html><head><meta charset="utf-8"><title>MQP joint poser</title>
<style>
body{background:#14161a;color:#e8e8ea;font-family:system-ui;padding:18px;max-width:780px}
h1{font-size:18px;margin:0 0 4px}p{color:#9aa0a6;font-size:13px;margin:0 0 12px}
.grp{margin:16px 0 4px;font-weight:700;color:#7fb3ff;border-bottom:1px solid #333;padding-bottom:2px}
.row{display:flex;align-items:center;gap:10px;margin:5px 0}
.row label{width:190px;font-size:13px}
.row input[type=range]{flex:1;accent-color:#4ea1ff}
.row .val{width:60px;text-align:right;font-family:monospace;font-size:13px;color:#9fe8a0}
button{background:#2b6cb0;color:#fff;border:0;border-radius:6px;padding:8px 14px;margin:0 8px 8px 0;font-size:14px;cursor:pointer}
button:hover{background:#3182ce}
</style></head><body>
<h1>MQP &mdash; manual joint poser</h1>
<p>Sliders command joint targets (rad; rack = m). Green number = actual measured joint angle.</p>
<div>
<button onclick="pose('zero')">Zero (rack retracted)</button>
<button onclick="pose('canonical')">CANONICAL arm</button>
<button onclick="pose('up')">UP posture</button>
</div>
<div id="rows"></div>
<script>
const joints = __SLOTS__;
const rows = document.getElementById('rows');
let lastgrp = null;
for (const j of joints) {
    if (j.group !== lastgrp) {
        const g = document.createElement('div'); g.className = 'grp';
        g.textContent = j.group; rows.appendChild(g); lastgrp = j.group;
    }
    const d = document.createElement('div'); d.className = 'row';
    d.innerHTML = `<label>${j.name}</label>` +
        `<input type=range min=${j.lo} max=${j.hi} step=0.01 id=s${j.i}>` +
        `<span class=val id=v${j.i}></span>`;
    rows.appendChild(d);
    d.querySelector('input').addEventListener('input',
        e => fetch(`/set?i=${j.i}&v=${e.target.value}`));
}
let first = true;
setInterval(async () => {
    try {
        const s = await (await fetch('/state')).json();
        for (const j of joints) {
            document.getElementById('v' + j.i).textContent = (s.actual[j.i] ?? 0).toFixed(2);
            if (first) document.getElementById('s' + j.i).value = s.target[j.i];
        }
        first = false;
    } catch (e) {}
}, 150);
function pose(p) { fetch('/button?name=' + p); }
</script></body></html>"""


def motor_info(robot):
    """(Helper) List the robot's actuated dofs: local indices, joint names, and URDF limits."""
    dofs, names = [], []
    for joint in robot.joints:
        idx = joint.dofs_idx_local
        if idx and joint.type != gs.JOINT_TYPE.FREE and joint.type != gs.JOINT_TYPE.FIXED:
            dofs += [int(i) for i in idx]
            names += [joint.name] * len(idx) if len(idx) == 1 else \
                     [f"{joint.name}_{k}" for k in range(len(idx))]
    lo, hi = (np.asarray(t.detach().cpu(), dtype=float) for t in robot.get_dofs_limit(dofs))
    lo = np.where(np.isfinite(lo), lo, -np.pi)
    hi = np.where(np.isfinite(hi), hi, np.pi)
    return dofs, names, lo, hi


def slots_of(names, lo, hi):
    """(Helper) Panel layout: one slot dict per dof, ordered by GROUPS, with its group label."""
    order = {n: k for _, js in GROUPS for k, n in enumerate(js)}
    gidx = {g: k for k, (g, _) in enumerate(GROUPS)}
    keys = sorted(range(len(names)), key=lambda i: (
        gidx.get(next((g for g, js in GROUPS if names[i] in js), "other"), len(GROUPS)),
        order.get(names[i], 0), names[i]))
    group = lambda n: next((g for g, js in GROUPS if n in js), "other")
    return [{"i": i, "name": names[i], "lo": round(float(lo[i]), 3),
             "hi": round(float(hi[i]), 3), "group": group(names[i])} for i in keys]


def make_handler(state, lock, names, lo, hi, rack_dofs):
    """(Helper) Build the HTTP handler serving the slider panel and joint commands."""

    def apply_pose(pose_name):
        with lock:
            if pose_name == "zero":
                state["target"][:] = np.clip(0.0, lo, hi)
                for i in rack_dofs:                       # rack: upper limit = retracted
                    state["target"][i] = hi[i]
            else:
                for jn, val in POSES[pose_name].items():
                    i = names.index(jn)
                    state["target"][i] = float(np.clip(val, lo[i], hi[i]))

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            u = urlparse(self.path)
            q = parse_qs(u.query)
            if u.path == "/":
                body = _HTML.replace("__SLOTS__", json.dumps(state["slots"])).encode()
                self._send("text/html", body)
            elif u.path == "/set":
                i, v = int(q["i"][0]), float(q["v"][0])
                with lock:
                    state["target"][i] = float(np.clip(v, lo[i], hi[i]))
                self._send("application/json", b'{"ok":true}')
            elif u.path == "/state":
                with lock:
                    body = json.dumps({"target": state["target"].tolist(),
                                       "actual": state["actual"].tolist()}).encode()
                self._send("application/json", body)
            elif u.path == "/button" and q.get("name", [""])[0] in ("zero", *POSES):
                apply_pose(q["name"][0])
                self._send("application/json", b'{"ok":true}')
            else:
                self.send_error(404)

        def _send(self, ctype, body):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *a):   # keep the terminal clean
            pass

    return Handler


def main():
    """Spawn the chess scene, serve the slider panel, and step the sim until Ctrl-C."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-viewer", action="store_true", help="headless (slider panel only)")
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))          # simulation/
    urdf = os.path.join(here, "master_magnet_hand_v5", "urdf", "master_magnet_hand_v5.urdf")

    sim = GenesisSetup(urdf, show_viewer=not args.no_viewer)
    sim.spawn_plane()
    sim.spawn_board(sim.board_length)
    sim.spawn_grave_yard("right")
    sim.spawn_grave_yard("left")
    sim.spawn_pieces()
    sim.spawn_robot()
    sim.scene.build()

    # Same servo gains the movement system uses, so the arm behaves identically.
    n = sim.robot.n_dofs
    sim.robot.set_dofs_kp(np.full(n, 500.0))
    sim.robot.set_dofs_kv(np.full(n, 25.0))
    sim.robot.set_dofs_force_range(np.full(n, -200.0), np.full(n, 200.0))

    dofs, names, lo, hi = motor_info(sim.robot)
    rack_dofs = [i for i, nm in enumerate(names) if nm.endswith("_magnet")]
    target = np.clip(np.zeros(len(dofs)), lo, hi)
    for i in rack_dofs:
        target[i] = hi[i]
    state = {"target": target, "actual": target.copy(), "slots": slots_of(names, lo, hi)}
    lock = threading.Lock()

    server = ThreadingHTTPServer(("127.0.0.1", args.port),
                                 make_handler(state, lock, names, lo, hi, rack_dofs))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"\nJoint panel:  http://127.0.0.1:{args.port}   (open in a browser)")
    print("Ctrl-C to quit.\n")

    try:                                        # main loop: sliders -> servos -> physics
        while True:
            with lock:
                q = target.copy()
            sim.robot.control_dofs_position(q, dofs)
            sim.scene.step()
            with lock:
                state["actual"] = np.asarray(sim.robot.get_dofs_position(dofs).detach().cpu(), dtype=float)
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()