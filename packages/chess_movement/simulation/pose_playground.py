# ============================================================
# pose_playground.py
# Purpose: Interactive pose inspection. Builds the chess scene, then lets you
#          step individual arm joints from the terminal keyboard while watching
#          the Genesis viewer, and prints the end-effector (finger) coordinates
#          after every move.
#
# Run from the 'simulation' directory:
#   python pose_playground.py            # interactive (viewer + keyboard)
#   python pose_playground.py --smoke    # headless build + one pose print
#
# Terminal commands (type a line, press Enter):
#   1..10   select joint (1-5 right arm, 6-10 left arm)
#   + / -   step selected joint by +/- step size
#   s 5     set step size to 5 degrees (default 2)
#   l r|l   switch which arm's finger pose is printed
#   p       print finger position + joint angles
#   n       run sim a bit longer (settle / watch) without moving anything
#   q       quit
# ============================================================
import os
import sys

import numpy as np

import genesis as gs  # noqa: E402  (genesis_setup silences torch warning first)
from genesis_setup import GenesisSetup

RIGHT_ARM_JOINTS = [
    "shoulderspin_right", "biceplift_right", "elbow_right",
    "wristspin_right", "handcurl_right",
]
LEFT_ARM_JOINTS = [
    "shoulderspin_left", "biceplift_left", "elbow_left",
    "wristspin_left", "handcurl_left",
]


class PosePlayground:

    def __init__(self, show_viewer=True):
        _DIR = os.path.dirname(os.path.abspath(__file__))
        urdf = os.path.join(_DIR, "master_magnet_hand_v5", "urdf",
                            "master_magnet_hand_v5.urdf")
        self.sim = GenesisSetup(urdf, show_viewer=show_viewer)
        self.sim.spawn_plane()
        self.sim.spawn_board(self.sim.board_length)
        self.sim.spawn_grave_yard("right")
        self.sim.spawn_grave_yard("left")
        self.sim.spawn_pieces()
        self.robot = self.sim.spawn_robot()
        self.sim.scene.build()

        # Same motor gains as the settle routine in genesis_setup.setup()
        n = self.robot.n_dofs
        self.robot.set_dofs_kp(np.full(n, 60.0))
        self.robot.set_dofs_kv(np.full(n, 4.0))
        self.robot.set_dofs_force_range(np.full(n, -10.0), np.full(n, 10.0))

        # Map joint names -> local dofs indices (same walk as ArmController)
        self.joints = {}          # display name -> [dof idx local]
        for side, names in (("R", RIGHT_ARM_JOINTS), ("L", LEFT_ARM_JOINTS)):
            for name in names:
                idxs = []
                for joint in self.robot.joints:
                    if joint.name == name and joint.n_dofs > 0:
                        idxs.extend(joint.dofs_idx_local)
                if not idxs:
                    raise ValueError(f"joint '{name}' not found in URDF")
                self.joints[f"{side}:{name}"] = idxs

        self.names = list(self.joints.keys())
        self.sel = 0                       # selected joint index
        self.step_deg = 2.0                # keyboard step size
        self.arm = "right"                 # which finger pose to print
        # Hold targets in radians, seeded from current pose after settling.
        self.target = np.zeros(self.robot.n_dofs)

    # -- helpers ---------------------------------------------------------

    def settle(self, seconds=3.0):
        steps = int(seconds / self.sim.scene.dt)
        home = np.zeros(self.robot.n_dofs)
        for _ in range(steps):
            self.robot.control_dofs_position(home)
            self.sim.scene.step()
        self.target = self.robot.get_dofs_position().cpu().numpy().copy()

    def run_steps(self, seconds=0.6):
        steps = int(seconds / self.sim.scene.dt)
        for _ in range(steps):
            self.robot.control_dofs_position(self.target)
            self.sim.scene.step()

    def print_pose(self):
        link = self.robot.get_link(f"finger_{self.arm}")
        pos = link.get_pos().cpu().numpy().reshape(-1)
        quat = link.get_quat().cpu().numpy().reshape(-1)   # (w, x, y, z)
        root = self.robot.get_pos().cpu().numpy().reshape(-1)
        rel = pos - root
        print(f"\n=== finger_{self.arm} ===")
        print(f"world pos  : x={pos[0]*1000:8.1f}  y={pos[1]*1000:8.1f}  z={pos[2]*1000:8.1f}  mm")
        print(f"rel-to-base: x={rel[0]*1000:8.1f}  y={rel[1]*1000:8.1f}  z={rel[2]*1000:8.1f}  mm")
        print(f"quat wxyz  : {quat[0]:.3f} {quat[1]:.3f} {quat[2]:.3f} {quat[3]:.3f}")
        print("--- joint angles (deg) ---")
        for i, name in enumerate(self.names):
            idxs = self.joints[name]
            deg = np.degrees(self.target[idxs]).mean()
            mark = ">" if i == self.sel else " "
            print(f"{mark}{i+1:2d} {name:28s} {deg:8.1f}")
        print()

    # -- main loop -------------------------------------------------------

    def repl(self):
        print("\ncommands: 1-10 select | + / - step | s <deg> | l r|l | p | n | q")
        while True:
            try:
                line = input("pose> ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0]
            if cmd == "q":
                break
            elif cmd == "p":
                self.print_pose()
            elif cmd == "n":
                self.run_steps(2.0)
            elif cmd in "+-":
                idxs = self.joints[self.names[self.sel]]
                self.target[idxs] += np.radians(self.step_deg) * (1 if cmd == "+" else -1)
                self.run_steps()
                self.print_pose()
            elif cmd.isdigit() and 1 <= int(cmd) <= len(self.names):
                self.sel = int(cmd) - 1
                print(f"selected {self.names[self.sel]}")
            elif cmd == "s" and len(parts) > 1:
                self.step_deg = float(parts[1])
                print(f"step size = {self.step_deg} deg")
            elif cmd == "l" and len(parts) > 1:
                self.arm = "left" if parts[1].startswith("l") else "right"
            else:
                print("unknown command")


def main():
    smoke = "--smoke" in sys.argv
    pg = PosePlayground(show_viewer=not smoke)
    pg.settle(1.0 if smoke else 3.0)
    if smoke:
        idxs = pg.joints[pg.names[1]]           # nudge biceplift_right
        pg.target[idxs] += np.radians(10)
        pg.run_steps(0.5)
        pg.print_pose()
        print("SMOKE OK")
        return
    pg.print_pose()
    pg.repl()


if __name__ == "__main__":
    main()