# ============================================================
# OLD_movement_system.py
# Purpose: Establish basic movement system for grasping and placing the end-effector, and moving it in 3d Space.
#
# Pipeline:  waypoint()   -> hover pose every turn starts from (hand-verified pad-down pose)
#            move_to()    -> waypoint-relative IK: the 5 arm joints are re-solved per target so the
#                            PAD lands on target XY at hover height with the pad vertical. The
#                            rack/pinion is NOT touched by move_to — it stays retracted; the hover
#                            sits HOVER_ABOVE_CROWN above the grasp point so the rack can close the
#                            remaining gap.
#            grasp()      -> (unimplemented) rack press-down + magnet weld live here
#            release()    -> (unimplemented) rack retract + magnet weld release live here
#            home()       -> safe rest pose (arm zero), routed via UP_POSTURE
#
# The solver (_solve_hover) is a weighted damped-least-squares on the 5 arm joints using Genesis FK
# only (no sim state touched):
#   XY error  — hard objective (the thing move_to must nail)
#   hover Z   — soft, pulls the pad to crown + HOVER_ABOVE_CROWN (mid rack stroke)
#   tilt      — weighted, keeps the rack axis vertical so its press travels straight down
# Multi-start seeds: the waypoint pose plus the two hand-verified corner poses (the far-corner
# solutions only converge from those basins). Verified on the full right half + right graveyard
# at king-crown height: 48/48 targets, XY <= 0.1 mm, tilt <= 1.3 deg, rack press 15.1..16.3 mm
# (rack travel is 32 mm).

import numpy as np
import os
import chess
from simulation.OLD_genesis_setup import GenesisSetup


def _np(x):
    """(Helper) Convert torch tensors (cpu/cuda) or arrays to a float64 numpy array."""
    if hasattr(x, "detach"):
        x = x.detach()
    if hasattr(x, "cpu"):
        x = x.cpu()
    if hasattr(x, "numpy"):
        x = x.numpy()
    return np.asarray(x, dtype=float)


class MovementSystem:

    # ---- Motion limits (joint space) ------------------------------------
    ARM_SPEED = 0.5          # rad/s cruise (heavy arm: keep inside servo tracking bandwidth)
    ARM_ACCEL = 1.5          # rad/s^2
    DT = 0.005               # physics step (matches GenesisSetup RigidOptions dt)

    # ---- Geometry --------------------------------------------------------
    PAD_LOCAL = (0.083, 0.0, -0.0075)        # pad face in finger-link frame
    PAD_DOWN_QUAT = (0.5, -0.5, 0.5, 0.5)    # (w,x,y,z) finger +x axis -> world -z (rack moves straight up/down)
    RACK_RETRACTED = 0.032   # r_magnet value with the pad at its HIGHEST (0 = fully extended/down)
    HOVER_ABOVE_CROWN = 0.016                # move_to parks the pad this far above the grasp point (mid rack stroke)

    # ---- Poses (r_shoulder_x, r_shoulder_y, r_elbow_x, r_wrist_z, r_wrist_x) ----
    CANONICAL_ARM = (1.0, 0.0, 1.0, 0.0, 0.5)    # verified pad-down posture, clear of the board
    UP_POSTURE = (0.0, 1.5, 1.0, 0.0, 0.0)       # stopover so the wrist clears the graveyard slab
    # Hover waypoint every turn starts from — hand-posed in the slider panel (Positions/
    # screenshots): pad hovers over the graveyard strip, rack retracted, tilt ~0.5 deg.
    WAYPOINT_ARM = (1.62, -0.03, -1.57, -0.02, -1.53)
    # Corner hover poses, hand-verified as proof the full right-half range exists.
    # Also used as solver seeds: the far-corner solutions only converge from these basins.
    WAYPOINT_TOP = (1.09, 0.0, -0.16, 0.13, -0.69)
    WAYPOINT_BOTTOM = (1.59, 0.26, -1.53, 2.12, -1.53)
    IK_SEEDS = (WAYPOINT_ARM, WAYPOINT_TOP, WAYPOINT_BOTTOM)

    # ---- Solver weights / tolerances -------------------------------------
    W_HOVER = 0.5            # soft weight on hover height (rack absorbs Z, so it's a preference)
    W_TILT = 0.3             # weight on pad-axis tilt (rad per rad of tilt)
    DLS_LAMBDA = 1e-4        # damped-least-squares damping
    DLS_STEP = 0.15          # rad, max joint step per iteration
    DLS_ITERS = 150
    JOINT_LO, JOINT_HI = -2.618, 2.618         # URDF arm limits
    SKIP_TOL = 0.05          # m, refuse the move if the solver can't get the pad's XY this close
    TILT_LIMIT_DEG = 20.0    # above this, the arm is not in the pad-down basin (entry guard)
    SETTLE_TOL = 0.01        # rad, "arrived" threshold
    SETTLE_MAX_TICKS = 600   # give up waiting after this many ticks

    ARM_JOINTS = ["r_shoulder_x", "r_shoulder_y", "r_elbow_x",
                  "r_wrist_z", "r_wrist_x"]
    RACK_JOINT = "r_magnet"
    FINGER_LINK = "finger_right"

    def __init__(self, robot, scene):
        """
        Constructor: Initialize the class variables.

        :param robot: Genesis robot entity (or None for dry tests).
        :param scene: Genesis scene — required for stepped motions (move_to()).
        """
        self._robot = robot
        self._scene = scene
        self.current_position = None     # last commanded pad (X, Y, Z)
        self.last_xy_error = 0.0         # XY error of the last move_to solve (m)
        self.last_tilt_deg = 0.0         # pad-axis tilt of the last move_to solution (deg)

        if robot is None:
            return

        # Store DOF state: look up indices/links once so every method can reuse them.
        self._arm_dofs = [int(robot.get_joint(n).dofs_idx_local[0]) for n in self.ARM_JOINTS]
        self._rack_dof = int(robot.get_joint(self.RACK_JOINT).dofs_idx_local[0])
        self._finger_link = robot.get_link(self.FINGER_LINK)
        self._link_idx = robot.links.index(self._finger_link)   # for the FK-only solver

    @staticmethod
    def _quat_to_rot(q):
        """(Helper) (w, x, y, z) quaternion -> 3x3 rotation matrix."""
        w, x, y, z = (float(v) for v in q)
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def pad_world_pos(self):
        """(Helper) World position (3,) of the magnet pad face right now."""
        pos = _np(self._finger_link.get_pos())
        R = self._quat_to_rot(_np(self._finger_link.get_quat()))
        return pos + R @ np.asarray(self.PAD_LOCAL)

    def pad_axis_tilt_deg(self):
        """(Helper) Angle between the rack/pad axis and straight down (0 = rack moves vertically)."""
        axis = self._quat_to_rot(_np(self._finger_link.get_quat()))[:, 0]
        return float(np.degrees(np.arccos(np.clip(axis @ np.array([0.0, 0.0, -1.0]), -1.0, 1.0))))

    # ---- Waypoint-relative IK (helper for move_to) ------------------------

    def _fk_pad(self, q_arm, q_rack):
        """(Helper) Pure-FK pad position + tilt for arm joints q_arm at rack value q_rack.
        Touches no sim state, so the solver can evaluate thousands of poses cheaply."""
        q_full = _np(self._robot.get_dofs_position())
        q_full[self._arm_dofs] = q_arm
        q_full[self._rack_dof] = q_rack
        pos, rot = self._robot.forward_kinematics(q_full, links_idx_local=[self._link_idx])
        p = _np(pos).reshape(-1, 3)[0]
        R = self._quat_to_rot(_np(rot).reshape(-1, 4)[0])
        tilt = float(np.degrees(np.arccos(np.clip(R[:, 0] @ np.array([0.0, 0.0, -1.0]), -1.0, 1.0))))
        return p + R @ np.asarray(self.PAD_LOCAL), tilt

    def _solve_hover(self, target_X, target_Y, target_Z):
        """
        Summary: (Helper) Waypoint-relative IK: find the 5 arm joints that put the PAD on
                 (target_X, target_Y) at hover height target_Z + HOVER_ABOVE_CROWN with the pad
                 vertical. Weighted damped least squares — XY hard, hover height soft (the rack
                 absorbs Z), tilt weighted (the rack must press straight down). Multi-started from
                 IK_SEEDS (waypoint + hand-verified corner poses); the best seed is kept by XY
                 error + stroke feasibility + tilt.

        :param target_X: world X for the pad face (XY is the hard objective).
        :param target_Y: world Y for the pad face.
        :param target_Z: world Z of the GRASP point (piece crown); the pad parks above it.
        :return: (q_arm (5,), xy_err (m), tilt (deg), hover_z (m)) best over all seeds.
        """
        q_hover = target_Z + self.HOVER_ABOVE_CROWN
        q_rack = float(_np(self._robot.get_dofs_position())[self._rack_dof])
        # (score, q, xy_err, tilt, hover_z); score=inf seed so any solve wins
        best = (np.inf, np.asarray(self.IK_SEEDS[0], dtype=float), np.inf, 99.0, q_hover)

        for seed_pose in self.IK_SEEDS:
            q = np.asarray(seed_pose, dtype=float).copy()
            for _ in range(self.DLS_ITERS):
                p0, t0 = self._fk_pad(q, q_rack)
                # residual: XY hard, hover soft, tilt in weighted radians
                r = np.array([p0[0] - target_X, p0[1] - target_Y,
                              self.W_HOVER * (p0[2] - q_hover), self.W_TILT * np.radians(t0)])
                if abs(r[0]) < 1e-4 and abs(r[1]) < 1e-4:
                    break
                # numerical Jacobian of the 4-weighted residual wrt the 5 arm joints
                J = np.zeros((4, 5))
                eps = 1e-4
                for c in range(5):
                    qq = q.copy()
                    qq[c] += eps
                    p1, t1 = self._fk_pad(qq, q_rack)
                    J[0, c] = (p1[0] - p0[0]) / eps
                    J[1, c] = (p1[1] - p0[1]) / eps
                    J[2, c] = self.W_HOVER * (p1[2] - p0[2]) / eps
                    J[3, c] = self.W_TILT * np.radians(t1 - t0) / eps
                dq = -np.linalg.solve(J.T @ J + self.DLS_LAMBDA * np.eye(5), J.T @ r)
                dq = np.clip(dq, -self.DLS_STEP, self.DLS_STEP)
                q = np.clip(q + dq, self.JOINT_LO, self.JOINT_HI)

            p, t = self._fk_pad(q, q_rack)
            xy_err = float(np.hypot(p[0] - target_X, p[1] - target_Y))
            stroke = p[2] - target_Z          # rack press needed to reach the grasp point
            # score: XY dominates; penalize stroke outside the rack travel and tilt
            score = (xy_err + 0.3 * max(0.0, stroke - self.RACK_RETRACTED)
                     + 0.3 * max(0.0, -stroke) + 0.002 * t)
            if best is None or score < best[0]:
                best = (score, q, xy_err, t, p[2])

        _, q_best, xy_err, tilt, hover_z = best
        return q_best, xy_err, tilt, hover_z

    # ---- Stepped motion (shared helpers) ----------------------------------

    def movement_profile(self, distance, v_max, a_max, dt=None):
        """
        Summary: (Helper) Build a trapezoidal velocity profile. Returns progress
                 fractions 0..1 sampled every dt: ramp up at a_max, cruise at
                 v_max, ramp down to zero. Falls back to a triangular profile
                 when the move is too short to reach full speed.

        :param distance: total joint distance to travel (abs value).
        :param v_max: cruise speed limit.
        :param a_max: acceleration limit.
        :param dt: control tick (defaults to physics dt).
        :return: list of progress fractions, first motion sample .. 1.0
        """
        dt = self.DT if dt is None else dt
        if distance <= 0:
            return []
        if distance >= v_max * v_max / a_max:          # long enough: trapezoid
            t_a = v_max / a_max
            total = distance / v_max + t_a
        else:                                          # short move: triangle
            t_a = np.sqrt(distance / a_max)
            total = 2.0 * t_a
        n = max(2, int(np.ceil(total / dt)))
        span = total / n                               # per-sample time
        fracs = []
        for k in range(1, n + 1):
            t = k * span
            t_rem = total - t
            if t < t_a:                                # accelerating
                s = 0.5 * a_max * t * t
            elif t_rem < t_a:                          # decelerating
                s = distance - 0.5 * a_max * t_rem * t_rem
            else:                                      # cruising
                v_peak = a_max * t_a
                s = 0.5 * v_peak * t_a + v_peak * (t - t_a)
            fracs.append(min(1.0, s / distance))
        return fracs

    def _settle(self, q_hold):
        """(Helper) Keep commanding q_hold until the arm joints are within SETTLE_TOL (or time runs out)."""
        for _ in range(self.SETTLE_MAX_TICKS):
            self._robot.control_dofs_position(q_hold)
            self._scene.step()
            q_now = _np(self._robot.get_dofs_position())
            if np.max(np.abs(q_now[self._arm_dofs] - q_hold[self._arm_dofs])) < self.SETTLE_TOL:
                break

    def _move_joints(self, q_goal, max_speed=None, dt=None, settle=True):
        """
        Summary: (Helper) Joint-space move: interpolate from the current arm angles to q_goal along the
                 trapezoid profile, one controller command + one sim step per sample. All non-arm joints
                 (torso, legs, RACK) are held at their current values so they don't go limp under gravity.

        :param q_goal: 5 target arm joint angles.
        :param max_speed: cruise speed in rad/s (None = ARM_SPEED).
        :param dt: control tick (None = physics dt).
        :param settle: wait for the servos to arrive before returning.
        :return: NA
        """
        v_max = self.ARM_SPEED if max_speed is None else max_speed
        q_goal = np.asarray(q_goal, dtype=float)

        q_full = _np(self._robot.get_dofs_position())     # snapshot: everything else is held here
        q_start = q_full[self._arm_dofs].copy()
        delta = q_goal - q_start
        distance = float(np.max(np.abs(delta)))           # slowest (farthest) joint sets the pace
        if distance < 1e-6:
            return

        for frac in self.movement_profile(distance, v_max, self.ARM_ACCEL, dt):
            q_cmd = q_full.copy()
            q_cmd[self._arm_dofs] = q_start + frac * delta   # every joint uses the SAME fraction
            self._robot.control_dofs_position(q_cmd)
            self._scene.step()

        if settle:
            q_final = q_full.copy()
            q_final[self._arm_dofs] = q_goal
            self._settle(q_final)

    def _retract_rack(self):
        """(Helper) Drive the rack/pinion to RACK_RETRACTED (pad at its highest) and hold it there.
        move_to keeps the rack retracted so the hover height matches what the solver FK'd; the
        press-down belongs to grasp()."""
        q_full = _np(self._robot.get_dofs_position())
        q_full[self._rack_dof] = self.RACK_RETRACTED
        for _ in range(100):
            self._robot.control_dofs_position(q_full)
            self._scene.step()

    # ---- Public API --------------------------------------------------------

    def move_to(self, target_X, target_Y, target_Z, max_speed=None, dt=None, settle=True):
        """
        Summary: Drive the arm so the PAD sits directly above (target_X, target_Y) at hover height
                 target_Z + HOVER_ABOVE_CROWN, pad vertical. The pose is solved waypoint-relative
                 (see _solve_hover) and reached by a joint-space trapezoid. The rack/pinion is NOT
                 moved — it stays retracted; grasp() owns the press-down. FAILS LOUD: if the solver
                 can't get the pad's XY within SKIP_TOL the move is skipped (returns False) —
                 driving a high-gain PD to an unconverged pose saturates and whips the arm.

        :param target_X: world X of the grasp point (pad will hover above it).
        :param target_Y: world Y of the grasp point.
        :param target_Z: world Z of the grasp point (piece crown).
        :param max_speed: cruise speed in rad/s (None = ARM_SPEED).
        :param dt: control tick (None = physics dt).
        :param settle: wait for the servos to arrive before returning.
        :return: True if the arm moved, False if the goal was unreachable and the move was skipped.
        """
        q_goal, xy_err, tilt, hover_z = self._solve_hover(target_X, target_Y, target_Z)
        self.last_xy_error = xy_err
        self.last_tilt_deg = tilt
        if xy_err > self.SKIP_TOL:
            print(f"[move_to] SKIPPED: ({target_X:.3f}, {target_Y:.3f}) unreachable "
                  f"(XY {xy_err * 1000:.0f} mm off, tilt {tilt:.1f} deg)")
            return False
        if tilt > self.TILT_LIMIT_DEG:
            print(f"[move_to] warning: solution tilt {tilt:.1f} deg at "
                  f"({target_X:.3f}, {target_Y:.3f}) — rack press will not be vertical")
        self._move_joints(q_goal, max_speed, dt, settle)
        self.current_position = (target_X, target_Y, target_Z)
        return True

    def waypoint(self, max_speed=None):
        """
        Summary: Drive the arm to the turn-start hover WAYPOINT_ARM — the hand-verified pad-down
                 hover pose every move_to sequence starts from. Rack is retracted first (the hover
                 height the solver assumes depends on it). Joint-space move entered via UP_POSTURE:
                 a direct swing from rest sweeps the wrist through the graveyard slab and pins the
                 arm mid-swing.

        :param max_speed: cruise speed in rad/s (None = ARM_SPEED).
        :return: NA
        """
        self._retract_rack()
        if self.pad_axis_tilt_deg() > self.TILT_LIMIT_DEG:
            self._move_joints(self.UP_POSTURE, max_speed, settle=False)
            self._move_joints(self.CANONICAL_ARM, max_speed, settle=False)
        self._move_joints(np.asarray(self.WAYPOINT_ARM, dtype=float), max_speed)
        self.current_position = tuple(self.pad_world_pos())

    def home(self, max_speed=None):
        """
        Summary: Drive the arm to the safe rest pose (all arm joints zero, rack retracted),
                 routed via UP_POSTURE so the wrist clears the graveyard slab on the way down.

        :param max_speed: cruise speed in rad/s (None = ARM_SPEED).
        :return: NA
        """
        self._retract_rack()
        self._move_joints(self.UP_POSTURE, max_speed, settle=False)
        self._move_joints(np.zeros(5), max_speed)
        self.current_position = None

    def grasp(self, piece_type=None):
        """
        Summary: (Unimplemented) Press the piece with the rack/pinion and weld the magnet.
        The rack lives HERE, not in move_to: extend r_magnet from RACK_RETRACTED toward
        0 (the pad descends up to 32 mm) onto the piece crown, then add the weld constraint
        (GenesisSetup.magnet_attach). move_to has already parked the pad at XY-on-target,
        crown + HOVER_ABOVE_CROWN, tilt ~0, so the press travels straight down.

        :param piece_type: chess piece type being grasped (reserved for press depth per height).
        :return: NA
        """
        pass

    def release(self, piece_type=None):
        """
        Summary: (Unimplemented) De-energize the magnet and retract the rack.
        Delete the weld constraint (GenesisSetup.magnet_release), then drive r_magnet
        back to RACK_RETRACTED so the pad clears the piece.

        :param piece_type: chess piece type being released (reserved).
        :return: NA
        """
        pass


# (Testing)  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def main(show_viewer=True):
    """
    Summary: Watch-the-arm test: build the scene, waypoint(), then move_to() over every
             right-half tile and every right graveyard slot at king-crown height (the worst case
             for rack stroke). Prints XY error + tilt per stop so each spot can be checked in the
             viewer as the arm visits it.

    :param show_viewer: open the Genesis viewer (False for headless runs).
    :return: NA
    """

    # --- 1. Build the scene (spawn helpers + build only; we settle below instead of using setup()) ---
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # simulation/
    urdf_file = os.path.join(here, "master_magnet_hand_v5", "urdf", "master_magnet_hand_v5.urdf")
    sim = GenesisSetup(urdf_file, show_viewer=show_viewer)
    sim.spawn_plane()
    sim.spawn_board(sim.board_length)
    sim.spawn_grave_yard("right")
    sim.spawn_grave_yard("left")
    sim.spawn_pieces()
    sim.spawn_robot()
    sim.scene.build()

    # Stronger gains than setup()'s defaults (arm sags under gravity otherwise), then settle on the zero pose.
    n = sim.robot.n_dofs
    sim.robot.set_dofs_kp(np.full(n, 500.0))
    sim.robot.set_dofs_kv(np.full(n, 25.0))
    sim.robot.set_dofs_force_range(np.full(n, -200.0), np.full(n, 200.0))
    for _ in range(200):
        sim.robot.control_dofs_position(np.zeros(n))
        sim.scene.step()

    # --- 2. Waypoint first — every move_to starts from the hover pose ---
    movement = MovementSystem(sim.robot, sim.scene)
    for _ in range(int(5 / MovementSystem.DT)):
        sim.scene.step()
    movement.waypoint()
    wp_pad = movement.pad_world_pos()
    print(f"[waypoint] pad=({wp_pad[0]:.3f},{wp_pad[1]:.3f},{wp_pad[2]:.3f}) "
          f"tilt={movement.pad_axis_tilt_deg():.1f}deg")

    # --- 3. Visit every right-half tile + right graveyard slot at king-crown height ---

    # Create list of coords
    BT = sim.BOARD_TOP_Z
    king_h = sim.PIECE_DIMS[chess.KING][1]
    targets = []
    for sq in chess.SQUARES:
        if chess.square_file(sq) >= 4:                       # right half: files e..h
            x, y = sim.square_center(sq)
            targets.append((chess.square_name(sq), x, y, BT + king_h))
    gy_top = BT - 0.003
    for slot in range(sim.GRAVEYARD_SLOTS):
        x, y = sim.graveyard_slot_xy("right", slot)
        targets.append((f"grave{slot}", x, y, gy_top + king_h))

    n_ok = 0

    # move_to() each target
    for name, x, y, z in targets:

        # actual function call
        ok = movement.move_to(x, y, z)
        pad = movement.pad_world_pos()

        # Error calculation
        xy_err = float(np.hypot(pad[0] - x, pad[1] - y))
        tilt = movement.pad_axis_tilt_deg()
        rack = float(_np(sim.robot.get_dofs_position())[movement._rack_dof])
        n_ok += ok
        print(f"[move_to] {name:8s} XY err {xy_err * 1000:5.1f}mm  tilt {tilt:4.1f}deg  "
              f"pad_z {pad[2]:.3f} (crown {z:.3f})  rack {rack:.3f}"
              + ("" if ok else "  <-- SKIPPED"))

        # Pause so the stop is visible
        if show_viewer:
            for _ in range(200):
                sim.scene.step()

    print(f"\nvisited {n_ok}/{len(targets)} targets")

    # Park safely and keep the viewer open so the result can be inspected.
    movement.home()
    if show_viewer:
        for _ in range(6000):
            sim.scene.step()


if __name__ == "__main__":
    main()