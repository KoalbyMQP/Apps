# ============================================================
# movement_system.py
# Purpose: Establish basic movement system for grasping and placing the end-effector, and moving it in 3d Space.
#
# Pipeline:  IK_solve (where do the joints need to end up?)
#            -> movement_profile (how fast, with smooth ramps)
#            -> move_to / _move_joints (send each tiny step to the controller)
#            -> apply_profile (chain moves into lift -> travel -> descend)

import numpy as np
import os
from simulation.genesis_setup import GenesisSetup


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

    # ---- Poses / limits (shoulderspin, biceplift, elbow, wristspin, handcurl)
    CANONICAL_ARM = (1.0, 0.0, 1.0, 0.0, 0.5)   # verified pad-down posture, clear of the board
    UP_POSTURE = (0.0, 1.5, 1.0, 0.0, 0.0)      # stopover so the wrist clears the graveyard slab
    BRANCH_LO = (-1.0, -2.2, 0.2, -2.2, -1.0)   # keeps IK on the elbow-up branch
    BRANCH_HI = (1.0, 2.2, 2.618, 2.2, 1.0)

    # ---- Path planning / settling ----------------------------------------
    SAFE_Z = 0.70            # pad height for travelling over the board
    TILT_LIMIT_DEG = 20.0    # above this, the arm is not in the pad-down basin
    REACH_TOL = 0.005        # m, warn if IK can't get the pad this close
    SETTLE_TOL = 0.01        # rad, "arrived" threshold
    SETTLE_MAX_TICKS = 600   # give up waiting after this many ticks

    ARM_JOINTS = ["shoulderspin_right", "biceplift_right", "elbow_right",
                  "wristspin_right", "handcurl_right"]
    RACK_JOINT = "fingerextend_right"
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

        if robot is None:
            return

        # Store DOF state: look up indices/links once so every method can reuse them.
        self._arm_dofs = [int(robot.get_joint(n).dofs_idx_local[0]) for n in self.ARM_JOINTS]
        self._rack_dof = [int(robot.get_joint(self.RACK_JOINT).dofs_idx_local[0])]
        self._finger_link = robot.get_link(self.FINGER_LINK)
        self._link_idx = robot.links.index(self._finger_link)   # only used for the FK reach check

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


    def IK_solve(self, target_X, target_Y, target_Z, magnet_orientation=None):
        """
        Summary: (Helper) convert target x, y, z, cartesian coords and magnet orientation to list of joint angles
                 with Inverse Kinematics. Always seeded from CANONICAL_ARM so the elbow branch is the same every
                 time, then clamped to BRANCH_LO/HI. Warns if the pad can't reach the goal.

        :param target_X: world X where the pad face should end up.
        :param target_Y: world Y where the pad face should end up.
        :param target_Z: world Z where the pad face should end up.
        :param magnet_orientation: (w, x, y, z) finger quaternion; None = pad-down.
        :return: numpy array of 5 arm joint angles (rad), ordered like ARM_JOINTS.
        """
        quat = self.PAD_DOWN_QUAT if magnet_orientation is None else tuple(magnet_orientation)
        pad_goal = np.array([target_X, target_Y, target_Z], dtype=float)

        # IK solves for the finger LINK origin, not the pad face: back out the pad offset.
        link_goal = pad_goal - self._quat_to_rot(quat) @ np.asarray(self.PAD_LOCAL)

        seed = _np(self._robot.get_dofs_position())
        seed[self._arm_dofs] = self.CANONICAL_ARM

        q_full = _np(self._robot.inverse_kinematics(
            link=self._finger_link,
            pos=tuple(link_goal),
            quat=quat,
            init_qpos=seed,
            dofs_idx_local=self._arm_dofs,
            max_solver_iters=50,
        ))
        q_goal = np.clip(q_full[self._arm_dofs], self.BRANCH_LO, self.BRANCH_HI)

        # Sanity check: clamping (or an unreachable goal) can leave the pad short of the target.
        q_check = q_full.copy()
        q_check[self._arm_dofs] = q_goal
        pos, rot = self._robot.forward_kinematics(q_check, links_idx_local=[self._link_idx])
        pos = _np(pos).reshape(-1, 3)[0]
        R = self._quat_to_rot(_np(rot).reshape(-1, 4)[0])
        err = float(np.linalg.norm(pos + R @ np.asarray(self.PAD_LOCAL) - pad_goal))
        if err > self.REACH_TOL:
            print(f"[IK] warning: pad ends {err * 1000:.1f} mm from goal "
                  f"({target_X:.3f}, {target_Y:.3f}, {target_Z:.3f})")
        return q_goal

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
                 are held at their current values so they don't go limp under gravity.

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


    def move_to(self, target_X, target_Y, target_Z, magnet_orientation=None, max_speed=None, dt=None,
                settle=True):
        """
        Summary: Solve IK for the target, then drive the arm there in joint space along a trapezoid profile.
                 Each micro-step is sent to the controller and ticks the sim by one step.
                 NOTE: the pad's path between start and goal is a curve, not a straight line — use
                 apply_profile() for safe multi-leg travel.

        :param target_X: world X for the pad face.
        :param target_Y: world Y for the pad face.
        :param target_Z: world Z for the pad face.
        :param magnet_orientation: (w, x, y, z) finger quaternion; None = pad-down.
        :param max_speed: cruise speed in rad/s (None = ARM_SPEED).
        :param dt: control tick (None = physics dt).
        :param settle: wait for the servos to arrive before returning.
        :return: NA
        """
        q_goal = self.IK_solve(target_X, target_Y, target_Z, magnet_orientation)
        self._move_joints(q_goal, max_speed, dt, settle)
        self.current_position = (target_X, target_Y, target_Z)


    def apply_profile(self, target_X, target_Y, target_Z, magnet_orientation=None, max_speed=None,
                      safe_z=None):
        """
        Summary: (Helper) Use profile to plan lift travel and descent: (0) if the arm isn't in the pad-down
                 posture yet, swing there via UP_POSTURE; (1) lift straight up to a safe height; (2) travel
                 over at that height; (3) descend straight down onto the target.

        :param target_X: world X for the pad face.
        :param target_Y: world Y for the pad face.
        :param target_Z: world Z for the pad face.
        :param magnet_orientation: (w, x, y, z) finger quaternion; None = pad-down.
        :param max_speed: cruise speed in rad/s (None = ARM_SPEED).
        :param safe_z: travel height (None = SAFE_Z).
        :return: NA
        """
        safe_z = self.SAFE_Z if safe_z is None else safe_z

        # 0. Entry: from rest, go UP first, then to the pad-down posture (a direct swing clips the graveyard).
        if self.pad_axis_tilt_deg() > self.TILT_LIMIT_DEG:
            self._move_joints(self.UP_POSTURE, max_speed, settle=False)
            self._move_joints(self.CANONICAL_ARM, max_speed)

        x0, y0, z0 = self.pad_world_pos()
        travel_z = max(safe_z, z0, target_Z)

        # 1-3. Intermediate legs skip settling (they only need to be roughly there); the last one settles.
        self.move_to(x0, y0, travel_z, magnet_orientation, max_speed, settle=False)            # lift
        self.move_to(target_X, target_Y, travel_z, magnet_orientation, max_speed, settle=False)  # travel
        self.move_to(target_X, target_Y, target_Z, magnet_orientation, max_speed)               # descend

    def home(self):
        """
        Summary: call move_to, to a hard coded convenient position to rest safely.

        :return:
        """
        pass

    def waypoint(self):
        """
        Summary: call move_to, to a hard coded convenient position to start a turn.

        :return:
        """
        pass

    def grasp(self, piece_type):
        """
        Summary:

        :param piece_type:
        :return:
        """
        pass

    def release(self, piece_type):
        """
        Summary:

        :param piece_type:
        :return:
        """
        pass


# (Testing)  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def main(show_viewer=True):
    """
    Summary: Smoke-test: build the Genesis scene, then call move_to() once and report how close the pad got.

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

    # --- 2. Move: pad face 40 mm above the board, mid-rank at the right edge ---
    movement = MovementSystem(sim.robot, sim.scene)

    for _ in range(int(5 / MovementSystem.DT)):  # 30 / 0.005 = 6000 steps
        sim.scene.step()

    goal1 = (0, 0, 0)
    movement.move_to(*goal1)

    goal2 = (0.27, 0.23, 0.7)
    movement.move_to(*goal2)

    goal5 = (0, 0, 0)
    movement.move_to(*goal5)

        # Keep the viewer open so the result can be inspected.
    if show_viewer:
        for _ in range(6000):
            sim.scene.step()


if __name__ == "__main__":
    main()