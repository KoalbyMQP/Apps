from abc import ABC, abstractmethod
import warnings
import numpy as np
from ikpy.chain import Chain

class ArmController(ABC):
    """
    arm_controller.py (Interface)

    Purpose: Initialize robot configuration interface.
    """

# Class Attributes =====================================================================================================

    # List of URDF joint handles (names) for the right and left arm.
    # NOTE - side (right / left), name (robot's joint), axis (which axis motor rotates around)
    ARMS = {
        "left": {"joints":
                     ["l_shoulder_x", "l_shoulder_y", "l_elbow_x", "l_wrist_z", "l_magnet"],
                 "ik_joints":
                     ["l_shoulder_x", "l_shoulder_y", "l_elbow_x", "l_wrist_z", "l_wrist_x"],
                 "finger": "finger_left", "rack": "l_magnet",
                 # tool center point (left hand)
                 "tcp_local": (0.128, 0.0, -0.0116),
                 # Multi-start seeds: hand-verified pad-down poses. solve_IK() solutions converge from these basins.
                 "ik_seeds": ((-1.58, 0.0, -1.58, 0.0, 1.58),      # chess_waypoint
                              (-1.58, 0.09, -1.45, 0.0, 1.45),     # grave 7 (farthest left tile)
                              (-1.58, 0.34, -1.38, 0.0, 1.38),     # a8 (black rook)
                              (-1.58, 0.6, -1.1, 0.0, 1.1),        # d8 (black queen)
                              (-1.58, -1.44, -1.58, 2.61, 1.58),   # grave 3
                              (-1.57, 0.2, -1.58, 1.93, 1.55),     # d1 (white queen)
                              (-1.58, 0.17, -1.58, 1.32, 1.58))},  # d4 (mirror of e4)

        "right": {"joints":
                      ["r_shoulder_x", "r_shoulder_y", "r_elbow_x", "r_wrist_z", "r_magnet"],
                  "ik_joints":
                      ["r_shoulder_x", "r_shoulder_y", "r_elbow_x", "r_wrist_z", "r_wrist_x"],
                  "finger": "finger_right", "rack": "r_magnet",
                  # tool center point (right hand)
                  "tcp_local": (0.128, 0.0, -0.0116),
                  # Multi-start seeds: hand-verified pad-down poses. solve_IK() solutions converge from these basins.
                  "ik_seeds": ((1.58, 0.0, -1.58, 0.0, -1.58),     # chess_waypoint
                               (1.58, -0.09, -1.45, 0.0, -1.45),   # grave 7 (farthest right tile)
                               (1.58, -0.34, -1.38, 0.0, -1.38),   # h8 (black rook)
                               (1.58, -0.6, -1.1, 0.0, -1.1),      # e8 (black king)
                               (1.58, 1.44, -1.58, 2.61, -1.58),   # grave 3
                               (1.57, -0.2, -1.58, 1.93, -1.55),   # e1 (white king)
                               (1.58, -0.17, -1.58, 1.32, -1.58))},  # e4 (center of board)
    }

    # Positional Information:
    ROBOT_POS = (0.0, 0.0, 0.735) # Robot spawn location
    URDF_ROOT_ANGLE = ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)) # URDF root points backward
    RACK_RETRACTED = 0.060 # Retracted rack pose for end-effector
    HOVER_ABOVE_CROWN = 0.016

    # IK Candidate Grading (solve_ik): 
    IK_TILT_PENALTY = 0.002  # score per degree of rack-axis tilt
    IK_MOTION_PENALTY = 0.001  # score per rad of joint motion from the current pose
    IK_SEED_PENALTY = 0.002  # score per rad of distance from the nearest hand-verified seed
    IK_GATE_POS = 0.05  # m
    IK_GATE_ROT = 20.0  # deg

    # Motion limits:
    ARM_SPEED = 3  # rad/s cruise
    ARM_ACCEL = 1  # rad/s^2
    RACK_SPEED = 0.05  # m/s cruise (URDF velocity limit on the rack joint is 0.1)
    RACK_ACCEL = 0.5  # m/s^2
    DT = 0.005  # control tick (sim: physics dt; hardware: its own loop rate)
    SETTLE_TOL = 0.01  # rad, "arrived" threshold
    SETTLE_MAX_TICKS = 300  # give up waiting after this many ticks

# Constructor ==========================================================================================================

    def __init__(self, urdf_file, arm_name="left"):
        """
        Constructor: Initialize the ArmController class for one arm of an already spawned robot in simulation scene.

        :param urdf_file: URDF file
        :param arm_name: Which arm this controller represents ('right' or 'left').
        """

        # Initialize parameters
        self.arm = self.ARMS[arm_name]
        self.arm_name = arm_name
        self.TCP_LOCAL = self.arm["tcp_local"]   # per-arm: left finger mounts +z, right -z

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            probe = Chain.from_urdf_file(urdf_file, base="chest", tip=self.arm["rack"],
                                         tip_offset=self.TCP_LOCAL)
        self.chain = Chain.from_urdf_file(
            urdf_file, base="chest", tip=self.arm["rack"], tip_offset=self.TCP_LOCAL,
            active_links_mask=[l.name in self.arm["ik_joints"] for l in probe.links])

        rack = [0.0] * len(self.chain.links)
        rack[[l.name for l in self.chain.links].index(self.arm["rack"])] = 0.0
        self._rack_seed = np.array(rack)  # rack slot held retracted in every seed (q=0 = retracted stop, commanded not solved)
        self._starts = np.array([self.chain.active_to_full(np.array(s, float), rack)
                                 for s in self.arm["ik_seeds"]])
        self._ik_seeds = np.asarray(self.arm["ik_seeds"], dtype=float)  # active-joint form, for solve_ik seed-proximity score

# Functions ============================================================================================================

    def solve_ik(self, target_X, target_Y, target_Z, orientation, mode="all", seed=None):
        """
        Summary: Compute Inverse Kinematics for arm movement using IKPY to place hand at given location.

        :param target_X: X position of the target location.
        :param target_Y: Y position of the target location.
        :param target_Z: X position of the target location.
        :param seed: current 5 arm joint angles (rad) to seed the optimizer FIRST; IKPY is
                     local, so seeding from where the arm actually is keeps far tiles in
                     the reachable basin instead of pinning against joint limits.
        :return: (q_arm (5,), pos_err (m), rot_err (deg)
        """

        # Input Orientation Mode must give a X, Y, or Z rotation axis restriction or a complete X, Y, and Z restriction (All).
        if mode not in ("all", "X", "Y", "Z"):
            raise ValueError(f"unknown mode {mode!r}: use 'all' or 'X'/'Y'/'Z'")

        # Convert tuple to float numPY array & format.
        rot = np.asarray(orientation, dtype=float)
        if mode == "all" and rot.shape != (3, 3):
            raise ValueError("mode='all' needs a 3x3 rotation matrix")
        if mode != "all" and rot.shape != (3,):
            raise ValueError(f"mode={mode!r} needs a 3-vector axis")

        # Format position and rotation information; world -> base (ikpy works in the URDF root frame)
        b2w, rp = np.asarray(self.URDF_ROOT_ANGLE), np.asarray(self.ROBOT_POS)
        p_base = b2w.T @ (np.array([target_X, target_Y, target_Z]) - rp)

        # Rotates the desired orientation into the base frame. For a single axis, it’s also normalized to unit length.
        if mode == "all":
            rot_base = b2w.T @ rot
        else:
            rot_base = (b2w.T @ rot) / np.linalg.norm(rot)
        starts = self._starts

        # Start search from seed positions
        if seed is not None:  # current pose first: minimal-motion solution wins when reachable
            starts = np.vstack([self.chain.active_to_full(np.asarray(seed, float), self._rack_seed),
                                self._starts])

        # Multi-start, but keep EVERY distinct solution: IKPY's own pick ranks by raw IK cost, which lets self-clipping poses win.
        # Instead, grade the candidates here (gate -> stroke-free tilt -> minimal motion).

        # Creates a 4x4 homogeneous transform with the target position in the last column.
        frame = np.eye(4)
        frame[:3, 3] = p_base
        if mode == "all":
            frame[:3, :3] = rot_base
        else:
            frame[:3, "XYZ".index(mode)] = rot_base

        # Solve: Start from all seed positions, and returns all solutions for custom grading.
        cands = self.chain._inverse_kinematics_solutions_frame(frame, initial_position=starts, orientation_mode=mode, optimizer_budget=5000, tol=1e-9)
        q_ref = np.asarray(seed, float) if seed is not None else None

        # Scoring: Runs forward kinematics or each solution, finds where the solution actually lands, and grades them.
        best, best_score, fallback = None, np.inf, None

        # Loop through candidates
        for q in cands:
            T = self.chain.forward_kinematics(q)
            pad_w = b2w @ T[:3, 3] + rp
            pos_err = float(np.linalg.norm(pad_w - np.array([target_X, target_Y, target_Z])))

            # Adjust for input mode (don't grade all 3 rotation axis if you only care about one.
            if mode == "all":
                R_err = T[:3, :3].T @ rot_base
                rot_err = float(np.degrees(np.arccos(np.clip((np.trace(R_err) - 1.0) / 2.0,
                                                             -1.0, 1.0))))
            else:
                axis = T[:3, "XYZ".index(mode)]
                rot_err = float(np.degrees(np.arccos(np.clip(axis @ rot_base, -1.0, 1.0))))

            # Extracts just the 5 active arm joints from the full vector.
            q_arm = np.array(self.chain.active_from_full(q))

            # Tracks the lowest-position-error candidate, ignoring gates, in case nothing passes.
            if fallback is None or pos_err < fallback[1]:
                fallback = (q_arm, pos_err, rot_err)  # closest hit, for move_to's skip report

            # The gate: candidates that are too inaccurate are discarded.
            if pos_err > self.IK_GATE_POS or rot_err > self.IK_GATE_ROT:
                continue

            # measure motion
            motion = 0.0 if q_ref is None else float(np.linalg.norm(q_arm - q_ref))

            # Seed proximity: the hand-verified seeds are the only ground truth for non-clipping poses.
            seed_d = float(np.min(np.linalg.norm(self._ik_seeds - q_arm, axis=1)))

            # Weighted Sum
            score = pos_err + self.IK_TILT_PENALTY * rot_err + self.IK_MOTION_PENALTY * motion \
                + self.IK_SEED_PENALTY * seed_d

            # Keeps lowest
            if score < best_score:
                best, best_score = (q_arm, pos_err, rot_err), score

        return best if best is not None else fallback

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
        if distance >= v_max * v_max / a_max:  # long enough: trapezoid
            t_a = v_max / a_max
            total = distance / v_max + t_a
        else:  # short move: triangle
            t_a = np.sqrt(distance / a_max)
            total = 2.0 * t_a
        n = max(2, int(np.ceil(total / dt)))
        span = total / n  # per-sample time
        fracs = []
        for k in range(1, n + 1):
            t = k * span
            t_rem = total - t
            if t < t_a:  # accelerating
                s = 0.5 * a_max * t * t
            elif t_rem < t_a:  # decelerating
                s = distance - 0.5 * a_max * t_rem * t_rem
            else:  # cruising
                v_peak = a_max * t_a
                s = 0.5 * v_peak * t_a + v_peak * (t - t_a)
            fracs.append(min(1.0, s / distance))
        return fracs

    def move_joints(self, q_goal, v_max=None):
        """
        Summary: Use trapezoidal profile calculated by movement profile. Break down the path from the starting to the
                 target location, and apply calculated speeds to each fraction before creating final movement instructions.

        :param q_goal: 5 target arm joint angles (rad, self.arm["ik_joints"] order).
        :param v_max: cruise speed rad/s (None = ARM_SPEED).
        :return: NA
        """
        v_max = self.ARM_SPEED if v_max is None else v_max
        q_goal = np.asarray(q_goal, dtype=float)
        q_start = np.asarray(self.read_joints(), dtype=float)
        delta = q_goal - q_start
        distance = float(np.max(np.abs(delta)))       # farthest joint sets the pace
        if distance < 1e-6:
            return
        for frac in self.movement_profile(distance, v_max, self.ARM_ACCEL, self.DT):
            self.command_joints(q_start + frac * delta)
            self.step()
        self.settle(q_goal)

    def settle(self, q_goal, tol=None):
        """
        Summary: Continue to call q_goal until every arm joint is within tol of it, in order to cancel any inertia.

        :param q_goal: joint angles to hold (rad).
        :param tol: arrival threshold (None = SETTLE_TOL).
        :return: True if arrived, False if it ran out of ticks.
        """
        tol = self.SETTLE_TOL if tol is None else tol
        for _ in range(self.SETTLE_MAX_TICKS):
            self.command_joints(q_goal)
            self.step()
            if np.max(np.abs(np.asarray(self.read_joints()) - q_goal)) < tol:
                return True
        return False

# Interface ============================================================================================================

    @abstractmethod
    def read_joints(self):
        """
        Summary:

        :return:
        """
        ...

    @abstractmethod
    def command_joints(self, q):
        """
        Summary:

        :param q:
        :return:
        """
        ...

    @abstractmethod
    def step(self):
        """
        Summary:

        :return:
        """
        ...

    @abstractmethod
    def move_rack(self, v, v_max=None):
        """
        Summary: Drive the rack & pinion to absolute extension v and hold it there.

        :param v: target rack extension (m; 0 = pad highest/retracted (default stop), RACK_RETRACTED = pad lowest (full stroke)).
        :param v_max: cruise speed m/s (None = RACK_SPEED).
        :return: NA
        """
        ...

    @abstractmethod
    def pad_world_z(self):
        """
        Summary: World Z of the pad face (release-seat invariant).

        :return: pad face world Z (m).
        """
        ...

    @abstractmethod
    def read_rack(self):
        """
        Summary: Read the rack's current extension.

        :return: current rack extension (m).
        """
        ...

    @abstractmethod
    def toggle_magnet(self) -> bool:
        """
        Summary: Energize/de-energize the electromagnet.
        :return: True if energize attached a piece (or de-energize ran); False if
                 energize found nothing in range.
        """
        ...