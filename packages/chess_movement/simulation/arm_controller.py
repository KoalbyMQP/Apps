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

    # World-blind: URDF names per arm. Both worlds look things up BY these names;
    # sim maps name->DOF index, hardware maps name->servo (bridge.toml).
    ARMS = {

        # List of joint handles (names/variables) for the right arm.
        # NOTE - side (right / left), name (robot's joint), axis (which axis motor rotates around)
        "left": {"joints":
                     ["l_shoulder_x", "l_shoulder_y", "l_elbow_x", "l_wrist_z", "l_magnet"],
                 "ik_joints":
                     ["l_shoulder_x", "l_shoulder_y", "l_elbow_x", "l_wrist_z", "l_wrist_x"],
                 "finger": "finger_left", "rack": "l_magnet",
                 # left finger mounts on the +z side of its link frame (URDF left_piece
                 # origin z=+0.01; right is -0.01), so the pad offset z flips.
                 "pad_local": (0.083, 0.0, 0.0075),
                 # Mirrored from the verified right-arm seeds (sign pattern
                 # (-1,-1,1,1,-1)); validated in Genesis: a1/d4/a8/l0/l15 arrive 1-5mm.
                 "ik_seeds": ((-1.62, 0.03, -1.57, -0.02, 1.53),
                              (-1.09, 0.0, -0.16, 0.13, 0.69),
                              (-1.59, -0.26, -1.53, 2.12, 1.53))},

        "right": {"joints":
                      ["r_shoulder_x", "r_shoulder_y", "r_elbow_x", "r_wrist_z", "r_magnet"],
                  "ik_joints":
                      ["r_shoulder_x", "r_shoulder_y", "r_elbow_x", "r_wrist_z", "r_wrist_x"],
                  "finger": "finger_right", "rack": "r_magnet",
                  "pad_local": (0.083, 0.0, -0.0075),
                  # Multi-start seeds: hand-verified pad-down poses (hover + two board
                  # corners); the far-corner solutions only converge from these basins.
                  # Plus 7 clean-branch poses posed in the slider panel (2026-11) and
                  # FK-verified to their tile within 10mm — panel order is
                  # wrist_x,wrist_z; these are ik_joints order (wrist_z,wrist_x).
                  # e4 / grave8 / h1 / e1 / grave15 / h8 / e8.
                  "ik_seeds": ((1.62, -0.03, -1.57, -0.02, -1.53),   # hover
                               (1.09, 0.0, -0.16, 0.13, -0.69),      # top corner
                               (1.59, 0.26, -1.53, 2.12, -1.53),     # bottom corner
                               (1.55, -0.6, -1.56, 0.47, -1.58),     # e4
                               (1.58, 1.18, -1.58, 2.6, -1.57),      # grave 8
                               (1.59, 0.23, -1.56, 2.37, -1.57),     # h1
                               (1.58, -0.47, -1.58, 1.52, -1.57),    # e1
                               (1.57, 0.0, -1.18, -0.05, -1.18),     # grave 15
                               (1.58, -0.28, -1.11, -0.05, -1.12),   # h8
                               (1.58, -0.56, -0.81, -0.05, -0.82),   # e8
                               (1.56, 0.0, -1.56, 2.16, -1.56))},    # g1 knight
    }

    # Positional Information
    ROBOT_POS = (0.0, 0.0, 0.735)
    BASE_TO_WORLD = ((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))
    RACK_RETRACTED = 0.032
    HOVER_ABOVE_CROWN = 0.016

    # Motion limits
    ARM_SPEED = 2  # rad/s cruise
    ARM_ACCEL = 3  # rad/s^2
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
        self.PAD_LOCAL = self.arm["pad_local"]   # per-arm: left finger mounts +z, right -z

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            probe = Chain.from_urdf_file(urdf_file, base="chest", tip=self.arm["rack"],
                                         tip_offset=self.PAD_LOCAL)
        self.chain = Chain.from_urdf_file(
            urdf_file, base="chest", tip=self.arm["rack"], tip_offset=self.PAD_LOCAL,
            active_links_mask=[l.name in self.arm["ik_joints"] for l in probe.links])

        rack = [0.0] * len(self.chain.links)
        rack[[l.name for l in self.chain.links].index(self.arm["rack"])] = self.RACK_RETRACTED
        self._rack_seed = np.array(rack)  # rack slot held retracted in every seed (commanded, not solved)
        self._starts = np.array([self.chain.active_to_full(np.array(s, float), rack)
                                 for s in self.arm["ik_seeds"]])

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
        if mode not in ("all", "X", "Y", "Z"):
            raise ValueError(f"unknown mode {mode!r}: use 'all' or 'X'/'Y'/'Z'")
        rot = np.asarray(orientation, dtype=float)
        if mode == "all" and rot.shape != (3, 3):
            raise ValueError("mode='all' needs a 3x3 rotation matrix")
        if mode != "all" and rot.shape != (3,):
            raise ValueError(f"mode={mode!r} needs a 3-vector axis")
        b2w, rp = np.asarray(self.BASE_TO_WORLD), np.asarray(self.ROBOT_POS)
        # world -> base frame (ikpy works in the URDF root frame)
        p_base = b2w.T @ (np.array([target_X, target_Y, target_Z]) - rp)
        if mode == "all":
            rot_base = b2w.T @ rot
        else:
            rot_base = (b2w.T @ rot) / np.linalg.norm(rot)
        starts = self._starts
        if seed is not None:  # current pose first: minimal-motion solution wins when reachable
            starts = np.vstack([self.chain.active_to_full(np.asarray(seed, float), self._rack_seed),
                                self._starts])
        q = self.chain.inverse_kinematics(target_position=p_base, target_orientation=rot_base,
                                           orientation_mode=mode, initial_position=starts,
                                           optimizer_budget=5000, tol=1e-9)
        # Grade the answer: FK the returned joints, compare against what was asked.
        T = self.chain.forward_kinematics(q)
        pad_w = b2w @ T[:3, 3] + rp
        pos_err = float(np.linalg.norm(pad_w - np.array([target_X, target_Y, target_Z])))
        if mode == "all":
            R_err = T[:3, :3].T @ rot_base
            rot_err = float(np.degrees(np.arccos(np.clip((np.trace(R_err) - 1.0) / 2.0,
                                                         -1.0, 1.0))))
        else:
            axis = T[:3, "XYZ".index(mode)]
            rot_err = float(np.degrees(np.arccos(np.clip(axis @ rot_base, -1.0, 1.0))))
        return np.array(self.chain.active_from_full(q)), pos_err, rot_err

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

        :param v: target rack extension (m; 0 = pad lowest, RACK_RETRACTED = pad highest).
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
    def toggle_magnet(self):
        """
        Summary:
        :return:
        """
        ...