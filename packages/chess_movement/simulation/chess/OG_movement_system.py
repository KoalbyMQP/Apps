# ============================================================
# movement_system.py
# Purpose: Establish basic movement system for grasping and placing the end-effector, and moving it in 3d Space.
import numpy as np


def _np(x):
    """(Helper) Genesis hands back torch tensors on the compute device;
    convert anything (ndarray / torch cpu / torch cuda) to a float64 numpy array."""
    if hasattr(x, "detach"):
        x = x.detach()
    if hasattr(x, "cpu"):
        x = x.cpu()
    if hasattr(x, "numpy"):
        x = x.numpy()
    return np.asarray(x, dtype=float)


class OGMovementSystem:

    # Motion limits (HerkuleX-safe; rack URDF velocity limit is 0.1 m/s).
    # NOTE: cruise speed must stay inside the PD servo's tracking bandwidth —
    # at 1.2 rad/s the heavy arm lagged the setpoint by >1 rad mid-move.
    ARM_SPEED = 0.5      # rad/s cruise
    ARM_ACCEL = 1.5      # rad/s^2
    # Cartesian (straight-line) path limits for target(). The hand tip moves
    # along a world-space line. IK is re-solved every tick, so faster motion
    # also means fewer ticks (faster wall-clock AND viewer playback).
    CART_SPEED = 0.5     # m/s cruise
    CART_ACCEL = 1.5     # m/s^2
    RACK_SPEED = 0.06    # m/s cruise
    RACK_ACCEL = 0.3     # m/s^2
    DT = 0.005           # physics step (matches GenesisSetup RigidOptions dt)

    # Geometry (VERIFY PAD constants in the viewer before trusting grasps):
    #   fingerextend_right: 0.032 = rack fully retracted, 0.0 = fully extended.
    #   Pad face sits ~83 mm out the finger's +x axis (mmh_chess magnet_face_local
    #   for this URDF family); in the pad-down chess pose that axis points down.
    STROKE = 0.032
    PAD_LOCAL = (0.083, 0.0, -0.0075)        # pad face in finger-link frame
    PAD_OFFSET_DOWN = (0.0, 0.0, -0.083)     # pad face below finger origin, pad-down pose
    # Desired finger orientation at every arrival: local +x (the rack/pad
    # axis) pointing straight DOWN at world -z. (w, x, y, z). Full-quat IK +
    # per-tick seeding keeps the magnet travel vertical (measured 2-3 deg).
    PAD_DOWN_QUAT = (0.5, -0.5, 0.5, 0.5)
    # Arm joint values of a verified pad-down posture (tilt 4.1 deg, 0.5 rad
    # margin to every joint limit, pad at (0.155, 0.327, 0.693) — behind and
    # right of the board, clear of it). Seeds the IK into the pad-down elbow
    # branch; the other branch (negative elbow) cannot orient the pad down.
    CANONICAL_ARM = (1.0, 0.0, 1.0, 0.0, 0.5)   # shoulderspin..handcurl
    # Waypoint of the ENTRY swing (rest -> up -> canonical). Chosen by sweep
    # probe: the wrist/hand/finger pass clear of the board, pedestal and
    # graveyard slab (30 mm margin) on both legs of the entry. A direct
    # rest->canonical swing sweeps the wrist THROUGH the graveyard slab.
    UP_POSTURE = (0.0, 1.5, 1.0, 0.0, 0.0)
    # Per-joint clamps applied to every IK solution (shoulderspin..handcurl).
    # The URDF arm limits were widened to +/-2.618 rad (professor's mmh_sim
    # values) to de-saturate the elbow, but the raw IK then wanders into an
    # elbow-DOWN branch that folds the bicep through the head/chest (measured
    # 1.6 mm mesh gap). Clamping keeps the elbow-UP branch the professor's
    # package uses (his solutions: elbow +0.3..+1.5, shoulderspin -0.6..+0.8).
    BRANCH_LO = (-1.0, -2.2, 0.2, -2.2, -1.0)
    BRANCH_HI = (1.0, 2.2, 2.618, 2.2, 1.0)

    ARM_JOINTS = ["shoulderspin_right", "biceplift_right", "elbow_right",
                  "wristspin_right", "handcurl_right"]
    RACK_JOINT = "fingerextend_right"

    def __init__(self, robot, scene=None):
        """
        Constructor: Initialize the class variables.

        :param robot: Genesis robot entity (or None for dry tests).
        :param scene: Genesis scene — required for stepped motions (target()).
        """
        self.robot = robot
        self.scene = scene
        self.current_position = None
        self.last_arm_goal = None

        if robot is not None:
            self.arm_dofs = [int(robot.get_joint(n).dofs_idx_local[0]) for n in self.ARM_JOINTS]
            self.rack_dof = [int(robot.get_joint(self.RACK_JOINT).dofs_idx_local[0])]
            self.finger_link = robot.get_link("finger_right")
            self._link_idx = robot.links.index(self.finger_link)
            self._dof_lo, self._dof_hi = (_np(t) for t in robot.get_dofs_limit())

    # ---- trapezoidal motion helpers - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def _profile(self, distance, v_max, a_max, dt=None):
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
        span = total / n                                # per-sample time
        fracs = []
        for k in range(1, n + 1):
            t = k * span
            t_rem = total - t
            if t < t_a:                                 # accelerating
                s = 0.5 * a_max * t * t
            elif t_rem < t_a:                           # decelerating
                s = distance - 0.5 * a_max * t_rem * t_rem
            else:                                       # cruising
                v_peak = a_max * t_a
                s = 0.5 * v_peak * t_a + v_peak * (t - t_a)
            fracs.append(min(1.0, s / distance))
        return fracs

    def _run_profile(self, dofs_idx, q_start, q_goal, v_max, a_max):
        """
        Summary: (Helper) Execute one trapezoidal move on the given dofs:
                 solve the profile, then command interpolated setpoints while
                 stepping the sim so the motion is visible and physical.

        :param dofs_idx: local dof indices to drive.
        :param q_start: starting joint values (current).
        :param q_goal: goal joint values.
        :param v_max: cruise speed limit.
        :param a_max: acceleration limit.
        :return: NA
        """
        q_start = np.asarray(q_start, dtype=float)
        q_goal = np.asarray(q_goal, dtype=float)
        distance = float(np.max(np.abs(q_goal - q_start)))
        if distance == 0.0:
            return
        # The robot has 26 dofs; only `dofs_idx` may move. Everything else
        # (waist, neck, legs, left arm) must be HELD at its current value or
        # it goes limp under gravity and drags the shoulder out of place.
        q_full_now = _np(self.robot.get_dofs_position())
        idx = np.asarray(dofs_idx, dtype=int)

        def setpoint(frac):
            q = q_full_now.copy()
            q[idx] = q_start + frac * (q_goal - q_start)
            return q

        for frac in self._profile(distance, v_max, a_max):
            self.robot.control_dofs_position(setpoint(frac))
            self.scene.step()
        # hold on the final setpoint until the PD servo converges — this arm is
        # heavy and tracks far below the trapezoid setpoint; it needs several
        # seconds of sim time on the final setpoint to actually get there
        q_final = setpoint(1.0)
        for _ in range(600):
            self.robot.control_dofs_position(q_final)
            self.scene.step()

    # ---- pose-constrained IK (damped least squares) - - - - - - - - - - - - - - -

    def _fk_pad(self, q):
        """
        Summary: (Helper) Forward kinematics WITHOUT touching sim state.

        :param q: full-qpos numpy array.
        :return: (pad_world_pos, pad_axis_world_dir) as (3,) arrays.
        """
        pos, quat = self.robot.forward_kinematics(q, links_idx_local=[self._link_idx])
        p = _np(pos).reshape(-1, 3)[0]
        w, x, y, z = _np(quat).reshape(-1, 4)[0]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        return p + R @ np.asarray(self.PAD_LOCAL, dtype=float), R[:, 0]

    def _ik_pose(self, q_seed, pad_goal, iters=15, damp=0.08, step=0.25):
        """
        Summary: (Helper) Damped-least-squares IK for the PAD POINT position
                 (3 rows) with the rack/pad axis held vertical pointing DOWN
                 (3 rows: axis local +x driven to world -z; roll about the
                 pad axis is left free). Genesis's built-in IK is
                 position-only (or starves position when given a partial
                 rot_mask); this weights position 1.0 and axis 3.0 and solves
                 only the arm dofs. Seeded from q_seed so the elbow branch
                 stays continuous along a path. NOTE: the axis error is
                 (down - a), NOT cross(a, down) — the cross product is also
                 zero when the axis points UP and the solver will happily
                 converge to that flipped branch.

        :param q_seed: full-qpos numpy array to start from.
        :param pad_goal: (3,) world position for the pad face.
        :param iters: DLS iterations.
        :param damp: damping factor (regularization).
        :param step: max joint step per iteration (rad).
        :return: full-qpos numpy array at the solution.
        """
        arm = np.asarray(self.arm_dofs, dtype=int)
        lo, hi = self._dof_lo, self._dof_hi
        down = np.array([0.0, 0.0, -1.0])
        W = np.diag([1.0, 1.0, 1.0, 3.0, 3.0, 3.0])
        q_live = _np(self.robot.get_dofs_position())   # restore when done
        q = q_seed.copy()
        for _ in range(iters):
            p, a = self._fk_pad(q)
            e = np.concatenate([pad_goal - p, down - a])
            self.robot.set_qpos(q)   # jacobian reads LIVE solver state — sync it
            J6 = _np(self.robot.get_jacobian(self.finger_link,
                                             local_point=np.asarray(self.PAD_LOCAL)))
            # rows 0-2: pad-point velocity (world). rows 3-5: angular velocity.
            # e_rot = down - a, so J_rot = -da/dq = +skew(a) @ omega_jacobian
            skew_a = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
            Ja = np.vstack([J6[:3, arm], skew_a @ J6[3:6, arm]])
            # weighted DLS: dq = (W J)^T [ (W J)(W J)^T + damp^2 I ]^-1 (W e)
            JW, eW = W @ Ja, W @ e
            dq = JW.T @ np.linalg.solve(JW @ JW.T + (damp ** 2) * np.eye(6), eW)
            q = q.copy()
            q[arm] = np.clip(q[arm] + np.clip(dq, -step, step), lo[arm], hi[arm])
        self.robot.set_qpos(q_live)
        return q

    # ---- public motion API - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def target(self, X, Y, Z):
        """
        Summary: Move the ARM (never the rack) so the magnet PAD face reaches
                 (X, Y, Z) with the rack axis VERTICAL (magnet travels only
                 up/down). Interpolates the pad goal along a straight
                 WORLD-space line, re-solving pose-constrained DLS IK at every
                 sample seeded from the previous solution (position + pad-axis
                 down; roll about the pad axis free), and commands the arm
                 along a trapezoidal velocity profile, stepping the sim each
                 tick. The rack is held retracted.

        :param X: world X where the pad face should end up.
        :param Y: world Y where the pad face should end up.
        :param Z: world Z where the pad face should end up.
        :return: NA
        """
        pad_goal = np.array((X, Y, Z))

        # 0. Entry: the pad-down requirement lives on ONE elbow branch (the
        #    other branch physically cannot point the pad down). If the arm
        #    isn't in that basin yet, drive it to the canonical pad-down
        #    posture first (joint-space trapezoid; posture is clear of the
        #    board — pad at (0.155, 0.327, 0.693)).
        arm = np.asarray(self.arm_dofs, dtype=int)
        if self.pad_axis_tilt_deg() > 20.0:
            q_full0 = _np(self.robot.get_dofs_position())
            q_can = q_full0.copy()
            q_can[arm] = self.CANONICAL_ARM
            self._run_profile(self.arm_dofs, q_full0[arm], q_can[arm],
                              self.ARM_SPEED, self.ARM_ACCEL)

        # 1. Straight-line pad path from the pad's CURRENT world position.
        p_now = self.pad_world_pos()
        distance = float(np.linalg.norm(pad_goal - p_now))
        if distance < 1e-4:
            self.current_position = (X, Y, Z)
            return

        q_full_now = _np(self.robot.get_dofs_position())

        # 2. Trapezoidal progress along the line; full-quat IK every sample
        #    (position + pad-down orientation), seeded from the previous
        #    solution so the elbow branch and roll stay continuous.
        q_prev = q_full_now.copy()
        q_goal_ik = None
        branch_lo = np.asarray(self.BRANCH_LO, dtype=float)
        branch_hi = np.asarray(self.BRANCH_HI, dtype=float)
        for frac in self._profile(distance, self.CART_SPEED, self.CART_ACCEL):
            goal = p_now + frac * (pad_goal - p_now)
            link_goal = goal - np.array(self.PAD_OFFSET_DOWN)
            q_full = _np(self.robot.inverse_kinematics(
                link=self.finger_link,
                pos=tuple(link_goal),
                quat=self.PAD_DOWN_QUAT,
                init_qpos=q_prev,
                dofs_idx_local=self.arm_dofs,
                max_solver_iters=50,
            ))
            # Elbow-UP branch clamp (BRANCH_LO/HI were dead code — never
            # applied). The widened +/-2.618 rad URDF limits let the raw IK
            # fold the bicep through the head/chest (real-mesh audit: 0.2 mm
            # bicep-vs-hand/finger, 1.4 mm bicep-vs-chest, 1.5 mm bicep-vs-
            # head). Solve -> clamp -> re-solve warm-started; if it still
            # escapes the band, keep the clamped pose anyway: a few mm of
            # position error beats visible self-clipping.
            for _ in range(3):
                q_clamped = q_full.copy()
                q_clamped[arm] = np.clip(q_clamped[arm], branch_lo, branch_hi)
                if np.allclose(q_full[arm], q_clamped[arm], atol=1e-3):
                    break
                q_full = _np(self.robot.inverse_kinematics(
                    link=self.finger_link,
                    pos=tuple(link_goal),
                    quat=self.PAD_DOWN_QUAT,
                    init_qpos=q_clamped,
                    dofs_idx_local=self.arm_dofs,
                    max_solver_iters=50,
                ))
            q_full[arm] = np.clip(q_full[arm], branch_lo, branch_hi)
            q_prev = q_full
            q_goal_ik = q_full[arm]
            q_cmd = q_full_now.copy()
            q_cmd[arm] = q_goal_ik
            self.robot.control_dofs_position(q_cmd)
            self.scene.step()

        # 3. Hold the final IK pose until the PD servo converges (all dofs —
        #    uncontrolled ones go limp under gravity and drag the shoulder).
        self.last_arm_goal = q_goal_ik
        q_final = q_full_now.copy()
        q_final[arm] = q_goal_ik
        for _ in range(200):
            self.robot.control_dofs_position(q_final)
            self.scene.step()

        # 4. Rack explicitly held retracted while the arm moves.
        self.robot.control_dofs_position(np.full(1, self.STROKE), self.rack_dof)

        self.current_position = (X, Y, Z)

    def pad_world_pos(self):
        """
        Summary: (Helper) World position of the magnet pad face, computed from
                 the finger link pose and the calibrated pad offset.

        :return: (x, y, z) numpy array.
        """
        pos = _np(self.finger_link.get_pos())
        quat = _np(self.finger_link.get_quat())  # (w, x, y, z)
        w, x, y, z = quat
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        return pos + R @ np.asarray(self.PAD_LOCAL, dtype=float)

    def pad_axis_tilt_deg(self):
        """
        Summary: (Helper) Angle in degrees between the rack/pad axis (finger
                 local +x) and straight down. 0° = magnet travels vertically.

        :return: tilt in degrees (float).
        """
        quat = _np(self.finger_link.get_quat())  # (w, x, y, z)
        w, x, y, z = quat
        ax = np.array([1 - 2 * (y * y + z * z),
                       2 * (x * y + w * z),
                       2 * (x * z - w * y)])     # world dir of local +x
        down = np.array([0.0, 0.0, -1.0])
        return float(np.degrees(np.arccos(np.clip(np.dot(ax, down), -1.0, 1.0))))

    def home(self):
        """
        Summary: Safely take the arm from its current position to a resting position by the robots side.

        :return: NA
        """
        self.robot.set_qpos(self.robot.init_qpos)

    def waypoint(self):
        """
        Summary: Move the arm from resting spot, or last grasp/release location to a safe point above the board, and off
                 to the side. This will be the consistent (X:0, Y:0, Z:0) starting point. From this point it can start
                 another turn.

        :return: NA
        """
        self.target(0.15, 0.10, 0.70)

    def grasp(self):
        """
        Summary: (Starting from safe operation height, and empty gripper) Open gripper and move hand vertically downward
                  in 3D space (to board surface height), close gripper, and lift back to original Z height without
                  ever-changing X-Y coordinates.

        :return: NA
        """
        X, Y, Z = self.current_position
        self.target(X, Y, 0.65)
        self.target(X, Y, Z)

    def release(self):
        """
        Summary: (Starting from safe operation height, and a piece in hand with gripper already closed) Move hand
                  vertically downward in 3D space, open gripper, lift back to original Z height, and open gripper.
                  (without ever-changing X-Y coordinates).

        :return: NA
        """
        X, Y, Z = self.current_position
        self.target(X, Y, 0.65)
        self.target(X, Y, Z)


# (Testing)  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def main():
    a_movement_system = OGMovementSystem(None)
    print("test 0")

if __name__ == "__main__":
    main()