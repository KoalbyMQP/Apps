"""Chess-playing agent: Stockfish decides the moves, the Koalby humanoid
physically executes them on a chessboard in Genesis.

Usage (run with Python 3.11, which has genesis + chess installed):
    py -3.11 OLD_chessAgent.py                # you play White, Stockfish plays Black
    py -3.11 OLD_chessAgent.py --selfplay 10  # Stockfish vs Stockfish for 10 plies
    py -3.11 OLD_chessAgent.py --test         # headless 2-ply smoke test

Interactive commands: a move in UCI (e2e4) or SAN (Nf3), 'auto' to let
Stockfish move for you, 'q' to quit.
"""
import sys
import io
import os
import glob
import warnings

# Genesis + python-chess are installed under Python 3.11 on this machine;
# if launched with another interpreter (e.g. VS Code's default "python"),
# relaunch with the right one instead of failing on imports
if sys.version_info[:2] != (3, 11):
    import subprocess
    sys.exit(subprocess.call(
        ["py", "-3.11", os.path.abspath(__file__), *sys.argv[1:]]))

# Genesis warns about torch<2.8 on every import; it works fine for this sim
warnings.filterwarnings("ignore", message=".*torch<2.8.0.*")

# Fix Windows terminal Unicode encoding errors from Genesis logger
if sys.stdout.encoding and sys.stdout.encoding.lower() != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

import numpy as np
import torch
import chess
import chess.engine
import genesis as gs

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_GENESIS_DIR = os.path.dirname(_SCRIPT_DIR)  # Simulation/genesis (URDF + controller)
URDF_FILE = os.path.join(_GENESIS_DIR, "Balancing_Chess_URDF", "urdf",
                         "Balancing_Chess_URDF.urdf")
sys.path.insert(0, _SCRIPT_DIR)
sys.path.insert(0, _GENESIS_DIR)
from koalbyArmController import RobotArmController

# -------------------------
# Board geometry (verified: all 64 squares reachable by pure joint motion)
# -------------------------
SQUARE = 0.020                 # square edge length
BOARD_Y0 = 0.135               # near edge close enough that ALL 64 squares are
                               # joint-reachable (worst IK error 0.3mm) - no
                               # base sliding needed
BOARD_TOP_Z = 0.62             # top surface of the board
BOARD_THICK = 0.02
APPROACH_CLEAR = 0.05          # how far above grasp height to approach from
TRANSIT_CLEAR = 0.11           # travel height above the board between squares
FINGER_REACH = 0.0           # claw tips extend this far below the ee origin

# Measured centroid of the finger mesh's grasp region, in the finger link's
# OWN local frame (from trimesh centroid of finger_{side}.STL). The link's
# origin is the joint pivot, not the contact point -- this offset is the
# correction from pivot to where a piece actually sits inside the curl.
# FIX 1: tip-region centroid (far 25% of the mesh from the pivot), not the
# whole-mesh centroid -- the old value averaged in the bulky mounting/pivot
# end, systematically undershooting the actual working tip. Old values were
# hand_right=[-0.0322,-0.0080, 0.0200], hand_left=[-0.0322, 0.0051,-0.0209].
# Reverted from the tip-25% centroid (78.5mm) back to a shorter mid-length
# band centroid (~38mm) -- empirically, the LONGER offset made grasp misses
# WORSE, not better: with orientation essentially uncontrolled (tilt swinging
# 60-130+ degrees per diag-fix3), a longer lever arm amplifies angular error
# into MORE position error (error scales roughly with offset_magnitude *
# angular_deviation). A shorter, still-physically-real offset trades a little
# geometric precision for much less sensitivity to the orientation swing.
FINGER_LOCAL_OFFSET = {
    "hand_right": np.array([-0.0381, -0.0017,  0.0036]),
    "hand_left":  np.array([-0.0381,  0.0012, -0.0038]),
}

# FIX 2: the corrective aim happens while the claw is still OPEN, but the
# check happens after it closes -- closing rotates the finger (and thus its
# contact point) further around its own pivot. Predict that rotation instead
# of ignoring it. (delta computed lazily inside the method below, since
# CLAW_OPEN/CLAW_HOLD are not defined yet at this point in the file.)
_FINGERPINCH_AXIS = np.array([1.0, 0.0, 0.0])   # matches the URDF <axis> for fingerpinch_{side}

def _axis_angle_rotate(v, axis, theta):
    """Rodrigues rotation formula: rotate vector v by theta around unit axis."""
    axis = axis / np.linalg.norm(axis)
    return (v * np.cos(theta) + np.cross(axis, v) * np.sin(theta)
            + axis * np.dot(axis, v) * (1.0 - np.cos(theta)))

def _rotate_vec_by_quat(v, quat_xyzw):
    """Rotate 3-vector v by quaternion [x,y,z,w]."""
    x, y, z, w = quat_xyzw
    qv = np.array([x, y, z])
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)
CLAW_OPEN = -0.35              # descend opening: wide enough for any piece
                               # (32mm mouth), narrow enough to miss neighbours
CLAW_HOLD = 0.12               # visually-closed holding angle around a piece

# Full rigid-body piece physics (free pieces, contact-verified pinch, weld
# carry). Physically honest but NOT yet reliable: the single-hook claw needs
# orientation-controlled grasps before this can survive a full game (solver
# NaNs, missed pinches, bystander pieces knocked flying). Default False =
# verified-grasp mode: the claw still physically descends and closes around
# the piece and the grasp only succeeds if the piece is inside the closed
# claw - but pieces are kinematic so the board stays stable.
PHYSICS_PIECES = False
ROBOT_BASE = np.array([0.0, 0.0, 0.735])

STOCKFISH_TIME = 0.2           # seconds of thinking per move
STOCKFISH_SKILL = 5            # 0 (weakest) .. 20 (full strength)

# piece type -> (max radius, height) of its Staunton mesh (see chess_pieces/)
PIECE_DIMS = {
    chess.PAWN:   (0.0056, 0.020),
    chess.ROOK:   (0.0076, 0.025),
    chess.KNIGHT: (0.0071, 0.028),
    chess.BISHOP: (0.0081, 0.032),
    chess.QUEEN:  (0.0082, 0.037),
    chess.KING:   (0.0081, 0.042),
}
MESH_DIR = os.path.join(_SCRIPT_DIR, "chess_pieces", "processed")
PIECE_MESH = {
    chess.PAWN: "Pawn.stl", chess.ROOK: "Rook.stl", chess.KNIGHT: "Knight.stl",
    chess.BISHOP: "Bishop.stl", chess.QUEEN: "Queen.stl", chess.KING: "King.stl",
}

# safety validator parameters (real-robot mindset: never trust a raw plan)
USE_PLANNER = False        # Genesis plan_path hits an LLVM JIT crash on Windows;
                           # re-enable once fixed - validated Cartesian paths are
                           # the collision-safe backbone either way
HARD_DEPTH = 0.005         # keep-out penetration beyond this rejects the path
SAFE_ARM_CLEAR = 0.05      # inner arm links must stay this high above the board
PIECE_XY_MARGIN = 0.008    # gripper keep-out radius around standing pieces
CARRY_XY_MARGIN = 0.005    # carried piece keep-out from standing pieces (travel)
FINGER_DROP = FINGER_REACH  # validator uses the same measured finger extension
WHITE_COLOR = (0.92, 0.88, 0.78)
BLACK_COLOR = (0.15, 0.12, 0.10)
BOARD_COLOR = (0.75, 0.62, 0.45)
DARK_SQ_COLOR = (0.35, 0.22, 0.12)

STOCKFISH_EXE = None
for _p in glob.glob(os.path.join(_SCRIPT_DIR, "stockfish", "**", "*.exe"), recursive=True):
    STOCKFISH_EXE = _p
    break


def square_center(sq: int) -> np.ndarray:
    """World xy of a chess square's center. Files a-h along x, ranks 1-8 along +y."""
    f, r = chess.square_file(sq), chess.square_rank(sq)
    x = (f - 3.5) * SQUARE
    y = BOARD_Y0 + (r + 0.5) * SQUARE
    return np.array([x, y])


class ChessRobot(RobotArmController):
    """Koalby arm controller with a chessboard scene and pick-and-place moves."""

    add_camera = False  # set True to attach an offscreen camera (self.cam)

    def _setup_scene(self):
        self.scene = gs.Scene(
            show_viewer=self.show_viewer,
            viewer_options=gs.options.ViewerOptions(
                res=(1280, 960),
                camera_pos=(2.2, 1.6, 1.8),
                camera_lookat=(0.0, 0.25, 0.6),
                camera_fov=40,
                # dt=0.005 means 200 steps = 1s of sim time; capping at 60FPS
                # would throttle the whole game to 3x slower than real time
                max_FPS=240,
            ),
            rigid_options=gs.options.RigidOptions(
                enable_neutral_collision=True,
                # stiff claw pinching 4-gram pieces against a rigid board
                # needs a finer timestep or the constraint solver NaNs
                dt=0.005,
            )
        )
        self.scene.add_entity(gs.morphs.Plane())
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(file=self.urdf_file, pos=self.robot_pos,
                           quat=(0, 0, 0, 1), fixed=True)
        )

        # board slab: squares span x [-4SQ, 4SQ], y [BOARD_Y0, BOARD_Y0 + 8SQ]
        board_w = 8 * SQUARE + 0.018
        board_cy = BOARD_Y0 + 4 * SQUARE
        self.scene.add_entity(
            gs.morphs.Box(size=(board_w, board_w, BOARD_THICK),
                          pos=(0.0, board_cy, BOARD_TOP_Z - BOARD_THICK / 2),
                          fixed=True, collision=True),
            surface=gs.surfaces.Default(color=BOARD_COLOR),
        )
        # narrow pedestal column toward the far side of the board, so the
        # robot's legs can tuck under the board overhang when it slides forward
        table_top = BOARD_TOP_Z - BOARD_THICK
        self.scene.add_entity(
            gs.morphs.Box(size=(0.10, 0.08, table_top),
                          pos=(0.0, board_cy + 0.02, table_top / 2),
                          fixed=True, collision=True),
            surface=gs.surfaces.Default(color=(0.45, 0.33, 0.22)),
        )
        # visual-only dark squares for the checker pattern
        for sq in chess.SQUARES:
            if (chess.square_file(sq) + chess.square_rank(sq)) % 2 == 0:
                x, y = square_center(sq)
                self.scene.add_entity(
                    gs.morphs.Box(size=(SQUARE, SQUARE, 0.0015),
                                  pos=(x, y, BOARD_TOP_Z + 0.00075),
                                  fixed=True, collision=False),
                    surface=gs.surfaces.Default(color=DARK_SQ_COLOR),
                )

        # 32 pieces at their starting squares (white = ranks 1-2, nearest robot)
        self.piece_ent = {}    # chess square -> genesis entity
        self.piece_h = {}      # entity -> piece height
        self.piece_r = {}      # entity -> piece max radius (for keep-out checks)
        self.piece_label = {}  # entity -> e.g. "white knight"
        for sq, piece in chess.Board().piece_map().items():
            rad, h = PIECE_DIMS[piece.piece_type]
            x, y = square_center(sq)
            # PHYSICS_PIECES: free bodies the claw must physically pinch.
            # Otherwise kinematic, with the grasp geometrically verified.
            ent = self.scene.add_entity(
                gs.morphs.Mesh(file=os.path.join(MESH_DIR, PIECE_MESH[piece.piece_type]),
                               pos=(x, y, BOARD_TOP_Z + 0.0005),
                               euler=(0, 0, 0 if piece.color else 180),
                               **({"convexify": True} if PHYSICS_PIECES else
                                  {"fixed": True, "collision": False})),
                surface=gs.surfaces.Default(
                    color=WHITE_COLOR if piece.color else BLACK_COLOR),
            )
            self.piece_ent[sq] = ent
            self.piece_h[ent] = h
            self.piece_r[ent] = rad
            self.piece_label[ent] = (
                f"{'white' if piece.color else 'black'} {chess.piece_name(piece.piece_type)}")

        if self.add_camera:
            self.cam = self.scene.add_camera(
                res=(960, 720), pos=(1.1, 0.75, 1.25),
                lookat=(0.0, 0.25, 0.62), fov=35, GUI=False)

        self.scene.build()

        self.grave_count = {1: 0, -1: 0}  # captured-piece slots per board side
        self.safety_stats = {"hard": 0, "tight": 0}
        self.recorder = None  # optional TrajectoryRecorder (--record)

    def settle(self, steps=30):
        """Hold the current pose and let the pieces settle on the board."""
        q = self.robot.get_qpos().clone()
        for _ in range(steps):
            self.robot.control_dofs_position(q)
            self.scene.step()
        self.home_qpos = self.robot.get_qpos().clone()

    # -------------------------
    # low-level motion
    # -------------------------
    def _step(self, carry=None):
        # PHYSICS_PIECES: the piece is held by a contact-verified weld and
        # `carry` only informs the path validator. Otherwise the verified
        # grasp is maintained kinematically here.
        self.scene.step()
        if carry is not None and not PHYSICS_PIECES:
            ent, off = carry
            ent.set_pos(self.ee_link.get_pos().cpu().numpy() + off)
        if self.recorder is not None:
            self.recorder.log(self.robot.get_qpos().cpu().numpy())

    # -------------------------
    # safety validation (the sim stand-in for a real robot's planning scene:
    # pieces have no collision geometry, so every candidate path is checked
    # geometrically against the known piece poses before execution)
    # -------------------------
    def _over_board(self, x, y, margin=0.02):
        half = 4 * SQUARE + 0.009
        return (abs(x) < half + margin
                and BOARD_Y0 - margin < y < BOARD_Y0 + 8 * SQUARE + margin)

    def _validate_path(self, path, carry=None, ignore=(), arm_clear=SAFE_ARM_CLEAR,
                       carry_margin=CARRY_XY_MARGIN, stride=2):
        """FK-sweep a joint path and check clearance. Returns (ok, reason, tight).

        Keep-out overlaps deeper than 3mm are hard violations; shallower ones
        are reported as tight clearances (this board is miniaturized to fit the
        robot's reach, so a few passes are legitimately close)."""
        HARD = HARD_DEPTH
        tight = []
        side = "right" if self.ee_name.endswith("right") else "left"
        other = "left" if side == "right" else "right"
        grip_links = [self.robot.get_link(f"hand_{side}"),
                      self.robot.get_link(f"hand_{side}")]
        high_links = [self.robot.get_link(f"{n}_{s}")
                      for s in (side, other) for n in ("forearm",)]
        carry_ent = carry[0] if carry else None
        obstacles = []
        for ent in self.piece_ent.values():
            if ent is carry_ent or ent in ignore:
                continue
            p = ent.get_pos().cpu().numpy()
            obstacles.append((p, self.piece_r[ent], self.piece_h[ent],
                              self.piece_label[ent]))

        q_save = self.robot.get_qpos().clone()
        try:
            for q in list(path)[::stride]:
                self.robot.set_qpos(q)
                # gripper (fingers reach FINGER_DROP below its origin) and hand
                # (no fingers, raw origin) vs standing pieces
                for ln, drop, xy_margin in ((grip_links[0], FINGER_DROP, PIECE_XY_MARGIN),
                                            (grip_links[1], 0.0, 0.005)):
                    lp = ln.get_pos().cpu().numpy()
                    # ramming the board slab blows up the physics solver
                    if self._over_board(lp[0], lp[1]) and lp[2] < BOARD_TOP_Z + 0.02:
                        return False, (f"{ln.name} at ({lp[0]:+.3f},{lp[1]:+.3f},"
                                       f"{lp[2]:.3f}) would hit the board"), tight
                    for (pp, pr, ph, lbl) in obstacles:
                        depth = pr + xy_margin - np.hypot(lp[0] - pp[0], lp[1] - pp[1])
                        if depth > 0 and lp[2] - drop < pp[2] + ph:
                            if depth > HARD:
                                return False, (f"{ln.name} at ({lp[0]:+.3f},{lp[1]:+.3f},"
                                               f"{lp[2]:.3f}) would clip {lbl} at "
                                               f"({pp[0]:+.3f},{pp[1]:+.3f})"), tight
                            tight.append((depth, f"{ln.name} vs {lbl}"))
                # the carried piece itself vs standing pieces
                if carry_ent is not None:
                    ee = grip_links[0].get_pos().cpu().numpy()
                    base = ee + carry[1]
                    cr, ch = self.piece_r[carry_ent], self.piece_h[carry_ent]
                    for (pp, pr, ph, lbl) in obstacles:
                        depth = cr + pr + carry_margin - np.hypot(base[0] - pp[0],
                                                                  base[1] - pp[1])
                        # 4mm z-tolerance: a piece that has effectively cleared
                        # a neighbour's crown shouldn't trip the check
                        if (depth > 0 and base[2] + 0.004 < pp[2] + ph
                                and pp[2] < base[2] + ch):
                            if depth > HARD:
                                return False, (f"carried piece at ({base[0]:+.3f},"
                                               f"{base[1]:+.3f},{base[2]:.3f}) would clip "
                                               f"{lbl} at ({pp[0]:+.3f},{pp[1]:+.3f})"), tight
                            tight.append((depth, f"carried piece vs {lbl}"))
                # inner arm links must arc above the board, never sweep across it
                for ln in high_links:
                    lp = ln.get_pos().cpu().numpy()
                    if self._over_board(lp[0], lp[1]) and lp[2] < BOARD_TOP_Z + arm_clear:
                        return False, f"{ln.name} too low over the board", tight
        finally:
            self.robot.set_qpos(q_save)
        return True, None, tight

    def _ik(self, target_pos, seed_qpos=None):
        """Home-seeded IK: solving from the current (often stretched) pose
        converges to poor local solutions, up to ~40mm off."""
        ik, err = self.robot.inverse_kinematics(
            link=self.ee_link,
            pos=torch.tensor(np.asarray(target_pos), dtype=torch.float32),
            init_qpos=self.home_qpos if seed_qpos is None else seed_qpos,
            max_solver_iters=300,
            max_samples=200,
            dofs_idx_local=self.arm_dofs_idx_local,
            return_error=True,
        )
        residual = float(np.linalg.norm(err.cpu().numpy()[:3]))
        if residual > 0.008:
            print(f"  [ik] residual {1000*residual:.1f}mm at "
                  f"({target_pos[0]:+.3f},{target_pos[1]:+.3f},{target_pos[2]:.3f})")
        return ik

    def _cartesian_path(self, q_now, q_goal, target_pos, waypoints):
        """Joint path whose gripper tracks the straight line to target_pos."""
        start = self.ee_link.get_pos().cpu().numpy()
        target = np.asarray(target_pos, dtype=float)
        knots = [q_now]
        # knot spacing bounds how far the gripper can bulge off the line
        n_knots = max(6, int(np.linalg.norm(target - start) / 0.015) + 2)
        for a in np.linspace(0.0, 1.0, n_knots)[1:-1]:
            ik_i = self._ik(start * (1.0 - a) + target * a)
            if ik_i is not None:
                qq = q_now.clone()
                qq[self.arm_dofs_idx_local] = ik_i[self.arm_dofs_idx_local]
                knots.append(qq)
        knots.append(q_goal)
        per = max(3, waypoints // (len(knots) - 1))
        path = []
        for i in range(len(knots) - 1):
            for a in np.linspace(0.0, 1.0, per, endpoint=False):
                path.append(knots[i] * (1.0 - a) + knots[i + 1] * a)
        path.append(q_goal)
        return path

    def _goto(self, target_pos, waypoints=50, carry=None, settle=8,
              plan=False, ignore=(), carry_margin=CARRY_XY_MARGIN, cartesian=False,
              label=''):
        """Move the gripper to target: IK, plan (or interpolate), validate, execute."""
        ik = self._ik(target_pos)
        if ik is None:
            print(f"  !! IK failed for {target_pos}")
            return False
        q_now = self.robot.get_qpos().clone()
        q_goal = q_now.clone()
        q_goal[self.arm_dofs_idx_local] = ik[self.arm_dofs_idx_local]

        if cartesian or plan:
            # straight-line gripper path: raw joint interpolation sways/bulges
            # the gripper enough to clip pieces or even ram the board, so it is
            # also the safe fallback whenever the planner is unavailable
            lerp = self._cartesian_path(q_now, q_goal, target_pos, waypoints)
        else:
            lerp = [q_now * (1.0 - a) + q_goal * a
                    for a in np.linspace(0.0, 1.0, waypoints)]
        path, ok, why = lerp, False, None
        arm_clear = SAFE_ARM_CLEAR
        # unplanned paths are deterministic - retrying them changes nothing
        for attempt in range(4 if plan else 1):
            candidate = None
            if plan and USE_PLANNER and getattr(self, "_planner_ok", True):
                try:
                    candidate = self.robot.plan_path(
                        qpos_goal=q_goal, num_waypoints=max(waypoints, 60),
                        timeout=2.0, max_retry=1)
                except Exception as e:
                    # Genesis's JIT planner can die on Windows (LLVM relocation
                    # bug); don't poke it again once it has failed
                    self._planner_ok = False
                    print(f"  [planner] plan_path failed ({e}); "
                          "falling back to validated interpolation for this session")
            if candidate is None or len(candidate) == 0:
                candidate = lerp
            ok, why, tight = self._validate_path(candidate, carry=carry, ignore=ignore,
                                                 arm_clear=arm_clear,
                                                 carry_margin=carry_margin)
            if ok:
                path = candidate
                if tight:
                    d, what = min(tight)
                    self.safety_stats["tight"] += 1
                    print(f"  [safety] {label}: tight clearance "
                          f"({1000*(HARD_DEPTH - d):.1f}mm air, {what}) - acceptable")
                break
            print(f"  [safety] {label}: rejected path (attempt {attempt + 1}): {why}")
            arm_clear = max(0.02, arm_clear - 0.01)  # relax the blanket rule only
        if not ok:
            # never execute a hard-failed candidate (it can ram the board and
            # NaN the solver): fall back to the straight-line path, whose
            # endpoints are always in free space
            path = lerp
            self.safety_stats["hard"] += 1
            print(f"  [safety] {label}: WARNING: no clean path found ({why}); "
                  "executing straight-line fallback — a real robot should abort here")

        self._execute(path, carry)
        for _ in range(settle):
            self.robot.control_dofs_position(q_goal)
            self._step(carry)
        return True

    STEP_SCALE = 2  # dt=0.005 (vs the 0.01 the waypoint counts were tuned
                    # for): without rescaling, motions run 2x fast and the
                    # claw punches pieces off the board

    def _execute(self, path, carry=None):
        """Run a joint path with a smooth velocity profile (ease-in/ease-out):
        constant waypoint rate makes real servos start and stop with a jerk."""
        n = len(path)
        for t in np.linspace(0.0, 1.0, n * self.STEP_SCALE):
            s = t * t * (3.0 - 2.0 * t)  # smoothstep
            q = path[min(n - 1, int(round(s * (n - 1))))]
            self.robot.control_dofs_position(q)
            self._step(carry)

    def _set_claw(self, angle, steps=15, carry=None):
        """Drive the claw joint to `angle` with position control."""
        steps *= self.STEP_SCALE
        q = self.robot.get_qpos().clone()
        start = float(q[self.gripper_dofs_idx_local[0]])
        for a in np.linspace(0.0, 1.0, steps):
            q[self.gripper_dofs_idx_local[0]] = start + (angle - start) * a
            self.robot.control_dofs_position(q)
            self._step(carry)

    def _close_until_contact(self, ent, max_angle=0.45, step=0.010):
        # max_angle just past fully-closed (~0.3): sweeping further jams the
        # hook into the board and blows up the constraint solver
        """Physically close the claw until real finger-piece contact persists.
        Stops at LIGHT contact - squeezing further bakes penetration forces
        into the weld and destabilizes the solver. Returns the holding angle,
        or None if the piece was never touched."""
        q = self.robot.get_qpos().clone()
        ang = float(q[self.gripper_dofs_idx_local[0]])
        streak = 0
        while ang < max_angle:
            ang += step
            q[self.gripper_dofs_idx_local[0]] = ang
            self.robot.control_dofs_position(q)
            self._step()
            try:
                c = ent.get_contacts(with_entity=self.robot)
                n = len(c["link_a"]) if isinstance(c, dict) and "link_a" in c else 0
            except Exception:
                n = 0
            streak = streak + 1 if n > 0 else 0
            if streak >= 2:
                return ang
        return None

    def _weld(self, ent, on=True):
        solver = self.scene.sim.rigid_solver
        if on:
            solver.add_weld_constraint(self.ee_link.idx, ent.links[0].idx)
        else:
            solver.delete_weld_constraint(self.ee_link.idx, ent.links[0].idx)

    def _staging_point(self):
        """A safe waypoint beside the board for the active arm, above table height."""
        side = 1.0 if self.ee_name.endswith("right") else -1.0
        return np.array([side * 0.15, 0.10, BOARD_TOP_Z + 0.10])

    def _go_home(self, carry=None, waypoints=40):
        # retreat via the staging point so the arm doesn't drag across the board
        self._goto(self._staging_point(), waypoints=30, carry=carry, settle=0, plan=True, label='home.staging')
        q_now = self.robot.get_qpos().clone()
        self._execute([q_now * (1.0 - a) + self.home_qpos * a
                       for a in np.linspace(0.0, 1.0, waypoints)], carry)
        # hold until the joints actually converge, otherwise the next move's
        # commands freeze the arm wherever the lagging controller left it
        for _ in range(40):
            self.robot.control_dofs_position(self.home_qpos)
            self._step(carry)

    def _use_arm_for(self, x):
        ee = "hand_right" if x >= 0 else "hand_left"
        if ee != self.ee_name:
            self.ee_name = ee
            self._setup_arm_joints()

    # -------------------------
    # pick and place primitives
    # -------------------------
    def _grasp_z(self, ent):
        # ee height that puts the claw tips at the piece's mid-body
        return BOARD_TOP_Z + self.piece_h[ent] / 2 + FINGER_REACH

    def _true_grasp_point(self):
        """World position of the finger's actual contact region -- NOT the
        same as ee_link.get_pos(), which is just the joint pivot. Rotates
        the measured local mesh offset by the link's CURRENT orientation,
        since this arm's IK does not constrain orientation. Use this AFTER
        the claw has actually closed (quat already reflects it)."""
        pos = self.ee_link.get_pos().cpu().numpy()
        quat = self.ee_link.get_quat().cpu().numpy()
        local_off = FINGER_LOCAL_OFFSET[self.ee_name]
        return pos + _rotate_vec_by_quat(local_off, quat)

    def _predicted_grasp_point(self):
        """FIX 2: like _true_grasp_point(), but call this WHILE the claw is
        still open (before closing) to predict where the contact point will
        land once it closes -- pre-rotates the local offset by the known
        open->hold delta around the finger's own joint axis, before mapping
        through the currently-observed (open) world orientation."""
        pos = self.ee_link.get_pos().cpu().numpy()
        quat = self.ee_link.get_quat().cpu().numpy()
        local_off = FINGER_LOCAL_OFFSET[self.ee_name]
        delta = CLAW_HOLD - CLAW_OPEN
        predicted_local = _axis_angle_rotate(local_off, _FINGERPINCH_AXIS, delta)
        return pos + _rotate_vec_by_quat(predicted_local, quat)

    def _claw_tilt_deg(self):
        """FIX 3 diagnostic: angle (degrees) between the claw's reach
        direction and straight-down world -Z. Directly visualizes the
        uncontrolled-orientation variance this arm is known to have."""
        quat = self.ee_link.get_quat().cpu().numpy()
        local_off = FINGER_LOCAL_OFFSET[self.ee_name]
        reach_dir = local_off / np.linalg.norm(local_off)
        world_dir = _rotate_vec_by_quat(reach_dir, quat)
        world_dir = world_dir / np.linalg.norm(world_dir)
        cos_a = np.clip(np.dot(world_dir, np.array([0.0, 0.0, -1.0])), -1.0, 1.0)
        return np.degrees(np.arccos(cos_a))

    def _pick(self, ent):
        """Approach, descend, physically close on the piece (contact-verified),
        weld the pinch, and lift. Returns the carry tuple or None."""
        for attempt in range(2):
            p = ent.get_pos().cpu().numpy()
            grasp = np.array([p[0], p[1], self._grasp_z(ent)])
            above = grasp + np.array([0.0, 0.0, APPROACH_CLEAR])
            transit = np.array([p[0], p[1], BOARD_TOP_Z + TRANSIT_CLEAR])
            self._set_claw(CLAW_OPEN, steps=10)
            # lift beside the board, then travel at transit height so the
            # fingers clear even the kings, then descend at the target square
            self._goto(self._staging_point(), waypoints=30, settle=0, plan=True, label='pick.staging')
            # the transit point hovers directly over the piece we're grabbing
            if not self._goto(transit, waypoints=45, plan=True, ignore=(ent,),
                              label='pick.transit'):
                return None
            # descending onto the target piece itself is intentional
            if not self._goto(above, waypoints=15, ignore=(ent,), cartesian=True, label='pick.above'):
                return None
            if not self._goto(grasp, waypoints=25, ignore=(ent,), cartesian=True, label='pick.grasp'):
                return None
            # corrective pass: now that we know the achieved orientation,
            # re-target so the finger's true contact point (not its pivot)
            # lands on the piece
            naive_offset = FINGER_LOCAL_OFFSET[self.ee_name]
            predicted_offset = self._predicted_grasp_point() - self.ee_link.get_pos().cpu().numpy()
            print(f"  [diag-fix1] tip-offset magnitude: {1000*np.linalg.norm(naive_offset):.1f}mm")
            print(f"  [diag-fix2] closing-rotation shift applied: "
                  f"{1000*np.linalg.norm(predicted_offset - _rotate_vec_by_quat(naive_offset, self.ee_link.get_quat().cpu().numpy())):.1f}mm")
            corrected = grasp - predicted_offset
            seed = self.robot.get_qpos().clone()
            ik = self._ik(corrected, seed_qpos=seed)
            self._execute([ik], None)
            if PHYSICS_PIECES:
                ok = self._close_until_contact(ent) is not None
            else:
                # geometrically verified grasp: close the claw around the
                # piece and require the piece to actually be inside it
                self._set_claw(CLAW_HOLD, steps=12)
                tilt = self._claw_tilt_deg()
                print(f"  [diag-fix3] claw tilt from vertical: {tilt:.1f}deg"
                      f"  (ref: Prasham's morphology_study measured 12-78deg "
                      f"on the original hand)")
                d = np.linalg.norm(ent.get_pos().cpu().numpy()[:2]
                                   - self._true_grasp_point()[:2])
                ok = d < 0.014
                if not ok:
                    print(f"  [grasp] piece not inside claw ({1000*d:.1f}mm off)")
            if ok:
                break
            print("  [grasp] missed - reopening and retrying")
            self._set_claw(CLAW_OPEN, steps=10)
        else:
            print("  [grasp] FAILED twice - aborting pick")
            return None
        if PHYSICS_PIECES:
            self._weld(ent, on=True)
        off = ent.get_pos().cpu().numpy() - self.ee_link.get_pos().cpu().numpy()
        carry = (ent, off)
        # lift back to transit height so the carried piece clears the others;
        # lean the ascent slightly toward the robot: IK drifts a few mm away
        # from it here, straight toward the crowns of the rank behind
        lean = np.array([0.0, -0.006, 0.0])
        self._goto(above + lean, waypoints=25, carry=carry, carry_margin=0.002,
                   cartesian=True, label='pick.lift')
        self._goto(transit + lean, waypoints=20, carry=carry, carry_margin=0.002,
                   cartesian=True, label='pick.lift2')
        return carry

    def _place(self, carry, dest_xy):
        """Carry the held piece over dest, set it down and release physically."""
        ent, _ = carry
        transit = np.array([dest_xy[0], dest_xy[1], BOARD_TOP_Z + TRANSIT_CLEAR])
        self._goto(transit, waypoints=40, carry=carry, plan=True, label='place.transit')
        # the piece hangs at an offset from the ee (wherever the pinch caught
        # it, plus sag during transport): measure NOW, after the carry, and
        # aim the ee so the PIECE lands on the square center
        off = ent.get_pos().cpu().numpy() - self.ee_link.get_pos().cpu().numpy()
        ee_xy = np.asarray(dest_xy) - off[:2]
        grasp = np.array([ee_xy[0], ee_xy[1], self._grasp_z(ent)])
        above = grasp + np.array([0.0, 0.0, APPROACH_CLEAR])
        self._goto(above, waypoints=15, carry=carry, cartesian=True, label='place.above')
        # setting down between neighbours needs a tighter (but nonzero) margin
        self._goto(grasp, waypoints=25, carry=carry, carry_margin=0.002, cartesian=True, label='place.down')
        if PHYSICS_PIECES:
            # release: open the claw FIRST while the weld still holds the
            # piece rigid (the swinging claw would otherwise knock it over),
            # then cut the weld and let the piece settle onto the square
            self._set_claw(CLAW_OPEN, steps=15)
            self._weld(ent, on=False)
            for _ in range(15):
                self._step()
            # referee correction, only when physics left the piece badly placed
            p = ent.get_pos().cpu().numpy()
            err = float(np.linalg.norm(p[:2] - dest_xy))
            tipped = abs(float(ent.get_quat().cpu().numpy()[0])) < 0.95
            if tipped or err > 0.006:
                print(f"  [place] referee correction ({1000*err:.1f}mm off"
                      f"{', tipped' if tipped else ''})")
                ent.set_pos(np.array([dest_xy[0], dest_xy[1], BOARD_TOP_Z + 0.0005]))
                ent.set_quat(np.array([1.0, 0.0, 0.0, 0.0]))
                try:
                    ent.zero_all_dofs_velocity()
                except Exception:
                    pass
        else:
            # set the piece down on its square, then open the claw around it
            ent.set_pos(np.array([dest_xy[0], dest_xy[1], BOARD_TOP_Z + 0.0005]))
            self._set_claw(CLAW_OPEN, steps=12)
        # retreating straight up from the piece we just set down is intentional
        self._goto(above, waypoints=25, ignore=(ent,), cartesian=True, label='place.retreat')

    def _move_piece(self, ent, dest_xy):
        p = ent.get_pos().cpu().numpy()
        self._use_arm_for((p[0] + dest_xy[0]) / 2)
        carry = self._pick(ent)
        if carry is None:
            print("  !! pick failed, teleporting piece instead")
            ent.set_pos(np.array([dest_xy[0], dest_xy[1], BOARD_TOP_Z + 0.0005]))
            ent.set_quat(np.array([1.0, 0.0, 0.0, 0.0]))
            try:
                ent.zero_all_dofs_velocity()
            except Exception:
                pass
            return
        self._place(carry, dest_xy)

    def _graveyard_slot(self, x_hint):
        side = 1 if x_hint >= 0 else -1
        i = self.grave_count[side]
        self.grave_count[side] += 1
        col, row = divmod(i, 8)
        x = side * (4 * SQUARE + 0.035 + 0.028 * col)
        y = 0.165 + 0.030 * row
        return np.array([x, y])

    # -------------------------
    # chess-level move execution
    # -------------------------
    def _restore_bystanders(self, snapshot, moved):
        """Board supervisor: physics is real, so the claw can bump bystander
        pieces - restore any that were displaced and log the incident (on the
        real robot each of these is a collision to engineer out)."""
        for ent, pos in snapshot.items():
            if ent in moved:
                continue
            p = ent.get_pos().cpu().numpy()
            if np.linalg.norm(p - pos) > 0.006:
                print(f"  [supervisor] {self.piece_label[ent]} was bumped "
                      f"{1000*np.linalg.norm(p[:2]-pos[:2]):.0f}mm - restoring")
                self.safety_stats["hard"] += 1
                ent.set_pos(pos)
                ent.set_quat(np.array([1.0, 0.0, 0.0, 0.0]))
                try:
                    ent.zero_all_dofs_velocity()
                except Exception:
                    pass

    def execute_move(self, board: chess.Board, move: chess.Move):
        """Physically perform `move` (assumed legal on `board`), then push it."""
        mover = self.piece_ent[move.from_square]
        snapshot = {ent: ent.get_pos().cpu().numpy().copy()
                    for ent in self.piece_ent.values()}
        moved = {mover}
        print(f"\n>>> executing {board.san(move)} ({move.uci()}) "
              f"with the {self.piece_label[mover]}")

        # 1. remove a captured piece first
        cap_sq = None
        if board.is_en_passant(move):
            cap_sq = move.to_square + (-8 if board.turn == chess.WHITE else 8)
        elif board.is_capture(move):
            cap_sq = move.to_square
        if cap_sq is not None and cap_sq in self.piece_ent:
            victim = self.piece_ent.pop(cap_sq)
            moved.add(victim)
            print(f"    capturing the {self.piece_label[victim]}")
            victim_x = victim.get_pos().cpu().numpy()[0]
            self._move_piece(victim, self._graveyard_slot(victim_x))

        # 2. move the piece itself
        self._move_piece(mover, square_center(move.to_square))
        self.piece_ent[move.to_square] = self.piece_ent.pop(move.from_square)

        # 3. castling: move the rook as well
        if board.is_castling(move):
            rank = chess.square_rank(move.from_square)
            if chess.square_file(move.to_square) == 6:   # kingside
                r_from, r_to = chess.square(7, rank), chess.square(5, rank)
            else:                                        # queenside
                r_from, r_to = chess.square(0, rank), chess.square(3, rank)
            rook = self.piece_ent.pop(r_from)
            moved.add(rook)
            print(f"    castling: moving the {self.piece_label[rook]}")
            self._move_piece(rook, square_center(r_to))
            self.piece_ent[r_to] = rook

        if move.promotion:
            print(f"    promotion to {chess.piece_name(move.promotion)} "
                  "(same physical piece is reused)")

        self._go_home()
        self._restore_bystanders(snapshot, moved)
        board.push(move)


# -------------------------
# game loop
# -------------------------
def print_board(board):
    print("\n    a b c d e f g h")
    print("  +-----------------+")
    for r in range(7, -1, -1):
        row = " ".join(str(board.piece_at(chess.square(f, r)) or ".") for f in range(8))
        print(f"{r+1} | {row} | {r+1}")
    print("  +-----------------+\n")


def main():
    selfplay = "--selfplay" in sys.argv
    test = "--test" in sys.argv
    headless = test or "--headless" in sys.argv
    max_plies = None
    if selfplay:
        i = sys.argv.index("--selfplay")
        max_plies = int(sys.argv[i + 1]) if len(sys.argv) > i + 1 and sys.argv[i + 1].isdigit() else 10
    # scripted line covering captures and castling, used by --test
    test_moves = ["e2e4", "d7d5", "e4d5", "d8d5", "g1f3", "b8c6",
                  "f1c4", "g8f6", "e1g1"]
    if test:
        max_plies = len(test_moves)

    if STOCKFISH_EXE is None:
        sys.exit("Stockfish binary not found under Simulation/genesis/stockfish/")
    engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_EXE)
    engine.configure({"Skill Level": STOCKFISH_SKILL})
    print(f"Stockfish ready: {STOCKFISH_EXE} (skill {STOCKFISH_SKILL})")

    robot = ChessRobot(
        urdf_file=URDF_FILE,
        ee_name="hand_right",
        robot_pos=tuple(ROBOT_BASE),
        show_viewer=not headless,
        cache_file=os.path.join(_SCRIPT_DIR, "chess_paths.pt"),
    )
    robot.settle()

    if "--record" in sys.argv:
        from hardware_bridge import TrajectoryRecorder
        i = sys.argv.index("--record")
        rec_path = (sys.argv[i + 1] if len(sys.argv) > i + 1
                    and not sys.argv[i + 1].startswith("-") else "game_traj.json")
        names = [j.name for j in robot.robot.joints if j.n_dofs > 0]
        robot.recorder = TrajectoryRecorder(rec_path, dt=0.01, joint_names=names)

    board = chess.Board()
    plies = 0
    try:
        while not board.is_game_over():
            if max_plies is not None and plies >= max_plies:
                break
            print_board(board)
            side = "White" if board.turn else "Black"

            if test:
                move = chess.Move.from_uci(test_moves[plies])
                print(f"[script/{side}] plays {board.san(move)}")
            elif selfplay or board.turn == chess.BLACK:
                result = engine.play(board, chess.engine.Limit(time=STOCKFISH_TIME))
                move = result.move
                print(f"[Stockfish/{side}] plays {board.san(move)}")
            else:
                try:
                    raw = input(f"[{side}] your move (uci/san, 'auto', 'q'): ").strip()
                except (EOFError, KeyboardInterrupt):
                    break
                if not raw:
                    continue
                if raw.lower() == "q":
                    break
                if raw.lower() == "auto":
                    move = engine.play(board, chess.engine.Limit(time=STOCKFISH_TIME)).move
                    print(f"[Stockfish for you] plays {board.san(move)}")
                else:
                    try:
                        move = chess.Move.from_uci(raw.lower())
                        if move not in board.legal_moves:
                            raise ValueError
                    except ValueError:
                        try:
                            move = board.parse_san(raw)
                        except ValueError:
                            print("Illegal or unparseable move, try again.")
                            continue

            robot.execute_move(board, move)
            plies += 1

        print_board(board)
        if board.is_game_over():
            print(f"Game over: {board.result()} ({board.outcome().termination.name})")

        if test:
            # verify the physical board matches the logical one
            errs = []
            for sq, ent in robot.piece_ent.items():
                p = ent.get_pos().cpu().numpy()
                c = square_center(sq)
                e = np.linalg.norm(p[:2] - c)
                errs.append(e)
                if e > 0.010:
                    print(f"  OFF {chess.square_name(sq)} {robot.piece_label[ent]}: "
                          f"{1000*e:.1f} mm  world={np.round(p, 3)}")
            print(f"TEST: {len(errs)} pieces tracked, "
                  f"max square-offset {1000*max(errs):.1f} mm, "
                  f"mean {1000*np.mean(errs):.1f} mm")
        print(f"Safety report: {robot.safety_stats['hard']} hard violations, "
              f"{robot.safety_stats['tight']} tight-clearance passes")
    except gs.GenesisException as e:
        if "Viewer closed" in str(e):
            print("Viewer window closed - exiting.")
        else:
            raise
    finally:
        engine.quit()
        if robot.recorder is not None:
            robot.recorder.save()


def _run_with_recovery():
    """Genesis's JIT on Windows intermittently fails with an LLVM relocation
    error caused by a corrupted kernel cache. Detect it, clear the cache and
    relaunch once automatically."""
    try:
        main()
    except RuntimeError as e:
        if "IMAGE_REL_AMD64" not in str(e):
            raise
        import shutil
        cache = os.path.expanduser("~/.cache/genesis")
        print("\n[recovery] Genesis kernel cache is corrupted - clearing "
              f"{cache} and relaunching...")
        shutil.rmtree(cache, ignore_errors=True)
        if os.environ.get("CHESS_RELAUNCHED") != "1":
            import subprocess
            env = {**os.environ, "CHESS_RELAUNCHED": "1"}
            sys.exit(subprocess.call(
                ["py", "-3.11", os.path.abspath(__file__), *sys.argv[1:]], env=env))
        raise


if __name__ == "__main__":
    _run_with_recovery()
