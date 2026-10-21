# ============================================================
# genesis_setup.py
# Purpose: Builds simulation scene, and spawns : ground plane, robot, chess board, pieces.
import os
import warnings

from numpy.distutils.fcompiler import none

import chess
import numpy as np

warnings.filterwarnings("ignore", category=FutureWarning, message=r".*torch\.jit\.script.*is deprecated.*")
import genesis as gs
import time


class GenesisSetup:

    # Constructor - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def __init__(self, urdf_file, show_viewer=True, robot_pos=(0.0, 0.0, 0.735), board_length=0.2032, physics_pieces=False, self_collide=False):
        """
        Constructor: Initialize GenesisSetup.

        :param self_collide: robot self-collision. OFF by default: this URDF
               ships full-resolution CAD meshes as collision geometry, which
               interpenetrate at adjacent joints and pin the arm mid-swing
               (measured 192 mm end-effector error). With it off the arm can
               visually clip through its own torso on body-crossing paths.
        """

        # Set Variables
        self.robot = none
        self.urdf_file = urdf_file
        self.show_viewer = show_viewer
        self.robot_pos = robot_pos
        self.board_length = board_length
        self.PHYSICS_PIECES = physics_pieces
        self.SELF_COLLIDE = self_collide

        # Use GPU
        self.scene = gs.init(backend = gs.gpu)

        # Configure the rigid body solver options
        rigid_options = gs.options.RigidOptions(
            enable_self_collision=True,
            enable_adjacent_collision=False,
            # Set to True ONLY if your links don't overlappingly intersect at the joints
        )

        # Define Genesis Viewport Setting
        self.scene = gs.Scene(
            # Show Viewport

            show_viewer = self.show_viewer,

            # Define resolution, camera position, camera angle, camera fov, and FPS
            viewer_options = gs.options.ViewerOptions(
                res=(1280, 960),
                camera_pos=(2.2, 1.6, 1.8),
                camera_lookat=(0.0, 0.25, 0.6),
                camera_fov=40,
                max_FPS=700,
            ),
            rigid_options = gs.options.RigidOptions(
                # Enable 'Neutral Collisions'
                enable_neutral_collision=False,
                # Robot self-collision: this URDF ships full-resolution CAD
                # meshes as collision geometry, which interpenetrate at
                # adjacent joints and can pin the arm mid-swing (measured
                # 192 mm end-effector error on the old direct-line route).
                # Toggle via GenesisSetup(self_collide=True/False).
                enable_self_collision=self_collide,
                # Physics Sim Step Time
                dt=0.005,
                # Reserve slots for the runtime magnet weld (added/removed live)
                max_dynamic_constraints=4,
                # Broad-phase candidate pair buffer: base * multiplier.
                # Dense piece meshes + CoACD hulls + graveyard slabs overflowed
                # the defaults (broad phase 150*8=1200, narrow phase 150*5=750)
                # during settle.
                max_collision_pairs=600,
                multiplier_collision_broad_phase=32,
            )
        )

        # Define Board Attributes (Default board is 8.0 in).
        self.SQUARE = self.board_length / 8
        self.BOARD_Y0 = 0.135
        self.BOARD_TOP_Z = 0.62
        self.BOARD_THICK = 0.02
        self.BOARD_COLOR = (0.1, 0.1, 0.1)
        self.LIGHT_SQ_COLOR = (0.8, 0.8, 0.8)
        self.PILLAR_COLOR = (0.2, 0.2, 0.2)

        # Define Piece Attributes — (max horizontal radius, height) in metres,
        # measured from the low-poly meshes in chess_pieces/processed (scaled so
        # king = 42 mm tall; bases 19.2 mm across fit the 25.4 mm squares).
        self.PIECE_DIMS = {
            chess.PAWN: (0.0096, 0.0306),
            chess.ROOK: (0.0096, 0.0389),
            chess.KNIGHT: (0.0097, 0.0411),
            chess.BISHOP: (0.0097, 0.0417),
            chess.QUEEN: (0.0097, 0.0420),
            chess.KING: (0.0097, 0.0420),
        }

        _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
        self.MESH_DIR = os.path.join(_SCRIPT_DIR, "chess_pieces", "processed")
        self.PIECE_MESH = {
            chess.PAWN: "Pawn.stl", chess.ROOK: "Rook.stl", chess.KNIGHT: "Knight.stl",
            chess.BISHOP: "Bishop.stl", chess.QUEEN: "Queen.stl", chess.KING: "King.stl",
        }
        self.WHITE_COLOR = (0.92, 0.88, 0.78)
        self.BLACK_COLOR = (0.15, 0.12, 0.10)

        # Piece material: PLA (1240 kg/m^3 solid) at 90% infill -> ~1116 kg/m^3.
        # NOTE: real pieces carry a steel washer (~12-19 g total); pure-PLA hulls
        # will be lighter. Recalibrate rho = target_mass / hull_volume later if
        # weld/placement behavior looks too floaty.
        self.PIECE_RHO = 1240.0 * 0.90
        self.PIECE_FRICTION = 0.8

        # Graveyard: two queues, 16 slots each (2 cols x 8 rows), one per team.
        # A captured piece goes to its OWN army's queue; promote() fetches from
        # the promoting side's own queue.
        self.GRAVEYARD_SLOTS = 16
        self.graveyard: dict = {chess.WHITE: [None] * self.GRAVEYARD_SLOTS,
                                chess.BLACK: [None] * self.GRAVEYARD_SLOTS}
        # Which physical strip each team's queue maps to (set after reach check;
        # right strip is the right arm's side).
        self.graveyard_side = {chess.WHITE: "right", chess.BLACK: "left"}

        # Magnet (electromagnet model): weld between finger link and held piece.
        self.attached_piece = None

        # URDF



    # Methods - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def spawn_plane(self):
        """
        Summary: (Helper) Place a ground plane in Genesis scene.

        :return: NA
        """
        return self.scene.add_entity(gs.morphs.Plane())


    def square_center(self, sq):
        """
        Summary:
        :return:
        """
        f, r = chess.square_file(sq), chess.square_rank(sq)
        x = (f - 3.5) * self.SQUARE
        y = self.BOARD_Y0 + (r + 0.5) * self.SQUARE
        return x, y


    def spawn_board(self, board_length):
        """
        Summary: (Helper) Place a chess board in Genesis scene with given dimensions.

        :param board_length: size of chess board
        :return: NA
        """

        # board slab
        board_w = 8 * self.SQUARE + 0.018
        board_cy = self.BOARD_Y0 + 4 * self.SQUARE
        board = self.scene.add_entity(
            gs.morphs.Box(size=(board_w, board_w, self.BOARD_THICK),
                          pos=(0.0, board_cy, self.BOARD_TOP_Z - self.BOARD_THICK / 2),
                          fixed=True, collision=True),
            surface=gs.surfaces.Default(color=self.BOARD_COLOR),
        )

        # pedestal
        table_top = self.BOARD_TOP_Z - self.BOARD_THICK
        pillar_w = board_w * 0.75
        self.scene.add_entity(
            gs.morphs.Box(size=(pillar_w, pillar_w, table_top),
                          pos=(0.0, board_cy, table_top / 2),
                          fixed=True, collision=True),
            surface=gs.surfaces.Default(color=self.PILLAR_COLOR),
        )

        # checker pattern (visual only)
        for sq in chess.SQUARES:
            if (chess.square_file(sq) + chess.square_rank(sq)) % 2 == 0:
                x, y = self.square_center(sq)
                self.scene.add_entity(
                    gs.morphs.Box(size=(self.SQUARE, self.SQUARE, 0.0015),
                                  pos=(x, y, self.BOARD_TOP_Z + 0.00075),
                                  fixed=True, collision=False),
                    surface=gs.surfaces.Default(color=self.LIGHT_SQ_COLOR),
                )

        return board

    def spawn_grave_yard(self, side):
        """
        Summary: (Helper) Place a mini (2 by 8) graveyard board next to the real board,
                 holding dead pieces for one team. Separate object from the main board:
                 plain slab (no checker pattern), top surface slightly below BOARD_TOP_Z
                 so the playing field reads as the raised stage.

        :param side: 'left' or 'right' — which side of the board the graveyard sits on.
        :return: the graveyard slab entity
        """
        sign = 1.0 if side == "right" else -1.0

        # mini slab: 2 columns x 8 rows of SQUARE, same thickness as the board,
        # top surface 3 mm below the board top.
        strip_w = 2 * self.SQUARE + 0.018
        strip_d = 8 * self.SQUARE + 0.018
        board_w = 8 * self.SQUARE + 0.018
        strip_cx = sign * (board_w / 2 + strip_w / 2)
        strip_cy = self.BOARD_Y0 + 4 * self.SQUARE
        strip_top_z = self.BOARD_TOP_Z - 0.003
        slab = self.scene.add_entity(
            gs.morphs.Box(size=(strip_w, strip_d, self.BOARD_THICK),
                          pos=(strip_cx, strip_cy, strip_top_z - self.BOARD_THICK / 2),
                          fixed=True, collision=True),
            surface=gs.surfaces.Default(color=self.PILLAR_COLOR),
        )

        # faint slot markers (visual only, no collision): 2 cols x 8 rows
        for i in range(self.GRAVEYARD_SLOTS):
            x, y = self.graveyard_slot_xy(side, i)
            self.scene.add_entity(
                gs.morphs.Box(size=(self.SQUARE * 0.9, self.SQUARE * 0.9, 0.001),
                              pos=(x, y, strip_top_z + 0.0005),
                              fixed=True, collision=False),
                surface=gs.surfaces.Default(color=self.LIGHT_SQ_COLOR),
            )

        return slab

    def graveyard_slot_xy(self, side, slot):
        """
        Summary: World (x, y) of a graveyard slot. slot 0..15: column = slot//8
                 (0 = inner column, nearest the board), row = slot%8 (same y as
                 board ranks).

        :param side: 'left' or 'right' strip.
        :param slot: slot index 0..15.
        :return: (x, y) world coordinates at the strip top surface.
        """
        sign = 1.0 if side == "right" else -1.0
        board_w = 8 * self.SQUARE + 0.018
        col, row = slot // 8, slot % 8
        x = sign * (board_w / 2 + (col + 0.5) * self.SQUARE)
        y = self.BOARD_Y0 + (row + 0.5) * self.SQUARE
        return x, y

    def graveyard_enqueue(self, piece_color, piece_type, entity):
        """
        Summary: Reserve the next free slot in the owning team's graveyard queue
                 and record the dead piece there. Does NOT move the entity —
                 physically carrying it to graveyard_slot_xy(side, slot) is the
                 arm's job (Phase B/C).

        :param piece_color: chess.WHITE / chess.BLACK — owning team of the dead piece.
        :param piece_type: chess piece type (kept for promote() lookups).
        :param entity: the piece's simulation entity.
        :return: slot index used, or None if the queue is full.
        """
        queue = self.graveyard[piece_color]
        for slot, entry in enumerate(queue):
            if entry is None:
                queue[slot] = (piece_type, entity)
                return slot
        print("graveyard queue full")
        return None

    def graveyard_dequeue(self, piece_color, piece_type):
        """
        Summary: Pull the first graveyard entry of the given type out of the
                 owning team's queue (used by promote()).

        :param piece_color: chess.WHITE / chess.BLACK — promoting side's own queue.
        :param piece_type: desired chess piece type.
        :return: (piece_type, entity, slot) or None if not present.
        """
        queue = self.graveyard[piece_color]
        for slot, entry in enumerate(queue):
            if entry is not None and entry[0] == piece_type:
                queue[slot] = None
                return (entry[0], entry[1], slot)
        return None



    def spawn_pieces(self):
        """
        Summary: (Helper) Place chess pieces in Genesis scene on chess board with correct spacing.

        :return: NA
        """
        self.piece_ent = {}  # chess square -> simulation entity
        self.piece_h = {}  # entity -> piece height
        self.piece_r = {}  # entity -> piece max radius (for keep-out checks)
        self.piece_label = {}  # entity -> e.g. "white knight"

        for sq, piece in chess.Board().piece_map().items():
            rad, h = self.PIECE_DIMS[piece.piece_type]
            x, y = self.square_center(sq)
            ent = self.scene.add_entity(
                gs.morphs.Mesh(file=os.path.join(self.MESH_DIR, self.PIECE_MESH[piece.piece_type]),
                               pos=(x, y, self.BOARD_TOP_Z + 0.0005),
                               euler=(0, 0, 0 if piece.color else 180),
                               **({"convexify": True} if self.PHYSICS_PIECES else
                                  {"fixed": True, "collision": True})),
                material=gs.materials.Rigid(rho=self.PIECE_RHO,
                                            friction=self.PIECE_FRICTION),
                surface=gs.surfaces.Default(
                    color=self.WHITE_COLOR if piece.color else self.BLACK_COLOR),
            )
            self.piece_ent[sq] = ent
            self.piece_h[ent] = h
            self.piece_r[ent] = rad
            self.piece_label[ent] = (
                f"{'white' if piece.color else 'black'} {chess.piece_name(piece.piece_type)}")


    def spawn_robot(self):
        """
        Summary: (Helper) Place a robot in Genesis scene.

        :return:
        """
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(file=self.urdf_file, pos=self.robot_pos,
                           quat=(0, 0, 0, 1), fixed=True)
        )
        # The 'fixed' URDF base gets a collision box proxy at the root
        # ('box_baselink'). The pedestal entity is a separate collision object
        # at the same spot, so the wrist/hand report constant phantom contacts
        # with the pedestal while reaching over it. The visible pedestal
        # handles collisions; silence the invisible base proxy. NOTE: contype
        # is baked into the solver at scene.build(), so this MUST run before
        # build (spawn_robot does).
        for geom in self.robot.geoms:
            if "baselink" in geom.link.name:
                geom._contype = 0
                geom._conaffinity = 0
        return self.robot


    # Magnet (electromagnet) - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def magnet_attach(self, piece_entity, arm="right"):
        """
        Summary: Energize the electromagnet: weld the finger link to the piece so
                 physics carries it (swings, collides, falls if yanked — like the
                 real magnet). Call when pad is within capture distance of the
                 piece top. Requires scene.build() to have run.

        :param piece_entity: the piece's simulation entity to latch.
        :param arm: 'right' or 'left' — which hand's magnet.
        :return: NA
        """
        if self.attached_piece is not None:
            print("magnet already holding a piece; release first")
            return
        finger = self.robot.get_link(f"finger_{arm}")
        self.scene.sim.rigid_solver.add_weld_constraint(
            finger.idx, piece_entity.base_link.idx)
        self.attached_piece = (piece_entity, arm)

    def magnet_release(self):
        """
        Summary: De-energize the electromagnet: delete the weld so the piece is
                 handed back to physics at the hand's current pose.

        :return: NA
        """
        if self.attached_piece is None:
            return
        piece_entity, arm = self.attached_piece
        finger = self.robot.get_link(f"finger_{arm}")
        self.scene.sim.rigid_solver.delete_weld_constraint(
            finger.idx, piece_entity.base_link.idx)
        self.attached_piece = None


    def setup(self, board_length = 100):
        """
        Summary: Build Genesis scene with helper functions.

        :return: NA
        """
        # Run Helpers
        plane = self.spawn_plane()
        board = self.spawn_board(board_length)
        graveyard_right = self.spawn_grave_yard("right")
        graveyard_left = self.spawn_grave_yard("left")
        pieces = self.spawn_pieces()
        robot = self.spawn_robot()
        self.scene.build()

        # "settle" robot

        # after self.scene.build()
        n = robot.n_dofs
        robot.set_dofs_kp(np.full(n, 60.0))
        robot.set_dofs_kv(np.full(n, 4.0))
        robot.set_dofs_force_range(np.full(n, -10.0), np.full(n, 10.0))
        q_home = np.zeros(n)

        start_time = time.time()
        settle_duration = 10.0
        while time.time() - start_time < settle_duration:
            robot.control_dofs_position(q_home)  # must be every step, before step()
            self.scene.step()

        # (Testing) Run sim for 60s, so viewport can be examined
        start_time = time.time()
        duration = 60.0  # seconds

        while time.time() - start_time < duration:
            self.scene.step()

        # Return list of object in scene.
        return [plane, board, pieces, robot]


    # (Testing)  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def main():
    print("Loading Genesis Setup: ")

    # Set URDF
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(_SCRIPT_DIR, "master_magnet_hand_v5", "urdf", "master_magnet_hand_v5.urdf")

    # Run Simulation
    sim = GenesisSetup(urdf_file)
    sim.setup()


if __name__ == "__main__":
    main()