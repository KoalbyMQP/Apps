import os
import time
import warnings
import chess
import genesis as gs
import numpy as np
from simulation.chess_piece_dims import PIECE_DIMS

class GenesisSetup:
    """
    OLD_genesis_setup.py

    Purpose: Builds simulation scene, and spawns : ground plane, robot, chess board, graveyards, and pieces.
    """

# Constructor ==========================================================================================================

    def __init__(self, urdf_file, show_viewer=True, robot_pos=(0.0, 0.0, 0.735), board_length=0.2032,
                 physics_pieces=False, self_collide=False):
        """
        Summary: Initialize GenesisSetup object.

        :param urdf_file: file path of robot's URDF.
        :param show_viewer: False = headless simulation, True = viewer.
        :param robot_pos: Spawn point for the robot URDF.
        :param board_length: Length and width of chess board, from one edge of the play area to another. (1 tile = board_length / 8)
        :param physics_pieces: True = pieces are dynamic bodies (convex hulls); False = pinned in place.
        :param self_collide: True = robot links collide with each other. Keep False: this URDF ships
                             full-resolution CAD meshes as collision geometry, which interpenetrate at
                             adjacent joints and can pin the arm mid-swing (measured 192 mm EE error).
        """

        # Use GPU
        gs.init(backend=gs.gpu, logging_level="warning")

        # Set Variables
        self.robot = None
        self.urdf_file = urdf_file
        self.show_viewer = show_viewer
        self.robot_pos = robot_pos
        self.board_length = board_length
        self.PHYSICS_PIECES = physics_pieces

        # Define Genesis Scene Settings
        self.scene = gs.Scene(

            # Show Viewport
            show_viewer=self.show_viewer,

            # Define resolution, camera position, camera angle, camera fov, and FPS
            viewer_options=gs.options.ViewerOptions(

                res=(1280, 960),                        # resolution
                camera_pos=(2.2, 1.6, 1.8),             # camera location
                camera_lookat=(0.0, 0.25, 0.6),         # camera orientation
                camera_fov=40,                          # camera fov
                refresh_rate=150,                            # simulation max framerate
            ),

            # Configure the rigid body solver options
            rigid_options=gs.options.RigidOptions(

                enable_self_collision=self_collide,       # Robot self-collision (see __init__ param doc)
                enable_adjacent_collision=False,        # Prevent adjacent pieces joint from clipping
                dt=0.005,                               # Simulation tick speed.
                max_dynamic_constraints=4,              # Runtime-added constraints reserved by solver
                max_collision_pairs=600,                # Narrow-phase contact pairs processed by solver
                multiplier_collision_broad_phase=32,
            )
        )

        # Define Board Attributes (Default board: 8.0 by 8.0 inches).
        self.SQUARE = self.board_length / 8             # one square's width/length
        self.BOARD_Y0 = 0.135 - 1.5 * self.SQUARE       # front edge of board (zero point in Y axis)
        self.PEDESTAL_Y0 = 0.135                        # front edge of support pillar (zero point in Y axis)
        self.BOARD_THICK = 0.02                         # Thickness of board
        self.BOARD_COLOR = (0.1, 0.1, 0.1)              # Color of Dark Squares
        self.LIGHT_SQ_COLOR = (0.8, 0.8, 0.8)           # Color of Light Squares
        self.PILLAR_COLOR = (0.2, 0.2, 0.2)             # Color of support pillar

        # Define Piece Attributes — (max horizontal radius, height) in metres,
        # NOTE - (scaled so king = 42 mm tall; bases 19.2 mm across fit the 25.4 mm squares).
        # Table lives in chess_piece_dims.py (shared with MovementSystem).
        self.PIECE_DIMS = PIECE_DIMS

        # Z height of board's playing surface.
        # 2026-11 dowel layout (king 45 mm): travel pinned to waypoint pad Z (0.621);
        # board floor from PAWN press fitting the 60 mm stroke: 0.621 - 0.060 - 0.0297
        # = 0.5313; 0.535 keeps 3.7 mm of margin. Must match MovementSystem.BOARD_TOP_Z.
        self.BOARD_TOP_Z = 0.535

        _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
        self.MESH_DIR = os.path.join(_SCRIPT_DIR, "chess_pieces", "processed")
        self.PIECE_MESH = {
            chess.PAWN: "Pawn.stl", chess.ROOK: "Rook.stl", chess.KNIGHT: "Knight.stl",
            chess.BISHOP: "Bishop.stl", chess.QUEEN: "Queen.stl", chess.KING: "King.stl",
        }
        self.WHITE_COLOR = (0.92, 0.88, 0.78)
        self.BLACK_COLOR = (0.15, 0.12, 0.10)

        # Piece material: PLA (1240 kg/m^3 solid) at 90% infill -> ~1116 kg/m^3.
        # NOTE - (Physics details if we decide to move pieces later on)
        self.PIECE_RHO = 1240.0 * 0.90
        self.PIECE_FRICTION = 0.8

        # Graveyard: two queues, 16 slots each (2 cols x 8 rows), one per team.
        # NOTE - A captured piece goes to its OWN army's queue;
        self.GRAVEYARD_SLOTS = 16
        self.graveyard: dict = {chess.WHITE: [None] * self.GRAVEYARD_SLOTS,
                                chess.BLACK: [None] * self.GRAVEYARD_SLOTS}

# Functions ============================================================================================================

    def spawn_plane(self):
        """
        Summary: (Helper) Place a ground plane in Genesis scene.

        :return: The ground-plane in Genesis scene.
        """
        return self.scene.add_entity(gs.morphs.Plane())

    def square_center(self, sq):
        """
        Summary: (Helper) find X and Y coordinates of a given game square.

        :param sq: Board square
        :return: X and Y coordinates of square
        """
        f, r = chess.square_file(sq), chess.square_rank(sq)
        x = (f - 3.5) * self.SQUARE
        y = self.BOARD_Y0 + (r + 0.5) * self.SQUARE
        return x, y

    def spawn_board(self):
        """
        Summary: (Helper) Place a chess board in Genesis scene with given dimensions.

        :return: The chess board in Genesis scene.
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
        pillar_height = self.BOARD_TOP_Z - self.BOARD_THICK
        pillar_w = board_w * 0.75
        pedestal_cy = self.PEDESTAL_Y0 + 4 * self.SQUARE
        self.scene.add_entity(
            gs.morphs.Box(size=(pillar_w, pillar_w, pillar_height),
                          pos=(0.0, pedestal_cy, pillar_height / 2),
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
        :return: the graveyard slab entity.
        """
        sign = 1.0 if side == "right" else -1.0

        # Graveyard Attributes
        strip_w = 2 * self.SQUARE + 0.018               # graveyard width (2 squares)
        strip_d = 8 * self.SQUARE + 0.018               # graveyard depth (8 squares)
        board_w = 8 * self.SQUARE + 0.018               # how wide the board actually is
        strip_cx = sign * (board_w / 2 + strip_w / 2)   # how far left/right the graveyard's center sits
        strip_cy = self.BOARD_Y0 + 4 * self.SQUARE      # how far forward/back the graveyard's center sits
        strip_top_z = self.BOARD_TOP_Z - 0.003          # graveyard top surface height

        # Create Entity + Add to Scene (function returns this entity)
        slab = self.scene.add_entity(
            gs.morphs.Box(size=(strip_w, strip_d, self.BOARD_THICK),
                          pos=(strip_cx, strip_cy, strip_top_z - self.BOARD_THICK / 2),
                          fixed=True, collision=True),
            surface=gs.surfaces.Default(color=self.PILLAR_COLOR),
        )

        # faint slot markers (visual only, no collision): 2 cols x 8 rows
        for i in range(self.GRAVEYARD_SLOTS):
            x, y = self.grave_center(side, i)
            self.scene.add_entity(
                gs.morphs.Box(size=(self.SQUARE * 0.9, self.SQUARE * 0.9, 0.001),
                              pos=(x, y, strip_top_z + 0.0005),
                              fixed=True, collision=False),
                surface=gs.surfaces.Default(color=self.LIGHT_SQ_COLOR),
            )

        return slab

    def grave_center(self, side, slot):
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

        :return: dict mapping chess square -> piece entity.
        """
        self.piece_ent = {}  # chess square -> simulation entity
        self.piece_h = {}  # entity -> piece height            # TODO: reserved for keep-out checks, unwired
        self.piece_r = {}  # entity -> piece max radius        # TODO: reserved for keep-out checks, unwired
        self.piece_label = {}  # entity -> e.g. "white knight"  # TODO: unwired

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

        return self.piece_ent

    def spawn_robot(self):
        """
        Summary: (Helper) Place a robot in Genesis scene.

        :return: The robot entity in Genesis scene.
        """
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(file=self.urdf_file, pos=self.robot_pos,
                           quat=(0, 0, 0, 1), fixed=True)
        )
        for geom in self.robot.geoms:
            if "baselink" in geom.link.name:
                geom._contype = 0
                geom._conaffinity = 0
        return self.robot

    def setup(self):
        """
        Summary: Build Genesis scene with helper functions.

        :return: List of objects in the Genesis scene.
        """

        # Run Helpers
        plane = self.spawn_plane()
        board = self.spawn_board()
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
        settle_duration = 2.0
        while time.time() - start_time < settle_duration:
            robot.control_dofs_position(q_home)  # must be every step, before step()
            self.scene.step()

        # Return list of object in scene.
        return [plane, board, pieces, robot]

# Main =================================================================================================================

def main():

        # Set URDF
        _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
        urdf_file = os.path.join(_SCRIPT_DIR, "master_assembly_v6", "urdf", "master_assembly_v6.urdf")

        # Run Simulation
        sim = GenesisSetup(urdf_file)
        plane, board, pieces, robot = sim.setup()

        # (Testing) step for 60s so the viewport can be examined
        start = time.time()
        while time.time() - start < 60.0:
            sim.scene.step()

if __name__ == "__main__":
    main()