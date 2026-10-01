# ============================================================
# genesis_setup.py
# Purpose: Builds genesis scene, and spawns : ground plane, robot, chess board, pieces.
import os
import warnings
import chess

warnings.filterwarnings("ignore", category=FutureWarning, message=r".*torch\.jit\.script.*is deprecated.*")
import genesis as gs
import time


class GenesisSetup:

    # Constructor - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def __init__(self, urdf_file, show_viewer = True, robot_pos=(0.0, 0.0, 0.735), board_length = 0.2032):
        """
        Constructor: Initialize GenesisSetup.
        """

        # Set Variables
        self.urdf_file = urdf_file
        self.show_viewer = show_viewer
        self.robot_pos = robot_pos
        self.board_length = board_length

        # Use GPU
        self.scene = gs.init(backend = gs.gpu)

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
                max_FPS=100,
            ),
            rigid_options = gs.options.RigidOptions(
                # Enable 'Neutral Collisions'
                enable_neutral_collision=True,
                # Physics Sim Step Time
                dt=0.005,
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

        # URDF



    # Methods - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def spawn_plane(self):
        """
        Summary: (Helper) Place a ground plane in Genesis scene.

        :return: NA
        """
        return self.scene.add_entity(gs.morphs.Plane())


    def spawn_board(self, board_length):
        """
        Summary: (Helper) Place a chess board in Genesis scene with given dimensions.

        :param board_length: size of chess board
        :return: NA
        """

        def square_center(sq):
            f, r = chess.square_file(sq), chess.square_rank(sq)
            x = (f - 3.5) * self.SQUARE
            y = self.BOARD_Y0 + (r + 0.5) * self.SQUARE
            return x, y

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
                x, y = square_center(sq)
                self.scene.add_entity(
                    gs.morphs.Box(size=(self.SQUARE, self.SQUARE, 0.0015),
                                  pos=(x, y, self.BOARD_TOP_Z + 0.00075),
                                  fixed=True, collision=False),
                    surface=gs.surfaces.Default(color=self.LIGHT_SQ_COLOR),
                )

        return board


    def spawn_pieces(self):
        """
        Summary: (Helper) Place chess pieces in Genesis scene on chess board with correct spacing.

        :return: NA
        """
        self.piece_ent = {}  # chess square -> genesis entity
        self.piece_h = {}  # entity -> piece height
        self.piece_r = {}  # entity -> piece max radius (for keep-out checks)
        self.piece_label = {}  # entity -> e.g. "white knight"

        for sq, piece in chess.Board().piece_map().items():
            rad, h = PIECE_DIMS[piece.piece_type]
            x, y = square_center(sq)
            ent = self.scene.add_entity(
                gs.morphs.Mesh(file=os.path.join(MESH_DIR, PIECE_MESH[piece.piece_type]),
                               pos=(x, y, self.BOARD_TOP_Z + 0.0005),
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


    def spawn_robot(self):
        """
        Summary: (Helper) Place a robot in Genesis scene.

        :return:
        """
        return self.scene.add_entity(
            gs.morphs.URDF(file=self.urdf_file, pos=self.robot_pos,
                           quat=(0, 0, 0, 1), fixed=True)
        )


    def setup(self, board_length = 100):
        """
        Summary: Build Genesis scene with helper functions.

        :return: NA
        """
        # Run Helpers
        plane = self.spawn_plane()
        board = self.spawn_board(board_length)
        pieces = self.spawn_pieces()
        robot = 1 #self.spawn_robot()
        self.scene.build()

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
    urdf_file = os.path.join(_SCRIPT_DIR, "new_hand_chessrobot_v3", "urdf", "new_hand_chessrobot_v3.urdf")

    # Run Simulation
    sim = GenesisSetup(urdf_file)
    sim.setup()


if __name__ == "__main__":
    main()