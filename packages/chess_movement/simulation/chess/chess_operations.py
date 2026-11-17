# ============================================================
# chess_operations.py
# Purpose: Create easy to use chess functions for taking, or moving pieces, as well as interfacing with Stock-fish

from movement_system import MovementSystem
import chess
import chess.engine

class ChessOrchestrator:

    def __init__(self, left_movement, right_movement, board_length, board_y0, lift_z=0.7):        
        self.engine = None
        #self.movement = movement
        self.left_movement = left_movement
        self.right_movement = right_movement
        self.board_length = board_length
        self.BOARD_Y0 = board_y0
        self.SQUARE = board_length / 8
        self.LIFT_Z = lift_z  # Z height for arm travel; tune against BOARD_TOP_Z
        self.GRAVEYARD_X0 = 0.30  # World-space X of first graveyard slot
        self.GRAVEYARD_Y = 0.10   # World-space Y of graveyard row
        self.GRAVEYARD_PITCH = 0.02
        self.HANDOFF_LOCATION = (
            0.0,
            board_y0 + board_length + 0.02
        )

        self.PROMOTION_SLOTS = {
            chess.QUEEN: ("graveyard", 0),
            chess.ROOK: ("graveyard", 1),
            chess.BISHOP: ("graveyard", 2),
            chess.KNIGHT: ("graveyard", 3)
        }

        self.CAPTURE_GRAVEYARD_START = 4

        self.graveyard = []
        self.board = chess.Board()
        # self.color = color
        # Color will need to be stored initially in game

    def start_stockfish(self):
        if self.engine is None:
            self.engine = chess.engine.SimpleEngine.popen_uci("/usr/games/stockfish")
    
    def get_stockfish_move(self):
        if self.board.is_game_over():
            return None

        if self.engine is None:
            self.start_stockfish()

        result = self.engine.play(
            self.board,
            chess.engine.Limit(time=1)
        )

        return result.move.uci()

    def board_conversion(self, location):
        #Turn into actual board location
        x, y = location

        world_x = -self.board_length / 2 + (x + 0.5) * self.SQUARE
        world_y = self.BOARD_Y0 + (y + 0.5) * self.SQUARE

        return (world_x, world_y)

    def get_arm_side(self, location):
        if location[0] == "graveyard":
            index = location[1]
            col, row = divmod(index, 8)

            x = self.board_length / 2 + 0.009 + (col + 0.5) * self.SQUARE
        else:
            x, y = self.board_conversion(location)

        if x < 0:
            return "left"
        else:
            return "right"
        
    def select_arm(self, location):
        if self.get_arm_side(location) == "left":
            return self.left_movement
        else:
            return self.right_movement

    def add_transfer_to_plan(self, plan, start_location, end_location):
        start_arm = self.get_arm_side(start_location)
        end_arm = self.get_arm_side(end_location)

        plan.append(("pick", start_location, start_arm))

        if start_arm != end_arm:
            plan.append(("place", self.HANDOFF_LOCATION, start_arm))
            plan.append(("pick", self.HANDOFF_LOCATION, end_arm))

        plan.append(("place", end_location, end_arm))

    def move_to_square(self, location, arm_side):
        movement = self.get_movement(arm_side)

        if location == self.HANDOFF_LOCATION:
            x, y = self.HANDOFF_LOCATION

        elif location[0] == "graveyard":
            index = location[1]
            x = self.GRAVEYARD_X0 + index * self.GRAVEYARD_PITCH
            y = self.GRAVEYARD_Y

        else:
            x, y = self.board_conversion(location)

        target = (x, y, movement.travel_z)
        print(f"Moving {arm_side} arm to: {target}")

        return movement.move_to(target)
    
    def create_move_plan(self, uci_move):
        move = chess.Move.from_uci(uci_move)

        if move not in self.board.legal_moves:
            raise ValueError(f"Illegal chess move: {uci_move}")

        start_location = (
            chess.square_file(move.from_square),
            chess.square_rank(move.from_square)
        )

        end_location = (
            chess.square_file(move.to_square),
            chess.square_rank(move.to_square)
        )

        plan = []

        if move.promotion:
            graveyard_index = self.CAPTURE_GRAVEYARD_START + len(self.graveyard)

            if self.board.is_capture(move):
                if self.board.is_en_passant(move):
                    captured_location = (
                        end_location[0],
                        start_location[1]
                    )
                else:
                    captured_location = end_location

                captured_graveyard = ("graveyard", graveyard_index)

                self.add_transfer_to_plan(
                    plan, captured_location, captured_graveyard
                )

                graveyard_index += 1

            pawn_graveyard = ("graveyard", graveyard_index)
            promotion_piece = self.PROMOTION_SLOTS[move.promotion]

            self.add_transfer_to_plan(
                plan, start_location, pawn_graveyard
            )

            self.add_transfer_to_plan(
                plan, promotion_piece, end_location
            )

            return move, plan

        if self.board.is_en_passant(move):
            captured_location = (
                end_location[0],
                start_location[1]
            )

            graveyard_location = (
                "graveyard",
                self.CAPTURE_GRAVEYARD_START + len(self.graveyard)
            )

            self.add_transfer_to_plan(
                plan, captured_location, graveyard_location
            )

            self.add_transfer_to_plan(
                plan, start_location, end_location
            )

            return move, plan

        if self.board.is_capture(move):
            graveyard_location = (
                "graveyard",
                self.CAPTURE_GRAVEYARD_START + len(self.graveyard)
            )

            self.add_transfer_to_plan(
                plan, end_location, graveyard_location
            )

            self.add_transfer_to_plan(
                plan, start_location, end_location
            )

            return move, plan

        if self.board.is_castling(move):
            rook_start_file = 7 if end_location[0] == 6 else 0
            rook_end_file = 5 if end_location[0] == 6 else 3
            rank = start_location[1]

            rook_start = (rook_start_file, rank)
            rook_end = (rook_end_file, rank)

            self.add_transfer_to_plan(
                plan, rook_start, rook_end
            )

            self.add_transfer_to_plan(
                plan, start_location, end_location
            )

            return move, plan

        self.add_transfer_to_plan(
            plan, start_location, end_location
        )

        return move, plan

    def commit_move(self, move):
        if move not in self.board.legal_moves:
            raise ValueError(f"Cannot commit illegal move: {move}")

        if self.board.is_capture(move):
            captured_square = move.to_square

            if self.board.is_en_passant(move):
                captured_square = (
                    move.to_square - 8
                    if self.board.turn == chess.WHITE
                    else move.to_square + 8
                )

            captured_piece = self.board.piece_at(captured_square)

            if captured_piece is not None:
                graveyard_location = (
                    "graveyard",
                    self.CAPTURE_GRAVEYARD_START + len(self.graveyard)
                )

                self.graveyard.append((
                    captured_piece.piece_type,
                    captured_piece.color,
                    graveyard_location
                ))

        if move.promotion:
            pawn_graveyard = (
                "graveyard",
                self.CAPTURE_GRAVEYARD_START + len(self.graveyard)
            )

            self.graveyard.append((
                chess.PAWN,
                self.board.turn,
                pawn_graveyard
            ))

        self.board.push(move)

    def execute_chess_move(self, uci_move):
        move = chess.Move.from_uci(uci_move)

        if move not in self.board.legal_moves:
            raise ValueError(f"Illegal chess move: {uci_move}")

        start_square = move.from_square
        end_square = move.to_square

        start_location = (
            chess.square_file(start_square),
            chess.square_rank(start_square)
        )
        end_location = (
            chess.square_file(end_square),
            chess.square_rank(end_square)
        )

        if self.board.is_castling(move):
            direction = "right" if chess.square_file(end_square) > chess.square_file(start_square) else "left"
            color = "white" if self.board.turn == chess.WHITE else "black"

            self.castle(direction, color)

        elif self.board.is_en_passant(move):
            capture_square = end_square - 8 if self.board.turn == chess.WHITE else end_square + 8
            capture_location = (
                chess.square_file(capture_square),
                chess.square_rank(capture_square)
            )

            self.en_passant(start_location, end_location, capture_location)

        elif move.promotion:
            promotion_type = chess.piece_symbol(move.promotion)
            self.promote(start_location, end_location, promotion_type)

        elif self.board.is_capture(move):
            self.take(start_location, end_location)

        else:
            self.move(start_location, end_location)

        print(f"Executed move: {uci_move}")

    def get_movement(self, arm_side):
        if arm_side == "left":
            return self.left_movement
        elif arm_side == "right":
            return self.right_movement
        else:
            raise ValueError(f"Unknown arm: {arm_side}")

    def _physically_move(self, start_world, end_world):
        """
        Summary: (Helper) Perform only the arm actions to carry an object between two
                 WORLD-space (x, y) points. Never touches the logic board — use this
                 for anything off-board (e.g. the graveyard).

        :param start_world: (x, y) world coordinate to grasp from.
        :param end_world: (x, y) world coordinate to place at.
        :return: NA
        """
        movement = self.get_movement(arm_side)

        movement.target(start_world[0], start_world[1], self.LIFT_Z)
        movement.grasp()
        movement.target(end_world[0], end_world[1], self.LIFT_Z)
        movement.release()
        movement.waypoint()

    def go_home(self, arm_side=None):
        if arm_side is None:
            self.left_movement.home()
            self.right_movement.home()
        else:
            movement = self.get_movement(arm_side)
            movement.home()

    def go_to_waypoint(self, arm_side=None):
        print(f"Waypoint command sent to: {arm_side}")
        if arm_side is None:
            self.left_movement.waypoint("home")
            self.right_movement.waypoint("home")
        else:
            movement = self.get_movement(arm_side)
            movement.waypoint("home")

    def toggle_grasp(self, location, arm_side, piece_type=None):
        movement = self.get_movement(arm_side)

        if piece_type is None:
            if location == self.HANDOFF_LOCATION:
                piece_type = self.held_piece_type

                if piece_type is None:
                    raise ValueError("No piece type stored for handoff")

            elif location[0] == "graveyard":
                for promotion_type, graveyard_location in self.PROMOTION_SLOTS.items():
                    if graveyard_location == location:
                        piece_type = promotion_type
                        break

                if piece_type is None:
                    raise ValueError(f"No promotion piece assigned to {location}")

            else:
                square = chess.square(*location)
                piece = self.board.piece_at(square)

                if piece is None:
                    raise ValueError(f"No piece at {chess.square_name(square)}")

                piece_type = piece.piece_type

            self.held_piece_type = piece_type

        return movement.toggle_grasp(piece_type)

# (Testing)  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


def main(show_viewer=True):
    """
    Summary: Smoke-test for the new trapezoidal target(): build the Genesis
             scene, wire up MovementSystem, command the pad to a point ABOVE
             THE ROBOT'S HEAD, then measure how far off the pad actually got.

    :param show_viewer: open the Genesis viewer (False for headless CI runs).
    :return: NA
    """
    import os
    import sys
    import time

    # genesis_setup.py lives one level up (simulation/), this file in chess/.
    _HERE = os.path.dirname(os.path.abspath(__file__))
    _SIM_DIR = os.path.dirname(_HERE)
    sys.path.insert(0, _SIM_DIR)

    from genesis_setup import GenesisSetup
    from OG_movement_system import MovementSystem, _np

    # --- 1. Define the target: over the middle right side of the board ----
    # Board spans x in [-0.1016, +0.1016], y in [0.135, 0.339]; mid-rank at
    # the right edge is (0.0889, 0.2366). target() now solves for the PAD
    # face directly (and keeps the rack axis vertical), so these are true pad
    # positions: finish with the pad 40 mm (~1.6 in) above the board top.
    PAD_GOAL = (0.0889, 0.2366, 0.66)
    WAYPOINT_PAD = (0.20, 0.12, 0.72)

    # --- 2. Build the scene (spawn helpers + build only — the settle/idle
    #        loops in setup() are left untouched; we settle here instead). --
    urdf_file = os.path.join(_SIM_DIR, "master_magnet_hand_v5", "urdf",
                             "master_magnet_hand_v5.urdf")
    sim = GenesisSetup(urdf_file, show_viewer=show_viewer)
    sim.spawn_plane()
    sim.spawn_board(sim.board_length)
    sim.spawn_grave_yard("right")
    sim.spawn_grave_yard("left")
    sim.spawn_pieces()
    sim.spawn_robot()
    sim.scene.build()

    # Position-loop gains. setup()'s 60/4/±10 is too weak for this arm — at full
    # reach gravity sags it >150 mm (measured). 500/25/±200 holds the raised
    # pose within ~40 mm. Tune per-joint later; the real HerkuleX servos will
    # have their own PID values.
    import numpy as np
    n = sim.robot.n_dofs
    sim.robot.set_dofs_kp(np.full(n, 500.0))
    sim.robot.set_dofs_kv(np.full(n, 25.0))
    sim.robot.set_dofs_force_range(np.full(n, -200.0), np.full(n, 200.0))
    for _ in range(200):  # short settle onto the zero pose
        sim.robot.control_dofs_position(np.zeros(n))
        sim.scene.step()

    # --- 3. Wire the movement layer and drive the arm ----------------------
    movement = MovementSystem(sim.robot, sim.scene)

    # Two-leg route: short hop out to a staging point off the board's front-right
    # corner, then one straight diagonal over the board down to the square.
    # No half-T detour, no climbing to the ceiling.
    print(f"target pad -> ({PAD_GOAL[0]:.3f}, {PAD_GOAL[1]:.3f}, {PAD_GOAL[2]:.3f})")
    t0 = time.time()
    movement.target(*WAYPOINT_PAD)   # leg 1: stage off the board corner
    movement.target(*PAD_GOAL)       # leg 2: diagonal in, finish ~2 in over board
    print(f"target() took {time.time() - t0:.1f}s of stepped sim time")

    # --- 4. Did the hand get there? ----------------------------------------
    # Report BOTH frames: the finger-link origin is what IK actually solves
    # for; the pad face is where the magnet is, but its position also depends
    # on wrist orientation, which position-only IK does not control.
    pad = movement.pad_world_pos()
    finger = _np(movement.finger_link.get_pos())
    link_goal = np.array(PAD_GOAL) - np.array(MovementSystem.PAD_OFFSET_DOWN)

    err = pad - np.array(PAD_GOAL)
    dist_mm = float(np.linalg.norm(err)) * 1000.0
    link_mm = float(np.linalg.norm(finger - link_goal)) * 1000.0
    print(f"finger actual -> ({finger[0]:.4f}, {finger[1]:.4f}, {finger[2]:.4f})  "
          f"(goal {np.round(link_goal, 4)}, error {link_mm:.1f} mm)")
    print(f"pad actual -> ({pad[0]:.4f}, {pad[1]:.4f}, {pad[2]:.4f})")
    print(f"pad error: dx={err[0]*1000:+.1f}mm dy={err[1]*1000:+.1f}mm dz={err[2]*1000:+.1f}mm")
    print(f"PAD TOTAL ERROR: {dist_mm:.1f} mm")
    print(f"PAD AXIS TILT FROM VERTICAL: {movement.pad_axis_tilt_deg():.1f} deg")

    # Hold the viewer open so the motion can be inspected after it lands.
    if show_viewer:
        for _ in range(6000):
            sim.scene.step()


if __name__ == "__main__":
    chess_game = ChessOrchestrator(
        movement=None,
        board_length=0.2032,
        board_y0=0.135
    )

    chess_game.board = chess.Board(
        "4k3/P7/8/8/8/8/8/4K3 w - - 0 1"
    )

    chess_game.graveyard.append(
        (chess.QUEEN, chess.WHITE, (0.30, 0.10))
    )

    chess_game._physically_move = lambda start, end: print(
        f"Physical movement: {start} -> {end}"
    )

    chess_game.execute_chess_move("a7a8q")

    print("Board after promotion:")
    print(chess_game.board)
    print("Piece on a8:", chess_game.board.piece_at(chess.A8))
    print("Side to move:", "White" if chess_game.board.turn else "Black")