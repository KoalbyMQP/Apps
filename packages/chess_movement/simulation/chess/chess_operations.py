# ============================================================
# chess_operations.py
# Purpose: Create easy to use chess functions for taking, or moving pieces, as well as interfacing with Stock-fish

import chess

class ChessOrchestrator:

    def __init__(self, movement, board_length, board_y0, lift_z=0.7):
        self.movement = movement
        self.board_length = board_length
        self.BOARD_Y0 = board_y0
        self.SQUARE = board_length / 8
        self.LIFT_Z = lift_z  # Z height for arm travel; tune against BOARD_TOP_Z
        self.GRAVEYARD_X0 = 0.30  # World-space X of first graveyard slot
        self.GRAVEYARD_Y = 0.10   # World-space Y of graveyard row
        self.GRAVEYARD_PITCH = 0.02
        self.graveyard = []
        self.board = chess.Board()
        # self.color = color
        # Color will need to be stored initially in game

    def parse_move(self, move):
        #take move in chess notation
        start_square = move[:2]
        end_square = move[2:]

        start_location = self.get_coordinate(start_square)
        end_location = self.get_coordinate(end_square)

        return start_location, end_location

    def get_coordinate(self, square):
        #convert to coordinate
        file = ord(square[0].lower()) - ord('a')
        rank = int(square[1]) - 1
        return (file, rank)

    def board_conversion(self, location):
        #Turn into actual board location
        x, y = location

        world_x = -self.board_length / 2 + (x + 0.5) * self.SQUARE
        world_y = self.BOARD_Y0 + (y + 0.5) * self.SQUARE

        return (world_x, world_y)

    def _physically_move(self, start_world, end_world):
        """
        Summary: (Helper) Perform only the arm actions to carry an object between two
                 WORLD-space (x, y) points. Never touches the logic board — use this
                 for anything off-board (e.g. the graveyard).

        :param start_world: (x, y) world coordinate to grasp from.
        :param end_world: (x, y) world coordinate to place at.
        :return: NA
        """
        self.movement.target(start_world[0], start_world[1], self.LIFT_Z)
        self.movement.grasp()
        self.movement.target(end_world[0], end_world[1], self.LIFT_Z)
        self.movement.release()
        self.movement.waypoint()

    def move(self, start_location, end_location):
        """
        Summary: Move chess piece from start_location to empty tile at end_location.
                 Runs the arm, then mirrors the move on the logic board.

        :param start_location: X-Y coordinate of the piece's starting location.
        :param end_location: X-Y coordinate of empty tile, where piece must be placed.
        :return: NA
        """
        start_world = self.board_conversion(start_location)
        end_world = self.board_conversion(end_location)

        self._physically_move(start_world, end_world)

        self.board.push(
            chess.Move(
                chess.square(start_location[0], start_location[1]),
                chess.square(end_location[0], end_location[1])
            )
        )

        print(f"Moving from {start_location} to {end_location}")

    def remove(self, start_location):
        """
        Summary: (Helper) Take piece from start_location to next spot in 'grave-yard',
                 add to grave-yard queue, and remove it from the logic board.

        :param start_location: X-Y coordinate of the piece's starting location.
        :return: NA
        """
        square = chess.square(start_location[0], start_location[1])
        piece = self.board.piece_at(square)
        if piece is None:
            print("no piece at location")
            return
        piece_type = piece.piece_type

        graveyard_index = len(self.graveyard)
        graveyard_world = (self.GRAVEYARD_X0 + graveyard_index * self.GRAVEYARD_PITCH,
                           self.GRAVEYARD_Y)  # world coords — not a board square

        self.graveyard.append((piece_type, graveyard_world))

        # Physical leg only: graveyard is off-board, so it must not go through
        # move()/board_conversion or push onto the logic board.
        self._physically_move(self.board_conversion(start_location), graveyard_world)

        # Keep the logic board in sync with the physical board.
        self.board.remove_piece_at(square)

    def take(self, start_location, end_location):
        """
        Summary: Use 'remove()' and 'move()' to complete a full chess move where one piece captures another.

        :param start_location: X-Y coordinate of the attacking piece's starting location.
        :param end_location: X-Y coordinate of captured piece.
        :return:
        """

        self.remove(end_location)
        self.move(start_location, end_location)

    def promote(self, start_location, end_location, promotion_type):
        """
        Summary: Use 'remove' to take the pawn off of the start_location, then pull desired piece from 'grave-yard' and place on
                 end_location in the back row, making sure to replace piece in grave-yard queue.

        :param start_location: X-Y coordinate of the pawn.
        :param end_location: X-Y coordinate of the promoted piece.
        :param promotion_type: The type of piece used to replace the pawn.
        :return: NA
        """
        found_piece = False
        graveyard_world = None
        for index, (piece_type, location) in enumerate(self.graveyard):
                    if piece_type == promotion_type:
                        found_piece = True
                        graveyard_world = location
                        self.graveyard.pop(index)
                        break

        if not found_piece:
            print(f"no {chess.piece_name(promotion_type)} in graveyard to promote with")
            return

        # Take the pawn off the board (physically to the graveyard, logically off).
        self.remove(start_location)

        # Carry the replacement piece from the graveyard onto the promotion square.
        # Both legs are graveyard/world-space aware: never through move().
        end_world = self.board_conversion(end_location)
        self._physically_move(graveyard_world, end_world)

        # Record the promotion on the logic board.
        color = self.board.turn
        self.board.set_piece_at(
            chess.square(end_location[0], end_location[1]),
            chess.Piece(promotion_type, color)
        )

    def castle(self, direction, color):
        # Color here temporarily, will remove once stored as actual variable
        """
        Summary: Depending on the 'direction' use the move function to move the King and corresponding Rook to "Castle".

        :param direction: String 'left' or 'right' represents the direction of the castle.
        :param color: String 'black' or 'white' represents the team being played for.
        :return: NA
        """
        if color == "white":
            rank = 0
            king_from, king_to = (4, 0), (6, 0)
            rook_from, rook_to = ((7, 0), (5, 0)) if direction == "right" else ((0, 0), (3, 0))
        elif color == "black":
            rank = 7
            king_from, king_to = (4, 7), (6, 7)
            rook_from, rook_to = ((7, 7), (5, 7)) if direction == "right" else ((0, 7), (3, 7))
        else:
            print(f"unknown color {color}")
            return

        if direction not in ("left", "right"):
            print(f"unknown castle direction {direction}")
            return

        # Physically move king, then rook.
        self._physically_move(self.board_conversion(king_from), self.board_conversion(king_to))
        self._physically_move(self.board_conversion(rook_from), self.board_conversion(rook_to))

        # Mirror the castle on the logic board: relocate both pieces and clear
        # this color's castling rights (a single push per piece would double-count
        # moves and leave stale rights).
        piece_color = chess.WHITE if color == "white" else chess.BLACK
        board = self.board
        king = board.piece_at(chess.square(*king_from))
        rook = board.piece_at(chess.square(*rook_from))
        board.remove_piece_at(chess.square(*king_from))
        board.remove_piece_at(chess.square(*rook_from))
        board.set_piece_at(chess.square(*king_to), king if king is not None
                           else chess.Piece(chess.KING, piece_color))
        board.set_piece_at(chess.square(*rook_to), rook if rook is not None
                           else chess.Piece(chess.ROOK, piece_color))
        # Clear this color's castling rights via its rook corner squares.
        corners = (chess.BB_A1 | chess.BB_H1) if piece_color == chess.WHITE \
            else (chess.BB_A8 | chess.BB_H8)
        board.castling_rights &= ~corners

    def en_passant(self, start_location, end_location, capture_location):
        """
        Summary: Similar to 'take()' use 'remove' and 'move()' to complete a full chess move where one piece captures another,
                (but the pawn being captured and the final destination of the attacking pawn are not the same).

        :param start_location: Original X-Y coordinate of the attacking pawn.
        :param end_location: New X-Y coordinate of attacking pawn.
        :param capture_location: X-Y coordinate of the captured pawn.
        :return: NA
        """
        self.remove(capture_location)
        self.move(start_location, end_location)

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
    main()