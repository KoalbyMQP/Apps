# ============================================================
# test_pickup_move.py
# Purpose: Simple main-method design to test picking up a piece and
#          moving it, WITHOUT Genesis. A MockMovementSystem records
#          every arm call so you can eyeball the exact motion script.

from chess_operations import ChessOrchestrator
import chess


class MockMovementSystem:
    """Stand-in for MovementSystem with the same call signatures.
    Records the arm's motion script instead of moving a robot."""

    def __init__(self):
        self.calls = []
        self.current_position = None

    def target(self, X, Y, Z):
        self.calls.append(("target", (round(X, 4), round(Y, 4), round(Z, 2))))
        self.current_position = (X, Y, Z)

    def grasp(self):
        self.calls.append(("grasp", self.current_position))

    def release(self):
        self.calls.append(("release", self.current_position))

    def waypoint(self):
        self.calls.append(("waypoint", (0.15, 0.10, 0.70)))
        self.current_position = (0.15, 0.10, 0.70)

    def dump(self, label):
        print(f"--- arm calls for: {label} ---")
        for name, pos in self.calls:
            print(f"  {name:9s} -> {pos}")
        self.calls.clear()


def main():
    movement = MockMovementSystem()
    # Same constants as genesis_setup.py: board_length=0.2032, BOARD_Y0=0.135
    game = ChessOrchestrator(movement, board_length=0.2032, board_y0=0.135)

    # --- Test 1: simple pawn move, e2 -> e4 (as a plain move) ---
    start, end = game.parse_move("e2e4")
    game.move(start, end)
    movement.dump("move e2->e4")
    print("board after move:\n", game.board, "\n")

    # --- Test 2: a capture, which exercises grasp + graveyard ---
    # Set up: white knight on d5, black pawn on d7 (knight can reach d7? no).
    # Use a simple position instead: white pawn e5, black pawn d5 -> exd5.
    game.board = chess.Board("4k3/8/8/3pP3/8/8/8/4K3 w - - 0 1")
    game.graveyard.clear()
    game.take(game.get_coordinate("e5"), game.get_coordinate("d5"))
    movement.dump("take exd5")
    print("graveyard:", [(p, loc) for p, loc in game.graveyard])
    print("board after capture:\n", game.board)


if __name__ == "__main__":
    main()