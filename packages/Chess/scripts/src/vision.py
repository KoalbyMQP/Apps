"""
Chess vision wrapper: imports from Sensing repo and exposes calibrate + get_board_state.

Replicates the flow in Sensing/Vision/modules/Chess/src/main.py using
Localize (localization) and Model (cv_model) from that module.

Requires the Chess vision package to be installed from the Sensing repo. Use the same
Python you run this script with (e.g. python3 -m pip ... then python3 vision.py):

  python3 -m pip install "git+https://github.com/KoalbyMQP/Sensing.git@raspberry-pi/vision#subdirectory=Vision/modules/Chess"
"""

import time

import depthai as dai

from sensing_chess.cv_model import Model
from sensing_chess.localization import Localize


class ChessVision:
    """
    Thin wrapper around Sensing/Chess: calibration and board state from camera.
    """

    def __init__(self, model_path: str):
        self.model_path = model_path
        self._squares = None
        self._localizer = None

    def calibrate(self, distortion: bool = True) -> dict:
        """
        Run board calibration (AprilTags -> grid -> square centers).
        Must be called once before get_board_state.
        Returns: dict mapping square names to pixel coords, e.g. {"A1": (x, y), ...}.
        """
        device = dai.Device()
        self._localizer = Localize(device)
        self._squares = self._localizer.calibrate(distortion=distortion)
        device.close()
        time.sleep(0.5)
        return self._squares

    def get_board_state(self, distortion: bool = True) -> dict:
        """
        Capture frame, detect pieces, map to squares.
        Returns: dict mapping square names to piece class names (or "" if empty),
                 e.g. {"A1": "rook", "A2": "pawn", "B1": "", ...}.
        """
        if self._squares is None or self._localizer is None:
            raise RuntimeError("Call calibrate() before get_board_state()")
        device = dai.Device()
        model = Model(self.model_path, device)
        chess_pieces = model.predict2(self.model_path, distortion=True)
        device.close()
        time.sleep(0.5)

        full_board = self._localizer.localize(chess_pieces, self._squares)

        self.print_full_board(full_board)
        print(full_board)

        return full_board

    def print_full_board(self, full_board):
        """Print the full chess board with pieces in a visual 8x8 grid."""
        print("\n" + "="*80)
        print("CHESS BOARD STATE")
        print("="*80)
        
        # Sort squares by row (8 to 1) and column (A to H)
        sorted_board = sorted(full_board.items(), 
                            key=lambda x: (-int(x[0][1]), x[0][0]))
        
        # Print header
        print("\n      ", end="")
        for col in 'ABCDEFGH':
            print(f"    {col}     ", end="")
        print("\n   " + "-" * 76)
        
        # Print rows 8 to 1
        current_row = None
        for square_name, piece_name in sorted_board:
            row = square_name[1]
            if current_row != row:
                if current_row is not None:
                    print()
                print(f" {row} |", end="")
                current_row = row
            
            # Display piece or empty square
            if piece_name:
                # Truncate long names to fit
                display = piece_name[:8] if len(piece_name) <= 8 else piece_name[:5] + "..."
                print(f" {display:8s} |", end="")
            else:
                print(f" {'·':8s} |", end="")
        
        print("\n   " + "-" * 76)
        print("="*80 + "\n")

    @property
    def squares(self) -> dict | None:
        """Calibrated square centers (None until calibrate has been called)."""
        return self._squares


# Re-export so callers can use Sensing classes directly if needed
__all__ = ["ChessVision", "Model", "Localize"]


def main() -> None:
    """Test that sensing_chess imports work."""
    print("Import test: Model and Localize from sensing_chess")
    print(f"  Model: {Model}")
    print(f"  Localize: {Localize}")
    print("OK — imports work.")

    vision  = ChessVision("/home/chess/Desktop/Apps/packages/Chess/scripts/src/yolov11m_snake_final.pt")
    vision.calibrate()
    board_state = vision.get_board_state()
    print(board_state)


if __name__ == "__main__":
    main()
