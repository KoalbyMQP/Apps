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
        return self._localizer.localize(chess_pieces, self._squares)

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
