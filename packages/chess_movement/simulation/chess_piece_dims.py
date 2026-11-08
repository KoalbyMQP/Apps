"""chess_piece_dims.py — (max radius, height) metres per chess piece. Single source
of truth: GenesisSetup and MovementSystem both import this; nothing else defines it.
2026-11 dowel pieces: king/queen z-scaled to 52 mm tall (Ø20 mm bases fit the 25.4 mm
squares); board lowered to 0.530 so the pawn press (56.7 mm) fits the 60 mm rack
stroke at the 0.621 travel anchor."""
import chess

PIECE_DIMS = {
    chess.PAWN:   (0.0100, 0.0297),
    chess.ROOK:   (0.0100, 0.0394),
    chess.KNIGHT: (0.0100, 0.0378),
    chess.BISHOP: (0.0100, 0.0399),
    chess.QUEEN:  (0.0100, 0.0450),
    chess.KING:   (0.0100, 0.0450),
}