"""chess_piece_dims.py — (max radius, height) metres per chess piece. Single source
of truth: GenesisSetup and MovementSystem both import this; nothing else defines it.
Scaled x0.5714 (king 42mm -> 24mm) so the pawn press fits the 32mm rack stroke at
travel height and a carried piece clears the tallest board piece (2026-11 layout)."""
import chess

PIECE_DIMS = {
    chess.PAWN:   (0.0055, 0.0175),
    chess.ROOK:   (0.0055, 0.0222),
    chess.KNIGHT: (0.0055, 0.0235),
    chess.BISHOP: (0.0055, 0.0238),
    chess.QUEEN:  (0.0055, 0.0240),
    chess.KING:   (0.0055, 0.0240),
}