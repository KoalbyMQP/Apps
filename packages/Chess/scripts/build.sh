#!/usr/bin/env bash
# Prepare + run for Chess package: install deps, sanity checks, then run game_loop.py.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SRC_DIR="$SCRIPT_DIR/src"

# Optional overrides (defaults match game_loop.py on the robot)
export STOCKFISH_PATH="${STOCKFISH_PATH:-/home/chess/Stockfish/src/stockfish}"
CHESS_MODEL_PATH="${CHESS_MODEL_PATH:-$SRC_DIR/yolov11m_snake_final.pt}"

echo "=== Chess: prepare ==="
cd "$PKG_ROOT"

# Use same Python we'll run with
PYTHON="${PYTHON:-python3}"
if ! command -v "$PYTHON" &>/dev/null; then
  echo "Error: $PYTHON not found." >&2
  exit 1
fi

# Install Python dependencies
if [ -f "$PKG_ROOT/requirements.txt" ]; then
  "$PYTHON" -m pip install -q -r "$PKG_ROOT/requirements.txt"
else
  "$PYTHON" -m pip install -q gpiozero depthai stockfish pyserial
fi

# Install Sensing Chess vision package (from git)
"$PYTHON" -m pip install -q "git+https://github.com/KoalbyMQP/Sensing.git@raspberry-pi/vision#subdirectory=Vision/modules/Chess"

# Sanity checks
if [ ! -f "$CHESS_MODEL_PATH" ]; then
  echo "Error: Chess model not found at $CHESS_MODEL_PATH" >&2
  exit 1
fi
if [ ! -x "$STOCKFISH_PATH" ] && [ ! -f "$STOCKFISH_PATH" ]; then
  echo "Error: Stockfish not found at $STOCKFISH_PATH" >&2
  exit 1
fi

echo "=== Chess: run ==="
cd "$SRC_DIR"
exec "$PYTHON" game_loop.py
