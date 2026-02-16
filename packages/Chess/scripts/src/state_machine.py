"""Simple game state machine."""

from enum import Enum


class State(Enum):
    CALIBRATING = "Calibrating"
    MY_TURN = "My turn"
    OPPONENT_TURN = "Opponent turn"
    GAME_OVER = "Game over"


def create():
    """Return a fresh state machine (current state = CALIBRATING)."""
    return {"current": State.CALIBRATING}


def get(sm):
    """Return current state."""
    return sm["current"]


def set_state(sm, state):
    """Set current state."""
    sm["current"] = state
