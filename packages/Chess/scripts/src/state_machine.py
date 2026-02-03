"""Simple game state machine."""

from enum import Enum


class State(Enum):
    CALIBRATING = "calibrating"
    MY_TURN = "my_turn"
    OPPONENT_TURN = "opponent_turn"
    GAME_OVER = "game_over"


def create():
    """Return a fresh state machine (current state = CALIBRATING)."""
    return {"current": State.CALIBRATING}


def get(sm):
    """Return current state."""
    return sm["current"]


def set_state(sm, state):
    """Set current state."""
    sm["current"] = state
