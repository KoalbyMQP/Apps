# ============================================================
# orchestrator.py
# Purpose: State Machine for chess robot logic

from enum import Enum, auto

class State(Enum):
    IDLE = auto()
    START_TURN = auto()
    HOME = auto()
    WAYPOINT = auto()
    MOVE_TO = auto()
    GRASP = auto()
    RELEASE = auto()
    FINISH_TURN = auto()
