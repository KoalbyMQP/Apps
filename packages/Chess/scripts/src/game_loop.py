"""Simple game loop: state machine + vision -> stockfish -> manipulation."""

from state_machine import State, create, get, set_state
from vision import ChessVision
from stockfish_core import Stockfish_Core
from manipulation import Manipulation


def _execute_move(man: Manipulation, move: str) -> None:
    """Parse UCI move (e.g. 'e2e4') and call move_to_square for from and to."""
    if len(move) != 4:
        return
    from_sq = move[:2].upper()
    to_sq = move[2:].upper()
    man.move_to_square(from_sq)
    man.move_to_square(to_sq)


def run(
    vision: ChessVision,
    stockfish: Stockfish_Core,
    manipulation: Manipulation,
    sm: dict,
) -> None:
    """
    Run one iteration of the loop based on current state.
    Call this repeatedly (e.g. from main or a while loop).
    """
    state = get(sm)

    if state == State.CALIBRATING:
        vision.calibrate()
        set_state(sm, State.MY_TURN)
        return

    if state == State.MY_TURN:
        board_dict = vision.get_board_state()
        if sm.get("last_board") is None:
            move = stockfish.make_move()
        else:
            move = stockfish.get_move_from_camera(board_dict)
        if move and move != 0:
            _execute_move(manipulation, move)
        sm["last_board"] = vision.get_board_state()
        set_state(sm, State.OPPONENT_TURN)
        return

    if state == State.OPPONENT_TURN:
        board_dict = vision.get_board_state()
        if board_dict != sm.get("last_board"):
            move = stockfish.get_move_from_camera(board_dict)
            if move and move != 0:
                _execute_move(manipulation, move)
            sm["last_board"] = board_dict
        return

    if state == State.GAME_OVER:
        pass
