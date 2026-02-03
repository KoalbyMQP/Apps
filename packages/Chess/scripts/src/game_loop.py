"""Simple game loop: state machine + vision -> stockfish. No IK; wait for Enter to advance."""

from state_machine import State, create, get, set_state
from vision import ChessVision
from stockfish_core import Stockfish_Core


def run(
    vision: ChessVision,
    stockfish: Stockfish_Core,
    sm: dict,
) -> None:
    """
    Run one iteration of the loop based on current state.
    Robot moves are printed; press Enter after manually moving the piece, then we advance.
    """
    state = get(sm)

    if state == State.CALIBRATING:
        vision.calibrate()
        set_state(sm, State.OPPONENT_TURN)
        return

    if state == State.MY_TURN:
        board_dict = vision.get_board_state()
        if sm.get("last_board") is None:
            move = stockfish.make_move()
        else:
            move = stockfish.get_move_from_camera(board_dict)
        if move and move != 0:
            print(f"Robot move: {move} — move the piece, then press Enter.")
            input()
        sm["last_board"] = vision.get_board_state()
        set_state(sm, State.OPPONENT_TURN)
        return

    if state == State.OPPONENT_TURN:
        print("Your turn. Press Enter when done.")
        input()
        set_state(sm, State.MY_TURN)
        return

    if state == State.GAME_OVER:
        pass
