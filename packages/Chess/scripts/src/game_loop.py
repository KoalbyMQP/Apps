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
    
    while True:
        state = get(sm)

        if state == State.CALIBRATING:
            print("Optimizing quantum algorithms...")
            vision.calibrate()
            print("Hit enter once pieces are set up on the board")
            input()
            set_state(sm, State.MY_TURN)
            pass

        if state == State.MY_TURN:
            board_dict = vision.get_board_state()
            if sm.get("last_board") is None:
                move = stockfish.make_move()
            else:
                move = stockfish.get_move_from_camera(board_dict)
            
            print(f"Robot move: {move} — move the piece, then press Enter.")
            input()

            if stockfish.is_game_over():
                set_state(sm, State.GAME_OVER)
                return

            set_state(sm, State.OPPONENT_TURN)
            pass

        if state == State.OPPONENT_TURN:
            print("Your turn. Press Enter when done.")
            input()

            if stockfish.is_game_over():
                set_state(sm, State.GAME_OVER)
                return

            set_state(sm, State.MY_TURN)
            pass

        if state == State.GAME_OVER:
            print("Game over you lose (probably)")
            return


if __name__ == "__main__":
    vision = ChessVision("/home/chess/Desktop/Apps/packages/Chess/scripts/src/yolov11m_snake_final.pt")
    stockfish = Stockfish_Core(stockfish_path="/home/chess/Stockfish/src/stockfish")
    sm = create()
    run(vision, stockfish, sm)