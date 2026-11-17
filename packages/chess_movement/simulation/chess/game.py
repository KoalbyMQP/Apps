# Imports & Files
import os
import time
import chess.engine
from stockfish import Stockfish
from simulation.genesis_setup import GenesisSetup
from simulation.sim_arm_controller import SimArmController
from simulation.chess.movement_system import MovementSystem
from simulation.chess.chess_operations import ChessOrchestrator
from simulation.chess.state_machine import Robot, RobotState

class Game:
    """
    game.py

    purpose: Implement turn taking, by utilizing the functionality of state_machine.py, and machine_vision.py
    """

# Constructor ==========================================================================================================

    def __init__(self):
        """
        Summary: Initialize the game object.

        :param typeless_state:
        """

        # Setup --------------------------------------------------------------------------------------------------------

        self.typeless_state = "BBBBBBBBBBBBBBBB32WWWWWWWWWWWWWWWW"
        self.robot_team = "black"
        self.last_game_state = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1" # Default game starting position.
        self.current_game_state = None
        self.new_game_state = None
        self.stockfish = Stockfish(path="/usr/games/stockfish")

        # Initialization -----------------------------------------------------------------------------------------------

        self._DIR = os.path.dirname(os.path.abspath(__file__))
        self.urdf = os.path.join(self._DIR, "..", "master_assembly_v6", "urdf", "master_assembly_v6.urdf")

        # Simulation
        self.sim = GenesisSetup(self.urdf)
        self.sim.setup()

        # Controller
        self.r_arm = SimArmController(self.sim.scene, self.sim.robot, self.urdf, "right", self.sim.piece_ent)
        self.l_arm = SimArmController(self.sim.scene, self.sim.robot, self.urdf, "left", self.sim.piece_ent)

        # Movement System
        self.r_ms = MovementSystem(self.r_arm)
        self.l_ms = MovementSystem(self.l_arm)

        # Chess Operations
        self.chess_operations = ChessOrchestrator(
            left_movement=self.l_ms,
            right_movement=self.r_ms,
            board_length=0.2032,
            board_y0=self.sim.BOARD_Y0,
            lift_z=0.621
        )

        # State Machine
        self.robot = Robot(chess=self.chess_operations)
        self.robot.initialize_robot()

# Functions ============================================================================================================

    def find_current_state(self, input_typeless_state, promotion=chess.QUEEN):
        """
        Summary: Given a last_game_state, and a typeless game state (chess board, but it only shows where black and
                 white pieces are not what type each piece is) return the difference in position.

        :param input_typeless_state: a FEN board, where there are only 'B' (black) and 'W' (white), where a number represents empty tiles.
        :return: Updated FEN of current game state.
        """

        target = "".join("." * int(c) if c.isdigit() else c.upper()
                         for c in input_typeless_state if c.isalnum())
        board = chess.Board(self.last_game_state)
        for move in board.legal_moves:
            if move.promotion not in (None, promotion):
                continue
            board.push(move)
            sig = "".join("." if (p := board.piece_at(chess.square(i % 8, 7 - i // 8))) is None
                          else "W" if p.color else "B" for i in range(64))
            if sig == target:
                self.last_game_state = board.fen()
                return self.last_game_state
            board.pop()
        raise ValueError("No move matches the new board state")

    def update_game(self, move_notation):
        """
        Summary: Given a move (in chess notation), update the game state and return new FEN string.

        :return:
        """
        board = chess.Board(self.current_game_state)
        move = chess.Move.from_uci(move_notation)
        board.push(move)
        self.current_game_state = board.fen()

    def ask_stockfish(self, search_time= 1.0):
        """
        Summary: Pass the current game state and team to the Stockfish engine, and return the best move.

        :param search_time: how long stockfish can spend searching for the answer.
        :return: Optimal move in chess notation format.
        """

        # Set the game state using a FEN string
        board = chess.Board(self.current_game_state)

        # Start Engine
        with chess.engine.SimpleEngine.popen_uci("/usr/games/stockfish") as engine:

            # Find best move
            result = engine.play(board, chess.engine.Limit(time=search_time))
            recommended_move = result.move

        # Return move in chess notation
        return recommended_move.uci()

    def take_turn(self):
        """
        Summary: Use helper functions: find_change(), find_legal_moves(), update_game(), and the functionality of
                 state_machine.py to complete a single chess turn.

        :return: NA
        """
        if self.current_game_state is None:
            self.current_game_state = self.last_game_state
        else:
            self.last_game_state = self.current_game_state
            self.current_game_state = self.find_current_state(self.typeless_state)

        move = self.ask_stockfish(search_time=0.5)
        self.update_game(move)
        self.chess_operations.pending_moves.append(move)

        try:
            while True:
                self.robot.robot_loop()
                time.sleep(self.robot.loop_period)
        finally:
            self.robot.emergency_shutdown()

# Main =================================================================================================================

def main():

    # Test Game.py -----------------------------------------------------------------------------------------------------

    game = Game()
    game.take_turn()


if __name__ == "__main__":
    main()