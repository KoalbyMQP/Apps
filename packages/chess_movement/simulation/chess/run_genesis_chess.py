import os
import time

from simulation.genesis_setup import GenesisSetup
from simulation.sim_arm_controller import SimArmController
from simulation.chess.movement_system import MovementSystem
from simulation.chess.chess_operations import ChessOrchestrator
from simulation.chess.orchestrator import Robot, RobotState


def print_board(board, label):
    print(f"\n--- {label} ---")
    print(board)
    print(f"Turn: {'White' if board.turn else 'Black'}")


def run_turn(robot):
    print_board(robot.chess.board, "Before turn")

    robot.enter_start_turn()

    if robot.robot_state == RobotState.ERROR:
        raise RuntimeError(robot.error_reason)

    if robot.robot_state != RobotState.IDLE:
        raise RuntimeError(
            f"Turn did not finish. Current state: {robot.robot_state}"
        )

    print_board(robot.chess.board, "After turn")
    print("\nTurn completed successfully!")


def main():
    here = os.path.dirname(os.path.abspath(__file__))

    urdf = os.path.join(
        here,
        "..",
        "master_magnet_hand_v5",
        "urdf",
        "master_magnet_hand_v5.urdf"
    )

    print("Initializing Genesis...")
    sim = GenesisSetup(urdf, show_viewer=True)
    sim.setup()

    print("Initializing simulated arm...")
    arm = SimArmController(
        sim.scene,
        sim.robot,
        urdf,
        "right",
        sim.piece_ent
    )

    movement = MovementSystem(arm)

    game = ChessOrchestrator(
        movement,
        board_length=0.2032,
        board_y0=sim.BOARD_Y0
    )

    # Testing with caputre move

    game.board.set_fen(
        "rnbqkbnr/ppp1pppp/8/3p4/4P3/8/PPPP1PPP/RNBQKBNR w KQkq - 0 2"
    )

    #game.get_stockfish_move = lambda: "e4d5" # Change to test different moves, comment out for stockfish
    game.start_stockfish() # Actual stockfish
    
    robot = Robot(game)

    try:
        print_board(game.board, "Initial logical board")

        run_turn(robot)

        print_board(game.board, "Final logical board")

        while True:
            sim.scene.step()

    except KeyboardInterrupt:
        print("\nSimulation stopped.")

    finally:
        game.close_stockfish()


if __name__ == "__main__":
    main()