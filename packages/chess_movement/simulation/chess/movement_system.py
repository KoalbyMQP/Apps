import numpy as np
import chess
from simulation.chess_piece_dims import PIECE_DIMS

class MovementSystem:
    """
    movement_system.py

    Purpose: Implement basic movement functionality for the robot's arm in 3d Space (hardware or simulation).
    """

# Class Attributes =====================================================================================================

    # Chess-robot specifics (the stuff that is NOT general motion) ----
    DOWN = (0.0, 0.0, -1.0)  # pad axis for press-down work
    RACK_RETRACTED = 0.060  # must match ArmController (rack stroke top)

    # guards (from the 48-target validation)
    SKIP_TOL = 0.05  # m, refuse the move if IK missed XY by more
    ROT_LIMIT_DEG = 20.0  # refuse if the pad isn't aimed close enough

    # Poses (r_shoulder_x, r_shoulder_y, r_elbow_x, r_wrist_z, r_wrist_x)
    CANONICAL_ARM = (1.0, 0.0, 1.0, 0.0, 0.5)
    UP_POSTURE = (0.0, 0.8, 0.8, 0.0, 0.0)  # stopover so the wrist clears the graveyard slab
    WAYPOINT_ARM = (1.62, -0.03, -1.57, -0.02, -1.53)
    REST_ARM = (0.0, 0.0, 0.0, 0.0, 0.0)

    # Grasp & Release
    MAGNET_HEIGHT = 0.020
    TRAVEL_MARGIN = 0.620 - 0.535 - 0.045  # -> 0.041; anchor 0.621 == waypoint pad Z (BOARD_TOP_Z + h_max + margin)
    GRASP_PRESS = 0.002  # extra mm of rack travel into contact
    BOARD_TOP_Z = 0.535  # board playing surface world Z (must match GenesisSetup; hardware: calibrate once)
    STROKE_EPS = 1e-9  # float slop so a full-stroke press (pawn) isn't rejected by rounding

# Constructor ==========================================================================================================

    def __init__(self, arm):
        """
        Summary: Initialize basic movement system object for moving and placing and using the end-effector in 3d Space.

        :param arm:
        """

        self.arm = arm
        self._hover_xy = []  

# Functions ============================================================================================================

    def toggle_grasp(self, piece_type, speed=None):
        """
        Summary: Grab a piece off the board using ONLY the rack & pinion. The caller
                 has already used move_to() to place the pad over the piece's XY at
                 travel height ("Start" Z); grasp does NOT call move_to(). It:

                    1. Extends the rack down onto this piece's crown
                       (target Z = surface + piece height; extension = Start - target)
                    2. Toggles the electromagnet on to claim the piece
                    3. Retracts the rack, carrying the piece up with the pad

        :param piece_type: The type of piece to grab from the board.
        :param speed: rack cruise speed m/s (None = RACK_SPEED default).
        :return: True if the piece was grabbed, False if the press exceeded stroke.
        """
        start_z = 0.621
        #start_z = self.BOARD_TOP_Z + max(h for _, h in PIECE_DIMS.values()) + self.TRAVEL_MARGIN
        target_z = self.BOARD_TOP_Z + PIECE_DIMS[piece_type][1]
        press = start_z - target_z
        if press > self.RACK_RETRACTED + self.STROKE_EPS:
            print(f"[grasp] SKIPPED {chess.piece_name(piece_type)}: press "
                  f"{press * 1000:.1f}mm exceeds rack stroke {self.RACK_RETRACTED * 1000:.1f}mm")
            return False
        self.arm.move_rack(press, speed)                      # extend down onto crown
        if not self.arm.toggle_magnet():                      # magnet on / claim piece
            self.arm.move_rack(0.0, speed)                    # nothing grabbed: retract, fail loudly
            print(f"[grasp] FAILED {chess.piece_name(piece_type)}: magnet found nothing in range")
            return False
        self.arm.move_rack(0.0, speed)                          # retract, piece rides up
        print(f"[grasp] holding {chess.piece_name(piece_type)}")
        return True

    def move_to(self, location=None, orientation=None, mode="X", speed=None):
        """
        Summary: Navigate the end-effector to target X, Y, Z location with given Pitch, Yaw, Roll orientation.
                 Does so by calling arm_controller functions, and then verifies placement and orientation accuracy.

        :param location: position tuple X,Y,Z.
        :param orientation: rotation tuple Pitch,Yaw,Roll.
        :param mode: passed through to ArmController.solve_ik.
        :param speed: cruise rad/s (None = arm default).
        :return: True if the arm moved, False if the goal was rejected (move skipped).
        """
        x, y, z = location
        orient = np.asarray(self.DOWN if orientation is None else orientation, float)
        q, pos_err, rot_err = self.arm.solve_ik(x, y, z, orient, mode=mode,
                                                seed=self.arm.read_joints())
        if pos_err > self.SKIP_TOL or rot_err > self.ROT_LIMIT_DEG:
            print(f"[move_to] SKIPPED {location}: pos {pos_err * 1000:.0f}mm, rot {rot_err:.1f}deg")
            return False
        self.arm.move_joints(q, speed)
        self._hover_xy = (x, y)  # remember the XY: grasp/release descend straight from here
        return True

    def waypoint(self, start="home",speed=None):
        """
        Summary: Move the arm from resting spot, safe point around the edge of the board, and off
                 to the side.

        :param start: Start location: "home" or "board"
        :param speed: Motion speed (in m/s)
        :return: NA
        """

        if start == "home":
            self.arm.move_rack(0.0)
            self.arm.move_joints(self.UP_POSTURE, speed)  # Dont Clip Table
            self.arm.move_joints(self.WAYPOINT_ARM, speed)

        elif start == "board":
            self.arm.move_joints(self.WAYPOINT_ARM, speed)

    def turn_transition(self, speed=None):
        """
        Summary: Move the arm from over the board, to a safe point above the board, and off
                 to the side. This will be the consistent (X:0, Y:0, Z:0) starting point. From this point it can start
                 another turn.

        :param speed: Motion speed (in m/s)
        :return: NA
        """
        self.arm.move_joints((), speed)

    def home(self, speed=None):
        """
        Summary: Safely take the arm from its current position to a resting position by the robots side.

        :param speed: Motion speed (in m/s)
        :return: NA
        """
        self.arm.move_joints((0.0, 0.8, 0.0, 0.0, 0.0), speed)
        self.arm.move_joints(self.REST_ARM, speed)

# Testing ==============================================================================================================

def all_targets(sim):
    """
    Summary: (Test helper) Every place the pad can work: 64 squares + 2x16 graveyard
             slots, as (name, (x, y, z)) tuples ready to splat into move_to().

    :param sim: the GenesisSetup object (source of board geometry).
    :return: list of (label, point).
    """
    z_board = sim.BOARD_TOP_Z + max(h for _, h in sim.PIECE_DIMS.values()) + MovementSystem.TRAVEL_MARGIN  # travel height over pieces
    z_grave = z_board - 0.003  # strip top is 3mm lower
    pts = [(chess.square_name(sq), (*sim.square_center(sq), z_board)) for sq in chess.SQUARES]
    for side in ("left", "right"):
        pts += [(f"grave_{side}_{i}", (*sim.grave_center(side, i), z_grave))
                for i in range(sim.GRAVEYARD_SLOTS)]
    return pts

# Main =================================================================================================================

def main():

    # Setup ----------------------------------------------------------------------------------------------------------------

    # Imports & Files
    import os
    from simulation.genesis_setup import GenesisSetup
    from simulation.sim_arm_controller import SimArmController

    _DIR = os.path.dirname(os.path.abspath(__file__))
    urdf = os.path.join(_DIR, "..", "master_assembly_v6", "urdf", "master_assembly_v6.urdf")

    # Simulation
    sim = GenesisSetup(urdf)
    sim.setup()

    # Controller
    arm = SimArmController(sim.scene, sim.robot, urdf, "right", sim.piece_ent)

    # Movement System
    ms = MovementSystem(arm)

    # Test -----------------------------------------------------------------------------------------------------------------

    # Operate Robot
    ms.waypoint("home")

    # Move Pawn
    ms.move_to((0.0127, 0.1350, 0.621))
    ms.toggle_grasp(chess.PAWN)
    ms.move_to((0.0127, 0.1858, 0.621))
    ms.toggle_grasp(chess.PAWN)

    # Move Rook
    ms.move_to((0.0889, 0.1096, 0.621))
    ms.toggle_grasp(chess.ROOK)
    ms.move_to((0.0635, 0.1858, 0.621))
    ms.toggle_grasp(chess.ROOK)

    # Move Queen
    ms.move_to((0.0127, 0.1096, 0.621))
    ms.toggle_grasp(chess.KING)
    ms.move_to((-0.0127, 0.1604, 0.621))
    ms.toggle_grasp(chess.KING)

    # Move Knight
    ms.move_to((0.0635, 0.1096, 0.621))
    ms.toggle_grasp(chess.KNIGHT)
    ms.move_to((0.0381, 0.1604, 0.621))
    ms.toggle_grasp(chess.KNIGHT)

    ms.waypoint("board")
    ms.home()

if __name__ == "__main__":
    main()