# ============================================================
# movement_system.py
# Purpose: Establish basic movement system for grasping and placing the end-effector, and moving it in 3d Space.

class MovementSystem:

    def __init__(self):
        """
        Constructor: Initialize the class variables.
        """
        pass

    def target(self, X, Y, Z):
        """
        Summary: (Helper) Use 'Inverse-Kinematics' to move the arm to the target location.

        :param X: X coordinate of the target location.
        :param Y: Y coordinate of the target location.
        :param Z: Z coordinate of the target location.
        :return:
        """
        pass

    def home(self):
        """
        Summary: Safely take the arm from its current position to a resting position by the robots side.

        :return:
        """
        pass

    def waypoint(self):
        """
        Summary: Move the arm from resting spot, or last grasp/release location to a safe point above the board, and off
                 to the side. This will be the consistent (X:0, Y:0, Z:0) starting point. From this point it can start
                 another turn.

        :return:
        """
        pass

    def grasp(self):
        """
        Summary: (Starting from safe operation height, and empty gripper) Open gripper and move hand vertically downward
                  in 3D space (to board surface height), close gripper, and lift back to original Z height without
                  ever-changing X-Y coordinates.

        :return:
        """
        pass

    def release(self):
        """
        Summary: (Starting from safe operation height, and a piece in hand with gripper already closed) Move hand
                  vertically downward in 3D space, open gripper, lift back to original Z height, and open gripper.
                  (without ever-changing X-Y coordinates).

        :return:
        """
        pass


# (Testing)  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def main():
    a_movement_system = MovementSystem()
    print("test 0")

if __name__ == "__main__":
    main()