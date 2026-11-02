from simulation.arm_controller import ArmController
import os
import numpy as np

class HardwareArmController(ArmController):
    """
    hardware_arm_controller.py

    Purpose: Initialize robot configuration and joint controls for URCHIN Framework motor drivers.
    """

# Constructor ==========================================================================================================

    def __init__(self, crab, urdf_file, arm_name="left"):
        """
        Constructor: Initialize the ArmController class for one arm of an already spawned robot in simulation scene.

        :param crab: Motor Drivers
        :param urdf_file: URDF file
        :param arm_name: Which arm this controller represents ('right' or 'left').
        """
        super().__init__(urdf_file, arm_name)

        # Initialize parameters
        self.crab = crab
        self.arm_name = arm_name
        self.check_bus()

# Functions ============================================================================================================

    def read_joints(self):
        """
        Summary:

        :return:
        """
        pass

    def command_joints(self, q):
        """
        Summary:

        :param q:
        :return:
        """
        pass

    def step(self):
        """
        Summary:

        :return:
        """
        pass

    def move_rack(self, v, v_max=None):
        """
        Summary: Drive the rack & pinion to absolute extension v and hold it there.

        :param v: target rack extension (m; 0 = pad lowest, RACK_RETRACTED = pad highest).
        :param v_max: cruise speed m/s (None = RACK_SPEED).
        :return: NA
        """
        pass

    def pad_world_z(self):
        """
        Summary: World Z of the pad face (release-seat invariant).

        :return: pad face world Z (m).
        """
        pass

    def read_rack(self):
        """
        Summary: Read the rack's current extension.

        :return: current rack extension (m).
        """
        pass

    def toggle_magnet(self):
        """
        Summary:
        :return:
        """
        pass

    def carry_gap(self):
        """
        Summary:

        :return:
        """
        pass

# Main =================================================================================================================

def main():

    # Create Objects
    l_ctrl = HardwareArmController()
    r_ctrl = HardwareArmController(arm_name="right")

    # PRIMITIVE SAFETY STUFF HERE

if __name__ == "__main__":
    main()