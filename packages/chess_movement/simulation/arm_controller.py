# ============================================================
# arm_controller.py
# Purpose: Initialize robot configuration and motor controls from URDF.

from simulation.genesis_setup import GenesisSetup
import os
import numpy as np
import torch


class ArmController:

    # List of joint handles (names/variables) for the right arm.
    RIGHT_ARM_JOINTS = [
        "shoulderspin_right",
        "biceplift_right",
        "elbow_right",
        "wristspin_right",
        "handcurl_right",
    ]

    # List of joint handles (names/variables) for the left arm.
    LEFT_ARM_JOINTS = [
        "shoulderspin_left",
        "biceplift_left",
        "elbow_left",
        "wristspin_left",
        "handcurl_left",
    ]



    def __init__(self, scene, robot, arm_name = "left"):
        """
        Constructor: Initialize the ArmController class for one arm of an already spawned robot in simulation scene.

        :param scene: The Genesis Scene from 'genesis_setup'.py object.
        :param robot: The Genesis Robot from 'genesis_setup'.py object.
        :param arm_name: Which arm this controller represents ('right' or 'left').
        """

        # Initialize parameters
        self.scene = scene
        self.robot = robot
        self.arm_name = arm_name

        self.ee_link = None
        self.arm_dofs_idx_local = []
        self.magnet_dof_idx = None


    def initialize(self):
        """
        Summary: Walk though the URDF and define a variable for each motor joint.

        IMPORTANT: must match master_magnet_hand_v4.urdf

        :return: NA
        """

        finger_name = "finger_left" if self.arm_name == "left" else "finger_right"
        self.ee_link = self.robot.get_link(finger_name)
        if self.ee_link is None:
            raise ValueError(f"End effector link '{finger_name}' not found in URDF!")

        joint_names = self.LEFT_ARM_JOINTS if self.arm_name == "left" else self.RIGHT_ARM_JOINTS
        self.arm_dofs_idx_local = []
        for joint in self.robot.joints:
            if joint.name in joint_names and joint.n_dofs > 0:
                self.arm_dofs_idx_local.extend(joint.dofs_idx_local)

        magnet_joint_name = "fingerextend_left" if self.arm_name == "left" else "fingerextend_right"
        self.magnet_dof_idx = None
        for joint in self.robot.joints:
            if joint.name == magnet_joint_name and joint.n_dofs > 0:
                self.magnet_dof_idx = joint.dofs_idx_local[0]
                break


# (Testing)  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def main():

# Create Scene with 'genesis_setup'

    # Set URDF
    _SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    urdf_file = os.path.join(_SCRIPT_DIR, "master_magnet_hand_v5", "urdf", "master_magnet_hand_v5.urdf")

    # Run Simulation
    sim = GenesisSetup(urdf_file)
    sim.setup()


# Use 'arm_controller'.py class

    # Create Objects
    l_ctrl = ArmController(scene = sim.scene, robot = sim.robot)
    r_ctrl = ArmController(scene = sim.scene, robot = sim.robot, arm_name = "right")

    # Initialize Arm Controllers
    l_ctrl.initialize()
    r_ctrl.initialize()

if __name__ == "__main__":
    main()