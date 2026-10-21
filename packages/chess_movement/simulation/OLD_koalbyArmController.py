import genesis as gs
import numpy as np
import torch
import os
from pathlib import Path

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


class RobotArmController:
    """Base class for controlling robot arm with IK and trajectory planning."""
    
    def __init__(
        self,
        urdf_file: str = None,
        ee_name: str = "hand_right",
        robot_pos: tuple = (0.0, 0.0, 0.735),
        show_viewer: bool = True,
        cache_file: str = "test_path.pt",
    ):
        """Initialize the robot arm controller.
        
        Args:
            urdf_file: Path to the URDF file
            ee_name: End effector link name ("hand_left" or "hand_right")
            robot_pos: Initial position of the robot
            show_viewer: Whether to show the Genesis viewer
            cache_file: Path to cache file for trajectories
        """
        if urdf_file is None:
            urdf_file = os.path.join(_SCRIPT_DIR, "Balancing_Chess_URDF", "urdf", "Balancing_Chess_URDF.urdf")
        self.urdf_file = urdf_file
        self.ee_name = ee_name
        self.robot_pos = robot_pos
        self.show_viewer = show_viewer
        self.cache_file = cache_file
        
        # Joint definitions
        self.left_arm_joints = [
            "shoulderspin_left",
            "biceplift_left",
            "elbow_left",
            "wristspin_left",
            "handcurl_left"
        ]
        
        self.right_arm_joints = [
            "shoulderspin_right",
            "biceplift_right",
            "elbow_right",
            "wristspin_right",
            "handcurl_right"
        ]
        
        # Initialize Genesis (logging_level="warning" silences the per-frame
        # FPS spam so real warnings and errors stay visible in the terminal)
        if torch.cuda.is_available():
            gs.init(backend=gs.gpu, logging_level="warning")
        else:
            gs.init(backend=gs.cpu, logging_level="warning")
        self.scene = None
        self.robot = None
        self.ee_link = None
        self.arm_dofs_idx_local = []
        self.offset = 0.0
        self.cached_paths = {}
        
        # Setup
        self._setup_scene()
        self._setup_arm_joints()
        self._load_cache()
    
    def _setup_scene(self):
        """Initialize the Genesis scene and add entities."""
        self.scene = gs.Scene(
            show_viewer=self.show_viewer,
            viewer_options=gs.options.ViewerOptions(
                res=(1280, 960),
                camera_pos=(3.5, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
                max_FPS=60,
            ),
            rigid_options = gs.options.RigidOptions(
                enable_neutral_collision=True
            )
        )
        
        # Add ground plane
        self.scene.add_entity(gs.morphs.Plane())
        
        # Add robot
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file=self.urdf_file,
                pos=self.robot_pos,
                quat=(0, 0, 0, 1),
                fixed=True,
            )
        )
        
        self.scene.build()
    
    def _setup_arm_joints(self):
        """Configure arm joints and end effector."""
        # Get end effector link
        _finger_name = "finger_left" if self.ee_name == "hand_left" else "finger_right"
        self.ee_link = self.robot.get_link(_finger_name)
        
        if self.ee_link is None:
            raise ValueError(f"End effector '{self.ee_name}' not found!")
        
        # Get joint indices for the selected arm
        if self.ee_name == "hand_left":
            self.offset = 0.145
            joint_names = self.left_arm_joints
        elif self.ee_name == "hand_right":
            self.offset = -0.145
            joint_names = self.right_arm_joints
        elif self.ee_name == "wrist_left":
            self.offset = 0.145
            joint_names = self.left_arm_joints
        elif self.ee_name == "wrist_right":
            self.offset = -0.145
            joint_names = self.right_arm_joints
        else:
            raise ValueError("End effector must be 'hand_left' or 'hand_right' or 'wrist_left' or 'wrist_right'")
        
        # Extract DOF indices for arm joints
        self.arm_dofs_idx_local = []
        for joint in self.robot.joints:
            if joint.name in joint_names:
                if joint.n_dofs > 0:
                    self.arm_dofs_idx_local.extend(joint.dofs_idx_local)
        
        print(f"\nEnd Effector: {self.ee_name}")
        print(f"Arm DOFs local indices: {self.arm_dofs_idx_local}")
        
        # Get gripper joint DOF indices
        # NOTE: this URDF has no separate gripper_left/gripper_right joint --
        # the grasp mechanism ("Chess Hand Assy") is now integrated directly
        # into hand_left/hand_right. There is currently no known joint name
        # to look up here; this will just find nothing until the actual
        # grasp-actuation joint (if any) for the new hand assembly is known.
        self.gripper_dofs_idx_local = []
        gripper_joint_name = "fingerpinch_left" if self.ee_name == "hand_left" else "fingerpinch_right"
        for joint in self.robot.joints:
            if joint.name == gripper_joint_name:
                if joint.n_dofs > 0:
                    self.gripper_dofs_idx_local.extend(joint.dofs_idx_local)
        
        print(f"Gripper DOFs local indices: {self.gripper_dofs_idx_local}")
    
    def _load_cache(self):
        """Load cached paths from file if it exists."""
        cache_path = Path(self.cache_file)
        if cache_path.exists():
            self.cached_paths = torch.load(self.cache_file, map_location=torch.device('cpu'))
            print(f"Loaded {len(self.cached_paths)} cached paths")
        else:
            self.cached_paths = {}
    
    def _save_cache(self):
        """Save cached paths to file."""
        torch.save(self.cached_paths, self.cache_file)
        print(f"Saved {len(self.cached_paths)} paths to cache")
    
    def select_end_effector_by_position(self, target_pos: np.ndarray):
        """Automatically select end effector based on target x-coordinate.
        
        Args:
            target_pos: Target position [x, y, z]
        """
        target_ee = "hand_right" if target_pos[0] >= 0 else "hand_left"
        
        if target_ee != self.ee_name:
            print(f"Switching from {self.ee_name} to {target_ee}")
            self.ee_name = target_ee
            self._setup_arm_joints()
        else:
            print(f"Using {self.ee_name}")
    
    def move_to_ik_target(self, target_pos: np.ndarray, target_quat: np.ndarray = None) -> torch.Tensor:
        """Solve inverse kinematics for a target position and optional orientation.
        
        Args:
            target_pos: Target position [x, y, z]
            target_quat: Target quaternion [x, y, z, w] (default: None)
            
        Returns:
            IK solution (joint angles)
        """
        
        # Convert target position to tensor
        target_pos_tensor = torch.tensor(
            target_pos,
            dtype=torch.float32
        )
        
        # Solve IK
        if target_quat is not None:
            target_quat = torch.tensor(target_quat, dtype=torch.float32)
            ik_result = self.robot.inverse_kinematics(
                link=self.ee_link,
                pos=target_pos_tensor,
                quat=target_quat,
                dofs_idx_local=self.arm_dofs_idx_local
            )
        else:
            ik_result = self.robot.inverse_kinematics(
                link=self.ee_link,
                pos=target_pos_tensor,
                dofs_idx_local=self.arm_dofs_idx_local
            )
        
        print(f"IK result: {ik_result}")
        return ik_result
    
    def plan_trajectory(self, target_qpos: torch.Tensor, num_waypoints: int = 200) -> list:
        """Plan a trajectory to target joint configuration.
        
        Args:
            target_qpos: Target joint configuration
            num_waypoints: Number of waypoints in trajectory
            
        Returns:
            Planned trajectory
        """
        path = self.robot.plan_path(
            qpos_goal=target_qpos,
            num_waypoints=num_waypoints,
        )
        
        print(f"Planned trajectory with {len(path)} waypoints")
        return path
    
    def execute_trajectory(self, path: list, pause_steps: int = 100):
        """Execute a trajectory.
        
        Args:
            path: List of waypoints to execute
            pause_steps: Number of steps to pause after trajectory completes
        """
        print(f"Executing trajectory with {len(path)} waypoints...")
        
        for waypoint in path:
            self.robot.control_dofs_position(waypoint)
            self.scene.step()
        
        # Pause at end position
        for _ in range(pause_steps):
            self.scene.step()
        
        print("Trajectory complete")
    
    def control_gripper(self, force: float, duration_steps: int = 50):
        """Control the gripper by applying force to open or close it.
        
        Args:
            force: Force to apply to the gripper joint (positive = open, negative = close)
            duration_steps: Number of simulation steps to apply the force
        """
        if not self.gripper_dofs_idx_local:
            print("Warning: No gripper DOFs found")
            return
        
        print(f"Applying gripper force: {force} for {duration_steps} steps")
        
        for _ in range(duration_steps):
            self.robot.control_dofs_force(
                np.array([force]),
                self.gripper_dofs_idx_local
            )
            self.scene.step()
        
        print("Gripper control complete")
    
    def open_gripper(self, force: float = 0.5, duration_steps: int = 50):
        """Open the gripper by applying positive force.
        
        Args:
            force: Force magnitude to apply (default: 0.5)
            duration_steps: Number of simulation steps to apply the force
        """
        self.control_gripper(force, duration_steps)
    
    def close_gripper(self, force: float = -0.5, duration_steps: int = 50):
        """Close the gripper by applying negative force.
        
        Args:
            force: Force magnitude to apply (negative for closing, default: -0.5)
            duration_steps: Number of simulation steps to apply the force
        """
        self.control_gripper(force, duration_steps)
    
    def align_gripper_to_quaternion(self, target_quat: np.ndarray = None, num_waypoints: int = 50):
        """Align the gripper to a target quaternion by rotating the handspin motor.
        
        This function rotates the handspin joint to achieve the target gripper orientation.
        
        Args:
            target_quat: Target quaternion [x, y, z, w] (default: [0, 0, 0, 1] = identity rotation)
            num_waypoints: Number of interpolation steps for smooth rotation (default: 50)
        
        Returns:
            bool: True if successful, False otherwise
        """
        # Default to identity quaternion if not specified
        if target_quat is None:
            target_quat = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Get handspin joint index
        handspin_joint_name = "wristspin_left" if self.ee_name == "hand_left" else "wristspin_right"
        
        handspin_dof_idx = None
        for joint in self.robot.joints:
            if joint.name == handspin_joint_name and joint.n_dofs > 0:
                handspin_dof_idx = joint.dofs_idx_local[0]
                break
        
        if handspin_dof_idx is None:
            print(f"Warning: Could not find {handspin_joint_name} joint")
            return False
        
        # Get current joint configuration
        current_qpos = self.robot.get_qpos().clone()
        current_handspin_angle = current_qpos[handspin_dof_idx].item()
        
        print(f"\nAligning gripper to target quaternion")
        print(f"Target quaternion: [{target_quat[0]:.4f}, {target_quat[1]:.4f}, {target_quat[2]:.4f}, {target_quat[3]:.4f}]")
        
        # Search for the handspin angle that achieves the target quaternion
        # Sample a range of angles to find the best match
        angle_search_range = np.linspace(-np.pi, np.pi, 36)  # 10-degree increments
        best_angle = current_handspin_angle
        best_error = float('inf')
        
        for test_angle in angle_search_range:
            test_qpos = current_qpos.clone()
            test_qpos[handspin_dof_idx] = test_angle
            self.robot.control_dofs_position(test_qpos)
            
            # Step a few times to let physics settle
            for _ in range(2):
                self.scene.step()
            
            # Get current gripper quaternion
            quat = self.ee_link.quat
            current_quat = quat.numpy() if hasattr(quat, 'numpy') else np.array(quat)
            
            # Calculate quaternion error (dot product, absolute value for angle difference)
            quat_dot = abs(np.dot(current_quat, target_quat))
            quat_error = np.arccos(np.clip(quat_dot, -1.0, 1.0))
            
            if quat_error < best_error:
                best_error = quat_error
                best_angle = test_angle
        
        print(f"Found best handspin angle: {best_angle:.4f} rad ({np.degrees(best_angle):.2f}°)")
        print(f"Quaternion error: {best_error:.6f} rad ({np.degrees(best_error):.4f}°)")
        
        # Interpolate smoothly from current angle to best angle
        angles = np.linspace(current_handspin_angle, best_angle, num_waypoints)
        
        # Execute trajectory
        for angle in angles:
            new_qpos = current_qpos.clone()
            new_qpos[handspin_dof_idx] = angle
            self.robot.control_dofs_position(new_qpos)
            self.scene.step()
        
        # Verify final orientation
        final_orientation = self.get_gripper_orientation()
        final_quat = final_orientation['quaternion']
        final_error = np.arccos(np.clip(abs(np.dot(final_quat, target_quat)), -1.0, 1.0))
        
        print(f"Gripper alignment complete")
        print(f"Final quaternion: [{final_quat[0]:.4f}, {final_quat[1]:.4f}, {final_quat[2]:.4f}, {final_quat[3]:.4f}]")
        print(f"Final quaternion error: {final_error:.6f} rad ({np.degrees(final_error):.4f}°)")
        
        return final_error < 0.1  # Success if error < ~5.7 degrees
    
    def align_gripper_z_axis(self, target_angle: float = 0.0, num_waypoints: int = 50):
        """Align the gripper's z-axis with the world's z-axis by rotating the handspin motor.
        
        This function rotates the handspin joint to align the gripper's orientation such that
        its z-axis points straight up/down (aligned with world z-axis).
        
        Args:
            target_angle: Target angle for the handspin joint in radians (default: 0.0)
            num_waypoints: Number of interpolation steps for smooth rotation (default: 50)
        
        Returns:
            bool: True if successful, False otherwise
        """
        # Get handspin joint index based on current end effector
        handspin_joint_name = "wristspin_left" if self.ee_name == "hand_left" else "wristspin_right"
        
        # Find the handspin joint DOF index
        handspin_dof_idx = None
        for joint in self.robot.joints:
            if joint.name == handspin_joint_name and joint.n_dofs > 0:
                handspin_dof_idx = joint.dofs_idx_local[0]
                break
        
        if handspin_dof_idx is None:
            print(f"Warning: Could not find {handspin_joint_name} joint")
            return False
        
        # Get current joint configuration
        current_qpos = self.robot.get_qpos().clone()
        
        # Create interpolated path from current handspin angle to target angle
        current_handspin_angle = current_qpos[handspin_dof_idx].item()
        
        print(f"\nAligning gripper z-axis with world z-axis")
        print(f"Joint: {handspin_joint_name}")
        print(f"Current angle: {current_handspin_angle:.4f} rad ({np.degrees(current_handspin_angle):.2f}°)")
        print(f"Target angle: {target_angle:.4f} rad ({np.degrees(target_angle):.2f}°)")
        
        # Interpolate between current and target angle
        angles = np.linspace(current_handspin_angle, target_angle, num_waypoints)
        
        for angle in angles:
            # Create a copy of current qpos and update handspin joint
            new_qpos = current_qpos.clone()
            new_qpos[handspin_dof_idx] = angle
            
            # Control the robot to this configuration
            self.robot.control_dofs_position(new_qpos)
            self.scene.step()
        
        # Verify final angle
        final_qpos = self.robot.get_qpos().clone()
        final_angle = final_qpos[handspin_dof_idx].item()
        angle_error = abs(final_angle - target_angle)
        
        print(f"Gripper z-axis alignment complete")
        print(f"Target angle: {target_angle:.4f} rad ({np.degrees(target_angle):.2f}°)")
        print(f"Final angle: {final_angle:.4f} rad ({np.degrees(final_angle):.2f}°)")
        print(f"Angle error: {angle_error:.6f} rad ({np.degrees(angle_error):.4f}°)")
        
        return True
    
    def get_gripper_orientation(self) -> dict:
        """Get the current orientation of the gripper.
        
        Returns:
            Dictionary containing:
                - 'quaternion': [x, y, z, w] quaternion
                - 'euler_angles': [roll, pitch, yaw] in radians
                - 'z_axis': [x, y, z] normalized z-axis of gripper in world frame
                - 'handspin_angle': Current handspin joint angle in radians
        """
        # Get gripper pose
        gripper_quat = self.ee_link.quat.clone()
        
        # Convert quaternion to rotation matrix to extract axes
        def quat_to_rotation_matrix(q):
            """Convert quaternion [x, y, z, w] to rotation matrix."""
            x, y, z, w = q[0], q[1], q[2], q[3]
            
            R = np.array([
                [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
                [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
                [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
            ])
            return R
        
        def rotation_matrix_to_euler(R):
            """Convert rotation matrix to Euler angles [roll, pitch, yaw]."""
            sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
            singular = sy < 1e-6
            
            if not singular:
                x = np.arctan2(R[2, 1], R[2, 2])
                y = np.arctan2(-R[2, 0], sy)
                z = np.arctan2(R[1, 0], R[0, 0])
            else:
                x = np.arctan2(-R[1, 2], R[1, 1])
                y = np.arctan2(-R[2, 0], sy)
                z = 0
            
            return np.array([x, y, z])
        
        # Get handspin angle
        handspin_joint_name = "wristspin_left" if self.ee_name == "hand_left" else "wristspin_right"
        handspin_angle = 0.0
        for joint in self.robot.joints:
            if joint.name == handspin_joint_name and joint.n_dofs > 0:
                dof_idx = joint.dofs_idx_local[0]
                handspin_angle = self.robot.qpos[dof_idx].item()
                break
        
        # Convert quaternion to rotation matrix
        R = quat_to_rotation_matrix(gripper_quat.numpy())
        
        # Extract z-axis (third column of rotation matrix)
        z_axis = R[:, 2]
        
        # Get Euler angles
        euler_angles = rotation_matrix_to_euler(R)
        
        return {
            'quaternion': gripper_quat.numpy(),
            'euler_angles': euler_angles,
            'z_axis': z_axis,
            'handspin_angle': handspin_angle
        }
    
    def run_interactive_loop(self):
        """Run the main interactive control loop."""
        print("\n" + "="*50)
        print("Robot Arm Control - Interactive Loop")
        print("="*50)
        print("Commands:")
        print("  'ik'    - Move arm to position using inverse kinematics")
        print("  'r'     - Execute a cached trajectory")
        print("  'o'     - Open gripper")
        print("  'cl'    - Close gripper")
        print("  'align' - Align gripper z-axis with world z-axis")
        print("  'info'  - Get gripper orientation info")
        print("  'c'     - Clear all cached paths")
        print("  'q'     - Quit")
        print("="*50 + "\n")
        
        while True:
            cmd = input("\nEnter command (ik/r/o/cl/align/info/c/q): ").strip().lower()
            
            if cmd == "q":
                self._save_cache()
                print("Exiting...")
                break
            
            elif cmd == "c":
                self.cached_paths = {}
                print("Cached paths cleared.")
            
            elif cmd == "o":
                self.open_gripper()
            
            elif cmd == "cl":
                self.close_gripper()
            
            elif cmd == "align":
                self._handle_align_command()
            
            elif cmd == "info":
                self._handle_info_command()
            
            elif cmd == "ik":
                self._handle_ik_command()
            
            elif cmd == "r":
                self._handle_replay_command()
            
            else:
                print("Unknown command. Try 'ik', 'r', 'o', 'cl', 'align', 'info', 'c', or 'q'")
    
    def _handle_ik_command(self):
        """Handle interactive IK command."""
        try:
            # Ask user which hand to use
            hand_choice = input("Which hand? (L/R): ").strip().upper()
            if hand_choice == "L":
                self.ee_name = "hand_left"
            elif hand_choice == "R":
                self.ee_name = "hand_right"
            else:
                print("Invalid choice. Using right hand.")
                self.ee_name = "hand_right"
            
            self._setup_arm_joints()
            
            x = float(input("Target X: "))
            y = float(input("Target Y: "))
            z = float(input("Target Z: "))
            
            target_pos = np.array([x, y, z])
            
            # Solve IK
            ik_result = self.move_to_ik_target(target_pos)
            
            # Plan and execute trajectory
            path = self.plan_trajectory(ik_result)
            
            # Ask to save
            save_answer = input("Save this path? (y/n): ").strip().lower()
            if save_answer == 'y':
                key = str(input("Path key name: "))
                self.cached_paths[key] = path
                print(f"Path saved as '{key}'")
            
            # Execute
            self.execute_trajectory(path)
            
        except ValueError:
            print("Invalid input. Please enter valid numbers.")
        except Exception as e:
            print(f"Error during IK: {e}")
    
    def _handle_replay_command(self):
        """Handle interactive replay command."""
        if not self.cached_paths:
            print("No cached paths available.")
            return
        
        print("\nAvailable paths:")
        for key in self.cached_paths.keys():
            print(f"  - {key}")
        
        key = str(input("Path key to replay: ")).strip()
        
        if key not in self.cached_paths:
            print(f"Path '{key}' not found.")
            return
        
        path = self.cached_paths[key]
        self.execute_trajectory(path)
    
    def _handle_align_command(self):
        """Handle interactive gripper alignment command."""
        try:
            quat_input = input("Target quaternion [x, y, z, w] (default [0, 0, 0, 1]): ").strip()
            
            if quat_input:
                quat_vals = [float(x) for x in quat_input.replace('[', '').replace(']', '').split(',')]
                if len(quat_vals) != 4:
                    raise ValueError("Quaternion must have 4 components")
                target_quat = np.array(quat_vals)
            else:
                target_quat = None  # Use default identity quaternion
            
            num_steps = int(input("Number of interpolation steps (default 50): ") or "50")
            
            self.align_gripper_to_quaternion(target_quat=target_quat, num_waypoints=num_steps)
            
        except ValueError:
            print("Invalid input. Please enter valid numbers (e.g., '0, 0, 0, 1').")
        except Exception as e:
            print(f"Error during alignment: {e}")
    
    def _handle_info_command(self):
        """Handle interactive gripper info command."""
        try:
            info = self.get_gripper_orientation()
            
            print("\n" + "="*50)
            print("Gripper Orientation Information")
            print("="*50)
            print(f"Quaternion (x, y, z, w): {info['quaternion']}")
            euler_deg = np.degrees(info['euler_angles'])
            print(f"Euler Angles (roll, pitch, yaw):")
            print(f"  Roll:  {euler_deg[0]:.2f}°")
            print(f"  Pitch: {euler_deg[1]:.2f}°")
            print(f"  Yaw:   {euler_deg[2]:.2f}°")
            print(f"Z-axis in world frame: {info['z_axis']}")
            print(f"Handspin angle: {info['handspin_angle']:.4f} rad ({np.degrees(info['handspin_angle']):.2f}°)")
            print("="*50 + "\n")
            
        except Exception as e:
            print(f"Error getting gripper info: {e}")


# -------------------------
# Main Execution
# -------------------------
if __name__ == "__main__":
    controller = RobotArmController(
        urdf_file="Balancing_Chess_URDF//urdf//Balancing_Chess_URDF.urdf",
        ee_name="hand_right",
        robot_pos=(0.0, 0.0, 0.735),
        show_viewer=True,
        cache_file="test_path.pt",
    )
    
    controller.run_interactive_loop()
