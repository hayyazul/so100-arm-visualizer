import numpy as np
import torch
from typing import List, Optional, Union
from lerobot.model.kinematics import RobotKinematics
import os

class SO100IKSolver:
    """
    Inverse Kinematics solver for the SO-100 robot arm using Hugging Face's lerobot library.
    """
    def __init__(self, urdf_path: str) -> None:
        """
        Initializes the SO100IKSolver.
        
        Args:
            urdf_path (str): The absolute or relative path to the so100.urdf file.
        """
        if not os.path.exists(urdf_path):
            raise FileNotFoundError(f"URDF file not found at: {urdf_path}")
            
        # Initialize the kinematics model for SO-100 built into lerobot
        # The SO100 URDF ends with "gripper_frame_link" or similar end-effector link.
        # We need to find the exact target frame in the URDF. Looking at the URDF, 
        # the end effector is likely 'jaw' or 'gripper'. We'll use 'jaw'.
        print(f"DEBUG: INITIALIZING ROBOT KINEMATICS WITH URDF PATH: {urdf_path}")
        self.kinematics: RobotKinematics = RobotKinematics(
            urdf_path=urdf_path,
            target_frame_name="jaw"
        )

    def calculate_ik(
        self,
        target_position: List[float],
        initial_joint_pos_rad: Optional[List[float]] = None
    ) -> List[float]:
        """
        Calculates the inverse kinematics for a given target end-effector position.

        Args:
            target_position (List[float]): The desired [x, y, z] target position in meters in the base frame.
            initial_joint_pos_rad (Optional[List[float]]): Starting joint angles in radians for the solver.
                If None, defaults to all zeros. Providing a warm-start (e.g. current arm config) improves
                convergence for interactive use.

        Returns:
            List[float]: A list of joint angles that achieves the target position.
        """
        num_joints: int = len(self.kinematics.joint_names)
        if initial_joint_pos_rad is not None:
            # lerobot expects degrees internally
            current_joint_pos: np.ndarray = np.rad2deg(np.array(initial_joint_pos_rad, dtype=np.float64))
        else:
            current_joint_pos: np.ndarray = np.zeros(num_joints, dtype=np.float64)
        
        # The lerobot inverse_kinematics expects a 4x4 homogenous matrix for desired pose
        desired_ee_pose: np.ndarray = np.eye(4, dtype=np.float64)
        desired_ee_pose[0, 3] = target_position[0]
        desired_ee_pose[1, 3] = target_position[1]
        desired_ee_pose[2, 3] = target_position[2]
        
        # Call the solver (we set orientation_weight to 0 for MVP Phase 1 since we only care about position)
        ik_solution: np.ndarray = self.kinematics.inverse_kinematics(
            current_joint_pos=current_joint_pos,
            desired_ee_pose=desired_ee_pose,
            position_weight=1.0,
            orientation_weight=0.0
        )
        
        # Return as a simple python list (converted back to radians if needed, lerobot ik returns degrees!)
        # AGENTS.md prefers radians usually for ROS/robotics unless specified. Since lerobot returns degrees,
        # we will return them as radians to match standard URDF limits.
        joint_angles_rad: np.ndarray = np.deg2rad(ik_solution)
        
        return joint_angles_rad.tolist()

if __name__ == "__main__":
    print("Testing SO100IKSolver Initialization...")
    
    urdf_file: str = "models/so100_kinematics.urdf"
    
    try:
        solver: SO100IKSolver = SO100IKSolver(urdf_path=urdf_file)
        print("Successfully initialized SO100IKSolver")
        
        target_pos: List[float] = [0.15, 0.0, 0.15] 
        print(f"Calculating IK for position: {target_pos}")
        
        joint_angles: List[float] = solver.calculate_ik(target_pos)
        print(f"Calculated Joint Angles (radians): {joint_angles}")
        
    except Exception as e:
        print(f"Error executing IK script: {e}")
