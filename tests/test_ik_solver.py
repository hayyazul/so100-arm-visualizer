import unittest
import numpy as np
from typing import List, Any
from ik_solver import SO100IKSolver

class TestSO100IKSolver(unittest.TestCase):
    def setUp(self) -> None:
        """Sets up the solver instance."""
        urdf_file: str = "models/so100_kinematics.urdf"
        try:
            self.solver: SO100IKSolver = SO100IKSolver(urdf_path=urdf_file)
        except Exception as e:
            self.fail(f"Failed to initialize SO100IKSolver: {e}")

    def test_ik_calculation_runs(self) -> None:
        """
        Tests if the IK calculation can be executed without throwing an error
        for a valid typical workspace position.
        """
        # A position generally expected to be in the workspace of a small desktop arm
        target_pos: List[float] = [0.0, -0.193069, 0.220042]
        try:
            joint_angles: List[float] = self.solver.calculate_ik(target_pos)
            self.assertIsInstance(joint_angles, list)
            # The SO100 has 6 joints (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)
            # Depending on how lerobot models it, the IK solver might return 5 or 6 depending on whether the gripper is included in the IK chain.
            self.assertGreaterEqual(len(joint_angles), 5, "Should return at least 5 joint angles for the arm")
            
        except Exception as e:
            self.fail(f"IK Calculation failed: {e}")

    def test_ik_forward_kinematics_consistency(self) -> None:
        """
        Tests if the result of the IK, when fed back into Forward Kinematics,
        yields the original requested position.
        """
        target_pos: List[float] = [0.0, -0.193069, 0.220042]
        
        # 1. Get IK joint angles in radians
        joint_angles: List[float] = self.solver.calculate_ik(target_pos)
        
        # 2. Convert to degrees as lerobot FK function expects degrees according to kinematics.py
        joint_angles_deg: np.ndarray = np.rad2deg(joint_angles)
        
        # 3. Compute FK
        fk_result: np.ndarray = self.solver.kinematics.forward_kinematics(joint_pos_deg=joint_angles_deg)
        
        # 'forward_kinematics' returns a 4x4 NumPy transformation matrix. 
        # The translation vector is the first 3 elements of the 4th column.
        computed_pos: List[float] = fk_result[:3, 3].tolist()
            
        np.testing.assert_allclose(computed_pos, target_pos, atol=1e-2, err_msg="IK result did not reach target position in FK")

if __name__ == '__main__':
    unittest.main()
