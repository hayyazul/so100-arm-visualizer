"""
Unit tests for Phase 3 utility functions in visualize_arm.py.

Human-interface test checklist (manual):
  1. Run: python visualize_arm.py
  2. Open the printed URL in a browser.
  3. Verify: MeshCat iframe shows the arm, sidebar has X/Y/Z sliders.
  4. Drag sliders to a reachable position -- arm should animate smoothly,
     goal sphere green, blue sphere tracks end-effector.
  5. Drag sliders to an extreme position (all max) -- goal sphere turns red,
     status says "Unreachable", arm stays at last valid pose.
  6. Return sliders to a reachable position -- goal turns green, arm resumes.
  7. Close browser tab, Ctrl+C in terminal -- clean exit.
"""

import unittest
import numpy as np
from typing import List

from visualize_arm import (
    compute_workspace_radius,
    validate_ik_solution,
    interpolate_joints,
)
from ik_solver import SO100IKSolver


class TestComputeWorkspaceRadius(unittest.TestCase):
    def test_radius_in_expected_range(self) -> None:
        """Radius from the real URDF should be between 0.25 and 0.50 m."""
        radius: float = compute_workspace_radius("so100_kinematics.urdf")
        self.assertGreater(radius, 0.25)
        self.assertLess(radius, 0.50)

    def test_radius_is_positive(self) -> None:
        radius: float = compute_workspace_radius("so100_kinematics.urdf")
        self.assertGreater(radius, 0.0)


class TestValidateIKSolution(unittest.TestCase):
    def setUp(self) -> None:
        self.solver: SO100IKSolver = SO100IKSolver(urdf_path="so100_kinematics.urdf")

    def test_reachable_position(self) -> None:
        """A known reachable position should validate as True."""
        target: List[float] = [0.0, -0.193069, 0.220042]
        ik_result: List[float] = self.solver.calculate_ik(target)
        self.assertTrue(validate_ik_solution(self.solver, ik_result, target))

    def test_unreachable_position(self) -> None:
        """A position far outside the workspace should validate as False."""
        target: List[float] = [1.0, 1.0, 1.0]
        ik_result: List[float] = self.solver.calculate_ik(target)
        self.assertFalse(validate_ik_solution(self.solver, ik_result, target))


class TestInterpolateJoints(unittest.TestCase):
    def test_alpha_zero_returns_current(self) -> None:
        current: np.ndarray = np.array([1.0, 2.0, 3.0])
        target: np.ndarray = np.array([4.0, 5.0, 6.0])
        result: np.ndarray = interpolate_joints(current, target, 0.0)
        np.testing.assert_array_equal(result, current)

    def test_alpha_one_returns_target(self) -> None:
        current: np.ndarray = np.array([1.0, 2.0, 3.0])
        target: np.ndarray = np.array([4.0, 5.0, 6.0])
        result: np.ndarray = interpolate_joints(current, target, 1.0)
        np.testing.assert_array_equal(result, target)

    def test_alpha_half_returns_midpoint(self) -> None:
        current: np.ndarray = np.array([0.0, 0.0])
        target: np.ndarray = np.array([2.0, 4.0])
        result: np.ndarray = interpolate_joints(current, target, 0.5)
        np.testing.assert_array_almost_equal(result, [1.0, 2.0])

    def test_alpha_clamped_above_one(self) -> None:
        """Alpha > 1 should be clamped to 1 (returns target)."""
        current: np.ndarray = np.array([0.0])
        target: np.ndarray = np.array([10.0])
        result: np.ndarray = interpolate_joints(current, target, 5.0)
        np.testing.assert_array_equal(result, target)

    def test_alpha_clamped_below_zero(self) -> None:
        """Alpha < 0 should be clamped to 0 (returns current)."""
        current: np.ndarray = np.array([0.0])
        target: np.ndarray = np.array([10.0])
        result: np.ndarray = interpolate_joints(current, target, -1.0)
        np.testing.assert_array_equal(result, current)


class TestIKWarmStart(unittest.TestCase):
    """Verify the warm-start parameter in calculate_ik works correctly."""

    def setUp(self) -> None:
        self.solver: SO100IKSolver = SO100IKSolver(urdf_path="so100_kinematics.urdf")

    def test_warm_start_does_not_crash(self) -> None:
        """Providing an initial guess should not raise."""
        target: List[float] = [0.0, -0.193069, 0.220042]
        initial: List[float] = [0.0] * len(self.solver.kinematics.joint_names)
        result: List[float] = self.solver.calculate_ik(target, initial_joint_pos_rad=initial)
        self.assertIsInstance(result, list)
        self.assertEqual(len(result), len(self.solver.kinematics.joint_names))

    def test_warm_start_from_known_solution(self) -> None:
        """Warm-starting from a known-good solution should still converge."""
        target: List[float] = [0.0, -0.193069, 0.220042]
        # First solve from zeros to get a known-good starting point
        good_solution: List[float] = self.solver.calculate_ik(target)
        # Now warm-start from that solution for the same target
        result: List[float] = self.solver.calculate_ik(
            target, initial_joint_pos_rad=good_solution
        )
        self.assertTrue(validate_ik_solution(self.solver, result, target))


if __name__ == "__main__":
    unittest.main()
