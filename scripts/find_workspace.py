import numpy as np
from lerobot.model.kinematics import RobotKinematics

urdf_file = "so100_kinematics.urdf"
kinematics = RobotKinematics(urdf_path=urdf_file, target_frame_name="jaw")

# All zero joint angles
zeros = np.zeros(len(kinematics.joint_names), dtype=np.float64)

# Get the forward kinematics position
fk_matrix = kinematics.forward_kinematics(zeros)
pos_at_zero = fk_matrix[:3, 3]

print(f"Position at all zeros: {pos_at_zero}")

# Try another joint config inside limits
valid_config = [0.0, 30.0, -30.0, -10.0, 0.0][:len(kinematics.joint_names)]
if len(valid_config) < len(kinematics.joint_names):
    valid_config.extend([0]*(len(kinematics.joint_names)-len(valid_config)))

fk_matrix2 = kinematics.forward_kinematics(np.array(valid_config, dtype=np.float64))
pos_at_valid = fk_matrix2[:3, 3]
print(f"Position at {valid_config}: {pos_at_valid}")
