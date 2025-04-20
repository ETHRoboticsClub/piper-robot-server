import numpy as np


def R_x(angle_deg):
    angle = np.deg2rad(angle_deg)
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])


def R_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta), np.cos(theta), 0],
        [0, 0, 1]
    ])


def T(R, px, py, pz):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [px, py, pz]
    return T


def forward_kinematics(theta1, theta2, theta3, theta4):
    # Link lengths and heights
    # l0, l1, l2, l3, l4 = 0.038, 0.115, 0.135, 0.0675, 0.1
    # h0, h1, h2 = 0.05, 0.03, -0.01

    l0, l1, l2, l3, l4 = 0.03, 0.11, 0.134, 0.07, 0.1
    h0, h1, h2 = 0.052, 0.03, -0.005

    # hole to motor 1 - 4 cm
    # motor1 to motor2 - 3 cm
    # motor2  to motor3 - 11 cm
    # motor3 to motor4 - 13.4 cm
    # motor4 to end effector - 16.4

    # hole to motor2 height - 5.2 cm
    # h1 - 3 cm
    # h2 - 0.5 cm

    # Example parameters (in radians and arbitrary length units)
    theta1 = np.deg2rad(theta1)  # 45 degrees [86.748046875, 8.0859375, 354.90234375, 354.90234375]
    theta2 = np.deg2rad(theta2)  # 60 degrees
    theta3 = np.deg2rad(theta3) # 30 degrees
    theta4 = np.deg2rad(theta4)  # 90 degrees

    # From 0 to 1
    R01 = R_z(theta1) @ R_x(-90)
    T01 = T(R01, l0, 0, h0)
    # From 1 to 2
    R12 = R_z(theta2)
    T12 = T(R12, l1, h1, 0)
    # From 2 to 3
    R23 = R_z(theta3)
    T23 = T(R23, l2, h2, 0)
    # From 3 to 4
    R34 = R_z(theta4)
    T34 = T(R34, l3, 0, 0)
    # From 4 to 5
    R45 = np.eye(3)
    T45 = T(R45, l4, 0, 0)

    # Combine all transforms
    T05 = T01 @ T12 @ T23 @ T34 @ T45
    
     # Extract position
    position = T05[:3, 3]
    
    # Extract rotation matrix
    R = T05[:3, :3]
    
    # Calculate RPY angles in degrees
    # Roll (around x)
    roll = np.arctan2(R[2, 1], R[2, 2])
    # Pitch (around y)
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    # Yaw (around z)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    
    # Convert angles to degrees
    rpy = np.array([roll, pitch, yaw])
    rpy = np.rad2deg(rpy)
    
    return position, rpy

if __name__ == "__main__":
    theta1 = -10
    theta2 = 0
    theta3 = 0
    theta4 = 0

    print(forward_kinematics(theta1, theta2, theta3, theta4))
