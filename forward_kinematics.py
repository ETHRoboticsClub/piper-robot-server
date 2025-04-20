# --- Forward Kinematics ---
# This function takes in the angles of the motors and returns the position and orientation of the end effector
# Modified from rabhishek100's uploaded code on https://github.com/huggingface/lerobot/issues/568
import numpy as np


def R_x(angle_deg):
    angle = np.deg2rad(angle_deg)
    return np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)]
    ])


def R_y(angle_deg):
    angle = np.deg2rad(angle_deg)
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
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


def forward_kinematics(theta1, theta2, theta3, theta4, return_intermediate_frames=False):
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

    # --- Calculate Individual Transforms --- 
    # From 0 to 1 (Base to Shoulder_Rotation Joint Frame)
    # URDF Origin: xyz="0 -0.0452 0.0181", rpy="1.5708 0 1.5708" (90, 0, 90 deg)
    # URDF Axis: "0 1 0" (Y-axis of the joint frame)
    
    # Static rotation from Base to Joint Frame origin orientation
    R_base_to_joint_static = R_z(np.rad2deg(1.5708)) @ R_y(0) @ R_x(np.rad2deg(1.5708)) 
    
    # Rotation due to theta1 around the joint's Y-axis
    R_joint_rotation = R_y(np.rad2deg(theta1)) # theta1 was already converted to rad, convert back for R_y

    # --- Add corrective rotation based on observation (~-20 deg around Y) ---
    correction_angle_deg_01 = -20.0 
    R_correction_01 = R_y(correction_angle_deg_01)
    # --- End correction ---
    
    # Combined rotation from Base to final Frame 1 orientation
    # Apply correction *after* static and dynamic rotations
    R01 = R_base_to_joint_static @ R_joint_rotation @ R_correction_01 
    
    # Translation from Base origin to Joint Frame origin
    p01 = np.array([0, -0.0452, 0.0181])
    
    # Combined Transformation Matrix T01
    T01 = T(R01, p01[0], p01[1], p01[2])

    # --- Original FK calculations (kept for reference, will need updating) ---
    # R01 = R_z(theta1) @ R_x(-90) 
    # T01 = T(R01, l0, 0, h0) 
    # --- End Original --- 

    # --- From 1 to 2 (Shoulder_Pitch Joint Frame) --- 
    # URDF Origin: xyz="0.000125 0.1086 0", rpy="3.1416 0 -1.5708" (180, 0, -90 deg)
    # URDF Axis: "0 0 1" (Z-axis of the joint frame)
    # --- REVERTING EXPERIMENTAL OFFSET --- 
    # urdf_yaw_12 = -1.5708
    # observed_offset_yaw_12 = -0.5236 # Approx -30 deg
    # adjusted_yaw_12 = urdf_yaw_12 + observed_offset_yaw_12
    # --- End Revert --- 

    # Static rotation from Frame 1 to Joint Frame 2 origin orientation
    # Use original URDF yaw
    R_frame1_to_joint2_static = R_z(np.rad2deg(-1.5708)) @ R_y(0) @ R_x(np.rad2deg(3.1416))
    
    # Rotation due to theta2 around the joint's Z-axis
    R_joint2_rotation = R_z(theta2) # theta2 is already in radians

    # --- Add corrective rotation based on observation (~-15 deg around Z) ---
    correction_angle_deg_12 = -15.0
    R_correction_12 = R_z(np.deg2rad(correction_angle_deg_12)) # Correct around Z
    # --- End correction ---

    # Combined rotation from Frame 1 to final Frame 2 orientation
    # Apply correction *after* static and dynamic rotations
    R12 = R_frame1_to_joint2_static @ R_joint2_rotation @ R_correction_12

    # Translation from Frame 1 origin to Joint Frame 2 origin
    p12 = np.array([0.000125, 0.1086, 0])

    # Combined Transformation Matrix T12
    T12 = T(R12, p12[0], p12[1], p12[2])

    # --- Original FK Calculation --- 
    # R12 = R_z(theta2)
    # T12 = T(R12, l1, h1, 0)
    # --- End Original --- 

    # --- From 2 to 3 (Elbow Joint Frame) --- 
    # URDF Origin: xyz="-0.11238 0.0282 0", rpy="0 0 -2.2391" (0, 0, -128.3 deg)
    # URDF Axis: "0 0 1" (Z-axis of the joint frame)
    
    # Static rotation from Frame 2 to Joint Frame 3 origin orientation
    R_frame2_to_joint3_static = R_z(np.rad2deg(-2.2391)) @ R_y(0) @ R_x(0)
    
    # Rotation due to theta3 around the joint's Z-axis
    R_joint3_rotation = R_z(theta3) # theta3 is already in radians

    # Combined rotation from Frame 2 to final Frame 3 orientation
    R23 = R_frame2_to_joint3_static @ R_joint3_rotation

    # Translation from Frame 2 origin to Joint Frame 3 origin
    p23 = np.array([-0.11238, 0.0282, 0])

    # Combined Transformation Matrix T23
    T23 = T(R23, p23[0], p23[1], p23[2])

    # --- Original FK Calculation --- 
    # R23 = R_z(theta3)
    # T23 = T(R23, l2, h2, 0)
    # --- End Original --- 

    # --- From 3 to 4 (Wrist_Pitch Joint Frame) --- 
    # URDF Origin: xyz="-0.1102 0.005375 0", rpy="0.90254 1.5708 0" (51.7, 90, 0 deg)
    # URDF Axis: "1 0 0" (X-axis of the joint frame)

    # Static rotation from Frame 3 to Joint Frame 4 origin orientation
    R_frame3_to_joint4_static = R_z(0) @ R_y(np.rad2deg(1.5708)) @ R_x(np.rad2deg(0.90254))

    # Rotation due to theta4 around the joint's X-axis
    # Convert theta4 (which is in rad) back to degrees for R_x function
    R_joint4_rotation = R_x(np.rad2deg(theta4))

    # Combined rotation from Frame 3 to final Frame 4 orientation
    R34 = R_frame3_to_joint4_static @ R_joint4_rotation

    # Translation from Frame 3 origin to Joint Frame 4 origin
    p34 = np.array([-0.1102, 0.005375, 0])

    # Combined Transformation Matrix T34
    T34 = T(R34, p34[0], p34[1], p34[2])

    # --- Original FK Calculation --- 
    # R34 = R_z(theta4)
    # T34 = T(R34, l3, 0, 0)
    # --- End Original ---

    # From 4 to 5 (End Effector point)
    # Define offset relative to Frame 4. Now -2cm along X, -12cm along Z.
    R45 = np.eye(3) # No extra rotation
    # l4 = 0.1 # Original offset along X
    T45 = T(R45, -0.04, 0, -0.12) # Offset -2cm X, -12cm Z relative to Frame 4

    # --- Calculate Cumulative Transforms --- 
    T02 = T01 @ T12
    T03 = T02 @ T23
    T04 = T03 @ T34
    T05 = T04 @ T45 # Final end effector transform
    
    # --- Extract Final Position and Orientation --- 
    position = T05[:3, 3]
    R = T05[:3, :3]
    # Calculate RPY angles in degrees
    # Roll (around x)
    roll = np.arctan2(R[2, 1], R[2, 2])
    # Pitch (around y)
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    # Yaw (around z)
    yaw = np.arctan2(R[1, 0], R[0, 0])
    
    # Convert angles to degrees
    rpy = np.rad2deg(np.array([roll, pitch, yaw]))
    
    # --- Return requested values --- 
    if return_intermediate_frames:
        # Return final position, orientation, and the first four frame transforms
        return position, rpy, T01, T02, T03, T04 # Return T01, T02, T03, T04
    else:
        # Default: return final position and orientation only
        return position, rpy

if __name__ == "__main__":
    theta1 = -10
    theta2 = 0
    theta3 = 0
    theta4 = 0

    print(forward_kinematics(theta1, theta2, theta3, theta4))
