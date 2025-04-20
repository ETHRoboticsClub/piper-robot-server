# --- Inverse Kinematics ---
# This function takes in a desired position and pitch angle and returns the angles of the motors
# Modified from rabhishek100's uploaded code on https://github.com/huggingface/lerobot/issues/568
import numpy as np
from forward_kinematics import forward_kinematics
import json
import os
import sys

# --- Load Joint Limits --- 
LIMITS_FILE = "limits.json"
joint_limits_deg = None
if os.path.exists(LIMITS_FILE):
    try:
        with open(LIMITS_FILE, 'r') as f:
            limits_data = json.load(f)
            # Ensure data is sorted by ID (1-4) if it isn't already
            # limits_data.sort(key=lambda x: x['id'])
            # Assuming the list is already in the correct order (ID 1, 2, 3, 4)
            if len(limits_data) >= 4:
                min_limits = [entry['min_degrees'] for entry in limits_data[:4]]
                max_limits = [entry['max_degrees'] for entry in limits_data[:4]]
                joint_limits_deg = (np.array(min_limits), np.array(max_limits))
                print(f"Loaded joint limits (degrees) from {LIMITS_FILE}:")
                print(f"  Min: {joint_limits_deg[0]}")
                print(f"  Max: {joint_limits_deg[1]}")
            else:
                print(f"Warning: {LIMITS_FILE} does not contain enough entries for 4 joints.", file=sys.stderr)
    except (IOError, json.JSONDecodeError, KeyError) as e:
        print(f"Warning: Could not load or parse {LIMITS_FILE}. Error: {e}", file=sys.stderr)
else:
    print(f"Warning: Limits file {LIMITS_FILE} not found. Inverse kinematics will operate without joint limits.", file=sys.stderr)
# --- End Load Joint Limits ---

def numeric_jacobian(fk_func, angles, eps=1e-6):
    # We now only consider position (x, y, z) and pitch angle.
    # So we produce a 4xN Jacobian.
    J = np.zeros((4, len(angles)))
    for i in range(len(angles)):
        angles_fwd = angles.copy()
        angles_bwd = angles.copy()
        angles_fwd[i] += eps
        angles_bwd[i] -= eps

        pos_fwd, rpy_fwd = fk_func(*angles_fwd)
        pos_bwd, rpy_bwd = fk_func(*angles_bwd)

        # Position derivative
        J[0:3, i] = (pos_fwd - pos_bwd) / (2*eps)

        # Pitch derivative (second component of RPY)
        pitch_diff = (rpy_fwd[1] - rpy_bwd[1])
        pitch_diff = (pitch_diff + 180) % 360 - 180
        J[3, i] = pitch_diff / (2*eps)

    return J

def iterative_ik(target_pos, target_pitch, initial_guess=[0,0,0,0], 
                 limits_deg=None, # Add limits parameter
                 max_iter=100,     # Reduced max iterations
                 tol=5e-4,         # Increased tolerance further
                 alpha=0.5):       # Increased alpha back to 0.5
    """Calculates inverse kinematics, optionally clamping angles to limits."""
    angles = np.array(initial_guess, dtype=float)
    
    # Separate min/max limits if provided
    min_angles = None
    max_angles = None
    if limits_deg is not None:
        min_angles, max_angles = limits_deg
        if len(min_angles) != len(angles) or len(max_angles) != len(angles):
            raise ValueError("Limits array size must match number of angles (4).")

    for iter_count in range(max_iter):
        current_pos, current_rpy = forward_kinematics(*angles)

        # Position error
        pos_error = target_pos - current_pos

        # Pitch error (ensure proper angle wrapping)
        pitch_error = target_pitch - current_rpy[1]
        pitch_error = (pitch_error + 180) % 360 - 180

        # Combined error: [ex, ey, ez, epitch]
        error = np.hstack((pos_error, pitch_error))
        
        if np.linalg.norm(error) < tol:
            # print(f"IK converged in {iter_count} iterations.") # Optional debug print
            break

        # --- Jacobian and Angle Update --- 
        J = numeric_jacobian(forward_kinematics, angles)
        
        # --- Use Damped Least Squares --- 
        lambda_damping = 0.03
        try:
            # Calculate DLS pseudo-inverse
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_damping**2 * np.eye(J.shape[0]))
        except np.linalg.LinAlgError:
            # Fallback to standard pinv if matrix is singular even with damping
            # print("Warning: DLS matrix inversion failed, falling back to pinv.", file=sys.stderr)
            J_pinv = np.linalg.pinv(J)

        delta_angles = alpha * (J_pinv @ error)
        angles += delta_angles

        # --- Clamp angles to limits (unchanged) --- 
        if min_angles is not None and max_angles is not None:
            angles = np.clip(angles, min_angles, max_angles)
        
    else:
        # Loop finished without converging
        # print(f"IK did not converge within {max_iter} iterations.") # Optional debug print
        pass 

    return angles

if __name__ == "__main__":
    # Desired target position and pitch angle
    target_pos = np.array([0.2, 0.1, 0.15])
    target_pitch = 90  # Just control pitch

    print("\nRunning IK example...")
    # Pass loaded limits (or None if loading failed)
    solution_angles = iterative_ik(target_pos, target_pitch, 
                                 initial_guess=[0,0,0,0], 
                                 limits_deg=joint_limits_deg) 
    print("IK solution angles (degrees):", solution_angles)

    # Check final error
    final_pos, final_rpy = forward_kinematics(*solution_angles)
    pos_error = target_pos - final_pos
    pitch_error = target_pitch - final_rpy[1]
    pitch_error = (pitch_error + 180) % 360 - 180

    print("Final position error:", pos_error, "Norm:", np.linalg.norm(pos_error))
    print("Final pitch error:", pitch_error)
