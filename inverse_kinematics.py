import numpy as np
from forward_kinematics import forward_kinematics

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
                 max_iter=1000, tol=1e-6, alpha=0.5):
    angles = np.array(initial_guess, dtype=float)
    for _ in range(max_iter):
        current_pos, current_rpy = forward_kinematics(*angles)

        # Position error
        pos_error = target_pos - current_pos

        # Pitch error (ensure proper angle wrapping)
        pitch_error = target_pitch - current_rpy[1]
        pitch_error = (pitch_error + 180) % 360 - 180

        # Combined error: [ex, ey, ez, epitch]
        error = np.hstack((pos_error, pitch_error))
        
        if np.linalg.norm(error) < tol:
            break

        # Compute numeric Jacobian (4 x N)
        J = numeric_jacobian(forward_kinematics, angles)
        J_pinv = np.linalg.pinv(J)

        angles += alpha * (J_pinv @ error)
        angles = np.mod(angles, 360)

    return angles

if __name__ == "__main__":
    # Desired target position and pitch angle
    target_pos = np.array([0.2, 0.1, 0.15])
    target_pitch = 90  # Just control pitch

    solution_angles = iterative_ik(target_pos, target_pitch, initial_guess=[0,0,0,0])
    print("IK solution angles (degrees):", solution_angles)

    # Check final error
    final_pos, final_rpy = forward_kinematics(*solution_angles)
    pos_error = target_pos - final_pos
    pitch_error = target_pitch - final_rpy[1]
    pitch_error = (pitch_error + 180) % 360 - 180

    print("Final position error:", pos_error, "Norm:", np.linalg.norm(pos_error))
    print("Final pitch error:", pitch_error)
