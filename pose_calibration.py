# --- Pose Calibration Script ---
# Allows commanding individual joints, disabling torque, and comparing
# the pybullet visualization (based on measured steps) with the physical robot.

import numpy as np
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus, TorqueMode, FeetechMotorsBusConfig
import time
from pynput import keyboard
import sys
import pybullet as p
import pybullet_data
import math

# --- Calibration Data (Copied from keyboard_teleop.py) --- 
def get_base_calibration(): # Keep for reference if needed, but use full below
    return [
        {"zero_step": 2048, "direction": -1}, # Shoulder_Rotation (ID 1)
        {"zero_step": 2052, "direction": -1},  # Shoulder_Pitch (ID 2)
        {"zero_step": 2047, "direction": -1},  # Elbow (ID 3)
        {"zero_step": 2047, "direction": -1},  # Wrist_Pitch (ID 4)
    ]

def get_full_calibration():
    """Calibration for all 6 joints (used for visualization)."""
    # --- Use Servos Offsets from JSON as Zero Step for Viz --- 
    # --- Use Experimentally Verified Directions --- 
    # servos_offsets = [2048.0, 2052.0, 2047.0, 2047.0, 2043.0, 2048.0]
    zero_steps = [2048.0, 2052.0, 2500.0, 2600.0, 3846.0, 2048.0] # found experimentally
    directions = [-1, -1, -1, 1, -1, 1] # found experimentally
    
    if len(zero_steps) != 6 or len(directions) != 6:
        raise ValueError("Incorrect number of calibration values defined.")

    return [
        {"zero_step": zero_steps[0], "direction": directions[0]}, # ID 1: Shoulder_Rotation 
        {"zero_step": zero_steps[1], "direction": directions[1]}, # ID 2: Shoulder_Pitch
        {"zero_step": zero_steps[2], "direction": directions[2]}, # ID 3: Elbow
        {"zero_step": zero_steps[3], "direction": directions[3]}, # ID 4: Wrist_Pitch
        {"zero_step": zero_steps[4], "direction": directions[4]}, # ID 5: Wrist_Roll 
        {"zero_step": zero_steps[5], "direction": directions[5]}  # ID 6: Gripper
    ]

# --- Angle Conversion for Viz (Copied) --- 
def steps_to_radians_for_viz(steps, calib_data):
    if len(steps) != len(calib_data):
        # Fallback if reading less than 6 motors initially
        print(f"Warning: Expected {len(calib_data)} steps, got {len(steps)}. Padding with zeros.", file=sys.stderr)
        steps = np.pad(np.array(steps), (0, len(calib_data) - len(steps)), 'constant')
        # raise ValueError(f"Expected {len(calib_data)} steps for visualization conversion.")
    radians_per_step = 2 * math.pi / 4096.0
    radian_values = []
    for i, step in enumerate(steps):
        cal = calib_data[i]
        zero_step = cal["zero_step"]
        direction = cal["direction"]
        angle_value = (step - zero_step) * direction * radians_per_step 
        radian_values.append(angle_value)
    return radian_values

# --- Robot and Motor Config (Copied, using URDF names) --- 
follower_port = "/dev/ttySO100follower"
follower_motors = {
    "Shoulder_Rotation": (1, "sts3215"),
    "Shoulder_Pitch": (2, "sts3215"),
    "Elbow": (3, "sts3215"),
    "Wrist_Pitch": (4, "sts3215"),
    "Wrist_Roll": (5, "sts3215"),
    "Gripper": (6, "sts3215"),
}
follower_config = FeetechMotorsBusConfig(port=follower_port, motors=follower_motors)
MOTOR_NAMES_ORDERED = list(follower_motors.keys())
NUM_MOTORS = len(MOTOR_NAMES_ORDERED)

# --- PyBullet Setup (Copied) --- 
def setup_pybullet():
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0,0,-9.81)
    p.loadURDF("plane.urdf")
    urdf_path = "URDF/SO_5DOF_ARM100_05d.SLDASM/urdf/SO_5DOF_ARM100_05d.SLDASM.urdf"
    robot_start_pos = [0, 0, 0]
    robot_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    print(f"Loading URDF: {urdf_path}")
    robotId = p.loadURDF(urdf_path, robot_start_pos, robot_start_orientation, useFixedBase=1)
    print(f"Pybullet Robot ID: {robotId}")
    num_joints = p.getNumJoints(robotId)
    p_joint_indices = {}
    for i in range(num_joints):
        info = p.getJointInfo(robotId, i)
        joint_name = info[1].decode('UTF-8')
        if joint_name in follower_motors:
            p_joint_indices[joint_name] = i
            p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, force=0)
    print(f"Mapped Pybullet Joint Indices: {p_joint_indices}")
    if len(p_joint_indices) != NUM_MOTORS:
         print(f"Warning: Mapped {len(p_joint_indices)} joints, expected {NUM_MOTORS}. Check URDF names vs follower_motors keys.")
    return physicsClient, robotId, p_joint_indices

# --- Global Variables for Keyboard Control --- 
selected_joint_index = 0 # Keep track for potential future use, but not actively controlled
# target_step_positions = [2048] * NUM_MOTORS # No longer adjusting target this way
# step_increment = 50 # No longer adjusting target this way
torque_enabled = True # Assume starts enabled
exit_requested = False
# --- Flags to signal actions from listener to main loop --- 
# send_goal_requested = False # Removed 'g' command
toggle_torque_requested = False

# --- Keyboard Handlers --- 
def on_press(key):
    # Remove unused globals if they aren't needed elsewhere
    global selected_joint_index, torque_enabled, exit_requested
    global toggle_torque_requested # Make flags global

    try:
        # --- Joint Selection (1-6) --- 
        if '1' <= key.char <= str(NUM_MOTORS):
            # Adjust index for 0-based list access
            selected_joint_index = int(key.char) - 1 
            print(f"\n(Selected joint index {selected_joint_index} for reference: {MOTOR_NAMES_ORDERED[selected_joint_index]})")
        # Removed 'g' key logic
        # --- Torque Toggle --- 
        elif key.char == 't':
            print(f"\n>>> TOGGLE torque requested (current: {'ENABLED' if torque_enabled else 'DISABLED'}) <<< ")
            toggle_torque_requested = True # Signal main loop to toggle
        # --- Exit --- 
        elif key.char == 'c':
             print("\nExit requested by user.")
             exit_requested = True
             
    except AttributeError:
        # --- Removed Arrow Key Logic --- 
        pass # Ignore arrow keys and other special keys

# --- Main Script --- 
if __name__ == "__main__":
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    physicsClient, robotId, p_joint_indices = setup_pybullet()
    follower_arm = None
    full_calibration = get_full_calibration()
    last_read_positions = [2048] * NUM_MOTORS
    all_motor_ids = [m[0] for m in follower_motors.values()]

    try:
        print("\nConnecting to follower arm...")
        follower_arm = FeetechMotorsBus(follower_config)
        follower_arm.connect()
        print("Connected.")
        initial_positions = follower_arm.read("Present_Position")
        if len(initial_positions) == NUM_MOTORS:
            target_step_positions = list(initial_positions) 
            last_read_positions = list(initial_positions)
        else:
             print(f"Warning: Read {len(initial_positions)} initial positions, expected {NUM_MOTORS}. Using defaults.")

        print("\n--- Controls ---:")
        print("  1-6: Select Joint")
        print("  t: Toggle Torque Enable/Disable")
        print("  c: Exit Script")
        print("-----------------")

        # Set initial torque state
        print("Ensuring torque is initially ENABLED.")
        follower_arm.write("Torque_Enable", TorqueMode.ENABLED.value)
        torque_enabled = True # Sync state variable

        last_print_time = time.monotonic()
        # Rename print_interval as status_print_interval
        status_print_interval = 0.5 # How often to print basic status 
        # Add timer for manual mode printing
        last_manual_print_time = time.monotonic()
        manual_print_interval = 1.0 # Print actual steps every 1 second in manual mode

        while not exit_requested:
            current_time = time.monotonic()

            # --- Process Action Flags Set by Listener --- 
            if toggle_torque_requested:
                torque_enabled = not torque_enabled # Toggle state first
                mode = TorqueMode.ENABLED.value if torque_enabled else TorqueMode.DISABLED.value
                status_str = "ENABLED" if torque_enabled else "DISABLED"
                print(f"\nSetting torque to {status_str}...", end='')
                try:
                    follower_arm.write("Torque_Enable", mode)
                    print(" Done.")
                    # If disabling torque, print header for manual mode
                    if not torque_enabled:
                        print("--- MANUAL MODE ACTIVE: Move arm physically, observe Actual Steps below --- ")
                    else:
                        print("--- TORQUE ENABLED --- ") # Clear previous status
                except Exception as e:
                    print(f" Error: {e}")
                toggle_torque_requested = False # Reset flag
            
            # --- Read Actual Position --- 
            read_error = False
            try:
                current_read = follower_arm.read("Present_Position")
                if len(current_read) == NUM_MOTORS:
                    last_read_positions = list(current_read) # Convert to list
                else:
                    # Keep last good reading, maybe print warning less often?
                    read_error = True
            except Exception as read_err:
                # Keep last good reading
                read_error = True
                # Maybe print warning less often here too

            # --- Update PyBullet Visualization --- 
            if len(last_read_positions) == NUM_MOTORS:
                current_radians_viz = steps_to_radians_for_viz(last_read_positions, full_calibration)
                for i in range(NUM_MOTORS):
                    motor_name = MOTOR_NAMES_ORDERED[i]
                    if motor_name in p_joint_indices:
                        joint_index = p_joint_indices[motor_name]
                        p.resetJointState(robotId, joint_index, targetValue=current_radians_viz[i])
            
            # --- Conditional Status Printing --- 
            # Print actual steps at 1Hz if torque is OFF
            if not torque_enabled and (current_time - last_manual_print_time >= manual_print_interval):
                actual_steps_str = ", ".join(map(str, last_read_positions))
                # Print on a new line to avoid messing with other prints
                print(f"\n[Manual Mode] Actual Steps: [{actual_steps_str}]") 
                last_manual_print_time = current_time
                last_print_time = current_time # Reset other timer too
            
            # Optionally print basic status less frequently or only when torque is on
            elif torque_enabled and (current_time - last_print_time >= status_print_interval):
                 # Simplified status print for when torque is on
                 print(f"\rTorque: ENABLED (Press 't' to disable for manual mode)   ", end='')
                 last_print_time = current_time
            elif read_error and (current_time - last_print_time >= status_print_interval):
                # Print read error status occasionally if it occurs
                print(f"\rRead Error Detected. Using last known positions. Torque: {torque_status}   ", end='')
                last_print_time = current_time

            # Let pybullet update
            p.stepSimulation() 
            time.sleep(1./240.)

    except KeyboardInterrupt:
        print("\nCtrl+C detected.")
    except Exception as e:
        print(f"\nAn error occurred in main loop: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\nCleaning up...")
        if listener and listener.is_alive():
            listener.stop()
            print("Keyboard listener stopped.")
        # Ensure torque is disabled before disconnecting if possible
        if follower_arm and follower_arm.is_connected:
            try:
                print("Attempting to disable torque...")
                follower_arm.write("Torque_Enable", TorqueMode.DISABLED.value)
                time.sleep(0.2)
            except Exception as e_torque:
                print(f"Could not disable torque on exit: {e_torque}", file=sys.stderr)
            finally:
                 follower_arm.disconnect()
                 print("Follower arm disconnected.")
        if physicsClient >= 0:
            p.disconnect()
            print("PyBullet disconnected.")
        print("Cleanup finished.") 