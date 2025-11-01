# Simulation Mode

This document explains how to use the Piper Robot Server in simulation mode.

## Overview

The robot server now supports running in simulation mode without physical hardware. This is useful for:
- Development and testing
- Training and demonstration
- Algorithm development
- When hardware is not available

## Supported Simulators

### 1. PyBullet (Default)
- Lightweight physics simulator
- Fast and easy to use
- Good for basic kinematics and dynamics
- Built-in collision detection

### 2. Newton (Coming Soon)
- More advanced physics
- Better visual quality
- Custom implementation

## Installation

### PyBullet (Recommended for now)

```bash
pip install pybullet
```

Or if you've already installed the package, just update:

```bash
pip install -e .
```

## Usage

### Basic Simulation Mode

Run with PyBullet simulation:

```bash
robotserver --sim
```

This will:
- Start PyBullet simulation
- Disable hardware connection
- Use VR controllers for input (or keyboard with `--keyboard`)
- Display simulation window if `--vis` is enabled

### With Visualization

```bash
robotserver --sim --vis
```

### With Keyboard Control

```bash
robotserver --sim --vis --keyboard
```

### Recording in Simulation

You can record demonstrations in simulation mode:

```bash
robotserver --sim --vis --keyboard --record --repo-id my-sim-dataset
```

### Newton Simulator (Placeholder)

When Newton is implemented:

```bash
robotserver --use-newton --vis
```

## Command Line Arguments

- `--sim`: Run in PyBullet simulation mode
- `--use-newton`: Run in Newton simulation mode (not yet implemented)
- `--vis`: Enable visualization (shows PyBullet GUI or meshcat)
- `--keyboard`: Use keyboard controls instead of VR
- `--no-robot`: Old way to disable hardware (now use `--sim` instead)

## Architecture

### Simulation Interface

The simulation is implemented through a common interface that matches the hardware SDK:

```python
# Hardware mode
piper_sdk_interface.py  # Real CAN bus communication

# Simulation mode  
sim_interface.py        # PyBullet or Newton simulation
```

Both provide the same API:
- `connect()` / `disconnect()`
- `set_joint_positions(positions)`
- `get_status()` -> joint states
- `get_end_effector_pose()` -> EE pose

### Integration

The `Piper` class automatically selects the correct interface:

```python
class Piper:
    def __init__(self, config: PiperConfig):
        if config.use_sim:
            self.sdk = PyBulletSimInterface(...)  # or NewtonSimInterface
        else:
            self.sdk = PiperSDKInterface(...)  # Hardware
```

## Testing

A test script is provided to verify the simulation:

```bash
python test_simulation.py
```

This will:
1. Initialize PyBullet simulation
2. Connect both robot arms
3. Send test commands
4. Verify joint states
5. Test end effector poses
6. Return to initial position

## Accessing Simulation Data

### From Python (Same Repo)

```python
from piper_teleop.config import TelegripConfig
from piper_teleop.robot_server.core.robot_interface import RobotInterface

# Create config for simulation
config = TelegripConfig()
config.enable_robot = False
config.run_in_newton = False  # Use PyBullet

# Create robot interface
robot_interface = RobotInterface(config)
robot_interface.setup_kinematics()
robot_interface.connect()

# Get observations
obs = robot_interface.get_observation()
print(obs['left'])  # Left arm joint states
print(obs['right']) # Right arm joint states

# Get actions being sent
action_dict = robot_interface.arm_angles
# This contains the 14 joint angles being commanded
```

### From Different Repository

If you want to access simulation data from a different repository, you have options:

#### Option A: Use as Python Library

```python
# In your separate simulation repo
from piper_teleop.robot_server.core.robot_interface import RobotInterface
from piper_teleop.config import TelegripConfig

# Use the interface directly
config = TelegripConfig()
config.run_in_newton = False
robot = RobotInterface(config)
```

#### Option B: Add Communication Layer (Future Enhancement)

We can add ZeroMQ, Redis, or ROS bridge to publish actions. See main README for details.

## Limitations

Current limitations of simulation mode:

1. **Physics fidelity**: PyBullet is approximate, not perfect
2. **Gripper**: Simplified gripper model
3. **Sensors**: Limited sensor simulation
4. **Newton**: Not yet implemented (placeholder only)
5. **Multi-robot**: Each arm runs in separate PyBullet instance

## Extending the Simulation

### Adding Newton Support

1. Implement `NewtonSimInterface` in `sim_interface.py`
2. Add Newton-specific initialization
3. Implement the interface methods:
   - `connect()`, `disconnect()`
   - `set_joint_positions()`
   - `get_status()`
   - `get_end_effector_pose()`

### Adding Other Simulators

Follow the same pattern:

```python
class MySimInterface(SimulationInterface):
    def __init__(self, urdf_path: str, use_gui: bool = False):
        super().__init__(urdf_path, use_gui)
        # Your initialization
        
    def connect(self):
        # Connect to your simulator
        
    # Implement other methods...
```

Then add to `Piper` class in `piper.py`:

```python
if config.sim_type == "mysim":
    self.sdk = MySimInterface(...)
```

## Troubleshooting

### PyBullet not found

```bash
pip install pybullet
```

### URDF not loading

Check that the URDF path is correct in config:
```python
config.urdf_path = "URDF/Piper/dual_piper.urdf"
```

### No visualization window

Make sure to use `--vis` flag:
```bash
robotserver --sim --vis
```

### Simulation too slow

1. Disable GUI if not needed
2. Reduce simulation frequency
3. Use `--no-vis` for faster execution

## Future Enhancements

Planned improvements:

- [ ] Newton simulator integration
- [ ] Better gripper simulation
- [ ] Sensor simulation (cameras, force/torque)
- [ ] Communication layer (ZeroMQ/ROS) for external access
- [ ] Multiple robots in same simulation
- [ ] Record and replay capabilities
- [ ] Domain randomization for sim-to-real transfer
