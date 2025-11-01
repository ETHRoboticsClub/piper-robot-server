# Piper Robot Server

## Installation

### Prerequisites

1. **Robot Hardware**: One or two AgileX Piper arms
2. **Conda Environment**: Miniconda or Anaconda (required for pinocchio from conda-forge)
3. **Python Environment**: Python 3.10 (managed by conda)
4. **VR Setup** (optional): Meta Quest or other headset with WebXR support (no app installation needed!)

### Package Installation

Install tactile-teleop with conda environment setup:

```bash
# Clone the repository

# Create and activate conda environment
conda create -n piper python=3.10
conda activate piper

# Install teleop sdk
git clone git@github.com:TactileRoboticsAI/tactile-teleop-python-sdk.git
cd tactile-teleop-python-sdk
pip install -e .

# Install piper server
cd ..
git clone git@github.com:ETHRoboticsClub/piper-robot-server.git
cd piper-robot-server
pip install -e .
micromamba install pinocchio==3.2.0 casadi==3.6.7 -c conda-forge
```

Run the robot server:

```bash
robotserver
```

Without the physical robot and with update to meshcat visualizer:

```bash
robotserver --no-robot --vis
```

## OpenPI Policy Bridge

The teleoperation stack now bundles the WebSocket policy bridge that used to
live in `piper-policy-server`. It streams robot observations to an upstream
OpenPI policy and plays back returned joint commands on the Piper arms.

### Usage

1. Launch an OpenPI policy server (see the OpenPI repository for details):

   ```bash
   uv run ../openpi/scripts/serve_policy.py --env=Aloha
   ```

2. Start the Piper policy bridge on the robot computer:

   ```bash
   policybridge \
     --policy-host 192.168.1.20 \
     --policy-port 8000 \
     --rate 20 \
     --log-level INFO
   ```

   Add `--dry-run` to exercise the control loop without touching hardware.

To exercise the entire pipeline without an OpenPI policy, start the bundled
mock policy server:

```bash
mock-policy-server --port 8000 --random --log-level INFO

policybridge \
  --policy-host 127.0.0.1 \
  --policy-port 8000 \
  --camera-device 0 \
  --camera-width 1280 \
  --camera-height 720 \
  --camera-key observation/front_camera \
  --visualize \
  --dry-run
```

Camera frames are converted to RGB `uint8` by default; pass `--camera-bgr` to
keep OpenCV's native channel order.

To reuse the dual-camera setup from the teleoperation stack, load the teleop
configuration and publish both feeds in the observation:

```bash
policybridge \
  --policy-host 127.0.0.1 \
  --policy-port 8000 \
  --dry-run \
  --teleop-camera-type dual_camera_opencv \
  --teleop-camera-prefix observation/stereo \
  --log-level INFO
```

Frames appear under `observation/stereo/left` and `observation/stereo/right`.

### Observation Format

The bridge emits observations with the structure:

```python
{
    "timestamp": 1710000000.0,
    "left_arm": {
        "joint_names": ["joint_0.pos", ..., "joint_6.pos"],
        "joints": [0.1, ..., 0.05],
        "joints_dict": {"joint_0.pos": 0.1, ...},
        "ee_pose": {"x": 0.2, "y": 0.1, "z": 0.25, "roll": 0.0, "pitch": 1.57, "yaw": 0.0},
    },
    "right_arm": { ... },
}
```

Use flags such as `--no-ee-pose`, `--no-joint-list`, `--no-joint-names`, or
`--no-joint-dict` to trim this payload. Camera frames appear under the key
supplied with `--camera-key` (default: `observation/image`).

### Policy Responses

The policy should respond with either:

- Per-arm dictionaries containing joint targets in radians:

  ```python
  {
      "left_arm": {"joint_0.pos": 0.12, ..., "joint_6.pos": 0.04},
      "right_arm": {...}
  }
  ```

- Or an `"actions"` entry containing a list of dictionaries. The bridge will
  queue the list and apply the actions sequentially.

### Extensibility

- Configure cameras and observation fields on the CLI, or construct your own
  `ServerConfig`.
- Enable Meshcat visualisation with `--visualize` (requires the teleoperation
  dependencies such as `pinocchio`, `casadi`, and `meshcat`).
- Hardware interactions live in `piper_teleop.policy_bridge.robot`; extend
  `_build_observation` or `_extract_action` in `policy_loop` for custom
  payloads.
