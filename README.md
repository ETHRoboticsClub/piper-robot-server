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
conda install pinocchio==3.2.0 casadi==3.6.7 -c conda-forge
```

## Usage

### Enabling Robot Control

Follow these steps to enable the piper arms:

1. **Connect the piper arms** to your computer via USB-C

2. **Check the visible CAN connections** via `ip link show`:
   - If only follower arms are connected (VR teleop): `can0` and `can1` should be visible
   - If leader-follower teleop is desired: `can0`, `can1`, `can2`, `can3` need to be visible

3. **Initial setup or after changing cable/robot arrangement**:
   - Unplug all the USB-C cables to the robots
   - Plug them in one by one and run `src/piper_teleop/robot_server/find_all_can_port.sh` each time
   - Assign the correct CAN port number to the right robot name in `src/piper_teleop/robot_server/can_config.sh` line 130-136.

4. **Rename the CAN ports**:
   - For follower arms only (2 CAN ports):
     ```bash
     sudo bash src/piper_teleop/robot_server/can_config.sh
     ```
   - For leader + follower arms (4 CAN ports):
     ```bash
     ENABLE_LEADER_ARMS=true sudo bash src/piper_teleop/robot_server/can_config.sh
     ```    

### Enabling Cameras

Follow these steps to enable camera support:

1. **Connect the cameras** to your computer e.g. via USB-A cables:
   - Typically, you'll need a wrist camera for each follower arm
   - One top-down camera for overall scene view

2. **Identify camera indices**:
   ```bash
   python scripts/find_cameras.py
   ```
   - Check which camera index corresponds to which physical camera
   - **Important**: Note which index corresponds to the right arm and which to the left arm

3. **Configure camera indices** in `src/piper_teleop/config.py`:
   - In the `DEFAULT_CONFIG` under `cameras`, set the correct index for:
     - Left arm camera
     - Right arm camera
     - Stereo camera (if applicable)
   - **Custom camera setup**: For example, to use a laptop webcam:
     - Delete the current camera entries
     - Add a single monocular camera entry with the webcam index


### Basic Commands

**Run the robot server with VR control (default):**
```bash
robotserver
```

**Run without physical robot (visualization only):**
```bash
robotserver --no-robot --vis
```

### Command Line Options

#### Control Modes

- **`--keyboard`**: Enable keyboard control instead of VR control
  - Use keyboard input to control robot arms
  - Default: `False` (VR control is used by default)

- **`--leader`**: Enable Leader-Follower setup
  - Uses physical leader robot arms to control follower arms
  - Leader arms must be connected on ports `leader_left` and `leader_right`
  - Reads joint positions from leader arms and sends them to follower arms

- **`--policy`**: Enable policy control
  - Uses a trained LeRobot policy to autonomously control the robot
  - Policy path and repo ID are configured in `config.py`
  - Requires robot observations and camera data for inference

#### Robot Connection

- **`--no-robot`**: Disable robot hardware connection
  - Runs in simulation/visualization mode only
  - Useful for testing without physical hardware
  - Can be combined with `--vis` for visualization

- **`--vis`**: Enable visualization
  - Enables Meshcat visualizer to display robot state
  - Shows robot pose and movements in 3D visualization
  - Useful for debugging and monitoring

#### Recording

- **`--record`**: Enable data recording
  - Records robot observations, actions, and camera data
  - Saves data in LeRobot dataset format
  - Requires at least one camera in recording or hybrid mode
  - Data is saved to `data/YYYY-MM-DD_HH-MM-SS/` directory

- **`--resume`**: Resume recording
  - Continues recording to an existing dataset
  - Appends new episodes to the current recording session

- **`--repo-id REPO_ID`**: Specify repository ID for dataset storage
  - Sets the HuggingFace repository ID for dataset storage
  - Default: `"default-piper"`
  - Used when uploading datasets to HuggingFace

#### Logging

- **`--log-level LEVEL`**: Set logging verbosity
  - Options: `debug`, `info`, `warning`, `error`, `critical`
  - Default: `info`
  - Use `debug` for detailed troubleshooting, `warning` for minimal output

### Common Usage Examples

**VR teleoperation with recording:**
```bash
robotserver --record
```

**Leader arm teleoperation with recording:**
```bash
robotserver --record --leader
```

**Keyboard control with visualization and no robot:**
```bash
robotserver --keyboard --vis --no-robot
```

**Policy control:**
```bash
robotserver --policy
```

### Common Data Recording Workflow

#### 1. Record a Dataset



Record a dataset with either VR:
```bash
robotserver --record
```

or with a leader arm:
```bash
robotserver --record --leader
```

**Keyboard Controls for Recording:**
- **`→` (Right Arrow)**: Navigate through states (reset env → start recording → save recording)
- **`←` (Left Arrow)**: Stop and discard the current recording
- **`Esc`**: Stop the recording session

#### 2. Locate Saved Data

Find the saved data under:
```
data/YYYY-MM-DD_HH-MM-SS/
```

#### 3. Convert to Video Format

Convert image dataset to video format:
```bash
python scripts/convert_images_to_video.py /path/to/dataset
```

#### 4. Upload to HuggingFace

Upload the dataset to HuggingFace Hub:
```bash
python scripts/upload_to_huggingface.py /path/to/dataset ETHRC/my-dataset-name
```

#### 5. Inspect Data

Inspect your dataset at [LeRobot Dataset Visualizer](https://huggingface.co/spaces/lerobot/visualize_dataset) by entering your repository ID (e.g., `ETHRC/my-dataset-name`).
