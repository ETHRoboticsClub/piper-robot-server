# Piper Robot Server

## Installation

### Prerequisites

1. **Robot Hardware**: One or two AgileX Piper arms
2. **Conda Environment**: Miniconda or Anaconda (required for pinocchio from conda-forge)
3. **Python Environment**: Python 3.10 (managed by conda)
4. **VR Setup** (optional): Meta Quest or other headset with WebXR support (no app installation needed!)

### Package Installation
Run either:
```bash
chmod +x setup_environment.sh
./setup_environment.sh
```
This will handle the installation of your conda environement. If you cannot build the wheel for pybullet (happens so far on mac), install it via:
```bash
conda activate piper
conda install -c conda-forge pybullet
```

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

