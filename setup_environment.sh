#!/bin/bash
# Setup script for piper robot server environment
# This creates a clean conda environment without cross-contamination

set -e  # Exit on error

echo "🚀 Setting up Piper Robot Server environment..."

# Check if piper environment exists
if conda env list | grep -q "^piper "; then
    echo "⚠️  Environment 'piper' already exists. Remove it first with: conda env remove -n piper"
    exit 1
fi

echo "📦 Creating conda environment 'piper' with Python 3.10..."
conda create -n piper python=3.10 -y

echo "🔧 Activating environment..."
eval "$(conda shell.bash hook)"
conda activate piper

# Ensure we're in the right environment
if [[ "$CONDA_DEFAULT_ENV" != "piper" ]]; then
    echo "❌ Failed to activate piper environment"
    exit 1
fi

echo "✅ Activated environment: $CONDA_DEFAULT_ENV"
echo "🐍 Python location: $(which python)"

# Clear any PYTHONPATH that might leak other environments
echo "🧹 Clearing PYTHONPATH..."
unset PYTHONPATH

echo "📚 Installing conda packages (pinocchio, casadi)..."
conda install pinocchio==3.2.0 casadi==3.6.7 -c conda-forge -y

echo "📚 Installing tactile-teleop-sdk..."
# Check if tactile-teleop-python-sdk exists
if [ -d "../tactile-teleop-python-sdk" ]; then
    pip install -e ../tactile-teleop-python-sdk
elif [ -d "../../tactile-teleop-python-sdk" ]; then
    pip install -e ../../tactile-teleop-python-sdk
else
    echo "⚠️  tactile-teleop-python-sdk not found in expected locations"
    echo "   Please install it manually: pip install -e /path/to/tactile-teleop-python-sdk"
fi

echo "📚 Installing piper-robot-server..."
pip install -e .

echo "📚 Installing PyBullet for simulation..."
pip install pybullet

echo "🔍 Verifying installation..."
python -c "import pinocchio; print('✅ Pinocchio:', pinocchio.__version__)"
python -c "import casadi; print('✅ CasADi:', casadi.__version__)"
python -c "import pybullet; print('✅ PyBullet installed')"
python -c "import piper_teleop; print('✅ piper_teleop installed')"

echo ""
echo "✅ Setup complete!"
echo ""
echo "To activate the environment, run:"
echo "  conda activate piper"
echo ""
echo "To test the simulation, run:"
echo "  python test_simulation.py"
echo ""
echo "To run the robot server in simulation mode:"
echo "  robotserver --sim --vis --keyboard"
