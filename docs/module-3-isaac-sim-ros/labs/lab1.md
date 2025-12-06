# Lab 1: Install Isaac Sim on RTX Workstation

## Objective
Set up the NVIDIA Isaac Sim platform for advanced robotics simulation.

## Prerequisites
- NVIDIA RTX GPU (RTX 3080 or better recommended)
- Ubuntu 22.04 LTS
- NVIDIA GPU drivers (535 or later)
- CUDA 11.8 or later
- 32GB+ RAM recommended
- 100GB+ free disk space

## Steps

### 1. Verify System Requirements
First, check that your system meets the requirements:

```bash
# Check GPU
nvidia-smi

# Check CUDA version
nvcc --version

# Check available disk space
df -h

# Check system memory
free -h
```

### 2. Install NVIDIA GPU Drivers
If not already installed:

```bash
# Update system
sudo apt update
sudo apt upgrade -y

# Install NVIDIA drivers
sudo apt install nvidia-driver-535 nvidia-utils-535

# Reboot system
sudo reboot
```

### 3. Install CUDA Toolkit
```bash
# Download CUDA 11.8 from NVIDIA website or use package manager
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda_11.8.0_520.61.05_linux.run

# Make executable and run
chmod +x cuda_11.8.0_520.61.05_linux.run
sudo sh cuda_11.8.0_520.61.05_linux.run
```

Follow the installer prompts:
- Accept the license
- Select "CUDA Toolkit 11.8" and "CUDA Samples 11.8"
- Skip driver installation if already installed
- Add CUDA to PATH (or do it manually in step 4)

### 4. Set up Environment Variables
Add to your `~/.bashrc`:

```bash
# CUDA
export PATH=/usr/local/cuda-11.8/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-11.8/lib64:$LD_LIBRARY_PATH

# Isaac Sim requirements
export ISAACSIM_PATH=$HOME/isaac-sim
```

Then source the file:
```bash
source ~/.bashrc
```

### 5. Install Isaac Sim Dependencies
```bash
# Install system dependencies
sudo apt update
sudo apt install -y build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev libsqlite3-dev wget libbz2-dev

# Install Python 3.10 (required for Isaac Sim)
sudo apt install python3.10 python3.10-dev python3.10-venv python3-pip

# Install additional dependencies
sudo apt install libgl1-mesa-glx libglib2.0-0 libsm6 libxext6 libxrender-dev libgomp1
```

### 6. Download and Install Isaac Sim
Visit the NVIDIA Omniverse Isaac Sim download page to get the latest version. For this example, we'll use the App launcher method:

```bash
# Download Omniverse App Launcher
wget https://developer.nvidia.com/downloads/remaken/omniverse-app-launcher-linux.AppImage

# Make executable
chmod +x omniverse-app-launcher-linux.AppImage

# Run the launcher (this will download and manage Isaac Sim)
./omniverse-app-launcher-linux.AppImage
```

### 7. Alternative: Install Isaac Sim via pip (for development)
```bash
# Create a Python virtual environment
python3.10 -m venv isaac-sim-env
source isaac-sim-env/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install Isaac Sim Python wheel (get the latest from NVIDIA developer site)
# Note: This is for development purposes; the App Launcher is recommended for most users
pip install --extra-index-url https://pypi.ngc.nvidia.com omni-isaac-sim
```

### 8. Verify Installation
After installing via App Launcher:

1. Launch Isaac Sim from the Omniverse App Launcher
2. Accept the license agreement
3. Complete the initial setup

Or if installed via pip:
```bash
# Activate environment
source isaac-sim-env/bin/activate

# Run Isaac Sim
python -m omni.isaac.kit --enable-extensions
```

### 9. Install Isaac ROS Dependencies
```bash
# Install Isaac ROS dependencies
sudo apt install ros-humble-isaac-ros-common

# Install additional packages for perception
sudo apt install ros-humble-isaac-ros-visual-slam ros-humble-isaac-ros-point-cloud-transport
```

### 10. Create Isaac Sim Workspace
```bash
mkdir -p ~/isaac_sim_ws
cd ~/isaac_sim_ws

# Create a basic project structure
mkdir -p projects/scenes
mkdir -p projects/robots
mkdir -p projects/scripts
mkdir -p projects/assets
```

### 11. Test Isaac Sim Installation
Create a simple test script `test_isaac.py`:

```python
# This script tests basic Isaac Sim functionality
import omni
from omni.isaac.kit import SimulationApp

# Start Isaac Sim
config = {
    'headless': False,  # Set to True for headless operation
    'rendering_updates': True,
    'physics_updates': True,
}
simulation_app = SimulationApp(config)

# Import Isaac Sim modules
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path

# Create world
world = World(stage_units_in_meters=1.0)

# Add a simple cube
from omni.isaac.core.utils.prims import create_primitive
create_primitive(
    prim_path="/World/Cube",
    primitive_type="Cube",
    position=[0, 0, 1.0],
    orientation=[0, 0, 0, 1],
    scale=[0.1, 0.1, 0.1]
)

# Reset and step the world
world.reset()
for i in range(100):
    world.step(render=True)

# Shutdown
simulation_app.close()
```

Run the test:
```bash
source isaac-sim-env/bin/activate
python test_isaac.py
```

### 12. Configure Isaac Sim for Optimal Performance
Create `~/.ovrc` file for configuration:

```
[App]
IsaacSim.app.renderer=Vulkan
IsaacSim.app.gpuAffinity=0
IsaacSim.app.enableAudio=false
IsaacSim.app.showConsoleWindow=true

[Physics]
IsaacSim.app.physics.engine=PhysX
IsaacSim.app.physics.gravity=-9.81
IsaacSim.app.physics.substeps=1
IsaacSim.app.physics.solverType=TGS
```

## Expected Output
- Isaac Sim launches successfully
- Test script runs without errors
- GPU acceleration is properly configured
- Isaac Sim interface is responsive

## Troubleshooting
- If Isaac Sim doesn't launch, check GPU drivers and CUDA installation
- If you get OpenGL errors, ensure proper graphics drivers are installed
- If performance is poor, verify that the GPU is properly recognized and used
- For Python import errors, ensure the virtual environment is activated

## Next Steps
- Learn to create basic scenes in Isaac Sim
- Import robot models and configure them
- Set up sensors and perception systems
- Integrate with ROS 2 for robot control