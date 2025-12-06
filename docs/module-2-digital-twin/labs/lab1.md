# Lab 1: Set up Gazebo on Ubuntu 22.04

## Objective
Install and configure the Gazebo physics simulator.

## Prerequisites
- Ubuntu 22.04 LTS
- Internet connection
- Administrative (sudo) access
- ROS 2 Humble Hawksbill installed

## Steps

### 1. Update system packages
```bash
sudo apt update
sudo apt upgrade -y
```

### 2. Install Gazebo Garden (recommended version for ROS 2 Humble)
```bash
# Add the Gazebo APT repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package list
sudo apt update

# Install Gazebo Garden
sudo apt install gz-garden
```

### 3. Verify Gazebo installation
```bash
gz --version
gz sim --version
```

### 4. Install ROS 2 Gazebo packages
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-dev
```

### 5. Alternative: Install Gazebo Classic (if needed for compatibility)
```bash
sudo apt install ros-humble-gazebo-classic-ros-pkgs
```

### 6. Test Gazebo with a simple world
```bash
gz sim shapes.sdf
```

Or if using Gazebo Classic:
```bash
gazebo
```

### 7. Create a basic Gazebo world file
Create a directory for your Gazebo worlds:
```bash
mkdir -p ~/gazebo_worlds
cd ~/gazebo_worlds
```

Create `simple_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Include a ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a sun for lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add a simple box -->
    <model name="box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Add a simple sphere -->
    <model name="sphere">
      <pose>2 0 1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.3 0.8 0.3 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>0.5</radius>
            </sphere>
          </geometry>
        </collision>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### 8. Launch your custom world
```bash
gz sim simple_world.sdf
```

### 9. Set up Gazebo environment variables (optional)
Add to your `~/.bashrc`:
```bash
export GZ_SIM_RESOURCE_PATH=$HOME/gazebo_worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gz-sim-6/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

Then source your bashrc:
```bash
source ~/.bashrc
```

## Expected Output
- Gazebo should start without errors
- You should see the Gazebo GUI with your custom world loaded
- The box and sphere should be visible in the simulation

## Troubleshooting
- If Gazebo fails to start, check that your graphics drivers are properly installed
- If you get OpenGL errors, ensure your system supports hardware acceleration
- If models don't appear, verify the SDF syntax and file paths

## Next Steps
- Learn to create more complex Gazebo worlds
- Integrate ROS 2 with Gazebo for robot simulation
- Add sensors to your simulated robots