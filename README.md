# High Level Robot Controller

This is a collection of packages for connecting with and simulating a robot a Schunk SVH gripper and ABB IRB1200 based on Drake and uses ROS2 as a messaging system and Colcon as a build system.

## Installation and Setup

Note: Only works on operating systems supported by Drake and ROS2 (Ubuntu 22.04, Ubuntu 24.04); command: wsl --install Ubuntu-22.04

1. Ensure that Drake is installed globally and is on PATH. See the [Drake Installation Page](https://drake.mit.edu/apt.html#stable-releases) for more information.

2. Ensure that ROS2 Humble is installed and has been sourced. See the [ROS2 Humble Installation Page](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) for more information.

```bash
source /opt/ros/humble/setup.bash
```

3. Build the drake-ros from source
```bash
# Create a workspace
cd ~
mkdir -p drake-ros-ws/src
cd drake-ros-ws/src

# Clone the drake-ros repository
git clone https://github.com/robotlocomotion/drake-ros.git
cd ..

# Download all the packages required for building the package using rosdep
# rosdep may have some required setup steps if you haven't used it yet
# If so the following command should guide you through the required setup steps
# before install the dependance initialize your rosdep and Recommended: please run rosdep update
sudo rosdep init
rosdep update
rosdep install --from-paths src -ryi

# Build the package using colcon with Release build type
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the drake-ros workspace so other packages can reference it
source install/local_setup.bash
```

4. Build the high_level_robot_controller package from source
```bash
# Create a workspace
cd ~
mkdir -p ws/src
cd ws/src

# Clone the high_level_robot_controller repository
git clone https://github.com/xandermaljaars/high_level_robot_controller.git
cd ..

# Download all the packages required for building the package using rosdep.
rosdep install --from-paths src -ryi

# Build the package using colcon
colcon build

# Source the workspace so ros2 commands can find the required packages
source install/local_setup.bash
```
6. Run any of the following applications
```bash
ros2 run simulation simulation
ros2 run abb_ros2_driver abb_ros2_driver
```
