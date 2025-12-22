# ROS 2 Humble Environment Setup

Complete guide for setting up a ROS 2 Humble development environment.

## System Requirements

- **OS**: Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- **RAM**: Minimum 4GB (8GB recommended)
- **Disk Space**: At least 10GB free
- **Python**: 3.10 or higher

## Installation Steps

### 1. Add ROS 2 Repository

```bash
sudo apt update
sudo apt install curl gnupg lsb-release ubuntu-keyring

curl -sSL https://raw.githubusercontent.com/ros/ros.key | sudo apt-key add -

sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main"
```

### 2. Install ROS 2 Humble

```bash
sudo apt update
sudo apt install ros-humble-desktop
# or for minimal installation:
sudo apt install ros-humble-ros-core
```

### 3. Install Development Tools

```bash
# colcon - build tool
sudo apt install python3-colcon-common-extensions

# Additional tools
sudo apt install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
```

### 4. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### 5. Set Up Shell Environment

Add these lines to your `~/.bashrc` or `~/.zshrc`:

```bash
# ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Optional: Replace 'bash' with 'zsh' if using zsh
# source /opt/ros/humble/setup.zsh
```

Then reload your shell:

```bash
source ~/.bashrc
```

## Project-Specific Setup

### Clone the Repository

```bash
git clone https://github.com/harsh-pandhe/ROS.git ros2_humble_ws
cd ros2_humble_ws
```

### Install Package Dependencies

```bash
# Install dependencies listed in package.xml files
rosdep install --from-paths src --ignore-src -r -y
```

### Build the Workspace

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build
colcon build

# Source workspace setup
source install/setup.bash
```

## Verification

### Verify ROS 2 Installation

```bash
ros2 --version
ros2 topic list  # Should show some internal ROS 2 topics
```

### Verify Workspace Build

```bash
# List available packages
ros2 pkg list | grep my_first_package

# Should output something like:
# my_first_package
```

## Testing the Turtle Control Node

### Terminal 1: Start TurtleSim

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

### Terminal 2: Run the Turtle Control Node

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_first_package my_test_node
```

The turtle should start moving in a circular pattern.

## Useful ROS 2 Commands

### Topic Operations

```bash
# List all topics
ros2 topic list

# Show topic type
ros2 topic info /turtle1/cmd_vel

# Echo topic messages
ros2 topic echo /turtle1/cmd_vel

# Publish to a topic
ros2 topic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### Node Operations

```bash
# List running nodes
ros2 node list

# Get info about a node
ros2 node info /turtle_circle_node
```

### Package Operations

```bash
# List all packages
ros2 pkg list

# Find a package
ros2 pkg find my_first_package

# Show package info
ros2 pkg info my_first_package
```

### Build Operations

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_first_package

# Build with verbose output
colcon build --event-handlers console_direct+

# Clean build artifacts
rm -rf build install log
colcon build
```

## Troubleshooting

### "ros2" command not found

**Solution**: Make sure ROS 2 is sourced:
```bash
source /opt/ros/humble/setup.bash
```

Add it to your `~/.bashrc` to make it permanent.

### Build fails with dependency errors

**Solution**: Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### TurtleSim window doesn't appear

**Solution**: Install TurtleSim:
```bash
sudo apt install ros-humble-turtlesim
```

### Python import errors

**Solution**: Make sure workspace is sourced:
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_humble_ws
source install/setup.bash
```

### Port already in use error

**Solution**: Another ROS 2 process might be running. Stop it:
```bash
killall -9 ros2
killall -9 turtlesim
```

## Performance Tips

1. **Use specific colcon builds** instead of building everything:
   ```bash
   colcon build --packages-select my_package_name
   ```

2. **Enable parallel builds** (if you have multiple cores):
   ```bash
   colcon build --parallel-workers $(nproc)
   ```

3. **Use verbose output only when debugging**:
   ```bash
   colcon build --event-handlers console_direct+
   ```

## IDE Setup

### VS Code with ROS 2

1. Install the **C++** extension
2. Install the **Python** extension
3. Install the **ROS** extension (ms-iot.vscode-ros)
4. Configure workspace ROS distro in settings

### PyCharm Professional Edition

1. Open the workspace folder
2. Configure Python interpreter to use the ROS 2 environment
3. Mark `src` folder as Sources Root

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [TurtleSim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

## Next Steps

1. Complete the ROS 2 tutorials
2. Create your own packages
3. Experiment with different message types
4. Join the ROS community!
