# ROS 2 Humble Workspace

A ROS 2 workspace for learning and developing ROS 2 applications using Humble distribution.

## Overview

This repository contains a ROS 2 workspace with packages designed for robotics learning and experimentation. It includes sample nodes that interact with the TurtleSim simulator and demonstrates fundamental ROS 2 concepts.

## Repository Structure

```
ros2_humble_ws/
├── README.md                    # This file
├── src/                         # ROS 2 source packages
│   └── my_first_package/        # Sample package with turtle control
├── build/                       # Build artifacts (generated)
├── install/                     # Installation directory (generated)
├── log/                         # Build logs (generated)
└── .git/                        # Git repository metadata
```

## Packages

### [my_first_package](src/my_first_package/README.md)

A beginner-friendly ROS 2 package that demonstrates basic concepts:

- **TurtleCircle Node**: Controls the TurtleSim turtle with circular motion
- **Published Topics**: `/turtle1/cmd_vel` (Twist messages)
- **Purpose**: Learning ROS 2 node creation, publishers, and message handling

See [my_first_package README](src/my_first_package/README.md) for detailed documentation.

## Prerequisites

- **ROS 2 Humble** - Full installation or minimal development setup
- **Python 3.10+**
- **colcon** - Build tool for ROS 2
- **TurtleSim** - For testing turtle control nodes

### Installation

Install ROS 2 Humble and required packages:

```bash
# Install ROS 2 Humble (follow official ROS 2 installation guide)
# Install colcon
sudo apt install python3-colcon-common-extensions

# Install additional dependencies
sudo apt install ros-humble-turtlesim ros-humble-geometry-msgs
```

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/harsh-pandhe/ROS.git
cd ros2_humble_ws
```

### 2. Source ROS 2

```bash
source /opt/ros/humble/setup.bash
```

### 3. Build the Workspace

```bash
colcon build
```

Or build a specific package:

```bash
colcon build --packages-select my_first_package
```

### 4. Source the Workspace

```bash
source install/setup.bash
```

### 5. Run a Node

#### Start TurtleSim:
```bash
ros2 run turtlesim turtlesim_node
```

#### Run the TurtleCircle Node (in another terminal):
```bash
source install/setup.bash
ros2 run my_first_package my_test_node
```

The turtle will move in a circular pattern.

## Common Commands

### Build Commands

```bash
# Build all packages
colcon build

# Build a specific package
colcon build --packages-select my_first_package

# Build with verbose output
colcon build --event-handlers console_direct+

# Clean build artifacts
rm -rf build install log
```

### Running Nodes

```bash
# List available packages
ros2 pkg list

# Run a node
ros2 run <package_name> <node_name>

# List topics
ros2 topic list

# Echo a topic
ros2 topic echo <topic_name>

# Show message type
ros2 topic info <topic_name>
```

### Testing

```bash
# Run tests for all packages
colcon test

# Run tests for a specific package
colcon test --packages-select my_first_package
```

## Workspace Layout

### Source Directory (`src/`)
Contains ROS 2 package source code. Each subdirectory is a package with its own:
- `package.xml` - Package metadata and dependencies
- `setup.py` - Python package configuration
- Source code files

### Build Directory (`build/`)
Generated during the build process. Contains:
- Compiled binaries (if applicable)
- CMake/setup configuration files
- Build logs

### Install Directory (`install/`)
Generated during the build process. Contains:
- Installed executables and libraries
- Environment setup scripts

### Log Directory (`log/`)
Contains build and test logs.

## Development Workflow

1. **Modify code** in `src/` directory
2. **Build** with `colcon build`
3. **Source** the workspace: `source install/setup.bash`
4. **Run and test** your nodes
5. **Commit and push** changes to git

## ROS 2 Resources

- [ROS 2 Official Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Humble Release](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)
- [TurtleSim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [Writing a Simple Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

## Troubleshooting

### Build Fails
- Make sure ROS 2 Humble is sourced: `source /opt/ros/humble/setup.bash`
- Clean and rebuild: `rm -rf build install log && colcon build`
- Check for missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`

### Node Not Found
- Ensure the workspace is sourced: `source install/setup.bash`
- Verify the package was built successfully: `ros2 pkg list`

### Import Errors
- Install missing Python dependencies using `apt` or `pip`
- Rebuild the package: `colcon build --packages-select <package_name>`

### TurtleSim Issues
- Verify TurtleSim is installed: `sudo apt install ros-humble-turtlesim`
- Make sure the node is still running when executing turtle control nodes

## Contributing

To contribute to this workspace:

1. Create a new branch: `git checkout -b feature/your-feature`
2. Make changes and test thoroughly
3. Commit with descriptive messages: `git commit -m "Description of changes"`
4. Push to the repository: `git push origin feature/your-feature`
5. Create a pull request

## License

This workspace is provided for educational purposes.

## Contact & Support

For issues, questions, or contributions, please refer to the repository on GitHub.

---

**Last Updated**: December 22, 2025  
**ROS 2 Distribution**: Humble  
**Python Version**: 3.10+
