# My First Package

A simple ROS 2 package that demonstrates basic node creation and publisher functionality using the TurtleSim simulator.

## Overview

This package contains a ROS 2 node (`my_test_node`) that controls a turtle in TurtleSim by publishing velocity commands. The turtle moves in a circular motion by combining linear and angular velocity.

## Features

- **TurtleCircle Node**: A ROS 2 node that publishes `Twist` messages to control turtle movement
- **Circular Motion**: The turtle moves forward and rotates simultaneously to create a circular path
- **Timer-based Publishing**: Uses a timer callback to publish velocity commands at regular intervals (0.1 seconds)

## Requirements

- ROS 2 Humble (or compatible version)
- `geometry_msgs` package
- `turtlesim` package

## Installation

1. Clone or copy this package into your ROS 2 workspace's `src` directory:
   ```bash
   cd ~/ros2_humble_ws/src
   ```

2. Build the package:
   ```bash
   cd ~/ros2_humble_ws
   colcon build --packages-select my_first_package
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

## Usage

### Run the TurtleCircle Node

1. **Start TurtleSim**:
   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. **Run the TurtleCircle Node** (in a new terminal):
   ```bash
   source install/setup.bash
   ros2 run my_first_package my_test_node
   ```

The turtle will move in a circular pattern as the node publishes velocity commands.

## Node Details

### TurtleCircle Node
- **Node Name**: `turtle_circle_node`
- **Published Topic**: `/turtle1/cmd_vel` (Twist messages)
- **Message Type**: `geometry_msgs/Twist`
- **Publish Rate**: 10 Hz (0.1 second timer)

### Velocity Parameters
- **Linear Velocity** (`linear.x`): 3.0 m/s (forward motion)
- **Angular Velocity** (`angular.z`): 1.5 rad/s (rotation)

## File Structure

```
my_first_package/
├── README.md                 # This file
├── package.xml              # Package metadata and dependencies
├── setup.py                 # Python package setup
├── setup.cfg                # Setup configuration
├── my_first_package/
│   ├── __init__.py
│   ├── my_test_node.py      # Main node implementation
│   └── __pycache__/
├── resource/
│   └── my_first_package
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

## Troubleshooting

- **Node not found**: Make sure you've built and sourced the package correctly
- **Turtle doesn't move**: Ensure TurtleSim is running and the topic `/turtle1/cmd_vel` is being published to
- **Import errors**: Verify that `geometry_msgs` is installed

## License

This is a sample ROS 2 learning package.
