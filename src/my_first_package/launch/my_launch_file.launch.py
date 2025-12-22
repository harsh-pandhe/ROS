from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Starts the TurtleSim Window
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        # Starts your custom Python script
        Node(
            package='my_first_package',
            executable='my_test_node',
            name='controller'
        )
    ])