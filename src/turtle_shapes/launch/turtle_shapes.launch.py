from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Launches the Turtlesim GUI
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # 2. Launches the Turtle drawing logic (Subscriber)
        Node(
            package='turtle_shapes',
            executable='turtle_commander',
            name='turtle_commander',
            output='screen' # Use 'log' or 'screen' for output
        )
    ])