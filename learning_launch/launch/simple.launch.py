from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace= "turtlesim1", 
            package='turtlesim', 
            executable='turtlesim_node', 
            output='screen'
        ),
        Node(
            namespace= "turtlesim2", 
            package='turtlesim', 
            executable='turtlesim_node', 
            output='screen'
        ),
    ])
