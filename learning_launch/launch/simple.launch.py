from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='learning_topic', 
            executable='topic_helloworld_pub', 
        ),
        Node(
            package='learning_topic', 
            executable='topic_helloworld_sub', 
        ),
    ])
