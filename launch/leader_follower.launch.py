from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        
        Node(
            package='turtle_follower',
            executable='teleop_leader',
            name='teleop_leader',
            output='screen'
        ),
        Node(
            package='turtle_follower',
            executable='turtle_spawner',
            name='turtle_spawner',
            output='screen'
        ),

        
        Node(
            package='turtle_follower',
            executable='follower',
            name='follower',
            output='screen'
        ),
    ])
