from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) Turtlesim simulation
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # 2) Your bridge: /turtle1/pose → /leader_pose
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

        # 3) Your follower controller (turtle2 follows leader_pose)
        Node(
            package='turtle_follower',
            executable='follower',
            name='follower',
            output='screen'
        ),
    ])
