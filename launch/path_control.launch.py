from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='path_control',
            executable='path_smoother',
            name='path_smoother',
            output='screen'
        ),

        Node(
            package='path_control',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen'
        ),

        Node(
            package='path_control',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen'
        )

    ])

