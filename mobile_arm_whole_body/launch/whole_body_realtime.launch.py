from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_arm_whole_body',
            executable='whole_body_input_node',
            name='whole_body_input',
            parameters=[{'buffer_horizon': 2.0}],
            output='screen'
        ),
        Node(
            package='mobile_arm_whole_body',
            executable='whole_body_coordinator_node',
            name='whole_body_coordinator',
            output='screen'
        ),
        Node(
            package='mobile_arm_whole_body',
            executable='whole_body_ramp_planner_node',
            name='whole_body_ramp_planner',
            output='screen'
        )
    ])
