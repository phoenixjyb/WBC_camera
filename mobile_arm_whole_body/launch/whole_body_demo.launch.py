from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path, PackageNotFoundError
from moveit_configs_utils import MoveItConfigsBuilder

import yaml


def load_yaml(path):
    with open(path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    pkg = get_package_share_path('mobile_arm_whole_body')

    moveit_config = (
        MoveItConfigsBuilder('mobile_arm', package_name='mobile_arm_whole_body')
        .robot_description(file_path='urdf/arm_on_car_center_rotZ_n90_center.urdf')
        .robot_description_semantic(file_path='srdf/arm_on_car_center_whole_body.srdf')
        .to_moveit_configs()
    )

    planning_yaml = load_yaml(pkg / 'config' / 'planning_pipelines.yaml')
    moveit_config.planning_pipelines.update(planning_yaml)

    kinematics_yaml = load_yaml(pkg / 'config' / 'kinematics.yaml')
    moveit_config.robot_description_kinematics.update(kinematics_yaml)

    trajectory_execution_file = pkg / 'config' / 'trajectory_execution.yaml'
    if trajectory_execution_file.exists():
        trajectory_execution_yaml = load_yaml(trajectory_execution_file)
        moveit_config.trajectory_execution.update(trajectory_execution_yaml)

    joint_limits_file = pkg / 'config' / 'joint_limits.yaml'
    if joint_limits_file.exists():
        joint_limits_yaml = load_yaml(joint_limits_file)
        moveit_config.joint_limits.update(joint_limits_yaml)

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz window')
    rviz_condition = IfCondition(LaunchConfiguration('rviz'))

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[moveit_config.robot_description]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','odom','chassis_center_link']
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_config.to_dict(),
                    {'publish_robot_description_semantic': True},
                    {'allow_trajectory_execution': True}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(pkg / 'config' / 'moveit.rviz')],
        output='screen',
        condition=rviz_condition
    )

    extra_nodes = []
    try:
        get_package_share_path('joint_state_publisher')
        jsp = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        )
        extra_nodes.append(jsp)
    except PackageNotFoundError:
        pass

    return LaunchDescription([rviz_arg, rsp, static_tf, move_group, rviz, *extra_nodes])
