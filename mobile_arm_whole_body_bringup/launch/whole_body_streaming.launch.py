from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

import yaml


def load_yaml(path):
    with open(path, 'r') as file:
        return yaml.safe_load(file)


def generate_launch_description():
    mobile_arm_pkg = get_package_share_directory('mobile_arm_whole_body')

    moveit_config = (
        MoveItConfigsBuilder('mobile_arm', package_name='mobile_arm_whole_body')
        .robot_description(file_path='urdf/arm_on_car_center_rotZ_n90_center.urdf')
        .robot_description_semantic(file_path='srdf/arm_on_car_center_whole_body.srdf')
        .to_moveit_configs()
    )

    planning_yaml = load_yaml(f"{mobile_arm_pkg}/config/planning_pipelines.yaml")
    moveit_config.planning_pipelines.update(planning_yaml)

    kinematics_yaml = load_yaml(f"{mobile_arm_pkg}/config/kinematics.yaml")
    moveit_config.robot_description_kinematics.update(kinematics_yaml)

    joint_limits_yaml = load_yaml(f"{mobile_arm_pkg}/config/joint_limits.yaml")
    moveit_config.joint_limits.update(joint_limits_yaml)

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(f"{mobile_arm_pkg}/launch/whole_body_demo.launch.py"),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()
    )

    trajectory_ingestor = Node(
        package='mobile_arm_whole_body_control',
        executable='trajectory_ingestor_node',
        name='trajectory_ingestor'
    )

    supervisor = Node(
        package='mobile_arm_whole_body_control',
        executable='whole_body_supervisor_node',
        name='whole_body_supervisor'
    )

    arm_publisher = Node(
        package='mobile_arm_whole_body_control',
        executable='arm_trajectory_publisher_node',
        name='arm_trajectory_publisher'
    )

    base_commander = Node(
        package='mobile_arm_whole_body_control',
        executable='base_motion_commander_node',
        name='base_motion_commander'
    )

    planner = Node(
        package='mobile_arm_whole_body_control',
        executable='whole_body_planner_node',
        name='whole_body_planner',
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines
        ]
    )

    base_planner = Node(
        package='mobile_arm_whole_body_control',
        executable='base_planner_node',
        name='base_planner',
        parameters=[{
            'map_resolution': 0.05,
            'map_width': 200,
            'map_height': 200,
            'map_origin': [-5.0, -5.0],
            'static_disc_obstacles': [
                0.053595, 0.113448, 0.1, 0.1,
                -1.0, -1.0, 0.1, 0.1
            ],
            'obstacle_threshold': 50.0,
            'step_resolution': 0.05,
        }]
    )

    obstacle_manager = Node(
        package='mobile_arm_whole_body_control',
        executable='obstacle_manager_node',
        name='obstacle_manager',
        parameters=[{
            'map_resolution': 0.05,
            'map_width': 200,
            'map_height': 200,
            'map_origin': [-5.0, -5.0],
            'disc_obstacles': [
                0.053595, 0.113448, 0.1, 0.1,
                -1.0, -1.0, 0.1, 0.1
            ],
            'map_frame': 'odom'
        }]
    )

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz monitor')
    record_bag_arg = DeclareLaunchArgument('record_bag', default_value='false', description='Record rosbag of core topics')

    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', 'whole_body_demo',
             '/whole_body/arm_plan',
             '/whole_body/base_plan',
             '/whole_body/state',
             '/whole_body/cmd_vel',
             '/whole_body/obstacle_grid'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('record_bag'))
    )

    return LaunchDescription([
        rviz_arg,
        record_bag_arg,
        moveit_demo,
        trajectory_ingestor,
        supervisor,
        arm_publisher,
        base_commander,
        planner,
        base_planner,
        obstacle_manager,
        TimerAction(period=5.0, actions=[bag_record])
    ])
