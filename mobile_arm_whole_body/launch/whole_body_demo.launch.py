from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    pkg = get_package_share_path('mobile_arm_whole_body')

    moveit_config = (
        MoveItConfigsBuilder('mobile_arm', package_name='mobile_arm_whole_body')
        .robot_description(file_path='urdf/arm_on_car_center_rotZ_n90_center.urdf')
        .robot_description_semantic(file_path='srdf/arm_on_car_center_whole_body.srdf')
        .planning_pipelines(file_path='config/planning_pipelines.yaml')
        .kinematics(file_path='config/kinematics.yaml')
        .to_moveit_configs()
    )

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
        output='screen'
    )

    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    return LaunchDescription([rsp, static_tf, move_group, rviz, jsp])
