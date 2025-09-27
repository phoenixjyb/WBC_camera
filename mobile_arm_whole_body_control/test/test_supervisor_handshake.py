#!/usr/bin/env python3
import os
import time
import unittest
import socket

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from mobile_arm_whole_body_interfaces.msg import CameraPoseTarget, TrackingPhaseState
LOG_DIR = os.path.join(os.getcwd(), 'ros_test_logs')
os.makedirs(LOG_DIR, exist_ok=True)
os.environ.setdefault('ROS_LOG_DIR', LOG_DIR)


def _has_udp_privileges() -> bool:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.close()
        return True
    except PermissionError:
        return False


NETWORK_AVAILABLE = _has_udp_privileges()


@pytest.mark.launch_test
def generate_test_description():
    if not NETWORK_AVAILABLE:
        ld = launch.LaunchDescription([
            launch_testing.actions.ReadyToTest(),
        ])
        return ld, {}

    ingest_node = launch_ros.actions.Node(
        package='mobile_arm_whole_body_control',
        executable='trajectory_ingestor_node',
        output='screen'
    )

    supervisor_node = launch_ros.actions.Node(
        package='mobile_arm_whole_body_control',
        executable='whole_body_supervisor_node',
        output='screen',
    )

    planner_stub_node = launch_ros.actions.Node(
        package='mobile_arm_whole_body_control',
        executable='planner_stub_node',
        output='screen'
    )

    ld = launch.LaunchDescription([
        ingest_node,
        supervisor_node,
        planner_stub_node,
        launch_testing.actions.ReadyToTest(),
    ])

    return ld, {}


@unittest.skipIf(not NETWORK_AVAILABLE, 'Supervisor handshake test requires UDP socket permissions')
class TestSupervisorHandshake(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = RclpyNode('handshake_test_node')
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.state_messages = []
        self.state_sub = self.node.create_subscription(
            TrackingPhaseState,
            'whole_body/state',
            self.state_messages.append,
            qos)

        self.target_pub = self.node.create_publisher(
            CameraPoseTarget,
            'camera_path/target',
            qos)

    def tearDown(self):
        self.state_sub.destroy()
        self.target_pub.destroy()
        self.node.destroy_node()

    def _spin_for(self, duration_sec: float):
        end_time = time.monotonic() + duration_sec
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_state_transitions(self):
        target = CameraPoseTarget()
        target.id = 0
        target.position_tolerance = 0.01
        target.orientation_tolerance = 0.01

        self.target_pub.publish(target)

        self._spin_for(1.0)
        modes = {msg.mode for msg in self.state_messages}
        self.assertIn(TrackingPhaseState.MODE_ARM_RAMP, modes, 'ARM_RAMP state not observed')

        self._spin_for(1.0)
        modes = {msg.mode for msg in self.state_messages}
        self.assertIn(TrackingPhaseState.MODE_BASE_RAMP, modes, 'BASE_RAMP state not observed')

        self._spin_for(1.0)
        modes = {msg.mode for msg in self.state_messages}
        self.assertIn(TrackingPhaseState.MODE_TRACKING, modes, 'TRACKING state not observed')
        self.assertIn(TrackingPhaseState.MODE_IDLE, modes, 'IDLE state not observed after completion')


if __name__ == '__main__':
    pytest.main()
