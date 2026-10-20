import time
import unittest

import launch
from launch.actions import EmitEvent, TimerAction
from launch.events.process import ProcessExited
from launch.events import Shutdown
import launch_ros.actions
import launch_testing.actions
import lifecycle_msgs.msg
import lifecycle_msgs.srv
import nav_msgs.msg
import nav2_msgs.action
import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


@pytest.mark.launch_test
def generate_test_description():
    task_node = launch_ros.actions.Node(
        package='airobot_action',
        executable='task_frame_nav_node',
        name='task_frame_nav_regression',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_topic': '/test/Odometry',
            'odom_frame': 'camera_init',
            'body_frame': 'body',
            'task_frame': 'task_frame',
            'capture_delay': 0.3,
            'nav_action_name': 'navigate_to_pose',
            'auto_goal': True,
            'goal_x': 1.0,
            'goal_y': 0.0,
        }],
    )

    return (
        launch.LaunchDescription([
            task_node,
            TimerAction(
                period=5.0,
                actions=[EmitEvent(event=Shutdown(reason='test complete'))],
            ),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'task_node': task_node},
    )


class TestAutomaticGoalKeepsTaskFrameAlive(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('task_frame_nav_regression_fixture')
        self.odom_publisher = self.node.create_publisher(
            nav_msgs.msg.Odometry, '/test/Odometry', 10)
        self.odom_timer = self.node.create_timer(0.05, self._publish_odometry)
        self.dynamic_task_frame_samples = []
        self.static_task_frame_samples = []
        self.nav_goal_attempts = 0
        self.nav_goal_times = []
        self.nav2_active = False
        self.nav2_activation_time = time.monotonic() + 0.8
        self.tf_subscription = self.node.create_subscription(
            TFMessage, '/tf', self._record_dynamic_task_frame, 100)
        static_tf_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.tf_static_subscription = self.node.create_subscription(
            TFMessage,
            '/tf_static',
            self._record_static_task_frame,
            static_tf_qos,
        )
        self.lifecycle_service = self.node.create_service(
            lifecycle_msgs.srv.GetState,
            '/bt_navigator/get_state',
            self._get_nav2_state,
        )
        self.lifecycle_timer = self.node.create_timer(
            0.05, self._update_nav2_state)
        self.nav_server = ActionServer(
            self.node,
            nav2_msgs.action.NavigateToPose,
            'navigate_to_pose',
            self._execute_navigation,
            goal_callback=self._handle_nav_goal,
        )

    def tearDown(self):
        self.nav_server.destroy()
        self.node.destroy_service(self.lifecycle_service)
        self.node.destroy_timer(self.lifecycle_timer)
        self.node.destroy_timer(self.odom_timer)
        self.node.destroy_subscription(self.tf_static_subscription)
        self.node.destroy_subscription(self.tf_subscription)
        self.node.destroy_publisher(self.odom_publisher)
        self.node.destroy_node()

    def _publish_odometry(self):
        message = nav_msgs.msg.Odometry()
        message.header.frame_id = 'camera_init'
        message.child_frame_id = 'body'
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.orientation.w = 1.0
        self.odom_publisher.publish(message)

    def _record_dynamic_task_frame(self, message):
        for transform in message.transforms:
            if (
                transform.header.frame_id == 'camera_init'
                and transform.child_frame_id == 'task_frame'
            ):
                self.dynamic_task_frame_samples.append(time.monotonic())

    def _record_static_task_frame(self, message):
        for transform in message.transforms:
            if (
                transform.header.frame_id == 'camera_init'
                and transform.child_frame_id == 'task_frame'
            ):
                self.static_task_frame_samples.append(time.monotonic())

    def _execute_navigation(self, goal_handle):
        goal_handle.succeed()
        return nav2_msgs.action.NavigateToPose.Result()

    def _handle_nav_goal(self, _goal_request):
        self.nav_goal_attempts += 1
        self.nav_goal_times.append(time.monotonic())
        if self.nav_goal_attempts == 1:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _update_nav2_state(self):
        if time.monotonic() >= self.nav2_activation_time:
            self.nav2_active = True

    def _get_nav2_state(self, _request, response):
        response.current_state.id = (
            lifecycle_msgs.msg.State.PRIMARY_STATE_ACTIVE
            if self.nav2_active
            else lifecycle_msgs.msg.State.PRIMARY_STATE_INACTIVE
        )
        response.current_state.label = 'active' if self.nav2_active else 'inactive'
        return response

    def test_locked_task_frame_is_latched_static_tf(self, task_node, proc_info):
        proc_info.assertWaitForStartup(task_node, timeout=5)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertGreaterEqual(
            len(self.static_task_frame_samples),
            1,
            'locked task_frame was not published on /tf_static',
        )
        self.assertEqual(
            len(self.dynamic_task_frame_samples),
            0,
            'locked task_frame was incorrectly published as time-limited dynamic TF',
        )
        self.assertGreaterEqual(
            self.nav_goal_attempts,
            2,
            'automatic goal was not retried after Nav2 rejected the first request',
        )
        self.assertTrue(
            self.nav_goal_times,
            'automatic goal was never sent to Nav2',
        )
        self.assertTrue(
            any(goal_time >= self.nav2_activation_time for goal_time in self.nav_goal_times),
            'automatic goal was not retried after Nav2 reached the active lifecycle state',
        )
        self.assertFalse(
            isinstance(proc_info[task_node], ProcessExited),
            'task_frame_nav_node exited while processing the automatic goal',
        )
