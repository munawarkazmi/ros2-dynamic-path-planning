#!/usr/bin/env python3
"""Integration test for the Nav2 plugin boundary.

The unit suites in core/tests cover the planning algorithms, but they run
against the ROS-free core. Everything between that core and Nav2 was
previously untested: whether pluginlib can find and load the class, whether
configure() and activate() succeed inside a lifecycle node, whether the
costmap is read with the right resolution and origin, and whether the path
that comes back out of the ComputePathToPose action is well formed in the
right frame. Those are exactly the failures that a green unit suite cannot
catch and that break a Nav2 plugin in the field.

The costmap here deliberately carries only an inflation layer. With no static
layer there is no map server to wait on and no sensor topics to fake, so the
test is deterministic: an empty 10 by 10 metre costmap in which any route
between two free cells exists. What is under test is the wiring, not the
search, and the search already has 81 unit cases of its own.

Run:  colcon test --packages-select ros2_dynamic_path_planning
"""

import os
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

# The plugin under test. Both planners implement nav2_core::GlobalPlanner
# through the same adapter path, so either can be substituted here; D* Lite
# is the default because it is the one config/nav2_params.yaml ships.
PLANNER_PLUGIN = os.environ.get(
    "PLANNER_PLUGIN_UNDER_TEST", "ros2_dynamic_path_planning/DStarLitePlanner"
)

GLOBAL_FRAME = "map"
ROBOT_FRAME = "base_link"

START_XY = (1.0, 1.0)
GOAL_XY = (8.0, 8.0)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    planner_params = {
        "use_sim_time": False,
        "expected_planner_frequency": 1.0,
        "planner_plugins": ["GridBased"],
        "GridBased.plugin": PLANNER_PLUGIN,
        # An inflation-only costmap needs neither a map server nor sensors.
        "global_costmap.global_costmap.ros__parameters.update_frequency": 2.0,
        "global_costmap.global_costmap.ros__parameters.publish_frequency": 2.0,
        "global_costmap.global_costmap.ros__parameters.global_frame": GLOBAL_FRAME,
        "global_costmap.global_costmap.ros__parameters.robot_base_frame": ROBOT_FRAME,
        "global_costmap.global_costmap.ros__parameters.rolling_window": False,
        "global_costmap.global_costmap.ros__parameters.width": 10,
        "global_costmap.global_costmap.ros__parameters.height": 10,
        "global_costmap.global_costmap.ros__parameters.resolution": 0.1,
        "global_costmap.global_costmap.ros__parameters.origin_x": 0.0,
        "global_costmap.global_costmap.ros__parameters.origin_y": 0.0,
        "global_costmap.global_costmap.ros__parameters.plugins": ["inflation_layer"],
        "global_costmap.global_costmap.ros__parameters.inflation_layer.plugin":
            "nav2_costmap_2d::InflationLayer",
        "global_costmap.global_costmap.ros__parameters.inflation_layer.inflation_radius": 0.2,
        "global_costmap.global_costmap.ros__parameters.inflation_layer.cost_scaling_factor": 3.0,
        "global_costmap.global_costmap.ros__parameters.always_send_full_costmap": True,
    }

    return launch.LaunchDescription(
        [
            # The costmap needs a transform chain from its global frame down to
            # the robot frame in order to update. Identity transforms are enough
            # because nothing here moves.
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", GLOBAL_FRAME, "odom"],
                output="screen",
            ),
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "odom", ROBOT_FRAME],
                output="screen",
            ),
            launch_ros.actions.Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[planner_params],
            ),
            # autostart drives planner_server through configure and activate,
            # so a plugin that fails to load shows up as a lifecycle failure
            # rather than as a silent absence.
            launch_ros.actions.Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_test",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": True,
                        "node_names": ["planner_server"],
                        "bond_timeout": 0.0,
                    }
                ],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _pose(x, y):
    pose = PoseStamped()
    pose.header.frame_id = GLOBAL_FRAME
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose


class TestPlannerServerServesAPath(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("planner_boundary_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_compute_path_to_pose_returns_a_usable_path(self, proc_output):
        client = ActionClient(self.node, ComputePathToPose, "compute_path_to_pose")

        # Generous: the server has to configure, activate and build a costmap
        # before it will advertise the action.
        self.assertTrue(
            client.wait_for_server(timeout_sec=60.0),
            "compute_path_to_pose action server never appeared, so planner_server "
            "did not reach the active state",
        )

        goal = ComputePathToPose.Goal()
        goal.start = _pose(*START_XY)
        goal.goal = _pose(*GOAL_XY)
        goal.planner_id = "GridBased"
        goal.use_start = True  # avoids depending on a localised robot pose

        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        self.assertIsNotNone(handle, "no response to the goal request")
        self.assertTrue(handle.accepted, "planner_server rejected the goal")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=30.0)
        outcome = result_future.result()
        self.assertIsNotNone(outcome, "planner_server never returned a result")
        self.assertEqual(
            outcome.status,
            GoalStatus.STATUS_SUCCEEDED,
            f"planning did not succeed, status {outcome.status}",
        )

        path = outcome.result.path
        self.assertGreater(len(path.poses), 1, "path has fewer than two poses")
        self.assertEqual(
            path.header.frame_id,
            GLOBAL_FRAME,
            "path came back in the wrong frame, which would silently break Nav2",
        )

        # Endpoints, to costmap resolution. The unit suite already proves the
        # path is optimal; what matters here is that the adapter did not drop,
        # swap or offset the ends on the way through.
        tolerance = 0.25
        first, last = path.poses[0].pose.position, path.poses[-1].pose.position
        self.assertAlmostEqual(first.x, START_XY[0], delta=tolerance)
        self.assertAlmostEqual(first.y, START_XY[1], delta=tolerance)
        self.assertAlmostEqual(last.x, GOAL_XY[0], delta=tolerance)
        self.assertAlmostEqual(last.y, GOAL_XY[1], delta=tolerance)

    def test_plugin_loaded_without_error(self, proc_output):
        # pluginlib reports a missing or misdeclared class here, and the message
        # is far more useful than the lifecycle failure it causes downstream.
        combined = "".join(
            event.text.decode("utf-8", errors="replace") for event in proc_output
        )
        for forbidden in ("Failed to create global planner", "does not exist"):
            self.assertNotIn(
                forbidden,
                combined,
                f"planner_server output contains {forbidden!r}",
            )


@launch_testing.post_shutdown_test()
class TestNoCrashOnShutdown(unittest.TestCase):
    def test_planner_server_exited_cleanly(self, proc_info):
        # A plugin that corrupts state on deactivate shows up as a non-zero or
        # signal exit here and nowhere else.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, launch_testing.asserts.EXIT_SIGINT],
        )
