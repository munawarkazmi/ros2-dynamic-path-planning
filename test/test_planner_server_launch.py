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
import time
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
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

LIFECYCLE_MANAGER = "lifecycle_manager_test"
COSTMAP_TOPIC = "/global_costmap/costmap"

# The plugin under test. Both planners implement nav2_core::GlobalPlanner
# through the same adapter path, so either can be substituted here; D* Lite
# is the default because it is the one config/nav2_params.yaml ships.
PLANNER_PLUGIN = os.environ.get(
    "PLANNER_PLUGIN_UNDER_TEST", "ros2_dynamic_path_planning/DStarLitePlanner"
)

GLOBAL_FRAME = "map"
ROBOT_FRAME = "base_link"

# Both well inside the 10 by 10 metre costmap declared in the params file.
START_XY = (1.0, 1.0)
GOAL_XY = (7.0, 7.0)

# The costmap is a separate node owned by planner_server, so its parameters
# cannot be passed as flat dotted keys on planner_server: they would become
# literal parameter names and the costmap would silently use its own defaults.
# They have to arrive as a properly nested YAML file.
PARAMS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "test_planner_params.yaml")


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
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
                # The YAML carries the nested costmap config; the dict after it
                # overrides only planner_server's own plugin choice, which is a
                # parameter of this node and so is fine as a dotted key.
                parameters=[PARAMS_FILE, {"GridBased.plugin": PLANNER_PLUGIN}],
            ),
            # autostart is off on purpose. With it on, the lifecycle manager
            # races the static transform publishers: activating the costmap is
            # what makes it look up map -> base_link, and on a CI runner that
            # lookup arrived 67 ms after the publishers started, before the
            # transforms had propagated. Any fixed delay only widens that
            # margin; it does not remove the race.
            #
            # Instead the test drives bringup itself, once it has confirmed the
            # transform is actually available. That also makes the plugin
            # failure louder rather than quieter: the STARTUP service returns a
            # boolean, so a plugin that fails to load is a false return here
            # rather than something to be inferred from the log.
            launch_ros.actions.Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name=LIFECYCLE_MANAGER,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "autostart": False,
                        "node_names": ["planner_server"],
                        "bond_timeout": 0.0,
                    }
                ],
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


# nav2's costmap logs 'Invalid frame ID "map" ... frame does not exist'
# when it activates before the static transforms have propagated. That
# substring collides with pluginlib's class-not-found message, which is the
# thing test_plugin_loaded_without_error is actually looking for, so a
# transient startup warning used to fail a test about plugin loading. Drop
# only those lines, and only those: anything else saying "does not exist"
# still fails.
def _without_transform_warnings(text):
    return "\n".join(
        line
        for line in text.splitlines()
        if "Invalid frame ID" not in line and "frame does not exist" not in line
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
        cls.tf_buffer = Buffer()
        cls.tf_listener = TransformListener(cls.tf_buffer, cls.node)
        cls.costmap_seen = False

        # Subscribed before bringup on purpose, for two reasons. nav2 only
        # publishes the costmap when it already has a subscriber, so arriving
        # late can mean waiting a whole publish cycle for the first message.
        # And the publisher is KeepLast(1), transient_local, reliable, so this
        # profile has to match it exactly: requesting transient_local against a
        # volatile publisher is an incompatible pair and delivers nothing at
        # all, which would look like the costmap never publishing.
        cls.node.create_subscription(
            OccupancyGrid,
            COSTMAP_TOPIC,
            cls._on_costmap,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Three waits, in the order the system actually becomes ready. Each one
        # replaces an assumption the previous version of this test was making.
        assert cls._spin_until(
            lambda: cls.tf_buffer.can_transform(GLOBAL_FRAME, ROBOT_FRAME, Time()),
            30.0,
        ), (
            f"{GLOBAL_FRAME} -> {ROBOT_FRAME} never became available, so the "
            "static transform publishers did not come up"
        )

        assert cls._startup(), (
            "the lifecycle manager could not bring planner_server up. With a "
            "transform already available this is the plugin failing to load or "
            "to configure, not a startup race"
        )

        assert cls._spin_until(lambda: cls.costmap_seen, 30.0), (
            f"planner_server reached active but never published {COSTMAP_TOPIC}, "
            "so the costmap never completed an update"
        )

    @classmethod
    def _on_costmap(cls, _msg):
        cls.costmap_seen = True

    @classmethod
    def _spin_until(cls, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    @classmethod
    def _startup(cls, timeout_sec=60.0):
        """Ask the lifecycle manager to configure and activate, and report."""
        client = cls.node.create_client(
            ManageLifecycleNodes, f"/{LIFECYCLE_MANAGER}/manage_nodes"
        )
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = client.call_async(request)
        rclpy.spin_until_future_complete(cls.node, future, timeout_sec=timeout_sec)
        response = future.result()
        return response is not None and response.success

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_compute_path_to_pose_returns_a_usable_path(self, proc_output):
        client = ActionClient(self.node, ComputePathToPose, "compute_path_to_pose")

        # setUpClass has already brought the system up and seen a costmap, so
        # the server is expected to be there; this is a check, not a wait.
        self.assertTrue(
            client.wait_for_server(timeout_sec=60.0),
            "compute_path_to_pose action server never appeared, so planner_server "
            "did not reach the active state",
        )

        # Precondition, checked explicitly because getting it wrong once cost an
        # afternoon: if the params file does not reach the costmap it falls back
        # to its own defaults, which are a 5 by 5 metre grid with a static layer
        # waiting forever for a map. The planning result then fails for reasons
        # that have nothing to do with the plugin. Fail here, with a clear
        # message, rather than 30 seconds later with a confusing one.
        startup = "".join(
            event.text.decode("utf-8", errors="replace") for event in proc_output
        )
        self.assertIn(
            'Using plugin "inflation_layer"',
            startup,
            "the costmap did not load the inflation layer, so the params file "
            "never reached it",
        )
        self.assertNotIn(
            'Using plugin "static_layer"',
            startup,
            "the costmap loaded a static layer, so it is using its own defaults "
            "rather than test_planner_params.yaml and will never populate",
        )

        goal = ComputePathToPose.Goal()
        goal.start = _pose(*START_XY)
        goal.goal = _pose(*GOAL_XY)
        goal.planner_id = "GridBased"
        goal.use_start = True  # avoids depending on a localised robot pose

        # With bringup driven explicitly and a costmap already published, a
        # rejection here should not happen at all. The retry stays as a bounded
        # guard so that if some readiness signal is still missing, this surfaces
        # as a slow test rather than a red one, and the message below says how
        # many attempts it took.
        handle = None
        deadline = time.monotonic() + 30.0
        attempts = 0
        while time.monotonic() < deadline:
            attempts += 1
            send = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=10.0)
            handle = send.result()
            if handle is not None and handle.accepted:
                break
            time.sleep(0.5)
        self.assertIsNotNone(handle, "no response to the goal request")
        self.assertTrue(
            handle.accepted,
            f"planner_server rejected the goal on all {attempts} attempts over "
            "30 s, so this is the plugin or the costmap rather than startup timing",
        )

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
        combined = _without_transform_warnings(
            "".join(
                event.text.decode("utf-8", errors="replace") for event in proc_output
            )
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
