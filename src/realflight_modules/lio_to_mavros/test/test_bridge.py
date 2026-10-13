#!/usr/bin/env python3
"""ROS boundary regression: run via catkin/rostest, with no flight hardware."""
import copy
import math
import time
import unittest

import roslaunch
import rospy
import rostest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.messages = {key: [] for key in ("legacy", "preview_pose", "preview", "formal", "armed", "forbidden", "stale_state")}
        self.health = {}
        self.status = {}
        self.subscribers = []
        for key, topic, kind in (
            ("legacy", "/test/legacy_pose", PoseStamped),
            ("preview_pose", "/test/preview_pose", PoseStamped),
            ("preview", "/test/preview_output", Odometry),
            ("formal", "/test/formal_output", Odometry),
            ("armed", "/test/armed_output", Odometry),
            ("forbidden", "/test/forbidden_pose", PoseStamped),
            ("stale_state", "/test/stale_output", Odometry),
        ):
            self.subscribers.append(rospy.Subscriber(topic, kind, lambda m, k=key: self.messages[k].append(m)))
        for node in ("legacy", "preview", "formal", "armed", "stale_state"):
            self.subscribers.append(rospy.Subscriber("/" + node + "/healthy", Bool,
                lambda m, k=node: self.health.update({k: m.data})))
        self.subscribers.append(rospy.Subscriber("/preview/preview/healthy", Bool,
            lambda m: self.health.update({"preview_full": m.data})))
        for node in ("formal", "stale_state"):
            self.subscribers.append(rospy.Subscriber("/" + node + "/status", String,
                lambda m, k=node: self.status.update({k: m.data})))
        self.old = rospy.Publisher("/test/legacy_input", Odometry, queue_size=10)
        self.preview = rospy.Publisher("/test/preview_input", Odometry, queue_size=10)
        self.formal = rospy.Publisher("/test/formal_input", Odometry, queue_size=10)
        self.state = rospy.Publisher("/test/formal_state", State, queue_size=10)
        self.armed_state = rospy.Publisher("/test/armed_state", State, queue_size=10)
        self.stale_state = rospy.Publisher("/test/stale_state", State, queue_size=10)
        self.next_state_at = 0.0
        self.stamps = set()
        expected_connections = ((self.old, 2), (self.preview, 1), (self.formal, 3),
                                (self.state, 1), (self.armed_state, 1), (self.stale_state, 1))
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and any(
                pub.get_num_connections() < count for pub, count in expected_connections):
            time.sleep(.05)
        for pub, count in expected_connections:
            self.assertEqual(pub.get_num_connections(), count, pub.resolved_name)
        self.last = None

    def measurement(self):
        m = Odometry()
        m.header.stamp = rospy.Time.now() - rospy.Duration(.02)
        m.header.frame_id, m.child_frame_id = "world", "body"
        # Nonzero initial pose/tilt exercises legacy reference subtraction too.
        m.pose.pose.position.x = 2
        m.pose.pose.position.y = -1
        m.pose.pose.position.z = .7
        m.pose.pose.orientation.x = math.sin(.1)
        m.pose.pose.orientation.w = math.cos(.1)
        for i in range(6):
            m.pose.covariance[7*i] = .02
            m.twist.covariance[7*i] = .03
        return m

    def pump(self, duration, preview_mode="good", formal_mode="good", connected=True,
             send_stale_state=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            # Heartbeat-like state cadence, intentionally just slower than 1 Hz.
            # The old 1 s receipt timeout would latch despite healthy odometry.
            if time.monotonic() >= self.next_state_at:
                self.state.publish(State(connected=connected, armed=False))
                if send_stale_state:
                    self.stale_state.publish(State(connected=True, armed=False))
                self.next_state_at = time.monotonic() + 1.1
            # An explicit disconnect must not wait for the heartbeat timeout.
            if not connected:
                self.state.publish(State(connected=False, armed=False))
            self.armed_state.publish(State(connected=True, armed=True))
            m = self.measurement()
            self.old.publish(m)
            if preview_mode != "none":
                candidate = copy.deepcopy(m)
                if preview_mode == "invalid":
                    candidate.pose.covariance[0] = -1
                self.preview.publish(candidate)
            if formal_mode == "good":
                self.stamps.add(m.header.stamp.to_nsec())
                self.formal.publish(m)
                self.last = copy.deepcopy(m)
            elif formal_mode == "duplicate":
                self.formal.publish(self.last)
            time.sleep(.05)

    def test_end_to_end(self):
        self.pump(4.5)
        for key in ("legacy", "preview_pose", "preview", "formal", "stale_state"):
            self.assertGreater(len(self.messages[key]), 2, key)
        self.assertFalse(self.messages["armed"])
        self.assertFalse(self.messages["forbidden"])
        self.assertFalse(self.health.get("armed", True))
        self.assertTrue(self.health["formal"])
        self.assertTrue(self.health["stale_state"])
        pose = self.messages["legacy"][-1]
        self.assertAlmostEqual(pose.pose.position.x, 0)
        self.assertAlmostEqual(pose.pose.position.y, 0)
        self.assertAlmostEqual(pose.pose.orientation.w, 1)
        for m in self.messages["formal"]:
            self.assertIn(m.header.stamp.to_nsec(), self.stamps)
            self.assertEqual((m.header.frame_id, m.child_frame_id), ("odom", "base_link"))
            self.assertAlmostEqual(m.pose.pose.orientation.x, math.sin(.1), places=6)
        stamps = [m.header.stamp.to_nsec() for m in self.messages["formal"]]
        self.assertEqual(stamps, sorted(set(stamps)))
        self.assertEqual(self.messages["preview"][-1].header.frame_id, "lio_preview_odom")
        # State loss must still latch even while fresh odometry keeps arriving.
        self.pump(3.5, send_stale_state=False)
        self.assertFalse(self.health["stale_state"])
        self.assertIn("MAVROS state expired", self.status["stale_state"])
        count = len(self.messages["stale_state"])
        self.stale_state.publish(State(connected=True, armed=False))
        self.pump(.3)
        self.assertEqual(len(self.messages["stale_state"]), count)
        self.assertFalse(self.health["stale_state"])
        self.assertTrue(self.health["formal"])
        # Duplicate samples do not create additional measurements.
        time.sleep(.1)
        count = len(self.messages["formal"])
        self.pump(.15, formal_mode="duplicate")
        self.assertEqual(len(self.messages["formal"]), count)
        # Preview covariance failure must leave both old pose gates running.
        before = len(self.messages["preview_pose"])
        self.pump(.4, preview_mode="invalid")
        self.assertGreater(len(self.messages["preview_pose"]), before)
        self.assertTrue(self.health["preview"])
        self.assertFalse(self.health["preview_full"])
        count = len(self.messages["preview"])
        self.pump(.3)
        self.assertEqual(len(self.messages["preview"]), count)
        # Disconnect latches the formal output; reconnect never re-zeros it.
        self.pump(.3, connected=False)
        self.assertFalse(self.health["formal"])
        self.assertIn("MAVROS disconnected", self.status["formal"])
        count = len(self.messages["formal"])
        self.state.publish(State(connected=True, armed=False))
        self.pump(.5, connected=True)
        self.assertEqual(len(self.messages["formal"]), count)
        self.assertFalse(self.health["formal"])
        self.assertFalse(self.messages["armed"])
        # Existing pose outputs stop after input timeout.
        time.sleep(.7)
        before = len(self.messages["legacy"])
        time.sleep(.2)
        self.assertEqual(len(self.messages["legacy"]), before)

        # Invalid startup modes/remaps fail before any formal data is sent.
        launcher = roslaunch.scriptapi.ROSLaunch()
        launcher.start()
        try:
            for suffix, args in (
                ("bad_mode", "_output_mode:=wrong"),
                ("bad_pair", "_output_mode:=odometry _odometry_preview:=true"),
                ("bad_remap", "_odometry_preview:=true /lio_to_mavros/odometry_preview:=/mavros/odometry/out"),
            ):
                proc = launcher.launch(roslaunch.core.Node("lio_to_mavros", "lio_to_mavros_node",
                    name=suffix, args=args, output="screen"))
                deadline = time.monotonic() + 5
                while proc.is_alive() and time.monotonic() < deadline:
                    time.sleep(.05)
                self.assertFalse(proc.is_alive(), suffix)
                self.assertNotEqual(proc.exit_code, 0, suffix)
        finally:
            launcher.stop()


if __name__ == "__main__":
    rospy.init_node("bridge_integration_test")
    rostest.rosrun("lio_to_mavros", "bridge_integration", BridgeTest)
