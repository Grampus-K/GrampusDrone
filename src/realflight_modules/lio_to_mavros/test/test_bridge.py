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
from std_msgs.msg import Bool


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.messages = {key: [] for key in ("legacy", "preview_pose", "preview", "formal", "armed", "forbidden")}
        self.health = {}
        self.subscribers = []
        for key, topic, kind in (
            ("legacy", "/test/legacy_pose", PoseStamped),
            ("preview_pose", "/test/preview_pose", PoseStamped),
            ("preview", "/test/preview_output", Odometry),
            ("formal", "/test/formal_output", Odometry),
            ("armed", "/test/armed_output", Odometry),
            ("forbidden", "/test/forbidden_pose", PoseStamped),
        ):
            self.subscribers.append(rospy.Subscriber(topic, kind, lambda m, k=key: self.messages[k].append(m)))
        for node in ("legacy", "preview", "formal", "armed"):
            self.subscribers.append(rospy.Subscriber("/" + node + "/healthy", Bool,
                lambda m, k=node: self.health.update({k: m.data})))
        self.subscribers.append(rospy.Subscriber("/preview/preview/healthy", Bool,
            lambda m: self.health.update({"preview_full": m.data})))
        self.old = rospy.Publisher("/test/legacy_input", Odometry, queue_size=10)
        self.preview = rospy.Publisher("/test/preview_input", Odometry, queue_size=10)
        self.formal = rospy.Publisher("/test/formal_input", Odometry, queue_size=10)
        self.state = rospy.Publisher("/test/formal_state", State, queue_size=10)
        self.armed_state = rospy.Publisher("/test/armed_state", State, queue_size=10)
        self.stamps = set()
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and (self.old.get_num_connections() < 2 or
                self.preview.get_num_connections() < 1 or self.formal.get_num_connections() < 2 or
                self.state.get_num_connections() < 1 or self.armed_state.get_num_connections() < 1):
            time.sleep(.05)
        self.assertEqual(self.old.get_num_connections(), 2)
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

    def pump(self, duration, preview_mode="good", formal_mode="good", connected=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.state.publish(State(connected=connected, armed=False))
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
        self.pump(2.5)
        for key in ("legacy", "preview_pose", "preview", "formal"):
            self.assertGreater(len(self.messages[key]), 2, key)
        self.assertFalse(self.messages["armed"])
        self.assertFalse(self.messages["forbidden"])
        self.assertFalse(self.health.get("armed", True))
        self.assertTrue(self.health["formal"])
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
        count = len(self.messages["formal"])
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
