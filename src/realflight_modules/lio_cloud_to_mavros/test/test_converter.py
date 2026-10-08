"""Offline callback tests with ROS message/transport stubs, not a ROS integration test."""

from pathlib import Path
import struct
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


def node_source():
    return (Path(__file__).resolve().parents[1] / "scripts" /
            "registered_cloud_to_mavros.py").read_text(encoding="utf-8")


class Stamp:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_nsec(self):
        return self.nanoseconds


class Publisher:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class ConverterTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.15
        self.epoch = 1790239125000000000
        self.clock_step = 0
        self.params = {
            "/mapping/extrinsic_est_en": False,
            "/mapping/extrinsic_T": [-0.011, -0.02329, 0.04412],
            "/mapping/extrinsic_R": [1, 0, 0, 0, 1, 0, 0, 0, 1]}
        ros = ModuleType("rospy")
        ros.Time = SimpleNamespace(now=lambda: Stamp(
            self.epoch + int(self.now*1e9) + self.clock_step))
        ros.resolve_name = lambda name: name
        ros.get_param = lambda key, *args: self.params[key] if key in self.params else (
            args[0] if args else self._missing(key))
        ros.Publisher = Publisher
        ros.Subscriber = lambda *args, **kwargs: None
        for method in ("loginfo", "logerr", "logwarn_throttle", "loginfo_throttle"):
            setattr(ros, method, lambda *args, **kwargs: None)
        modules = {"rospy": ros}
        for package, classes in {
                "std_msgs": ("Bool", "String"), "sensor_msgs": ("PointCloud2",),
                "nav_msgs": ("Odometry",), "mavros_msgs": ("State",)}.items():
            modules[package] = ModuleType(package)
            messages = ModuleType(package + ".msg")
            for name in classes:
                setattr(messages, name, SimpleNamespace)
            modules[package + ".msg"] = messages
        self.module_patch = patch.dict(sys.modules, modules)
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.node_module = ModuleType("cloud_converter_test_node")
        exec(compile(node_source(), "registered_cloud_to_mavros.py", "exec"), self.node_module.__dict__)
        self.clock_patch = patch.object(self.node_module.time, "monotonic", lambda: self.now)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.node = self.node_module.RegisteredCloudConverter()
        self.node._on_state(SimpleNamespace(connected=True))
        self.node._on_vision_health(SimpleNamespace(data=True))

    @staticmethod
    def _missing(key):
        raise KeyError(key)

    def odom(self, seconds, kind="mavros", position=(0, 0, 0)):
        header = SimpleNamespace(stamp=Stamp(self.epoch+int(seconds*1e9)),
                                 frame_id="world" if kind == "lio" else "map")
        return SimpleNamespace(
            header=header, child_frame_id="body" if kind == "lio" else "base_link",
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
                orientation=SimpleNamespace(x=0, y=0, z=0, w=1))))

    def cloud(self, seconds=0.1):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=Stamp(self.epoch+int(seconds*1e9)), frame_id="world"),
            data=struct.pack("<3f", -0.011, -0.02329, 0.04412),
            fields=[SimpleNamespace(name=name, offset=i*4, datatype=7, count=1)
                    for i, name in enumerate(("x", "y", "z"))],
            width=1, height=1, point_step=12, row_step=12, is_bigendian=False,
            is_dense=True)

    def ready_inputs(self):
        self.node._on_odom(self.odom(0.05), "mavros")
        self.node._on_odom(self.odom(0.15), "mavros")
        self.node._on_odom(self.odom(0.1, "lio"), "lio")
        self.node._on_cloud(self.cloud())

    def test_waits_for_exact_lio_and_bracketing_mavros_then_preserves_stamp(self):
        cloud = self.cloud()
        self.node._on_cloud(cloud)
        self.node._on_odom(self.odom(0.05), "mavros")
        self.node._on_odom(self.odom(0.1, "lio"), "lio")
        self.node.step()
        self.assertEqual(len(self.node.cloud_pub.messages), 0)
        self.node._on_odom(self.odom(0.15), "mavros")
        self.node.step()
        output = self.node.cloud_pub.messages[-1]
        self.assertEqual(output.header.frame_id, "map")
        self.assertEqual(output.header.stamp.to_nsec(), cloud.header.stamp.to_nsec())
        self.assertEqual(cloud.header.frame_id, "world")
        x, y, z = struct.unpack("<3f", output.data)
        self.assertAlmostEqual(x, 0, places=6)
        self.assertAlmostEqual(y, 0, places=6)
        self.assertAlmostEqual(z, 0.07, places=6)
        self.assertTrue(self.node.last_health)

    def test_missing_exact_lio_does_not_use_latest_pose(self):
        self.ready_inputs()
        self.node.buffers["lio"].samples.clear()
        self.node._on_odom(self.odom(0.11, "lio"), "lio")
        self.node.step()
        self.assertFalse(self.node.cloud_pub.messages)
        self.now += 0.6
        self.node.step()
        self.assertFalse(self.node.pending)
        self.assertEqual(self.node.dropped, 1)
        self.assertFalse(self.node.last_health)

    def test_missing_extrinsics_and_closed_vision_gate_block_output(self):
        self.ready_inputs()
        self.params.clear()
        self.node.step()
        self.assertFalse(self.node.cloud_pub.messages)
        self.params.update({"/mapping/extrinsic_est_en": False,
                            "/mapping/extrinsic_T": [0, 0, 0],
                            "/mapping/extrinsic_R": [1, 0, 0, 0, 1, 0, 0, 0, 1]})
        self.node.last_param_check = -float("inf")
        self.node._on_vision_health(SimpleNamespace(data=False))
        self.node.step()
        self.assertFalse(self.node.cloud_pub.messages)

    def test_wrong_frame_and_online_extrinsics_latch_faults(self):
        msg = self.odom(0.1)
        msg.child_frame_id = "unexpected"
        self.node._on_odom(msg, "mavros")
        self.assertIsNotNone(self.node.fault_reason)
        other = self.node_module.RegisteredCloudConverter()
        self.params["/mapping/extrinsic_est_en"] = True
        other.step()
        self.assertIn("online extrinsic", other.fault_reason)

    def test_vision_failure_after_publication_latches_fault(self):
        self.ready_inputs()
        self.node.step()
        self.node._on_vision_health(SimpleNamespace(data=False))
        self.node._on_vision_health(SimpleNamespace(data=True))
        self.node.step()
        self.assertFalse(self.node.last_health)
        self.assertEqual(len(self.node.cloud_pub.messages), 1)

    def test_clock_step_and_extrinsic_change_stop_output(self):
        self.ready_inputs()
        self.node.step()
        self.clock_step = 2000000000
        self.node.step()
        self.assertIn("system clock jumped", self.node.fault_reason)
        other = self.node_module.RegisteredCloudConverter()
        other._check_extrinsics(self.now)
        self.params["/mapping/extrinsic_T"] = [0, 0, 0]
        other._check_extrinsics(self.now+1.1)
        self.assertIn("extrinsics changed", other.fault_reason)

    def test_stale_output_health(self):
        self.ready_inputs()
        self.node.step()
        self.now += 0.6
        self.node.step()
        self.assertFalse(self.node.last_health)
        self.assertEqual(len(self.node.cloud_pub.messages), 1)

    def test_invalid_cloud_layout_is_dropped_without_publishing(self):
        self.ready_inputs()
        self.node.pending[0][0].fields[0].datatype = 5  # Integer XYZ is unsupported.
        self.node.step()
        self.assertFalse(self.node.cloud_pub.messages)
        self.assertEqual(self.node.dropped, 1)
        self.assertFalse(self.node.last_health)


if __name__ == "__main__":
    unittest.main()
