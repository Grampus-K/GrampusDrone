import math
import struct
from types import SimpleNamespace
import unittest

import numpy as np

from lio_cloud_to_mavros.geometry import (
    PoseBuffer, PoseSample, checked_rotation, cloud_transform, quaternion,
    rotation, rpy_rotation_degrees, transform_cloud_data)


def pose(stamp, position=(0, 0, 0), q=(0, 0, 0, 1)):
    return PoseSample(stamp, np.array(position, dtype=float), np.array(q, dtype=float))


def field(name, offset, datatype=7, count=1):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=count)


class GeometryTests(unittest.TestCase):
    def test_lidar_center_height_and_imu_offset(self):
        # Laser-origin point in world includes the LIO IMU-to-lidar translation.
        t_il = np.array([-0.011, -0.02329, 0.04412])
        lio = pose(1, (5, -3, 2))
        local = pose(1, (10, 20, 30))
        matrix, translation = cloud_transform(
            lio, local, np.eye(3), t_il, np.eye(3), np.array([0, 0, 0.07]))
        point_world = lio.position + t_il
        np.testing.assert_allclose(matrix @ point_world + translation, [10, 20, 30.07])

    def test_random_full_attitude_roundtrip(self):
        rng = np.random.RandomState(123)
        for _ in range(50):
            q_wi = rng.randn(4)
            q_wi /= np.linalg.norm(q_wi)
            q_mb = rng.randn(4)
            q_mb /= np.linalg.norm(q_mb)
            lio = pose(1, rng.randn(3), q_wi)
            local = pose(1, rng.randn(3), q_mb)
            r_il = rpy_rotation_degrees(rng.uniform(-90, 90, 3))
            r_bl = rpy_rotation_degrees(rng.uniform(-90, 90, 3))
            t_il, t_bl = rng.randn(3)*0.1, rng.randn(3)*0.1
            lidar_points = rng.randn(30, 3)
            world_points = (lidar_points @ r_il.T + t_il) @ rotation(q_wi).T + lio.position
            expected = (lidar_points @ r_bl.T + t_bl) @ rotation(q_mb).T + local.position
            matrix, translation = cloud_transform(lio, local, r_il, t_il, r_bl, t_bl)
            np.testing.assert_allclose(world_points @ matrix.T + translation, expected, atol=1e-12)
            np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-12)

    def test_degrees_and_rotation_validation(self):
        np.testing.assert_allclose(rpy_rotation_degrees([0, 0, 90]) @ [1, 0, 0],
                                   [0, 1, 0], atol=1e-12)
        checked_rotation(np.eye(3).reshape(-1))
        for value in ([1, 2], np.diag([1, 1, -1]), np.ones((3, 3))):
            with self.assertRaises(ValueError):
                checked_rotation(value)
        with self.assertRaises(ValueError):
            quaternion([0, 0, 0, 0])
        with self.assertRaises(ValueError):
            quaternion([0, 0, math.nan, 1])

    def test_interpolation_no_extrapolation_or_wide_gaps(self):
        start = 1790239125463162422
        buffer = PoseBuffer(max_speed=100, max_angle_step_degrees=180)
        buffer.add(pose(start))
        buffer.add(pose(start+100000000, (2, 0, 0), (0, 0, 1, 0)))
        halfway = buffer.lookup(start+50000000, 0.15)
        np.testing.assert_allclose(halfway.position, [1, 0, 0])
        np.testing.assert_allclose(rotation(halfway.orientation) @ [1, 0, 0], [0, 1, 0], atol=1e-12)
        self.assertIsNone(buffer.lookup(start-1, 0.15))
        self.assertIsNone(buffer.lookup(start+100000001, 0.15))
        self.assertIsNone(buffer.lookup(start+50000000, 0.05))
        self.assertIsNone(buffer.lookup(start+50000000, 0.15, exact=True))
        self.assertEqual(buffer.lookup(start, 0.15, exact=True).stamp_ns, start)

    def test_quaternion_sign_is_not_a_reset(self):
        buffer = PoseBuffer()
        buffer.add(pose(1000000000))
        buffer.add(pose(1100000000, q=(0, 0, 0, -1)))
        result = buffer.lookup(1050000000, 0.2)
        np.testing.assert_allclose(rotation(result.orientation), np.eye(3))

    def test_discontinuities_and_bounded_history(self):
        buffer = PoseBuffer(max_samples=3)
        for index in range(5):
            buffer.add(pose(1000000000+index*100000000))
        self.assertEqual(len(buffer.samples), 3)
        with self.assertRaisesRegex(ValueError, "backwards"):
            buffer.add(pose(1000000000))
        with self.assertRaisesRegex(ValueError, "position jumped"):
            buffer.add(pose(1410000000, (100, 0, 0)))
        with self.assertRaisesRegex(ValueError, "attitude jumped"):
            buffer.add(pose(1410000000, q=(0, 0, 1, 0)))
        with self.assertRaisesRegex(ValueError, "share one timestamp"):
            buffer.add(pose(1400000000, (1, 0, 0)))
        buffer.add(pose(1400000000))  # Identical duplicate is harmless.
        self.assertEqual(len(buffer.samples), 3)

    def test_cloud_normals_endianness_padding_and_auxiliary_fields(self):
        fields = [field(name, index*4) for index, name in enumerate(
            ["x", "y", "z", "normal_x", "normal_y", "normal_z", "intensity"])]
        matrix = rpy_rotation_degrees([0, 0, 90])
        for endian in ("<", ">"):
            point = struct.pack(endian+"7f", 1, 2, 3, 1, 0, 0, 42) + b"PAD!"
            row = point*2 + b"ROW-PADD"
            result = transform_cloud_data(row*2, fields, 2, 2, 32, 72, endian==">",
                                          matrix, np.array([10, 20, 30]))
            self.assertEqual(len(result), len(row)*2)
            for offset in (0, 32, 72, 104):
                values = struct.unpack_from(endian+"7f", result, offset)
                np.testing.assert_allclose(values[:3], [8, 21, 33], atol=1e-6)
                np.testing.assert_allclose(values[3:6], [0, 1, 0], atol=1e-6)
                self.assertEqual(values[6], 42)
                self.assertEqual(result[offset+28:offset+32], b"PAD!")
            self.assertEqual(result[64:72], b"ROW-PADD")
            self.assertEqual(result[136:144], b"ROW-PADD")

    def test_float64_and_invalid_layouts(self):
        fields = [field("x", 0, 8), field("y", 8, 8), field("z", 16, 8)]
        data = struct.pack("<3d", 1, 2, 3)
        result = transform_cloud_data(data, fields, 1, 1, 24, 24, False,
                                      np.eye(3), np.array([0, 0, 0.07]))
        np.testing.assert_allclose(struct.unpack("<3d", result), [1, 2, 3.07])
        invalid = [fields[:2], fields + [field("normal_x", 0)],
                   [field("x", 0, 8), field("y", 0, 8), field("z", 16, 8)],
                   fields + [field("intensity", 0)],
                   [field("x", 0, 8, 2), field("y", 8, 8), field("z", 16, 8)]]
        for bad in invalid:
            with self.assertRaises(ValueError):
                transform_cloud_data(data, bad, 1, 1, 24, 24, False, np.eye(3), np.zeros(3))
        with self.assertRaises(ValueError):
            transform_cloud_data(data[:-1], fields, 1, 1, 24, 24, False, np.eye(3), np.zeros(3))


if __name__ == "__main__":
    unittest.main()
