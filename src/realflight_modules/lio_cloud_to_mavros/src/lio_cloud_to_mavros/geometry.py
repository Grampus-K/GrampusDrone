"""Transforms use column vectors: p_parent = R_parent_child @ p_child + t."""

from bisect import bisect_left
from dataclasses import dataclass
import math

import numpy as np


def vector3(values):
    result = np.asarray(values, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError("expected three finite numbers")
    return result


def quaternion(values):
    """ROS order: x, y, z, w. Reject grossly invalid input, then normalize."""
    result = np.asarray(values, dtype=float)
    if result.shape != (4,) or not np.isfinite(result).all():
        raise ValueError("invalid quaternion")
    norm = np.linalg.norm(result)
    if not 0.9 <= norm <= 1.1:
        raise ValueError("quaternion norm is not close to one")
    return result / norm


def rotation(q):
    x, y, z, w = quaternion(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def rpy_rotation_degrees(values):
    roll, pitch, yaw = np.deg2rad(vector3(values))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # Fixed-axis RPY, equivalently Rz(yaw) * Ry(pitch) * Rx(roll).
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]])


def checked_rotation(values):
    matrix = np.asarray(values, dtype=float)
    if matrix.size != 9:
        raise ValueError("rotation requires nine row-major numbers")
    matrix = matrix.reshape(3, 3)
    if (not np.isfinite(matrix).all() or
            not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-5) or
            not np.isclose(np.linalg.det(matrix), 1.0, atol=1e-5)):
        raise ValueError("extrinsic_R is not a proper rotation")
    return matrix


def slerp(q0, q1, fraction):
    q0, q1 = quaternion(q0), quaternion(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    dot = np.clip(dot, -1.0, 1.0)
    if dot > 0.9995:
        value = q0 + fraction * (q1 - q0)
        return value / np.linalg.norm(value)
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*q0 +
            math.sin(fraction*angle)*q1) / math.sin(angle)


@dataclass
class PoseSample:
    stamp_ns: int
    position: np.ndarray
    orientation: np.ndarray


class PoseBuffer:
    """Bounded history. No extrapolation; discontinuities are explicit faults."""

    def __init__(self, history_seconds=3.0, max_samples=1000,
                 max_position_step=0.5, max_speed=20.0,
                 max_angle_step_degrees=20.0, max_angular_speed=6.0):
        self.samples = []
        self.history_ns = int(history_seconds * 1e9)
        self.max_samples = max_samples
        self.position_step = max_position_step
        self.max_speed = max_speed
        self.angle_step = math.radians(max_angle_step_degrees)
        self.max_angular_speed = max_angular_speed

    def add(self, sample):
        if sample.stamp_ns <= 0:
            raise ValueError("zero/negative odometry timestamp")
        sample = PoseSample(sample.stamp_ns, vector3(sample.position),
                            quaternion(sample.orientation))
        if self.samples:
            previous = self.samples[-1]
            dt = (sample.stamp_ns - previous.stamp_ns) * 1e-9
            if dt < 0:
                raise ValueError("odometry timestamp moved backwards")
            angle = 2 * math.acos(float(np.clip(
                abs(np.dot(previous.orientation, sample.orientation)), 0, 1)))
            distance = np.linalg.norm(sample.position - previous.position)
            if dt == 0:
                if distance > 1e-6 or angle > 1e-6:
                    raise ValueError("different poses share one timestamp")
                return
            if distance > self.position_step + self.max_speed * dt:
                raise ValueError("odometry position jumped")
            if angle > self.angle_step + self.max_angular_speed * dt:
                raise ValueError("odometry attitude jumped")
        self.samples.append(sample)
        cutoff = sample.stamp_ns - self.history_ns
        while (len(self.samples) > 1 and self.samples[1].stamp_ns < cutoff):
            self.samples.pop(0)
        if len(self.samples) > self.max_samples:
            del self.samples[:-self.max_samples]

    def lookup(self, stamp_ns, max_gap_seconds, exact=False):
        index = bisect_left([p.stamp_ns for p in self.samples], stamp_ns)
        if index < len(self.samples) and self.samples[index].stamp_ns == stamp_ns:
            return self.samples[index]
        if exact or index == 0 or index == len(self.samples):
            return None
        before, after = self.samples[index-1:index+1]
        gap_ns = after.stamp_ns - before.stamp_ns
        if gap_ns > max_gap_seconds * 1e9:
            return None
        fraction = (stamp_ns - before.stamp_ns) / gap_ns
        return PoseSample(stamp_ns,
                          before.position + fraction*(after.position-before.position),
                          slerp(before.orientation, after.orientation, fraction))


def cloud_transform(lio_pose, local_pose, rotation_imu_lidar,
                    translation_imu_lidar, rotation_base_lidar,
                    translation_base_lidar):
    """Recover lidar coordinates, then project via base_link to PX4 local.

    W: FAST-LIO world; I: its IMU; L: lidar; B: MAVROS base_link; M: PX4 local.
    T_MW = T_MB * T_BL * inverse(T_IL) * inverse(T_WI).
    This uses per-scan poses, not a fixed world-to-map alignment assumption.
    """
    r_wi = rotation(lio_pose.orientation)
    r_mb = rotation(local_pose.orientation)
    r_bi = rotation_base_lidar @ rotation_imu_lidar.T
    r_mw = r_mb @ r_bi @ r_wi.T
    t_mw = (local_pose.position + r_mb @ (
        translation_base_lidar - r_bi @ translation_imu_lidar)
        - r_mw @ lio_pose.position)
    return r_mw, t_mw


def transform_cloud_data(data, fields, width, height, point_step, row_step,
                         is_bigendian, matrix, translation):
    """Vectorized XYZ (and normals) conversion; retain other fields/padding.

    Fields use the sensor_msgs/PointField name/offset/datatype/count interface.
    Only FLOAT32=7 or FLOAT64=8 scalar coordinates are accepted.
    """
    if (width <= 0 or height <= 0 or point_step <= 0 or
            row_step < width*point_step or len(data) != height*row_step):
        raise ValueError("invalid or empty PointCloud2 layout")
    names = [field.name for field in fields]
    if len(names) != len(set(names)):
        raise ValueError("duplicate PointCloud2 field names")
    lookup = {field.name: field for field in fields}
    vectors = [("x", "y", "z")]
    normals = ("normal_x", "normal_y", "normal_z")
    if any(name in lookup for name in normals):
        vectors.append(normals)
    selected = []
    ranges = []
    endian = ">" if is_bigendian else "<"
    for group in vectors:
        for name in group:
            if name not in lookup:
                raise ValueError("missing coordinate field: " + name)
            field = lookup[name]
            if field.count != 1 or field.datatype not in (7, 8):
                raise ValueError("coordinates must be FLOAT32/FLOAT64 scalars")
            size = 4 if field.datatype == 7 else 8
            if field.offset < 0 or field.offset + size > point_step:
                raise ValueError("coordinate field exceeds point_step")
            ranges.append((field.offset, field.offset + size))
            selected.append((name, endian + "f" + str(size), field.offset))
    ranges.sort()
    if any(end > start for (_, end), (start, _) in zip(ranges, ranges[1:])):
        raise ValueError("overlapping coordinate fields")
    # Reject aliases of XYZ/normals so modifying them cannot corrupt intensity etc.
    datatype_sizes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    selected_names = {name for name, _, _ in selected}
    for field in fields:
        if field.name not in selected_names:
            size = datatype_sizes.get(field.datatype)
            if size is None or field.count < 1 or field.offset < 0:
                raise ValueError("invalid auxiliary field")
            end = field.offset + size*field.count
            if end > point_step or any(field.offset < hi and end > lo for lo, hi in ranges):
                raise ValueError("auxiliary field overlaps coordinates or exceeds point_step")
    dtype = np.dtype({"names": [s[0] for s in selected],
                      "formats": [s[1] for s in selected],
                      "offsets": [s[2] for s in selected], "itemsize": point_step})
    result = bytearray(data)
    view = np.ndarray((height, width), dtype=dtype, buffer=result,
                      strides=(row_step, point_step))
    for index, group in enumerate(vectors):
        points = np.stack([view[name] for name in group], axis=-1).astype(float)
        transformed = points @ matrix.T
        if index == 0:
            transformed += translation
        for axis, name in enumerate(group):
            view[name] = transformed[..., axis]
    return bytes(result)
