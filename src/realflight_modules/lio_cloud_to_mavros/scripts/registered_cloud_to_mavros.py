#!/usr/bin/env python3
"""Standalone converter. It deliberately does not rewire EGO or publish vision."""

from collections import deque
import copy
import math
import threading
import time

import numpy as np
import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String

from lio_cloud_to_mavros.geometry import (
    PoseBuffer, PoseSample, checked_rotation, cloud_transform,
    rpy_rotation_degrees, transform_cloud_data, vector3)


class RegisteredCloudConverter:
    def __init__(self):
        self.lock = threading.RLock()
        self.fault_reason = None
        self.last_status = None
        self.last_health = None
        self.last_output_at = None
        self.last_ros_ns = None
        self.last_clock_at = None
        self.last_param_check = -math.inf
        self.extrinsics = None
        self.pending = deque()
        self.last_cloud_ns = 0
        self.local_frame = None
        self.connected = False
        self.state_at = -math.inf
        self.vision_healthy = False
        self.received_at = {"lio": -math.inf, "mavros": -math.inf}
        self.published = 0
        self.dropped = 0

        self.cloud_topic = self._text("cloud_topic", "/cloud_registered")
        self.lio_topic = self._text("lio_odom_topic", "/Odometry")
        self.mavros_topic = self._text("mavros_odom_topic", "/mavros/local_position/odom")
        self.output_topic = self._text("output_topic", "/cloud_registered_mavros")
        if rospy.resolve_name(self.cloud_topic) == rospy.resolve_name(self.output_topic):
            raise ValueError("input and output cloud topics must differ")
        self.mapping_ns = self._text("fastlio_mapping_namespace", "/mapping").rstrip("/")
        self.world_frame = self._text("lio_world_frame", "world")
        self.imu_frame = self._text("lio_imu_frame", "body")
        self.body_frame = self._text("mavros_body_frame", "base_link")
        self.output_frame = rospy.get_param("~output_frame", "")
        if not isinstance(self.output_frame, str):
            raise ValueError("output_frame must be a string")
        self.require_vision = rospy.get_param("~require_vision_healthy", True)
        if type(self.require_vision) is not bool:
            raise ValueError("require_vision_healthy must be a bool")
        self.t_bl = vector3(rospy.get_param("~lidar_in_base_link/translation", [0, 0, 0.07]))
        self.r_bl = rpy_rotation_degrees(rospy.get_param(
            "~lidar_in_base_link/rpy_degrees", [0, 0, 0]))

        self.interpolation_gap = self._positive("max_interpolation_gap", 0.15)
        self.cloud_wait = self._positive("max_cloud_wait", 0.5)
        self.stamp_error = self._positive("max_stamp_error", 0.5)
        self.odom_timeout = self._positive("odom_receipt_timeout", 0.5)
        self.output_timeout = self._positive("output_timeout", 0.5)
        self.clock_tolerance = self._positive("clock_step_tolerance", 0.5)
        self.max_pending = self._integer("max_pending_clouds", 5)
        self.max_points = self._integer("max_points", 200000)
        buffer_parameters = dict(
            history_seconds=self._positive("history_seconds", 3.0),
            max_samples=self._integer("max_pose_samples", 1000, minimum=2),
            max_position_step=self._positive("max_position_step", 0.5),
            max_speed=self._positive("max_speed", 20.0),
            max_angle_step_degrees=self._positive("max_angle_step_degrees", 20.0),
            max_angular_speed=self._positive("max_angular_speed", 6.0))
        self.buffers = {kind: PoseBuffer(**buffer_parameters) for kind in self.received_at}

        self.cloud_pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        self.health_pub = rospy.Publisher("~healthy", Bool, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self._status("WAITING: odometry, extrinsics and guarded vision", False)
        self.subscribers = [
            rospy.Subscriber(self.lio_topic, Odometry,
                             lambda msg: self._on_odom(msg, "lio"), queue_size=100,
                             tcp_nodelay=True),
            rospy.Subscriber(self.mavros_topic, Odometry,
                             lambda msg: self._on_odom(msg, "mavros"), queue_size=100,
                             tcp_nodelay=True),
            rospy.Subscriber(self.cloud_topic, PointCloud2, self._on_cloud,
                             queue_size=2, buff_size=32*1024*1024, tcp_nodelay=True),
            rospy.Subscriber(self._text("mavros_state_topic", "/mavros/state"),
                             State, self._on_state, queue_size=1)]
        if self.require_vision:
            self.subscribers.append(rospy.Subscriber(
                self._text("vision_health_topic", "/lio_to_mavros/healthy"),
                Bool, self._on_vision_health, queue_size=1))
        rospy.loginfo("Cloud test only: %s -> %s; lidar translation=%s m; EGO unchanged",
                      self.cloud_topic, self.output_topic, self.t_bl.tolist())

    @staticmethod
    def _text(name, default):
        value = rospy.get_param("~" + name, default)
        if not isinstance(value, str) or not value.strip():
            raise ValueError(name + " must be a nonempty string")
        return value

    @staticmethod
    def _positive(name, default):
        value = float(rospy.get_param("~" + name, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(name + " must be positive and finite")
        return value

    @staticmethod
    def _integer(name, default, minimum=1):
        value = rospy.get_param("~" + name, default)
        if type(value) is not int or value < minimum:
            raise ValueError(name + " must be an integer >= " + str(minimum))
        return value

    def _status(self, text, healthy):
        if text != self.last_status:
            self.status_pub.publish(String(data=text))
            self.last_status = text
            rospy.loginfo(text)
        if healthy != self.last_health:
            self.health_pub.publish(Bool(data=healthy))
            self.last_health = healthy

    def _fault(self, reason):
        if self.fault_reason is None:
            self.fault_reason = reason
            self.pending.clear()
            self._status("FAULT: " + reason + "; inspect and restart this converter", False)
            rospy.logerr("Cloud conversion disabled: %s", reason)

    def _on_state(self, msg):
        with self.lock:
            self.connected = msg.connected
            self.state_at = time.monotonic()

    def _on_vision_health(self, msg):
        with self.lock:
            self.vision_healthy = msg.data
            if not msg.data and self.last_output_at is not None:
                self._fault("vision gate became unhealthy after cloud publication")

    def _on_odom(self, msg, kind):
        with self.lock:
            if self.fault_reason:
                return
            try:
                stamp = msg.header.stamp.to_nsec()
                if abs(rospy.Time.now().to_nsec() - stamp) > self.stamp_error*1e9:
                    raise ValueError("odometry timestamp disagrees with system clock")
                if kind == "lio":
                    if msg.header.frame_id != self.world_frame or msg.child_frame_id != self.imu_frame:
                        raise ValueError("FAST-LIO frame mismatch (expected world/IMU frames)")
                else:
                    frame = msg.header.frame_id
                    if not frame or msg.child_frame_id != self.body_frame:
                        raise ValueError("MAVROS frame mismatch (expected local/base_link)")
                    if frame == self.world_frame:
                        raise ValueError("MAVROS local and FAST-LIO world need distinct frame names")
                    if self.output_frame and frame != self.output_frame:
                        raise ValueError("output_frame must match MAVROS header.frame_id")
                    if self.local_frame and frame != self.local_frame:
                        raise ValueError("MAVROS local frame changed")
                    self.local_frame = frame
                now = time.monotonic()
                if (self.last_output_at is not None and
                        now - self.received_at[kind] > self.odom_timeout):
                    raise ValueError("odometry reception interrupted after publication")
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                self.buffers[kind].add(PoseSample(
                    stamp, np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])))
                self.received_at[kind] = now
            except (ValueError, TypeError, OverflowError) as error:
                self._fault(kind + ": " + str(error))

    def _on_cloud(self, msg):
        with self.lock:
            if self.fault_reason:
                return
            if msg.header.frame_id != self.world_frame:
                self._fault("cloud is not in configured FAST-LIO world frame")
                return
            stamp = msg.header.stamp.to_nsec()
            if stamp <= 0 or stamp < self.last_cloud_ns:
                self._fault("cloud timestamp is zero or moved backwards")
                return
            if stamp == self.last_cloud_ns:
                return
            self.last_cloud_ns = stamp
            if abs(rospy.Time.now().to_nsec()-stamp) > self.stamp_error*1e9:
                self._drop("cloud timestamp disagrees with system clock")
                return
            if not 0 < msg.width*msg.height <= self.max_points:
                self._drop("empty cloud or max_points exceeded")
                return
            if len(self.pending) >= self.max_pending:
                self.pending.popleft()
                self._drop("pending queue overflow")
            self.pending.append((msg, time.monotonic()))

    def _drop(self, reason):
        self.dropped += 1
        rospy.logwarn_throttle(3, "Cloud dropped: %s (total %d)", reason, self.dropped)

    def _check_extrinsics(self, now):
        if now - self.last_param_check < 1.0:
            return
        self.last_param_check = now
        try:
            enabled = rospy.get_param(self.mapping_ns + "/extrinsic_est_en")
            if type(enabled) is not bool or enabled:
                raise ValueError("FAST-LIO online extrinsic estimation must be explicitly false")
            t_il = vector3(rospy.get_param(self.mapping_ns + "/extrinsic_T"))
            r_il = checked_rotation(rospy.get_param(self.mapping_ns + "/extrinsic_R"))
        except KeyError:
            if self.extrinsics is not None:
                self._fault("running FAST-LIO extrinsic parameters disappeared")
            return
        except (ValueError, TypeError) as error:
            self._fault(str(error))
            return
        if self.extrinsics is None:
            self.extrinsics = (r_il, t_il)
            rospy.loginfo("Using FAST-LIO T_IL from %s: translation=%s m",
                          self.mapping_ns, t_il.tolist())
        elif not (np.array_equal(r_il, self.extrinsics[0]) and
                  np.array_equal(t_il, self.extrinsics[1])):
            self._fault("FAST-LIO extrinsics changed; restart with consistent LIO configuration")

    def step(self):
        """Use monotonic deadlines; ROS time only for measurement timestamps."""
        with self.lock:
            now = time.monotonic()
            ros_ns = rospy.Time.now().to_nsec()
            if self.last_ros_ns is not None:
                ros_elapsed = (ros_ns - self.last_ros_ns)*1e-9
                if abs(ros_elapsed - (now-self.last_clock_at)) > self.clock_tolerance:
                    self._fault("system clock jumped")
            self.last_ros_ns, self.last_clock_at = ros_ns, now
            if self.fault_reason:
                return
            self._check_extrinsics(now)
            if self.fault_reason:
                return

            # Never reuse a stale queued scan when a dependency becomes ready.
            while self.pending and now - self.pending[0][1] > self.cloud_wait:
                self.pending.popleft()
                self._drop("timed out waiting for matching poses")
            if self.extrinsics is None:
                self._status("WAITING: running FAST-LIO /mapping extrinsics", False)
                return
            if not self.connected or now-self.state_at > 2.0:
                self._status("WAITING: fresh MAVROS connected state", False)
                return
            if self.require_vision and not self.vision_healthy:
                self._status("WAITING: /lio_to_mavros/healthy=true", False)
                return
            if any(now-received > self.odom_timeout for received in self.received_at.values()):
                self._status("WAITING: fresh FAST-LIO and MAVROS odometry", False)
                return

            r_il, t_il = self.extrinsics
            while self.pending:
                msg, _ = self.pending[0]
                stamp = msg.header.stamp.to_nsec()
                if abs(ros_ns-stamp) > self.stamp_error*1e9:
                    self.pending.popleft()
                    self._drop("scan became stale while waiting")
                    continue
                # FAST-LIO publishes this scan and /Odometry from the same update.
                # Do not approximate its corrected pose using adjacent updates.
                lio = self.buffers["lio"].lookup(stamp, self.interpolation_gap, exact=True)
                local = self.buffers["mavros"].lookup(stamp, self.interpolation_gap)
                if lio is None or local is None:
                    break
                self.pending.popleft()
                matrix, translation = cloud_transform(lio, local, r_il, t_il, self.r_bl, self.t_bl)
                try:
                    converted = transform_cloud_data(
                        msg.data, msg.fields, msg.width, msg.height, msg.point_step,
                        msg.row_step, msg.is_bigendian, matrix, translation)
                except (ValueError, TypeError, OverflowError) as error:
                    self._drop("invalid PointCloud2: " + str(error))
                    continue
                output = copy.copy(msg)
                output.header = copy.copy(msg.header)
                output.header.frame_id = self.local_frame
                output.data = converted
                # Source stamp, fields, intensity, row padding, width/height retained.
                self.cloud_pub.publish(output)
                self.last_output_at = time.monotonic()
                self.published += 1
                self._status("READY: time-aligned cloud projection (not a flight-health verdict)", True)
                rospy.loginfo_throttle(10, "Cloud projection: published=%d dropped=%d frame=%s",
                                       self.published, self.dropped, self.local_frame)
            if (self.last_output_at is None or
                    time.monotonic()-self.last_output_at > self.output_timeout):
                self._status("WAITING: matched cloud/LIO/MAVROS timestamps", False)

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            time.sleep(0.01)


if __name__ == "__main__":
    rospy.init_node("registered_cloud_to_mavros")
    try:
        RegisteredCloudConverter().run()
    except (ValueError, TypeError) as error:
        rospy.logfatal("Invalid cloud-converter configuration: %s", error)
        raise SystemExit(1)
