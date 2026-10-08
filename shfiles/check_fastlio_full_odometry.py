#!/usr/bin/env python3
"""Ground-only checks of full odometry and its same-stamp legacy counterpart."""
import argparse
import collections
import sys
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry


def pose_vector(message):
    p = message.pose.pose.position
    q = message.pose.pose.orientation
    return np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w], dtype=float)


def linear_vector(message):
    v = message.twist.twist.linear
    return np.array([v.x, v.y, v.z], dtype=float)


def check_message(message):
    if message.header.frame_id != "world" or message.child_frame_id != "body":
        raise ValueError("expected world/body frames")
    if message.header.stamp.to_nsec() <= 0:
        raise ValueError("zero/invalid timestamp")
    pose = pose_vector(message)
    w = message.twist.twist.angular
    if not np.isfinite(pose).all() or not np.isfinite(linear_vector(message)).all() or not np.isfinite([w.x, w.y, w.z]).all():
        raise ValueError("non-finite pose/twist")
    if abs(np.linalg.norm(pose[3:]) - 1.0) > 1e-6:
        raise ValueError("non-unit quaternion")
    for name, values in (("pose", message.pose.covariance), ("twist", message.twist.covariance)):
        covariance = np.asarray(values).reshape(6, 6)
        if not np.isfinite(covariance).all() or not np.allclose(covariance, covariance.T, atol=1e-9, rtol=1e-7):
            raise ValueError(name + " covariance is non-finite/asymmetric")
        if np.min(np.diag(covariance)) <= 0 or np.linalg.eigvalsh(covariance).min() < -1e-9:
            raise ValueError(name + " covariance has non-positive diagonal/negative eigenvalue")


def check_pair(full, legacy):
    if full.header.frame_id != legacy.header.frame_id or full.child_frame_id != legacy.child_frame_id:
        raise ValueError("legacy and full frames disagree")
    a, b = pose_vector(full), pose_vector(legacy)
    if not np.allclose(a[:3], b[:3], atol=1e-8, rtol=1e-8):
        raise ValueError("same-stamp position changed")
    if min(np.linalg.norm(a[3:] - b[3:]), np.linalg.norm(a[3:] + b[3:])) > 1e-8:
        raise ValueError("same-stamp orientation changed")
    if not np.allclose(linear_vector(full), linear_vector(legacy), atol=1e-8, rtol=1e-8):
        raise ValueError("same-stamp body linear velocity changed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=60.0, help="wall-clock seconds")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.samples < 1 or not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("samples and timeout must be positive")
    lock = threading.Lock()
    pending = {"full": collections.OrderedDict(), "legacy": collections.OrderedDict()}
    errors, velocities, angular_rates, pose_diagonals, twist_diagonals = [], [], [], [], []
    last_stamp = [None]
    matched = [0]
    complete = threading.Event()
    # init_node precedes any subscriptions; timeout uses monotonic time even if ROS time jumps.
    rospy.init_node("check_fastlio_full_odometry", anonymous=True)

    def receive(message, kind):
        with lock:
            if complete.is_set():
                return
            key = message.header.stamp.to_nsec()
            try:
                if kind == "full":
                    check_message(message)
                    if last_stamp[0] is not None and key <= last_stamp[0]:
                        raise ValueError("full timestamp repeated or moved backwards")
                    last_stamp[0] = key
                pending[kind][key] = message
                other = "legacy" if kind == "full" else "full"
                if key in pending[other]:
                    full = pending["full"].pop(key)
                    legacy = pending["legacy"].pop(key)
                    check_pair(full, legacy)
                    velocities.append(np.linalg.norm(linear_vector(full)))
                    w = full.twist.twist.angular
                    angular_rates.append(np.linalg.norm([w.x, w.y, w.z]))
                    pose_diagonals.append(np.asarray(full.pose.covariance).reshape(6, 6).diagonal())
                    twist_diagonals.append(np.asarray(full.twist.covariance).reshape(6, 6).diagonal())
                    matched[0] += 1
                    if matched[0] >= args.samples:
                        complete.set()
                while len(pending[kind]) > 1000:
                    pending[kind].popitem(last=False)
            except ValueError as error:
                errors.append(str(error))
                complete.set()

    full_subscriber = rospy.Subscriber("/fast_lio/odometry_full", Odometry, receive, callback_args="full", queue_size=200)
    legacy_subscriber = rospy.Subscriber("/Odometry", Odometry, receive, callback_args="legacy", queue_size=200)
    deadline = time.monotonic() + args.timeout
    while not rospy.is_shutdown() and not complete.is_set() and time.monotonic() < deadline:
        complete.wait(0.1)
    full_subscriber.unregister()
    legacy_subscriber.unregister()
    with lock:
        if errors:
            print("FAIL: " + errors[0])
            return 1
        if matched[0] < args.samples:
            print("FAIL: timeout/shutdown; matched {}/{} frames. Check enabled flag, IMU gaps and laserMapping warnings.".format(matched[0], args.samples))
            return 1
        print("PASS: {} same-stamp frames; world/body, pose and linear velocity match; covariance finite, symmetric and PSD.".format(matched[0]))
        print("Speed range: {:.4f}..{:.4f} m/s; angular rate: {:.4f}..{:.4f} rad/s".format(min(velocities), max(velocities), min(angular_rates), max(angular_rates)))
        print("Minimum pose stddev [m,m,m,rad,rad,rad]:", np.sqrt(np.min(pose_diagonals, axis=0)))
        print("Minimum twist stddev [m/s,m/s,m/s,rad/s,rad/s,rad/s]:", np.sqrt(np.min(twist_diagonals, axis=0)))
        print("This checks message consistency only, NOT physical accuracy, health gating or PX4 fusion readiness.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
