#!/usr/bin/env python3
"""Read-only same-stamp check of the full bridge (preview by default)."""
import argparse
import collections
import sys
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry


def rotation(q):
    q = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-5:
        raise ValueError("invalid output quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def vector(v):
    return np.array([v.x, v.y, v.z])


def hat(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/lio_to_mavros/odometry_preview")
    parser.add_argument("--node", default="/lio_to_mavros")
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=60)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.samples < 2 or not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("samples >= 2 and finite positive timeout required")
    rospy.init_node("check_lio_bridge_odometry", anonymous=True)
    cfg = rospy.get_param(args.node + "/full")
    mapping = rospy.get_param(cfg["mapping_namespace"])
    if mapping["extrinsic_est_en"]:
        raise ValueError("online extrinsic estimation is enabled")
    r_bl = np.asarray(cfg["lidar_in_base"]["rotation"]).reshape(3, 3)
    t_bl = np.asarray(cfg["lidar_in_base"]["translation"])
    r_il = np.asarray(mapping["extrinsic_R"]).reshape(3, 3)
    t_il = np.asarray(mapping["extrinsic_T"])
    r_ib, lever = r_il @ r_bl.T, t_il - r_il @ r_bl.T @ t_bl
    preview = rospy.get_param(args.node + "/output_mode", "vision_pose") == "vision_pose"
    expected_frames = (("lio_preview_odom", "lio_preview_base_link") if preview else
                       (cfg["output_frame"], cfg["output_child_frame"]))
    buffers = [collections.OrderedDict(), collections.OrderedDict()]
    lock, complete = threading.Lock(), threading.Event()
    errors, alignment, stamps, speeds, rates = [], [], [], [], []
    last = [0]

    def receive(m, kind):
        with lock:
            if complete.is_set():
                return
            try:
                stamp = m.header.stamp.to_nsec()
                if kind == 1:
                    if stamp <= last[0]:
                        raise ValueError("output stamp is zero, duplicated or backwards")
                    last[0] = stamp
                    if (m.header.frame_id, m.child_frame_id) != expected_frames:
                        raise ValueError("unexpected output frames")
                buffers[kind][stamp] = m
                if stamp in buffers[1-kind]:
                    src, out = buffers[0].pop(stamp), buffers[1].pop(stamp)
                    r_wi, r_ab = rotation(src.pose.pose.orientation), rotation(out.pose.pose.orientation)
                    p_wb = vector(src.pose.pose.position) + r_wi @ lever
                    p_ab = vector(out.pose.pose.position)
                    if not alignment:
                        r_aw = r_ab @ r_ib.T @ r_wi.T
                        alignment.extend((r_aw, p_ab-r_aw @ p_wb))
                    r_aw, t_aw = alignment
                    checks = [(p_ab, r_aw @ p_wb+t_aw), (r_ab, r_aw @ r_wi @ r_ib),
                        (vector(out.twist.twist.linear), r_ib.T @ (vector(src.twist.twist.linear)+np.cross(vector(src.twist.twist.angular), lever))),
                        (vector(out.twist.twist.angular), r_ib.T @ vector(src.twist.twist.angular))]
                    jp, jt = np.zeros((6, 6)), np.zeros((6, 6))
                    jp[:3, :3] = jp[3:, 3:] = r_aw
                    jp[:3, 3:] = -r_aw @ hat(r_wi @ lever)
                    jt[:3, :3] = jt[3:, 3:] = r_ib.T
                    jt[:3, 3:] = -r_ib.T @ hat(lever)
                    for source, target, jacobian in ((src.pose, out.pose, jp), (src.twist, out.twist, jt)):
                        cov = np.asarray(target.covariance).reshape(6, 6)
                        if not np.isfinite(cov).all() or not np.allclose(cov, cov.T, atol=1e-9):
                            raise ValueError("non-finite/asymmetric covariance")
                        if np.linalg.eigvalsh(cov).min() < -1e-9 or np.min(cov.diagonal()) <= 0:
                            raise ValueError("invalid covariance spectrum")
                        checks.append((cov, jacobian @ np.asarray(source.covariance).reshape(6, 6) @ jacobian.T))
                    for actual, expected in checks:
                        if not np.allclose(actual, expected, atol=1e-7, rtol=1e-6):
                            raise ValueError("pose/twist/covariance transform mismatch")
                    stamps.append(stamp)
                    speeds.append(np.linalg.norm(vector(out.twist.twist.linear)))
                    rates.append(np.linalg.norm(vector(out.twist.twist.angular)))
                    if len(stamps) >= args.samples:
                        complete.set()
                while len(buffers[kind]) > 1000:
                    buffers[kind].popitem(last=False)
            except (ValueError, np.linalg.LinAlgError) as error:
                errors.append(str(error))
                complete.set()

    subs = [rospy.Subscriber(cfg["input_topic"], Odometry, receive, callback_args=0, queue_size=200),
            rospy.Subscriber(args.topic, Odometry, receive, callback_args=1, queue_size=200)]
    deadline = time.monotonic() + args.timeout
    while not rospy.is_shutdown() and not complete.is_set() and time.monotonic() < deadline:
        complete.wait(.1)
    for sub in subs:
        sub.unregister()
    with lock:
        if errors or len(stamps) < args.samples:
            print("FAIL:", errors[0] if errors else "timeout; matched {}/{}".format(len(stamps), args.samples))
            return 1
        print("PASS: {} same-stamp frames; fixed reference, lever arm, twist and covariance consistent".format(len(stamps)))
        print("Measurement rate: {:.2f} Hz; speed {:.4f}..{:.4f} m/s; angular {:.4f}..{:.4f} rad/s".format(
            (len(stamps)-1)*1e9/(stamps[-1]-stamps[0]), min(speeds), max(speeds), min(rates), max(rates)))
        print("Reference transform inferred from first pair; this does not verify absolute heading, mounting accuracy or PX4 fusion.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
