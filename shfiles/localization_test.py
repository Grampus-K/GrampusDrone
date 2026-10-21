#!/usr/bin/env python3
"""Collect and analyze a GrampusDrone localization test run.

The ``record`` command replaces the manual section 3.4 procedure.  It keeps
rosbag in the foreground so Ctrl-C stops the bag and writes the end-of-run
metadata.  The ``analyze`` command is intentionally conservative: it checks
the recorded data path and reports the measurements needed for the flight
decision, but it cannot decide whether a pilot observed a safe hover.
"""

import argparse
import datetime as _datetime
import json
import math
import re
import signal
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TOPICS = (
    "/fast_lio/odometry_full",
    "/mavros/odometry/out",
    "/lio_to_mavros/healthy",
    "/lio_to_mavros/status",
    "/system_time/healthy",
    "/mavros/state",
    "/mavros/local_position/odom",
    "/mavros/imu/data",
    "/mavros/global_position/global",
    "/mavros/global_position/raw/fix",
    "/mavros/timesync_status",
    "/rosout_agg",
)
CORE_ODOMETRY = ("/fast_lio/odometry_full", "/mavros/odometry/out")


def now_iso():
    return _datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def write_text(path, text):
    Path(path).write_text(str(text), encoding="utf-8")


def run_capture(command, path, cwd=ROOT, sudo=False):
    """Run a command and save combined output; return its exit code."""
    command = list(command)
    if sudo:
        command = ["sudo", "-n"] + command
    try:
        result = subprocess.run(
            command, cwd=str(cwd), stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, check=False,
        )
        write_text(path, result.stdout)
        return result.returncode
    except OSError as error:
        write_text(path, "command failed: {}\n{}\n".format(" ".join(command), error))
        return 127


def prepare_run_dir(test_id, root):
    stamp = _datetime.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    safe_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", test_id).strip("_.") or "test"
    run_dir = Path(root).expanduser() / (stamp + "_" + safe_id)
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def capture_start(run_dir, use_sudo):
    write_text(run_dir / "start-time.txt", now_iso() + "\n")
    run_capture(["git", "rev-parse", "HEAD"], run_dir / "commit.txt")
    run_capture(["git", "status", "--short", "--branch"], run_dir / "git-status.txt")
    run_capture(["cat", "/proc/sys/kernel/random/boot_id"], run_dir / "boot-id.txt")
    run_capture(["hwclock", "--show"], run_dir / "rtc.txt", sudo=use_sudo)
    run_capture(["systemctl", "cat", "grampusdrone.service", "--no-pager"],
                run_dir / "service.txt", sudo=use_sudo)
    run_capture([
        "systemctl", "show", "grampusdrone.service", "--no-pager",
        "-p", "Environment", "-p", "EnvironmentFiles", "-p", "NRestarts",
        "-p", "ActiveEnterTimestamp", "-p", "MainPID",
    ], run_dir / "service-state.txt", sudo=use_sudo)
    run_capture(["rosparam", "dump", str(run_dir / "rosparams.yaml")],
                run_dir / "rosparam-dump.txt")
    run_capture(["df", "-h", str(Path.home())], run_dir / "disk.txt")


def capture_end(run_dir, use_sudo):
    write_text(run_dir / "end-time.txt", now_iso() + "\n")
    run_capture(["journalctl", "-u", "grampusdrone.service", "-b", "--no-pager"],
                run_dir / "service-journal.txt", sudo=use_sudo)
    run_capture(["systemctl", "show", "grampusdrone.service", "--no-pager",
                 "-p", "NRestarts", "-p", "MainPID", "-p", "Result",
                 "-p", "ExecMainStatus"], run_dir / "restarts-after.txt",
                sudo=use_sudo)
    bag = run_dir / "ros.bag"
    if bag.exists():
        run_capture(["rosbag", "info", str(bag)], run_dir / "bag-info.txt")


def stop_process(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def record(args):
    run_dir = prepare_run_dir(args.test_id, args.log_root)
    print("记录目录: {}".format(run_dir))
    if args.sudo:
        # Request credentials once, before the long rosbag run.  A failure is
        # recorded by the individual metadata commands and does not lose the bag.
        subprocess.run(["sudo", "-v"], check=False)
    capture_start(run_dir, args.sudo)
    bag_path = run_dir / "ros.bag"
    command = ["rosbag", "record", "-O", str(bag_path)] + list(args.topics)
    print("开始 rosbag 记录；测试结束时按 Ctrl-C。")
    process = None
    try:
        process = subprocess.Popen(command, cwd=str(ROOT))
        if args.duration > 0:
            process.wait(timeout=args.duration)
        else:
            process.wait()
    except KeyboardInterrupt:
        print("收到 Ctrl-C，正在停止 rosbag...")
    except subprocess.TimeoutExpired:
        print("达到记录时长，正在停止 rosbag...")
    finally:
        if process is not None:
            stop_process(process)
        capture_end(run_dir, args.sudo)
    print("记录完成: {}".format(run_dir))
    print("分析命令: python3 shfiles/localization_test.py analyze {}".format(run_dir))
    return 0


def stamp_ns(message, bag_time):
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is not None:
        try:
            return int(stamp.secs) * 1000000000 + int(stamp.nsecs)
        except (AttributeError, TypeError, ValueError):
            pass
    return int(round(float(bag_time) * 1e9))


def finite_numbers(values):
    return all(math.isfinite(float(value)) for value in values)


def vector3(value):
    return [float(getattr(value, axis)) for axis in ("x", "y", "z")]


def odometry_values(message):
    pose = message.pose.pose.position
    linear = message.twist.twist.linear
    angular = message.twist.twist.angular
    position = vector3(pose)
    velocity = vector3(linear)
    rate = vector3(angular)
    return position, velocity, rate


def empty_topic():
    return {"count": 0, "stamps": [], "bag_times": [], "false": 0,
            "fault_status": 0, "connected_false": 0, "armed": 0,
            "positions": [], "speeds": [], "angular_rates": [],
            "last_status": "", "finite": True}


def read_bag(path):
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError("需要在 Ubuntu 的 ROS 环境运行 analyze（无法导入 rosbag: {})".format(error))
    data = {topic: empty_topic() for topic in DEFAULT_TOPICS}
    try:
        bag = rosbag.Bag(str(path), "r")
    except Exception as error:
        raise RuntimeError("无法打开 rosbag: {}".format(error))
    try:
        for topic, message, bag_time in bag.read_messages():
            if topic not in data:
                continue
            item = data[topic]
            t = float(bag_time.to_sec()) if hasattr(bag_time, "to_sec") else float(bag_time)
            stamp = stamp_ns(message, t)
            item["count"] += 1
            item["stamps"].append(stamp)
            item["bag_times"].append(t)
            if topic.endswith("/healthy") or topic == "/system_time/healthy":
                if not bool(getattr(message, "data", False)):
                    item["false"] += 1
            elif topic.endswith("/status"):
                text = str(getattr(message, "data", ""))
                item["last_status"] = text
                if "FAULT" in text.upper():
                    item["fault_status"] += 1
            elif topic == "/mavros/state":
                if not bool(getattr(message, "connected", False)):
                    item["connected_false"] += 1
                if bool(getattr(message, "armed", False)):
                    item["armed"] += 1
            elif topic in CORE_ODOMETRY or topic == "/mavros/local_position/odom":
                try:
                    position, velocity, rate = odometry_values(message)
                    values = position + velocity + rate
                    item["finite"] = item["finite"] and finite_numbers(values)
                    item["positions"].append(position)
                    item["speeds"].append(math.sqrt(sum(x * x for x in velocity)))
                    item["angular_rates"].append(math.sqrt(sum(x * x for x in rate)))
                except (AttributeError, TypeError, ValueError):
                    item["finite"] = False
    finally:
        bag.close()
    return data


def topic_metrics(item):
    stamps = item["stamps"]
    deltas = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:]) if b > a]
    backwards = sum(1 for a, b in zip(stamps, stamps[1:]) if b <= a)
    return {
        "count": item["count"],
        "duration_s": ((stamps[-1] - stamps[0]) / 1e9 if len(stamps) > 1 else 0.0),
        "rate_hz": ((len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)
                    if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0),
        "max_gap_s": max(deltas) if deltas else 0.0,
        "backwards_or_duplicate": backwards,
    }


def parse_restart_count(path):
    try:
        text = Path(path).read_text(encoding="utf-8", errors="replace")
    except OSError:
        return None
    match = re.search(r"^NRestarts=(\d+)\s*$", text, re.MULTILINE)
    return int(match.group(1)) if match else None


def position_summary(values):
    if not values:
        return None
    columns = list(zip(*values))
    minimum = [min(column) for column in columns]
    maximum = [max(column) for column in columns]
    return {
        "min_m": minimum,
        "max_m": maximum,
        "extent_m": [high - low for low, high in zip(minimum, maximum)],
    }


def analyze(args):
    run_dir = Path(args.run_dir).expanduser().resolve()
    bag_path = run_dir / "ros.bag"
    if not bag_path.exists():
        print("FAIL: 找不到 {}".format(bag_path))
        return 1
    try:
        data = read_bag(bag_path)
    except RuntimeError as error:
        print("FAIL: {}".format(error))
        return 1
    metrics = {topic: topic_metrics(item) for topic, item in data.items()}
    checks = []

    def check(name, passed, detail):
        checks.append({"name": name, "pass": bool(passed), "detail": detail})

    for topic in CORE_ODOMETRY:
        item = data[topic]
        m = metrics[topic]
        check("存在 {}".format(topic), item["count"] > 1,
              "{} 条, {:.2f} Hz".format(item["count"], m["rate_hz"]))
        check("{} 频率".format(topic), 9.0 <= m["rate_hz"] <= 11.0,
              "{:.2f} Hz".format(m["rate_hz"]))
        check("{} 无长间隔".format(topic), 0 < m["max_gap_s"] <= 0.5,
              "最大间隔 {:.3f} s".format(m["max_gap_s"]))
        check("{} 时间单调".format(topic), m["backwards_or_duplicate"] == 0,
              "重复/倒退 {} 次".format(m["backwards_or_duplicate"]))
        check("{} 数值有限".format(topic), item["finite"], "位置、速度、角速度")

    input_stamps = set(data[CORE_ODOMETRY[0]]["stamps"])
    output_stamps = set(data[CORE_ODOMETRY[1]]["stamps"])
    matched = len(input_stamps & output_stamps)
    ratio = matched / float(len(output_stamps)) if output_stamps else 0.0
    check("输入输出时间戳匹配", bool(output_stamps) and ratio >= 0.95,
          "匹配 {}/{} ({:.1f}%)".format(matched, len(output_stamps), ratio * 100.0))

    for topic in ("/lio_to_mavros/healthy", "/system_time/healthy"):
        item = data[topic]
        check("{} 始终健康".format(topic), item["count"] > 0 and item["false"] == 0,
              "{} 条，False {} 次".format(item["count"], item["false"]))
    status = data["/lio_to_mavros/status"]
    check("桥接状态无 FAULT", status["count"] > 0 and status["fault_status"] == 0,
          "FAULT {} 次".format(status["fault_status"]))
    check("桥接最后状态 READY", "READY" in status["last_status"].upper(),
          status["last_status"] or "未记录状态")
    state = data["/mavros/state"]
    check("MAVROS 始终连接", state["count"] > 0 and state["connected_false"] == 0,
          "连接 False {} 次".format(state["connected_false"]))
    local_position = data["/mavros/local_position/odom"]
    check("PX4 local_position 有记录", local_position["count"] > 1,
          "{} 条".format(local_position["count"]))

    before = parse_restart_count(run_dir / "service-state.txt")
    after = parse_restart_count(run_dir / "restarts-after.txt")
    if before is not None and after is not None:
        check("记录期间 systemd 无重启", after == before,
              "NRestarts {} -> {}".format(before, after))
    else:
        checks.append({"name": "记录期间 systemd 无重启", "pass": None,
                       "detail": "缺少 service-state.txt 或 restarts-after.txt"})

    def range_text(values, unit):
        return "未记录" if not values else "{:.4f}..{:.4f} {}".format(min(values), max(values), unit)

    result = {
        "run_dir": str(run_dir),
        "generated_at": now_iso(),
        "metrics": metrics,
        "input_output_matches": matched,
        "input_output_match_ratio": ratio,
        "speed_range_mps": range_text(data[CORE_ODOMETRY[1]]["speeds"], "m/s"),
        "angular_rate_range_rads": range_text(data[CORE_ODOMETRY[1]]["angular_rates"], "rad/s"),
        "output_position": position_summary(data[CORE_ODOMETRY[1]]["positions"]),
        "local_position": position_summary(local_position["positions"]),
        "checks": checks,
        "automatic_pass": all(item["pass"] is True for item in checks),
        "manual_review": [
            "确认飞手观察到的悬停、运动、接管和降落行为",
            "检查 QGC/PX4 ULog 中期望的 EV 融合、创新量和 reset counters",
            "确认绝对航向、安装准确性和 GPS/EV 切换符合本次测试项目",
        ],
    }
    write_text(run_dir / "analysis.json", json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print("测试目录: {}".format(run_dir))
    print("自动检查: {}".format("PASS" if result["automatic_pass"] else "FAIL"))
    for item in checks:
        mark = "PASS" if item["pass"] is True else ("FAIL" if item["pass"] is False else "SKIP")
        print("[{}] {}: {}".format(mark, item["name"], item["detail"]))
    print("速度范围: {}".format(result["speed_range_mps"]))
    print("角速度范围: {}".format(result["angular_rate_range_rads"]))
    if result["output_position"]:
        print("正式输出位置范围(m): min={} max={} extent={}".format(
            [round(value, 4) for value in result["output_position"]["min_m"]],
            [round(value, 4) for value in result["output_position"]["max_m"]],
            [round(value, 4) for value in result["output_position"]["extent_m"]]))
    if result["local_position"]:
        print("PX4 local_position 范围(m): min={} max={} extent={}".format(
            [round(value, 4) for value in result["local_position"]["min_m"]],
            [round(value, 4) for value in result["local_position"]["max_m"]],
            [round(value, 4) for value in result["local_position"]["extent_m"]]))
    print("注意: 自动 PASS 只表示记录链路和数据完整性通过；飞行行为、PX4 融合和安全门槛仍需人工/QGC/ULog 判定。")
    return 0 if result["automatic_pass"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    record_parser = sub.add_parser("record", help="创建目录、采集元数据并记录 rosbag")
    record_parser.add_argument("--test-id", required=True, help="例如 001C_ev_hover")
    record_parser.add_argument("--log-root", default="~/flight_test_logs")
    record_parser.add_argument("--duration", type=float, default=0,
                               help="秒数；默认一直记录到 Ctrl-C")
    record_parser.add_argument("--no-sudo", action="store_true",
                               help="不调用 sudo 采集 systemd/journal/RTC")
    record_parser.add_argument("--topics", nargs="+", default=list(DEFAULT_TOPICS))
    record_parser.set_defaults(func=lambda a: record(a))
    record_parser.set_defaults(sudo=True)
    analyze_parser = sub.add_parser("analyze", help="分析已完成的 rosbag 记录")
    analyze_parser.add_argument("run_dir")
    analyze_parser.set_defaults(func=analyze)
    args = parser.parse_args()
    if getattr(args, "no_sudo", False):
        args.sudo = False
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
