#!/usr/bin/env python3
"""Wait for network access, this boot's NTP sync and a stable clock before ROS.

Targets systemd-timesyncd on the Orin. Read-only, no ROS or root required.
This is NOT a runtime network watchdog: disconnects must not stop localization.
"""
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

SYNC_MARKER = Path("/run/systemd/timesync/synchronized")
# A child process bounds DNS as well as connect (socket timeout alone cannot).
TCP_PROBE = (
    "import socket,sys; "
    "s=socket.create_connection((sys.argv[1],int(sys.argv[2])),float(sys.argv[3])); "
    "s.close()"
)


@dataclass
class Settings:
    host: str = "connectivity-check.ubuntu.com"
    port: int = 80
    probe_timeout: float = 5.0
    min_epoch: float = 1735689600.0
    stable_seconds: float = 10.0
    step_tolerance: float = 0.01
    wait_timeout: float = 0.0

    @classmethod
    def from_env(cls):
        value = cls(
            host=os.environ.get("NETWORK_CHECK_HOST", cls.host).strip(),
            port=int(os.environ.get("NETWORK_CHECK_PORT", cls.port)),
            probe_timeout=float(os.environ.get("NETWORK_CHECK_TIMEOUT", cls.probe_timeout)),
            min_epoch=float(os.environ.get("MIN_VALID_EPOCH", cls.min_epoch)),
            stable_seconds=float(os.environ.get("CLOCK_STABLE_SECONDS", cls.stable_seconds)),
            # Separate from the runtime monitor's existing 0.5 s threshold.
            step_tolerance=float(os.environ.get("CLOCK_START_STEP_TOLERANCE", cls.step_tolerance)),
            wait_timeout=float(os.environ.get("CLOCK_WAIT_TIMEOUT", cls.wait_timeout)),
        )
        positive = (value.probe_timeout, value.min_epoch, value.stable_seconds,
                    value.step_tolerance)
        if (not value.host or value.host.startswith("-") or
                not 1 <= value.port <= 65535 or
                any(not math.isfinite(x) or x <= 0 for x in positive) or
                not math.isfinite(value.wait_timeout) or value.wait_timeout < 0):
            raise ValueError("invalid network/clock startup settings")
        return value


def command(argv, timeout):
    try:
        result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, timeout=timeout,
                                env=dict(os.environ, LC_ALL="C"))
        return result.returncode == 0, result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return False, ""


def prerequisites(settings, run=command, marker=SYNC_MARKER):
    ok, _ = run([sys.executable, "-c", TCP_PROBE, settings.host,
                 str(settings.port), str(settings.probe_timeout)], settings.probe_timeout)
    if not ok:
        return False, "network TCP probe failed: {}:{}".format(settings.host, settings.port)
    ok, state = run(["systemctl", "is-active", "systemd-timesyncd.service"], 3.0)
    if not ok or state != "active":
        return False, "systemd-timesyncd is not active (other NTP backends are not supported)"
    # /run is cleared on reboot. The persistent /var/lib/.../clock is NOT proof.
    if not marker.is_file():
        return False, "waiting for this boot's successful systemd-timesyncd synchronization"
    ok, state = run(["timedatectl", "show", "--property=NTPSynchronized", "--value"], 3.0)
    if not ok or state != "yes":
        return False, "waiting for NTPSynchronized=yes"
    return True, "network reachable and NTP synchronized this boot"


class StabilityWindow:
    def __init__(self, settings):
        self.settings = settings
        self.previous = None
        self.since = None

    def update(self, wall, steady, prerequisites_ok):
        step = 0.0
        monotonic_ok = True
        if self.previous is not None:
            previous_wall, previous_steady = self.previous
            step = (wall - previous_wall) - (steady - previous_steady)
            monotonic_ok = steady >= previous_steady
        self.previous = (wall, steady)
        if not prerequisites_ok:
            self.since = None
            return False, "prerequisites unavailable"
        if not math.isfinite(wall) or not math.isfinite(steady) or wall < self.settings.min_epoch:
            self.since = None
            return False, "calendar time is invalid or earlier than the configured minimum"
        if not monotonic_ok or not math.isfinite(step) or abs(step) > self.settings.step_tolerance:
            self.since = None
            return False, "clock step detected ({:+.6f} s); restarting stability window".format(step)
        if self.since is None:
            self.since = steady
        elapsed = steady - self.since
        return elapsed >= self.settings.stable_seconds, "clock stability {:.1f}/{:.1f} s".format(
            elapsed, self.settings.stable_seconds)


def main():
    try:
        settings = Settings.from_env()
    except ValueError as error:
        print("Startup configuration error: {}".format(error), file=sys.stderr, flush=True)
        return 2
    print("Waiting for network {}:{}, this boot's NTP synchronization, then {:.1f} s "
          "of stable clock; ROS is not started.".format(
              settings.host, settings.port, settings.stable_seconds), flush=True)
    started = time.monotonic()
    window = StabilityWindow(settings)
    window.update(time.time(), started, False)
    last_reason, last_report = None, started
    while True:
        ok, reason = prerequisites(settings)
        steady = time.monotonic()
        wall = time.time()
        ready, clock_reason = window.update(wall, steady, ok)
        if settings.wait_timeout and steady - started >= settings.wait_timeout:
            print("Timed out waiting for network/NTP/stable clock", file=sys.stderr, flush=True)
            return 1
        if ready:
            print("Startup gate passed: network reachable; NTP synchronized this boot; "
                  "clock stable. System clock accepted: " +
                  datetime.fromtimestamp(wall).astimezone().isoformat(timespec="seconds"), flush=True)
            return 0
        if ok:
            reason = clock_reason
        if reason != last_reason or steady - last_report >= 30:
            print("Still waiting: " + reason, flush=True)
            last_reason, last_report = reason, steady
        time.sleep(2.0)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
