#!/usr/bin/env python3

import os
import signal
import sys
import time

import rospy
from mavros_msgs.msg import State
from std_msgs.msg import Bool


STEP_TOLERANCE = float(os.environ.get("CLOCK_STEP_TOLERANCE", "0.5"))
STATE_TIMEOUT = 2.0
RESTART_EXIT_CODE = 42


class SystemTimeMonitor:
    def __init__(self):
        self.armed = False
        self.connected = False
        self.have_state = False
        self.state_received_at = 0.0
        self.clock_fault = False
        self.previous_wall = time.time()
        self.previous_monotonic = time.monotonic()
        self.health_pub = rospy.Publisher(
            "/system_time/healthy", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("/mavros/state", State, self.on_state, queue_size=5)
        self.health_pub.publish(Bool(data=True))

    def on_state(self, message):
        self.armed = message.armed
        self.connected = message.connected
        self.have_state = True
        self.state_received_at = time.monotonic()

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)
            wall_now = time.time()
            monotonic_now = time.monotonic()
            wall_elapsed = wall_now - self.previous_wall
            monotonic_elapsed = monotonic_now - self.previous_monotonic
            difference = abs(wall_elapsed - monotonic_elapsed)

            if difference > STEP_TOLERANCE:
                self.clock_fault = True
                self.health_pub.publish(Bool(data=False))
                rospy.logerr(
                    "System clock jumped by approximately %.3f seconds; "
                    "position output must no longer be trusted",
                    wall_elapsed - monotonic_elapsed,
                )

            self.previous_wall = wall_now
            self.previous_monotonic = monotonic_now

            state_is_fresh = (
                self.have_state
                and monotonic_now - self.state_received_at <= STATE_TIMEOUT
            )
            if self.clock_fault and state_is_fresh and self.connected and not self.armed:
                rospy.logerr(
                    "PX4 is confirmed disarmed; requesting a restart of the "
                    "complete onboard stack"
                )
                # Interrupt start_onboard.sh immediately. Its trap terminates
                # roslaunch, and systemd then starts the complete service again.
                os.kill(os.getppid(), signal.SIGTERM)
                return RESTART_EXIT_CODE

            if self.clock_fault and (not state_is_fresh or self.armed):
                rospy.logerr_throttle(
                    5.0,
                    "Clock fault remains latched. Automatic restart is blocked "
                    "while PX4 is armed or its state is unknown.",
                )

        return 0


if __name__ == "__main__":
    rospy.init_node("system_time_monitor")
    sys.exit(SystemTimeMonitor().run())
