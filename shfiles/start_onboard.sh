#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="/home/orin/GrampusDrone"
FCU_DEVICE="${FCU_DEVICE:-/dev/ttyACM0}"
FCU_BAUD="${FCU_BAUD:-57600}"
START_CONTROLLER="${START_CONTROLLER:-false}"
START_PLANNER="${START_PLANNER:-false}"

source /opt/ros/noetic/setup.bash
source /home/orin/livox_ws/devel/setup.bash
source "${WORKSPACE}/devel/setup.bash"

if [[ ! -e "${FCU_DEVICE}" ]]; then
    echo "PX4 serial device ${FCU_DEVICE} does not exist" >&2
    exit 1
fi

roslaunch px4ctrl onboard_stack.launch \
    fcu_url:="${FCU_DEVICE}:${FCU_BAUD}" \
    start_controller:="${START_CONTROLLER}" \
    start_planner:="${START_PLANNER}" &
LAUNCH_PID=$!

cleanup() {
    kill "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

"${WORKSPACE}/shfiles/wait_for_stack.sh" 120
wait "${LAUNCH_PID}"
