#!/usr/bin/env bash
set -euo pipefail

TIMEOUT_SEC="${1:-120}"

wait_for_topic() {
    local topic="$1"
    local deadline=$((SECONDS + TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        if timeout 4 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
            sleep 1
            if timeout 4 rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
                echo "[ready] $topic"
                return 0
            fi
        fi
        sleep 1
    done
    echo "[timeout] no message on $topic within ${TIMEOUT_SEC}s" >&2
    return 1
}

wait_for_mavros() {
    local deadline=$((SECONDS + TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        local state
        state="$(timeout 4 rostopic echo -n 1 /mavros/state 2>/dev/null || true)"
        if printf '%s\n' "$state" | grep -qiE 'connected:[[:space:]]*true'; then
            echo "[ready] MAVROS connected to PX4"
            return 0
        fi
        sleep 1
    done
    echo "[timeout] MAVROS did not report connected=true within ${TIMEOUT_SEC}s" >&2
    return 1
}

echo "Waiting for MAVROS/PX4 and sensor topics..."
wait_for_mavros

# Keep the two PX4 stream requests used by the original ready_go.sh.
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
sleep 1
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0

wait_for_topic /livox/lidar
wait_for_topic /livox/imu
wait_for_topic /Odometry
wait_for_topic /mavros/vision_pose/pose
wait_for_topic /mavros/local_position/odom
echo "[ready] GrampusDrone onboard topics are available."
