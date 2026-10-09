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

wait_for_system_time() {
    local deadline=$((SECONDS + TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        local health
        health="$(timeout 4 rostopic echo -n 1 /system_time/healthy 2>/dev/null || true)"
        if printf '%s\n' "$health" | grep -qiE 'data:[[:space:]]*true'; then
            echo "[ready] system clock monitor is healthy"
            return 0
        fi
        sleep 1
    done
    echo "[timeout] system clock monitor did not report healthy" >&2
    return 1
}

wait_for_healthy_odometry() {
    local deadline=$((SECONDS + TIMEOUT_SEC))
    while (( SECONDS < deadline )); do
        local health
        health="$(timeout 4 rostopic echo -n 1 /lio_to_mavros/healthy 2>/dev/null || true)"
        if printf '%s\n' "$health" | grep -qiE 'data:[[:space:]]*true'; then
            echo "[ready] FAST-LIO odometry gate is healthy"
            return 0
        fi
        sleep 1
    done
    echo "[timeout] FAST-LIO odometry gate did not become healthy" >&2
    return 1
}

echo "Waiting for MAVROS/PX4 and sensor topics..."
wait_for_mavros
if [[ "${REQUIRE_SYSTEM_TIME_MONITOR:-false}" == "true" ]]; then
    wait_for_system_time
fi

# Keep the two PX4 stream requests used by the original ready_go.sh.
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0
sleep 1
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0

wait_for_topic /livox/lidar
wait_for_topic /livox/imu
LIO_MODE="$(rosparam get /lio_to_mavros/output_mode)"
case "$LIO_MODE" in
    vision_pose)
        wait_for_topic /Odometry
        wait_for_healthy_odometry
        wait_for_topic /mavros/vision_pose/pose
        ;;
    odometry)
        wait_for_topic "$(rosparam get /lio_to_mavros/full/input_topic)"
        wait_for_healthy_odometry
        wait_for_topic "$(rosparam get /lio_to_mavros/full/output_topic)"
        ;;
    *) echo "[error] unknown lio_to_mavros output_mode: $LIO_MODE" >&2; exit 1 ;;
esac
wait_for_topic /mavros/local_position/odom
echo "[ready] GrampusDrone onboard topics are available."
