#!/usr/bin/env bash
# Invoke with bash. Uses the existing isolated SDK; no catkin/SDK rebuild or sudo.
set -euo pipefail
mode="${1:-all}"
serial="${2:-}"
if [[ $# -gt 2 || ! "$mode" =~ ^(all|imu|ir)$ ]]; then
    echo 'Usage: bash run_d455_sdk_probe.sh [all|imu|ir] [serial]' >&2
    exit 2
fi
if [[ "$(uname -s)" != Linux ]]; then
    echo 'Run this diagnostic on the Ubuntu machine connected to the D455.' >&2
    exit 2
fi
prefix="${GRAMPUS_RS_SDK_PREFIX:-/opt/grampus/librealsense-2.55.1}"
if [[ "$prefix" != /* ]]; then
    echo 'GRAMPUS_RS_SDK_PREFIX must be an absolute SDK installation path.' >&2
    exit 2
fi
source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for tool in g++ timeout ldd tee; do
    command -v "$tool" >/dev/null || { echo "Missing command: $tool" >&2; exit 2; }
done
[[ -f "$prefix/include/librealsense2/rs.hpp" && -e "$prefix/lib/librealsense2.so" ]] || {
    echo "SDK headers/library missing under $prefix; stop and check the installation." >&2
    exit 2
}
mkdir -p "$HOME/grampus_vio_deps/d455_diagnostics"
output_dir="$(mktemp -d "$HOME/grampus_vio_deps/d455_diagnostics/${mode}_XXXXXXXX")"
exec > >(tee "$output_dir/report.log") 2>&1
echo "Report: $output_dir/report.log"
echo "SDK prefix: $prefix"
date -Is
uname -a
if [[ -r /etc/nv_tegra_release ]]; then head -n 1 /etc/nv_tegra_release; fi
if command -v lsusb >/dev/null; then lsusb -t || true; fi
echo 'Compile standalone probe against the existing SDK (no ROS).'
g++ -std=c++14 -O2 -Wall -Wextra -pthread \
    -I"$prefix/include" "$source_dir/d455_sdk_probe.cpp" \
    -L"$prefix/lib" -Wl,-rpath,"$prefix/lib" -lrealsense2 \
    -o "$output_dir/d455_sdk_probe"
echo 'Dynamic libraries (same environment as the probe):'
env -u LD_LIBRARY_PATH -u LD_PRELOAD ldd "$output_dir/d455_sdk_probe"
echo 'Start one test; 45-second limit covers initialization, receive and cleanup.'
set +e
env -u LD_LIBRARY_PATH -u LD_PRELOAD \
    timeout --signal=TERM --kill-after=5s 45s \
    "$output_dir/d455_sdk_probe" "$mode" "$serial"
status=$?
set -e
echo "PROBE_EXIT_CODE=$status"
if [[ $status -eq 124 || $status -eq 137 ]]; then
    echo 'Probe timed out or was killed. Inspect the last STAGE line and preceding SDK errors.'
fi
echo "Report saved: $output_dir/report.log"
exit "$status"
