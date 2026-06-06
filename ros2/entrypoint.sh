#!/bin/bash
# Production entrypoint for drone-control container.
#
# Supervises two long-running processes — mavros_node and the FastAPI
# api_gateway — as a single unit. If either one dies, the entrypoint
# tears down its peer and exits, and Docker's restart policy
# (`restart: unless-stopped`) brings the whole container back cleanly.
#
# This replaces the previous unsupervised pattern:
#   ros2 run mavros mavros_node ... &
#   sleep 5 && python3 api_gateway.py
# which left mavros_node as an unmonitored background job — when it
# crashed (e.g. during an FC reboot reconnect storm), the container
# stayed up but with a dead MAVROS, breaking every service call.

# NOTE: do not use `set -u` — the ROS setup script references several
# unset AMENT_* variables and would abort under nounset.
source /opt/ros/humble/setup.bash

# MAVROS_REQUIRED (default=1): when 0, run api_gateway alone — useful
# during development when the FC isn't plugged in (composite-out / VTX /
# camera work). Production should leave it unset or set to 1 so a missing
# FC fails loudly via the Docker restart cycle.
MAVROS_REQUIRED="${MAVROS_REQUIRED:-1}"

if [ "$MAVROS_REQUIRED" = "0" ]; then
    echo "[entrypoint] MAVROS_REQUIRED=0 — skipping FC + MAVROS, running api_gateway alone"
    exec python3 /ros2_ws/src/drone_control/drone_control/api_gateway.py
fi

# Resolve the FC device. Supports Pixhawk 2.4.8 (ArduPilot generic
# descriptor) and CubeOrange+ (CubePilot descriptor); picks the first
# match. The `-if00` suffix is the MAVLink data port. We retry briefly
# so a USB hiccup or bootloader→firmware transition doesn't fail the
# container before the FC has finished enumerating.
shopt -s nullglob
FC_DEV=""
for _ in $(seq 1 15); do
    matches=(
        /dev/serial/by-id/usb-CubePilot_CubeOrange+_*-if00
        /dev/serial/by-id/usb-ArduPilot_Pixhawk1_*-if00
        /dev/serial/by-id/usb-ArduPilot_MicoAir743v2_*-if00
    )
    if [ ${#matches[@]} -gt 0 ]; then
        FC_DEV="${matches[0]}"
        break
    fi
    sleep 1
done
shopt -u nullglob
if [ -z "$FC_DEV" ]; then
    echo "[entrypoint] ERROR: no flight controller at /dev/serial/by-id/" >&2
    exit 1
fi

echo "[entrypoint] starting mavros_node (fc=$FC_DEV)"
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:=serial://"$FC_DEV":115200 \
    -p system_id:=255 &
MAVROS_PID=$!

# Let MAVROS register on the ROS graph before the api_gateway opens its
# subscriptions and service clients against it.
sleep 5

echo "[entrypoint] starting api_gateway (mavros pid=$MAVROS_PID)"
python3 /ros2_ws/src/drone_control/drone_control/api_gateway.py &
API_PID=$!

# Forward SIGTERM/SIGINT (e.g. from `docker stop`) to both children so the
# container shuts down promptly instead of waiting for the grace timeout.
trap 'echo "[entrypoint] forwarding shutdown"; kill -TERM "$MAVROS_PID" "$API_PID" 2>/dev/null' TERM INT

echo "[entrypoint] supervising (mavros=$MAVROS_PID api=$API_PID)"

# Block until either child terminates.
wait -n "$MAVROS_PID" "$API_PID"
EXIT_CODE=$?

if ! kill -0 "$MAVROS_PID" 2>/dev/null; then
    DEAD="mavros_node"
elif ! kill -0 "$API_PID" 2>/dev/null; then
    DEAD="api_gateway"
else
    DEAD="unknown"
fi
echo "[entrypoint] $DEAD exited (code=$EXIT_CODE) — tearing down peer"

kill "$MAVROS_PID" "$API_PID" 2>/dev/null || true
sleep 1
kill -9 "$MAVROS_PID" "$API_PID" 2>/dev/null || true

exit "$EXIT_CODE"
