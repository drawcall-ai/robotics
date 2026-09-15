#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090 &
bridge=$!
python3 /app/controller.py --ros-args -p use_sim_time:=true &
controller=$!
cleanup() {
  kill "$bridge" "$controller" 2>/dev/null || true
  wait "$bridge" "$controller" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
set +e
wait -n "$bridge" "$controller"
result=$?
set -e
echo "ROS child exited (status $result); stopping warehouse service." >&2
# Either child exiting unexpectedly means the service is no longer usable.
exit 1
