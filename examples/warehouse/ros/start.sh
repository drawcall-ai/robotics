#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
pids=()
cleanup() {
  kill "${pids[@]}" 2>/dev/null || true
  wait "${pids[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' TERM INT
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0 port:=9090 &
pids+=("$!")
ros2 launch /app/bringup.launch.py &
pids+=("$!")
python3 /app/bridge.py --ros-args -p use_sim_time:=true &
pids+=("$!")
python3 /app/mission.py --ros-args -p use_sim_time:=true &
pids+=("$!")
set +e
wait -n "${pids[@]}"
result=$?
set -e
echo "ROS child exited (status $result); stopping service." >&2
exit 1
