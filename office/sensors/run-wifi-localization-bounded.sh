#!/usr/bin/env bash
# Load checkout-local configuration before using paths or hosts.
if [[ -n "${HB_CONFIG:-}" ]]; then
  source "$HB_CONFIG"
else
  hb_search="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  while [[ "$hb_search" != / && ! -f "$hb_search/config/env.sh" ]]; do hb_search="$(dirname -- "$hb_search")"; done
  [[ -f "$hb_search/config/env.sh" ]] || { echo "Missing config/env.sh; set HB_CONFIG" >&2; exit 1; }
  source "$hb_search/config/env.sh"
fi
# Bounded local-only localization; source connection sends subscribe/unsubscribe only.
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${HB_SENSOR_INSTALL:-/opt/homebody-sensor-install}/setup.bash"
export PATH=/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin
export PYTHONPATH=/workspace/vendor:$PYTHONPATH
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
set -u
run_dir=/workspace/runtime/wifi-20261106-current
mkdir -p "$run_dir"
export ROS_LOG_DIR="$run_dir/ros-logs"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
setsid /usr/bin/python3 -u /workspace/rosbridge_to_ros2.py --source ${HB_ROSBRIDGE_SOURCE} > "$run_dir/relay.log" 2>&1 &
pids+=($!)
setsid ros2 launch g1_fastlio_bridge g1_fastlio_mapping.launch.py rviz:=false map_file_path:="$run_dir/session-map.pcd" > "$run_dir/fastlio.log" 2>&1 &
pids+=($!)
sleep 5
timeout --signal=TERM --kill-after=5 48 /usr/bin/python3 /workspace/capture_registered_cloud.py --duration 40 --output "$run_dir/registered.npz" > "$run_dir/capture.log" 2>&1
echo 'Bounded read-only capture finished; stopping localization.'
