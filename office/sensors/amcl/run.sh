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
set -eo pipefail
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1
export FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/amcl/dds.xml
export FASTDDS_DEFAULT_PROFILES_FILE=/workspace/amcl/dds.xml
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
stage=/workspace/amcl
amcl_pid= adapter_pid= seed_pid= running=1
cleanup() {
  [[ -z "$adapter_pid" ]] || kill -TERM "$adapter_pid" 2>/dev/null || true
  [[ -z "$amcl_pid" ]] || kill -TERM "$amcl_pid" 2>/dev/null || true
  [[ -z "$seed_pid" ]] || kill -TERM "$seed_pid" 2>/dev/null || true
  wait || true
  amcl_pid= adapter_pid= seed_pid=
}
trap cleanup EXIT
trap 'running=0; cleanup' TERM INT
while [[ "$running" == 1 ]]; do
/opt/ros/humble/lib/nav2_amcl/amcl --ros-args -r __ns:=/homebody_amcl --params-file "$stage/params.yaml" > "$stage/amcl.log" 2>&1 &
amcl_pid=$!
/usr/bin/python3 -u "$stage/adapter.py" > "$stage/adapter.log" 2>&1 &
adapter_pid=$!
/usr/bin/python3 -u "$stage/seed.py" > "$stage/seed.log" 2>&1 &
seed_pid=$!
if wait -n "$amcl_pid" "$adapter_pid" "$seed_pid"; then true; else true; fi
cleanup
if [[ "$running" == 1 ]]; then sleep 2; fi
done
