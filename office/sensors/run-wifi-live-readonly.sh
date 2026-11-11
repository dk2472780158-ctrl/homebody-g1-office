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
source "${HB_SENSOR_INSTALL:-/opt/homebody-sensor-install}/setup.bash"
export PATH=/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin
export PYTHONPATH=/workspace/vendor:$PYTHONPATH
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
exec /usr/bin/python3 -u /workspace/readonly_supervisor.py
