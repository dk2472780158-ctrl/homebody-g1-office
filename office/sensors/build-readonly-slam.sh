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
export PATH=/opt/ros/humble/bin:/usr/bin:/bin:/usr/local/bin
set -u
cd /workspace
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon build --install-base "${HB_SENSOR_INSTALL:-/opt/homebody-sensor-install}" \
  --packages-up-to g1_fastlio_bridge --parallel-workers 2 \
  --event-handlers console_direct+ --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
