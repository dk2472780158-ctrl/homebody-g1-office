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
set -euo pipefail
artifacts=${HB_OFFICE_ROOT}
script="$artifacts/office-click-viewer.py"
pidfile="$artifacts/office-click-viewer.pid"
if [[ -f "$pidfile" ]]; then
  sim_pid=$(cat "$pidfile")
  if [[ "$sim_pid" =~ ^[0-9]+$ ]] && [[ -r "/proc/$sim_pid/cmdline" ]] &&
      tr '\0' '\n' < "/proc/$sim_pid/cmdline" | grep -Fxq "$script"; then
    echo "Already running: $sim_pid"
    exit 0
  fi
fi
cd "$HB_REPO_ROOT"
nohup env DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 \
  "$HB_PYTHON" -u "$script" \
  > "$artifacts/office-click-viewer.log" 2>&1 < /dev/null &
sim_pid=$!
echo "$sim_pid" > "$pidfile"
echo "Started HomeBody click navigation: $sim_pid"
