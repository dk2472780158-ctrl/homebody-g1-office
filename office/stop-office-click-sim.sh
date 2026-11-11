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
pidfile="$artifacts/office-click-viewer.pid"
if [[ ! -f "$pidfile" ]]; then echo 'No saved simulation PID'; exit 0; fi
sim_pid=$(cat "$pidfile")
if [[ "$sim_pid" =~ ^[0-9]+$ ]] && [[ -r "/proc/$sim_pid/cmdline" ]] &&
    tr '\0' '\n' < "/proc/$sim_pid/cmdline" | grep -Fxq "$artifacts/office-click-viewer.py"; then
  kill -TERM "$sim_pid"
  echo "Stopped HomeBody click navigation: $sim_pid"
else
  echo 'Saved PID is no longer this simulation; no process stopped.'
fi
