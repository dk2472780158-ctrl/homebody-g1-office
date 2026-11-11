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
script="$artifacts/office-agent-console.py"
pidfile="$artifacts/office-agent.pid"
if [[ ! -f "$pidfile" ]]; then
  echo "Office agent console is not running"
  exit 0
fi
pid=$(<"$pidfile")
if [[ "$pid" =~ ^[0-9]+$ ]] && [[ -r "/proc/$pid/cmdline" ]] &&
    tr '\0' '\n' < "/proc/$pid/cmdline" | grep -Fxq "$script"; then
  kill -TERM "$pid"
  echo "Stopped office agent console: $pid"
else
  echo "Stale office agent PID; no process stopped"
fi
rm -f "$pidfile"
