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
script="$artifacts/g1-readonly-viewer.py"
pidfile="$artifacts/g1-readonly-viewer.pid"
if [[ -f "$pidfile" ]]; then
  pid=$(<"$pidfile")
  if [[ "$pid" =~ ^[0-9]+$ ]] && [[ -r "/proc/$pid/cmdline" ]] &&
      tr '\0' '\n' < "/proc/$pid/cmdline" | grep -Fxq "$script"; then
    echo "Read-only 3D viewer already running: $pid"
    exit 0
  fi
fi
"$HB_PYTHON" -c 'import pathlib,sys; p=pathlib.Path(sys.argv[1]); compile(p.read_bytes(),str(p),"exec")' "$script"
nohup "$HB_PYTHON" -u "$script" > "$artifacts/g1-readonly-viewer.log" 2>&1 < /dev/null &
echo "$!" > "$pidfile"
echo "Started read-only 3D viewer: $!"
