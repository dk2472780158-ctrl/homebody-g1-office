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

root=${HB_REPO_ROOT}
artifacts=${HB_OFFICE_ROOT}
if [[ -z "${OPENAI_API_KEY:-}" && -n "${OPENAI_API_KEY_FILE:-}" ]]; then
  [[ -s "$OPENAI_API_KEY_FILE" ]] || { echo "Configured API key file is missing" >&2; exit 1; }
  export OPENAI_API_KEY
  OPENAI_API_KEY=$(<"$OPENAI_API_KEY_FILE")
fi
[[ -n "${OPENAI_API_KEY:-}" ]] || { echo "Set OPENAI_API_KEY or OPENAI_API_KEY_FILE" >&2; exit 1; }
export OPENAI_BASE_URL
export MUJOCO_GL=egl
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$root/src"

if (( $# )); then
  command=$1
  shift
else
  command=provider-check
fi
cd "$root"
exec "$HB_PYTHON" -m homebody "$command" \
  --scene "$artifacts/office-map-proxy-scene" --model astra-api --output "$artifacts" "$@"
