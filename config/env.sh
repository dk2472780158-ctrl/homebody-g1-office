#!/usr/bin/env bash
# Source this file from Bash. Store machine-specific values in local.env.
export HB_REPO_ROOT="${HB_REPO_ROOT:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}"
if [[ -f "$HB_REPO_ROOT/config/local.env" ]]; then
  set -a
  source "$HB_REPO_ROOT/config/local.env"
  set +a
fi
export HB_OFFICE_ROOT="${HB_OFFICE_ROOT:-$HB_REPO_ROOT/office}"
export HB_VENV="${HB_VENV:-$HB_REPO_ROOT/.venv}"
export HB_USER_HOME="${HB_USER_HOME:-$HOME}"
export HB_SECRET_DIR="${HB_SECRET_DIR:-$HOME/.config/homebody}"
export HB_TELEOP_TOKEN_FILE="${HB_TELEOP_TOKEN_FILE:-$HB_SECRET_DIR/g1-teleop.token}"
export HB_ROBOT_HOST="${HB_ROBOT_HOST:-robot.local}"
export HB_WORKSTATION_HOST="${HB_WORKSTATION_HOST:-workstation.local}"
export HB_WORKSTATION_HOST_KEY_ALIAS="${HB_WORKSTATION_HOST_KEY_ALIAS:-$HB_WORKSTATION_HOST}"
export HB_ROSBRIDGE_SOURCE="${HB_ROSBRIDGE_SOURCE:-ws://127.0.0.1:19090}"
export HB_PYTHON="${HB_PYTHON:-$HB_VENV/bin/python}"
export HB_CONTAINER="${HB_CONTAINER:-homebody-office-sensors}"
export OPENAI_BASE_URL="${OPENAI_BASE_URL:-https://api.openai.com/v1}"
export HB_MODEL="${HB_MODEL:-astra-api}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYTHONDONTWRITEBYTECODE=1
