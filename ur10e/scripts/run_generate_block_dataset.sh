#!/usr/bin/env bash
set -euo pipefail

ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-/home/minwoo/isaacsim}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

unset PYTHONPATH
export PYTHONPATH="${PROJECT_DIR}"

exec "${ISAAC_SIM_ROOT}/python.sh" "${PROJECT_DIR}/apps/generate_block_dataset.py" "$@"
