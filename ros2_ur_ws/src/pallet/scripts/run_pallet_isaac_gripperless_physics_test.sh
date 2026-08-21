#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
isaac_python="${ISAAC_SIM_PYTHON:-/home/minwoo/isaacsim/python.sh}"

if [[ ! -x "${isaac_python}" ]]; then
  echo "Isaac Sim Python was not found: ${isaac_python}" >&2
  echo "Set ISAAC_SIM_PYTHON to the python.sh path." >&2
  exit 2
fi

exec "${isaac_python}" "${script_dir}/pallet_isaac_gripperless_physics_test.py" "$@"
