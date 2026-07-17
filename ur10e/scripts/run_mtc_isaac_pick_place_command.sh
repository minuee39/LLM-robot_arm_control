#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="/home/minwoo/Desktop/LLM/ur10e"

python3 "${PROJECT_DIR}/mtc_command_runner.py" "$@"
