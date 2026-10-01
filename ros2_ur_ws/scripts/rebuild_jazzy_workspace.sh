#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
backup_root="${workspace_dir}/.distro_artifacts_backup"
backup_stamp="$(date +%Y%m%d-%H%M%S)"
backup_dir="${backup_root}/${backup_stamp}"

if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
set -u

missing=0
for package in moveit moveit_task_constructor_core xacro; do
  if ! ros2 pkg prefix "${package}" >/dev/null 2>&1; then
    echo "Missing Jazzy package: ${package}" >&2
    missing=1
  fi
done
if [[ "${missing}" -ne 0 ]]; then
  echo "Run ${workspace_dir}/scripts/install_jazzy_dependencies.sh first." >&2
  exit 1
fi

mkdir -p "${backup_dir}"
for artifact in build install log; do
  if [[ -e "${workspace_dir}/${artifact}" ]]; then
    mv "${workspace_dir}/${artifact}" "${backup_dir}/${artifact}"
  fi
done

cleanup_failed_build() {
  status=$?
  if [[ "${status}" -ne 0 ]]; then
    echo "Jazzy build failed. Previous artifacts remain at ${backup_dir}." >&2
  fi
  exit "${status}"
}
trap cleanup_failed_build EXIT

cd "${workspace_dir}"
colcon build --symlink-install --event-handlers console_cohesion+

trap - EXIT
echo "Jazzy workspace build completed."
echo "Previous Humble artifacts: ${backup_dir}"
echo "Activate with: source ${workspace_dir}/install/setup.bash"
