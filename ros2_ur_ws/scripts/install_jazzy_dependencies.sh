#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  apt_prefix=()
else
  apt_prefix=(sudo)
fi

if [[ ! -r /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy." >&2
  exit 1
fi

"${apt_prefix[@]}" apt-get update
"${apt_prefix[@]}" apt-get install -y \
  python3-rosdep \
  python3-can \
  libuvc-dev \
  libgoogle-glog-dev \
  nlohmann-json3-dev \
  ros-jazzy-controller-manager \
  ros-jazzy-gripper-controllers \
  ros-jazzy-parallel-gripper-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-moveit \
  ros-jazzy-moveit-resources-panda-moveit-config \
  ros-jazzy-moveit-task-constructor-capabilities \
  ros-jazzy-moveit-task-constructor-visualization \
  ros-jazzy-moveit-visual-tools \
  ros-jazzy-ur-description \
  ros-jazzy-xacro

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
astra_source_dir="${workspace_dir}/src/ros2_astra_camera"
astra_patch="${workspace_dir}/patches/ros2_astra_camera-jazzy.patch"
if [[ ! -d "${astra_source_dir}/.git" ]]; then
  echo "Missing source dependency: ${astra_source_dir}" >&2
  echo "Run vcs import src < dependencies.repos first." >&2
  exit 1
fi
if git -C "${astra_source_dir}" apply --reverse --check "${astra_patch}" > /dev/null 2>&1; then
  echo "Astra Jazzy patch already applied."
elif git -C "${astra_source_dir}" apply --check "${astra_patch}"; then
  git -C "${astra_source_dir}" apply "${astra_patch}"
  echo "Applied Astra Jazzy compatibility patch."
else
  echo "Cannot apply Astra Jazzy compatibility patch cleanly." >&2
  exit 1
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${apt_prefix[@]}" rosdep init
fi
rosdep update
rosdep install \
  --from-paths "${workspace_dir}/src" \
  --ignore-src \
  --rosdistro jazzy \
  --skip-keys ament_python \
  -y

echo "ROS 2 Jazzy MoveIt/MTC dependencies installed."
