#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(git -C "$script_dir/../../.." rev-parse --show-toplevel)"
workspace="${FLYING_CAR_TEST_WS:-/home/px4/flying_car_test_ws}"
workspace="$(realpath -m "$workspace")"

case "$workspace" in
  /|/home|/home/px4|"$repo_root")
    echo "refusing unsafe workspace path: $workspace" >&2
    exit 1
    ;;
esac

package_source="$(realpath "$repo_root/Tools/flying_car_test")"
package_path="$workspace/src/flying_car_test"

mkdir -p "$workspace/src"

if [[ -e "$package_path" || -L "$package_path" ]]; then
  if [[ ! -L "$package_path" ]] || [[ "$(readlink -f "$package_path")" != "$package_source" ]]; then
    echo "existing package path is not the expected symlink" >&2
    exit 1
  fi
else
  ln -s "$package_source" "$package_path"
fi

# Optional: RealSense D435 simulation package used by
# FS-FASMv8_octo_realsense (librealsense_gazebo_plugin.so).
realsense_source="/home/px4/realsense_ros_gazebo"
realsense_path="$workspace/src/realsense_ros_gazebo"
if [[ -d "$realsense_source" ]]; then
  if [[ -e "$realsense_path" || -L "$realsense_path" ]]; then
    if [[ ! -L "$realsense_path" ]] || [[ "$(readlink -f "$realsense_path")" != "$(realpath "$realsense_source")" ]]; then
      echo "existing realsense_ros_gazebo path is not the expected symlink" >&2
      exit 1
    fi
  else
    ln -s "$realsense_source" "$realsense_path"
  fi
else
  echo "realsense_ros_gazebo not found at $realsense_source; skipping" >&2
fi

set +u
source /opt/ros/noetic/setup.bash
set -u

if ! python3 -c "import pyulog" >/dev/null 2>&1; then
  echo "pyulog is required for ULog evidence extraction in flying_car_test validation." >&2
  echo "python3 -m pip install --user 'pyulog>=0.5.0'" >&2
  exit 1
fi

catkin_make -C "$workspace"
