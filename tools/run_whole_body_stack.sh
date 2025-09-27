#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WORKSPACE_DIR}"

SKIP_BUILD=false
BUILD_ARGS=()
LAUNCH_ARGS=()

usage() {
  cat <<USAGE
Usage: $(basename "$0") [options]

Options:
  --skip-build           Skip the 'colcon build' step (assumes workspace already built)
  --build-args "ARGS"    Extra arguments to pass to 'colcon build'
  --launch-args "ARGS"   Extra arguments to pass to 'ros2 launch'
  -h, --help             Show this message and exit

Examples:
  $(basename "$0")
  $(basename "$0") --build-args "--packages-up-to mobile_arm_whole_body_control"
  $(basename "$0") --skip-build --launch-args "rviz:=false"
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=true
      shift
      ;;
    --build-args)
      shift
      [[ $# -gt 0 ]] || { echo "--build-args requires an argument" >&2; exit 1; }
      # shellcheck disable=SC2206
      BUILD_ARGS=(${BUILD_ARGS[*]} $1)
      shift
      ;;
    --launch-args)
      shift
      [[ $# -gt 0 ]] || { echo "--launch-args requires an argument" >&2; exit 1; }
      # shellcheck disable=SC2206
      LAUNCH_ARGS=(${LAUNCH_ARGS[*]} $1)
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "[WARN] /opt/ros/humble/setup.bash not found; ensure ROS 2 Humble is sourced" >&2
fi

if [[ "${SKIP_BUILD}" == false ]]; then
  # Clean stale build/install directories that block ament symlink creation
  for stale in \
    "${WORKSPACE_DIR}/build/mobile_arm_whole_body_interfaces" \
    "${WORKSPACE_DIR}/install/mobile_arm_whole_body_interfaces" \
    "${WORKSPACE_DIR}/build/mobile_arm_whole_body" \
    "${WORKSPACE_DIR}/install/mobile_arm_whole_body"
  do
    if [[ -e "${stale}" && ! -L "${stale}" ]]; then
      echo "==> Removing stale directory ${stale}" >&2
      rm -rf "${stale}"
    fi
  done

  echo "==> Building workspace (colcon build --symlink-install ${BUILD_ARGS[*]})"
  colcon build --symlink-install "${BUILD_ARGS[@]}"
else
  echo "==> Skipping build step"
fi

if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${WORKSPACE_DIR}/install/setup.bash"
  set -u
else
  echo "[ERROR] install/setup.bash not found; build the workspace first or remove --skip-build" >&2
  exit 1
fi

echo "==> Launching whole_body_streaming.launch.py ${LAUNCH_ARGS[*]}"
exec ros2 launch mobile_arm_whole_body_bringup whole_body_streaming.launch.py "${LAUNCH_ARGS[@]}"
