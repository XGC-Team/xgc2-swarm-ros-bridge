#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# shellcheck disable=SC1004,SC2016
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/../.." && pwd)"
noetic_image="${NOETIC_DOCKER_IMAGE:-ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-ros-noetic:1.0.0}"
noetic_work_dir="$(mktemp -d /tmp/swarm-ros-bridge-noetic.XXXXXX)"

cleanup() {
  case "${noetic_work_dir}" in
    /tmp/swarm-ros-bridge-noetic.*)
      if ! find "${noetic_work_dir}" -depth -delete; then
        echo "could not fully clean Noetic work directory" >&2
      fi
      ;;
    *)
      echo "refusing to clean unexpected Noetic work directory" >&2
      ;;
  esac
}
trap cleanup EXIT

docker pull "${noetic_image}"
docker run --rm --network none \
  -e DEBIAN_FRONTEND=noninteractive \
  -e XGC2_BUILD_GID="$(id -g)" \
  -e XGC2_BUILD_UID="$(id -u)" \
  -v "${repo_dir}:/workspace/repo:ro" \
  -v "${noetic_work_dir}:/workspace/work" \
  "${noetic_image}" \
  bash -lc '
    set -euo pipefail
    trap '\''build_status=$?; chown -R "${XGC2_BUILD_UID}:${XGC2_BUILD_GID}" /workspace/work; exit "${build_status}"'\'' EXIT
    export DEBIAN_FRONTEND=noninteractive
    for pkg in \
      ros-noetic-geometry-msgs \
      ros-noetic-roscpp \
      ros-noetic-sensor-msgs \
      ros-noetic-std-msgs \
      libzmqpp-dev
    do
      if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
        echo "image is missing ${pkg}" >&2
        exit 1
      fi
    done

    mkdir -p /workspace/work/src/swarm_ros_bridge
    rsync -a --exclude .git --exclude .work --exclude debs \
      /workspace/repo/ /workspace/work/src/swarm_ros_bridge/

    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    cd /workspace/work
    parallel_jobs="$(nproc)"
    catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
      -DCATKIN_ENABLE_TESTING=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/workspace/work/install

    test -x /workspace/work/install/lib/swarm_ros_bridge/bridge_node
    test ! -e /workspace/work/install/lib/libswarm_ros_bridge_protocol_v2.so
  '

echo "ROS Noetic/Focal catkin source build passed"
