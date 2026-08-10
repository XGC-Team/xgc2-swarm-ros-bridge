#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# shellcheck disable=SC1004,SC2016
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/../.." && pwd)"
noetic_image="${NOETIC_DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
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
docker run --rm \
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
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      curl \
      libzmqpp-dev \
      rsync \
      ros-noetic-geometry-msgs \
      ros-noetic-nav-msgs \
      ros-noetic-roscpp \
      ros-noetic-sensor-msgs \
      ros-noetic-std-msgs

    ROS_DISTRO=noetic \
      /workspace/repo/.xgc2/scripts/install_scout_msgs_dependency.sh

    mkdir -p /workspace/work/src/swarm_ros_bridge
    rsync -a /workspace/repo/ /workspace/work/src/swarm_ros_bridge/

    set +u
    source /opt/ros/noetic/setup.bash
    set -u
    cd /workspace/work
    parallel_jobs="$(nproc)"
    catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" \
      swarm_ros_bridge_v2_ros1_codec_test \
      -DCATKIN_ENABLE_TESTING=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/workspace/work/install
    /workspace/work/devel/lib/swarm_ros_bridge/swarm_ros_bridge_v2_ros1_codec_test
    catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
      -DCATKIN_ENABLE_TESTING=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/workspace/work/install

    test -x /workspace/work/install/lib/swarm_ros_bridge/bridge_node
    test -f /workspace/work/install/lib/libswarm_ros_bridge_protocol_v2.so
    test -f /workspace/work/install/lib/libswarm_ros_bridge_ros1_codec_v2.so
    test -f /workspace/work/install/include/swarm_ros_bridge/v2/protocol.hpp
    test -f /workspace/work/install/include/swarm_ros_bridge/v2/ros1_codec.hpp
    test -f /workspace/work/install/share/swarm_ros_bridge/docs/protocol-v2.md
  '

echo "ROS Noetic/Focal catkin source build passed"
