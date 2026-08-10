#!/usr/bin/env bash
# shellcheck disable=SC1004,SC2016
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-melodic}"
DOCKER_IMAGE="${DOCKER_IMAGE:-}"
DOCKER_NETWORK="${DOCKER_NETWORK:-}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --network)
      DOCKER_NETWORK="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

case "${ROS_DISTRO}" in
  melodic)
    DEFAULT_DOCKER_IMAGE="ros:melodic-ros-base-bionic"
    ;;
  noetic)
    DEFAULT_DOCKER_IMAGE="ros:noetic-ros-base-focal"
    ;;
  *)
    echo "unsupported ROS distro: ${ROS_DISTRO}" >&2
    exit 1
    ;;
esac
if [[ -z "${DOCKER_IMAGE}" ]]; then
  DOCKER_IMAGE="${DEFAULT_DOCKER_IMAGE}"
fi

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker_network_args=()
if [[ -n "${DOCKER_NETWORK}" ]]; then
  docker_network_args=(--network "${DOCKER_NETWORK}")
fi

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -e XGC2_BUILD_GID="$(id -g)" \
  -e XGC2_BUILD_UID="$(id -u)" \
  -e ROS_DISTRO="${ROS_DISTRO}" \
  "${docker_network_args[@]}" \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail
    trap '\''build_status=$?; chown -R "${XGC2_BUILD_UID}:${XGC2_BUILD_GID}" /workspace/work /workspace/out; exit "${build_status}"'\'' EXIT

    export DEBIAN_FRONTEND=noninteractive
    ros_prefix="/opt/ros/${ROS_DISTRO}"
    ros_dependencies=(
      "ros-${ROS_DISTRO}-geometry-msgs"
      "ros-${ROS_DISTRO}-nav-msgs"
      "ros-${ROS_DISTRO}-roscpp"
      "ros-${ROS_DISTRO}-roslaunch"
      "ros-${ROS_DISTRO}-rospack"
      "ros-${ROS_DISTRO}-sensor-msgs"
      "ros-${ROS_DISTRO}-std-msgs"
    )
    apt-get update
    apt-get install -y --no-install-recommends \
      build-essential \
      clang-format \
      clang-tidy \
      cmake \
      curl \
      diffutils \
      dpkg-dev \
      fakeroot \
      git \
      libzmqpp-dev \
      python3 \
      rsync \
      shellcheck \
      "${ros_dependencies[@]}"

    ROS_DISTRO="${ROS_DISTRO}" \
      /workspace/repo/.xgc2/scripts/install_scout_msgs_dependency.sh

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/swarm_ros_bridge
    rsync -a --delete /workspace/repo/ /workspace/work/src/swarm_ros_bridge/

    /workspace/work/src/swarm_ros_bridge/test/run_v2_core_tests.sh

    cd /workspace/work
    set +u
    source "${ros_prefix}/setup.bash"
    set -u
    parallel_jobs="$(nproc)"
    catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" \
      swarm_ros_bridge_v2_ros1_codec_test \
      -DCATKIN_ENABLE_TESTING=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"
    /workspace/work/devel/lib/swarm_ros_bridge/swarm_ros_bridge_v2_ros1_codec_test
    DESTDIR=/workspace/work/install-root catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" install \
      -DCMAKE_INSTALL_PREFIX="${ros_prefix}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
      -DCMAKE_C_FLAGS_RELEASE="-O3 -DNDEBUG"

    /workspace/repo/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    mapfile -t package_debs < <(
      find /workspace/out -maxdepth 1 -type f \
        -name "ros-${ROS_DISTRO}-swarm-ros-bridge_*.deb" -print | sort
    )
    if [[ "${#package_debs[@]}" -ne 1 ]]; then
      echo "expected exactly one swarm_ros_bridge deb, found ${#package_debs[@]}" >&2
      exit 1
    fi

    deb_root=/workspace/work/deb-root
    rm -rf "${deb_root}"
    mkdir -p "${deb_root}"
    dpkg-deb -x "${package_debs[0]}" "${deb_root}"
    required_deb_files=(
      "${ros_prefix}/lib/libswarm_ros_bridge_protocol_v2.so"
      "${ros_prefix}/lib/libswarm_ros_bridge_ros1_codec_v2.so"
      "${ros_prefix}/include/swarm_ros_bridge/v2/protocol.hpp"
      "${ros_prefix}/include/swarm_ros_bridge/v2/ros1_codec.hpp"
      "${ros_prefix}/share/swarm_ros_bridge/docs/protocol-v2.md"
      "${ros_prefix}/lib/swarm_ros_bridge/tests/swarm_ros_bridge_v2_protocol_test"
      "${ros_prefix}/lib/swarm_ros_bridge/tests/swarm_ros_bridge_v2_ros1_codec_test"
    )
    for required_file in "${required_deb_files[@]}"; do
      if [[ ! -f "${deb_root}${required_file}" ]]; then
        echo "deb is missing required v2 file: ${required_file}" >&2
        exit 1
      fi
    done

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y "${package_debs[0]}"
      package_name="ros-${ROS_DISTRO}-swarm-ros-bridge"
      verification="$(dpkg -V "${package_name}")"
      if [[ -n "${verification}" ]]; then
        echo "dpkg verification failed for ${package_name}:" >&2
        printf "%s\n" "${verification}" >&2
        exit 1
      fi
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
