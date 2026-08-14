#!/usr/bin/env bash
# shellcheck disable=SC1004,SC2016
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-melodic}"
DOCKER_IMAGE="${DOCKER_IMAGE:-}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --skip-install-check) INSTALL_CHECK=false; shift ;;
    --network) shift 2 ;; # ignored; container stays offline
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

case "${ROS_DISTRO}" in
  melodic)
    DEFAULT_DOCKER_IMAGE="ghcr.io/xgc-team/xgc2-images/xgc2-build-bionic-ros-melodic:1.0.0"
    ;;
  noetic)
    DEFAULT_DOCKER_IMAGE="ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-ros-noetic:1.0.0"
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

docker pull "${DOCKER_IMAGE}"
docker run --rm --network none \
  -e XGC2_BUILD_GID="$(id -g)" \
  -e XGC2_BUILD_UID="$(id -u)" \
  -e ROS_DISTRO="${ROS_DISTRO}" \
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
    : "${ROS_DISTRO:?}"
    for pkg in \
      "ros-${ROS_DISTRO}-geometry-msgs" \
      "ros-${ROS_DISTRO}-roscpp" \
      "ros-${ROS_DISTRO}-sensor-msgs" \
      "ros-${ROS_DISTRO}-std-msgs" \
      libzmqpp-dev
    do
      if ! dpkg -s "${pkg}" >/dev/null 2>&1; then
        echo "image is missing ${pkg}; use xgc2-build-<ubuntu>-ros-<distro>" >&2
        exit 1
      fi
    done

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel /workspace/work/install-root
    mkdir -p /workspace/work/src/swarm_ros_bridge
    rsync -a --delete \
      --exclude .git --exclude .work --exclude debs \
      /workspace/repo/ /workspace/work/src/swarm_ros_bridge/

    cd /workspace/work
    set +u
    source "${ros_prefix}/setup.bash"
    set -u
    parallel_jobs="$(nproc)"
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
      "${ros_prefix}/lib/swarm_ros_bridge/bridge_node"
      "${ros_prefix}/share/swarm_ros_bridge/package.xml"
      "${ros_prefix}/share/swarm_ros_bridge/launch/test.launch"
    )
    for required_file in "${required_deb_files[@]}"; do
      if [[ ! -f "${deb_root}${required_file}" ]]; then
        echo "deb is missing required file: ${required_file}" >&2
        exit 1
      fi
    done

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      dpkg -i "${package_debs[0]}"
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
