#!/usr/bin/env bash
set -euo pipefail

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-melodic}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PACKAGE="ros-${ROS_DISTRO}-swarm-ros-bridge"
ROS_PACKAGE="swarm_ros_bridge"

product_version() {
  sed -n 's/^version:[[:space:]]*//p' "${REPO_ROOT}/.xgc2/product.yml" | head -n 1 | tr -d '\r'
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"

case "${ROS_DISTRO}" in
  melodic|noetic) ;;
  *)
    echo "unsupported ROS distro: ${ROS_DISTRO}" >&2
    exit 1
    ;;
esac

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi
if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

ARCH="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
PKG_ROOT="$(mktemp -d /tmp/swarm-ros-bridge-package.XXXXXX)"

cleanup() {
  case "${PKG_ROOT}" in
    /tmp/swarm-ros-bridge-package.*)
      find "${PKG_ROOT}" -depth -delete
      ;;
    *)
      echo "refusing to clean unexpected package directory: ${PKG_ROOT}" >&2
      ;;
  esac
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}" "${PKG_ROOT}/DEBIAN" "${PKG_ROOT}/usr/share/doc/${PACKAGE}"
rm -f "${OUTPUT_DIR}/${PACKAGE}_"*.deb

copy_path() {
  local src="$1"
  if [[ -e "${src}" ]]; then
    mkdir -p "${PKG_ROOT}$(dirname "${src#${INSTALL_ROOT}}")"
    cp -a "${src}" "${PKG_ROOT}${src#${INSTALL_ROOT}}"
  fi
}

copy_path "${PREFIX_ROOT}/share/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/lib/${ROS_PACKAGE}"

if [[ ! -f "${PKG_ROOT}${PREFIX}/share/${ROS_PACKAGE}/package.xml" ]]; then
  echo "missing installed package.xml for ${ROS_PACKAGE}" >&2
  exit 1
fi
if [[ ! -x "${PKG_ROOT}${PREFIX}/lib/${ROS_PACKAGE}/bridge_node" ]]; then
  echo "missing installed bridge_node executable" >&2
  exit 1
fi

cat > "${PKG_ROOT}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <apt@example.com>
Depends: libzmqpp-dev, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-sensor-msgs, ros-${ROS_DISTRO}-std-msgs
Description: ZeroMQ ROS1 bridge for swarm robot topics
 ROS ${ROS_DISTRO} swarm_ros_bridge package for forwarding configured ROS
 topics between robots over ZeroMQ.
EOF

cat > "${PKG_ROOT}/usr/share/doc/${PACKAGE}/README" <<EOF
${PACKAGE}

ROS package:
  ${ROS_PACKAGE}

Version:
  ${VERSION}

Compiled message types:
  sensor_msgs/Imu
  geometry_msgs/Twist
  std_msgs/String
  std_msgs/Float32
  std_msgs/UInt32
EOF

find "${PKG_ROOT}" -type d -exec chmod 0755 {} +
chmod 0755 "${PKG_ROOT}/DEBIAN"
chmod 0644 "${PKG_ROOT}/DEBIAN/control" "${PKG_ROOT}/usr/share/doc/${PACKAGE}/README"

for executable in bridge_node listener.py talker.py; do
  if [[ ! -x "${PKG_ROOT}${PREFIX}/lib/${ROS_PACKAGE}/${executable}" ]]; then
    echo "packaged ROS node is not executable: ${PREFIX}/lib/${ROS_PACKAGE}/${executable}" >&2
    exit 1
  fi
done

fakeroot dpkg-deb --build "${PKG_ROOT}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "${PACKAGE}_*.deb" -print | sort
