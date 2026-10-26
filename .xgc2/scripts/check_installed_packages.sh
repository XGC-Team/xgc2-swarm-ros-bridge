#!/usr/bin/env bash
# shellcheck disable=SC1090
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-melodic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
PACKAGE="ros-${ROS_DISTRO}-swarm-ros-bridge"
ROS_PACKAGE="swarm_ros_bridge"

case "${ROS_DISTRO}" in
  melodic|noetic) ;;
  *)
    echo "unsupported ROS distro: ${ROS_DISTRO}" >&2
    exit 1
    ;;
esac

dpkg -s "${PACKAGE}" >/dev/null
verification="$(dpkg -V "${PACKAGE}")"
if [[ -n "${verification}" ]]; then
  echo "dpkg verification failed for ${PACKAGE}:" >&2
  printf '%s\n' "${verification}" >&2
  exit 1
fi

set +u
source "${PREFIX}/setup.bash"
set -u

test "$(rospack find "${ROS_PACKAGE}")" = "${PREFIX}/share/${ROS_PACKAGE}"
test -f "${PREFIX}/share/${ROS_PACKAGE}/package.xml"
test -f "${PREFIX}/share/${ROS_PACKAGE}/config/ros_topics.yaml"
test -f "${PREFIX}/share/${ROS_PACKAGE}/launch/test.launch"
test -x "${PREFIX}/lib/${ROS_PACKAGE}/bridge_node"
test -x "${PREFIX}/lib/${ROS_PACKAGE}/listener.py"
test -x "${PREFIX}/lib/${ROS_PACKAGE}/talker.py"
test ! -e "${PREFIX}/lib/lib${ROS_PACKAGE}_protocol_v2.so"
test ! -e "${PREFIX}/include/${ROS_PACKAGE}/v2/protocol.hpp"

legacy_ldd="$(ldd "${PREFIX}/lib/${ROS_PACKAGE}/bridge_node")"
if grep -Fq 'not found' <<<"${legacy_ldd}"; then
  echo "bridge_node has unresolved dynamic dependencies" >&2
  printf '%s\n' "${legacy_ldd}" >&2
  exit 1
fi
grep -Fq "libzmq" <<<"${legacy_ldd}"
grep -Fq "libroscpp" <<<"${legacy_ldd}"
readelf -h "${PREFIX}/lib/${ROS_PACKAGE}/bridge_node" >/dev/null

roslaunch --files "${ROS_PACKAGE}" test.launch >/dev/null

echo "Installed package check passed"
