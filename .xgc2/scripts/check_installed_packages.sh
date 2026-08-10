#!/usr/bin/env bash
# shellcheck disable=SC1090
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-melodic}"
SCOUT_MSGS_VERSION="0.3.3-10"
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
test -f "${PREFIX}/lib/lib${ROS_PACKAGE}_protocol_v2.so"
test -f "${PREFIX}/lib/lib${ROS_PACKAGE}_ros1_codec_v2.so"
test -f "${PREFIX}/include/${ROS_PACKAGE}/v2/protocol.hpp"
test -f "${PREFIX}/include/${ROS_PACKAGE}/v2/ros1_codec.hpp"
test -f "${PREFIX}/share/${ROS_PACKAGE}/docs/protocol-v2.md"
test -x "${PREFIX}/lib/${ROS_PACKAGE}/tests/swarm_ros_bridge_v2_protocol_test"
test -x "${PREFIX}/lib/${ROS_PACKAGE}/tests/swarm_ros_bridge_v2_ros1_codec_test"
dpkg -s "ros-${ROS_DISTRO}-nav-msgs" >/dev/null
dpkg -s "ros-${ROS_DISTRO}-scout-msgs" >/dev/null
installed_scout_msgs_version="$(
  dpkg-query -W -f='${Version}' "ros-${ROS_DISTRO}-scout-msgs"
)"
if [[ "${installed_scout_msgs_version}" != "${SCOUT_MSGS_VERSION}" ]]; then
  echo "unexpected installed scout_msgs version: ${installed_scout_msgs_version}" >&2
  exit 1
fi

legacy_ldd="$(ldd "${PREFIX}/lib/${ROS_PACKAGE}/bridge_node")"
if grep -Fq 'not found' <<<"${legacy_ldd}"; then
  echo "legacy bridge has unresolved dynamic dependencies" >&2
  printf '%s\n' "${legacy_ldd}" >&2
  exit 1
fi
grep -Fq "libzmq" <<<"${legacy_ldd}"
grep -Fq "libroscpp" <<<"${legacy_ldd}"
readelf -h "${PREFIX}/lib/${ROS_PACKAGE}/bridge_node" >/dev/null

"${PREFIX}/lib/${ROS_PACKAGE}/tests/swarm_ros_bridge_v2_protocol_test"
"${PREFIX}/lib/${ROS_PACKAGE}/tests/swarm_ros_bridge_v2_ros1_codec_test"

probe_dir="$(mktemp -d /tmp/swarm-ros-bridge-installed-check.XXXXXX)"
cleanup() {
  case "${probe_dir}" in
    /tmp/swarm-ros-bridge-installed-check.*)
      find "${probe_dir}" -depth -delete
      ;;
    *)
      echo "refusing to clean unexpected probe directory: ${probe_dir}" >&2
      ;;
  esac
}
trap cleanup EXIT

g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -x c++ - \
  -I"${PREFIX}/include" \
  -L"${PREFIX}/lib" \
  -Wl,-rpath,"${PREFIX}/lib" \
  -lswarm_ros_bridge_protocol_v2 \
  -o "${probe_dir}/v2-installed-consumer" <<'CPP'
#include <cstring>

#include <swarm_ros_bridge/v2/protocol.hpp>

int main() {
  using xgc2::swarm_bridge::v2::ErrorCode;
  using xgc2::swarm_bridge::v2::PeerRole;
  using xgc2::swarm_bridge::v2::SendWindow;
  using xgc2::swarm_bridge::v2::SendWindowConfig;
  using xgc2::swarm_bridge::v2::errorCodeName;
  using xgc2::swarm_bridge::v2::kGroundRequiredCapabilities;
  using xgc2::swarm_bridge::v2::kVehicleRequiredCapabilities;
  using xgc2::swarm_bridge::v2::validatePeerCompatibility;

  SendWindowConfig config;
  config.local_role = PeerRole::kGround;
  config.local_capabilities = kGroundRequiredCapabilities;
  config.peer_role = PeerRole::kVehicle;
  config.peer_capabilities = kVehicleRequiredCapabilities;
  SendWindow window(config);
  const bool linked_contract =
      validatePeerCompatibility(
          PeerRole::kGround, kGroundRequiredCapabilities,
          PeerRole::kVehicle, kVehicleRequiredCapabilities)
          .ok() &&
      window.acknowledgedFrontier() == 0U;
  return std::strcmp(errorCodeName(ErrorCode::kAckFuture), "ack_future") == 0 &&
                 linked_contract
             ? 0
             : 1;
}
CPP
"${probe_dir}/v2-installed-consumer"

g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -x c++ - \
  -I"${PREFIX}/include" \
  -L"${PREFIX}/lib" \
  -Wl,-rpath,"${PREFIX}/lib" \
  -lswarm_ros_bridge_ros1_codec_v2 \
  -lswarm_ros_bridge_protocol_v2 \
  -lroscpp_serialization \
  -lrostime \
  -lcpp_common \
  -o "${probe_dir}/v2-ros1-codec-installed-consumer" <<'CPP'
#include <cstdint>

#include <swarm_ros_bridge/v2/ros1_codec.hpp>

int main() {
  namespace v2 = xgc2::swarm_bridge::v2;
  sensor_msgs::Imu message;
  message.header.stamp.fromNSec(1234567890ULL);
  message.header.frame_id = "imu_link";
  v2::FrameHeader header;
  header.sequence = 1U;
  header.monotonic_ns = 1U;
  header.session_epoch = 1U;
  header.capabilities = v2::kVehicleRequiredCapabilities;
  header.slot_id = "scout-01";
  header.asset_id = "asset-a";
  header.robot_kind = "scout";
  header.run_id = "run-1";
  header.boot_id = "boot-1";
  header.build_id = "build-1";
  v2::Frame frame;
  if (!v2::encodeImuTelemetry(message, header, &frame).ok()) {
    return 1;
  }
  sensor_msgs::Imu decoded;
  return v2::decodeImuTelemetry(frame, &decoded).ok() &&
                 decoded.header.stamp.toNSec() == 1234567890ULL
             ? 0
             : 1;
}
CPP
"${probe_dir}/v2-ros1-codec-installed-consumer"

roslaunch --files "${ROS_PACKAGE}" test.launch >/dev/null

echo "Installed package check passed"
