#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# shellcheck disable=SC2016
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

require_literal() {
  local file="$1"
  local literal="$2"
  if ! grep -Fq -- "$literal" "$file"; then
    echo "missing v2 release contract in ${file#"$repo_dir"/}: ${literal}" >&2
    exit 1
  fi
}

package_script="$repo_dir/.xgc2/scripts/package_debs.sh"
installed_script="$repo_dir/.xgc2/scripts/check_installed_packages.sh"
builder_script="$repo_dir/.xgc2/scripts/build_debs_in_docker.sh"
codec_header="$repo_dir/include/swarm_ros_bridge/v2/ros1_codec.hpp"

require_literal "$package_script" \
  'copy_path "${PREFIX_ROOT}/lib/lib${ROS_PACKAGE}_protocol_v2.so"'
require_literal "$package_script" \
  'copy_path "${PREFIX_ROOT}/lib/lib${ROS_PACKAGE}_ros1_codec_v2.so"'
require_literal "$package_script" \
  'copy_path "${PREFIX_ROOT}/include/${ROS_PACKAGE}"'
require_literal "$package_script" \
  '"${PREFIX}/share/${ROS_PACKAGE}/docs/protocol-v2.md"'
require_literal "$package_script" \
  'ros-${ROS_DISTRO}-scout-msgs (= ${SCOUT_MSGS_VERSION})'
require_literal "$package_script" \
  '"${PREFIX}/lib/${ROS_PACKAGE}/tests/swarm_ros_bridge_v2_protocol_test"'
require_literal "$package_script" \
  '"${PREFIX}/lib/${ROS_PACKAGE}/tests/swarm_ros_bridge_v2_ros1_codec_test"'

require_literal "$installed_script" \
  'test -f "${PREFIX}/lib/lib${ROS_PACKAGE}_protocol_v2.so"'
require_literal "$installed_script" \
  'test -f "${PREFIX}/lib/lib${ROS_PACKAGE}_ros1_codec_v2.so"'
require_literal "$installed_script" \
  'test -f "${PREFIX}/include/${ROS_PACKAGE}/v2/protocol.hpp"'
require_literal "$installed_script" \
  'test -f "${PREFIX}/include/${ROS_PACKAGE}/v2/ros1_codec.hpp"'
require_literal "$installed_script" \
  'test -f "${PREFIX}/share/${ROS_PACKAGE}/docs/protocol-v2.md"'
require_literal "$installed_script" \
  '-lswarm_ros_bridge_protocol_v2'
require_literal "$installed_script" \
  'validatePeerCompatibility('
require_literal "$installed_script" \
  'window.acknowledgedFrontier() == 0U'
require_literal "$installed_script" \
  '-lswarm_ros_bridge_ros1_codec_v2'
require_literal "$installed_script" \
  'encodeImuTelemetry(message, header, &frame)'
require_literal "$installed_script" \
  'verification="$(dpkg -V "${PACKAGE}")"'
require_literal "$installed_script" \
  'installed_scout_msgs_version'
require_literal "$installed_script" \
  'tests/swarm_ros_bridge_v2_protocol_test'
require_literal "$installed_script" \
  'tests/swarm_ros_bridge_v2_ros1_codec_test'
require_literal "$installed_script" \
  'readelf -h "${PREFIX}/lib/${ROS_PACKAGE}/bridge_node"'

require_literal "$builder_script" \
  '/workspace/work/src/swarm_ros_bridge/test/run_v2_core_tests.sh'
require_literal "$builder_script" \
  'clang-format'
require_literal "$builder_script" \
  'clang-tidy'
require_literal "$builder_script" \
  'shellcheck'
require_literal "$builder_script" \
  'dpkg-deb -x "${package_debs[0]}" "${deb_root}"'
require_literal "$builder_script" \
  'ROS_DISTRO="${ROS_DISTRO:-melodic}"'
require_literal "$builder_script" \
  '--ros-distro'
require_literal "$builder_script" \
  'ros:melodic-ros-base-bionic'
require_literal "$builder_script" \
  'ros:noetic-ros-base-focal'
require_literal "$builder_script" \
  '"${ros_prefix}/lib/libswarm_ros_bridge_protocol_v2.so"'
require_literal "$builder_script" \
  '"${ros_prefix}/lib/libswarm_ros_bridge_ros1_codec_v2.so"'
require_literal "$builder_script" \
  'tests/swarm_ros_bridge_v2_protocol_test'
require_literal "$builder_script" \
  'tests/swarm_ros_bridge_v2_ros1_codec_test'
require_literal "$builder_script" \
  'apt-get install -y "${package_debs[0]}"'
require_literal "$builder_script" \
  'verification="$(dpkg -V "${package_name}")"'
require_literal "$builder_script" \
  'swarm_ros_bridge_v2_ros1_codec_test'

dependency_script="$repo_dir/.xgc2/scripts/install_scout_msgs_dependency.sh"
require_literal "$dependency_script" 'version="0.3.3-10"'
require_literal "$dependency_script" \
  '3f4c77e92198a506c9436e02576fb59a6b96b12feeec7fdd6393f51093ffa9a4'
require_literal "$dependency_script" \
  '8577a994e8615c14a8617ddc6ac283dc6e482a19d29e681b7ae8478e6f7dc794'
require_literal "$dependency_script" 'sha256sum --check --strict'

require_literal "$repo_dir/.xgc2/scripts/check_noetic_catkin_in_docker.sh" \
  'swarm_ros_bridge_v2_ros1_codec_test'

require_literal "$codec_header" \
  'Error encodeImuTelemetry(const sensor_msgs::Imu &message,'
require_literal "$codec_header" \
  'Error encodeOdometryTelemetry(const nav_msgs::Odometry &message,'
require_literal "$codec_header" \
  'Error encodeScoutStatusTelemetry(const scout_msgs::ScoutStatus &message,'
require_literal "$codec_header" \
  'Error makeAdmittedPositiveZeroTwist(const Admission &admission,'
require_literal "$codec_header" '6a62c6daae103f4ff57a132d6f95cec2'
require_literal "$codec_header" 'cd5e73d190d741a2f92e81eda573aca7'
require_literal "$codec_header" '7a49e199fd32bf5d7341d653c6b3ba6e'
if grep -Eq 'Error (encode|decode)[A-Za-z0-9_]*Twist' "$codec_header"; then
  echo "generic Twist encoder/decoder is forbidden by the zero-only slice" >&2
  exit 1
fi

require_literal "$repo_dir/.github/workflows/ci.yml" \
  '.xgc2/scripts/build_debs_in_docker.sh'
require_literal "$repo_dir/.github/workflows/release.yml" \
  '.xgc2/scripts/build_debs_in_docker.sh'
require_literal "$repo_dir/.github/workflows/ci.yml" \
  '.xgc2/scripts/check_noetic_catkin_in_docker.sh'
require_literal "$repo_dir/.github/workflows/release.yml" \
  '.xgc2/scripts/check_noetic_catkin_in_docker.sh'
require_literal "$repo_dir/.github/workflows/ci.yml" \
  'ros_distro: noetic'
require_literal "$repo_dir/.github/workflows/ci.yml" \
  'distribution: focal'
require_literal "$repo_dir/.github/workflows/ci.yml" \
  '--ros-distro "${{ matrix.ros_distro }}"'
require_literal "$repo_dir/.github/workflows/release.yml" \
  'ros_distro: noetic'
require_literal "$repo_dir/.github/workflows/release.yml" \
  'distribution: focal'
require_literal "$repo_dir/.github/workflows/release.yml" \
  '--ros-distro "${{ matrix.ros_distro }}"'

if grep -E -R -n --include='*.cpp' --include='*.hpp' '^#if[[:space:]]+0' \
  "$repo_dir/include/swarm_ros_bridge/v2" \
  "$repo_dir/src/v2" \
  "$repo_dir/test/v2_protocol_test.cpp" \
  "$repo_dir/test/v2_ros1_codec_test.cpp"; then
  echo "disabled v2 C++ test or implementation block is forbidden" >&2
  exit 1
fi

echo "swarm_ros_bridge v2 release contract checks passed"
