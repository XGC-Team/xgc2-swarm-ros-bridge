// SPDX-License-Identifier: BSD-3-Clause

#ifndef SWARM_ROS_BRIDGE_V2_ROS1_CODEC_HPP_
#define SWARM_ROS_BRIDGE_V2_ROS1_CODEC_HPP_

#include "swarm_ros_bridge/v2/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <scout_msgs/ScoutStatus.h>
#include <sensor_msgs/Imu.h>

namespace xgc2 {
namespace swarm_bridge {
namespace v2 {

// These values deliberately duplicate the ROS 1 generated traits. Every
// codec call compares the generated trait with this frozen contract before it
// serializes or accepts a payload. A rebuilt message package with a changed
// definition therefore fails closed instead of silently sharing a datatype.
constexpr const char *kRosImuDatatype = "sensor_msgs/Imu";
constexpr const char *kRosImuMd5 = "6a62c6daae103f4ff57a132d6f95cec2";
constexpr const char *kRosImuSchema =
    "xgc.swarm-bridge.ros1.sensor_msgs.Imu.v1";
constexpr const char *kRosOdometryDatatype = "nav_msgs/Odometry";
constexpr const char *kRosOdometryMd5 = "cd5e73d190d741a2f92e81eda573aca7";
constexpr const char *kRosOdometrySchema =
    "xgc.swarm-bridge.ros1.nav_msgs.Odometry.v1";
constexpr const char *kRosScoutStatusDatatype = "scout_msgs/ScoutStatus";
constexpr const char *kRosScoutStatusMd5 = "7a49e199fd32bf5d7341d653c6b3ba6e";
constexpr const char *kRosScoutStatusSchema =
    "xgc.swarm-bridge.ros1.scout_msgs.ScoutStatus.v1";

constexpr std::size_t kMaxRosFrameIdBytes = 255U;
constexpr std::size_t kMaxRosChildFrameIdBytes = 255U;
constexpr std::size_t kMaxImuSerializedBytes = 4096U;
constexpr std::size_t kMaxOdometrySerializedBytes = 4096U;
constexpr std::size_t kMaxScoutStatusSerializedBytes = 4096U;

// The public surface is a closed allowlist. There is intentionally no generic
// ROS-message template and no geometry_msgs/Twist telemetry/control codec.
Error encodeImuTelemetry(const sensor_msgs::Imu &message, FrameHeader envelope,
                         Frame *output);
Error decodeImuTelemetry(const Frame &frame, sensor_msgs::Imu *output);
Error encodeOdometryTelemetry(const nav_msgs::Odometry &message,
                              FrameHeader envelope, Frame *output);
Error decodeOdometryTelemetry(const Frame &frame, nav_msgs::Odometry *output);
Error encodeScoutStatusTelemetry(const scout_msgs::ScoutStatus &message,
                                 FrameHeader envelope, Frame *output);
Error decodeScoutStatusTelemetry(const Frame &frame,
                                 scout_msgs::ScoutStatus *output);

// A Twist can be constructed only from a ReceiveGuard-issued ZERO_STOP
// Admission and that admission's exact canonical frame payload. The function
// has no caller-supplied Twist input and can only emit six positive-zero bits.
Error makeAdmittedPositiveZeroTwist(const Admission &admission,
                                    const Frame &frame,
                                    geometry_msgs::Twist *output);

constexpr std::size_t kMinimumZeroProofSamples = 2U;
constexpr std::size_t kMaximumZeroProofSamples = 1024U;
constexpr std::uint64_t kMaximumZeroProofGapNs = 10000000000ULL;
constexpr std::uint64_t kMaximumZeroProofSpanNs = 10000000000ULL;

struct ZeroProofPolicy {
  // The default is 21 observations spanning at least one second, which can
  // represent a 20 Hz inclusive window while tolerating gaps up to 100 ms.
  std::size_t required_samples = 21U;
  std::uint64_t minimum_span_ns = 1000000000ULL;
  std::uint64_t maximum_gap_ns = 100000000ULL;
};

struct ZeroOutputProof {
  bool proven = false;
  std::size_t consecutive_samples = 0U;
  std::uint64_t first_source_timestamp_ns = 0U;
  std::uint64_t last_source_timestamp_ns = 0U;
  std::uint64_t observed_span_ns = 0U;
};

// Pure status-window evaluators. They do not subscribe, sleep, use wall time,
// or claim physical rest. Count and minimum source-time span must both pass. A
// non-positive-zero observation or excessive gap resets the current suffix;
// invalid/non-advancing source stamps fail closed.
Error proveScoutConsecutivePositiveZero(
    const std::vector<scout_msgs::ScoutStatus> &samples,
    const ZeroProofPolicy &policy, ZeroOutputProof *output);
Error proveMecanumConsecutivePositiveZero(
    const std::vector<nav_msgs::Odometry> &samples,
    const ZeroProofPolicy &policy, ZeroOutputProof *output);

} // namespace v2
} // namespace swarm_bridge
} // namespace xgc2

#endif // SWARM_ROS_BRIDGE_V2_ROS1_CODEC_HPP_
