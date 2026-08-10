// SPDX-License-Identifier: BSD-3-Clause

#include "swarm_ros_bridge/v2/ros1_codec.hpp"

#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <utility>

#include <ros/message_traits.h>
#include <ros/serialization.h>

namespace xgc2 {
namespace swarm_bridge {
namespace v2 {
namespace {

static_assert(std::numeric_limits<double>::is_iec559 &&
                  std::numeric_limits<double>::digits == 53,
              "ROS 1 zero-output proof requires IEEE-754 binary64 doubles");

Error MakeError(ErrorCode code, const std::string &detail) {
  return Error{code, detail};
}

bool IsPositiveZero(double value) {
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits == 0U;
}

double PositiveZero() {
  std::uint64_t bits = 0U;
  double value = 1.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

template <typename Message>
Error VerifyTraits(const char *datatype, const char *md5) {
  if (std::string(ros::message_traits::DataType<Message>::value()) !=
          datatype ||
      std::string(ros::message_traits::MD5Sum<Message>::value()) != md5) {
    return MakeError(
        ErrorCode::kInvalidMetadata,
        "generated ROS datatype or MD5 differs from the frozen codec contract");
  }
  return Error{};
}

Error SourceTimestamp(const ros::Time &stamp, std::uint64_t *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "source timestamp output is null");
  }
  if (stamp.nsec >= 1000000000U || (stamp.sec == 0U && stamp.nsec == 0U)) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "ROS source stamp must be normalized and non-zero");
  }
  *output = static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
            static_cast<std::uint64_t>(stamp.nsec);
  return Error{};
}

Error ValidateMessageBounds(const sensor_msgs::Imu &message) {
  if (message.header.frame_id.size() > kMaxRosFrameIdBytes) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "IMU frame_id exceeds the frozen byte bound");
  }
  return Error{};
}

Error ValidateMessageBounds(const nav_msgs::Odometry &message) {
  if (message.header.frame_id.size() > kMaxRosFrameIdBytes ||
      message.child_frame_id.size() > kMaxRosChildFrameIdBytes) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "odometry frame_id exceeds the frozen byte bound");
  }
  return Error{};
}

Error ValidateMessageBounds(const scout_msgs::ScoutStatus &message) {
  if (message.header.frame_id.size() > kMaxRosFrameIdBytes) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "ScoutStatus frame_id exceeds the frozen byte bound");
  }
  return Error{};
}

template <typename Message>
Error Serialize(const Message &message, std::size_t maximum,
                std::vector<std::uint8_t> *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput,
                     "ROS serialization output is null");
  }
  try {
    const std::uint32_t length =
        ros::serialization::serializationLength(message);
    if (length == 0U) {
      return MakeError(ErrorCode::kInvalidPayload,
                       "serialized ROS payload must not be empty");
    }
    if (static_cast<std::size_t>(length) > maximum) {
      return MakeError(ErrorCode::kLimitExceeded,
                       "serialized ROS payload exceeds its type bound");
    }
    std::vector<std::uint8_t> candidate(length);
    ros::serialization::OStream stream(candidate.data(), length);
    ros::serialization::serialize(stream, message);
    if (stream.getLength() != 0U) {
      return MakeError(ErrorCode::kInvalidPayload,
                       "ROS serializer did not consume its declared length");
    }
    *output = std::move(candidate);
    return Error{};
  } catch (const std::exception &) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "ROS serialization rejected the typed payload");
  } catch (...) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "ROS serialization failed with an unknown exception");
  }
}

template <typename Message>
Error DeserializeCanonical(const std::vector<std::uint8_t> &payload,
                           std::size_t maximum, Message *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput,
                     "ROS deserialization output is null");
  }
  if (payload.empty()) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "serialized ROS payload must not be empty");
  }
  if (payload.size() > maximum) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "serialized ROS payload exceeds its type bound");
  }
  try {
    // ROS 1's IStream accepts a mutable pointer even though deserialization
    // does not alter the bytes. A bounded copy avoids casting away constness.
    std::vector<std::uint8_t> scratch(payload);
    Message candidate;
    ros::serialization::IStream stream(
        scratch.data(), static_cast<std::uint32_t>(scratch.size()));
    ros::serialization::deserialize(stream, candidate);
    if (stream.getLength() != 0U) {
      return MakeError(ErrorCode::kInvalidPayload,
                       "serialized ROS payload has trailing bytes");
    }

    Error error = ValidateMessageBounds(candidate);
    if (!error.ok()) {
      return error;
    }
    std::vector<std::uint8_t> canonical;
    error = Serialize(candidate, maximum, &canonical);
    if (!error.ok()) {
      return error;
    }
    if (canonical != payload) {
      return MakeError(ErrorCode::kInvalidPayload,
                       "ROS payload does not round-trip byte-for-byte");
    }
    *output = std::move(candidate);
    return Error{};
  } catch (const std::exception &) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "ROS deserialization rejected the typed payload");
  } catch (...) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "ROS deserialization failed with an unknown exception");
  }
}

template <typename Message>
Error EncodeTelemetry(const Message &message, FrameHeader envelope,
                      const char *datatype, const char *md5, const char *schema,
                      std::size_t maximum, Frame *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput,
                     "typed telemetry frame output is null");
  }
  Error error = VerifyTraits<Message>(datatype, md5);
  if (!error.ok()) {
    return error;
  }
  error = ValidateMessageBounds(message);
  if (!error.ok()) {
    return error;
  }
  std::uint64_t source_timestamp_ns = 0U;
  error = SourceTimestamp(message.header.stamp, &source_timestamp_ns);
  if (!error.ok()) {
    return error;
  }

  Frame candidate;
  candidate.header = std::move(envelope);
  candidate.header.channel = Channel::kTelemetry;
  candidate.header.kind = MessageKind::kRosMessage;
  candidate.header.source_timestamp_ns = source_timestamp_ns;
  candidate.header.ros_datatype = datatype;
  candidate.header.ros_md5 = md5;
  candidate.header.schema = schema;
  error = Serialize(message, maximum, &candidate.payload);
  if (!error.ok()) {
    return error;
  }
  error = validateFrame(candidate);
  if (!error.ok()) {
    return error;
  }
  *output = std::move(candidate);
  return Error{};
}

template <typename Message>
Error DecodeTelemetry(const Frame &frame, const char *datatype, const char *md5,
                      const char *schema, std::size_t maximum,
                      Message *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput,
                     "typed telemetry message output is null");
  }
  Error error = VerifyTraits<Message>(datatype, md5);
  if (!error.ok()) {
    return error;
  }
  error = validateFrame(frame);
  if (!error.ok()) {
    return error;
  }
  if (frame.header.channel != Channel::kTelemetry ||
      frame.header.kind != MessageKind::kRosMessage ||
      frame.header.ros_datatype != datatype || frame.header.ros_md5 != md5 ||
      frame.header.schema != schema) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "ROS frame is outside the typed telemetry allowlist");
  }

  Message candidate;
  error = DeserializeCanonical(frame.payload, maximum, &candidate);
  if (!error.ok()) {
    return error;
  }
  std::uint64_t source_timestamp_ns = 0U;
  error = SourceTimestamp(candidate.header.stamp, &source_timestamp_ns);
  if (!error.ok()) {
    return error;
  }
  if (source_timestamp_ns != frame.header.source_timestamp_ns) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "ROS Header stamp differs from frame source timestamp");
  }
  *output = std::move(candidate);
  return Error{};
}

Error ValidateProofPolicy(const ZeroProofPolicy &policy) {
  if (policy.required_samples < kMinimumZeroProofSamples ||
      policy.required_samples > kMaximumZeroProofSamples) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "zero proof sample count is outside its frozen bounds");
  }
  if (policy.maximum_gap_ns == 0U ||
      policy.maximum_gap_ns > kMaximumZeroProofGapNs) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "zero proof maximum gap is outside its frozen bounds");
  }
  if (policy.minimum_span_ns == 0U ||
      policy.minimum_span_ns > kMaximumZeroProofSpanNs) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "zero proof minimum span is outside its frozen bounds");
  }
  const std::uint64_t maximum_observable_span_ns =
      static_cast<std::uint64_t>(kMaximumZeroProofSamples - 1U) *
      policy.maximum_gap_ns;
  if (policy.minimum_span_ns > maximum_observable_span_ns) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "zero proof policy cannot satisfy its minimum span");
  }
  return Error{};
}

bool IsScoutPositiveZero(const scout_msgs::ScoutStatus &message) {
  if (!IsPositiveZero(message.linear_velocity) ||
      !IsPositiveZero(message.angular_velocity)) {
    return false;
  }
  for (const scout_msgs::ScoutMotorState &motor : message.motor_states) {
    if (!IsPositiveZero(motor.rpm)) {
      return false;
    }
  }
  return true;
}

bool IsTwistPositiveZero(const geometry_msgs::Twist &twist) {
  return IsPositiveZero(twist.linear.x) && IsPositiveZero(twist.linear.y) &&
         IsPositiveZero(twist.linear.z) && IsPositiveZero(twist.angular.x) &&
         IsPositiveZero(twist.angular.y) && IsPositiveZero(twist.angular.z);
}

bool IsMecanumPositiveZero(const nav_msgs::Odometry &message) {
  return IsTwistPositiveZero(message.twist.twist);
}

template <typename Message, typename ZeroPredicate>
Error ProveConsecutivePositiveZero(const std::vector<Message> &samples,
                                   const ZeroProofPolicy &policy,
                                   ZeroPredicate is_zero,
                                   ZeroOutputProof *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "zero proof output is null");
  }
  Error error = ValidateProofPolicy(policy);
  if (!error.ok()) {
    return error;
  }
  if (samples.size() > kMaximumZeroProofSamples) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "zero proof observation window is too large");
  }

  ZeroOutputProof candidate;
  std::uint64_t previous_timestamp_ns = 0U;
  for (const Message &sample : samples) {
    std::uint64_t timestamp_ns = 0U;
    error = SourceTimestamp(sample.header.stamp, &timestamp_ns);
    if (!error.ok()) {
      return error;
    }
    if (previous_timestamp_ns != 0U && timestamp_ns <= previous_timestamp_ns) {
      return MakeError(ErrorCode::kOutOfOrderTimestamp,
                       "zero proof source stamps must strictly advance");
    }
    const bool excessive_gap =
        previous_timestamp_ns != 0U &&
        timestamp_ns - previous_timestamp_ns > policy.maximum_gap_ns;
    const bool positive_zero = is_zero(sample);
    if (excessive_gap || !positive_zero) {
      candidate.consecutive_samples = 0U;
      candidate.first_source_timestamp_ns = 0U;
      candidate.last_source_timestamp_ns = 0U;
    }
    if (positive_zero) {
      if (candidate.consecutive_samples == 0U) {
        candidate.first_source_timestamp_ns = timestamp_ns;
      }
      ++candidate.consecutive_samples;
      candidate.last_source_timestamp_ns = timestamp_ns;
    }
    previous_timestamp_ns = timestamp_ns;
  }
  if (candidate.consecutive_samples != 0U) {
    candidate.observed_span_ns = candidate.last_source_timestamp_ns -
                                 candidate.first_source_timestamp_ns;
  }
  candidate.proven = candidate.consecutive_samples >= policy.required_samples &&
                     candidate.observed_span_ns >= policy.minimum_span_ns;
  *output = candidate;
  return Error{};
}

} // namespace

Error encodeImuTelemetry(const sensor_msgs::Imu &message, FrameHeader envelope,
                         Frame *output) {
  return EncodeTelemetry(message, std::move(envelope), kRosImuDatatype,
                         kRosImuMd5, kRosImuSchema, kMaxImuSerializedBytes,
                         output);
}

Error decodeImuTelemetry(const Frame &frame, sensor_msgs::Imu *output) {
  return DecodeTelemetry(frame, kRosImuDatatype, kRosImuMd5, kRosImuSchema,
                         kMaxImuSerializedBytes, output);
}

Error encodeOdometryTelemetry(const nav_msgs::Odometry &message,
                              FrameHeader envelope, Frame *output) {
  return EncodeTelemetry(message, std::move(envelope), kRosOdometryDatatype,
                         kRosOdometryMd5, kRosOdometrySchema,
                         kMaxOdometrySerializedBytes, output);
}

Error decodeOdometryTelemetry(const Frame &frame, nav_msgs::Odometry *output) {
  return DecodeTelemetry(frame, kRosOdometryDatatype, kRosOdometryMd5,
                         kRosOdometrySchema, kMaxOdometrySerializedBytes,
                         output);
}

Error encodeScoutStatusTelemetry(const scout_msgs::ScoutStatus &message,
                                 FrameHeader envelope, Frame *output) {
  return EncodeTelemetry(message, std::move(envelope), kRosScoutStatusDatatype,
                         kRosScoutStatusMd5, kRosScoutStatusSchema,
                         kMaxScoutStatusSerializedBytes, output);
}

Error decodeScoutStatusTelemetry(const Frame &frame,
                                 scout_msgs::ScoutStatus *output) {
  return DecodeTelemetry(frame, kRosScoutStatusDatatype, kRosScoutStatusMd5,
                         kRosScoutStatusSchema, kMaxScoutStatusSerializedBytes,
                         output);
}

Error makeAdmittedPositiveZeroTwist(const Admission &admission,
                                    const Frame &frame,
                                    geometry_msgs::Twist *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput,
                     "positive-zero Twist output is null");
  }
  if (frame.header.channel != Channel::kControl ||
      frame.header.kind != MessageKind::kZeroStop ||
      !admission.matchesCanonicalFrame(frame)) {
    return MakeError(
        ErrorCode::kInvalidState,
        "positive-zero Twist requires the exact admitted ZERO_STOP frame");
  }
  Error error = validateFrame(frame);
  if (!error.ok()) {
    return error;
  }
  ZeroStop stop;
  error = decodeZeroStopPayload(frame.payload, &stop);
  if (!error.ok()) {
    return error;
  }
  if (stop.command_id != admission.commandId()) {
    return MakeError(ErrorCode::kInvalidState,
                     "ZERO_STOP command does not match its admission");
  }

  geometry_msgs::Twist candidate;
  candidate.linear.x = PositiveZero();
  candidate.linear.y = PositiveZero();
  candidate.linear.z = PositiveZero();
  candidate.angular.x = PositiveZero();
  candidate.angular.y = PositiveZero();
  candidate.angular.z = PositiveZero();
  if (!IsTwistPositiveZero(candidate)) {
    return MakeError(ErrorCode::kUnsafeNonZero,
                     "constructed Twist is not bit-exact positive zero");
  }
  *output = candidate;
  return Error{};
}

Error proveScoutConsecutivePositiveZero(
    const std::vector<scout_msgs::ScoutStatus> &samples,
    const ZeroProofPolicy &policy, ZeroOutputProof *output) {
  Error error = VerifyTraits<scout_msgs::ScoutStatus>(kRosScoutStatusDatatype,
                                                      kRosScoutStatusMd5);
  if (!error.ok()) {
    return error;
  }
  return ProveConsecutivePositiveZero(samples, policy, IsScoutPositiveZero,
                                      output);
}

Error proveMecanumConsecutivePositiveZero(
    const std::vector<nav_msgs::Odometry> &samples,
    const ZeroProofPolicy &policy, ZeroOutputProof *output) {
  Error error =
      VerifyTraits<nav_msgs::Odometry>(kRosOdometryDatatype, kRosOdometryMd5);
  if (!error.ok()) {
    return error;
  }
  return ProveConsecutivePositiveZero(samples, policy, IsMecanumPositiveZero,
                                      output);
}

} // namespace v2
} // namespace swarm_bridge
} // namespace xgc2
