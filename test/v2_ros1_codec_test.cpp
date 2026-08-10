// SPDX-License-Identifier: BSD-3-Clause

#include "swarm_ros_bridge/v2/ros1_codec.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <ros/message_traits.h>

namespace v2 = xgc2::swarm_bridge::v2;

namespace {

int failures = 0;

void Fail(const char *expression, const char *file, int line,
          const std::string &detail = std::string()) {
  std::cerr << file << ':' << line << ": CHECK failed: " << expression;
  if (!detail.empty()) {
    std::cerr << " (" << detail << ')';
  }
  std::cerr << '\n';
  ++failures;
}

#define CHECK(expression)                                                      \
  do {                                                                         \
    if (!(expression)) {                                                       \
      Fail(#expression, __FILE__, __LINE__);                                   \
    }                                                                          \
  } while (false)

#define CHECK_ERROR(expression, expected)                                      \
  do {                                                                         \
    const v2::Error check_error = (expression);                                \
    if (check_error.code != (expected)) {                                      \
      Fail(#expression, __FILE__, __LINE__,                                    \
           std::string("expected ") + v2::errorCodeName(expected) + ", got " + \
               v2::errorCodeName(check_error.code) + ": " +                    \
               check_error.detail);                                            \
    }                                                                          \
  } while (false)

std::uint64_t Bits(double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

ros::Time Stamp(std::uint64_t nanoseconds) {
  ros::Time stamp;
  stamp.fromNSec(nanoseconds);
  return stamp;
}

v2::FrameHeader BaseEnvelope(std::uint64_t sequence, std::uint64_t monotonic_ns,
                             v2::PeerRole role = v2::PeerRole::kVehicle) {
  v2::FrameHeader header;
  header.sequence = sequence;
  header.monotonic_ns = monotonic_ns;
  header.session_epoch = 17U;
  header.capabilities = role == v2::PeerRole::kVehicle
                            ? v2::kVehicleRequiredCapabilities
                            : v2::kGroundRequiredCapabilities;
  header.slot_id = "scout-01";
  header.asset_id = "asset-a";
  header.robot_kind = "scout";
  header.run_id = "run-42";
  header.boot_id =
      role == v2::PeerRole::kVehicle ? "vehicle-boot" : "ground-boot";
  header.build_id = "build-9";
  return header;
}

sensor_msgs::Imu MakeImu(std::uint64_t source_timestamp_ns) {
  sensor_msgs::Imu message;
  message.header.seq = 7U;
  message.header.stamp = Stamp(source_timestamp_ns);
  message.header.frame_id = "imu_link";
  message.orientation.w = 1.0;
  message.angular_velocity.x = 0.125;
  message.angular_velocity.y = -0.25;
  message.angular_velocity.z = 0.5;
  message.linear_acceleration.x = 9.81;
  for (std::size_t index = 0U; index < 9U; ++index) {
    message.orientation_covariance[index] = static_cast<double>(index) / 10.0;
    message.angular_velocity_covariance[index] =
        static_cast<double>(index + 10U) / 10.0;
    message.linear_acceleration_covariance[index] =
        static_cast<double>(index + 20U) / 10.0;
  }
  return message;
}

nav_msgs::Odometry MakeOdometry(std::uint64_t source_timestamp_ns) {
  nav_msgs::Odometry message;
  message.header.seq = 8U;
  message.header.stamp = Stamp(source_timestamp_ns);
  message.header.frame_id = "odom";
  message.child_frame_id = "base_link";
  message.pose.pose.position.x = 1.25;
  message.pose.pose.position.y = -2.5;
  message.pose.pose.orientation.w = 1.0;
  message.twist.twist.linear.x = 0.75;
  message.twist.twist.linear.y = -0.125;
  message.twist.twist.angular.z = 0.25;
  return message;
}

scout_msgs::ScoutStatus MakeScoutStatus(std::uint64_t source_timestamp_ns) {
  scout_msgs::ScoutStatus message;
  message.header.seq = 9U;
  message.header.stamp = Stamp(source_timestamp_ns);
  message.header.frame_id = "base_link";
  message.linear_velocity = 0.5;
  message.angular_velocity = -0.25;
  message.base_state = 1U;
  message.control_mode = 2U;
  message.fault_code = 3U;
  message.battery_voltage = 48.5;
  message.motor_states[0].rpm = 10.0;
  message.motor_states[3].temperature = 31.0;
  message.light_control_enabled = true;
  message.front_light_state.mode = 1U;
  message.rear_light_state.custom_value = 17U;
  return message;
}

void CheckOuterRoundTrip(const v2::Frame &frame) {
  std::vector<std::uint8_t> wire;
  CHECK(v2::encodeFrame(frame, &wire).ok());
  v2::Frame decoded;
  CHECK(v2::decodeFrame(wire, &decoded).ok());
  CHECK(decoded.header.ros_datatype == frame.header.ros_datatype);
  CHECK(decoded.header.ros_md5 == frame.header.ros_md5);
  CHECK(decoded.header.schema == frame.header.schema);
  CHECK(decoded.header.source_timestamp_ns == frame.header.source_timestamp_ns);
  CHECK(decoded.payload == frame.payload);
}

void TestFrozenTraitsAndTypedRoundTrips() {
  CHECK(std::string(ros::message_traits::DataType<sensor_msgs::Imu>::value()) ==
        v2::kRosImuDatatype);
  CHECK(std::string(ros::message_traits::MD5Sum<sensor_msgs::Imu>::value()) ==
        v2::kRosImuMd5);
  CHECK(
      std::string(ros::message_traits::DataType<nav_msgs::Odometry>::value()) ==
      v2::kRosOdometryDatatype);
  CHECK(std::string(ros::message_traits::MD5Sum<nav_msgs::Odometry>::value()) ==
        v2::kRosOdometryMd5);
  CHECK(std::string(
            ros::message_traits::DataType<scout_msgs::ScoutStatus>::value()) ==
        v2::kRosScoutStatusDatatype);
  CHECK(std::string(
            ros::message_traits::MD5Sum<scout_msgs::ScoutStatus>::value()) ==
        v2::kRosScoutStatusMd5);
  CHECK(std::string(
            ros::message_traits::DataType<geometry_msgs::Twist>::value()) ==
        "geometry_msgs/Twist");
  CHECK(
      std::string(ros::message_traits::MD5Sum<geometry_msgs::Twist>::value()) ==
      "9f195f881246fdfa2798d1d3eebca84a");

  constexpr std::uint64_t kImuStamp = 12345678901ULL;
  const sensor_msgs::Imu imu = MakeImu(kImuStamp);
  v2::Frame imu_frame;
  CHECK(v2::encodeImuTelemetry(imu, BaseEnvelope(1U, 1000000000ULL), &imu_frame)
            .ok());
  CHECK(imu_frame.header.channel == v2::Channel::kTelemetry);
  CHECK(imu_frame.header.kind == v2::MessageKind::kRosMessage);
  CHECK(imu_frame.header.source_timestamp_ns == kImuStamp);
  CHECK(imu_frame.header.ros_datatype == v2::kRosImuDatatype);
  CHECK(imu_frame.header.ros_md5 == v2::kRosImuMd5);
  CHECK(imu_frame.header.schema == v2::kRosImuSchema);
  sensor_msgs::Imu decoded_imu;
  CHECK(v2::decodeImuTelemetry(imu_frame, &decoded_imu).ok());
  CHECK(decoded_imu.header.stamp.toNSec() == kImuStamp);
  CHECK(decoded_imu.header.frame_id == "imu_link");
  CHECK(decoded_imu.angular_velocity.y == -0.25);
  CHECK(decoded_imu.linear_acceleration.x == 9.81);
  CheckOuterRoundTrip(imu_frame);

  constexpr std::uint64_t kOdomStamp = 22345678901ULL;
  const nav_msgs::Odometry odometry = MakeOdometry(kOdomStamp);
  v2::Frame odometry_frame;
  CHECK(v2::encodeOdometryTelemetry(odometry, BaseEnvelope(2U, 1010000000ULL),
                                    &odometry_frame)
            .ok());
  CHECK(odometry_frame.header.source_timestamp_ns == kOdomStamp);
  CHECK(odometry_frame.header.ros_datatype == v2::kRosOdometryDatatype);
  nav_msgs::Odometry decoded_odometry;
  CHECK(v2::decodeOdometryTelemetry(odometry_frame, &decoded_odometry).ok());
  CHECK(decoded_odometry.header.stamp.toNSec() == kOdomStamp);
  CHECK(decoded_odometry.child_frame_id == "base_link");
  CHECK(decoded_odometry.pose.pose.position.y == -2.5);
  CHECK(decoded_odometry.twist.twist.linear.x == 0.75);
  CheckOuterRoundTrip(odometry_frame);

  constexpr std::uint64_t kScoutStamp = 32345678901ULL;
  const scout_msgs::ScoutStatus scout = MakeScoutStatus(kScoutStamp);
  v2::Frame scout_frame;
  CHECK(v2::encodeScoutStatusTelemetry(scout, BaseEnvelope(3U, 1020000000ULL),
                                       &scout_frame)
            .ok());
  CHECK(scout_frame.header.source_timestamp_ns == kScoutStamp);
  CHECK(scout_frame.header.ros_datatype == v2::kRosScoutStatusDatatype);
  scout_msgs::ScoutStatus decoded_scout;
  CHECK(v2::decodeScoutStatusTelemetry(scout_frame, &decoded_scout).ok());
  CHECK(decoded_scout.header.stamp.toNSec() == kScoutStamp);
  CHECK(decoded_scout.linear_velocity == 0.5);
  CHECK(decoded_scout.motor_states[0].rpm == 10.0);
  CHECK(decoded_scout.rear_light_state.custom_value == 17U);
  CheckOuterRoundTrip(scout_frame);
}

void TestTelemetryFailuresAndBounds() {
  v2::Frame frame;
  sensor_msgs::Imu imu = MakeImu(11000000000ULL);
  CHECK(v2::encodeImuTelemetry(imu, BaseEnvelope(1U, 1000000000ULL), &frame)
            .ok());

  sensor_msgs::Imu ignored;
  for (std::size_t length = 0U; length < frame.payload.size(); ++length) {
    v2::Frame truncated = frame;
    truncated.payload.resize(length);
    CHECK(!v2::decodeImuTelemetry(truncated, &ignored).ok());
  }
  v2::Frame wrong = frame;
  wrong.header.ros_datatype = v2::kRosOdometryDatatype;
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kInvalidMetadata);
  wrong = frame;
  wrong.header.ros_md5 = v2::kRosOdometryMd5;
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kInvalidMetadata);
  wrong = frame;
  wrong.header.schema = v2::kRosOdometrySchema;
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kInvalidMetadata);
  wrong = frame;
  wrong.header.channel = v2::Channel::kControl;
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kInvalidMetadata);
  wrong = frame;
  ++wrong.header.source_timestamp_ns;
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kInvalidMetadata);

  wrong = frame;
  wrong.payload.pop_back();
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kInvalidPayload);
  wrong = frame;
  wrong.payload.push_back(0U);
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kInvalidPayload);
  wrong = frame;
  CHECK(wrong.payload.size() > 16U);
  wrong.payload[12U] = 0xffU;
  wrong.payload[13U] = 0xffU;
  wrong.payload[14U] = 0xffU;
  wrong.payload[15U] = 0x7fU;
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kInvalidPayload);
  wrong = frame;
  wrong.payload.assign(v2::kMaxImuSerializedBytes + 1U, 0U);
  CHECK_ERROR(v2::decodeImuTelemetry(wrong, &ignored),
              v2::ErrorCode::kLimitExceeded);

  nav_msgs::Odometry odometry = MakeOdometry(11000000000ULL);
  v2::Frame odometry_frame;
  CHECK(v2::encodeOdometryTelemetry(odometry, BaseEnvelope(2U, 1010000000ULL),
                                    &odometry_frame)
            .ok());
  wrong = frame;
  wrong.payload = odometry_frame.payload;
  CHECK(!v2::decodeImuTelemetry(wrong, &ignored).ok());

  imu.header.stamp = ros::Time{};
  CHECK_ERROR(
      v2::encodeImuTelemetry(imu, BaseEnvelope(3U, 1020000000ULL), &frame),
      v2::ErrorCode::kInvalidMetadata);
  imu = MakeImu(11000000000ULL);
  imu.header.stamp.nsec = 1000000000U;
  CHECK_ERROR(
      v2::encodeImuTelemetry(imu, BaseEnvelope(3U, 1020000000ULL), &frame),
      v2::ErrorCode::kInvalidMetadata);
  imu = MakeImu(11000000000ULL);
  imu.header.frame_id.assign(v2::kMaxRosFrameIdBytes, 'f');
  CHECK(v2::encodeImuTelemetry(imu, BaseEnvelope(3U, 1020000000ULL), &frame)
            .ok());
  imu.header.frame_id.assign(v2::kMaxRosFrameIdBytes + 1U, 'f');
  CHECK_ERROR(
      v2::encodeImuTelemetry(imu, BaseEnvelope(3U, 1020000000ULL), &frame),
      v2::ErrorCode::kLimitExceeded);

  odometry.header.frame_id.assign(v2::kMaxRosFrameIdBytes + 1U, 'f');
  CHECK_ERROR(v2::encodeOdometryTelemetry(
                  odometry, BaseEnvelope(4U, 1030000000ULL), &frame),
              v2::ErrorCode::kLimitExceeded);
  odometry = MakeOdometry(11000000000ULL);
  odometry.child_frame_id.assign(v2::kMaxRosChildFrameIdBytes + 1U, 'c');
  CHECK_ERROR(v2::encodeOdometryTelemetry(
                  odometry, BaseEnvelope(4U, 1030000000ULL), &frame),
              v2::ErrorCode::kLimitExceeded);

  scout_msgs::ScoutStatus scout = MakeScoutStatus(11000000000ULL);
  scout.header.frame_id.assign(v2::kMaxRosFrameIdBytes + 1U, 'f');
  CHECK_ERROR(v2::encodeScoutStatusTelemetry(
                  scout, BaseEnvelope(5U, 1040000000ULL), &frame),
              v2::ErrorCode::kLimitExceeded);

  CHECK_ERROR(v2::encodeImuTelemetry(MakeImu(11000000000ULL),
                                     BaseEnvelope(6U, 1050000000ULL), nullptr),
              v2::ErrorCode::kNullOutput);
  CHECK_ERROR(v2::decodeImuTelemetry(frame, nullptr),
              v2::ErrorCode::kNullOutput);
}

v2::Frame MakeControlHello(std::uint64_t sequence, std::uint64_t monotonic_ns) {
  v2::Frame frame;
  frame.header = BaseEnvelope(sequence, monotonic_ns, v2::PeerRole::kGround);
  frame.header.channel = v2::Channel::kControl;
  frame.header.kind = v2::MessageKind::kHello;
  frame.header.schema = v2::kHelloSchema;
  CHECK(v2::encodeHelloPayload(v2::Hello{}, &frame.payload).ok());
  return frame;
}

v2::Frame MakeControlHeartbeat(std::uint64_t sequence,
                               std::uint64_t monotonic_ns) {
  v2::Frame frame;
  frame.header = BaseEnvelope(sequence, monotonic_ns, v2::PeerRole::kGround);
  frame.header.channel = v2::Channel::kControl;
  frame.header.kind = v2::MessageKind::kHeartbeat;
  frame.header.schema = v2::kHeartbeatSchema;
  CHECK(v2::encodeHeartbeatPayload(v2::Heartbeat{sequence - 1U, 1U},
                                   &frame.payload)
            .ok());
  return frame;
}

v2::Frame MakeZeroStopFrame(std::uint64_t sequence,
                            std::uint64_t monotonic_ns) {
  v2::ZeroStop stop;
  stop.command_id = "stop-0001";
  stop.deadline_monotonic_ns = monotonic_ns + 500000000ULL;
  v2::Frame frame;
  frame.header = BaseEnvelope(sequence, monotonic_ns, v2::PeerRole::kGround);
  frame.header.channel = v2::Channel::kControl;
  frame.header.kind = v2::MessageKind::kZeroStop;
  frame.header.schema = v2::kZeroStopSchema;
  CHECK(v2::encodeZeroStopPayload(stop, &frame.payload).ok());
  return frame;
}

void StoreU64(std::vector<std::uint8_t> *bytes, std::size_t offset,
              std::uint64_t value) {
  CHECK(bytes != nullptr);
  CHECK(bytes != nullptr && offset + 8U <= bytes->size());
  if (bytes == nullptr || offset + 8U > bytes->size()) {
    return;
  }
  for (std::size_t index = 0U; index < 8U; ++index) {
    const unsigned shift = static_cast<unsigned>((7U - index) * 8U);
    (*bytes)[offset + index] =
        static_cast<std::uint8_t>((value >> shift) & 0xffU);
  }
}

void TestAdmissionOnlyPositiveZeroTwist() {
  v2::ExpectedPeer expected;
  expected.slot_id = "scout-01";
  expected.asset_id = "asset-a";
  expected.robot_kind = "scout";
  expected.run_id = "run-42";
  expected.session_epoch = 17U;
  expected.boot_id = "ground-boot";
  expected.build_id = "build-9";
  expected.local_role = v2::PeerRole::kVehicle;
  expected.local_capabilities = v2::kVehicleRequiredCapabilities;
  expected.peer_role = v2::PeerRole::kGround;
  expected.capabilities = v2::kGroundRequiredCapabilities;
  v2::MonotonicTimebase timebase;
  timebase.sender_anchor_ns = 1000000000ULL;
  timebase.receiver_anchor_ns = 10000000000ULL;
  timebase.maximum_error_ns = 1000000ULL;
  timebase.valid_until_receiver_ns = 15000000000ULL;
  v2::ReceiveGuard guard(expected, v2::Channel::kControl, timebase);

  v2::Admission admission;
  CHECK(guard
            .accept(MakeControlHello(1U, 1000000000ULL), 10000000000ULL,
                    &admission)
            .ok());
  CHECK(guard
            .accept(MakeControlHeartbeat(2U, 1010000000ULL), 10010000000ULL,
                    &admission)
            .ok());

  const v2::Frame valid_stop = MakeZeroStopFrame(3U, 1020000000ULL);
  const std::size_t first_axis_offset =
      8U + 2U + std::string("stop-0001").size();
  const std::uint64_t forbidden_bits[] = {
      0x8000000000000000ULL, // negative zero
      0x7ff8000000000000ULL, // quiet NaN
      0x3ff0000000000000ULL, // positive one
      0xbff0000000000000ULL, // negative one
  };
  for (std::uint64_t bits : forbidden_bits) {
    v2::Frame unsafe = valid_stop;
    StoreU64(&unsafe.payload, first_axis_offset, bits);
    v2::Admission rejected;
    CHECK_ERROR(guard.accept(unsafe, 10020000000ULL, &rejected),
                v2::ErrorCode::kUnsafeNonZero);
    CHECK(!rejected.valid());
  }
  v2::Frame malformed = valid_stop;
  malformed.payload.pop_back();
  CHECK_ERROR(guard.accept(malformed, 10020000000ULL, &admission),
              v2::ErrorCode::kInvalidLength);
  CHECK(!admission.valid());

  CHECK(guard.accept(valid_stop, 10020000000ULL, &admission).ok());
  CHECK(admission.valid());
  CHECK(admission.matchesCanonicalFrame(valid_stop));
  geometry_msgs::Twist twist;
  twist.linear.x = 123.0;
  CHECK(v2::makeAdmittedPositiveZeroTwist(admission, valid_stop, &twist).ok());
  CHECK(Bits(twist.linear.x) == 0U);
  CHECK(Bits(twist.linear.y) == 0U);
  CHECK(Bits(twist.linear.z) == 0U);
  CHECK(Bits(twist.angular.x) == 0U);
  CHECK(Bits(twist.angular.y) == 0U);
  CHECK(Bits(twist.angular.z) == 0U);

  v2::Admission missing;
  CHECK_ERROR(v2::makeAdmittedPositiveZeroTwist(missing, valid_stop, &twist),
              v2::ErrorCode::kInvalidState);
  v2::Frame wrong_frame = valid_stop;
  ++wrong_frame.header.sequence;
  CHECK_ERROR(v2::makeAdmittedPositiveZeroTwist(admission, wrong_frame, &twist),
              v2::ErrorCode::kInvalidState);
  wrong_frame = valid_stop;
  ++wrong_frame.header.monotonic_ns;
  CHECK_ERROR(v2::makeAdmittedPositiveZeroTwist(admission, wrong_frame, &twist),
              v2::ErrorCode::kInvalidState);
  wrong_frame = valid_stop;
  wrong_frame.header.source_timestamp_ns = 1U;
  CHECK_ERROR(v2::makeAdmittedPositiveZeroTwist(admission, wrong_frame, &twist),
              v2::ErrorCode::kInvalidState);
  wrong_frame = valid_stop;
  wrong_frame.header.schema = "xgc.swarm-bridge.zero-stop.changed.v2";
  CHECK_ERROR(v2::makeAdmittedPositiveZeroTwist(admission, wrong_frame, &twist),
              v2::ErrorCode::kInvalidState);
  wrong_frame = valid_stop;
  wrong_frame.payload.back() ^= 1U;
  CHECK_ERROR(v2::makeAdmittedPositiveZeroTwist(admission, wrong_frame, &twist),
              v2::ErrorCode::kInvalidState);
  CHECK_ERROR(v2::makeAdmittedPositiveZeroTwist(admission, valid_stop, nullptr),
              v2::ErrorCode::kNullOutput);
}

scout_msgs::ScoutStatus ScoutZero(std::uint64_t timestamp_ns) {
  scout_msgs::ScoutStatus sample;
  sample.header.stamp = Stamp(timestamp_ns);
  sample.linear_velocity = 0.0;
  sample.angular_velocity = 0.0;
  return sample;
}

nav_msgs::Odometry MecanumZero(std::uint64_t timestamp_ns) {
  nav_msgs::Odometry sample;
  sample.header.stamp = Stamp(timestamp_ns);
  sample.twist.twist.linear.x = 0.0;
  sample.twist.twist.linear.y = 0.0;
  sample.twist.twist.linear.z = 0.0;
  sample.twist.twist.angular.x = 0.0;
  sample.twist.twist.angular.y = 0.0;
  sample.twist.twist.angular.z = 0.0;
  return sample;
}

void TestPureConsecutiveZeroProofs() {
  const v2::ZeroProofPolicy policy{3U, 20000000ULL, 20000000ULL};
  v2::ZeroOutputProof proof;
  std::vector<scout_msgs::ScoutStatus> scout_samples{ScoutZero(1000000000ULL),
                                                     ScoutZero(1010000000ULL),
                                                     ScoutZero(1020000000ULL)};
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(proof.proven);
  CHECK(proof.consecutive_samples == 3U);
  CHECK(proof.first_source_timestamp_ns == 1000000000ULL);
  CHECK(proof.last_source_timestamp_ns == 1020000000ULL);
  CHECK(proof.observed_span_ns == 20000000ULL);

  scout_samples[1].linear_velocity = -0.0;
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 1U);
  scout_samples[1] = ScoutZero(1010000000ULL);
  scout_samples[2].angular_velocity = std::numeric_limits<double>::quiet_NaN();
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 0U);
  scout_samples[2] = ScoutZero(1020000000ULL);
  scout_samples[0].linear_velocity = -1.0;
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 2U);

  scout_samples = {ScoutZero(1200000000ULL), ScoutZero(1200000001ULL),
                   ScoutZero(1200000002ULL)};
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 3U);
  CHECK(proof.observed_span_ns == 2U);
  scout_samples = {ScoutZero(1000000000ULL), ScoutZero(1010000000ULL),
                   ScoutZero(1020000000ULL)};
  scout_samples[1].motor_states[2].rpm = 1.0;
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 1U);
  scout_samples[1] = ScoutZero(1010000000ULL);
  scout_samples[1].motor_states[0].rpm = -0.0;
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 1U);
  scout_samples[1] = ScoutZero(1010000000ULL);
  scout_samples[2].motor_states[3].rpm =
      std::numeric_limits<double>::quiet_NaN();
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 0U);

  scout_samples = {ScoutZero(1000000000ULL), ScoutZero(1100000000ULL),
                   ScoutZero(1110000000ULL)};
  CHECK(v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 2U);
  scout_samples[2].header.stamp = scout_samples[1].header.stamp;
  CHECK_ERROR(
      v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof),
      v2::ErrorCode::kOutOfOrderTimestamp);
  scout_samples = {ScoutZero(0U)};
  CHECK_ERROR(
      v2::proveScoutConsecutivePositiveZero(scout_samples, policy, &proof),
      v2::ErrorCode::kInvalidMetadata);

  std::vector<nav_msgs::Odometry> mecanum_samples{MecanumZero(2000000000ULL),
                                                  MecanumZero(2010000000ULL),
                                                  MecanumZero(2020000000ULL)};
  CHECK(v2::proveMecanumConsecutivePositiveZero(mecanum_samples, policy, &proof)
            .ok());
  CHECK(proof.proven && proof.consecutive_samples == 3U);
  mecanum_samples[1].twist.twist.linear.y = -0.0;
  CHECK(v2::proveMecanumConsecutivePositiveZero(mecanum_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 1U);
  mecanum_samples[1] = MecanumZero(2010000000ULL);
  mecanum_samples[2].twist.twist.angular.z =
      std::numeric_limits<double>::quiet_NaN();
  CHECK(v2::proveMecanumConsecutivePositiveZero(mecanum_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 0U);
  mecanum_samples[2] = MecanumZero(2020000000ULL);
  mecanum_samples[0].twist.twist.linear.x = 0.01;
  CHECK(v2::proveMecanumConsecutivePositiveZero(mecanum_samples, policy, &proof)
            .ok());
  CHECK(!proof.proven && proof.consecutive_samples == 2U);

  CHECK_ERROR(v2::proveMecanumConsecutivePositiveZero(
                  mecanum_samples,
                  v2::ZeroProofPolicy{1U, 20000000ULL, 20000000ULL}, &proof),
              v2::ErrorCode::kLimitExceeded);
  CHECK_ERROR(
      v2::proveMecanumConsecutivePositiveZero(
          mecanum_samples, v2::ZeroProofPolicy{3U, 20000000ULL, 0U}, &proof),
      v2::ErrorCode::kLimitExceeded);
  CHECK_ERROR(
      v2::proveMecanumConsecutivePositiveZero(
          mecanum_samples, v2::ZeroProofPolicy{3U, 0U, 20000000ULL}, &proof),
      v2::ErrorCode::kLimitExceeded);
  CHECK_ERROR(
      v2::proveMecanumConsecutivePositiveZero(mecanum_samples, policy, nullptr),
      v2::ErrorCode::kNullOutput);
  CHECK_ERROR(v2::proveMecanumConsecutivePositiveZero(
                  mecanum_samples,
                  v2::ZeroProofPolicy{v2::kMaximumZeroProofSamples + 1U,
                                      20000000ULL, 20000000ULL},
                  &proof),
              v2::ErrorCode::kLimitExceeded);
  std::vector<nav_msgs::Odometry> oversized_window(
      v2::kMaximumZeroProofSamples + 1U, MecanumZero(3000000000ULL));
  CHECK_ERROR(
      v2::proveMecanumConsecutivePositiveZero(oversized_window, policy, &proof),
      v2::ErrorCode::kLimitExceeded);
}

} // namespace

int main() {
  TestFrozenTraitsAndTypedRoundTrips();
  TestTelemetryFailuresAndBounds();
  TestAdmissionOnlyPositiveZeroTwist();
  TestPureConsecutiveZeroProofs();
  if (failures != 0) {
    std::cerr << failures << " ROS1 codec test failure(s)\n";
    return 1;
  }
  std::cout << "swarm_ros_bridge v2 ROS1 typed codec tests passed\n";
  return 0;
}
