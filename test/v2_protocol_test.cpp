// SPDX-License-Identifier: BSD-3-Clause

#include "swarm_ros_bridge/v2/protocol.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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

std::string Hex(const std::vector<std::uint8_t> &bytes) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::uint8_t byte : bytes) {
    stream << std::setw(2) << static_cast<unsigned>(byte);
  }
  return stream.str();
}

void StoreU16(std::vector<std::uint8_t> *bytes, std::size_t offset,
              std::uint16_t value) {
  CHECK(bytes != nullptr);
  CHECK(offset + 2U <= bytes->size());
  if (bytes == nullptr || offset + 2U > bytes->size()) {
    return;
  }
  const std::uint32_t widened = static_cast<std::uint32_t>(value);
  (*bytes)[offset] = static_cast<std::uint8_t>((widened >> 8U) & 0xffU);
  (*bytes)[offset + 1U] = static_cast<std::uint8_t>(widened & 0xffU);
}

void StoreU32(std::vector<std::uint8_t> *bytes, std::size_t offset,
              std::uint32_t value) {
  CHECK(bytes != nullptr);
  CHECK(offset + 4U <= bytes->size());
  if (bytes == nullptr || offset + 4U > bytes->size()) {
    return;
  }
  (*bytes)[offset] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
  (*bytes)[offset + 1U] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  (*bytes)[offset + 2U] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  (*bytes)[offset + 3U] = static_cast<std::uint8_t>(value & 0xffU);
}

std::uint16_t LoadU16(const std::vector<std::uint8_t> &bytes,
                      std::size_t offset) {
  CHECK(offset + 2U <= bytes.size());
  if (offset + 2U > bytes.size()) {
    return 0U;
  }
  return static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(bytes[offset]) << 8U) |
      static_cast<std::uint16_t>(bytes[offset + 1U]));
}

void RepairCrc(std::vector<std::uint8_t> *bytes) {
  CHECK(bytes != nullptr);
  CHECK(bytes != nullptr && bytes->size() >= v2::kIntegrityBytes);
  if (bytes == nullptr || bytes->size() < v2::kIntegrityBytes) {
    return;
  }
  const std::size_t offset = bytes->size() - v2::kIntegrityBytes;
  StoreU32(bytes, offset, v2::crc32c(bytes->data(), offset));
}

std::uint64_t Capabilities(v2::PeerRole role) {
  switch (role) {
  case v2::PeerRole::kGround:
    return v2::kGroundRequiredCapabilities;
  case v2::PeerRole::kVehicle:
    return v2::kVehicleRequiredCapabilities;
  case v2::PeerRole::kUnspecified:
    return 0U;
  }
  return 0U;
}

v2::PeerRole OppositeRole(v2::PeerRole role) {
  return role == v2::PeerRole::kGround ? v2::PeerRole::kVehicle
                                       : v2::PeerRole::kGround;
}

v2::FrameHeader BaseHeader(v2::Channel channel, v2::MessageKind kind,
                           std::uint64_t sequence, std::uint64_t monotonic_ns,
                           std::uint64_t session_epoch,
                           const std::string &boot_id,
                           v2::PeerRole role = v2::PeerRole::kVehicle) {
  v2::FrameHeader header;
  header.channel = channel;
  header.kind = kind;
  header.sequence = sequence;
  header.monotonic_ns = monotonic_ns;
  header.source_timestamp_ns = 0U;
  header.session_epoch = session_epoch;
  header.capabilities = Capabilities(role);
  header.slot_id = "scout-01";
  header.asset_id = "asset-a";
  header.robot_kind = "scout";
  header.run_id = "run-42";
  header.boot_id = boot_id;
  header.build_id = "build-9";
  return header;
}

v2::Frame MakeHelloFrame(std::uint64_t sequence = 0x0102030405060708ULL,
                         std::uint64_t monotonic_ns = 0x1112131415161718ULL,
                         std::uint64_t session_epoch = 0x2122232425262728ULL,
                         const std::string &boot_id = "boot-a") {
  v2::Frame frame;
  frame.header = BaseHeader(v2::Channel::kManagement, v2::MessageKind::kHello,
                            sequence, monotonic_ns, session_epoch, boot_id);
  frame.header.schema = v2::kHelloSchema;
  CHECK(v2::encodeHelloPayload(v2::Hello{}, &frame.payload).ok());
  return frame;
}

v2::Frame MakeHeartbeatFrame(std::uint64_t sequence, std::uint64_t monotonic_ns,
                             std::uint64_t session_epoch,
                             const std::string &boot_id = "boot-a",
                             v2::Channel channel = v2::Channel::kManagement,
                             v2::PeerRole role = v2::PeerRole::kVehicle) {
  v2::Frame frame;
  frame.header = BaseHeader(channel, v2::MessageKind::kHeartbeat, sequence,
                            monotonic_ns, session_epoch, boot_id, role);
  frame.header.schema = v2::kHeartbeatSchema;
  CHECK(v2::encodeHeartbeatPayload(v2::Heartbeat{sequence - 1U, 2U},
                                   &frame.payload)
            .ok());
  return frame;
}

v2::ZeroStop MakeStop() {
  v2::ZeroStop stop;
  stop.command_id = "stop-0001";
  stop.deadline_monotonic_ns = 0x4142434445464748ULL;
  return stop;
}

v2::Frame MakeStopFrame() {
  v2::Frame frame;
  frame.header =
      BaseHeader(v2::Channel::kControl, v2::MessageKind::kZeroStop,
                 0x090a0b0c0d0e0f10ULL, 0x1112131415161728ULL,
                 0x2122232425262728ULL, "boot-a", v2::PeerRole::kGround);
  frame.header.schema = v2::kZeroStopSchema;
  CHECK(v2::encodeZeroStopPayload(MakeStop(), &frame.payload).ok());
  return frame;
}

v2::Frame MakeReceiptFrame(std::uint64_t sequence, std::uint64_t monotonic_ns,
                           std::uint64_t session_epoch,
                           const v2::StopReceipt &receipt,
                           const std::string &boot_id = "boot-a") {
  v2::Frame frame;
  frame.header =
      BaseHeader(v2::Channel::kTelemetry, v2::MessageKind::kStopReceipt,
                 sequence, monotonic_ns, session_epoch, boot_id);
  frame.header.schema = v2::kStopReceiptSchema;
  CHECK(v2::encodeStopReceiptPayload(receipt, &frame.payload).ok());
  return frame;
}

v2::ExpectedPeer Expected(std::uint64_t epoch = 0x2122232425262728ULL,
                          v2::PeerRole peer_role = v2::PeerRole::kVehicle) {
  v2::ExpectedPeer expected;
  expected.slot_id = "scout-01";
  expected.asset_id = "asset-a";
  expected.robot_kind = "scout";
  expected.run_id = "run-42";
  expected.session_epoch = epoch;
  expected.build_id = "build-9";
  expected.local_role = OppositeRole(peer_role);
  expected.local_capabilities = Capabilities(expected.local_role);
  expected.peer_role = peer_role;
  expected.capabilities = Capabilities(peer_role);
  return expected;
}

v2::Frame HelloOn(v2::Channel channel, std::uint64_t sequence,
                  std::uint64_t monotonic_ns, std::uint64_t session_epoch,
                  const std::string &boot_id = "boot-a",
                  v2::PeerRole role = v2::PeerRole::kVehicle) {
  v2::Frame hello =
      MakeHelloFrame(sequence, monotonic_ns, session_epoch, boot_id);
  hello.header.channel = channel;
  hello.header.capabilities = Capabilities(role);
  return hello;
}

v2::MonotonicTimebase Timebase(std::uint64_t sender_anchor_ns,
                               std::uint64_t receiver_anchor_ns) {
  v2::MonotonicTimebase timebase;
  timebase.sender_anchor_ns = sender_anchor_ns;
  timebase.receiver_anchor_ns = receiver_anchor_ns;
  timebase.maximum_error_ns = 1000000ULL;
  timebase.valid_until_receiver_ns = receiver_anchor_ns + 5000000000ULL;
  return timebase;
}

void SetHeartbeatAck(v2::Frame *frame, std::uint64_t acknowledged_sequence) {
  CHECK(frame != nullptr);
  if (frame == nullptr) {
    return;
  }
  CHECK(frame->header.kind == v2::MessageKind::kHeartbeat);
  CHECK(v2::encodeHeartbeatPayload(v2::Heartbeat{acknowledged_sequence, 2U},
                                   &frame->payload)
            .ok());
}

v2::SendWindowConfig
WindowConfig(v2::Channel channel, std::uint64_t epoch,
             v2::PeerRole local_role = v2::PeerRole::kGround,
             const std::string &local_boot_id = "ground-boot",
             const std::string &peer_boot_id = "vehicle-boot") {
  v2::SendWindowConfig config;
  config.channel = channel;
  config.local_role = local_role;
  config.local_capabilities = Capabilities(local_role);
  config.peer_role = OppositeRole(local_role);
  config.peer_capabilities = Capabilities(config.peer_role);
  config.session_epoch = epoch;
  config.local_boot_id = local_boot_id;
  config.peer_slot_id = "scout-01";
  config.peer_asset_id = "asset-a";
  config.peer_robot_kind = "scout";
  config.peer_run_id = "run-42";
  config.peer_boot_id = peer_boot_id;
  config.peer_build_id = "build-9";
  return config;
}

void TestCrcAndGoldenRoundTrips() {
  const std::string crc_input = "123456789";
  CHECK(v2::crc32c(reinterpret_cast<const std::uint8_t *>(crc_input.data()),
                   crc_input.size()) == 0xe3069283U);

  v2::Frame heartbeat = MakeHeartbeatFrame(
      0x0203040506070809ULL, 0x1112131415161720ULL, 0x2122232425262728ULL);
  SetHeartbeatAck(&heartbeat, 0x0102030405060708ULL);
  const v2::Frame frames[] = {MakeHelloFrame(), heartbeat, MakeStopFrame()};
  for (const v2::Frame &frame : frames) {
    std::vector<std::uint8_t> wire;
    CHECK(v2::encodeFrame(frame, &wire).ok());
    CHECK(wire.size() >= v2::kFixedHeaderBytes + v2::kIntegrityBytes);
    CHECK(wire[0] == 'X' && wire[1] == 'S' && wire[2] == 'B' && wire[3] == '2');
    CHECK(wire[4] == 2U);
    CHECK(wire[16] == static_cast<std::uint8_t>(frame.header.sequence >> 56U));
    v2::Frame decoded;
    CHECK(v2::decodeFrame(wire, &decoded).ok());
    CHECK(decoded.header.slot_id == frame.header.slot_id);
    CHECK(decoded.header.asset_id == frame.header.asset_id);
    CHECK(decoded.header.robot_kind == frame.header.robot_kind);
    CHECK(decoded.header.run_id == frame.header.run_id);
    CHECK(decoded.header.boot_id == frame.header.boot_id);
    CHECK(decoded.header.build_id == frame.header.build_id);
    CHECK(decoded.header.sequence == frame.header.sequence);
    CHECK(decoded.header.session_epoch == frame.header.session_epoch);
    CHECK(decoded.payload == frame.payload);
  }
}

void TestEnvelopeNegativeCases() {
  std::vector<std::uint8_t> canonical;
  CHECK(v2::encodeFrame(MakeHelloFrame(), &canonical).ok());
  v2::Frame decoded;

  for (std::size_t length = 0U; length < canonical.size(); ++length) {
    std::vector<std::uint8_t> truncated(
        canonical.begin(),
        canonical.begin() + static_cast<std::ptrdiff_t>(length));
    CHECK(!v2::decodeFrame(truncated, &decoded).ok());
  }

  std::vector<std::uint8_t> trailing = canonical;
  trailing.push_back(0U);
  CHECK_ERROR(v2::decodeFrame(trailing, &decoded),
              v2::ErrorCode::kInvalidLength);

  std::vector<std::uint8_t> bad_magic = canonical;
  bad_magic[0] ^= 1U;
  CHECK_ERROR(v2::decodeFrame(bad_magic, &decoded), v2::ErrorCode::kBadMagic);

  std::vector<std::uint8_t> bad_version = canonical;
  bad_version[4] = 3U;
  CHECK_ERROR(v2::decodeFrame(bad_version, &decoded),
              v2::ErrorCode::kUnsupportedVersion);

  std::vector<std::uint8_t> corrupt = canonical;
  corrupt[corrupt.size() - v2::kIntegrityBytes - 1U] ^= 0x80U;
  CHECK_ERROR(v2::decodeFrame(corrupt, &decoded),
              v2::ErrorCode::kIntegrityMismatch);

  std::vector<std::uint8_t> reserved = canonical;
  reserved[10] = 1U;
  RepairCrc(&reserved);
  CHECK_ERROR(v2::decodeFrame(reserved, &decoded),
              v2::ErrorCode::kInvalidFlags);

  std::vector<std::uint8_t> short_header = canonical;
  StoreU16(&short_header, 8U,
           static_cast<std::uint16_t>(v2::kFixedHeaderBytes - 1U));
  RepairCrc(&short_header);
  CHECK_ERROR(v2::decodeFrame(short_header, &decoded),
              v2::ErrorCode::kInvalidLength);

  std::vector<std::uint8_t> wrong_string_sum = canonical;
  StoreU16(&wrong_string_sum, 56U, 1U);
  RepairCrc(&wrong_string_sum);
  CHECK_ERROR(v2::decodeFrame(wrong_string_sum, &decoded),
              v2::ErrorCode::kInvalidLength);

  v2::Frame oversized = MakeHelloFrame();
  oversized.payload.assign(v2::kMaxPayloadBytes + 1U, 0U);
  std::vector<std::uint8_t> ignored;
  CHECK_ERROR(v2::encodeFrame(oversized, &ignored),
              v2::ErrorCode::kLimitExceeded);

  v2::Frame maximum_ros;
  maximum_ros.header =
      BaseHeader(v2::Channel::kTelemetry, v2::MessageKind::kRosMessage, 10U,
                 1000U, 7U, "boot-max");
  maximum_ros.header.source_timestamp_ns = 2000U;
  maximum_ros.header.ros_datatype = "sensor_msgs/Imu";
  maximum_ros.header.ros_md5.assign(v2::kMaxRosMd5Bytes, '0');
  maximum_ros.header.schema = "ros1.serialized.v2";
  maximum_ros.payload.assign(v2::kMaxPayloadBytes, 0U);
  CHECK(v2::encodeFrame(maximum_ros, &ignored).ok());
  CHECK(ignored.size() <= v2::kMaxFrameBytes);
  CHECK(v2::decodeFrame(ignored, &decoded).ok());
  maximum_ros.payload.push_back(0U);
  CHECK_ERROR(v2::encodeFrame(maximum_ros, &ignored),
              v2::ErrorCode::kLimitExceeded);

  v2::Frame maximum_slot = MakeHelloFrame();
  maximum_slot.header.slot_id.assign(v2::kMaxSlotIdBytes, 's');
  CHECK(v2::encodeFrame(maximum_slot, &ignored).ok());

  v2::Frame bad_identity = MakeHelloFrame();
  bad_identity.header.slot_id.assign(v2::kMaxSlotIdBytes + 1U, 'a');
  CHECK_ERROR(v2::encodeFrame(bad_identity, &ignored),
              v2::ErrorCode::kLimitExceeded);
  bad_identity = MakeHelloFrame();
  bad_identity.header.asset_id = "asset with space";
  CHECK_ERROR(v2::encodeFrame(bad_identity, &ignored),
              v2::ErrorCode::kInvalidIdentity);

  v2::Frame bad_ros = maximum_ros;
  bad_ros.payload.assign(1U, 0U);
  bad_ros.header.ros_md5.assign(v2::kMaxRosMd5Bytes, 'A');
  CHECK_ERROR(v2::encodeFrame(bad_ros, &ignored),
              v2::ErrorCode::kInvalidMetadata);
}

void TestDeterministicMutationCorpus() {
  std::vector<std::uint8_t> canonical;
  CHECK(v2::encodeFrame(MakeStopFrame(), &canonical).ok());
  v2::Frame decoded;

  for (std::size_t index = 0U; index < canonical.size(); ++index) {
    std::vector<std::uint8_t> mutated = canonical;
    mutated[index] = static_cast<std::uint8_t>(
        mutated[index] ^ static_cast<std::uint8_t>(1U << (index % 8U)));
    CHECK(!v2::decodeFrame(mutated, &decoded).ok());
  }

  std::uint32_t state = 0x5eed1234U;
  for (std::size_t iteration = 0U; iteration < 512U; ++iteration) {
    state = state * 1664525U + 1013904223U;
    const std::size_t length = state % (canonical.size() + 33U);
    std::vector<std::uint8_t> corpus(length);
    for (std::uint8_t &byte : corpus) {
      state = state * 1664525U + 1013904223U;
      byte = static_cast<std::uint8_t>(state >> 24U);
    }
    CHECK(!v2::decodeFrame(corpus, &decoded).ok());
  }

  // The corpus above mostly proves the integrity boundary. These mutations
  // deliberately repair CRC32C so the semantic decoder, not just CRC, must
  // reject a well-integrity-framed violation.
  std::vector<std::uint8_t> semantic = canonical;
  semantic[5] = 0xffU;
  RepairCrc(&semantic);
  CHECK_ERROR(v2::decodeFrame(semantic, &decoded), v2::ErrorCode::kInvalidEnum);

  semantic = canonical;
  semantic[6] = 0xffU;
  RepairCrc(&semantic);
  CHECK_ERROR(v2::decodeFrame(semantic, &decoded), v2::ErrorCode::kInvalidEnum);

  semantic = canonical;
  semantic[5] = static_cast<std::uint8_t>(v2::Channel::kManagement);
  RepairCrc(&semantic);
  CHECK_ERROR(v2::decodeFrame(semantic, &decoded),
              v2::ErrorCode::kInvalidMetadata);

  std::vector<std::uint8_t> hello;
  CHECK(v2::encodeFrame(MakeHelloFrame(), &hello).ok());
  const std::size_t hello_payload_offset = LoadU16(hello, 8U);
  CHECK(hello_payload_offset + 8U + v2::kIntegrityBytes == hello.size());
  hello[hello_payload_offset] = 1U;
  RepairCrc(&hello);
  CHECK_ERROR(v2::decodeFrame(hello, &decoded), v2::ErrorCode::kInvalidPayload);

  hello.clear();
  CHECK(v2::encodeFrame(MakeHelloFrame(), &hello).ok());
  std::size_t schema_offset = v2::kFixedHeaderBytes;
  for (std::size_t index = 0U; index < 8U; ++index) {
    schema_offset += LoadU16(hello, 56U + index * 2U);
  }
  CHECK(schema_offset < LoadU16(hello, 8U));
  hello[schema_offset] = hello[schema_offset] == 'x' ? 'y' : 'x';
  RepairCrc(&hello);
  CHECK_ERROR(v2::decodeFrame(hello, &decoded),
              v2::ErrorCode::kInvalidMetadata);
}

void TestZeroOnlyStopAndReceipts() {
  v2::ZeroStop stop = MakeStop();
  std::vector<std::uint8_t> payload;
  CHECK(v2::encodeZeroStopPayload(stop, &payload).ok());
  v2::ZeroStop decoded;
  CHECK(v2::decodeZeroStopPayload(payload, &decoded).ok());
  CHECK(decoded.command_id == stop.command_id);
  CHECK(decoded.deadline_monotonic_ns == stop.deadline_monotonic_ns);
  for (double axis : decoded.axes) {
    CHECK(axis == 0.0 && !std::signbit(axis));
  }

  for (std::size_t axis = 0U; axis < stop.axes.size(); ++axis) {
    stop = MakeStop();
    stop.axes[axis] = 1.0;
    CHECK_ERROR(v2::encodeZeroStopPayload(stop, &payload),
                v2::ErrorCode::kUnsafeNonZero);
  }
  stop = MakeStop();
  stop.axes[0] = std::numeric_limits<double>::denorm_min();
  CHECK_ERROR(v2::encodeZeroStopPayload(stop, &payload),
              v2::ErrorCode::kUnsafeNonZero);
  stop = MakeStop();
  stop.axes[1] = std::numeric_limits<double>::quiet_NaN();
  CHECK_ERROR(v2::encodeZeroStopPayload(stop, &payload),
              v2::ErrorCode::kUnsafeNonZero);
  stop = MakeStop();
  stop.axes[2] = std::numeric_limits<double>::infinity();
  CHECK_ERROR(v2::encodeZeroStopPayload(stop, &payload),
              v2::ErrorCode::kUnsafeNonZero);
  stop = MakeStop();
  stop.axes[5] = -0.0;
  CHECK_ERROR(v2::encodeZeroStopPayload(stop, &payload),
              v2::ErrorCode::kUnsafeNonZero);

  stop = MakeStop();
  CHECK(v2::validateStopDeadline(stop, stop.deadline_monotonic_ns).ok());
  CHECK_ERROR(v2::validateStopDeadline(stop, stop.deadline_monotonic_ns + 1U),
              v2::ErrorCode::kDeadlineExpired);
  CHECK_ERROR(v2::validateStopDeadline(
                  stop, stop.deadline_monotonic_ns -
                            v2::kMaximumStopDeadlineHorizonNs - 1U),
              v2::ErrorCode::kDeadlineTooFar);
  CHECK_ERROR(v2::validateStopDeadline(stop, 0U),
              v2::ErrorCode::kInvalidMetadata);

  v2::Heartbeat invalid_heartbeat{0U, 4U};
  CHECK_ERROR(v2::encodeHeartbeatPayload(invalid_heartbeat, &payload),
              v2::ErrorCode::kInvalidPayload);

  v2::Hello invalid_hello;
  invalid_hello.heartbeat_timeout_ms = 0U;
  CHECK_ERROR(v2::encodeHelloPayload(invalid_hello, &payload),
              v2::ErrorCode::kInvalidPayload);

  v2::StopReceipt receipt{"stop-0001", v2::ReceiptPhase::kAccepted,
                          v2::ReceiptStatus::kOk, 1234U, "accepted"};
  CHECK(v2::encodeStopReceiptPayload(receipt, &payload).ok());
  v2::StopReceipt decoded_receipt;
  CHECK(v2::decodeStopReceiptPayload(payload, &decoded_receipt).ok());
  CHECK(decoded_receipt.phase == v2::ReceiptPhase::kAccepted);
  receipt.phase = v2::ReceiptPhase::kApplied;
  receipt.observed_monotonic_ns = 1240U;
  receipt.detail = "applied";
  CHECK(v2::encodeStopReceiptPayload(receipt, &payload).ok());
  CHECK(v2::decodeStopReceiptPayload(payload, &decoded_receipt).ok());
  CHECK(decoded_receipt.phase == v2::ReceiptPhase::kApplied);
}

void TestCapabilityRoleContract() {
  CHECK(v2::validateRoleCapabilities(v2::PeerRole::kGround,
                                     v2::kGroundRequiredCapabilities)
            .ok());
  CHECK(v2::validateRoleCapabilities(v2::PeerRole::kVehicle,
                                     v2::kVehicleRequiredCapabilities)
            .ok());
  CHECK(v2::validatePeerCompatibility(
            v2::PeerRole::kGround, v2::kGroundRequiredCapabilities,
            v2::PeerRole::kVehicle, v2::kVehicleRequiredCapabilities)
            .ok());
  CHECK(v2::validatePeerCompatibility(
            v2::PeerRole::kVehicle, v2::kVehicleRequiredCapabilities,
            v2::PeerRole::kGround, v2::kGroundRequiredCapabilities)
            .ok());

  CHECK_ERROR(v2::validateRoleCapabilities(v2::PeerRole::kGround, 0U),
              v2::ErrorCode::kCapabilityMismatch);
  CHECK_ERROR(v2::validateRoleCapabilities(v2::PeerRole::kGround,
                                           v2::kGroundRequiredCapabilities |
                                               (1ULL << 63U)),
              v2::ErrorCode::kCapabilityMismatch);
  CHECK_ERROR(v2::validateRoleCapabilities(v2::PeerRole::kGround,
                                           v2::kGroundRequiredCapabilities &
                                               ~v2::kCapabilityPerChannelAck),
              v2::ErrorCode::kCapabilityMismatch);
  CHECK_ERROR(v2::validateRoleCapabilities(v2::PeerRole::kGround,
                                           v2::kGroundRequiredCapabilities |
                                               v2::kCapabilityRoleVehicle),
              v2::ErrorCode::kCapabilityMismatch);
  CHECK_ERROR(v2::validateRoleCapabilities(v2::PeerRole::kVehicle,
                                           v2::kGroundRequiredCapabilities),
              v2::ErrorCode::kCapabilityMismatch);
  CHECK_ERROR(v2::validatePeerCompatibility(
                  v2::PeerRole::kGround, v2::kGroundRequiredCapabilities,
                  v2::PeerRole::kGround, v2::kGroundRequiredCapabilities),
              v2::ErrorCode::kCapabilityMismatch);

  std::vector<std::uint8_t> ignored;
  v2::Frame invalid = MakeHelloFrame();
  invalid.header.capabilities = 0U;
  CHECK_ERROR(v2::encodeFrame(invalid, &ignored),
              v2::ErrorCode::kCapabilityMismatch);
  invalid = MakeHelloFrame();
  invalid.header.capabilities |= (1ULL << 63U);
  CHECK_ERROR(v2::encodeFrame(invalid, &ignored),
              v2::ErrorCode::kCapabilityMismatch);
  invalid = MakeHelloFrame();
  invalid.header.capabilities &= ~v2::kCapabilityPerChannelAck;
  CHECK_ERROR(v2::encodeFrame(invalid, &ignored),
              v2::ErrorCode::kCapabilityMismatch);

  invalid = MakeStopFrame();
  invalid.header.capabilities = v2::kVehicleRequiredCapabilities;
  CHECK_ERROR(v2::encodeFrame(invalid, &ignored),
              v2::ErrorCode::kCapabilityMismatch);
  v2::StopReceipt receipt{"capability-role", v2::ReceiptPhase::kAccepted,
                          v2::ReceiptStatus::kOk, 1U, "accepted"};
  invalid = MakeReceiptFrame(1U, 1U, 1U, receipt);
  invalid.header.capabilities = v2::kGroundRequiredCapabilities;
  CHECK_ERROR(v2::encodeFrame(invalid, &ignored),
              v2::ErrorCode::kCapabilityMismatch);

  v2::ExpectedPeer downgraded = Expected(17U);
  downgraded.local_capabilities &= ~v2::kCapabilityPerChannelAck;
  v2::ReceiveGuard guard(downgraded, v2::Channel::kManagement,
                         Timebase(1000000000ULL, 9000000000ULL));
  v2::Admission admission;
  CHECK_ERROR(guard.accept(MakeHelloFrame(1U, 1000000000ULL, 17U),
                           9000000000ULL, &admission),
              v2::ErrorCode::kCapabilityMismatch);
  CHECK(!admission.valid());

  v2::ReceiveGuard vehicle_control(Expected(17U, v2::PeerRole::kVehicle),
                                   v2::Channel::kControl,
                                   Timebase(1000000000ULL, 9000000000ULL));
  CHECK(vehicle_control
            .accept(HelloOn(v2::Channel::kControl, 1U, 1000000000ULL, 17U),
                    9000000000ULL, &admission)
            .ok());
  v2::StopWatchdog onboard_watchdog(17U);
  v2::SafetyAction action = v2::SafetyAction::kNone;
  CHECK_ERROR(onboard_watchdog.onHello(admission, 9000000000ULL, &action),
              v2::ErrorCode::kInvalidMetadata);
  CHECK(onboard_watchdog.state() == v2::LinkState::kAwaitHello);
}

void TestPerChannelSendWindowAndAckReadiness() {
  constexpr std::uint64_t kEpoch = 47U;
  constexpr std::uint64_t kVehicleBase = 2000000000ULL;
  constexpr std::uint64_t kGroundBase = 10000000000ULL;

  v2::ExpectedPeer vehicle = Expected(kEpoch, v2::PeerRole::kVehicle);
  vehicle.boot_id = "vehicle-boot";
  v2::ReceiveGuard guard(vehicle, v2::Channel::kManagement,
                         Timebase(kVehicleBase, kGroundBase));
  v2::SendWindow window(WindowConfig(v2::Channel::kManagement, kEpoch));

  v2::Frame local_heartbeat =
      MakeHeartbeatFrame(9U, kGroundBase, kEpoch, "ground-boot",
                         v2::Channel::kManagement, v2::PeerRole::kGround);
  CHECK_ERROR(window.recordSent(local_heartbeat),
              v2::ErrorCode::kHandshakeRequired);

  const v2::Frame local_hello =
      HelloOn(v2::Channel::kManagement, 10U, kGroundBase, kEpoch, "ground-boot",
              v2::PeerRole::kGround);
  CHECK(window.recordSent(local_hello).ok());
  CHECK(window.outboundEstablished());
  CHECK(window.lastSentSequence() == 10U);
  CHECK(window.outstandingCount() == 1U);
  CHECK_ERROR(window.recordSent(local_hello), v2::ErrorCode::kInvalidState);
  const v2::Frame local_gap =
      MakeHeartbeatFrame(12U, kGroundBase + 1U, kEpoch, "ground-boot",
                         v2::Channel::kManagement, v2::PeerRole::kGround);
  CHECK_ERROR(window.recordSent(local_gap), v2::ErrorCode::kSequenceGap);

  v2::Admission hello_admission;
  const v2::Frame peer_hello =
      HelloOn(v2::Channel::kManagement, 1U, kVehicleBase, kEpoch,
              "vehicle-boot", v2::PeerRole::kVehicle);
  CHECK(guard.accept(peer_hello, kGroundBase, &hello_admission).ok());
  CHECK(window.bindPeerHello(hello_admission).ok());
  CHECK(window.peerHelloBound());
  CHECK_ERROR(window.bindPeerHello(hello_admission),
              v2::ErrorCode::kInvalidState);

  v2::Frame peer_gap =
      MakeHeartbeatFrame(3U, kVehicleBase + 50000000ULL, kEpoch, "vehicle-boot",
                         v2::Channel::kManagement, v2::PeerRole::kVehicle);
  SetHeartbeatAck(&peer_gap, 10U);
  v2::Admission heartbeat_admission;
  CHECK_ERROR(
      guard.accept(peer_gap, kGroundBase + 50000000ULL, &heartbeat_admission),
      v2::ErrorCode::kSequenceGap);

  v2::Frame peer_heartbeat = MakeHeartbeatFrame(
      2U, kVehicleBase + 100000000ULL, kEpoch, "vehicle-boot",
      v2::Channel::kManagement, v2::PeerRole::kVehicle);
  SetHeartbeatAck(&peer_heartbeat, 9U);
  CHECK(guard
            .accept(peer_heartbeat, kGroundBase + 100000000ULL,
                    &heartbeat_admission)
            .ok());
  v2::Heartbeat heartbeat{9U, 2U};
  v2::AckDisposition disposition = v2::AckDisposition::kDuplicate;
  CHECK_ERROR(
      window.observeHeartbeat(heartbeat, heartbeat_admission, &disposition),
      v2::ErrorCode::kAckOutOfWindow);
  CHECK(window.acknowledgedFrontier() == 0U);

  peer_heartbeat = MakeHeartbeatFrame(3U, kVehicleBase + 200000000ULL, kEpoch,
                                      "vehicle-boot", v2::Channel::kManagement,
                                      v2::PeerRole::kVehicle);
  SetHeartbeatAck(&peer_heartbeat, 10U);
  CHECK(guard
            .accept(peer_heartbeat, kGroundBase + 200000000ULL,
                    &heartbeat_admission)
            .ok());
  heartbeat.last_received_sequence = 10U;
  CHECK(window.observeHeartbeat(heartbeat, heartbeat_admission, &disposition)
            .ok());
  CHECK(disposition == v2::AckDisposition::kAdvanced);
  CHECK(window.acknowledgedFrontier() == 10U);
  CHECK(window.outstandingCount() == 0U);

  local_heartbeat =
      MakeHeartbeatFrame(11U, kGroundBase + 210000000ULL, kEpoch, "ground-boot",
                         v2::Channel::kManagement, v2::PeerRole::kGround);
  CHECK(window.recordSent(local_heartbeat).ok());

  peer_heartbeat = MakeHeartbeatFrame(4U, kVehicleBase + 300000000ULL, kEpoch,
                                      "vehicle-boot", v2::Channel::kManagement,
                                      v2::PeerRole::kVehicle);
  SetHeartbeatAck(&peer_heartbeat, 10U);
  CHECK(guard
            .accept(peer_heartbeat, kGroundBase + 300000000ULL,
                    &heartbeat_admission)
            .ok());
  CHECK(window.observeHeartbeat(heartbeat, heartbeat_admission, &disposition)
            .ok());
  CHECK(disposition == v2::AckDisposition::kDuplicate);
  CHECK(window.acknowledgedFrontier() == 10U);

  peer_heartbeat = MakeHeartbeatFrame(5U, kVehicleBase + 400000000ULL, kEpoch,
                                      "vehicle-boot", v2::Channel::kManagement,
                                      v2::PeerRole::kVehicle);
  SetHeartbeatAck(&peer_heartbeat, 12U);
  CHECK(guard
            .accept(peer_heartbeat, kGroundBase + 400000000ULL,
                    &heartbeat_admission)
            .ok());
  heartbeat.last_received_sequence = 12U;
  CHECK_ERROR(
      window.observeHeartbeat(heartbeat, heartbeat_admission, &disposition),
      v2::ErrorCode::kAckFuture);
  CHECK(window.acknowledgedFrontier() == 10U);

  peer_heartbeat = MakeHeartbeatFrame(6U, kVehicleBase + 500000000ULL, kEpoch,
                                      "vehicle-boot", v2::Channel::kManagement,
                                      v2::PeerRole::kVehicle);
  SetHeartbeatAck(&peer_heartbeat, 0U);
  CHECK(guard
            .accept(peer_heartbeat, kGroundBase + 500000000ULL,
                    &heartbeat_admission)
            .ok());
  heartbeat.last_received_sequence = 0U;
  CHECK_ERROR(
      window.observeHeartbeat(heartbeat, heartbeat_admission, &disposition),
      v2::ErrorCode::kAckOutOfWindow);
  CHECK(window.acknowledgedFrontier() == 10U);

  peer_heartbeat = MakeHeartbeatFrame(7U, kVehicleBase + 600000000ULL, kEpoch,
                                      "vehicle-boot", v2::Channel::kManagement,
                                      v2::PeerRole::kVehicle);
  SetHeartbeatAck(&peer_heartbeat, 11U);
  CHECK(guard
            .accept(peer_heartbeat, kGroundBase + 600000000ULL,
                    &heartbeat_admission)
            .ok());
  heartbeat.last_received_sequence = 11U;
  CHECK(window.observeHeartbeat(heartbeat, heartbeat_admission, &disposition)
            .ok());
  CHECK(disposition == v2::AckDisposition::kAdvanced);
  CHECK(window.acknowledgedFrontier() == 11U);

  peer_heartbeat = MakeHeartbeatFrame(8U, kVehicleBase + 700000000ULL, kEpoch,
                                      "vehicle-boot", v2::Channel::kManagement,
                                      v2::PeerRole::kVehicle);
  SetHeartbeatAck(&peer_heartbeat, 10U);
  CHECK(guard
            .accept(peer_heartbeat, kGroundBase + 700000000ULL,
                    &heartbeat_admission)
            .ok());
  heartbeat.last_received_sequence = 10U;
  CHECK_ERROR(
      window.observeHeartbeat(heartbeat, heartbeat_admission, &disposition),
      v2::ErrorCode::kAckRegression);
  CHECK(window.acknowledgedFrontier() == 11U);

  v2::Heartbeat altered{11U, 2U};
  CHECK_ERROR(
      window.observeHeartbeat(altered, heartbeat_admission, &disposition),
      v2::ErrorCode::kInvalidMetadata);

  guard.reset(vehicle, Timebase(kVehicleBase, kGroundBase));
  const v2::Frame reset_peer_hello =
      HelloOn(v2::Channel::kManagement, 1U, kVehicleBase + 710000000ULL, kEpoch,
              "vehicle-boot", v2::PeerRole::kVehicle);
  CHECK(guard
            .accept(reset_peer_hello, kGroundBase + 710000000ULL,
                    &hello_admission)
            .ok());
  v2::Frame reset_peer_heartbeat = MakeHeartbeatFrame(
      2U, kVehicleBase + 720000000ULL, kEpoch, "vehicle-boot",
      v2::Channel::kManagement, v2::PeerRole::kVehicle);
  SetHeartbeatAck(&reset_peer_heartbeat, 11U);
  CHECK(guard
            .accept(reset_peer_heartbeat, kGroundBase + 720000000ULL,
                    &heartbeat_admission)
            .ok());
  heartbeat.last_received_sequence = 11U;
  CHECK_ERROR(
      window.observeHeartbeat(heartbeat, heartbeat_admission, &disposition),
      v2::ErrorCode::kInvalidState);

  v2::SendWindowConfig bounded_config =
      WindowConfig(v2::Channel::kControl, kEpoch);
  bounded_config.capacity = 2U;
  v2::SendWindow bounded(bounded_config);
  CHECK(bounded
            .recordSent(HelloOn(v2::Channel::kControl, 1U, kGroundBase, kEpoch,
                                "ground-boot", v2::PeerRole::kGround))
            .ok());
  CHECK(bounded
            .recordSent(MakeHeartbeatFrame(2U, kGroundBase + 1U, kEpoch,
                                           "ground-boot", v2::Channel::kControl,
                                           v2::PeerRole::kGround))
            .ok());
  CHECK_ERROR(bounded.recordSent(MakeHeartbeatFrame(
                  3U, kGroundBase + 2U, kEpoch, "ground-boot",
                  v2::Channel::kControl, v2::PeerRole::kGround)),
              v2::ErrorCode::kSendWindowFull);

  v2::SendWindowConfig next_generation =
      WindowConfig(v2::Channel::kManagement, kEpoch + 1U, v2::PeerRole::kGround,
                   "ground-boot-2", "vehicle-boot-2");
  window.reset(next_generation);
  CHECK(!window.outboundEstablished());
  CHECK(window.acknowledgedFrontier() == 0U);
  CHECK_ERROR(window.recordSent(local_hello), v2::ErrorCode::kEpochMismatch);
  const v2::Frame next_local_hello =
      HelloOn(v2::Channel::kManagement, 1U, kGroundBase + 700000000ULL,
              kEpoch + 1U, "ground-boot-2", v2::PeerRole::kGround);
  CHECK(window.recordSent(next_local_hello).ok());
  CHECK_ERROR(window.bindPeerHello(hello_admission),
              v2::ErrorCode::kEpochMismatch);

  v2::ReceiveGuard management(vehicle, v2::Channel::kManagement,
                              Timebase(kVehicleBase, kGroundBase));
  v2::ReceiveGuard control(vehicle, v2::Channel::kControl,
                           Timebase(kVehicleBase, kGroundBase));
  v2::ReceiveGuard telemetry(vehicle, v2::Channel::kTelemetry,
                             Timebase(kVehicleBase, kGroundBase));
  v2::SendWindow management_send(
      WindowConfig(v2::Channel::kManagement, kEpoch));
  v2::SendWindow control_send(WindowConfig(v2::Channel::kControl, kEpoch));
  v2::SendWindow telemetry_send(WindowConfig(v2::Channel::kTelemetry, kEpoch));
  v2::SendWindow unacknowledged_telemetry(
      WindowConfig(v2::Channel::kTelemetry, kEpoch));

  v2::ReceiveGuard *guards[] = {&management, &control, &telemetry};
  v2::SendWindow *windows[] = {&management_send, &control_send,
                               &telemetry_send};
  const v2::Channel channels[] = {
      v2::Channel::kManagement, v2::Channel::kControl, v2::Channel::kTelemetry};
  for (std::size_t index = 0U; index < 3U; ++index) {
    const v2::Frame outbound_hello =
        HelloOn(channels[index], 1U, kGroundBase, kEpoch, "ground-boot",
                v2::PeerRole::kGround);
    CHECK(windows[index]->recordSent(outbound_hello).ok());
    if (index == 2U) {
      CHECK(unacknowledged_telemetry.recordSent(outbound_hello).ok());
    }

    const v2::Frame inbound_hello =
        HelloOn(channels[index], 1U, kVehicleBase, kEpoch, "vehicle-boot",
                v2::PeerRole::kVehicle);
    v2::Admission inbound_admission;
    CHECK(guards[index]
              ->accept(inbound_hello, kGroundBase, &inbound_admission)
              .ok());
    CHECK(windows[index]->bindPeerHello(inbound_admission).ok());
    if (index == 2U) {
      CHECK(unacknowledged_telemetry.bindPeerHello(inbound_admission).ok());
    }

    v2::Frame inbound_heartbeat = MakeHeartbeatFrame(
        2U, kVehicleBase + 100000000ULL, kEpoch, "vehicle-boot",
        channels[index], v2::PeerRole::kVehicle);
    SetHeartbeatAck(&inbound_heartbeat, 1U);
    CHECK(guards[index]
              ->accept(inbound_heartbeat, kGroundBase + 100000000ULL,
                       &inbound_admission)
              .ok());
    v2::Heartbeat inbound_dto{1U, 2U};
    CHECK(windows[index]
              ->observeHeartbeat(inbound_dto, inbound_admission, &disposition)
              .ok());
  }
  CHECK(v2::requireThreeChannelReady(management, control, telemetry,
                                     management_send, control_send,
                                     telemetry_send, kGroundBase + 100000001ULL)
            .ok());
  CHECK_ERROR(v2::requireThreeChannelReady(
                  management, control, telemetry, control_send, management_send,
                  telemetry_send, kGroundBase + 100000001ULL),
              v2::ErrorCode::kChannelMismatch);
  CHECK_ERROR(v2::requireThreeChannelReady(
                  management, control, telemetry, management_send, control_send,
                  unacknowledged_telemetry, kGroundBase + 100000001ULL),
              v2::ErrorCode::kHandshakeRequired);
}

void TestReceiveGuardIdentityReplayEpochAndFreshness() {
  constexpr std::uint64_t kEpoch = 17U;
  constexpr std::uint64_t kSenderBase = 1000000000ULL;
  constexpr std::uint64_t kReceiverBase = 9000000000ULL;
  const v2::MonotonicTimebase timebase = Timebase(kSenderBase, kReceiverBase);
  v2::ReceiveGuard management(Expected(kEpoch), v2::Channel::kManagement,
                              timebase);
  v2::ReceiveGuard control(Expected(kEpoch), v2::Channel::kControl, timebase);
  v2::ReceiveGuard telemetry(Expected(kEpoch), v2::Channel::kTelemetry,
                             timebase);
  v2::Admission admission;

  v2::Frame heartbeat = MakeHeartbeatFrame(1U, kSenderBase, kEpoch);
  CHECK_ERROR(management.accept(heartbeat, kReceiverBase, &admission),
              v2::ErrorCode::kHandshakeRequired);
  CHECK(!admission.valid());

  const v2::Frame management_hello =
      HelloOn(v2::Channel::kManagement, 1U, kSenderBase, kEpoch);
  const v2::Frame control_hello =
      HelloOn(v2::Channel::kControl, 1U, kSenderBase, kEpoch);
  const v2::Frame telemetry_hello =
      HelloOn(v2::Channel::kTelemetry, 1U, kSenderBase, kEpoch);
  CHECK(management.accept(management_hello, kReceiverBase, &admission).ok());
  CHECK(admission.valid());
  CHECK(admission.channel() == v2::Channel::kManagement);
  CHECK(admission.kind() == v2::MessageKind::kHello);
  CHECK(control.accept(control_hello, kReceiverBase, &admission).ok());
  CHECK(telemetry.accept(telemetry_hello, kReceiverBase, &admission).ok());
  CHECK_ERROR(v2::requireThreeChannelFresh(management, control, telemetry,
                                           kReceiverBase + 1U),
              v2::ErrorCode::kHandshakeRequired);

  heartbeat = MakeHeartbeatFrame(2U, kSenderBase + 100000000ULL, kEpoch);
  CHECK(management.accept(heartbeat, kReceiverBase + 100000000ULL, &admission)
            .ok());
  const v2::Frame control_heartbeat = MakeHeartbeatFrame(
      2U, kSenderBase + 100000000ULL, kEpoch, "boot-a", v2::Channel::kControl);
  CHECK(control
            .accept(control_heartbeat, kReceiverBase + 100000000ULL, &admission)
            .ok());
  const v2::Frame telemetry_heartbeat =
      MakeHeartbeatFrame(2U, kSenderBase + 100000000ULL, kEpoch, "boot-a",
                         v2::Channel::kTelemetry);
  CHECK(
      telemetry
          .accept(telemetry_heartbeat, kReceiverBase + 100000000ULL, &admission)
          .ok());
  CHECK(v2::requireThreeChannelFresh(management, control, telemetry,
                                     kReceiverBase + 100000001ULL)
            .ok());
  CHECK_ERROR(
      management.accept(heartbeat, kReceiverBase + 100000001ULL, &admission),
      v2::ErrorCode::kReplay);
  CHECK_ERROR(
      control.accept(heartbeat, kReceiverBase + 100000001ULL, &admission),
      v2::ErrorCode::kChannelMismatch);
  CHECK(!admission.valid());

  v2::Frame backwards =
      MakeHeartbeatFrame(3U, kSenderBase + 50000000ULL, kEpoch);
  CHECK_ERROR(
      management.accept(backwards, kReceiverBase + 101000000ULL, &admission),
      v2::ErrorCode::kOutOfOrderTimestamp);
  CHECK(!admission.valid());

  v2::Frame stale = MakeHeartbeatFrame(3U, kSenderBase + 200000000ULL, kEpoch);
  CHECK_ERROR(
      management.accept(stale, kReceiverBase + 1000000001ULL, &admission),
      v2::ErrorCode::kStale);
  CHECK(!admission.valid());

  // Recent non-heartbeat traffic must not mask a dead channel heartbeat.
  const v2::Frame management_refresh =
      MakeHeartbeatFrame(3U, kSenderBase + 850000001ULL, kEpoch, "boot-a",
                         v2::Channel::kManagement);
  const v2::Frame control_refresh = MakeHeartbeatFrame(
      3U, kSenderBase + 850000001ULL, kEpoch, "boot-a", v2::Channel::kControl);
  CHECK(
      management
          .accept(management_refresh, kReceiverBase + 850000001ULL, &admission)
          .ok());
  CHECK(
      control.accept(control_refresh, kReceiverBase + 850000001ULL, &admission)
          .ok());
  v2::Frame telemetry_ros;
  telemetry_ros.header =
      BaseHeader(v2::Channel::kTelemetry, v2::MessageKind::kRosMessage, 3U,
                 kSenderBase + 850000001ULL, kEpoch, "boot-a");
  telemetry_ros.header.source_timestamp_ns = 1U;
  telemetry_ros.header.ros_datatype = "std_msgs/String";
  telemetry_ros.header.ros_md5.assign(v2::kMaxRosMd5Bytes, '0');
  telemetry_ros.header.schema = "ros1.serialized.v2";
  telemetry_ros.payload.assign(1U, 0U);
  CHECK(
      telemetry.accept(telemetry_ros, kReceiverBase + 850000001ULL, &admission)
          .ok());
  CHECK_ERROR(v2::requireThreeChannelFresh(management, control, telemetry,
                                           kReceiverBase + 850000002ULL),
              v2::ErrorCode::kHeartbeatTimeout);

  const auto check_three_channel_contract_rejection =
      [&](v2::ExpectedPeer telemetry_expected,
          v2::Frame candidate_telemetry_hello,
          v2::Frame candidate_telemetry_heartbeat,
          v2::ErrorCode expected_error) {
        v2::ReceiveGuard candidate_management(
            Expected(kEpoch), v2::Channel::kManagement, timebase);
        v2::ReceiveGuard candidate_control(Expected(kEpoch),
                                           v2::Channel::kControl, timebase);
        v2::ReceiveGuard candidate_telemetry(std::move(telemetry_expected),
                                             v2::Channel::kTelemetry, timebase);
        v2::Admission candidate_admission;
        CHECK(candidate_management
                  .accept(management_hello, kReceiverBase, &candidate_admission)
                  .ok());
        CHECK(candidate_control
                  .accept(control_hello, kReceiverBase, &candidate_admission)
                  .ok());
        CHECK(candidate_telemetry
                  .accept(candidate_telemetry_hello, kReceiverBase,
                          &candidate_admission)
                  .ok());
        CHECK(candidate_management
                  .accept(heartbeat, kReceiverBase + 100000000ULL,
                          &candidate_admission)
                  .ok());
        CHECK(candidate_control
                  .accept(control_heartbeat, kReceiverBase + 100000000ULL,
                          &candidate_admission)
                  .ok());
        CHECK(candidate_telemetry
                  .accept(candidate_telemetry_heartbeat,
                          kReceiverBase + 100000000ULL, &candidate_admission)
                  .ok());
        CHECK_ERROR(v2::requireThreeChannelFresh(
                        candidate_management, candidate_control,
                        candidate_telemetry, kReceiverBase + 100000001ULL),
                    expected_error);
      };

  check_three_channel_contract_rejection(
      Expected(kEpoch),
      HelloOn(v2::Channel::kTelemetry, 1U, kSenderBase, kEpoch, "boot-b"),
      MakeHeartbeatFrame(2U, kSenderBase + 100000000ULL, kEpoch, "boot-b",
                         v2::Channel::kTelemetry),
      v2::ErrorCode::kBootMismatch);

  v2::ExpectedPeer different_peer = Expected(kEpoch);
  different_peer.asset_id = "asset-b";
  v2::Frame different_peer_hello = telemetry_hello;
  different_peer_hello.header.asset_id = "asset-b";
  v2::Frame different_peer_heartbeat = telemetry_heartbeat;
  different_peer_heartbeat.header.asset_id = "asset-b";
  check_three_channel_contract_rejection(different_peer, different_peer_hello,
                                         different_peer_heartbeat,
                                         v2::ErrorCode::kIdentityMismatch);

  check_three_channel_contract_rejection(
      Expected(kEpoch + 1U),
      HelloOn(v2::Channel::kTelemetry, 1U, kSenderBase, kEpoch + 1U),
      MakeHeartbeatFrame(2U, kSenderBase + 100000000ULL, kEpoch + 1U, "boot-a",
                         v2::Channel::kTelemetry),
      v2::ErrorCode::kEpochMismatch);

  v2::ExpectedPeer different_capabilities =
      Expected(kEpoch, v2::PeerRole::kGround);
  v2::Frame different_capabilities_hello = telemetry_hello;
  different_capabilities_hello.header.capabilities =
      v2::kGroundRequiredCapabilities;
  v2::Frame different_capabilities_heartbeat = telemetry_heartbeat;
  different_capabilities_heartbeat.header.capabilities =
      v2::kGroundRequiredCapabilities;
  check_three_channel_contract_rejection(
      different_capabilities, different_capabilities_hello,
      different_capabilities_heartbeat, v2::ErrorCode::kCapabilityMismatch);

  management.reset(Expected(kEpoch + 1U), timebase);
  CHECK_ERROR(management.accept(management_hello, kReceiverBase, &admission),
              v2::ErrorCode::kEpochMismatch);
  CHECK(!admission.valid());

  const auto check_identity_rejection = [&](v2::Frame candidate,
                                            const v2::ExpectedPeer &expected,
                                            v2::ErrorCode expected_error) {
    v2::ReceiveGuard guard(expected, v2::Channel::kManagement, timebase);
    v2::Admission rejected;
    CHECK_ERROR(guard.accept(candidate, kReceiverBase, &rejected),
                expected_error);
    CHECK(!rejected.valid());
    CHECK(!guard.established());
  };

  v2::Frame wrong = management_hello;
  wrong.header.slot_id = "scout-02";
  check_identity_rejection(wrong, Expected(kEpoch),
                           v2::ErrorCode::kIdentityMismatch);
  wrong = management_hello;
  wrong.header.asset_id = "asset-b";
  check_identity_rejection(wrong, Expected(kEpoch),
                           v2::ErrorCode::kIdentityMismatch);
  wrong = management_hello;
  wrong.header.robot_kind = "fs150";
  check_identity_rejection(wrong, Expected(kEpoch),
                           v2::ErrorCode::kIdentityMismatch);
  wrong = management_hello;
  wrong.header.run_id = "run-43";
  check_identity_rejection(wrong, Expected(kEpoch),
                           v2::ErrorCode::kIdentityMismatch);
  wrong = management_hello;
  wrong.header.build_id = "build-10";
  check_identity_rejection(wrong, Expected(kEpoch),
                           v2::ErrorCode::kIdentityMismatch);
  wrong = management_hello;
  wrong.header.capabilities ^= 1U;
  check_identity_rejection(wrong, Expected(kEpoch),
                           v2::ErrorCode::kCapabilityMismatch);
  wrong = management_hello;
  wrong.header.session_epoch = kEpoch + 1U;
  check_identity_rejection(wrong, Expected(kEpoch),
                           v2::ErrorCode::kEpochMismatch);
  v2::ExpectedPeer boot_bound = Expected(kEpoch);
  boot_bound.boot_id = "boot-a";
  wrong = management_hello;
  wrong.header.boot_id = "boot-b";
  check_identity_rejection(wrong, boot_bound, v2::ErrorCode::kBootMismatch);

  v2::ReceiveGuard delayed(Expected(kEpoch), v2::Channel::kManagement,
                           timebase);
  CHECK_ERROR(delayed.accept(management_hello, kReceiverBase + 800000000ULL,
                             &admission),
              v2::ErrorCode::kStale);
  CHECK(!delayed.established());

  v2::ReceiveGuard future(Expected(kEpoch), v2::Channel::kManagement, timebase);
  const v2::Frame future_hello =
      HelloOn(v2::Channel::kManagement, 1U, kSenderBase + 60000000ULL, kEpoch);
  CHECK_ERROR(future.accept(future_hello, kReceiverBase, &admission),
              v2::ErrorCode::kFutureTimestamp);

  v2::MonotonicTimebase expired_timebase = timebase;
  expired_timebase.valid_until_receiver_ns = kReceiverBase + 100000000ULL;
  v2::ReceiveGuard expired(Expected(kEpoch), v2::Channel::kManagement,
                           expired_timebase);
  CHECK(expired.accept(management_hello, kReceiverBase, &admission).ok());
  CHECK_ERROR(expired.checkFresh(kReceiverBase + 100000001ULL),
              v2::ErrorCode::kHeartbeatTimeout);
  CHECK_ERROR(
      expired.accept(heartbeat, kReceiverBase + 100000001ULL, &admission),
      v2::ErrorCode::kStale);

  v2::MonotonicTimebase uncertain = timebase;
  uncertain.maximum_error_ns = v2::kMaximumTimebaseErrorNs + 1U;
  v2::ReceiveGuard invalid_clock(Expected(kEpoch), v2::Channel::kManagement,
                                 uncertain);
  CHECK_ERROR(invalid_clock.accept(management_hello, kReceiverBase, &admission),
              v2::ErrorCode::kInvalidMetadata);

  v2::ReceivePolicy zero_timeout;
  zero_timeout.maximum_age_ns = 0U;
  v2::ReceiveGuard invalid_timeout(Expected(kEpoch), v2::Channel::kManagement,
                                   timebase, zero_timeout);
  CHECK_ERROR(
      invalid_timeout.accept(management_hello, kReceiverBase, &admission),
      v2::ErrorCode::kInvalidMetadata);

  v2::ReceiveGuard invalid_epoch(Expected(0U), v2::Channel::kManagement,
                                 timebase);
  CHECK_ERROR(invalid_epoch.accept(management_hello, kReceiverBase, &admission),
              v2::ErrorCode::kInvalidIdentity);

  // A 50 ms transit delay is already charged against the sender deadline. The
  // receiver deadline is an absolute conservative lower bound, never a fresh
  // TTL starting at arrival.
  v2::ReceiveGuard stop_guard(Expected(kEpoch, v2::PeerRole::kGround),
                              v2::Channel::kControl, timebase);
  const v2::Frame ground_control_hello =
      HelloOn(v2::Channel::kControl, 1U, kSenderBase, kEpoch, "boot-a",
              v2::PeerRole::kGround);
  const v2::Frame ground_control_heartbeat =
      MakeHeartbeatFrame(2U, kSenderBase + 100000000ULL, kEpoch, "boot-a",
                         v2::Channel::kControl, v2::PeerRole::kGround);
  CHECK(
      stop_guard.accept(ground_control_hello, kReceiverBase, &admission).ok());
  CHECK(stop_guard
            .accept(ground_control_heartbeat, kReceiverBase + 100000000ULL,
                    &admission)
            .ok());
  v2::ZeroStop transport_stop;
  transport_stop.command_id = "transport-stop";
  transport_stop.deadline_monotonic_ns = kSenderBase + 500000000ULL;
  v2::Frame transport_frame;
  transport_frame.header = BaseHeader(
      v2::Channel::kControl, v2::MessageKind::kZeroStop, 3U,
      kSenderBase + 200000000ULL, kEpoch, "boot-a", v2::PeerRole::kGround);
  transport_frame.header.schema = v2::kZeroStopSchema;
  CHECK(
      v2::encodeZeroStopPayload(transport_stop, &transport_frame.payload).ok());
  CHECK(stop_guard
            .accept(transport_frame, kReceiverBase + 250000000ULL, &admission)
            .ok());
  CHECK(admission.mappedStopDeadlineNs() == kReceiverBase + 499000000ULL);

  v2::ZeroStop late_stop = transport_stop;
  late_stop.command_id = "late-stop";
  v2::Frame late_frame = transport_frame;
  late_frame.header.sequence = 4U;
  late_frame.header.monotonic_ns = kSenderBase + 300000000ULL;
  CHECK(v2::encodeZeroStopPayload(late_stop, &late_frame.payload).ok());
  CHECK_ERROR(
      stop_guard.accept(late_frame, kReceiverBase + 499000001ULL, &admission),
      v2::ErrorCode::kDeadlineExpired);
  CHECK(!admission.valid());
}

void TestWatchdogCompleteTransitions() {
  constexpr std::uint64_t kEpoch = 9U;
  constexpr std::uint64_t kSenderBase = 1000000000ULL;
  constexpr std::uint64_t kReceiverBase = 9000000000ULL;
  v2::StopWatchdog watchdog(kEpoch);
  v2::SafetyAction action = v2::SafetyAction::kNone;
  v2::StopReceipt receipt;
  v2::ZeroStop stop;
  stop.command_id = "startup-stop";
  stop.deadline_monotonic_ns = kSenderBase + 500000000ULL;

  v2::Admission invalid;
  CHECK_ERROR(
      watchdog.acceptZeroStop(stop, invalid, kReceiverBase, &receipt, &action),
      v2::ErrorCode::kHandshakeRequired);
  CHECK(watchdog.state() == v2::LinkState::kAwaitHello);
  CHECK(receipt.command_id.empty());

  v2::StopWatchdog zero_epoch(0U);
  CHECK(zero_epoch.state() == v2::LinkState::kSafeHold);
  CHECK_ERROR(zero_epoch.reenable(0U, "boot-a", kReceiverBase),
              v2::ErrorCode::kHandshakeRequired);

  v2::ReceiveGuard control_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                 v2::Channel::kControl,
                                 Timebase(kSenderBase, kReceiverBase));
  const v2::Frame hello = HelloOn(v2::Channel::kControl, 1U, kSenderBase,
                                  kEpoch, "boot-a", v2::PeerRole::kGround);
  v2::Admission hello_admission;
  CHECK(control_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(watchdog.onHello(hello_admission, kReceiverBase, &action).ok());
  CHECK(watchdog.state() == v2::LinkState::kInhibited);
  CHECK(watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());

  const v2::Frame heartbeat =
      MakeHeartbeatFrame(2U, kSenderBase + 100000000ULL, kEpoch, "boot-a",
                         v2::Channel::kControl, v2::PeerRole::kGround);
  v2::Admission heartbeat_admission;
  CHECK(
      control_guard
          .accept(heartbeat, kReceiverBase + 100000000ULL, &heartbeat_admission)
          .ok());
  CHECK(watchdog
            .onHeartbeat(heartbeat_admission, kReceiverBase + 100000001ULL,
                         &action)
            .ok());

  v2::Frame stop_frame;
  stop_frame.header = BaseHeader(
      v2::Channel::kControl, v2::MessageKind::kZeroStop, 3U,
      kSenderBase + 200000000ULL, kEpoch, "boot-a", v2::PeerRole::kGround);
  stop_frame.header.schema = v2::kZeroStopSchema;
  CHECK(v2::encodeZeroStopPayload(stop, &stop_frame.payload).ok());
  v2::Admission stop_admission;
  CHECK(control_guard
            .accept(stop_frame, kReceiverBase + 250000000ULL, &stop_admission)
            .ok());
  CHECK(stop_admission.mappedStopDeadlineNs() == kReceiverBase + 499000000ULL);
  CHECK(watchdog
            .acceptZeroStop(stop, stop_admission, kReceiverBase + 250000001ULL,
                            &receipt, &action)
            .ok());
  CHECK(receipt.phase == v2::ReceiptPhase::kAccepted);
  CHECK(action == v2::SafetyAction::kApplyZeroStop);
  CHECK(watchdog.state() == v2::LinkState::kSafeHold);
  CHECK(watchdog
            .confirmStopApplied("startup-stop", kReceiverBase + 300000000ULL,
                                &receipt)
            .ok());
  CHECK(receipt.phase == v2::ReceiptPhase::kApplied);

  CHECK(watchdog.onEpochChanged(0U, &action).code ==
        v2::ErrorCode::kInvalidMetadata);
  CHECK(action == v2::SafetyAction::kEnterSafeHold);

  // A guard-issued but stale token may not generate an OK receipt.
  v2::StopWatchdog stale_watchdog(kEpoch);
  v2::ReceiveGuard stale_guard(Expected(kEpoch, v2::PeerRole::kGround),
                               v2::Channel::kControl,
                               Timebase(kSenderBase, kReceiverBase));
  CHECK(stale_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(stale_watchdog.onHello(hello_admission, kReceiverBase, &action).ok());
  CHECK(stale_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());
  CHECK(
      stale_guard
          .accept(heartbeat, kReceiverBase + 100000000ULL, &heartbeat_admission)
          .ok());
  CHECK(stale_watchdog
            .onHeartbeat(heartbeat_admission, kReceiverBase + 100000001ULL,
                         &action)
            .ok());
  v2::ZeroStop long_stop;
  long_stop.command_id = "stale-token-stop";
  long_stop.deadline_monotonic_ns = kSenderBase + 1200000000ULL;
  v2::Frame long_stop_frame;
  long_stop_frame.header = BaseHeader(
      v2::Channel::kControl, v2::MessageKind::kZeroStop, 3U,
      kSenderBase + 200000000ULL, kEpoch, "boot-a", v2::PeerRole::kGround);
  long_stop_frame.header.schema = v2::kZeroStopSchema;
  CHECK(v2::encodeZeroStopPayload(long_stop, &long_stop_frame.payload).ok());
  v2::Admission stale_stop_admission;
  CHECK(stale_guard
            .accept(long_stop_frame, kReceiverBase + 200000000ULL,
                    &stale_stop_admission)
            .ok());
  receipt = v2::StopReceipt{"sentinel", v2::ReceiptPhase::kApplied,
                            v2::ReceiptStatus::kRejected, 1U, "unchanged"};
  CHECK_ERROR(stale_watchdog.acceptZeroStop(long_stop, stale_stop_admission,
                                            kReceiverBase + 950000001ULL,
                                            &receipt, &action),
              v2::ErrorCode::kStale);
  CHECK(action == v2::SafetyAction::kEnterSafeHold);
  CHECK(receipt.command_id == "sentinel");
  CHECK(receipt.status == v2::ReceiptStatus::kRejected);

  // Reusing an already-consumed admission is a fail-safe replay.
  v2::StopWatchdog replay_watchdog(kEpoch);
  v2::ReceiveGuard replay_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                v2::Channel::kControl,
                                Timebase(kSenderBase, kReceiverBase));
  CHECK(replay_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(replay_watchdog.onHello(hello_admission, kReceiverBase, &action).ok());
  CHECK(replay_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());
  CHECK(
      replay_guard
          .accept(heartbeat, kReceiverBase + 100000000ULL, &heartbeat_admission)
          .ok());
  CHECK(replay_watchdog
            .onHeartbeat(heartbeat_admission, kReceiverBase + 100000001ULL,
                         &action)
            .ok());
  CHECK_ERROR(replay_watchdog.onHeartbeat(
                  heartbeat_admission, kReceiverBase + 100000002ULL, &action),
              v2::ErrorCode::kReplay);
  CHECK(action == v2::SafetyAction::kEnterSafeHold);

  // Valid admissions from a differently frozen epoch or boot must stop the
  // current watchdog; admission validity alone is not enough.
  v2::StopWatchdog wrong_epoch_watchdog(kEpoch);
  v2::ReceiveGuard proper_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                v2::Channel::kControl,
                                Timebase(kSenderBase, kReceiverBase));
  CHECK(proper_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(wrong_epoch_watchdog.onHello(hello_admission, kReceiverBase, &action)
            .ok());
  CHECK(
      wrong_epoch_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());
  v2::ReceiveGuard other_epoch_guard(
      Expected(kEpoch + 1U, v2::PeerRole::kGround), v2::Channel::kControl,
      Timebase(kSenderBase, kReceiverBase));
  const v2::Frame other_epoch_hello =
      HelloOn(v2::Channel::kControl, 1U, kSenderBase, kEpoch + 1U, "boot-a",
              v2::PeerRole::kGround);
  v2::Admission other_epoch_admission;
  CHECK(other_epoch_guard
            .accept(other_epoch_hello, kReceiverBase, &other_epoch_admission)
            .ok());
  const v2::Frame other_epoch_heartbeat =
      MakeHeartbeatFrame(2U, kSenderBase + 100000000ULL, kEpoch + 1U, "boot-a",
                         v2::Channel::kControl, v2::PeerRole::kGround);
  CHECK(other_epoch_guard
            .accept(other_epoch_heartbeat, kReceiverBase + 100000000ULL,
                    &other_epoch_admission)
            .ok());
  CHECK_ERROR(wrong_epoch_watchdog.onHeartbeat(
                  other_epoch_admission, kReceiverBase + 100000001ULL, &action),
              v2::ErrorCode::kEpochMismatch);
  CHECK(action == v2::SafetyAction::kEnterSafeHold);

  v2::StopWatchdog wrong_boot_watchdog(kEpoch);
  v2::ReceiveGuard proper_boot_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                     v2::Channel::kControl,
                                     Timebase(kSenderBase, kReceiverBase));
  CHECK(proper_boot_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(wrong_boot_watchdog.onHello(hello_admission, kReceiverBase, &action)
            .ok());
  CHECK(
      wrong_boot_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());
  v2::ExpectedPeer other_boot_expected =
      Expected(kEpoch, v2::PeerRole::kGround);
  other_boot_expected.boot_id = "boot-b";
  v2::ReceiveGuard other_boot_guard(other_boot_expected, v2::Channel::kControl,
                                    Timebase(kSenderBase, kReceiverBase));
  const v2::Frame other_boot_hello =
      HelloOn(v2::Channel::kControl, 1U, kSenderBase, kEpoch, "boot-b",
              v2::PeerRole::kGround);
  v2::Admission other_boot_admission;
  CHECK(other_boot_guard
            .accept(other_boot_hello, kReceiverBase, &other_boot_admission)
            .ok());
  const v2::Frame other_boot_heartbeat =
      MakeHeartbeatFrame(2U, kSenderBase + 100000000ULL, kEpoch, "boot-b",
                         v2::Channel::kControl, v2::PeerRole::kGround);
  CHECK(other_boot_guard
            .accept(other_boot_heartbeat, kReceiverBase + 100000000ULL,
                    &other_boot_admission)
            .ok());
  CHECK_ERROR(wrong_boot_watchdog.onHeartbeat(
                  other_boot_admission, kReceiverBase + 100000001ULL, &action),
              v2::ErrorCode::kBootMismatch);
  CHECK(action == v2::SafetyAction::kEnterSafeHold);

  v2::StopWatchdog restart_watchdog(kEpoch);
  v2::ReceiveGuard restart_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                 v2::Channel::kControl,
                                 Timebase(kSenderBase, kReceiverBase));
  CHECK(restart_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(restart_watchdog.onHello(hello_admission, kReceiverBase, &action).ok());
  CHECK(restart_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());
  const v2::Frame enabled_restart_hello =
      HelloOn(v2::Channel::kControl, 2U, kSenderBase + 100000000ULL, kEpoch,
              "boot-a", v2::PeerRole::kGround);
  CHECK_ERROR(restart_guard.accept(enabled_restart_hello,
                                   kReceiverBase + 100000000ULL,
                                   &hello_admission),
              v2::ErrorCode::kInvalidState);
  restart_guard.reset(Expected(kEpoch, v2::PeerRole::kGround),
                      Timebase(kSenderBase, kReceiverBase));
  CHECK(restart_guard
            .accept(enabled_restart_hello, kReceiverBase + 100000000ULL,
                    &hello_admission)
            .ok());
  CHECK(restart_watchdog
            .onHello(hello_admission, kReceiverBase + 100000001ULL, &action)
            .ok());
  CHECK(action == v2::SafetyAction::kEnterSafeHold);
  CHECK(restart_watchdog.state() == v2::LinkState::kInhibited);

  v2::StopWatchdog epoch_change_watchdog(kEpoch);
  v2::ReceiveGuard epoch_change_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                      v2::Channel::kControl,
                                      Timebase(kSenderBase, kReceiverBase));
  CHECK(epoch_change_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(epoch_change_watchdog.onHello(hello_admission, kReceiverBase, &action)
            .ok());
  CHECK(epoch_change_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U)
            .ok());
  CHECK(epoch_change_watchdog.onEpochChanged(kEpoch + 1U, &action).ok());
  CHECK(action == v2::SafetyAction::kEnterSafeHold);
  CHECK(epoch_change_watchdog.state() == v2::LinkState::kSafeHold);
  v2::ReceiveGuard new_epoch_guard(Expected(kEpoch + 1U, v2::PeerRole::kGround),
                                   v2::Channel::kControl,
                                   Timebase(kSenderBase, kReceiverBase));
  CHECK(new_epoch_guard
            .accept(other_epoch_hello, kReceiverBase, &other_epoch_admission)
            .ok());
  CHECK(epoch_change_watchdog
            .onHello(other_epoch_admission, kReceiverBase + 1U, &action)
            .ok());
  CHECK(epoch_change_watchdog.state() == v2::LinkState::kInhibited);
  CHECK(
      epoch_change_watchdog.reenable(kEpoch + 1U, "boot-a", kReceiverBase + 2U)
          .ok());
  CHECK(epoch_change_watchdog.state() == v2::LinkState::kEnabled);

  // Heartbeat timeout enters safe hold. A heartbeat alone cannot recover;
  // recovery needs a newly admitted HELLO and explicit re-enable.
  v2::StopWatchdog recovery_watchdog(kEpoch);
  v2::ReceiveGuard recovery_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                  v2::Channel::kControl,
                                  Timebase(kSenderBase, kReceiverBase));
  CHECK(recovery_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(
      recovery_watchdog.onHello(hello_admission, kReceiverBase, &action).ok());
  CHECK(recovery_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());
  CHECK(
      recovery_guard
          .accept(heartbeat, kReceiverBase + 100000000ULL, &heartbeat_admission)
          .ok());
  CHECK(recovery_watchdog
            .onHeartbeat(heartbeat_admission, kReceiverBase + 100000001ULL,
                         &action)
            .ok());
  CHECK_ERROR(recovery_watchdog.poll(kReceiverBase + 850000001ULL, &action),
              v2::ErrorCode::kHeartbeatTimeout);
  CHECK(action == v2::SafetyAction::kEnterSafeHold);
  const v2::Frame recovery_heartbeat =
      MakeHeartbeatFrame(3U, kSenderBase + 900000000ULL, kEpoch, "boot-a",
                         v2::Channel::kControl, v2::PeerRole::kGround);
  v2::Admission recovery_admission;
  CHECK(recovery_guard
            .accept(recovery_heartbeat, kReceiverBase + 900000000ULL,
                    &recovery_admission)
            .ok());
  CHECK_ERROR(recovery_watchdog.onHeartbeat(
                  recovery_admission, kReceiverBase + 900000001ULL, &action),
              v2::ErrorCode::kHandshakeRequired);
  CHECK(recovery_watchdog.state() == v2::LinkState::kSafeHold);
  const v2::Frame recovery_hello =
      HelloOn(v2::Channel::kControl, 4U, kSenderBase + 1000000000ULL, kEpoch,
              "boot-a", v2::PeerRole::kGround);
  CHECK_ERROR(recovery_guard.accept(recovery_hello,
                                    kReceiverBase + 1000000000ULL,
                                    &recovery_admission),
              v2::ErrorCode::kInvalidState);
  recovery_guard.reset(Expected(kEpoch, v2::PeerRole::kGround),
                       Timebase(kSenderBase, kReceiverBase));
  CHECK(recovery_guard
            .accept(recovery_hello, kReceiverBase + 1000000000ULL,
                    &recovery_admission)
            .ok());
  CHECK(recovery_watchdog
            .onHello(recovery_admission, kReceiverBase + 1000000001ULL, &action)
            .ok());
  CHECK(recovery_watchdog.state() == v2::LinkState::kInhibited);
  CHECK(recovery_watchdog
            .reenable(kEpoch, "boot-a", kReceiverBase + 1000000002ULL)
            .ok());
  CHECK(recovery_watchdog.state() == v2::LinkState::kEnabled);

  // Even a valid ZERO_STOP token cannot authorize a nonzero DTO.
  v2::StopWatchdog nonzero_watchdog(kEpoch);
  v2::ReceiveGuard nonzero_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                 v2::Channel::kControl,
                                 Timebase(kSenderBase, kReceiverBase));
  CHECK(nonzero_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(nonzero_watchdog.onHello(hello_admission, kReceiverBase, &action).ok());
  CHECK(nonzero_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());
  CHECK(
      nonzero_guard
          .accept(heartbeat, kReceiverBase + 100000000ULL, &heartbeat_admission)
          .ok());
  CHECK(nonzero_watchdog
            .onHeartbeat(heartbeat_admission, kReceiverBase + 100000001ULL,
                         &action)
            .ok());
  v2::ZeroStop zero_for_token;
  zero_for_token.command_id = "nonzero-rejected";
  zero_for_token.deadline_monotonic_ns = kSenderBase + 600000000ULL;
  v2::Frame zero_for_token_frame;
  zero_for_token_frame.header = BaseHeader(
      v2::Channel::kControl, v2::MessageKind::kZeroStop, 3U,
      kSenderBase + 200000000ULL, kEpoch, "boot-a", v2::PeerRole::kGround);
  zero_for_token_frame.header.schema = v2::kZeroStopSchema;
  CHECK(v2::encodeZeroStopPayload(zero_for_token, &zero_for_token_frame.payload)
            .ok());
  v2::Admission zero_token;
  CHECK(nonzero_guard
            .accept(zero_for_token_frame, kReceiverBase + 200000000ULL,
                    &zero_token)
            .ok());
  v2::ZeroStop unsafe_stop = zero_for_token;
  unsafe_stop.axes[0] = 0.01;
  receipt = v2::StopReceipt{"sentinel", v2::ReceiptPhase::kApplied,
                            v2::ReceiptStatus::kRejected, 1U, "unchanged"};
  CHECK_ERROR(nonzero_watchdog.acceptZeroStop(unsafe_stop, zero_token,
                                              kReceiverBase + 200000001ULL,
                                              &receipt, &action),
              v2::ErrorCode::kUnsafeNonZero);
  CHECK(action == v2::SafetyAction::kEnterSafeHold);
  CHECK(receipt.command_id == "sentinel");

  // An Admission binds every canonical ZERO_STOP payload byte, including the
  // sender-domain deadline; the same command ID is not sufficient.
  const v2::Frame binding_hello =
      HelloOn(v2::Channel::kControl, 4U, kSenderBase + 300000000ULL, kEpoch,
              "boot-a", v2::PeerRole::kGround);
  nonzero_guard.reset(Expected(kEpoch, v2::PeerRole::kGround),
                      Timebase(kSenderBase, kReceiverBase));
  CHECK(
      nonzero_guard
          .accept(binding_hello, kReceiverBase + 300000000ULL, &hello_admission)
          .ok());
  CHECK(nonzero_watchdog
            .onHello(hello_admission, kReceiverBase + 300000001ULL, &action)
            .ok());
  CHECK(
      nonzero_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 300000002ULL)
          .ok());
  const v2::Frame binding_heartbeat =
      MakeHeartbeatFrame(5U, kSenderBase + 400000000ULL, kEpoch, "boot-a",
                         v2::Channel::kControl, v2::PeerRole::kGround);
  CHECK(nonzero_guard
            .accept(binding_heartbeat, kReceiverBase + 400000000ULL,
                    &heartbeat_admission)
            .ok());
  CHECK(nonzero_watchdog
            .onHeartbeat(heartbeat_admission, kReceiverBase + 400000001ULL,
                         &action)
            .ok());
  v2::ZeroStop binding_stop;
  binding_stop.command_id = "deadline-binding";
  binding_stop.deadline_monotonic_ns = kSenderBase + 900000000ULL;
  v2::Frame binding_stop_frame;
  binding_stop_frame.header = BaseHeader(
      v2::Channel::kControl, v2::MessageKind::kZeroStop, 6U,
      kSenderBase + 500000000ULL, kEpoch, "boot-a", v2::PeerRole::kGround);
  binding_stop_frame.header.schema = v2::kZeroStopSchema;
  CHECK(v2::encodeZeroStopPayload(binding_stop, &binding_stop_frame.payload)
            .ok());
  CHECK(
      nonzero_guard
          .accept(binding_stop_frame, kReceiverBase + 550000000ULL, &zero_token)
          .ok());
  v2::ZeroStop different_deadline = binding_stop;
  different_deadline.deadline_monotonic_ns = kSenderBase + 950000000ULL;
  receipt = v2::StopReceipt{"sentinel", v2::ReceiptPhase::kApplied,
                            v2::ReceiptStatus::kRejected, 1U, "unchanged"};
  CHECK_ERROR(nonzero_watchdog.acceptZeroStop(different_deadline, zero_token,
                                              kReceiverBase + 550000001ULL,
                                              &receipt, &action),
              v2::ErrorCode::kInvalidMetadata);
  CHECK(action == v2::SafetyAction::kEnterSafeHold);
  CHECK(receipt.command_id == "sentinel");

  // APPLIED after the absolute mapped deadline never becomes an OK proof.
  v2::StopWatchdog late_apply_watchdog(kEpoch);
  v2::ReceiveGuard late_apply_guard(Expected(kEpoch, v2::PeerRole::kGround),
                                    v2::Channel::kControl,
                                    Timebase(kSenderBase, kReceiverBase));
  CHECK(late_apply_guard.accept(hello, kReceiverBase, &hello_admission).ok());
  CHECK(late_apply_watchdog.onHello(hello_admission, kReceiverBase, &action)
            .ok());
  CHECK(
      late_apply_watchdog.reenable(kEpoch, "boot-a", kReceiverBase + 1U).ok());
  CHECK(
      late_apply_guard
          .accept(heartbeat, kReceiverBase + 100000000ULL, &heartbeat_admission)
          .ok());
  CHECK(late_apply_watchdog
            .onHeartbeat(heartbeat_admission, kReceiverBase + 100000001ULL,
                         &action)
            .ok());
  v2::ZeroStop short_stop;
  short_stop.command_id = "late-apply";
  short_stop.deadline_monotonic_ns = kSenderBase + 400000000ULL;
  v2::Frame short_stop_frame;
  short_stop_frame.header = BaseHeader(
      v2::Channel::kControl, v2::MessageKind::kZeroStop, 3U,
      kSenderBase + 200000000ULL, kEpoch, "boot-a", v2::PeerRole::kGround);
  short_stop_frame.header.schema = v2::kZeroStopSchema;
  CHECK(v2::encodeZeroStopPayload(short_stop, &short_stop_frame.payload).ok());
  v2::Admission short_stop_admission;
  CHECK(late_apply_guard
            .accept(short_stop_frame, kReceiverBase + 200000000ULL,
                    &short_stop_admission)
            .ok());
  CHECK(late_apply_watchdog
            .acceptZeroStop(short_stop, short_stop_admission,
                            kReceiverBase + 200000001ULL, &receipt, &action)
            .ok());
  receipt = v2::StopReceipt{"sentinel", v2::ReceiptPhase::kAccepted,
                            v2::ReceiptStatus::kRejected, 1U, "unchanged"};
  CHECK_ERROR(late_apply_watchdog.confirmStopApplied(
                  "late-apply", kReceiverBase + 399000001ULL, &receipt),
              v2::ErrorCode::kDeadlineExpired);
  CHECK(receipt.command_id == "sentinel");
}

void TestStopReceiptCorrelator() {
  constexpr std::uint64_t kEpoch = 31U;
  constexpr std::uint64_t kRobotBase = 2000000000ULL;
  constexpr std::uint64_t kGroundBase = 10000000000ULL;
  v2::ReceiveGuard telemetry_guard(Expected(kEpoch), v2::Channel::kTelemetry,
                                   Timebase(kRobotBase, kGroundBase));
  v2::Admission admission;
  CHECK(telemetry_guard
            .accept(HelloOn(v2::Channel::kTelemetry, 1U, kRobotBase, kEpoch),
                    kGroundBase, &admission)
            .ok());

  v2::ZeroStop stop;
  stop.command_id = "receipt-stop-1";
  stop.deadline_monotonic_ns = kGroundBase + 600000000ULL;
  v2::StopReceiptCorrelator correlator(kEpoch, "boot-a");
  CHECK(correlator.begin(stop, kGroundBase).ok());
  CHECK(correlator.progress() == v2::ReceiptProgress::kAwaitAccepted);

  v2::StopReceipt applied{stop.command_id, v2::ReceiptPhase::kApplied,
                          v2::ReceiptStatus::kOk, kRobotBase + 100000000ULL,
                          "applied"};
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(2U, kRobotBase + 100000000ULL, kEpoch,
                                     applied),
                    kGroundBase + 100000000ULL, &admission)
            .ok());
  CHECK_ERROR(
      correlator.observe(applied, admission, kGroundBase + 100000001ULL),
      v2::ErrorCode::kInvalidState);
  CHECK(correlator.progress() == v2::ReceiptProgress::kAwaitAccepted);

  v2::StopReceipt applied_rejected = applied;
  applied_rejected.status = v2::ReceiptStatus::kRejected;
  applied_rejected.observed_monotonic_ns = kRobotBase + 150000000ULL;
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(3U, kRobotBase + 150000000ULL, kEpoch,
                                     applied_rejected),
                    kGroundBase + 150000000ULL, &admission)
            .ok());
  CHECK_ERROR(correlator.observe(applied_rejected, admission,
                                 kGroundBase + 150000001ULL),
              v2::ErrorCode::kInvalidState);
  CHECK(correlator.progress() == v2::ReceiptProgress::kAwaitAccepted);

  v2::StopReceipt accepted{stop.command_id, v2::ReceiptPhase::kAccepted,
                           v2::ReceiptStatus::kOk, kRobotBase + 200000000ULL,
                           "accepted"};
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(4U, kRobotBase + 200000000ULL, kEpoch,
                                     accepted),
                    kGroundBase + 200000000ULL, &admission)
            .ok());
  v2::StopReceipt different_observed_time = accepted;
  different_observed_time.observed_monotonic_ns += 1U;
  CHECK_ERROR(correlator.observe(different_observed_time, admission,
                                 kGroundBase + 200000001ULL),
              v2::ErrorCode::kInvalidMetadata);
  CHECK(correlator.progress() == v2::ReceiptProgress::kAwaitAccepted);
  v2::StopReceipt different_detail = accepted;
  different_detail.detail = "different-detail";
  CHECK_ERROR(correlator.observe(different_detail, admission,
                                 kGroundBase + 200000001ULL),
              v2::ErrorCode::kInvalidMetadata);
  CHECK(correlator.progress() == v2::ReceiptProgress::kAwaitAccepted);
  CHECK(
      correlator.observe(accepted, admission, kGroundBase + 200000001ULL).ok());
  CHECK(correlator.progress() == v2::ReceiptProgress::kAwaitApplied);

  accepted.observed_monotonic_ns = kRobotBase + 300000000ULL;
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(5U, kRobotBase + 300000000ULL, kEpoch,
                                     accepted),
                    kGroundBase + 300000000ULL, &admission)
            .ok());
  CHECK_ERROR(
      correlator.observe(accepted, admission, kGroundBase + 300000001ULL),
      v2::ErrorCode::kReplay);
  CHECK(correlator.progress() == v2::ReceiptProgress::kAwaitApplied);

  applied.observed_monotonic_ns = kRobotBase + 400000000ULL;
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(6U, kRobotBase + 400000000ULL, kEpoch,
                                     applied),
                    kGroundBase + 400000000ULL, &admission)
            .ok());
  CHECK(
      correlator.observe(applied, admission, kGroundBase + 400000001ULL).ok());
  CHECK(correlator.complete());

  applied.observed_monotonic_ns = kRobotBase + 450000000ULL;
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(7U, kRobotBase + 450000000ULL, kEpoch,
                                     applied),
                    kGroundBase + 450000000ULL, &admission)
            .ok());
  CHECK_ERROR(
      correlator.observe(applied, admission, kGroundBase + 450000001ULL),
      v2::ErrorCode::kInvalidState);
  CHECK_ERROR(correlator.begin(stop, kGroundBase + 450000001ULL),
              v2::ErrorCode::kReplay);

  v2::ZeroStop second_stop;
  second_stop.command_id = "receipt-stop-2";
  second_stop.deadline_monotonic_ns = kGroundBase + 900000000ULL;
  CHECK(correlator.begin(second_stop, kGroundBase + 500000000ULL).ok());

  v2::StopReceipt wrong_command{"other-command", v2::ReceiptPhase::kAccepted,
                                v2::ReceiptStatus::kOk,
                                kRobotBase + 500000000ULL, "accepted"};
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(8U, kRobotBase + 500000000ULL, kEpoch,
                                     wrong_command),
                    kGroundBase + 500000000ULL, &admission)
            .ok());
  CHECK_ERROR(
      correlator.observe(wrong_command, admission, kGroundBase + 500000001ULL),
      v2::ErrorCode::kIdentityMismatch);

  v2::StopReceipt second_accepted{
      second_stop.command_id, v2::ReceiptPhase::kAccepted,
      v2::ReceiptStatus::kOk, kRobotBase + 600000000ULL, "accepted"};
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(9U, kRobotBase + 600000000ULL, kEpoch,
                                     second_accepted),
                    kGroundBase + 600000000ULL, &admission)
            .ok());
  v2::StopReceipt token_mismatch = second_accepted;
  token_mismatch.phase = v2::ReceiptPhase::kApplied;
  CHECK_ERROR(
      correlator.observe(token_mismatch, admission, kGroundBase + 600000001ULL),
      v2::ErrorCode::kInvalidMetadata);
  CHECK(correlator.progress() == v2::ReceiptProgress::kAwaitAccepted);
  CHECK(
      correlator.observe(second_accepted, admission, kGroundBase + 600000002ULL)
          .ok());

  v2::StopReceipt second_applied{
      second_stop.command_id, v2::ReceiptPhase::kApplied,
      v2::ReceiptStatus::kOk, kRobotBase + 900000000ULL, "applied"};
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(10U, kRobotBase + 900000000ULL, kEpoch,
                                     second_applied),
                    kGroundBase + 900000001ULL, &admission)
            .ok());
  CHECK_ERROR(
      correlator.observe(second_applied, admission, kGroundBase + 900000002ULL),
      v2::ErrorCode::kDeadlineExpired);
  CHECK(correlator.progress() == v2::ReceiptProgress::kExpired);

  v2::ZeroStop rejected_stop;
  rejected_stop.command_id = "receipt-stop-3";
  rejected_stop.deadline_monotonic_ns = kGroundBase + 1300000000ULL;
  CHECK(correlator.begin(rejected_stop, kGroundBase + 1000000000ULL).ok());
  v2::StopReceipt rejected{
      rejected_stop.command_id, v2::ReceiptPhase::kAccepted,
      v2::ReceiptStatus::kRejected, kRobotBase + 1100000000ULL, "rejected"};
  CHECK(telemetry_guard
            .accept(MakeReceiptFrame(11U, kRobotBase + 1100000000ULL, kEpoch,
                                     rejected),
                    kGroundBase + 1100000000ULL, &admission)
            .ok());
  CHECK(correlator.observe(rejected, admission, kGroundBase + 1100000001ULL)
            .ok());
  CHECK(correlator.progress() == v2::ReceiptProgress::kRejected);

  v2::StopReceiptCorrelator polling(kEpoch, "boot-a");
  v2::ZeroStop polling_stop;
  polling_stop.command_id = "poll-expiry";
  polling_stop.deadline_monotonic_ns = kGroundBase + 200000000ULL;
  CHECK(polling.begin(polling_stop, kGroundBase).ok());
  CHECK_ERROR(polling.poll(kGroundBase + 200000001ULL),
              v2::ErrorCode::kDeadlineExpired);
  CHECK(polling.progress() == v2::ReceiptProgress::kExpired);

  v2::StopReceiptCorrelator invalid_identity(0U, std::string());
  CHECK_ERROR(invalid_identity.begin(polling_stop, kGroundBase),
              v2::ErrorCode::kInvalidIdentity);

  v2::ZeroStop identity_stop;
  identity_stop.command_id = "identity-receipt";
  identity_stop.deadline_monotonic_ns = kGroundBase + 900000000ULL;
  v2::StopReceipt identity_receipt{
      identity_stop.command_id, v2::ReceiptPhase::kAccepted,
      v2::ReceiptStatus::kOk, kRobotBase + 100000000ULL, "accepted"};

  v2::StopReceiptCorrelator wrong_epoch(kEpoch, "boot-a");
  CHECK(wrong_epoch.begin(identity_stop, kGroundBase).ok());
  v2::ReceiveGuard wrong_epoch_guard(Expected(kEpoch + 1U),
                                     v2::Channel::kTelemetry,
                                     Timebase(kRobotBase, kGroundBase));
  v2::Admission wrong_identity_admission;
  CHECK(
      wrong_epoch_guard
          .accept(HelloOn(v2::Channel::kTelemetry, 1U, kRobotBase, kEpoch + 1U),
                  kGroundBase, &wrong_identity_admission)
          .ok());
  CHECK(wrong_epoch_guard
            .accept(MakeReceiptFrame(2U, kRobotBase + 100000000ULL, kEpoch + 1U,
                                     identity_receipt),
                    kGroundBase + 100000000ULL, &wrong_identity_admission)
            .ok());
  CHECK_ERROR(wrong_epoch.observe(identity_receipt, wrong_identity_admission,
                                  kGroundBase + 100000001ULL),
              v2::ErrorCode::kEpochMismatch);
  CHECK(wrong_epoch.progress() == v2::ReceiptProgress::kRejected);

  v2::StopReceiptCorrelator wrong_boot(kEpoch, "boot-a");
  CHECK(wrong_boot.begin(identity_stop, kGroundBase).ok());
  v2::ExpectedPeer boot_b_expected = Expected(kEpoch);
  boot_b_expected.boot_id = "boot-b";
  v2::ReceiveGuard wrong_boot_guard(boot_b_expected, v2::Channel::kTelemetry,
                                    Timebase(kRobotBase, kGroundBase));
  CHECK(wrong_boot_guard
            .accept(HelloOn(v2::Channel::kTelemetry, 1U, kRobotBase, kEpoch,
                            "boot-b"),
                    kGroundBase, &wrong_identity_admission)
            .ok());
  CHECK(wrong_boot_guard
            .accept(MakeReceiptFrame(2U, kRobotBase + 100000000ULL, kEpoch,
                                     identity_receipt, "boot-b"),
                    kGroundBase + 100000000ULL, &wrong_identity_admission)
            .ok());
  CHECK_ERROR(wrong_boot.observe(identity_receipt, wrong_identity_admission,
                                 kGroundBase + 100000001ULL),
              v2::ErrorCode::kBootMismatch);
  CHECK(wrong_boot.progress() == v2::ReceiptProgress::kRejected);

  v2::StopReceiptCorrelator stale_receipt(kEpoch, "boot-a");
  CHECK(stale_receipt.begin(identity_stop, kGroundBase).ok());
  v2::ReceiveGuard stale_receipt_guard(Expected(kEpoch),
                                       v2::Channel::kTelemetry,
                                       Timebase(kRobotBase, kGroundBase));
  CHECK(stale_receipt_guard
            .accept(HelloOn(v2::Channel::kTelemetry, 1U, kRobotBase, kEpoch),
                    kGroundBase, &wrong_identity_admission)
            .ok());
  CHECK(stale_receipt_guard
            .accept(MakeReceiptFrame(2U, kRobotBase + 100000000ULL, kEpoch,
                                     identity_receipt),
                    kGroundBase + 100000000ULL, &wrong_identity_admission)
            .ok());
  CHECK_ERROR(stale_receipt.observe(identity_receipt, wrong_identity_admission,
                                    kGroundBase + 850000001ULL),
              v2::ErrorCode::kStale);
  CHECK(stale_receipt.progress() == v2::ReceiptProgress::kAwaitAccepted);
}

void EmitGolden() {
  std::vector<std::uint8_t> hello;
  std::vector<std::uint8_t> heartbeat;
  std::vector<std::uint8_t> stop;
  v2::Frame heartbeat_frame = MakeHeartbeatFrame(
      0x0203040506070809ULL, 0x1112131415161720ULL, 0x2122232425262728ULL);
  SetHeartbeatAck(&heartbeat_frame, 0x0102030405060708ULL);
  const v2::Error hello_error = v2::encodeFrame(MakeHelloFrame(), &hello);
  const v2::Error heartbeat_error =
      v2::encodeFrame(heartbeat_frame, &heartbeat);
  const v2::Error stop_error = v2::encodeFrame(MakeStopFrame(), &stop);
  if (!hello_error.ok() || !heartbeat_error.ok() || !stop_error.ok()) {
    std::cerr << "failed to build golden vectors\n";
    std::exit(2);
  }
  std::cout << "hello " << Hex(hello) << '\n';
  std::cout << "heartbeat " << Hex(heartbeat) << '\n';
  std::cout << "stop " << Hex(stop) << '\n';
}

} // namespace

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--emit-golden") {
    EmitGolden();
    return 0;
  }
  if (argc != 1) {
    std::cerr << "usage: v2_protocol_test [--emit-golden]\n";
    return 2;
  }

  TestCrcAndGoldenRoundTrips();
  TestEnvelopeNegativeCases();
  TestDeterministicMutationCorpus();
  TestZeroOnlyStopAndReceipts();
  TestCapabilityRoleContract();
  TestPerChannelSendWindowAndAckReadiness();
  TestReceiveGuardIdentityReplayEpochAndFreshness();
  TestWatchdogCompleteTransitions();
  TestStopReceiptCorrelator();

  if (failures != 0) {
    std::cerr << failures << " v2 protocol checks failed\n";
    return 1;
  }
  std::cout << "swarm_ros_bridge v2 protocol checks passed\n";
  return 0;
}
