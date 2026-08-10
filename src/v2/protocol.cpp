// SPDX-License-Identifier: BSD-3-Clause

#include "swarm_ros_bridge/v2/protocol.hpp"

#include <algorithm>
#include <cstring>
#include <limits>
#include <utility>

namespace xgc2 {
namespace swarm_bridge {
namespace v2 {
namespace {

static_assert(std::numeric_limits<double>::is_iec559 &&
                  std::numeric_limits<double>::digits == 53,
              "v2 ZERO_STOP requires IEEE-754 binary64 doubles");

Error MakeError(ErrorCode code, const std::string &detail) {
  return Error{code, detail};
}

void AppendU8(std::vector<std::uint8_t> *output, std::uint8_t value) {
  output->push_back(value);
}

void AppendU16(std::vector<std::uint8_t> *output, std::uint16_t value) {
  const std::uint32_t widened = static_cast<std::uint32_t>(value);
  output->push_back(static_cast<std::uint8_t>((widened >> 8U) & 0xffU));
  output->push_back(static_cast<std::uint8_t>(widened & 0xffU));
}

void AppendU32(std::vector<std::uint8_t> *output, std::uint32_t value) {
  output->push_back(static_cast<std::uint8_t>((value >> 24U) & 0xffU));
  output->push_back(static_cast<std::uint8_t>((value >> 16U) & 0xffU));
  output->push_back(static_cast<std::uint8_t>((value >> 8U) & 0xffU));
  output->push_back(static_cast<std::uint8_t>(value & 0xffU));
}

void AppendU64(std::vector<std::uint8_t> *output, std::uint64_t value) {
  for (int shift = 56; shift >= 0; shift -= 8) {
    output->push_back(static_cast<std::uint8_t>(
        (value >> static_cast<unsigned>(shift)) & 0xffU));
  }
}

void AppendString(std::vector<std::uint8_t> *output, const std::string &value) {
  output->insert(output->end(), value.begin(), value.end());
}

class Reader {
public:
  Reader(const std::uint8_t *data, std::size_t size)
      : data_(data), size_(size) {}

  bool ReadU8(std::uint8_t *output) {
    if (output == nullptr || remaining() < 1U) {
      return false;
    }
    *output = data_[offset_++];
    return true;
  }

  bool ReadU16(std::uint16_t *output) {
    if (output == nullptr || remaining() < 2U) {
      return false;
    }
    const std::uint32_t value =
        (static_cast<std::uint32_t>(data_[offset_]) << 8U) |
        static_cast<std::uint32_t>(data_[offset_ + 1U]);
    *output = static_cast<std::uint16_t>(value);
    offset_ += 2U;
    return true;
  }

  bool ReadU32(std::uint32_t *output) {
    if (output == nullptr || remaining() < 4U) {
      return false;
    }
    *output = (static_cast<std::uint32_t>(data_[offset_]) << 24U) |
              (static_cast<std::uint32_t>(data_[offset_ + 1U]) << 16U) |
              (static_cast<std::uint32_t>(data_[offset_ + 2U]) << 8U) |
              static_cast<std::uint32_t>(data_[offset_ + 3U]);
    offset_ += 4U;
    return true;
  }

  bool ReadU64(std::uint64_t *output) {
    if (output == nullptr || remaining() < 8U) {
      return false;
    }
    std::uint64_t value = 0U;
    for (std::size_t index = 0U; index < 8U; ++index) {
      value =
          (value << 8U) | static_cast<std::uint64_t>(data_[offset_ + index]);
    }
    offset_ += 8U;
    *output = value;
    return true;
  }

  bool ReadString(std::size_t length, std::string *output) {
    if (output == nullptr || length > remaining()) {
      return false;
    }
    output->assign(reinterpret_cast<const char *>(data_ + offset_), length);
    offset_ += length;
    return true;
  }

  bool ReadBytes(std::size_t length, std::vector<std::uint8_t> *output) {
    if (output == nullptr || length > remaining()) {
      return false;
    }
    output->assign(data_ + offset_, data_ + offset_ + length);
    offset_ += length;
    return true;
  }

  std::size_t offset() const { return offset_; }
  std::size_t remaining() const { return size_ - offset_; }

private:
  const std::uint8_t *data_;
  std::size_t size_;
  std::size_t offset_ = 0U;
};

bool IsKnownChannel(Channel channel) {
  switch (channel) {
  case Channel::kManagement:
  case Channel::kControl:
  case Channel::kTelemetry:
    return true;
  }
  return false;
}

bool IsKnownRole(PeerRole role) {
  return role == PeerRole::kGround || role == PeerRole::kVehicle;
}

Error InferRole(std::uint64_t capabilities, PeerRole *role) {
  if (role == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "peer role output is null");
  }
  const std::uint64_t role_bits =
      capabilities & (kCapabilityRoleGround | kCapabilityRoleVehicle);
  if (role_bits == kCapabilityRoleGround) {
    *role = PeerRole::kGround;
    return Error{};
  }
  if (role_bits == kCapabilityRoleVehicle) {
    *role = PeerRole::kVehicle;
    return Error{};
  }
  return MakeError(ErrorCode::kCapabilityMismatch,
                   "capabilities require exactly one role bit");
}

bool IsKnownKind(MessageKind kind) {
  switch (kind) {
  case MessageKind::kHello:
  case MessageKind::kHeartbeat:
  case MessageKind::kZeroStop:
  case MessageKind::kStopReceipt:
  case MessageKind::kRosMessage:
    return true;
  }
  return false;
}

bool IsKnownReceiptPhase(ReceiptPhase phase) {
  return phase == ReceiptPhase::kAccepted || phase == ReceiptPhase::kApplied;
}

bool IsKnownReceiptStatus(ReceiptStatus status) {
  return status == ReceiptStatus::kOk || status == ReceiptStatus::kRejected;
}

Error ValidateBoundedToken(const std::string &value, std::size_t maximum,
                           bool allow_empty, const char *name) {
  if (!allow_empty && value.empty()) {
    return MakeError(ErrorCode::kInvalidIdentity,
                     std::string(name) + " must not be empty");
  }
  if (value.size() > maximum) {
    return MakeError(ErrorCode::kLimitExceeded,
                     std::string(name) + " exceeds its byte limit");
  }
  for (char raw_character : value) {
    const unsigned char character = static_cast<unsigned char>(raw_character);
    if (character < 0x21U || character > 0x7eU) {
      return MakeError(ErrorCode::kInvalidIdentity,
                       std::string(name) +
                           " must contain printable non-space ASCII only");
    }
  }
  return Error{};
}

Error ValidateMd5(const std::string &value) {
  if (value.size() != kMaxRosMd5Bytes) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "ROS MD5 must contain exactly 32 lowercase hex bytes");
  }
  for (char character : value) {
    const bool decimal = character >= '0' && character <= '9';
    const bool lower_hex = character >= 'a' && character <= 'f';
    if (!decimal && !lower_hex) {
      return MakeError(ErrorCode::kInvalidMetadata,
                       "ROS MD5 must contain exactly 32 lowercase hex bytes");
    }
  }
  return Error{};
}

bool IsPositiveZero(double value) {
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
  std::memcpy(&bits, &value, sizeof(bits));
  return bits == 0U;
}

void AppendDouble(std::vector<std::uint8_t> *output, double value) {
  std::uint64_t bits = 0U;
  std::memcpy(&bits, &value, sizeof(bits));
  AppendU64(output, bits);
}

bool ReadDouble(Reader *reader, double *output) {
  if (reader == nullptr || output == nullptr) {
    return false;
  }
  std::uint64_t bits = 0U;
  if (!reader->ReadU64(&bits)) {
    return false;
  }
  std::memcpy(output, &bits, sizeof(bits));
  return true;
}

Error ValidateChannelAndKind(const FrameHeader &header) {
  if (!IsKnownChannel(header.channel) || !IsKnownKind(header.kind)) {
    return MakeError(ErrorCode::kInvalidEnum,
                     "channel or message kind is not defined by v2");
  }
  switch (header.kind) {
  case MessageKind::kHello:
  case MessageKind::kHeartbeat:
    // Every physical channel owns an independent HELLO, sequence window, and
    // heartbeat. This prevents one live management socket from masking a dead
    // control or telemetry socket.
    break;
  case MessageKind::kZeroStop:
    if (header.channel != Channel::kControl) {
      return MakeError(ErrorCode::kInvalidMetadata,
                       "ZERO_STOP requires control channel");
    }
    break;
  case MessageKind::kStopReceipt:
    if (header.channel != Channel::kTelemetry) {
      return MakeError(ErrorCode::kInvalidMetadata,
                       "STOP_RECEIPT requires telemetry channel");
    }
    break;
  case MessageKind::kRosMessage:
    if (header.channel != Channel::kControl &&
        header.channel != Channel::kTelemetry) {
      return MakeError(ErrorCode::kInvalidMetadata,
                       "ROS_MESSAGE requires control or telemetry channel");
    }
    break;
  }
  return Error{};
}

Error ValidatePayloadMetadata(const Frame &frame) {
  const FrameHeader &header = frame.header;
  const bool is_ros = header.kind == MessageKind::kRosMessage;
  if (is_ros) {
    Error error = ValidateBoundedToken(
        header.ros_datatype, kMaxRosDatatypeBytes, false, "ros_datatype");
    if (!error.ok()) {
      return error;
    }
    error = ValidateMd5(header.ros_md5);
    if (!error.ok()) {
      return error;
    }
    if (header.source_timestamp_ns == 0U) {
      return MakeError(ErrorCode::kInvalidMetadata,
                       "ROS_MESSAGE requires a non-zero source timestamp");
    }
  } else if (!header.ros_datatype.empty() || !header.ros_md5.empty()) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "non-ROS control payload must not claim ROS type or MD5");
  }

  const char *required_schema = nullptr;
  switch (header.kind) {
  case MessageKind::kHello:
    required_schema = kHelloSchema;
    break;
  case MessageKind::kHeartbeat:
    required_schema = kHeartbeatSchema;
    break;
  case MessageKind::kZeroStop:
    required_schema = kZeroStopSchema;
    break;
  case MessageKind::kStopReceipt:
    required_schema = kStopReceiptSchema;
    break;
  case MessageKind::kRosMessage:
    break;
  }
  if (required_schema != nullptr && header.schema != required_schema) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "message kind does not match its frozen payload schema");
  }
  return ValidateBoundedToken(header.schema, kMaxSchemaBytes, false, "schema");
}

Error ValidateTypedPayload(const Frame &frame) {
  switch (frame.header.kind) {
  case MessageKind::kHello: {
    Hello hello;
    return decodeHelloPayload(frame.payload, &hello);
  }
  case MessageKind::kHeartbeat: {
    Heartbeat heartbeat;
    return decodeHeartbeatPayload(frame.payload, &heartbeat);
  }
  case MessageKind::kZeroStop: {
    ZeroStop stop;
    return decodeZeroStopPayload(frame.payload, &stop);
  }
  case MessageKind::kStopReceipt: {
    StopReceipt receipt;
    return decodeStopReceiptPayload(frame.payload, &receipt);
  }
  case MessageKind::kRosMessage:
    return Error{};
  }
  return MakeError(ErrorCode::kInvalidEnum, "unknown message kind");
}

Error CheckIdentityField(const std::string &actual, const std::string &expected,
                         const char *name) {
  if (actual != expected) {
    return MakeError(ErrorCode::kIdentityMismatch,
                     std::string(name) + " does not match expected peer");
  }
  return Error{};
}

} // namespace

const char *errorCodeName(ErrorCode code) {
  switch (code) {
  case ErrorCode::kNone:
    return "none";
  case ErrorCode::kNullOutput:
    return "null_output";
  case ErrorCode::kFrameTooLarge:
    return "frame_too_large";
  case ErrorCode::kTruncated:
    return "truncated";
  case ErrorCode::kBadMagic:
    return "bad_magic";
  case ErrorCode::kUnsupportedVersion:
    return "unsupported_version";
  case ErrorCode::kInvalidEnum:
    return "invalid_enum";
  case ErrorCode::kInvalidFlags:
    return "invalid_flags";
  case ErrorCode::kInvalidLength:
    return "invalid_length";
  case ErrorCode::kLimitExceeded:
    return "limit_exceeded";
  case ErrorCode::kIntegrityMismatch:
    return "integrity_mismatch";
  case ErrorCode::kInvalidIdentity:
    return "invalid_identity";
  case ErrorCode::kInvalidMetadata:
    return "invalid_metadata";
  case ErrorCode::kInvalidPayload:
    return "invalid_payload";
  case ErrorCode::kIdentityMismatch:
    return "identity_mismatch";
  case ErrorCode::kChannelMismatch:
    return "channel_mismatch";
  case ErrorCode::kCapabilityMismatch:
    return "capability_mismatch";
  case ErrorCode::kEpochMismatch:
    return "epoch_mismatch";
  case ErrorCode::kBootMismatch:
    return "boot_mismatch";
  case ErrorCode::kReplay:
    return "replay";
  case ErrorCode::kSequenceGap:
    return "sequence_gap";
  case ErrorCode::kOutOfOrderTimestamp:
    return "out_of_order_timestamp";
  case ErrorCode::kStale:
    return "stale";
  case ErrorCode::kFutureTimestamp:
    return "future_timestamp";
  case ErrorCode::kHandshakeRequired:
    return "handshake_required";
  case ErrorCode::kHeartbeatTimeout:
    return "heartbeat_timeout";
  case ErrorCode::kUnsafeNonZero:
    return "unsafe_non_zero";
  case ErrorCode::kDeadlineExpired:
    return "deadline_expired";
  case ErrorCode::kDeadlineTooFar:
    return "deadline_too_far";
  case ErrorCode::kSendWindowFull:
    return "send_window_full";
  case ErrorCode::kAckFuture:
    return "ack_future";
  case ErrorCode::kAckRegression:
    return "ack_regression";
  case ErrorCode::kAckOutOfWindow:
    return "ack_out_of_window";
  case ErrorCode::kInvalidState:
    return "invalid_state";
  }
  return "unknown";
}

Error validateRoleCapabilities(PeerRole role, std::uint64_t capabilities) {
  if (!IsKnownRole(role)) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "peer role is not ground or vehicle");
  }
  if (capabilities == 0U) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "zero capability bitmap is forbidden");
  }
  if ((capabilities & ~kKnownCapabilities) != 0U) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "capability bitmap contains unknown bits");
  }

  PeerRole encoded_role = PeerRole::kUnspecified;
  Error error = InferRole(capabilities, &encoded_role);
  if (!error.ok()) {
    return error;
  }
  if (encoded_role != role) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "capability role bit does not match configured role");
  }

  const std::uint64_t required = role == PeerRole::kGround
                                     ? kGroundRequiredCapabilities
                                     : kVehicleRequiredCapabilities;
  const std::uint64_t allowed = role == PeerRole::kGround
                                    ? kGroundAllowedCapabilities
                                    : kVehicleAllowedCapabilities;
  if ((capabilities & required) != required) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "capability bitmap is a forbidden role downgrade");
  }
  if ((capabilities & ~allowed) != 0U) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "capability bitmap is not allowed for its role");
  }
  return Error{};
}

Error validatePeerCompatibility(PeerRole local_role,
                                std::uint64_t local_capabilities,
                                PeerRole peer_role,
                                std::uint64_t peer_capabilities) {
  Error error = validateRoleCapabilities(local_role, local_capabilities);
  if (!error.ok()) {
    return error;
  }
  error = validateRoleCapabilities(peer_role, peer_capabilities);
  if (!error.ok()) {
    return error;
  }
  const bool complementary =
      (local_role == PeerRole::kGround && peer_role == PeerRole::kVehicle) ||
      (local_role == PeerRole::kVehicle && peer_role == PeerRole::kGround);
  if (!complementary) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "v2 requires one ground peer and one vehicle peer");
  }
  const std::uint64_t shared_required =
      kCapabilityThreeChannel | kCapabilityPerChannelAck;
  if ((local_capabilities & peer_capabilities & shared_required) !=
      shared_required) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "peers lack shared three-channel ACK capabilities");
  }
  return Error{};
}

std::uint32_t crc32c(const std::uint8_t *data, std::size_t size) {
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t index = 0U; index < size; ++index) {
    crc ^= static_cast<std::uint32_t>(data[index]);
    for (int bit = 0; bit < 8; ++bit) {
      const std::uint32_t mask = static_cast<std::uint32_t>(
          -static_cast<std::int32_t>(crc & static_cast<std::uint32_t>(1U)));
      crc = (crc >> 1U) ^ (0x82f63b78U & mask);
    }
  }
  return ~crc;
}

Error validateFrame(const Frame &frame) {
  const FrameHeader &header = frame.header;
  if (header.flags != 0U) {
    return MakeError(ErrorCode::kInvalidFlags,
                     "v2 reserves all frame flags as zero");
  }
  if (header.sequence == 0U || header.monotonic_ns == 0U ||
      header.session_epoch == 0U) {
    return MakeError(
        ErrorCode::kInvalidMetadata,
        "sequence, monotonic timestamp, and epoch must be positive");
  }
  PeerRole encoded_role = PeerRole::kUnspecified;
  Error error = InferRole(header.capabilities, &encoded_role);
  if (!error.ok()) {
    return error;
  }
  error = validateRoleCapabilities(encoded_role, header.capabilities);
  if (!error.ok()) {
    return error;
  }
  if (header.kind == MessageKind::kZeroStop &&
      encoded_role != PeerRole::kGround) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "only the ground role may originate ZERO_STOP");
  }
  if (header.kind == MessageKind::kStopReceipt &&
      encoded_role != PeerRole::kVehicle) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "only the vehicle role may originate STOP_RECEIPT");
  }
  if (frame.payload.size() > kMaxPayloadBytes) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "payload exceeds the v2 payload limit");
  }

  error = ValidateChannelAndKind(header);
  if (!error.ok()) {
    return error;
  }
  const struct {
    const std::string *value;
    std::size_t limit;
    const char *name;
  } identity_fields[] = {
      {&header.slot_id, kMaxSlotIdBytes, "slot_id"},
      {&header.asset_id, kMaxAssetIdBytes, "asset_id"},
      {&header.robot_kind, kMaxRobotKindBytes, "robot_kind"},
      {&header.run_id, kMaxRunIdBytes, "run_id"},
      {&header.boot_id, kMaxBootIdBytes, "boot_id"},
      {&header.build_id, kMaxBuildIdBytes, "build_id"},
  };
  for (const auto &field : identity_fields) {
    error = ValidateBoundedToken(*field.value, field.limit, false, field.name);
    if (!error.ok()) {
      return error;
    }
  }
  error = ValidatePayloadMetadata(frame);
  if (!error.ok()) {
    return error;
  }

  const std::size_t variable_header_bytes =
      header.slot_id.size() + header.asset_id.size() +
      header.robot_kind.size() + header.run_id.size() + header.boot_id.size() +
      header.build_id.size() + header.ros_datatype.size() +
      header.ros_md5.size() + header.schema.size();
  if (variable_header_bytes > kMaxHeaderBytes - kFixedHeaderBytes) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "encoded metadata exceeds the v2 header limit");
  }
  if (kFixedHeaderBytes + variable_header_bytes + frame.payload.size() +
          kIntegrityBytes >
      kMaxFrameBytes) {
    return MakeError(ErrorCode::kFrameTooLarge,
                     "encoded frame exceeds the v2 frame limit");
  }
  return ValidateTypedPayload(frame);
}

Error encodeFrame(const Frame &frame, std::vector<std::uint8_t> *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "encode output is null");
  }
  Error error = validateFrame(frame);
  if (!error.ok()) {
    return error;
  }

  const FrameHeader &header = frame.header;
  const std::string *strings[] = {
      &header.slot_id,      &header.asset_id, &header.robot_kind,
      &header.run_id,       &header.boot_id,  &header.build_id,
      &header.ros_datatype, &header.ros_md5,  &header.schema,
  };
  std::size_t header_size = kFixedHeaderBytes;
  for (const std::string *value : strings) {
    header_size += value->size();
  }

  output->clear();
  output->reserve(header_size + frame.payload.size() + kIntegrityBytes);
  AppendU32(output, kWireMagic);
  AppendU8(output, kWireVersion);
  AppendU8(output, static_cast<std::uint8_t>(header.channel));
  AppendU8(output, static_cast<std::uint8_t>(header.kind));
  AppendU8(output, header.flags);
  AppendU16(output, static_cast<std::uint16_t>(header_size));
  AppendU16(output, 0U);
  AppendU32(output, static_cast<std::uint32_t>(frame.payload.size()));
  AppendU64(output, header.sequence);
  AppendU64(output, header.monotonic_ns);
  AppendU64(output, header.source_timestamp_ns);
  AppendU64(output, header.session_epoch);
  AppendU64(output, header.capabilities);
  for (const std::string *value : strings) {
    AppendU16(output, static_cast<std::uint16_t>(value->size()));
  }
  for (const std::string *value : strings) {
    AppendString(output, *value);
  }
  output->insert(output->end(), frame.payload.begin(), frame.payload.end());
  AppendU32(output, crc32c(output->data(), output->size()));
  return Error{};
}

Error decodeFrame(const std::vector<std::uint8_t> &wire, Frame *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "decode output is null");
  }
  if (wire.size() > kMaxFrameBytes) {
    return MakeError(ErrorCode::kFrameTooLarge,
                     "wire frame exceeds the v2 frame limit");
  }
  if (wire.size() < kFixedHeaderBytes + kIntegrityBytes) {
    return MakeError(ErrorCode::kTruncated,
                     "wire frame is shorter than the fixed envelope");
  }

  Reader reader(wire.data(), wire.size());
  std::uint32_t magic = 0U;
  std::uint8_t version = 0U;
  std::uint8_t channel = 0U;
  std::uint8_t kind = 0U;
  std::uint8_t flags = 0U;
  std::uint16_t header_size = 0U;
  std::uint16_t reserved = 0U;
  std::uint32_t payload_size = 0U;
  if (!reader.ReadU32(&magic) || !reader.ReadU8(&version) ||
      !reader.ReadU8(&channel) || !reader.ReadU8(&kind) ||
      !reader.ReadU8(&flags) || !reader.ReadU16(&header_size) ||
      !reader.ReadU16(&reserved) || !reader.ReadU32(&payload_size)) {
    return MakeError(ErrorCode::kTruncated, "fixed envelope is truncated");
  }
  if (magic != kWireMagic) {
    return MakeError(ErrorCode::kBadMagic, "wire magic is not XSB2");
  }
  if (version != kWireVersion) {
    return MakeError(ErrorCode::kUnsupportedVersion, "wire version is not v2");
  }
  if (flags != 0U || reserved != 0U) {
    return MakeError(ErrorCode::kInvalidFlags,
                     "reserved v2 envelope fields must be zero");
  }
  if (header_size < kFixedHeaderBytes || header_size > kMaxHeaderBytes) {
    return MakeError(ErrorCode::kInvalidLength,
                     "header length is outside the v2 bounds");
  }
  if (payload_size > kMaxPayloadBytes) {
    return MakeError(ErrorCode::kLimitExceeded,
                     "payload length exceeds the v2 limit");
  }
  const std::size_t expected_size = static_cast<std::size_t>(header_size) +
                                    static_cast<std::size_t>(payload_size) +
                                    kIntegrityBytes;
  if (expected_size != wire.size()) {
    return MakeError(ErrorCode::kInvalidLength,
                     "wire length is not exactly header + payload + CRC32C");
  }
  const std::size_t crc_offset = wire.size() - kIntegrityBytes;
  Reader crc_reader(wire.data() + crc_offset, kIntegrityBytes);
  std::uint32_t wire_crc = 0U;
  if (!crc_reader.ReadU32(&wire_crc) ||
      wire_crc != crc32c(wire.data(), crc_offset)) {
    return MakeError(ErrorCode::kIntegrityMismatch,
                     "CRC32C does not match the frame bytes");
  }

  Frame candidate;
  candidate.header.channel = static_cast<Channel>(channel);
  candidate.header.kind = static_cast<MessageKind>(kind);
  candidate.header.flags = flags;
  if (!reader.ReadU64(&candidate.header.sequence) ||
      !reader.ReadU64(&candidate.header.monotonic_ns) ||
      !reader.ReadU64(&candidate.header.source_timestamp_ns) ||
      !reader.ReadU64(&candidate.header.session_epoch) ||
      !reader.ReadU64(&candidate.header.capabilities)) {
    return MakeError(ErrorCode::kTruncated,
                     "fixed v2 identity fields are truncated");
  }

  std::array<std::uint16_t, 9U> lengths{};
  for (std::uint16_t &length : lengths) {
    if (!reader.ReadU16(&length)) {
      return MakeError(ErrorCode::kTruncated,
                       "v2 string lengths are truncated");
    }
  }
  std::size_t encoded_header_size = kFixedHeaderBytes;
  for (std::uint16_t length : lengths) {
    encoded_header_size += static_cast<std::size_t>(length);
  }
  if (encoded_header_size != header_size) {
    return MakeError(ErrorCode::kInvalidLength,
                     "header length does not match its string lengths");
  }

  std::string *strings[] = {
      &candidate.header.slot_id,      &candidate.header.asset_id,
      &candidate.header.robot_kind,   &candidate.header.run_id,
      &candidate.header.boot_id,      &candidate.header.build_id,
      &candidate.header.ros_datatype, &candidate.header.ros_md5,
      &candidate.header.schema,
  };
  for (std::size_t index = 0U; index < lengths.size(); ++index) {
    if (!reader.ReadString(lengths[index], strings[index])) {
      return MakeError(ErrorCode::kTruncated,
                       "v2 identity metadata is truncated");
    }
  }
  if (reader.offset() != header_size ||
      !reader.ReadBytes(payload_size, &candidate.payload) ||
      reader.offset() != crc_offset) {
    return MakeError(ErrorCode::kInvalidLength,
                     "v2 header or payload boundary is inconsistent");
  }
  Error error = validateFrame(candidate);
  if (!error.ok()) {
    return error;
  }
  *output = std::move(candidate);
  return Error{};
}

bool Admission::matchesCanonicalFrame(const Frame &candidate) const {
  const FrameHeader &header = candidate.header;
  const FrameHeader &canonical = canonical_header_;
  return valid_ && header.channel == canonical.channel &&
         header.kind == canonical.kind && header.flags == canonical.flags &&
         header.sequence == canonical.sequence &&
         header.monotonic_ns == canonical.monotonic_ns &&
         header.source_timestamp_ns == canonical.source_timestamp_ns &&
         header.session_epoch == canonical.session_epoch &&
         header.capabilities == canonical.capabilities &&
         header.slot_id == canonical.slot_id &&
         header.asset_id == canonical.asset_id &&
         header.robot_kind == canonical.robot_kind &&
         header.run_id == canonical.run_id &&
         header.boot_id == canonical.boot_id &&
         header.build_id == canonical.build_id &&
         header.ros_datatype == canonical.ros_datatype &&
         header.ros_md5 == canonical.ros_md5 &&
         header.schema == canonical.schema &&
         candidate.payload == canonical_payload_;
}

Error encodeHelloPayload(const Hello &hello,
                         std::vector<std::uint8_t> *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "HELLO output is null");
  }
  if (hello.heartbeat_period_ms != kHeartbeatPeriodMs ||
      hello.heartbeat_timeout_ms != kHeartbeatTimeoutMs) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "HELLO timing must be the frozen 200/750 ms contract");
  }
  output->clear();
  output->reserve(8U);
  AppendU16(output, hello.heartbeat_period_ms);
  AppendU16(output, hello.heartbeat_timeout_ms);
  AppendU32(output, 0U);
  return Error{};
}

Error decodeHelloPayload(const std::vector<std::uint8_t> &payload,
                         Hello *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "HELLO output is null");
  }
  if (payload.size() != 8U) {
    return MakeError(ErrorCode::kInvalidLength,
                     "HELLO payload must be exactly 8 bytes");
  }
  Reader reader(payload.data(), payload.size());
  Hello candidate;
  std::uint32_t reserved = 0U;
  if (!reader.ReadU16(&candidate.heartbeat_period_ms) ||
      !reader.ReadU16(&candidate.heartbeat_timeout_ms) ||
      !reader.ReadU32(&reserved)) {
    return MakeError(ErrorCode::kTruncated, "HELLO payload is truncated");
  }
  if (candidate.heartbeat_period_ms != kHeartbeatPeriodMs ||
      candidate.heartbeat_timeout_ms != kHeartbeatTimeoutMs || reserved != 0U) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "HELLO timing/reserved fields violate v2 contract");
  }
  *output = candidate;
  return Error{};
}

Error encodeHeartbeatPayload(const Heartbeat &heartbeat,
                             std::vector<std::uint8_t> *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "HEARTBEAT output is null");
  }
  if (heartbeat.safety_state > 3U) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "HEARTBEAT safety state is outside the frozen range");
  }
  output->clear();
  output->reserve(16U);
  AppendU64(output, heartbeat.last_received_sequence);
  AppendU8(output, heartbeat.safety_state);
  for (int index = 0; index < 7; ++index) {
    AppendU8(output, 0U);
  }
  return Error{};
}

Error decodeHeartbeatPayload(const std::vector<std::uint8_t> &payload,
                             Heartbeat *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "HEARTBEAT output is null");
  }
  if (payload.size() != 16U) {
    return MakeError(ErrorCode::kInvalidLength,
                     "HEARTBEAT payload must be exactly 16 bytes");
  }
  Reader reader(payload.data(), payload.size());
  Heartbeat candidate;
  if (!reader.ReadU64(&candidate.last_received_sequence) ||
      !reader.ReadU8(&candidate.safety_state)) {
    return MakeError(ErrorCode::kTruncated, "HEARTBEAT payload is truncated");
  }
  for (int index = 0; index < 7; ++index) {
    std::uint8_t reserved = 0U;
    if (!reader.ReadU8(&reserved) || reserved != 0U) {
      return MakeError(ErrorCode::kInvalidPayload,
                       "HEARTBEAT reserved bytes must be zero");
    }
  }
  if (candidate.safety_state > 3U) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "HEARTBEAT safety state is outside the frozen range");
  }
  *output = candidate;
  return Error{};
}

Error encodeZeroStopPayload(const ZeroStop &stop,
                            std::vector<std::uint8_t> *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "ZERO_STOP output is null");
  }
  Error error = ValidateBoundedToken(stop.command_id, kMaxCommandIdBytes, false,
                                     "command_id");
  if (!error.ok()) {
    return error;
  }
  if (stop.deadline_monotonic_ns == 0U) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "ZERO_STOP deadline must be positive");
  }
  for (double axis : stop.axes) {
    if (!IsPositiveZero(axis)) {
      return MakeError(ErrorCode::kUnsafeNonZero,
                       "all six ZERO_STOP axes must be bit-exact +0.0");
    }
  }
  output->clear();
  output->reserve(58U + stop.command_id.size());
  AppendU64(output, stop.deadline_monotonic_ns);
  AppendU16(output, static_cast<std::uint16_t>(stop.command_id.size()));
  AppendString(output, stop.command_id);
  for (double axis : stop.axes) {
    AppendDouble(output, axis);
  }
  return Error{};
}

Error decodeZeroStopPayload(const std::vector<std::uint8_t> &payload,
                            ZeroStop *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "ZERO_STOP output is null");
  }
  if (payload.size() < 58U || payload.size() > 58U + kMaxCommandIdBytes) {
    return MakeError(ErrorCode::kInvalidLength,
                     "ZERO_STOP payload length is outside its bounds");
  }
  Reader reader(payload.data(), payload.size());
  ZeroStop candidate;
  std::uint16_t command_length = 0U;
  if (!reader.ReadU64(&candidate.deadline_monotonic_ns) ||
      !reader.ReadU16(&command_length)) {
    return MakeError(ErrorCode::kTruncated,
                     "ZERO_STOP fixed payload is truncated");
  }
  if (command_length == 0U || command_length > kMaxCommandIdBytes ||
      payload.size() != 58U + command_length ||
      !reader.ReadString(command_length, &candidate.command_id)) {
    return MakeError(ErrorCode::kInvalidLength,
                     "ZERO_STOP command length is inconsistent");
  }
  Error error = ValidateBoundedToken(candidate.command_id, kMaxCommandIdBytes,
                                     false, "command_id");
  if (!error.ok()) {
    return error;
  }
  if (candidate.deadline_monotonic_ns == 0U) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "ZERO_STOP deadline must be positive");
  }
  for (double &axis : candidate.axes) {
    if (!ReadDouble(&reader, &axis)) {
      return MakeError(ErrorCode::kTruncated,
                       "ZERO_STOP axis payload is truncated");
    }
    if (!IsPositiveZero(axis)) {
      return MakeError(ErrorCode::kUnsafeNonZero,
                       "all six ZERO_STOP axes must be bit-exact +0.0");
    }
  }
  if (reader.remaining() != 0U) {
    return MakeError(ErrorCode::kInvalidLength,
                     "ZERO_STOP contains trailing bytes");
  }
  *output = candidate;
  return Error{};
}

Error validateStopDeadline(const ZeroStop &stop,
                           std::uint64_t sender_monotonic_ns) {
  if (sender_monotonic_ns == 0U) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "sender monotonic timestamp must be positive");
  }
  if (stop.deadline_monotonic_ns < sender_monotonic_ns) {
    return MakeError(ErrorCode::kDeadlineExpired,
                     "ZERO_STOP deadline has already expired");
  }
  if (stop.deadline_monotonic_ns - sender_monotonic_ns >
      kMaximumStopDeadlineHorizonNs) {
    return MakeError(ErrorCode::kDeadlineTooFar,
                     "ZERO_STOP deadline exceeds the one-second horizon");
  }
  return Error{};
}

Error encodeStopReceiptPayload(const StopReceipt &receipt,
                               std::vector<std::uint8_t> *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "STOP_RECEIPT output is null");
  }
  Error error = ValidateBoundedToken(receipt.command_id, kMaxCommandIdBytes,
                                     false, "command_id");
  if (!error.ok()) {
    return error;
  }
  error = ValidateBoundedToken(receipt.detail, kMaxReceiptDetailBytes, true,
                               "receipt detail");
  if (!error.ok()) {
    return error;
  }
  if (!IsKnownReceiptPhase(receipt.phase) ||
      !IsKnownReceiptStatus(receipt.status) ||
      receipt.observed_monotonic_ns == 0U) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "STOP_RECEIPT phase, status, or timestamp is invalid");
  }
  output->clear();
  output->reserve(16U + receipt.command_id.size() + receipt.detail.size());
  AppendU64(output, receipt.observed_monotonic_ns);
  AppendU8(output, static_cast<std::uint8_t>(receipt.phase));
  AppendU8(output, static_cast<std::uint8_t>(receipt.status));
  AppendU16(output, static_cast<std::uint16_t>(receipt.command_id.size()));
  AppendU16(output, static_cast<std::uint16_t>(receipt.detail.size()));
  AppendU16(output, 0U);
  AppendString(output, receipt.command_id);
  AppendString(output, receipt.detail);
  return Error{};
}

Error decodeStopReceiptPayload(const std::vector<std::uint8_t> &payload,
                               StopReceipt *output) {
  if (output == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "STOP_RECEIPT output is null");
  }
  if (payload.size() < 16U ||
      payload.size() > 16U + kMaxCommandIdBytes + kMaxReceiptDetailBytes) {
    return MakeError(ErrorCode::kInvalidLength,
                     "STOP_RECEIPT payload length is outside its bounds");
  }
  Reader reader(payload.data(), payload.size());
  StopReceipt candidate;
  std::uint8_t phase = 0U;
  std::uint8_t status = 0U;
  std::uint16_t command_length = 0U;
  std::uint16_t detail_length = 0U;
  std::uint16_t reserved = 0U;
  if (!reader.ReadU64(&candidate.observed_monotonic_ns) ||
      !reader.ReadU8(&phase) || !reader.ReadU8(&status) ||
      !reader.ReadU16(&command_length) || !reader.ReadU16(&detail_length) ||
      !reader.ReadU16(&reserved)) {
    return MakeError(ErrorCode::kTruncated,
                     "STOP_RECEIPT fixed payload is truncated");
  }
  candidate.phase = static_cast<ReceiptPhase>(phase);
  candidate.status = static_cast<ReceiptStatus>(status);
  if (!IsKnownReceiptPhase(candidate.phase) ||
      !IsKnownReceiptStatus(candidate.status) || reserved != 0U ||
      candidate.observed_monotonic_ns == 0U) {
    return MakeError(ErrorCode::kInvalidPayload,
                     "STOP_RECEIPT fields violate the v2 contract");
  }
  if (command_length == 0U || command_length > kMaxCommandIdBytes ||
      detail_length > kMaxReceiptDetailBytes ||
      payload.size() != 16U + command_length + detail_length ||
      !reader.ReadString(command_length, &candidate.command_id) ||
      !reader.ReadString(detail_length, &candidate.detail)) {
    return MakeError(ErrorCode::kInvalidLength,
                     "STOP_RECEIPT string lengths are inconsistent");
  }
  Error error = ValidateBoundedToken(candidate.command_id, kMaxCommandIdBytes,
                                     false, "command_id");
  if (!error.ok()) {
    return error;
  }
  error = ValidateBoundedToken(candidate.detail, kMaxReceiptDetailBytes, true,
                               "receipt detail");
  if (!error.ok()) {
    return error;
  }
  *output = std::move(candidate);
  return Error{};
}

namespace {

Error ValidateTimebase(const MonotonicTimebase &timebase) {
  if (timebase.sender_anchor_ns == 0U || timebase.receiver_anchor_ns == 0U ||
      timebase.valid_until_receiver_ns < timebase.receiver_anchor_ns) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "monotonic timebase anchors/validity are invalid");
  }
  if (timebase.maximum_error_ns > kMaximumTimebaseErrorNs) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "monotonic timebase uncertainty exceeds 50 ms");
  }
  return Error{};
}

Error MapSenderTimestamp(const MonotonicTimebase &timebase,
                         std::uint64_t sender_timestamp_ns,
                         std::uint64_t *lower_receiver_ns,
                         std::uint64_t *upper_receiver_ns) {
  if (lower_receiver_ns == nullptr || upper_receiver_ns == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "mapped timestamp output is null");
  }
  Error error = ValidateTimebase(timebase);
  if (!error.ok()) {
    return error;
  }

  std::uint64_t center = 0U;
  if (sender_timestamp_ns >= timebase.sender_anchor_ns) {
    const std::uint64_t delta = sender_timestamp_ns - timebase.sender_anchor_ns;
    if (timebase.receiver_anchor_ns >
        std::numeric_limits<std::uint64_t>::max() - delta) {
      return MakeError(ErrorCode::kInvalidMetadata,
                       "mapped timestamp overflows receiver clock");
    }
    center = timebase.receiver_anchor_ns + delta;
  } else {
    const std::uint64_t delta = timebase.sender_anchor_ns - sender_timestamp_ns;
    if (delta > timebase.receiver_anchor_ns) {
      return MakeError(ErrorCode::kInvalidMetadata,
                       "mapped timestamp underflows receiver clock");
    }
    center = timebase.receiver_anchor_ns - delta;
  }
  if (center < timebase.maximum_error_ns ||
      center > std::numeric_limits<std::uint64_t>::max() -
                   timebase.maximum_error_ns) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "mapped timestamp uncertainty overflows receiver clock");
  }
  *lower_receiver_ns = center - timebase.maximum_error_ns;
  *upper_receiver_ns = center + timebase.maximum_error_ns;
  return Error{};
}

Error ValidateAdmissionProcessingTime(const Admission &admission,
                                      std::uint64_t receiver_monotonic_ns) {
  if (!admission.valid() || admission.receivedMonotonicNs() == 0U ||
      receiver_monotonic_ns < admission.receivedMonotonicNs()) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "admission token or processing timestamp is invalid");
  }
  if (receiver_monotonic_ns - admission.receivedMonotonicNs() >
      kHeartbeatTimeoutNs) {
    return MakeError(ErrorCode::kStale,
                     "admission token was not consumed within 750 ms");
  }
  return Error{};
}

} // namespace

ReceiveGuard::ReceiveGuard(ExpectedPeer expected, Channel expected_channel,
                           MonotonicTimebase timebase, ReceivePolicy policy)
    : expected_(std::move(expected)), expected_channel_(expected_channel),
      timebase_(timebase), policy_(policy) {}

Error ReceiveGuard::accept(const Frame &frame,
                           std::uint64_t receiver_monotonic_ns,
                           Admission *admission) {
  if (admission == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "admission output is null");
  }
  *admission = Admission{};
  if (generation_ == 0U) {
    return MakeError(ErrorCode::kInvalidState,
                     "receive-guard generation counter is exhausted");
  }
  Error error = validateFrame(frame);
  if (!error.ok()) {
    return error;
  }
  if (!IsKnownChannel(expected_channel_) || expected_.session_epoch == 0U ||
      expected_.slot_id.empty() || expected_.asset_id.empty() ||
      expected_.robot_kind.empty() || expected_.run_id.empty() ||
      expected_.build_id.empty()) {
    return MakeError(ErrorCode::kInvalidIdentity,
                     "expected peer/channel identity is incomplete");
  }
  error = validatePeerCompatibility(
      expected_.local_role, expected_.local_capabilities, expected_.peer_role,
      expected_.capabilities);
  if (!error.ok()) {
    return error;
  }
  if (policy_.maximum_age_ns == 0U ||
      policy_.maximum_age_ns > kHeartbeatTimeoutNs ||
      policy_.future_tolerance_ns > kFutureToleranceNs) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "receive policy may not weaken frozen v2 bounds");
  }
  error = ValidateTimebase(timebase_);
  if (!error.ok()) {
    return error;
  }
  if (receiver_monotonic_ns == 0U ||
      receiver_monotonic_ns > timebase_.valid_until_receiver_ns) {
    return MakeError(ErrorCode::kStale,
                     "receiver time is outside the established timebase");
  }
  const FrameHeader &header = frame.header;
  if (header.channel != expected_channel_) {
    return MakeError(ErrorCode::kChannelMismatch,
                     "frame arrived on a different channel guard");
  }
  error = CheckIdentityField(header.slot_id, expected_.slot_id, "slot_id");
  if (!error.ok()) {
    return error;
  }
  error = CheckIdentityField(header.asset_id, expected_.asset_id, "asset_id");
  if (!error.ok()) {
    return error;
  }
  error =
      CheckIdentityField(header.robot_kind, expected_.robot_kind, "robot_kind");
  if (!error.ok()) {
    return error;
  }
  error = CheckIdentityField(header.run_id, expected_.run_id, "run_id");
  if (!error.ok()) {
    return error;
  }
  error = CheckIdentityField(header.build_id, expected_.build_id, "build_id");
  if (!error.ok()) {
    return error;
  }
  if (header.capabilities != expected_.capabilities) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "capability bitmap does not match expected peer");
  }
  PeerRole encoded_role = PeerRole::kUnspecified;
  error = InferRole(header.capabilities, &encoded_role);
  if (!error.ok() || encoded_role != expected_.peer_role) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "frame role does not match the expected peer role");
  }
  if (header.session_epoch != expected_.session_epoch) {
    return MakeError(ErrorCode::kEpochMismatch,
                     "session epoch does not match the frozen run");
  }
  if (!expected_.boot_id.empty() && header.boot_id != expected_.boot_id) {
    return MakeError(ErrorCode::kBootMismatch,
                     "boot identity does not match expected peer");
  }

  std::uint64_t mapped_lower_ns = 0U;
  std::uint64_t mapped_upper_ns = 0U;
  error = MapSenderTimestamp(timebase_, header.monotonic_ns, &mapped_lower_ns,
                             &mapped_upper_ns);
  if (!error.ok()) {
    return error;
  }
  if (receiver_monotonic_ns > mapped_lower_ns &&
      receiver_monotonic_ns - mapped_lower_ns > policy_.maximum_age_ns) {
    return MakeError(ErrorCode::kStale,
                     "frame exceeds absolute timebase freshness bound");
  }
  if (mapped_upper_ns > receiver_monotonic_ns &&
      mapped_upper_ns - receiver_monotonic_ns > policy_.future_tolerance_ns) {
    return MakeError(ErrorCode::kFutureTimestamp,
                     "frame exceeds absolute future-time bound");
  }

  if (!established_) {
    if (header.kind != MessageKind::kHello) {
      return MakeError(ErrorCode::kHandshakeRequired,
                       "each channel's first accepted frame must be HELLO");
    }
  } else {
    if (header.kind == MessageKind::kHello) {
      return MakeError(ErrorCode::kInvalidState,
                       "a new HELLO requires an explicit receive-guard reset");
    }
    if (header.boot_id != bound_boot_id_) {
      return MakeError(ErrorCode::kBootMismatch,
                       "boot identity changed without guard reset");
    }
    if (header.sequence <= last_sequence_) {
      return MakeError(ErrorCode::kReplay,
                       "channel sequence is replayed or out of order");
    }
    if (last_sequence_ == std::numeric_limits<std::uint64_t>::max() ||
        header.sequence != last_sequence_ + 1U) {
      return MakeError(
          ErrorCode::kSequenceGap,
          "channel sequence must advance contiguously within a generation");
    }
    if (header.monotonic_ns <= last_sender_monotonic_ns_ ||
        receiver_monotonic_ns < last_receiver_monotonic_ns_) {
      return MakeError(ErrorCode::kOutOfOrderTimestamp,
                       "channel monotonic timestamp did not advance");
    }
  }

  std::uint64_t mapped_stop_deadline_ns = 0U;
  std::string command_id;
  ReceiptPhase receipt_phase = ReceiptPhase::kAccepted;
  ReceiptStatus receipt_status = ReceiptStatus::kOk;
  if (header.kind == MessageKind::kZeroStop) {
    ZeroStop stop;
    error = decodeZeroStopPayload(frame.payload, &stop);
    if (!error.ok()) {
      return error;
    }
    error = validateStopDeadline(stop, header.monotonic_ns);
    if (!error.ok()) {
      return error;
    }
    std::uint64_t unused_upper_ns = 0U;
    error = MapSenderTimestamp(timebase_, stop.deadline_monotonic_ns,
                               &mapped_stop_deadline_ns, &unused_upper_ns);
    if (!error.ok()) {
      return error;
    }
    mapped_stop_deadline_ns =
        std::min(mapped_stop_deadline_ns, timebase_.valid_until_receiver_ns);
    if (receiver_monotonic_ns > mapped_stop_deadline_ns) {
      return MakeError(ErrorCode::kDeadlineExpired,
                       "ZERO_STOP arrived after conservative mapped deadline");
    }
    command_id = stop.command_id;
  } else if (header.kind == MessageKind::kStopReceipt) {
    StopReceipt receipt;
    error = decodeStopReceiptPayload(frame.payload, &receipt);
    if (!error.ok()) {
      return error;
    }
    command_id = receipt.command_id;
    receipt_phase = receipt.phase;
    receipt_status = receipt.status;
  }

  if (!established_) {
    established_ = true;
    bound_boot_id_ = header.boot_id;
  }
  if (header.kind == MessageKind::kHello) {
    last_heartbeat_receiver_monotonic_ns_ = 0U;
  } else if (header.kind == MessageKind::kHeartbeat) {
    last_heartbeat_receiver_monotonic_ns_ = receiver_monotonic_ns;
  }
  last_sequence_ = header.sequence;
  last_sender_monotonic_ns_ = header.monotonic_ns;
  last_receiver_monotonic_ns_ = receiver_monotonic_ns;

  admission->valid_ = true;
  admission->channel_ = header.channel;
  admission->kind_ = header.kind;
  admission->sequence_ = header.sequence;
  admission->session_epoch_ = header.session_epoch;
  admission->slot_id_ = header.slot_id;
  admission->asset_id_ = header.asset_id;
  admission->robot_kind_ = header.robot_kind;
  admission->run_id_ = header.run_id;
  admission->boot_id_ = header.boot_id;
  admission->build_id_ = header.build_id;
  admission->received_monotonic_ns_ = receiver_monotonic_ns;
  admission->mapped_stop_deadline_ns_ = mapped_stop_deadline_ns;
  admission->command_id_ = std::move(command_id);
  admission->receipt_phase_ = receipt_phase;
  admission->receipt_status_ = receipt_status;
  admission->peer_role_ = expected_.peer_role;
  admission->peer_capabilities_ = header.capabilities;
  admission->issuer_ = this;
  admission->receive_generation_ = generation_;
  admission->canonical_header_ = header;
  admission->canonical_payload_ = frame.payload;
  return Error{};
}

Error ReceiveGuard::checkFresh(std::uint64_t receiver_monotonic_ns) const {
  if (!established_) {
    return MakeError(ErrorCode::kHandshakeRequired,
                     "channel has not accepted HELLO");
  }
  Error error = ValidateTimebase(timebase_);
  if (!error.ok()) {
    return error;
  }
  if (receiver_monotonic_ns < last_receiver_monotonic_ns_) {
    return MakeError(ErrorCode::kOutOfOrderTimestamp,
                     "receiver monotonic timestamp moved backwards");
  }
  if (receiver_monotonic_ns > timebase_.valid_until_receiver_ns ||
      receiver_monotonic_ns - last_receiver_monotonic_ns_ >
          policy_.maximum_age_ns) {
    return MakeError(ErrorCode::kHeartbeatTimeout,
                     "channel is no longer fresh");
  }
  return Error{};
}

void ReceiveGuard::reset(ExpectedPeer expected, MonotonicTimebase timebase) {
  generation_ = generation_ == std::numeric_limits<std::uint64_t>::max()
                    ? 0U
                    : generation_ + 1U;
  expected_ = std::move(expected);
  timebase_ = timebase;
  established_ = false;
  bound_boot_id_.clear();
  last_sequence_ = 0U;
  last_sender_monotonic_ns_ = 0U;
  last_receiver_monotonic_ns_ = 0U;
  last_heartbeat_receiver_monotonic_ns_ = 0U;
}

Error requireThreeChannelFresh(const ReceiveGuard &management,
                               const ReceiveGuard &control,
                               const ReceiveGuard &telemetry,
                               std::uint64_t receiver_monotonic_ns) {
  if (management.expectedChannel() != Channel::kManagement ||
      control.expectedChannel() != Channel::kControl ||
      telemetry.expectedChannel() != Channel::kTelemetry) {
    return MakeError(ErrorCode::kChannelMismatch,
                     "three-channel readiness guards are misassigned");
  }

  const ExpectedPeer &reference = management.expected_;
  Error error = validatePeerCompatibility(
      reference.local_role, reference.local_capabilities, reference.peer_role,
      reference.capabilities);
  if (!error.ok()) {
    return error;
  }
  const auto same_peer_identity = [&reference](const ExpectedPeer &candidate) {
    return candidate.slot_id == reference.slot_id &&
           candidate.asset_id == reference.asset_id &&
           candidate.robot_kind == reference.robot_kind &&
           candidate.run_id == reference.run_id &&
           candidate.build_id == reference.build_id;
  };
  if (!same_peer_identity(control.expected_) ||
      !same_peer_identity(telemetry.expected_)) {
    return MakeError(ErrorCode::kIdentityMismatch,
                     "three-channel guards do not describe one frozen peer");
  }
  if (control.expected_.session_epoch != reference.session_epoch ||
      telemetry.expected_.session_epoch != reference.session_epoch) {
    return MakeError(ErrorCode::kEpochMismatch,
                     "three-channel guards use different Session epochs");
  }
  if (control.expected_.capabilities != reference.capabilities ||
      telemetry.expected_.capabilities != reference.capabilities) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "three-channel guards use different capability sets");
  }
  if (control.expected_.peer_role != reference.peer_role ||
      telemetry.expected_.peer_role != reference.peer_role ||
      control.expected_.local_role != reference.local_role ||
      telemetry.expected_.local_role != reference.local_role ||
      control.expected_.local_capabilities != reference.local_capabilities ||
      telemetry.expected_.local_capabilities != reference.local_capabilities) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "three-channel guards use different role contracts");
  }
  if (control.expected_.boot_id != reference.boot_id ||
      telemetry.expected_.boot_id != reference.boot_id) {
    return MakeError(ErrorCode::kBootMismatch,
                     "three-channel guards use different configured boot IDs");
  }
  if (!management.established_ || !control.established_ ||
      !telemetry.established_ || management.bound_boot_id_.empty() ||
      control.bound_boot_id_.empty() || telemetry.bound_boot_id_.empty()) {
    return MakeError(ErrorCode::kHandshakeRequired,
                     "all three channels require an admitted HELLO");
  }
  if (control.bound_boot_id_ != management.bound_boot_id_ ||
      telemetry.bound_boot_id_ != management.bound_boot_id_) {
    return MakeError(ErrorCode::kBootMismatch,
                     "three-channel guards are bound to different boots");
  }

  const auto require_fresh_heartbeat = [receiver_monotonic_ns](
                                           const ReceiveGuard &guard) {
    Error error = guard.checkFresh(receiver_monotonic_ns);
    if (!error.ok()) {
      return error;
    }
    if (guard.last_heartbeat_receiver_monotonic_ns_ == 0U) {
      return MakeError(
          ErrorCode::kHandshakeRequired,
          "every channel requires HEARTBEAT after its latest HELLO");
    }
    if (receiver_monotonic_ns - guard.last_heartbeat_receiver_monotonic_ns_ >
        guard.policy_.maximum_age_ns) {
      return MakeError(ErrorCode::kHeartbeatTimeout,
                       "channel HEARTBEAT is no longer fresh");
    }
    return Error{};
  };

  error = require_fresh_heartbeat(management);
  if (!error.ok()) {
    return error;
  }
  error = require_fresh_heartbeat(control);
  if (!error.ok()) {
    return error;
  }
  return require_fresh_heartbeat(telemetry);
}

namespace {

Error ValidateSendWindowConfig(const SendWindowConfig &config) {
  if (!IsKnownChannel(config.channel) || config.session_epoch == 0U ||
      config.capacity == 0U || config.capacity > kMaximumSendWindowCapacity) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "send-window channel, epoch, or capacity is invalid");
  }
  Error error =
      validatePeerCompatibility(config.local_role, config.local_capabilities,
                                config.peer_role, config.peer_capabilities);
  if (!error.ok()) {
    return error;
  }
  const struct {
    const std::string *value;
    std::size_t limit;
    bool allow_empty;
    const char *name;
  } identities[] = {
      {&config.local_boot_id, kMaxBootIdBytes, false, "local_boot_id"},
      {&config.peer_slot_id, kMaxSlotIdBytes, false, "peer_slot_id"},
      {&config.peer_asset_id, kMaxAssetIdBytes, false, "peer_asset_id"},
      {&config.peer_robot_kind, kMaxRobotKindBytes, false, "peer_robot_kind"},
      {&config.peer_run_id, kMaxRunIdBytes, false, "peer_run_id"},
      {&config.peer_boot_id, kMaxBootIdBytes, true, "peer_boot_id"},
      {&config.peer_build_id, kMaxBuildIdBytes, false, "peer_build_id"},
  };
  for (const auto &identity : identities) {
    error = ValidateBoundedToken(*identity.value, identity.limit,
                                 identity.allow_empty, identity.name);
    if (!error.ok()) {
      return error;
    }
  }
  return Error{};
}

bool AdmissionMatchesPeer(const Admission &admission,
                          const SendWindowConfig &config) {
  return admission.channel() == config.channel &&
         admission.sessionEpoch() == config.session_epoch &&
         admission.slotId() == config.peer_slot_id &&
         admission.assetId() == config.peer_asset_id &&
         admission.robotKind() == config.peer_robot_kind &&
         admission.runId() == config.peer_run_id &&
         admission.buildId() == config.peer_build_id &&
         admission.peerRole() == config.peer_role &&
         admission.peerCapabilities() == config.peer_capabilities &&
         (config.peer_boot_id.empty() ||
          admission.bootId() == config.peer_boot_id);
}

bool FrameMatchesLocalGeneration(const FrameHeader &header,
                                 const SendWindowConfig &config) {
  return header.channel == config.channel &&
         header.session_epoch == config.session_epoch &&
         header.boot_id == config.local_boot_id &&
         header.capabilities == config.local_capabilities;
}

} // namespace

SendWindow::SendWindow(SendWindowConfig config) : config_(std::move(config)) {}

Error SendWindow::recordSent(const Frame &frame) {
  Error error = ValidateSendWindowConfig(config_);
  if (!error.ok()) {
    return error;
  }
  error = validateFrame(frame);
  if (!error.ok()) {
    return error;
  }
  if (!FrameMatchesLocalGeneration(frame.header, config_)) {
    if (frame.header.channel != config_.channel) {
      return MakeError(ErrorCode::kChannelMismatch,
                       "sent frame does not belong to this channel window");
    }
    if (frame.header.session_epoch != config_.session_epoch) {
      return MakeError(ErrorCode::kEpochMismatch,
                       "sent frame does not belong to this Session epoch");
    }
    if (frame.header.boot_id != config_.local_boot_id) {
      return MakeError(ErrorCode::kBootMismatch,
                       "sent frame does not belong to this local boot");
    }
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "sent frame does not match local role capabilities");
  }

  PeerRole encoded_role = PeerRole::kUnspecified;
  error = InferRole(frame.header.capabilities, &encoded_role);
  if (!error.ok() || encoded_role != config_.local_role) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "sent frame role does not match local role");
  }
  if (!outbound_established_) {
    if (frame.header.kind != MessageKind::kHello) {
      return MakeError(ErrorCode::kHandshakeRequired,
                       "first recorded send on each channel must be HELLO");
    }
  } else {
    if (frame.header.kind == MessageKind::kHello) {
      return MakeError(ErrorCode::kInvalidState,
                       "a new local HELLO requires explicit window reset");
    }
    if (frame.header.slot_id != local_slot_id_ ||
        frame.header.asset_id != local_asset_id_ ||
        frame.header.robot_kind != local_robot_kind_ ||
        frame.header.run_id != local_run_id_ ||
        frame.header.build_id != local_build_id_) {
      return MakeError(ErrorCode::kIdentityMismatch,
                       "sent frame changed the local HELLO identity");
    }
    if (frame.header.sequence <= last_sent_sequence_) {
      return MakeError(ErrorCode::kReplay,
                       "sent sequence is replayed or out of order");
    }
    if (last_sent_sequence_ == std::numeric_limits<std::uint64_t>::max() ||
        frame.header.sequence != last_sent_sequence_ + 1U) {
      return MakeError(
          ErrorCode::kSequenceGap,
          "sent sequence must advance contiguously within a generation");
    }
  }
  if (outstanding_.size() >= config_.capacity) {
    return MakeError(ErrorCode::kSendWindowFull,
                     "unacknowledged send window is full");
  }

  if (!outbound_established_) {
    outbound_established_ = true;
    hello_sequence_ = frame.header.sequence;
    local_slot_id_ = frame.header.slot_id;
    local_asset_id_ = frame.header.asset_id;
    local_robot_kind_ = frame.header.robot_kind;
    local_run_id_ = frame.header.run_id;
    local_build_id_ = frame.header.build_id;
  }
  last_sent_sequence_ = frame.header.sequence;
  outstanding_.push_back(frame.header.sequence);
  return Error{};
}

Error SendWindow::bindPeerHello(const Admission &admission) {
  Error error = ValidateSendWindowConfig(config_);
  if (!error.ok()) {
    return error;
  }
  if (!outbound_established_) {
    return MakeError(ErrorCode::kHandshakeRequired,
                     "local HELLO must be recorded before peer HELLO binding");
  }
  if (!admission.valid_ || admission.kind_ != MessageKind::kHello) {
    return MakeError(ErrorCode::kHandshakeRequired,
                     "send window requires an admitted peer HELLO");
  }
  if (admission.issuer_ == nullptr || admission.receive_generation_ == 0U) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "peer HELLO admission has no receive generation");
  }
  if (!AdmissionMatchesPeer(admission, config_)) {
    if (admission.channel_ != config_.channel) {
      return MakeError(ErrorCode::kChannelMismatch,
                       "peer HELLO belongs to another channel");
    }
    if (admission.session_epoch_ != config_.session_epoch) {
      return MakeError(ErrorCode::kEpochMismatch,
                       "peer HELLO belongs to another Session epoch");
    }
    if (!config_.peer_boot_id.empty() &&
        admission.boot_id_ != config_.peer_boot_id) {
      return MakeError(ErrorCode::kBootMismatch,
                       "peer HELLO boot does not match frozen peer");
    }
    if (admission.peer_role_ != config_.peer_role ||
        admission.peer_capabilities_ != config_.peer_capabilities) {
      return MakeError(ErrorCode::kCapabilityMismatch,
                       "peer HELLO role capabilities do not match");
    }
    return MakeError(ErrorCode::kIdentityMismatch,
                     "peer HELLO identity does not match send window");
  }
  if (peer_hello_bound_) {
    return MakeError(ErrorCode::kInvalidState,
                     "a new peer HELLO requires explicit window reset");
  }
  Hello hello;
  error = decodeHelloPayload(admission.canonical_payload_, &hello);
  if (!error.ok()) {
    return error;
  }
  peer_hello_bound_ = true;
  bound_peer_boot_id_ = admission.boot_id_;
  bound_receive_guard_ = admission.issuer_;
  bound_receive_generation_ = admission.receive_generation_;
  return Error{};
}

Error SendWindow::observeHeartbeat(const Heartbeat &heartbeat,
                                   const Admission &admission,
                                   AckDisposition *disposition) {
  if (disposition == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "ACK disposition output is null");
  }
  *disposition = AckDisposition::kDuplicate;
  Error error = ValidateSendWindowConfig(config_);
  if (!error.ok()) {
    return error;
  }
  if (!outbound_established_ || !peer_hello_bound_) {
    return MakeError(ErrorCode::kHandshakeRequired,
                     "both local and peer HELLO are required before ACK");
  }
  if (!admission.valid_ || admission.kind_ != MessageKind::kHeartbeat) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "ACK requires an admitted HEARTBEAT token");
  }
  if (admission.issuer_ != bound_receive_guard_ ||
      admission.receive_generation_ != bound_receive_generation_) {
    return MakeError(
        ErrorCode::kInvalidState,
        "HEARTBEAT ACK belongs to another receive-guard generation");
  }
  if (!AdmissionMatchesPeer(admission, config_) ||
      admission.boot_id_ != bound_peer_boot_id_) {
    if (admission.channel_ != config_.channel) {
      return MakeError(ErrorCode::kChannelMismatch,
                       "HEARTBEAT ACK belongs to another channel");
    }
    if (admission.session_epoch_ != config_.session_epoch) {
      return MakeError(ErrorCode::kEpochMismatch,
                       "HEARTBEAT ACK belongs to another Session epoch");
    }
    if (admission.boot_id_ != bound_peer_boot_id_) {
      return MakeError(ErrorCode::kBootMismatch,
                       "HEARTBEAT ACK belongs to another peer boot");
    }
    if (admission.peer_role_ != config_.peer_role ||
        admission.peer_capabilities_ != config_.peer_capabilities) {
      return MakeError(ErrorCode::kCapabilityMismatch,
                       "HEARTBEAT ACK role capabilities changed");
    }
    return MakeError(ErrorCode::kIdentityMismatch,
                     "HEARTBEAT ACK peer identity changed");
  }
  std::vector<std::uint8_t> canonical;
  error = encodeHeartbeatPayload(heartbeat, &canonical);
  if (!error.ok()) {
    return error;
  }
  if (!admission.matchesCanonicalPayload(canonical)) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "HEARTBEAT DTO does not match its admission token");
  }

  const std::uint64_t acknowledged = heartbeat.last_received_sequence;
  if (acknowledged == 0U) {
    return MakeError(ErrorCode::kAckOutOfWindow,
                     "zero does not acknowledge a sent v2 sequence");
  }
  if (acknowledged > last_sent_sequence_) {
    return MakeError(ErrorCode::kAckFuture,
                     "HEARTBEAT acknowledges a future local sequence");
  }
  if (acknowledged < acknowledged_frontier_) {
    return MakeError(ErrorCode::kAckRegression,
                     "HEARTBEAT ACK frontier moved backwards");
  }
  if (acknowledged == acknowledged_frontier_) {
    *disposition = AckDisposition::kDuplicate;
    return Error{};
  }

  const auto acknowledged_entry =
      std::find(outstanding_.begin(), outstanding_.end(), acknowledged);
  if (acknowledged_entry == outstanding_.end()) {
    return MakeError(
        ErrorCode::kAckOutOfWindow,
        "HEARTBEAT ACK is not an exact retained local send sequence");
  }
  acknowledged_frontier_ = acknowledged;
  outstanding_.erase(outstanding_.begin(), acknowledged_entry + 1);
  *disposition = AckDisposition::kAdvanced;
  return Error{};
}

void SendWindow::reset(SendWindowConfig config) {
  config_ = std::move(config);
  outbound_established_ = false;
  peer_hello_bound_ = false;
  local_slot_id_.clear();
  local_asset_id_.clear();
  local_robot_kind_.clear();
  local_run_id_.clear();
  local_build_id_.clear();
  bound_peer_boot_id_.clear();
  bound_receive_guard_ = nullptr;
  bound_receive_generation_ = 0U;
  hello_sequence_ = 0U;
  last_sent_sequence_ = 0U;
  acknowledged_frontier_ = 0U;
  outstanding_.clear();
}

Error requireThreeChannelReady(const ReceiveGuard &management,
                               const ReceiveGuard &control,
                               const ReceiveGuard &telemetry,
                               const SendWindow &management_send,
                               const SendWindow &control_send,
                               const SendWindow &telemetry_send,
                               std::uint64_t receiver_monotonic_ns) {
  Error error = requireThreeChannelFresh(management, control, telemetry,
                                         receiver_monotonic_ns);
  if (!error.ok()) {
    return error;
  }
  if (management_send.config_.channel != Channel::kManagement ||
      control_send.config_.channel != Channel::kControl ||
      telemetry_send.config_.channel != Channel::kTelemetry) {
    return MakeError(ErrorCode::kChannelMismatch,
                     "three send windows are assigned to wrong channels");
  }

  const SendWindow *windows[] = {&management_send, &control_send,
                                 &telemetry_send};
  const ReceiveGuard *guards[] = {&management, &control, &telemetry};
  const SendWindowConfig &reference = management_send.config_;
  for (std::size_t index = 0U; index < 3U; ++index) {
    const SendWindow &window = *windows[index];
    const ReceiveGuard &guard = *guards[index];
    error = ValidateSendWindowConfig(window.config_);
    if (!error.ok()) {
      return error;
    }
    if (window.config_.session_epoch != reference.session_epoch ||
        window.config_.local_role != reference.local_role ||
        window.config_.local_capabilities != reference.local_capabilities ||
        window.config_.peer_role != reference.peer_role ||
        window.config_.peer_capabilities != reference.peer_capabilities ||
        window.config_.local_boot_id != reference.local_boot_id ||
        window.local_slot_id_ != management_send.local_slot_id_ ||
        window.local_asset_id_ != management_send.local_asset_id_ ||
        window.local_robot_kind_ != management_send.local_robot_kind_ ||
        window.local_run_id_ != management_send.local_run_id_ ||
        window.local_build_id_ != management_send.local_build_id_) {
      return MakeError(ErrorCode::kIdentityMismatch,
                       "three send windows do not describe one local peer");
    }
    if (window.config_.peer_slot_id != guard.expected_.slot_id ||
        window.config_.peer_asset_id != guard.expected_.asset_id ||
        window.config_.peer_robot_kind != guard.expected_.robot_kind ||
        window.config_.peer_run_id != guard.expected_.run_id ||
        window.config_.peer_build_id != guard.expected_.build_id ||
        window.config_.session_epoch != guard.expected_.session_epoch ||
        window.config_.local_role != guard.expected_.local_role ||
        window.config_.local_capabilities !=
            guard.expected_.local_capabilities ||
        window.config_.peer_role != guard.expected_.peer_role ||
        window.config_.peer_capabilities != guard.expected_.capabilities) {
      return MakeError(ErrorCode::kIdentityMismatch,
                       "send window and receive guard contracts differ");
    }
    if (!window.outbound_established_ || !window.peer_hello_bound_ ||
        window.bound_receive_guard_ != &guard ||
        window.bound_receive_generation_ != guard.generation_ ||
        window.bound_peer_boot_id_ != guard.bound_boot_id_ ||
        window.hello_sequence_ == 0U ||
        window.acknowledged_frontier_ < window.hello_sequence_) {
      return MakeError(
          ErrorCode::kHandshakeRequired,
          "each channel requires a peer ACK of its current local HELLO");
    }
  }
  return Error{};
}

StopWatchdog::StopWatchdog(std::uint64_t session_epoch)
    : state_(session_epoch == 0U ? LinkState::kSafeHold
                                 : LinkState::kAwaitHello),
      session_epoch_(session_epoch) {}

Error StopWatchdog::onHello(const Admission &admission,
                            std::uint64_t receiver_monotonic_ns,
                            SafetyAction *action) {
  if (action == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "safety action output is null");
  }
  *action = SafetyAction::kNone;
  Error error =
      ValidateAdmissionProcessingTime(admission, receiver_monotonic_ns);
  if (!error.ok() || admission.kind() != MessageKind::kHello ||
      admission.channel() != Channel::kControl || admission.bootId().empty() ||
      admission.peerRole() != PeerRole::kGround ||
      admission.peerCapabilities() != kGroundRequiredCapabilities ||
      admission.issuer_ == nullptr || admission.receive_generation_ == 0U ||
      session_epoch_ == 0U) {
    if (state_ != LinkState::kAwaitHello) {
      enterSafeHold();
      *action = SafetyAction::kEnterSafeHold;
    }
    return error.ok()
               ? MakeError(ErrorCode::kInvalidMetadata,
                           "watchdog requires a control-channel HELLO token")
               : error;
  }
  if (admission.sessionEpoch() != session_epoch_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kEpochMismatch,
                     "HELLO epoch does not match watchdog epoch");
  }
  if (admission.bootId() == boot_id_ &&
      admission.sequence() <= last_admission_sequence_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kReplay, "HELLO admission token was replayed");
  }
  const bool enabled_before_hello = state_ == LinkState::kEnabled;
  boot_id_ = admission.bootId();
  bound_receive_guard_ = admission.issuer_;
  bound_receive_generation_ = admission.receive_generation_;
  last_admission_sequence_ = admission.sequence();
  last_signal_monotonic_ns_ = admission.receivedMonotonicNs();
  pending_stop_command_id_.clear();
  pending_stop_accepted_ns_ = 0U;
  pending_stop_receiver_deadline_ns_ = 0U;
  state_ = LinkState::kInhibited;
  if (enabled_before_hello) {
    *action = SafetyAction::kEnterSafeHold;
  }
  return Error{};
}

Error StopWatchdog::onHeartbeat(const Admission &admission,
                                std::uint64_t receiver_monotonic_ns,
                                SafetyAction *action) {
  if (action == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "safety action output is null");
  }
  *action = SafetyAction::kNone;
  Error error =
      ValidateAdmissionProcessingTime(admission, receiver_monotonic_ns);
  if (!error.ok() || admission.kind() != MessageKind::kHeartbeat ||
      admission.channel() != Channel::kControl ||
      admission.peerRole() != PeerRole::kGround ||
      admission.peerCapabilities() != kGroundRequiredCapabilities) {
    if (state_ == LinkState::kEnabled || state_ == LinkState::kInhibited) {
      enterSafeHold();
      *action = SafetyAction::kEnterSafeHold;
    }
    return error.ok() ? MakeError(ErrorCode::kInvalidMetadata,
                                  "watchdog requires a control HEARTBEAT token")
                      : error;
  }
  if (admission.sessionEpoch() != session_epoch_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kEpochMismatch,
                     "HEARTBEAT epoch changed during the run");
  }
  if (state_ == LinkState::kAwaitHello || state_ == LinkState::kSafeHold ||
      boot_id_.empty()) {
    return MakeError(ErrorCode::kHandshakeRequired,
                     "a new HELLO is required before HEARTBEAT");
  }
  if (admission.bootId() != boot_id_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kBootMismatch,
                     "HEARTBEAT boot identity changed");
  }
  if (admission.issuer_ != bound_receive_guard_ ||
      admission.receive_generation_ != bound_receive_generation_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(
        ErrorCode::kInvalidState,
        "HEARTBEAT receive-guard generation changed during the run");
  }
  if (admission.sequence() <= last_admission_sequence_ ||
      admission.receivedMonotonicNs() < last_signal_monotonic_ns_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kReplay,
                     "HEARTBEAT admission token was replayed");
  }
  if (admission.receivedMonotonicNs() - last_signal_monotonic_ns_ >
      kHeartbeatTimeoutNs) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kHeartbeatTimeout,
                     "HEARTBEAT arrived after watchdog timeout");
  }
  last_admission_sequence_ = admission.sequence();
  last_signal_monotonic_ns_ = admission.receivedMonotonicNs();
  return Error{};
}

Error StopWatchdog::reenable(std::uint64_t session_epoch,
                             const std::string &boot_id,
                             std::uint64_t receiver_monotonic_ns) {
  if (state_ != LinkState::kInhibited || boot_id_.empty() ||
      bound_receive_guard_ == nullptr || bound_receive_generation_ == 0U) {
    return MakeError(ErrorCode::kHandshakeRequired,
                     "explicit enable requires a fresh HELLO");
  }
  if (session_epoch != session_epoch_) {
    enterSafeHold();
    return MakeError(ErrorCode::kEpochMismatch,
                     "enable epoch does not match watchdog epoch");
  }
  if (boot_id != boot_id_) {
    enterSafeHold();
    return MakeError(ErrorCode::kBootMismatch,
                     "enable boot identity does not match HELLO");
  }
  if (receiver_monotonic_ns < last_signal_monotonic_ns_ ||
      receiver_monotonic_ns - last_signal_monotonic_ns_ > kHeartbeatTimeoutNs) {
    enterSafeHold();
    return MakeError(ErrorCode::kHeartbeatTimeout,
                     "HELLO is no longer fresh enough to enable");
  }
  state_ = LinkState::kEnabled;
  return Error{};
}

Error StopWatchdog::poll(std::uint64_t receiver_monotonic_ns,
                         SafetyAction *action) {
  if (action == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "safety action output is null");
  }
  *action = SafetyAction::kNone;
  if (state_ == LinkState::kAwaitHello || state_ == LinkState::kSafeHold) {
    return Error{};
  }
  if (receiver_monotonic_ns < last_signal_monotonic_ns_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kOutOfOrderTimestamp,
                     "receiver monotonic timestamp moved backwards");
  }
  if (receiver_monotonic_ns - last_signal_monotonic_ns_ > kHeartbeatTimeoutNs) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kHeartbeatTimeout,
                     "watchdog heartbeat deadline elapsed");
  }
  return Error{};
}

Error StopWatchdog::onEpochChanged(std::uint64_t new_session_epoch,
                                   SafetyAction *action) {
  if (action == nullptr) {
    return MakeError(ErrorCode::kNullOutput, "safety action output is null");
  }
  *action = SafetyAction::kNone;
  if (new_session_epoch == 0U) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kInvalidMetadata,
                     "watchdog epoch must be positive");
  }
  if (new_session_epoch == session_epoch_) {
    return Error{};
  }
  session_epoch_ = new_session_epoch;
  last_admission_sequence_ = 0U;
  enterSafeHold();
  *action = SafetyAction::kEnterSafeHold;
  return Error{};
}

Error StopWatchdog::acceptZeroStop(const ZeroStop &stop,
                                   const Admission &admission,
                                   std::uint64_t receiver_monotonic_ns,
                                   StopReceipt *accepted_receipt,
                                   SafetyAction *action) {
  if (accepted_receipt == nullptr || action == nullptr) {
    return MakeError(ErrorCode::kNullOutput,
                     "STOP receipt or safety action output is null");
  }
  *action = SafetyAction::kNone;
  if (state_ != LinkState::kEnabled || boot_id_.empty()) {
    return MakeError(ErrorCode::kHandshakeRequired,
                     "ZERO_STOP requires HELLO and explicit enable");
  }
  Error error =
      ValidateAdmissionProcessingTime(admission, receiver_monotonic_ns);
  if (!error.ok() || admission.channel() != Channel::kControl ||
      admission.kind() != MessageKind::kZeroStop ||
      admission.commandId() != stop.command_id ||
      admission.peerRole() != PeerRole::kGround ||
      admission.peerCapabilities() != kGroundRequiredCapabilities ||
      admission.issuer_ != bound_receive_guard_ ||
      admission.receive_generation_ != bound_receive_generation_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return error.ok()
               ? MakeError(ErrorCode::kInvalidMetadata,
                           "ZERO_STOP does not match its admission token")
               : error;
  }
  std::vector<std::uint8_t> encoded;
  error = encodeZeroStopPayload(stop, &encoded);
  if (!error.ok()) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return error;
  }
  if (!admission.matchesCanonicalPayload(encoded)) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kInvalidMetadata,
                     "ZERO_STOP payload does not match its admission token");
  }
  if (admission.sessionEpoch() != session_epoch_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kEpochMismatch,
                     "ZERO_STOP admission epoch changed");
  }
  if (admission.bootId() != boot_id_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kBootMismatch,
                     "ZERO_STOP admission boot identity changed");
  }
  if (admission.sequence() <= last_admission_sequence_ ||
      receiver_monotonic_ns < last_signal_monotonic_ns_) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kReplay,
                     "ZERO_STOP admission token was replayed");
  }
  if (receiver_monotonic_ns - last_signal_monotonic_ns_ > kHeartbeatTimeoutNs) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kHeartbeatTimeout,
                     "control heartbeat expired before ZERO_STOP");
  }
  if (admission.mappedStopDeadlineNs() == 0U ||
      receiver_monotonic_ns > admission.mappedStopDeadlineNs()) {
    enterSafeHold();
    *action = SafetyAction::kEnterSafeHold;
    return MakeError(ErrorCode::kDeadlineExpired,
                     "ZERO_STOP mapped deadline expired before admission");
  }
  last_admission_sequence_ = admission.sequence();
  enterSafeHold();
  pending_stop_command_id_ = stop.command_id;
  pending_stop_accepted_ns_ = receiver_monotonic_ns;
  pending_stop_receiver_deadline_ns_ = admission.mappedStopDeadlineNs();
  *action = SafetyAction::kApplyZeroStop;
  *accepted_receipt =
      StopReceipt{stop.command_id, ReceiptPhase::kAccepted, ReceiptStatus::kOk,
                  receiver_monotonic_ns, "accepted"};
  return Error{};
}

Error StopWatchdog::confirmStopApplied(const std::string &command_id,
                                       std::uint64_t receiver_monotonic_ns,
                                       StopReceipt *applied_receipt) {
  if (applied_receipt == nullptr) {
    return MakeError(ErrorCode::kNullOutput,
                     "applied STOP receipt output is null");
  }
  if (state_ != LinkState::kSafeHold || pending_stop_command_id_.empty() ||
      command_id != pending_stop_command_id_ || receiver_monotonic_ns == 0U ||
      receiver_monotonic_ns < pending_stop_accepted_ns_) {
    return MakeError(
        ErrorCode::kInvalidState,
        "STOP cannot be marked applied before matching acceptance");
  }
  if (receiver_monotonic_ns > pending_stop_receiver_deadline_ns_) {
    pending_stop_command_id_.clear();
    pending_stop_accepted_ns_ = 0U;
    pending_stop_receiver_deadline_ns_ = 0U;
    return MakeError(ErrorCode::kDeadlineExpired,
                     "STOP applied proof arrived after its mapped deadline");
  }
  *applied_receipt =
      StopReceipt{command_id, ReceiptPhase::kApplied, ReceiptStatus::kOk,
                  receiver_monotonic_ns, "applied"};
  pending_stop_command_id_.clear();
  pending_stop_accepted_ns_ = 0U;
  pending_stop_receiver_deadline_ns_ = 0U;
  return Error{};
}

void StopWatchdog::enterSafeHold() {
  state_ = LinkState::kSafeHold;
  last_signal_monotonic_ns_ = 0U;
  boot_id_.clear();
  bound_receive_guard_ = nullptr;
  bound_receive_generation_ = 0U;
  pending_stop_command_id_.clear();
  pending_stop_accepted_ns_ = 0U;
  pending_stop_receiver_deadline_ns_ = 0U;
}

StopReceiptCorrelator::StopReceiptCorrelator(std::uint64_t session_epoch,
                                             std::string boot_id)
    : session_epoch_(session_epoch), boot_id_(std::move(boot_id)) {}

Error StopReceiptCorrelator::begin(const ZeroStop &stop,
                                   std::uint64_t sender_monotonic_ns) {
  if (session_epoch_ == 0U || boot_id_.empty()) {
    return MakeError(ErrorCode::kInvalidIdentity,
                     "receipt peer epoch/boot identity is incomplete");
  }
  if (progress_ == ReceiptProgress::kAwaitAccepted ||
      progress_ == ReceiptProgress::kAwaitApplied) {
    return MakeError(ErrorCode::kInvalidState,
                     "another STOP receipt sequence is still active");
  }
  if (!command_id_.empty() && stop.command_id == command_id_) {
    return MakeError(ErrorCode::kReplay, "STOP command ID may not be reused");
  }
  std::vector<std::uint8_t> encoded;
  Error error = encodeZeroStopPayload(stop, &encoded);
  if (!error.ok()) {
    return error;
  }
  error = validateStopDeadline(stop, sender_monotonic_ns);
  if (!error.ok()) {
    return error;
  }
  command_id_ = stop.command_id;
  deadline_monotonic_ns_ = stop.deadline_monotonic_ns;
  bound_receive_guard_ = nullptr;
  bound_receive_generation_ = 0U;
  progress_ = ReceiptProgress::kAwaitAccepted;
  return Error{};
}

Error StopReceiptCorrelator::observe(const StopReceipt &receipt,
                                     const Admission &admission,
                                     std::uint64_t receiver_monotonic_ns) {
  Error error =
      ValidateAdmissionProcessingTime(admission, receiver_monotonic_ns);
  if (!error.ok()) {
    return error;
  }
  std::vector<std::uint8_t> encoded;
  error = encodeStopReceiptPayload(receipt, &encoded);
  if (!error.ok()) {
    return error;
  }
  if (!admission.matchesCanonicalPayload(encoded)) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "STOP_RECEIPT payload does not match its admission token");
  }
  if (admission.peerRole() != PeerRole::kVehicle ||
      admission.peerCapabilities() != kVehicleRequiredCapabilities ||
      admission.issuer_ == nullptr || admission.receive_generation_ == 0U) {
    return MakeError(ErrorCode::kCapabilityMismatch,
                     "STOP_RECEIPT requires the exact vehicle role contract");
  }
  if (bound_receive_guard_ != nullptr &&
      (admission.issuer_ != bound_receive_guard_ ||
       admission.receive_generation_ != bound_receive_generation_)) {
    progress_ = ReceiptProgress::kRejected;
    return MakeError(ErrorCode::kInvalidState,
                     "STOP_RECEIPT receive generation changed mid-command");
  }
  if (admission.channel() != Channel::kTelemetry ||
      admission.kind() != MessageKind::kStopReceipt ||
      admission.commandId() != receipt.command_id ||
      admission.receiptPhase() != receipt.phase ||
      admission.receiptStatus() != receipt.status) {
    return MakeError(ErrorCode::kInvalidMetadata,
                     "STOP_RECEIPT does not match its admission token");
  }
  if (admission.sessionEpoch() != session_epoch_) {
    progress_ = ReceiptProgress::kRejected;
    return MakeError(ErrorCode::kEpochMismatch,
                     "STOP_RECEIPT epoch does not match active STOP");
  }
  if (admission.bootId() != boot_id_) {
    progress_ = ReceiptProgress::kRejected;
    return MakeError(ErrorCode::kBootMismatch,
                     "STOP_RECEIPT boot does not match active STOP");
  }
  if (progress_ != ReceiptProgress::kAwaitAccepted &&
      progress_ != ReceiptProgress::kAwaitApplied) {
    return MakeError(ErrorCode::kInvalidState,
                     "no active STOP receipt sequence exists");
  }
  if (receipt.command_id != command_id_) {
    return MakeError(ErrorCode::kIdentityMismatch,
                     "STOP_RECEIPT command ID does not match active STOP");
  }
  if (receiver_monotonic_ns > deadline_monotonic_ns_) {
    progress_ = ReceiptProgress::kExpired;
    return MakeError(ErrorCode::kDeadlineExpired,
                     "STOP_RECEIPT arrived after sender deadline");
  }
  if (receipt.phase == ReceiptPhase::kAccepted) {
    if (progress_ != ReceiptProgress::kAwaitAccepted) {
      return MakeError(ErrorCode::kReplay,
                       "ACCEPTED receipt is duplicate or out of order");
    }
    bound_receive_guard_ = admission.issuer_;
    bound_receive_generation_ = admission.receive_generation_;
    progress_ = receipt.status == ReceiptStatus::kRejected
                    ? ReceiptProgress::kRejected
                    : ReceiptProgress::kAwaitApplied;
    return Error{};
  }
  if (receipt.phase == ReceiptPhase::kApplied) {
    if (progress_ != ReceiptProgress::kAwaitApplied) {
      return MakeError(ErrorCode::kInvalidState,
                       "APPLIED receipt requires prior ACCEPTED receipt");
    }
    progress_ = receipt.status == ReceiptStatus::kRejected
                    ? ReceiptProgress::kRejected
                    : ReceiptProgress::kComplete;
    return Error{};
  }
  return MakeError(ErrorCode::kInvalidPayload,
                   "STOP_RECEIPT phase is not defined");
}

Error StopReceiptCorrelator::poll(std::uint64_t receiver_monotonic_ns) {
  if (progress_ != ReceiptProgress::kAwaitAccepted &&
      progress_ != ReceiptProgress::kAwaitApplied) {
    return Error{};
  }
  if (receiver_monotonic_ns > deadline_monotonic_ns_) {
    progress_ = ReceiptProgress::kExpired;
    return MakeError(ErrorCode::kDeadlineExpired,
                     "STOP receipt sequence exceeded its deadline");
  }
  return Error{};
}

} // namespace v2
} // namespace swarm_bridge
} // namespace xgc2
