// SPDX-License-Identifier: BSD-3-Clause

#ifndef SWARM_ROS_BRIDGE_V2_PROTOCOL_HPP_
#define SWARM_ROS_BRIDGE_V2_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace xgc2 {
namespace swarm_bridge {
namespace v2 {

constexpr std::uint32_t kWireMagic = 0x58534232U; // "XSB2"
constexpr std::uint8_t kWireVersion = 2U;
constexpr std::size_t kFixedHeaderBytes = 74U;
constexpr std::size_t kIntegrityBytes = 4U;
constexpr std::size_t kMaxHeaderBytes = 4096U;
constexpr std::size_t kMaxPayloadBytes = 1024U * 1024U;
constexpr std::size_t kMaxFrameBytes =
    kMaxHeaderBytes + kMaxPayloadBytes + kIntegrityBytes;

constexpr std::uint16_t kHeartbeatPeriodMs = 200U;
constexpr std::uint16_t kHeartbeatTimeoutMs = 750U;
constexpr std::uint64_t kHeartbeatTimeoutNs = 750000000ULL;
constexpr std::uint64_t kFutureToleranceNs = 50000000ULL;
constexpr std::uint64_t kMaximumTimebaseErrorNs = 50000000ULL;
constexpr std::uint64_t kMaximumStopDeadlineHorizonNs = 1000000000ULL;
constexpr std::size_t kDefaultSendWindowCapacity = 64U;
constexpr std::size_t kMaximumSendWindowCapacity = 4096U;

constexpr std::size_t kMaxSlotIdBytes = 64U;
constexpr std::size_t kMaxAssetIdBytes = 128U;
constexpr std::size_t kMaxRobotKindBytes = 32U;
constexpr std::size_t kMaxRunIdBytes = 128U;
constexpr std::size_t kMaxBootIdBytes = 64U;
constexpr std::size_t kMaxBuildIdBytes = 128U;
constexpr std::size_t kMaxRosDatatypeBytes = 128U;
constexpr std::size_t kMaxRosMd5Bytes = 32U;
constexpr std::size_t kMaxSchemaBytes = 128U;
constexpr std::size_t kMaxCommandIdBytes = 64U;
constexpr std::size_t kMaxReceiptDetailBytes = 256U;

constexpr const char *kHelloSchema = "xgc.swarm-bridge.hello.v2";
constexpr const char *kHeartbeatSchema = "xgc.swarm-bridge.heartbeat.v2";
constexpr const char *kZeroStopSchema = "xgc.swarm-bridge.zero-stop.v2";
constexpr const char *kStopReceiptSchema = "xgc.swarm-bridge.stop-receipt.v2";

enum class Channel : std::uint8_t {
  kManagement = 1U,
  kControl = 2U,
  kTelemetry = 3U,
};

enum class PeerRole : std::uint8_t {
  kUnspecified = 0U,
  kGround = 1U,
  kVehicle = 2U,
};

// Exactly one role bit is present. Each role currently has a closed
// required/allowed set, so unknown bits and silent capability downgrades are
// rejected instead of being guessed by the peer.
constexpr std::uint64_t kCapabilityRoleGround = 1ULL << 0U;
constexpr std::uint64_t kCapabilityRoleVehicle = 1ULL << 1U;
constexpr std::uint64_t kCapabilityThreeChannel = 1ULL << 2U;
constexpr std::uint64_t kCapabilityPerChannelAck = 1ULL << 3U;
constexpr std::uint64_t kCapabilityZeroStopCommand = 1ULL << 4U;
constexpr std::uint64_t kCapabilityZeroStopApplyReceipt = 1ULL << 5U;
constexpr std::uint64_t kKnownCapabilities =
    kCapabilityRoleGround | kCapabilityRoleVehicle | kCapabilityThreeChannel |
    kCapabilityPerChannelAck | kCapabilityZeroStopCommand |
    kCapabilityZeroStopApplyReceipt;
constexpr std::uint64_t kGroundRequiredCapabilities =
    kCapabilityRoleGround | kCapabilityThreeChannel | kCapabilityPerChannelAck |
    kCapabilityZeroStopCommand;
constexpr std::uint64_t kGroundAllowedCapabilities =
    kGroundRequiredCapabilities;
constexpr std::uint64_t kVehicleRequiredCapabilities =
    kCapabilityRoleVehicle | kCapabilityThreeChannel |
    kCapabilityPerChannelAck | kCapabilityZeroStopApplyReceipt;
constexpr std::uint64_t kVehicleAllowedCapabilities =
    kVehicleRequiredCapabilities;

enum class MessageKind : std::uint8_t {
  kHello = 1U,
  kHeartbeat = 2U,
  kZeroStop = 3U,
  kStopReceipt = 4U,
  kRosMessage = 5U,
};

enum class ReceiptPhase : std::uint8_t {
  kAccepted = 1U,
  kApplied = 2U,
};

enum class ReceiptStatus : std::uint8_t {
  kOk = 1U,
  kRejected = 2U,
};

enum class ErrorCode {
  kNone = 0,
  kNullOutput,
  kFrameTooLarge,
  kTruncated,
  kBadMagic,
  kUnsupportedVersion,
  kInvalidEnum,
  kInvalidFlags,
  kInvalidLength,
  kLimitExceeded,
  kIntegrityMismatch,
  kInvalidIdentity,
  kInvalidMetadata,
  kInvalidPayload,
  kIdentityMismatch,
  kChannelMismatch,
  kCapabilityMismatch,
  kEpochMismatch,
  kBootMismatch,
  kReplay,
  kSequenceGap,
  kOutOfOrderTimestamp,
  kStale,
  kFutureTimestamp,
  kHandshakeRequired,
  kHeartbeatTimeout,
  kUnsafeNonZero,
  kDeadlineExpired,
  kDeadlineTooFar,
  kSendWindowFull,
  kAckFuture,
  kAckRegression,
  kAckOutOfWindow,
  kInvalidState,
};

struct Error {
  ErrorCode code = ErrorCode::kNone;
  std::string detail;

  bool ok() const { return code == ErrorCode::kNone; }
};

const char *errorCodeName(ErrorCode code);
Error validateRoleCapabilities(PeerRole role, std::uint64_t capabilities);
Error validatePeerCompatibility(PeerRole local_role,
                                std::uint64_t local_capabilities,
                                PeerRole peer_role,
                                std::uint64_t peer_capabilities);

struct FrameHeader {
  Channel channel = Channel::kManagement;
  MessageKind kind = MessageKind::kHello;
  std::uint8_t flags = 0U;
  std::uint64_t sequence = 0U;
  std::uint64_t monotonic_ns = 0U;
  std::uint64_t source_timestamp_ns = 0U;
  std::uint64_t session_epoch = 0U;
  std::uint64_t capabilities = 0U;
  std::string slot_id;
  std::string asset_id;
  std::string robot_kind;
  std::string run_id;
  std::string boot_id;
  std::string build_id;
  std::string ros_datatype;
  std::string ros_md5;
  std::string schema;
};

struct Frame {
  FrameHeader header;
  std::vector<std::uint8_t> payload;
};

std::uint32_t crc32c(const std::uint8_t *data, std::size_t size);
Error validateFrame(const Frame &frame);
Error encodeFrame(const Frame &frame, std::vector<std::uint8_t> *output);
Error decodeFrame(const std::vector<std::uint8_t> &wire, Frame *output);

struct Hello {
  std::uint16_t heartbeat_period_ms = kHeartbeatPeriodMs;
  std::uint16_t heartbeat_timeout_ms = kHeartbeatTimeoutMs;
};

struct Heartbeat {
  std::uint64_t last_received_sequence = 0U;
  std::uint8_t safety_state = 0U;
};

struct ZeroStop {
  std::string command_id;
  std::uint64_t deadline_monotonic_ns = 0U;
  std::array<double, 6U> axes{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

struct StopReceipt {
  std::string command_id;
  ReceiptPhase phase = ReceiptPhase::kAccepted;
  ReceiptStatus status = ReceiptStatus::kOk;
  std::uint64_t observed_monotonic_ns = 0U;
  std::string detail;
};

Error encodeHelloPayload(const Hello &hello, std::vector<std::uint8_t> *output);
Error decodeHelloPayload(const std::vector<std::uint8_t> &payload,
                         Hello *output);
Error encodeHeartbeatPayload(const Heartbeat &heartbeat,
                             std::vector<std::uint8_t> *output);
Error decodeHeartbeatPayload(const std::vector<std::uint8_t> &payload,
                             Heartbeat *output);
Error encodeZeroStopPayload(const ZeroStop &stop,
                            std::vector<std::uint8_t> *output);
Error decodeZeroStopPayload(const std::vector<std::uint8_t> &payload,
                            ZeroStop *output);
Error validateStopDeadline(const ZeroStop &stop,
                           std::uint64_t sender_monotonic_ns);
Error encodeStopReceiptPayload(const StopReceipt &receipt,
                               std::vector<std::uint8_t> *output);
Error decodeStopReceiptPayload(const std::vector<std::uint8_t> &payload,
                               StopReceipt *output);

struct ExpectedPeer {
  std::string slot_id;
  std::string asset_id;
  std::string robot_kind;
  std::string run_id;
  std::uint64_t session_epoch = 0U;
  std::string boot_id;
  std::string build_id;
  PeerRole local_role = PeerRole::kUnspecified;
  std::uint64_t local_capabilities = 0U;
  PeerRole peer_role = PeerRole::kUnspecified;
  std::uint64_t capabilities = 0U;
};

struct ReceivePolicy {
  std::uint64_t maximum_age_ns = kHeartbeatTimeoutNs;
  std::uint64_t future_tolerance_ns = kFutureToleranceNs;
};

// This timebase is established outside the codec by a bounded clock-sync or
// challenge/response protocol. The core deliberately does not infer an
// absolute offset from one-way HELLO arrival time.
struct MonotonicTimebase {
  std::uint64_t sender_anchor_ns = 0U;
  std::uint64_t receiver_anchor_ns = 0U;
  std::uint64_t maximum_error_ns = 0U;
  std::uint64_t valid_until_receiver_ns = 0U;
};

class ReceiveGuard;
class SendWindow;

class Admission {
public:
  Admission() = default;

  bool valid() const { return valid_; }
  Channel channel() const { return channel_; }
  MessageKind kind() const { return kind_; }
  std::uint64_t sequence() const { return sequence_; }
  std::uint64_t sessionEpoch() const { return session_epoch_; }
  const std::string &slotId() const { return slot_id_; }
  const std::string &assetId() const { return asset_id_; }
  const std::string &robotKind() const { return robot_kind_; }
  const std::string &runId() const { return run_id_; }
  const std::string &bootId() const { return boot_id_; }
  const std::string &buildId() const { return build_id_; }
  std::uint64_t receivedMonotonicNs() const { return received_monotonic_ns_; }
  std::uint64_t mappedStopDeadlineNs() const {
    return mapped_stop_deadline_ns_;
  }
  const std::string &commandId() const { return command_id_; }
  ReceiptPhase receiptPhase() const { return receipt_phase_; }
  ReceiptStatus receiptStatus() const { return receipt_status_; }
  PeerRole peerRole() const { return peer_role_; }
  std::uint64_t peerCapabilities() const { return peer_capabilities_; }
  bool
  matchesCanonicalPayload(const std::vector<std::uint8_t> &candidate) const {
    return valid_ && canonical_payload_ == candidate;
  }
  bool matchesCanonicalFrame(const Frame &candidate) const;

private:
  friend class ReceiveGuard;
  friend class SendWindow;
  friend class StopReceiptCorrelator;
  friend class StopWatchdog;

  bool valid_ = false;
  Channel channel_ = Channel::kManagement;
  MessageKind kind_ = MessageKind::kHello;
  std::uint64_t sequence_ = 0U;
  std::uint64_t session_epoch_ = 0U;
  std::string slot_id_;
  std::string asset_id_;
  std::string robot_kind_;
  std::string run_id_;
  std::string boot_id_;
  std::string build_id_;
  std::uint64_t received_monotonic_ns_ = 0U;
  std::uint64_t mapped_stop_deadline_ns_ = 0U;
  std::string command_id_;
  ReceiptPhase receipt_phase_ = ReceiptPhase::kAccepted;
  ReceiptStatus receipt_status_ = ReceiptStatus::kOk;
  PeerRole peer_role_ = PeerRole::kUnspecified;
  std::uint64_t peer_capabilities_ = 0U;
  const ReceiveGuard *issuer_ = nullptr;
  std::uint64_t receive_generation_ = 0U;
  FrameHeader canonical_header_;
  std::vector<std::uint8_t> canonical_payload_;
};

class ReceiveGuard {
public:
  ReceiveGuard(ExpectedPeer expected, Channel expected_channel,
               MonotonicTimebase timebase,
               ReceivePolicy policy = ReceivePolicy{});
  ReceiveGuard(const ReceiveGuard &) = delete;
  ReceiveGuard &operator=(const ReceiveGuard &) = delete;
  ReceiveGuard(ReceiveGuard &&) = delete;
  ReceiveGuard &operator=(ReceiveGuard &&) = delete;

  Error accept(const Frame &frame, std::uint64_t receiver_monotonic_ns,
               Admission *admission);
  Error checkFresh(std::uint64_t receiver_monotonic_ns) const;
  void reset(ExpectedPeer expected, MonotonicTimebase timebase);
  bool established() const { return established_; }
  std::uint64_t lastSequence() const { return last_sequence_; }
  const std::string &boundBootId() const { return bound_boot_id_; }
  Channel expectedChannel() const { return expected_channel_; }

private:
  friend Error requireThreeChannelFresh(const ReceiveGuard &management,
                                        const ReceiveGuard &control,
                                        const ReceiveGuard &telemetry,
                                        std::uint64_t receiver_monotonic_ns);
  friend class SendWindow;
  friend Error requireThreeChannelReady(const ReceiveGuard &management,
                                        const ReceiveGuard &control,
                                        const ReceiveGuard &telemetry,
                                        const SendWindow &management_send,
                                        const SendWindow &control_send,
                                        const SendWindow &telemetry_send,
                                        std::uint64_t receiver_monotonic_ns);

  ExpectedPeer expected_;
  Channel expected_channel_ = Channel::kManagement;
  MonotonicTimebase timebase_;
  ReceivePolicy policy_;
  bool established_ = false;
  std::string bound_boot_id_;
  std::uint64_t last_sequence_ = 0U;
  std::uint64_t last_sender_monotonic_ns_ = 0U;
  std::uint64_t last_receiver_monotonic_ns_ = 0U;
  std::uint64_t last_heartbeat_receiver_monotonic_ns_ = 0U;
  std::uint64_t generation_ = 1U;
};

Error requireThreeChannelFresh(const ReceiveGuard &management,
                               const ReceiveGuard &control,
                               const ReceiveGuard &telemetry,
                               std::uint64_t receiver_monotonic_ns);

enum class AckDisposition {
  kAdvanced = 0,
  kDuplicate,
};

struct SendWindowConfig {
  Channel channel = Channel::kManagement;
  PeerRole local_role = PeerRole::kUnspecified;
  std::uint64_t local_capabilities = 0U;
  PeerRole peer_role = PeerRole::kUnspecified;
  std::uint64_t peer_capabilities = 0U;
  std::uint64_t session_epoch = 0U;
  std::string local_boot_id;
  std::string peer_slot_id;
  std::string peer_asset_id;
  std::string peer_robot_kind;
  std::string peer_run_id;
  std::string peer_boot_id;
  std::string peer_build_id;
  std::size_t capacity = kDefaultSendWindowCapacity;
};

// One SendWindow belongs to one physical bidirectional channel. recordSent
// must be serialized with receive processing and called only after an atomic
// transport handoff succeeds; the ROS-free core cannot enforce that runtime
// integration rule itself.
class SendWindow {
public:
  explicit SendWindow(SendWindowConfig config);
  SendWindow(const SendWindow &) = delete;
  SendWindow &operator=(const SendWindow &) = delete;
  SendWindow(SendWindow &&) = delete;
  SendWindow &operator=(SendWindow &&) = delete;

  Error recordSent(const Frame &frame);
  Error bindPeerHello(const Admission &admission);
  Error observeHeartbeat(const Heartbeat &heartbeat, const Admission &admission,
                         AckDisposition *disposition);
  void reset(SendWindowConfig config);

  bool outboundEstablished() const { return outbound_established_; }
  bool peerHelloBound() const { return peer_hello_bound_; }
  std::uint64_t acknowledgedFrontier() const { return acknowledged_frontier_; }
  std::uint64_t lastSentSequence() const { return last_sent_sequence_; }
  std::size_t outstandingCount() const { return outstanding_.size(); }
  Channel channel() const { return config_.channel; }

private:
  friend Error requireThreeChannelReady(const ReceiveGuard &management,
                                        const ReceiveGuard &control,
                                        const ReceiveGuard &telemetry,
                                        const SendWindow &management_send,
                                        const SendWindow &control_send,
                                        const SendWindow &telemetry_send,
                                        std::uint64_t receiver_monotonic_ns);

  SendWindowConfig config_;
  bool outbound_established_ = false;
  bool peer_hello_bound_ = false;
  std::string local_slot_id_;
  std::string local_asset_id_;
  std::string local_robot_kind_;
  std::string local_run_id_;
  std::string local_build_id_;
  std::string bound_peer_boot_id_;
  const ReceiveGuard *bound_receive_guard_ = nullptr;
  std::uint64_t bound_receive_generation_ = 0U;
  std::uint64_t hello_sequence_ = 0U;
  std::uint64_t last_sent_sequence_ = 0U;
  std::uint64_t acknowledged_frontier_ = 0U;
  std::vector<std::uint64_t> outstanding_;
};

// This stronger readiness check proves current bidirectional progress: every
// peer HEARTBEAT cumulatively acknowledged at least this generation's local
// HELLO on its matching channel.
Error requireThreeChannelReady(const ReceiveGuard &management,
                               const ReceiveGuard &control,
                               const ReceiveGuard &telemetry,
                               const SendWindow &management_send,
                               const SendWindow &control_send,
                               const SendWindow &telemetry_send,
                               std::uint64_t receiver_monotonic_ns);

enum class LinkState {
  kAwaitHello = 0,
  kInhibited,
  kEnabled,
  kSafeHold,
};

enum class SafetyAction {
  kNone = 0,
  kEnterSafeHold,
  kApplyZeroStop,
};

class StopWatchdog {
public:
  explicit StopWatchdog(std::uint64_t session_epoch);

  Error onHello(const Admission &admission, std::uint64_t receiver_monotonic_ns,
                SafetyAction *action);
  Error onHeartbeat(const Admission &admission,
                    std::uint64_t receiver_monotonic_ns, SafetyAction *action);
  Error reenable(std::uint64_t session_epoch, const std::string &boot_id,
                 std::uint64_t receiver_monotonic_ns);
  Error poll(std::uint64_t receiver_monotonic_ns, SafetyAction *action);
  Error onEpochChanged(std::uint64_t new_session_epoch, SafetyAction *action);
  Error acceptZeroStop(const ZeroStop &stop, const Admission &admission,
                       std::uint64_t receiver_monotonic_ns,
                       StopReceipt *accepted_receipt, SafetyAction *action);
  Error confirmStopApplied(const std::string &command_id,
                           std::uint64_t receiver_monotonic_ns,
                           StopReceipt *applied_receipt);

  LinkState state() const { return state_; }
  std::uint64_t sessionEpoch() const { return session_epoch_; }
  const std::string &bootId() const { return boot_id_; }

private:
  void enterSafeHold();

  LinkState state_ = LinkState::kAwaitHello;
  std::uint64_t session_epoch_ = 0U;
  std::uint64_t last_signal_monotonic_ns_ = 0U;
  std::uint64_t last_admission_sequence_ = 0U;
  std::string boot_id_;
  const ReceiveGuard *bound_receive_guard_ = nullptr;
  std::uint64_t bound_receive_generation_ = 0U;
  std::string pending_stop_command_id_;
  std::uint64_t pending_stop_accepted_ns_ = 0U;
  std::uint64_t pending_stop_receiver_deadline_ns_ = 0U;
};

enum class ReceiptProgress {
  kIdle = 0,
  kAwaitAccepted,
  kAwaitApplied,
  kComplete,
  kRejected,
  kExpired,
};

class StopReceiptCorrelator {
public:
  StopReceiptCorrelator(std::uint64_t session_epoch, std::string boot_id);

  Error begin(const ZeroStop &stop, std::uint64_t sender_monotonic_ns);
  Error observe(const StopReceipt &receipt, const Admission &admission,
                std::uint64_t receiver_monotonic_ns);
  Error poll(std::uint64_t receiver_monotonic_ns);

  ReceiptProgress progress() const { return progress_; }
  bool complete() const { return progress_ == ReceiptProgress::kComplete; }

private:
  std::uint64_t session_epoch_ = 0U;
  std::string boot_id_;
  ReceiptProgress progress_ = ReceiptProgress::kIdle;
  std::string command_id_;
  std::uint64_t deadline_monotonic_ns_ = 0U;
  const ReceiveGuard *bound_receive_guard_ = nullptr;
  std::uint64_t bound_receive_generation_ = 0U;
};

} // namespace v2
} // namespace swarm_bridge
} // namespace xgc2

#endif // SWARM_ROS_BRIDGE_V2_PROTOCOL_HPP_
