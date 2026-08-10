# XGC2 swarm ROS bridge protocol v2 core

Status: ROS-free protocol and safety core only. This document does not claim a
network peer, ROS topic adapter, robot driver, package publication, or live
physical closure.

The legacy `bridge_node` wire format remains unchanged. V2 uses a separate C++
namespace (`xgc2::swarm_bridge::v2`), library target
`swarm_ros_bridge_protocol_v2`, and `XSB2` envelope. A peer must never guess a
wire version or feed a v2 frame to the legacy decoder.

## Frozen transport contract

All integers use network byte order (big endian). Text is printable non-space
ASCII and is not NUL terminated. A frame must be consumed atomically; trailing
bytes are invalid.

| Offset | Bytes | Field |
| ---: | ---: | --- |
| 0 | 4 | magic, `0x58534232` (`XSB2`) |
| 4 | 1 | version, exactly `2` |
| 5 | 1 | channel |
| 6 | 1 | message kind |
| 7 | 1 | flags, exactly zero |
| 8 | 2 | total header length |
| 10 | 2 | reserved, exactly zero |
| 12 | 4 | payload length |
| 16 | 8 | per-generation channel sequence |
| 24 | 8 | sender monotonic timestamp, nanoseconds |
| 32 | 8 | source timestamp, nanoseconds; nonzero for ROS payloads |
| 40 | 8 | Session epoch |
| 48 | 8 | closed sender-role capability bitmap |
| 56 | 18 | nine unsigned 16-bit string lengths |
| 74 | variable | nine strings in the order below |
| header length | variable | typed payload |
| final 4 | 4 | CRC32C (Castagnoli) of every preceding frame byte |

The nine header strings are, in order:

1. robot slot ID;
2. asset ID;
3. robot kind;
4. run ID;
5. boot ID;
6. build ID;
7. ROS datatype;
8. ROS MD5;
9. payload schema.

The maximum header is 4,096 bytes, the maximum payload is 1 MiB, and each text
field has its smaller field-specific bound in `protocol.hpp`. CRC32C detects
accidental corruption; it is not authentication. A deployed peer still needs
authenticated transport or a keyed frame authenticator at the product/network
layer.

The capability bitmap is nonzero and closed in this v2 contract:

| Bit | Meaning | Ground | Vehicle |
| ---: | --- | :---: | :---: |
| 0 | ground role | required | forbidden |
| 1 | vehicle role | forbidden | required |
| 2 | independent management/control/telemetry channels | required | required |
| 3 | bounded same-channel cumulative ACK | required | required |
| 4 | ZERO_STOP command origin | required | forbidden |
| 5 | ZERO_STOP apply receipt origin | forbidden | required |

All other bits are unknown and rejected. Exactly one role bit is required;
missing required bits are a downgrade and rejected. The currently allowed set
equals the required set for each role: ground is `0x1d`, vehicle is `0x2e`.
Compatibility requires exactly one ground peer and one vehicle peer. Only the
ground role may originate ZERO_STOP, and only the vehicle role may originate
STOP_RECEIPT. A future optional bit therefore requires a deliberate protocol
contract update on both sides; it cannot be silently ignored.

Channels are management (`1`), control (`2`), and telemetry (`3`). Kinds are
HELLO (`1`), HEARTBEAT (`2`), ZERO_STOP (`3`), STOP_RECEIPT (`4`), and generic
ROS message (`5`). Every channel carries its own HELLO and HEARTBEAT;
ZERO_STOP is control-only, STOP_RECEIPT is telemetry-only, and ROS messages may
use control or telemetry. The current core intentionally contains no socket
code.

## Peer admission and freshness

Each physical channel owns one `ReceiveGuard`, one HELLO, one independent
sequence window, and one freshness clock. Sharing a guard between connections
is forbidden: cross-channel delivery order is not a sequence relation. A
consumer may use `requireThreeChannelFresh` only after assigning management,
control, and telemetry guards to their matching channels. Readiness requires
all three guards to describe the same frozen slot/asset/kind/run/build,
Session epoch, configured capabilities, configured boot, and HELLO-bound boot.
Each channel must also have admitted a HEARTBEAT after its latest HELLO, and
that HEARTBEAT itself must remain within the freshness window; intervening ROS
or control traffic does not refresh heartbeat readiness. A live management
connection therefore cannot mask a dead control or telemetry connection, and
three channels from different peer boots cannot be combined into one ready
peer.

`ReceiveGuard` requires an exact match for slot, asset, robot kind, run, build,
peer role, peer capabilities, local role/capabilities, and Session epoch. A
configured boot ID is exact; otherwise the first valid per-channel HELLO binds
it until an explicit guard reset. The first accepted frame on every channel
must be HELLO. That channel's sequence and sender monotonic time must then
advance. The HELLO sequence may be any positive value, but every subsequent
frame in that generation must be exactly the previous value plus one; gaps,
replay, wraparound, and out-of-order delivery are rejected. Contiguity is what
makes a numeric cumulative ACK unambiguous. A second HELLO, even with the same
boot and epoch, is rejected: reconnect, boot change, epoch change, or handshake
restart requires the owner to install the new frozen contract and explicitly
reset the receive guard and matching send window. No packet performs an
implicit reset.

The two hosts' monotonic clocks are not assumed to share an origin. Before a
guard can admit frames, its owner must supply a `MonotonicTimebase` established
by a bounded clock-sync or challenge/response protocol. It contains sender and
receiver anchors, a maximum error, and a receiver-domain validity deadline.
The codec deliberately does not infer an offset from one-way HELLO arrival.
Every frame is mapped through this external timebase and rejected if its
conservative lower bound is older than 750 ms, its upper bound leads the
receiver by more than 50 ms, or the timebase has expired. Source timestamps
remain data provenance and are not confused with the watchdog clock.

HELLO freezes a 5 Hz heartbeat period (200 ms) and a 750 ms timeout. An epoch
change is rejected by the old receive guard; the owner must install a new
Session contract and reset the guard before accepting a new HELLO.

`Heartbeat.last_received_sequence` is a cumulative ACK for frames sent by the
receiver of that HEARTBEAT on the same `frame.header.channel`. It never refers
to another channel. Each channel owns a bounded `SendWindow` (64 entries by
default, hard maximum 4,096). The window records exact successfully handed-off
local sequence numbers, beginning with the local HELLO. It never evicts an
unacknowledged entry: a full window rejects another record and the runtime must
fail closed. An ACK greater than the last sent sequence is future, lower than
the acknowledged frontier is rollback, and a value between the frontier and
last sent that does not exactly match a retained send is out-of-window or
ambiguous; all are rejected without state mutation. ACK zero is not evidence.
A duplicate of the current nonzero frontier is idempotent, while every actual
frontier update is strictly increasing and removes only the cumulatively
acknowledged prefix.

The `Admission` token binds the HEARTBEAT payload and exact remote identity,
role, capabilities, channel, epoch, and boot before the send window can consume
its ACK. It also carries the issuing receive-guard identity and reset
generation; an ACK admitted after that guard was reset cannot update an old
send window even when boot and epoch strings were accidentally reused.
`bindPeerHello` likewise requires the exact frozen peer HELLO and can run only
once per explicit reset. `requireThreeChannelFresh` remains the one-way
receive-freshness primitive. `requireThreeChannelReady` additionally requires
three correctly assigned send windows, identical complementary role contracts,
the exact current receive-guard generations/peer boots, and a current peer ACK
of each local channel HELLO. It is the ROS-free bidirectional readiness
primitive.

The runtime must serialize `recordSent` with ACK observation and call it only
after an atomic transport handoff succeeds. This library provides the bounded
ledger and checks, not that transport integration. CRC32C and these identity
checks are also not peer authentication.

## Zero-only safety slice

ZERO_STOP schema is `xgc.swarm-bridge.zero-stop.v2`. Its payload is:

| Bytes | Field |
| ---: | --- |
| 8 | sender-domain monotonic deadline |
| 2 | command ID length |
| variable | command ID |
| 48 | six IEEE-754 binary64 axes, network byte order |

Every axis must have the exact all-zero bit pattern. This rejects nonzero
values, NaNs, infinities, subnormals, and negative zero. The deadline must be
between the frame's sender-monotonic timestamp and one second after it. The
receive guard maps the sender deadline to a conservative receiver-domain lower
bound with the established timebase and caps it at the timebase validity
deadline. Transit time is therefore already consumed: receipt does not create
a fresh TTL. An expired or uncertain timebase, a late frame, or a deadline that
has already passed cannot produce an admission token. The two hosts' monotonic
origins are never compared directly. There is deliberately no nonzero command
DTO, encoder, decoder, or positive-motion test in this slice.

`ReceiveGuard::accept` must return an `Admission` token to the next state
machine. There is no overload that silently discards admission. The token
freezes the complete frame header (including sender monotonic/source times,
capabilities, all identity and ROS/schema metadata), receive time, and, where
applicable, command/receipt metadata, the mapped STOP deadline, and the complete
canonical typed payload bytes. A separately supplied frame/DTO must reproduce
that header and payload exactly; retaining a command ID while changing a sender
timestamp, schema, STOP deadline, or receipt observation/detail is rejected.
`StopWatchdog` refuses a raw decoded STOP and cannot emit an OK receipt in
AwaitHello, while stale, after epoch/boot drift, or without a fresh
control-channel heartbeat.

STOP_RECEIPT distinguishes `accepted` from `applied`. The onboard watchdog can
emit `accepted` only after zero-only syntax, identity, freshness, and mapped
deadline admission. It permits `applied` only after a matching accepted command
and before that same mapped deadline. The ground-side
`StopReceiptCorrelator` requires an active command, rejects APPLIED before
ACCEPTED, duplicate/out-of-order phases, command mismatch, token/DTO mismatch,
and receipts after the sender-domain deadline. A robot integration must
withhold `applied` until a fresh typed driver-status observation proves the
local zero hold; this core cannot supply that physical proof.

For a positive epoch, `StopWatchdog` starts in AwaitHello; epoch zero starts in
safe hold. A valid control-channel HELLO moves it to inhibited until an explicit
enable for the same epoch and boot. Heartbeat loss, boot drift, epoch change, or
ZERO_STOP enters safe hold and invalidates that enable. Heartbeats alone never
clear safe hold: recovery requires a newly admitted control HELLO followed by
explicit enable. A deployment must continuously apply zero locally (the
planned robot contract is 20 Hz for at least one second); this core only emits
the `kEnterSafeHold`/`kApplyZeroStop` actions.

## Closed ROS1 typed codec slice

`swarm_ros_bridge_ros1_codec_v2` is a separate ROS-facing library layered on
`swarm_ros_bridge_protocol_v2`. The protocol target and its standalone test
remain ROS-free. The typed library contains no publisher, subscriber, socket,
ROS master call, driver call, or runtime loop.

Telemetry ROS_MESSAGE input/output is a closed allowlist:

| ROS datatype | Frozen ROS MD5 | Frozen payload schema | Maximum serialized bytes |
| --- | --- | --- | ---: |
| `sensor_msgs/Imu` | `6a62c6daae103f4ff57a132d6f95cec2` | `xgc.swarm-bridge.ros1.sensor_msgs.Imu.v1` | 4,096 |
| `nav_msgs/Odometry` | `cd5e73d190d741a2f92e81eda573aca7` | `xgc.swarm-bridge.ros1.nav_msgs.Odometry.v1` | 4,096 |
| `scout_msgs/ScoutStatus` | `7a49e199fd32bf5d7341d653c6b3ba6e` | `xgc.swarm-bridge.ros1.scout_msgs.ScoutStatus.v1` | 4,096 |

Each call checks the generated ROS datatype and MD5 traits against that table,
uses ROS1 `serializationLength`/`serialize`/`deserialize`, rejects truncated or
trailing bytes, and requires a deserialize/serialize byte-exact round trip.
Header frame IDs are bounded to 255 bytes. The source stamp must be normalized,
nonzero, and exactly equal to `frame.header.source_timestamp_ns`; receive wall
time is never substituted for source provenance. A future message-definition
change, including a `ScoutStatus` MD5 change, therefore requires an explicit
contract revision.

There is deliberately no generic public template, generic ROS datatype
registry, or ROS_MESSAGE Twist codec. `geometry_msgs/Twist` can be produced
only by `makeAdmittedPositiveZeroTwist`, which requires a valid
ReceiveGuard-issued control ZERO_STOP Admission and that admission's exact
canonical frame identity and payload. It has no caller-supplied Twist input and
sets all six IEEE-754 values to the positive-zero bit pattern. Negative zero,
NaN, infinity, subnormal, and any nonzero payload fail before Admission.

The status-proof functions are bounded pure window evaluators. Scout evidence
uses `ScoutStatus.linear_velocity`, `angular_velocity`, and all four
`motor_states[].rpm` values; mecanum evidence uses all six
`Odometry.twist.twist` axes. A proof requires a configured count and minimum
source-time span of bit-exact positive-zero samples with strictly advancing
source stamps and a bounded inter-sample gap. Merely clustering the required
sample count at one instant is not proof. The default is 21 inclusive samples
spanning at least one second, with no gap above 100 ms. A nonzero sample or
excessive gap resets the current suffix. These functions neither observe driver
application nor prove physical rest; a later runtime/driver layer must bind
their result and frozen policy to a fresh receipt.

The pinned build dependency is the published XGC2 `scout_msgs 0.3.3-10` for
Melodic/Bionic and Noetic/Focal, with independent amd64/arm64 artifact SHA256
checks before installation. This source slice does not publish or replace that
package.

## Verification

Run:

```bash
test/run_v2_core_tests.sh
```

The gate compiles with C++17 warnings-as-errors, executes semantic and complete
watchdog-transition tests, compares C++ bytes with an independent Python
encoder and frozen HELLO/HEARTBEAT-ACK/ZERO_STOP golden vectors, exercises
truncation/bounds/endianness/CRC and a deterministic malformed corpus
(including CRC-repaired semantic mutations), then repeats under ASan+UBSan,
checks formatting/static analysis, and verifies the DEB/CI release contract.
The package build performs additional install-tree and extracted-DEB consumer
checks for both v2 shared libraries, public headers, role/ACK API, a typed IMU
round trip, and this document. Focused Noetic/Focal and Melodic/Bionic catkin
targets compile and execute the typed codec test without starting ROS nodes.

These gates never open a network socket, start a ROS node, contact a robot, or
exercise a driver. The ROS runtime adapter, authenticated network peer,
deployment configuration, and physical safe-hold proof remain separate
unresolved integration work; passing these codecs does not claim them.
