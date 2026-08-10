#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause

"""Independent Python encoder for frozen swarm_ros_bridge v2 vectors."""

import pathlib
import struct
import subprocess
import sys


MAGIC = 0x58534232
VERSION = 2
FIXED_HEADER_BYTES = 74


def crc32c(data):
    """Bitwise Castagnoli implementation, independent of the C++ code."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return (~crc) & 0xFFFFFFFF


def encode_frame(
    channel, kind, sequence, monotonic_ns, epoch, capabilities, schema, payload
):
    strings = [
        b"scout-01",
        b"asset-a",
        b"scout",
        b"run-42",
        b"boot-a",
        b"build-9",
        b"",
        b"",
        schema.encode("ascii"),
    ]
    header_length = FIXED_HEADER_BYTES + sum(map(len, strings))
    prefix = struct.pack(
        ">IBBBBHHIQQQQQ",
        MAGIC,
        VERSION,
        channel,
        kind,
        0,
        header_length,
        0,
        len(payload),
        sequence,
        monotonic_ns,
        0,
        epoch,
        capabilities,
    )
    lengths = struct.pack(">9H", *(len(value) for value in strings))
    without_crc = prefix + lengths + b"".join(strings) + payload
    assert len(prefix) + len(lengths) == FIXED_HEADER_BYTES
    assert len(without_crc) == header_length + len(payload)
    return without_crc + struct.pack(">I", crc32c(without_crc))


def reference_vectors():
    hello_payload = struct.pack(">HHI", 200, 750, 0)
    heartbeat_payload = struct.pack(">QB7x", 0x0102030405060708, 2)
    stop_payload = (
        struct.pack(">QH", 0x4142434445464748, len(b"stop-0001"))
        + b"stop-0001"
        + struct.pack(">6Q", 0, 0, 0, 0, 0, 0)
    )
    return {
        "hello": encode_frame(
            1,
            1,
            0x0102030405060708,
            0x1112131415161718,
            0x2122232425262728,
            0x2E,
            "xgc.swarm-bridge.hello.v2",
            hello_payload,
        ),
        "heartbeat": encode_frame(
            1,
            2,
            0x0203040506070809,
            0x1112131415161720,
            0x2122232425262728,
            0x2E,
            "xgc.swarm-bridge.heartbeat.v2",
            heartbeat_payload,
        ),
        "stop": encode_frame(
            2,
            3,
            0x090A0B0C0D0E0F10,
            0x1112131415161728,
            0x2122232425262728,
            0x1D,
            "xgc.swarm-bridge.zero-stop.v2",
            stop_payload,
        ),
    }


def parse_vectors(text):
    vectors = {}
    for line in text.splitlines():
        name, encoded = line.split(" ", 1)
        vectors[name] = bytes.fromhex(encoded)
    return vectors


def main():
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: v2_reference_encoder_test.py "
            "/path/to/v2_protocol_test /path/to/golden_v2_vectors.txt"
        )
    binary = pathlib.Path(sys.argv[1])
    golden_path = pathlib.Path(sys.argv[2])
    reference = reference_vectors()
    frozen = parse_vectors(golden_path.read_text(encoding="ascii"))
    cpp = parse_vectors(
        subprocess.check_output(
            [str(binary), "--emit-golden"], universal_newlines=True, timeout=10
        )
    )
    if reference != frozen:
        raise AssertionError("independent Python encoding diverges from golden bytes")
    if cpp != frozen:
        raise AssertionError("C++ encoding diverges from golden bytes")
    print("independent Python/C++ v2 golden vectors match")


if __name__ == "__main__":
    main()
