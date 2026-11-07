"""Independent protocol v1 reference for firmware golden-frame tests; no serial/ROS runtime."""
import binascii
import struct

VELOCITY, ARM, STOP, STATUS = 1, 2, 3, 0x81
STATES = ("DISABLED", "ENABLING", "ACTIVE", "STOPPING", "FAULT")
FAULTS = ("OK", "COMMAND_TIMEOUT", "LINK_LOST", "MOTOR_ERROR", "ENABLE_TIMEOUT",
          "STOP_TIMEOUT", "INVALID_COMMAND", "NOT_READY")


def encode(kind, sequence, payload=b""):
    if len(payload) > 228:
        raise ValueError("Payload exceeds 228 bytes")
    frame = struct.pack("<2sBBHI", b"\xa5\x5a", 1, kind, len(payload), sequence & 0xFFFFFFFF) + payload
    return frame + struct.pack("<H", binascii.crc_hqx(frame, 0xFFFF))


def velocity(sequence, vx=0.0, vy=0.0, wz=0.0):
    """SI input; firmware holds the latest command without chassis speed scaling."""
    import math
    if not all(math.isfinite(x) for x in (vx, vy, wz)):
        raise ValueError("Velocity must be finite")
    values = tuple(round(x * 1000) for x in (vx, vy, wz))
    if any(x < -2147483648 or x > 2147483647 for x in values):
        raise ValueError("Velocity exceeds int32 milli-unit wire range")
    return encode(VELOCITY, sequence, struct.pack("<iii", *values))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        frames = []
        self.buffer.extend(data)
        while self.buffer:
            start = self.buffer.find(b"\xa5\x5a")
            if start < 0:
                self.buffer[:] = b"\xa5" if self.buffer[-1] == 0xA5 else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 10:
                break
            _, version, kind, length, sequence = struct.unpack_from("<2sBBHI", self.buffer)
            if version != 1 or length > 228:
                del self.buffer[0]
                continue
            total = length + 12
            if len(self.buffer) < total:
                break
            if binascii.crc_hqx(self.buffer[:total - 2], 0xFFFF) != struct.unpack_from("<H", self.buffer, total - 2)[0]:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            frames.append((kind, sequence, bytes(self.buffer[10:total - 2])))
            del self.buffer[:total]
        return frames


def decode_status(payload):
    if len(payload) != 196:
        raise ValueError("Protocol v1 status payload must be 196 bytes")
    u32 = lambda offset: struct.unpack_from("<I", payload, offset)[0]

    def body(offset):
        values = struct.unpack_from("<iii", payload, offset)
        return [None if value == -2147483648 else value / 1000 for value in values]

    result = dict(uptime_ms=u32(0), state=payload[4], fault=payload[5],
                  state_name=STATES[payload[4]] if payload[4] < len(STATES) else "UNKNOWN",
                  fault_name=FAULTS[payload[5]] if payload[5] < len(FAULTS) else "UNKNOWN",
                  connected=bool(payload[6]), motion_allowed=bool(payload[7]),
                  last_command_sequence=u32(8), control_rejected=u32(12), received_frames=u32(16),
                  crc_errors=u32(20), format_errors=u32(24), rx_overflows=u32(28),
                  tx_skipped=u32(32), link_epoch=u32(36),
                  requested=body(40), applied=body(52), measured=body(64), measured_valid=bool(u32(76)),
                  bus_timeouts=u32(80), bus_protocol_errors=u32(84), bus_rejected=u32(88),
                  bus_transport_errors=u32(92), bus_rx_errors=u32(96))
    result["wheels"] = []
    for i in range(4):
        raw, speed, position_ms, flags, enabled_ms = struct.unpack_from("<qiIII", payload, 100 + i * 24)
        wheel = dict(address=i + 1, position_raw=raw,
                     speed_rad_s=None if speed == -2147483648 else speed / 1000,
                     position_ms=position_ms, enabled_ms=enabled_ms)
        for bit, name in enumerate(("position_valid", "speed_valid", "enabled_valid", "enabled",
                                    "stall_valid", "stalled", "protection")):
            wheel[name] = bool(flags & (1 << bit))
        result["wheels"].append(wheel)
    return result
