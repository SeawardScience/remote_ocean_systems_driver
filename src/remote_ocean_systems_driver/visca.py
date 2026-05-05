#!/usr/bin/env python3

MAX_VISCA_SPEED = 0x18   # 24 deg/s
MIN_VISCA_SPEED = 0x01   # ~0.2 deg/s


def _to_nibbles(val: int) -> list:
    val = int(val) & 0xFFFF
    return [(val >> 12) & 0xF, (val >> 8) & 0xF, (val >> 4) & 0xF, val & 0xF]


def _from_nibbles(data: bytes) -> int:
    return (data[0] << 12) | (data[1] << 8) | (data[2] << 4) | data[3]


def _response_byte(address: int) -> int:
    return ((address + 8) & 0xF) << 4


def deg_to_val(degrees: float) -> int:
    return int(round(degrees * 65536.0 / 360.0)) & 0xFFFF


def val_to_deg(val: int) -> float:
    return (val & 0xFFFF) * 360.0 / 65536.0


def int32_to_visca_speed(speed: int) -> int:
    return max(MIN_VISCA_SPEED, min(MAX_VISCA_SPEED, round(abs(speed) * MAX_VISCA_SPEED / 80)))


def encode_get_position(address: int) -> bytes:
    return bytes([0x80 | address, 0x09, 0x06, 0x12, 0xFF])


def encode_cancel(address: int) -> bytes:
    return bytes([0x80 | address, 0x21, 0xFF])


def encode_drive_open_loop(address: int, pan_speed: int, tilt_speed: int) -> bytes:
    pan_abs  = max(0, min(MAX_VISCA_SPEED, abs(pan_speed)))
    tilt_abs = max(0, min(MAX_VISCA_SPEED, abs(tilt_speed)))
    pan_dir  = 0x03 if pan_speed == 0 else (0x02 if pan_speed > 0 else 0x01)
    tilt_dir = 0x03 if tilt_speed == 0 else (0x01 if tilt_speed > 0 else 0x02)
    return bytes([0x80 | address, 0x01, 0x06, 0x01,
                  pan_abs, tilt_abs, pan_dir, tilt_dir, 0xFF])


def encode_drive_absolute(address: int, pan_deg: float, tilt_deg: float,
                          pan_spd: int = 6, tilt_spd: int = 6) -> bytes:
    pan_spd  = max(MIN_VISCA_SPEED, min(MAX_VISCA_SPEED, pan_spd))
    tilt_spd = max(MIN_VISCA_SPEED, min(MAX_VISCA_SPEED, tilt_spd))
    data = [0x80 | address, 0x01, 0x06, 0x02, pan_spd, tilt_spd]
    data.extend(_to_nibbles(deg_to_val(pan_deg)))
    data.extend(_to_nibbles(deg_to_val(tilt_deg)))
    data.append(0xFF)
    return bytes(data)


def encode_get_position_limit(address: int, direction: int) -> bytes:
    """direction: 1 = up/right, 0 = down/left"""
    return bytes([0x80 | address, 0x09, 0x06, 0x13, direction & 0x0F, 0xFF])


def encode_set_position_limit(address: int, direction: int,
                               pan_deg: float, tilt_deg: float) -> bytes:
    """direction: 1 = up/right, 0 = down/left"""
    data = [0x80 | address, 0x01, 0x06, 0x07, 0x00, direction & 0x0F]
    data.extend(_to_nibbles(deg_to_val(pan_deg)))
    data.extend(_to_nibbles(deg_to_val(tilt_deg)))
    data.append(0xFF)
    return bytes(data)


def encode_clear_position_limits(address: int) -> bytes:
    """Clears both directions; 0x7FFF is the full-range sentinel per OEM docs."""
    return bytes([0x80 | address, 0x01, 0x06, 0x07, 0x01, 0x00,
                  0x07, 0x0F, 0x0F, 0x0F, 0x07, 0x0F, 0x0F, 0x0F, 0xFF])


def decode_position(packet: bytes, address: int):
    """Parse a complete Visca packet (including 0xFF terminator).
    Returns (pan_deg, tilt_deg) or None if not a position response.
    """
    resp = _response_byte(address)
    # Must be at least: resp_byte 0x50 + 8 nibble bytes + 0xFF = 11 bytes
    if len(packet) < 11:
        return None
    if packet[0] != resp or packet[1] != 0x50:
        return None
    pan_deg  = val_to_deg(_from_nibbles(packet[2:6]))
    tilt_deg = val_to_deg(_from_nibbles(packet[6:10]))
    return pan_deg, tilt_deg
