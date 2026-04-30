#!/usr/bin/env python3
import math

SERIAL_TIMEOUT = 0.1
ADDRESSES = ('A', 'B')
POLL_DELAY = 0.01
CHAR_DELAY = 0.01
COMMAND_DELAY = 0.1
MIN_DEVICE_ROTATE_SPEED = 0     # device units; 0.5 deg/s per unit
MAX_DEVICE_ROTATE_SPEED = 80    # device units; max ~40 deg/s


def encode_get_settings(address: str) -> bytes:
    return (address + '?000').encode()


def encode_set_ccw_limit(address: str, limit: int) -> bytes:
    return (address + 'd' + str(int(limit)).zfill(3)).encode()


def encode_set_cw_limit(address: str, limit: int) -> bytes:
    return (address + 'u' + str(int(limit)).zfill(3)).encode()


def encode_set(address: str, degrees: float, settings: dict) -> bytes | None:
    ccw = settings['factory_ccw_limit']
    cw  = settings['factory_cw_limit']
    if degrees >= 1.0 and degrees <= 359.5:
        counts = math.ceil(degrees / (360.0 / float(cw - ccw)) + float(ccw) + 0.5)
    elif degrees >= 0 and degrees < 0.5:
        counts = ccw
    elif degrees >= 0.5 and degrees < 1.0:
        counts = ccw + 1
    elif degrees > 359.5:
        counts = cw
    else:
        return None
    return (address + 'p' + str(int(counts)).zfill(3)).encode()


def encode_stop(address: str) -> bytes:
    return (address + 's128').encode()


def encode_rotate_cw(address: str, speed: int) -> bytes:
    return (address + '>' + str(int(speed)).zfill(3)).encode()


def encode_rotate_ccw(address: str, speed: int) -> bytes:
    return (address + '<' + str(int(speed)).zfill(3)).encode()


def encode_poll(address: str) -> bytes:
    return (address + 'f').encode()


def decode_settings(data: str) -> dict | None:
    parts = data.split(',')
    if len(parts) < 11:
        return None
    try:
        return {
            'factory_ccw_limit':        int(parts[1]),
            'factory_cw_limit':         int(parts[2]),
            'user_ccw_limit':           int(parts[3]),
            'user_cw_limit':            int(parts[4]),
            'pcb_dash_number':          int(parts[5]),
            'position_feedback_enable': parts[6] == 'y',
            'pcb_serial_number':        int(parts[7]),
            'baud_rate':                int(parts[8]),
            'positioner_type':          int(parts[9]),
            'firmware_revision':        int(parts[10]),
        }
    except (ValueError, IndexError):
        return None


def decode_poll(data: str, address: str, settings: dict) -> float | None:
    if len(data) < 4:
        return None
    if data[0:2] != address + 'f':
        return None
    if data[2] != address:
        return None
    try:
        counts = int(data[3:])
        ccw = settings['factory_ccw_limit']
        cw  = settings['factory_cw_limit']
        return 360.0 * float(counts - ccw) / float(cw - ccw)
    except (ValueError, IndexError):
        return None


def sanitize_speed(speed: int, user_max: int = MAX_DEVICE_ROTATE_SPEED) -> int:
    speed = int(speed)
    speed = max(MIN_DEVICE_ROTATE_SPEED, min(abs(speed), MAX_DEVICE_ROTATE_SPEED))
    return min(speed, user_max)
