#!/usr/bin/env python3
"""
Drag-cancellation "frictionless flywheel" mode for a VESC.

The other VESC drives the shared shaft. This script reads our motor's RPM and
applies a small forward current to cancel our own internal losses (Coulomb +
viscous), so the shaft behaves as if our motor weren't on it.

Loop: measure rpm -> i_cmd = sign(rpm) * (I_STATIC + K_VISCOUS * |rpm|),
clamped to I_MAX, deadbanded near zero. SetCurrent at CONTROL_RATE_HZ.

Tune I_STATIC and K_VISCOUS up until the rotor coasts without slowing.
Stop: Ctrl+C — sends SetCurrent(0) and the motor freewheels.
"""

import serial
import struct
import time

PORT = "/dev/cu.usbmodem3041"
BAUD = 115200

# --- Drag-cancellation tuning ---
# Start conservative; raise both until the shaft coasts without decelerating.
# If the motor starts spinning on its own when the shaft is at rest, lower
# I_STATIC or raise DEADBAND_ERPM.
I_STATIC_A = 1.0              # Coulomb friction comp (amps), forward in dir of motion
K_VISCOUS_A_PER_ERPM = 1.5e-4   # viscous friction comp (amps per ERPM)
I_MAX_A = 6.0                 # safety clamp on commanded current
DEADBAND_ERPM = 100           # below |rpm| this, command 0 (don't self-start)

# --- Loop timing ---
CONTROL_RATE_HZ = 100         # how often we update SetCurrent — must be >> 1Hz watchdog
TELEMETRY_INTERVAL_S = 0.5    # how often to read+print GetValues

# --- VESC commands (subset of mc_interface COMM_PACKET_ID) ---
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8


# --- CRC16-CCITT (poly 0x1021, init 0x0000), VESC variant ---
def crc16_ccitt(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_packet(payload: bytes) -> bytes:
    """Wrap payload in a VESC frame."""
    n = len(payload)
    if n <= 255:
        header = bytes([0x02, n])
    elif n <= 65535:
        header = bytes([0x03]) + struct.pack(">H", n)
    else:
        raise ValueError("payload too large")
    crc = crc16_ccitt(payload)
    return header + payload + struct.pack(">H", crc) + b"\x03"


def parse_packet(buf: bytearray):
    """
    Try to parse one VESC packet from the start of buf.
    Returns (payload_bytes_or_None, bytes_consumed).
    """
    if not buf:
        return None, 0
    if buf[0] == 0x02:
        if len(buf) < 2:
            return None, 0
        n = buf[1]
        header_len = 2
    elif buf[0] == 0x03:
        if len(buf) < 3:
            return None, 0
        n = struct.unpack(">H", bytes(buf[1:3]))[0]
        header_len = 3
    else:
        # Resync: drop one byte
        return None, 1

    total = header_len + n + 2 + 1  # payload + crc + stop
    if len(buf) < total:
        return None, 0
    payload = bytes(buf[header_len:header_len + n])
    crc_rx = struct.unpack(">H", bytes(buf[header_len + n:header_len + n + 2]))[0]
    stop = buf[header_len + n + 2]
    if stop != 0x03 or crc16_ccitt(payload) != crc_rx:
        return None, 1  # malformed — drop one byte and retry
    return payload, total


# --- Command builders ---
def set_rpm(rpm: int) -> bytes:
    return encode_packet(struct.pack(">Bi", COMM_SET_RPM, int(rpm)))


def set_current(amps: float) -> bytes:
    # VESC wants milliamps as int32
    return encode_packet(struct.pack(">Bi", COMM_SET_CURRENT, int(amps * 1000)))


def request_values() -> bytes:
    return encode_packet(bytes([COMM_GET_VALUES]))


# --- GetValues response parsing (FW 5.x layout, fields we care about) ---
class Values:
    __slots__ = ("temp_fet", "temp_motor", "i_motor", "i_in", "duty",
                 "rpm", "v_in", "fault")


def parse_values(payload: bytes):
    if not payload or payload[0] != COMM_GET_VALUES:
        return None
    p = payload[1:]
    # Layout (big-endian):
    # int16 temp_fet/10, int16 temp_motor/10,
    # int32 i_motor/100, int32 i_in/100,
    # int32 id/100, int32 iq/100,
    # int16 duty/1000, int32 rpm,
    # int16 v_in/10, ... (more fields after; we stop here)
    try:
        (t_fet, t_mot, i_mot, i_in, _id, _iq, duty, rpm, v_in) = struct.unpack_from(
            ">hhiiiihih", p, 0
        )
    except struct.error:
        return None
    fault = p[28] if len(p) > 28 else 0  # rough — fault byte sits past v_in in FW 5.x
    v = Values()
    v.temp_fet = t_fet / 10.0
    v.temp_motor = t_mot / 10.0
    v.i_motor = i_mot / 100.0
    v.i_in = i_in / 100.0
    v.duty = duty / 1000.0
    v.rpm = rpm
    v.v_in = v_in / 10.0
    v.fault = fault
    return v


# --- Serial helpers ---
def read_telemetry(ser, timeout_s=0.2):
    ser.reset_input_buffer()
    ser.write(request_values())
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
            payload, consumed = parse_packet(buf)
            if payload is not None:
                del buf[:consumed]
                return parse_values(payload)
            if consumed:
                del buf[:consumed]
        else:
            time.sleep(0.01)
    return None


def drag_compensation_amps(rpm):
    """Forward current that cancels our own friction at this RPM."""
    if abs(rpm) < DEADBAND_ERPM:
        return 0.0
    sign = 1.0 if rpm > 0 else -1.0
    i = sign * (I_STATIC_A + K_VISCOUS_A_PER_ERPM * abs(rpm))
    if i > I_MAX_A:
        i = I_MAX_A
    elif i < -I_MAX_A:
        i = -I_MAX_A
    return i


def main():
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        v = read_telemetry(ser)
        if not v:
            print("No response from VESC — check port and power.")
            return
        print(f"Connected. V_in={v.v_in:.2f}V  RPM={v.rpm}  FET={v.temp_fet:.1f}°C")
        print(
            f"Drag-cancel: I_static={I_STATIC_A}A  K_viscous={K_VISCOUS_A_PER_ERPM}A/ERPM  "
            f"deadband=±{DEADBAND_ERPM} ERPM  I_max={I_MAX_A}A"
        )
        print("Spin the shaft with the other VESC. Ctrl+C to stop.")

        period = 1.0 / CONTROL_RATE_HZ
        last_print = 0.0
        last_rpm = 0
        try:
            while True:
                tick = time.monotonic()

                # Telemetry tick: read RPM + print. Quick reads only.
                if tick - last_print >= TELEMETRY_INTERVAL_S:
                    t = read_telemetry(ser)
                    if t is not None:
                        last_rpm = t.rpm
                        i_cmd = drag_compensation_amps(last_rpm)
                        print(
                            f"RPM={t.rpm:>7}  i_cmd={i_cmd:+5.2f}A  "
                            f"i_motor={t.i_motor:+5.2f}A  V_in={t.v_in:5.2f}V  "
                            f"FET={t.temp_fet:5.1f}°C"
                        )
                    last_print = tick
                else:
                    # Fast path: quick GetValues to update RPM without printing.
                    t = read_telemetry(ser, timeout_s=0.05)
                    if t is not None:
                        last_rpm = t.rpm

                ser.write(set_current(drag_compensation_amps(last_rpm)))

                # Sleep the remainder of the control period
                elapsed = time.monotonic() - tick
                if elapsed < period:
                    time.sleep(period - elapsed)
        except KeyboardInterrupt:
            print("\nReleasing motor (SetCurrent 0). Coasting.")
            ser.write(set_current(0))


if __name__ == "__main__":
    main()
