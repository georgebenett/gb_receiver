#!/usr/bin/env python3
"""
USB Serial Test Script for gb_receiver
Tests the binary protocol communication over USB serial
"""

import serial
import serial.tools.list_ports
import time
import struct
from typing import Optional, Tuple

# Protocol constants
PACKET_START_BYTE = 0xAA
PACKET_MAX_PAYLOAD_SIZE = 512

# Command IDs
CMD_PING = 0x01
CMD_GET_FIRMWARE_VERSION = 0x02
CMD_GET_CONFIG = 0x03
CMD_START_STREAMING = 0x10
CMD_STOP_STREAMING = 0x11
CMD_SET_STREAM_RATE = 0x12

# Response IDs
RSP_ACK = 0x80
RSP_ERROR = 0x81
RSP_FIRMWARE_VERSION = 0x82
RSP_CONFIG = 0x83
RSP_STREAM_DATA = 0x90

# Error codes
ERR_OK = 0x00
ERR_UNKNOWN_CMD = 0x01
ERR_INVALID_PAYLOAD = 0x02
ERR_CRC_MISMATCH = 0x03
ERR_OUT_OF_RANGE = 0x07
ERR_NOT_SUPPORTED = 0x08

ERROR_NAMES = {
    ERR_OK: "OK",
    ERR_UNKNOWN_CMD: "UNKNOWN_CMD",
    ERR_INVALID_PAYLOAD: "INVALID_PAYLOAD",
    ERR_CRC_MISMATCH: "CRC_MISMATCH",
    ERR_OUT_OF_RANGE: "OUT_OF_RANGE",
    ERR_NOT_SUPPORTED: "NOT_SUPPORTED",
}


def calculate_crc16(data: bytes) -> int:
    """Calculate CRC-16-CCITT (polynomial: 0x1021)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc


def build_packet(cmd_id: int, payload: bytes = b'') -> bytes:
    """Build a binary protocol packet"""
    if len(payload) > PACKET_MAX_PAYLOAD_SIZE:
        raise ValueError(f"Payload too large: {len(payload)} bytes")
    
    # Build packet: [START][CMD][LEN_LSB][LEN_MSB][PAYLOAD][CRC_LSB][CRC_MSB]
    packet = bytearray()
    packet.append(PACKET_START_BYTE)
    packet.append(cmd_id)
    packet.append(len(payload) & 0xFF)  # LEN_LSB
    packet.append((len(payload) >> 8) & 0xFF)  # LEN_MSB
    
    if payload:
        packet.extend(payload)
    
    # Calculate CRC over [CMD][LEN][PAYLOAD]
    crc_data = packet[1:]  # Skip START byte
    crc = calculate_crc16(crc_data)
    packet.append(crc & 0xFF)  # CRC_LSB
    packet.append((crc >> 8) & 0xFF)  # CRC_MSB
    
    return bytes(packet)


def parse_packet(data: bytes) -> Optional[Tuple[int, bytes]]:
    """Parse a received packet, returns (cmd_id, payload) or None if invalid"""
    if len(data) < 6:  # Minimum packet size
        return None
    
    if data[0] != PACKET_START_BYTE:
        return None
    
    cmd_id = data[1]
    payload_len = data[2] | (data[3] << 8)
    
    if len(data) < 6 + payload_len:
        return None
    
    payload = data[4:4+payload_len]
    received_crc = data[4+payload_len] | (data[5+payload_len] << 8)
    
    # Calculate expected CRC
    crc_data = data[1:4+payload_len]  # CMD + LEN + PAYLOAD
    expected_crc = calculate_crc16(crc_data)
    
    if received_crc != expected_crc:
        print(f"  CRC mismatch: expected 0x{expected_crc:04X}, got 0x{received_crc:04X}")
        return None
    
    return (cmd_id, payload)


def send_command(ser: serial.Serial, cmd_id: int, payload: bytes = b'', timeout: float = 1.0) -> Optional[Tuple[int, bytes]]:
    """Send a command and wait for response"""
    packet = build_packet(cmd_id, payload)
    
    print(f"  Sending: CMD=0x{cmd_id:02X}, payload_len={len(payload)}")
    ser.write(packet)
    ser.flush()
    
    # Wait for response
    start_time = time.time()
    buffer = bytearray()
    
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
            
            # Try to parse packet
            result = parse_packet(bytes(buffer))
            if result is not None:
                return result
            
            # If buffer is getting too large, something is wrong
            if len(buffer) > 1024:
                print(f"  Buffer overflow, clearing...")
                buffer.clear()
        
        time.sleep(0.01)
    
    print(f"  Timeout waiting for response")
    return None


def test_ping(ser: serial.Serial) -> bool:
    """Test ping command"""
    print("\n[TEST] Ping...")
    result = send_command(ser, CMD_PING)
    
    if result is None:
        print("  FAILED: No response")
        return False
    
    cmd_id, payload = result
    
    if cmd_id != RSP_ACK:
        print(f"  FAILED: Expected ACK (0x{RSP_ACK:02X}), got 0x{cmd_id:02X}")
        return False
    
    if len(payload) < 2:
        print("  FAILED: ACK payload too short")
        return False
    
    original_cmd = payload[0]
    error_code = payload[1]
    
    if original_cmd != CMD_PING:
        print(f"  FAILED: ACK for wrong command (0x{original_cmd:02X})")
        return False
    
    if error_code != ERR_OK:
        error_name = ERROR_NAMES.get(error_code, f"UNKNOWN(0x{error_code:02X})")
        print(f"  FAILED: Error code {error_name}")
        return False
    
    print("  SUCCESS: Ping responded correctly")
    return True


def test_get_firmware_version(ser: serial.Serial) -> bool:
    """Test get firmware version command"""
    print("\n[TEST] Get Firmware Version...")
    result = send_command(ser, CMD_GET_FIRMWARE_VERSION, timeout=2.0)
    
    if result is None:
        print("  FAILED: No response")
        return False
    
    cmd_id, payload = result
    
    if cmd_id != RSP_FIRMWARE_VERSION:
        print(f"  FAILED: Expected FIRMWARE_VERSION (0x{RSP_FIRMWARE_VERSION:02X}), got 0x{cmd_id:02X}")
        return False
    
    if len(payload) < 3:
        print("  FAILED: Payload too short")
        return False
    
    idx = 0
    major = payload[idx]
    idx += 1
    minor = payload[idx]
    idx += 1
    patch = payload[idx]
    idx += 1
    
    # Build date
    if idx >= len(payload):
        print("  FAILED: Missing build date")
        return False
    build_date_len = payload[idx]
    idx += 1
    if idx + build_date_len > len(payload):
        print("  FAILED: Invalid build date length")
        return False
    build_date = payload[idx:idx+build_date_len].decode('utf-8', errors='ignore')
    idx += build_date_len
    
    # Model name
    if idx >= len(payload):
        print("  FAILED: Missing model name")
        return False
    model_len = payload[idx]
    idx += 1
    if idx + model_len > len(payload):
        print("  FAILED: Invalid model name length")
        return False
    model_name = payload[idx:idx+model_len].decode('utf-8', errors='ignore')
    idx += model_len
    
    # IDF version
    if idx >= len(payload):
        print("  FAILED: Missing IDF version")
        return False
    idf_len = payload[idx]
    idx += 1
    if idx + idf_len > len(payload):
        print("  FAILED: Invalid IDF version length")
        return False
    idf_version = payload[idx:idx+idf_len].decode('utf-8', errors='ignore')
    
    print(f"  SUCCESS:")
    print(f"    Version: {major}.{minor}.{patch}")
    print(f"    Build: {build_date}")
    print(f"    Model: {model_name}")
    print(f"    IDF: {idf_version}")
    
    return True


def test_get_config(ser: serial.Serial) -> bool:
    """Test get config command"""
    print("\n[TEST] Get Config...")
    result = send_command(ser, CMD_GET_CONFIG)
    
    if result is None:
        print("  FAILED: No response")
        return False
    
    cmd_id, payload = result
    
    if cmd_id != RSP_CONFIG:
        print(f"  FAILED: Expected CONFIG (0x{RSP_CONFIG:02X}), got 0x{cmd_id:02X}")
        return False
    
    if len(payload) < 3:
        print("  FAILED: Payload too short")
        return False
    
    flags = payload[0]
    throttle_value = payload[1] | (payload[2] << 8)
    
    ble_connected = bool(flags & 0x01)
    
    print(f"  SUCCESS:")
    print(f"    BLE Connected: {ble_connected}")
    print(f"    Throttle Value: {throttle_value}")
    
    return True


def test_streaming(ser: serial.Serial, duration: float = 5.0) -> bool:
    """Test streaming functionality"""
    print(f"\n[TEST] Streaming (duration: {duration}s)...")
    
    # Start streaming at 10Hz
    rate_hz = 10
    payload = struct.pack('<H', rate_hz)  # Little-endian uint16
    result = send_command(ser, CMD_START_STREAMING, payload)
    
    if result is None:
        print("  FAILED: No response to START_STREAMING")
        return False
    
    cmd_id, ack_payload = result
    if cmd_id != RSP_ACK or len(ack_payload) < 2 or ack_payload[1] != ERR_OK:
        print("  FAILED: START_STREAMING not acknowledged")
        return False
    
    print(f"  Streaming started at {rate_hz}Hz, receiving data...")
    
    # Collect stream data
    start_time = time.time()
    packet_count = 0
    buffer = bytearray()
    
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            buffer.extend(ser.read(ser.in_waiting))
            
            # Try to parse packets
            while len(buffer) >= 6:
                result = parse_packet(bytes(buffer))
                if result is None:
                    # Try to find start byte
                    try:
                        start_idx = buffer.index(PACKET_START_BYTE, 1)
                        buffer = buffer[start_idx:]
                    except ValueError:
                        buffer.clear()
                    break
                
                cmd_id, payload = result
                packet_len = 6 + len(payload)
                
                if cmd_id == RSP_STREAM_DATA:
                    packet_count += 1
                    if packet_count <= 3:  # Print first 3 packets
                        print(f"    Packet {packet_count}: {len(payload)} bytes")
                
                buffer = buffer[packet_len:]
        
        time.sleep(0.01)
    
    # Stop streaming
    result = send_command(ser, CMD_STOP_STREAMING)
    if result is None or result[0] != RSP_ACK:
        print("  WARNING: STOP_STREAMING not acknowledged")
    
    print(f"  SUCCESS: Received {packet_count} stream packets")
    return packet_count > 0


def list_serial_ports():
    """List available serial ports"""
    ports = serial.tools.list_ports.comports()
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")
    return [port.device for port in ports]


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Test USB Serial communication with gb_receiver')
    parser.add_argument('--port', '-p', type=str, help='Serial port (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--no-streaming', action='store_true', help='Skip streaming test')
    parser.add_argument('--list-ports', action='store_true', help='List available serial ports and exit')
    
    args = parser.parse_args()
    
    if args.list_ports:
        list_serial_ports()
        return
    
    if args.port is None:
        ports = list_serial_ports()
        if not ports:
            print("No serial ports found!")
            return
        if len(ports) == 1:
            port = ports[0]
            print(f"\nUsing only available port: {port}")
        else:
            print("\nPlease specify a port with --port")
            return
    else:
        port = args.port
    
    print(f"\nConnecting to {port} at {args.baud} baud...")
    
    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
        time.sleep(2)  # Wait for device to be ready
        
        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print("Connected!")
        
        # Run tests
        tests_passed = 0
        tests_total = 0
        
        tests_total += 1
        if test_ping(ser):
            tests_passed += 1
        
        tests_total += 1
        if test_get_firmware_version(ser):
            tests_passed += 1
        
        tests_total += 1
        if test_get_config(ser):
            tests_passed += 1
        
        if not args.no_streaming:
            tests_total += 1
            if test_streaming(ser, duration=5.0):
                tests_passed += 1
        
        # Summary
        print(f"\n{'='*50}")
        print(f"Tests passed: {tests_passed}/{tests_total}")
        if tests_passed == tests_total:
            print("All tests PASSED!")
        else:
            print("Some tests FAILED")
        print(f"{'='*50}")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()

