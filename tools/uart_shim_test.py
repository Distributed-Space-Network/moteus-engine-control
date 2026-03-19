import argparse
import serial
import time
import sys

def main():
    parser = argparse.ArgumentParser(description="Test Moteus USART1 CAN Shim")
    parser.add_argument("--port", "-p", default="COM3", help="Serial port")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--dest", "-d", type=int, default=1, help="Destination ID")
    args = parser.parse_args()

    print(f"[INFO] Opening {args.port} at {args.baud} bps...")
    ser = serial.Serial(args.port, args.baud, timeout=3.0)

    # Brief settle + flush
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Minimal multiplex subframe: read 1 int8 from register 0 (mode)
    # 0x10 = read 1 int8, 0x00 = start register 0
    multiplex_payload = bytes([0x10, 0x00])
    dest = args.dest
    # Bit 7 of source MUST be set — mjlib MicroServer only responds if (source & 0x80) != 0
    source_id = 0x80
    header = bytes([dest, source_id, len(multiplex_payload), 0x00])
    packet = header + multiplex_payload

    print(f"\n[TX] Sending to dest={dest}: {packet.hex().upper()} ({len(packet)} bytes)")
    ser.write(packet)
    ser.flush()

    # Listen for response
    print(f"\n[INFO] Listening for response (3s timeout)...")
    response = ser.read(256)
    if response:
        print(f"[RX] Got {len(response)} bytes!")
        print(f"[RX] Hex: {response.hex().upper()}")
        if len(response) >= 4:
            r_dest, r_src, r_len, r_flags = response[0], response[1], response[2], response[3]
            print(f"[RX] Header: dest={r_dest:#04x} src={r_src:#04x} len={r_len} flags={r_flags}")
            if r_len > 0 and len(response) >= 4 + r_len:
                payload = response[4:4+r_len]
                print(f"[RX] Payload: {payload.hex().upper()}")
        print(f"\n[SUCCESS] Got response!")
    else:
        print(f"[RX] Got 0 bytes. No response.")

    ser.close()

if __name__ == '__main__':
    main()
