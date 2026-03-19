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
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Alignment padding
    print("\n[INFO] Sending 100 x 0x00 alignment padding...")
    ser.write(b'\x00' * 100)
    ser.flush()
    time.sleep(0.3)
    ser.reset_input_buffer()

    # Moteus make_position(query=True) payload
    multiplex_payload = bytes.fromhex("01000a0d200000c07f11001f01130d")
    dest = args.dest
    source_id = 0x00
    header = bytes([dest, source_id, len(multiplex_payload), 0x00])
    packet = header + multiplex_payload

    print(f"\n[TX] Sending to dest={dest}: {packet.hex().upper()} ({len(packet)} bytes)")
    ser.write(packet)
    ser.flush()

    # Now just listen for ANY response bytes for 3 seconds
    ser.timeout = 3.0
    print(f"\n[INFO] Listening for ANY response bytes (3s timeout)...")
    response = ser.read(256)
    if response:
        print(f"[RX] Got {len(response)} bytes!")
        print(f"[RX] Hex: {response.hex().upper()}")
        print(f"[RX] ASCII: {response!r}")
    else:
        print(f"[RX] Got 0 bytes. No response at all.")
        print()
        print("[INFO] Trying a simple 'tel list\\n' text command on tunnel 1...")
        # Try sending raw text command through the tunnel
        # This tests if the multiplex protocol is even alive
        cmd = b'\x01\x00\x0e\x00' + b'tel list\n'  # dest=1, src=0, len=9, flags=0
        print(f"[TX] {cmd.hex().upper()}")
        ser.write(cmd)
        ser.flush()
        response2 = ser.read(256)
        if response2:
            print(f"[RX] {len(response2)} bytes: {response2!r}")
        else:
            print(f"[RX] Still nothing.")

    ser.close()

if __name__ == '__main__':
    main()
