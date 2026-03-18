import argparse
import serial
import struct
import time
import sys

def main():
    parser = argparse.ArgumentParser(description="Test Moteus UartMicroServer CAN Shim")
    parser.add_argument("--port", "-p", default="COM3", help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--dest", "-d", type=int, default=1, help="Destination Moteus ID (default 1)")
    parser.add_argument("--src", "-s", type=int, default=0, help="Source ID (default 0)")
    args = parser.parse_args()

    print(f"[INFO] Opening {args.port} at {args.baud} bps...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except Exception as e:
        print(f"[ERROR] Failed to open port: {e}")
        sys.exit(1)

    # -------------------------------------------------------------
    # Moteus Multiplex Command: make_position(query=True)
    # -------------------------------------------------------------
    multiplex_payload = bytes.fromhex("01000a0d200000c07f11001f01130d")

    potential_destinations = [args.dest, 2, 3, 0x7F, 0xFF]
    
    for dest in potential_destinations:
        print(f"\n[INFO] Pinging Destination ID: {dest}...")
        # UartMicroServer Header: [dest] [src] [length] [flags]
        header = struct.pack('<BBBB', dest, args.src, len(multiplex_payload), 0)
        
        # Flush input buffer just in case
        ser.reset_input_buffer()
        
        ser.write(header)
        ser.write(multiplex_payload)
        ser.flush()

        resp_header = ser.read(4)

        if not resp_header or len(resp_header) < 4:
            print(f"[WARNING] Timed out waiting for response from ID {dest}.")
            continue

        rx_dest, src, payload_len, flags = struct.unpack('<BBBB', resp_header)
        print(f"[RX SUCCESS] Received Header -> Dest: {rx_dest}, Src: {src}, Length: {payload_len}, Flags: {flags}")

        if payload_len > 0:
            resp_payload = ser.read(payload_len)
            print(f"[RX] Received Payload ({len(resp_payload)} bytes): {resp_payload.hex().upper()}")
        else:
            print("[RX] Payload length is 0.")

        print("\n[SUCCESS] Custom Moteus USART Shim transport is fully functional!")
        ser.close()
        return

    print("\n[ERROR] No IDs responded. Please double check that your TTL adapter TX is plugged into the board PB_7 pin stably!")
    ser.close()

if __name__ == '__main__':
    main()
