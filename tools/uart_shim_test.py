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
    try:
        ser = serial.Serial(args.port, args.baud, timeout=20.0)
    except Exception as e:
        print(f"[ERROR] Failed to open port: {e}")
        sys.exit(1)

    # Capture boot diagnostic
    print("[INFO] Waiting for boot data (reset board now if needed)...")
    boot = ser.read(256)
    if boot:
        print(f"[BOOT] {len(boot)} bytes: {boot!r}")
    else:
        print("[BOOT] No boot data (board may have already booted)")
    
    # Flush any remaining boot noise
    time.sleep(0.2)
    ser.reset_input_buffer()

    # Moteus Multiplex: make_position(query=True) payload
    multiplex_payload = bytes.fromhex("01000a0d200000c07f11001f01130d")

    # Send zero padding to re-align the UartMicroServer's 4-byte parser
    print("\n[INFO] Sending alignment padding (100 x 0x00)...")
    ser.write(b'\x00' * 100)
    ser.flush()
    time.sleep(0.1)
    ser.reset_input_buffer()

    potential_destinations = [args.dest, 2, 3, 0x7F, 0xFF]
    
    for dest in potential_destinations:
        source_id = 0x00
        payload_len = len(multiplex_payload)
        flags = 0x00
        header = bytes([dest, source_id, payload_len, flags])
        
        print(f"\n[INFO] Pinging Destination ID: {dest}...")
        print(f"[TX] Header (4 bytes): {header.hex().upper()}")
        print(f"[TX] Payload ({payload_len} bytes): {multiplex_payload.hex().upper()}")
        
        ser.write(header + multiplex_payload)
        ser.flush()

        # Wait for 4-byte response header
        resp_header = ser.read(4)
        if len(resp_header) == 4:
            r_dest, r_src, r_len, r_flags = resp_header
            print(f"[RX] Header: dest={r_dest} src={r_src} len={r_len} flags={r_flags}")
            if r_len > 0:
                resp_payload = ser.read(r_len)
                print(f"[RX] Payload ({len(resp_payload)} bytes): {resp_payload.hex().upper()}")
            print(f"\n[SUCCESS] Got response from ID {r_src}!")
            ser.close()
            return
        else:
            print(f"[WARNING] Timed out waiting for response from ID {dest}.")

    print(f"\n[ERROR] No IDs responded.")
    ser.close()

if __name__ == '__main__':
    main()
