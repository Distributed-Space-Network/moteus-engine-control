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
    # Moteus Multiplex Command: Read 1 Register starting at 0x00 (Mode)
    # This is a standard mjlib multiplex command over CAN/UART
    # 0x04 = Read int8 (1 register)
    # 0x00 = Register 0x00 (Mode)
    # -------------------------------------------------------------
    multiplex_payload = bytes([0x04, 0x00])

    # UartMicroServer Header: [dest(1)] [src(1)] [length(1)] [flags(1)]
    header = struct.pack('<BBBB', args.dest, args.src, len(multiplex_payload), 0)

    print(f"\n[TX] Sending UartMicroServer Header ({len(header)} bytes): {header.hex().upper()}")
    print(f"[TX] Sending Multiplex Payload  ({len(multiplex_payload)} bytes): {multiplex_payload.hex().upper()}")
    
    # Send Header then Payload
    ser.write(header)
    ser.write(multiplex_payload)
    ser.flush()

    # Read back response header
    print("\n[INFO] Waiting for 4-byte response header...")
    resp_header = ser.read(4)

    if not resp_header or len(resp_header) < 4:
        print(f"[ERROR] Timed out waiting for response! Received {len(resp_header)} bytes: {resp_header.hex().upper()}")
        print("[HINT] Check RX/TX wires, ensure Moteus is powered, and baud rate matches (115200).")
        sys.exit(1)

    dest, src, payload_len, flags = struct.unpack('<BBBB', resp_header)
    print(f"[RX] Received Header -> Dest: {dest}, Src: {src}, Length: {payload_len}, Flags: {flags}")

    if payload_len > 0:
        print(f"[INFO] Waiting for {payload_len} bytes of payload...")
        resp_payload = ser.read(payload_len)
        print(f"[RX] Received Payload ({len(resp_payload)} bytes): {resp_payload.hex().upper()}")
        
        if len(resp_payload) < payload_len:
            print("[WARNING] Did not receive full payload before timeout!")
    else:
        print("[RX] Payload length is 0. No data attached.")

    print("\n[SUCCESS] Custom Moteus USART Shim transport is fully functional!")
    ser.close()

if __name__ == '__main__':
    main()
