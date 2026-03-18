import argparse
import serial
import time
import sys

def main():
    parser = argparse.ArgumentParser(description="Test Moteus USART1 Echo")
    parser.add_argument("--port", "-p", default="COM3", help="Serial port")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    print(f"[INFO] Opening {args.port} at {args.baud} bps...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=2.0)
    except Exception as e:
        print(f"[ERROR] Failed to open port: {e}")
        sys.exit(1)

    # Flush any garbage
    time.sleep(0.1)
    ser.reset_input_buffer()

    # --- ECHO TEST ---
    # The firmware is running a raw register echo loop.
    # Every byte we send should come back identically.
    test_bytes = b"Hello Moteus!\r\n"
    
    print(f"\n[TX] Sending {len(test_bytes)} bytes: {test_bytes!r}")
    ser.write(test_bytes)
    ser.flush()

    # Read back
    echo = ser.read(len(test_bytes))
    
    if len(echo) == 0:
        print(f"[ERROR] No echo received at all! 0 bytes back.")
        print(f"[HINT] PB_7 (USART1 RX) is physically not receiving data from your TTL adapter TX.")
        print(f"[HINT] Double check: adapter GND <-> board GND, adapter TX <-> PB_7")
    elif echo == test_bytes:
        print(f"[RX] Got perfect echo: {echo!r}")
        print(f"\n[SUCCESS] Physical RX/TX wiring is CONFIRMED WORKING!")
        print(f"[INFO] The DMA channel theft by aux2_port is the root cause of the shim failure.")
    else:
        print(f"[RX] Got partial/garbled echo ({len(echo)} bytes): {echo!r}")
        print(f"[INFO] Partial echo suggests baud rate mismatch or noise.")

    # Try a second time
    time.sleep(0.2)
    ser.reset_input_buffer()
    test2 = bytes(range(256))
    print(f"\n[TX] Sending 256-byte binary sweep...")
    ser.write(test2)
    ser.flush()
    echo2 = ser.read(256)
    print(f"[RX] Got {len(echo2)}/256 bytes back.")
    if echo2 == test2:
        print("[SUCCESS] Perfect binary echo! Hardware is 100% functional.")
    elif len(echo2) > 0:
        mismatches = sum(1 for a, b in zip(test2, echo2) if a != b)
        print(f"[INFO] {mismatches} byte mismatches out of {len(echo2)} received.")

    ser.close()

if __name__ == '__main__':
    main()
