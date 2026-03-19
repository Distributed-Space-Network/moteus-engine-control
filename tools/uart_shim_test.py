import argparse
import serial
import time
import sys

def main():
    parser = argparse.ArgumentParser(description="Test Moteus USART1 Echo + Diagnostics")
    parser.add_argument("--port", "-p", default="COM3", help="Serial port")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    print(f"[INFO] Opening {args.port} at {args.baud} bps...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=20.0)
    except Exception as e:
        print(f"[ERROR] Failed to open port: {e}")
        sys.exit(1)

    # Wait for the diagnostic message from boot
    print("\n[INFO] Waiting for boot diagnostic (reset/power-cycle the board now)...")
    print("[INFO] Reading for up to 20 seconds...\n")
    
    boot_data = ser.read(512)
    if boot_data:
        print(f"[BOOT] Received {len(boot_data)} bytes:")
        try:
            print(boot_data.decode('ascii', errors='replace'))
        except:
            print(boot_data.hex())
    else:
        print("[BOOT] No boot data received (might have already booted).")
    
    # Flush
    time.sleep(0.1)
    ser.reset_input_buffer()

    # --- ECHO TEST ---
    test_bytes = b"Hello Moteus!\r\n"
    
    print(f"\n[TX] Sending {len(test_bytes)} bytes: {test_bytes!r}")
    ser.write(test_bytes)
    ser.flush()

    echo = ser.read(len(test_bytes))
    
    if len(echo) == 0:
        print(f"[ERROR] No echo received at all! 0 bytes back.")
        print(f"[HINT] PB_7 (USART1 RX) is not receiving.")
    elif echo == test_bytes:
        print(f"[RX] Got perfect echo: {echo!r}")
        print(f"\n[SUCCESS] Echo works! Physical RX/TX confirmed!")
    else:
        print(f"[RX] Got {len(echo)} bytes: {echo!r}")
        print(f"[RX] Hex: {echo.hex().upper()}")

    ser.close()

if __name__ == '__main__':
    main()
