"""
Moteus UART Shim CLI — interactive motor controller over USART1.

Usage:
    python uart_shim_test.py -p COM5
    python uart_shim_test.py -p COM5 -d 1 -b 115200

Commands:
    stop            - Stop the motor
    query           - Read mode, position, velocity, torque, voltage, temp, fault
    pos <p> [v] [t] - Position command (pos=revs, vel=rev/s, torque=Nm)
    brake           - Brake mode (motor shorts windings)
    zero [pos]      - Set current position as 'pos' (default 0.0)
    raw <hex>       - Send raw multiplex subframe bytes
    help            - Show this help
    quit            - Exit
"""
import argparse
import serial
import struct
import time
import sys

# Moteus register map
REG_MODE = 0x000
REG_POSITION = 0x001
REG_VELOCITY = 0x002
REG_TORQUE = 0x003
REG_VOLTAGE = 0x00D
REG_TEMPERATURE = 0x00E
REG_FAULT = 0x00F
REG_CMD_POSITION = 0x020
REG_CMD_VELOCITY = 0x021
REG_CMD_FEEDFORWARD_TORQUE = 0x022
REG_CMD_KP_SCALE = 0x023
REG_CMD_KD_SCALE = 0x024

# Moteus modes
MODE_STOPPED = 0
MODE_FAULT = 1
MODE_POSITION = 10
MODE_BRAKE = 15
MODE_ZERO_VEL = 5

# Multiplex subframe opcodes
WRITE_INT8  = 0x00  # + (count-1)
WRITE_INT16 = 0x04
WRITE_INT32 = 0x08
WRITE_FLOAT = 0x0C
READ_INT8   = 0x10
READ_INT16  = 0x14
READ_INT32  = 0x18
READ_FLOAT  = 0x1C
REPLY_INT8  = 0x20
REPLY_INT16 = 0x24
REPLY_INT32 = 0x28
REPLY_FLOAT = 0x2C

MODE_NAMES = {
    0: "stopped", 1: "fault", 2: "preparing", 3: "stopped_cal",
    5: "zero_vel", 10: "position", 11: "timeout", 12: "zero_vel_timed",
    15: "brake",
}


def encode_varuint(value):
    """Encode a varuint for the multiplex subframe register addresses."""
    result = bytearray()
    while value >= 0x80:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value & 0x7F)
    return bytes(result)


def build_write_int8(register, value):
    return bytes([WRITE_INT8]) + encode_varuint(register) + struct.pack('<b', value)


def build_write_float(register, value):
    return bytes([WRITE_FLOAT]) + encode_varuint(register) + struct.pack('<f', value)


def build_write_floats(start_register, values):
    n = len(values)
    assert 1 <= n <= 4
    data = bytes([WRITE_FLOAT + n - 1]) + encode_varuint(start_register)
    for v in values:
        data += struct.pack('<f', v)
    return data


def build_read_int8(start_register, count=1):
    assert 1 <= count <= 4
    return bytes([READ_INT8 + count - 1]) + encode_varuint(start_register)


def build_read_float(start_register, count=1):
    assert 1 <= count <= 4
    return bytes([READ_FLOAT + count - 1]) + encode_varuint(start_register)


def decode_response(payload):
    """Decode multiplex reply subframes from a response payload."""
    results = {}
    i = 0
    while i < len(payload):
        cmd = payload[i]
        i += 1
        if cmd >= REPLY_FLOAT and cmd < REPLY_FLOAT + 4:
            count = (cmd - REPLY_FLOAT) + 1
            if i >= len(payload):
                break
            start_reg = payload[i]
            i += 1
            for j in range(count):
                if i + 4 <= len(payload):
                    val = struct.unpack('<f', payload[i:i+4])[0]
                    results[start_reg + j] = val
                    i += 4
        elif cmd >= REPLY_INT8 and cmd < REPLY_INT8 + 4:
            count = (cmd - REPLY_INT8) + 1
            if i >= len(payload):
                break
            start_reg = payload[i]
            i += 1
            for j in range(count):
                if i < len(payload):
                    results[start_reg + j] = payload[i]
                    i += 1
        elif cmd >= REPLY_INT16 and cmd < REPLY_INT16 + 4:
            count = (cmd - REPLY_INT16) + 1
            if i >= len(payload):
                break
            start_reg = payload[i]
            i += 1
            for j in range(count):
                if i + 2 <= len(payload):
                    val = struct.unpack('<h', payload[i:i+2])[0]
                    results[start_reg + j] = val
                    i += 2
        elif cmd >= REPLY_INT32 and cmd < REPLY_INT32 + 4:
            count = (cmd - REPLY_INT32) + 1
            if i >= len(payload):
                break
            start_reg = payload[i]
            i += 1
            for j in range(count):
                if i + 4 <= len(payload):
                    val = struct.unpack('<i', payload[i:i+4])[0]
                    results[start_reg + j] = val
                    i += 4
        else:
            break  # Unknown subframe type
    return results


def send_command(ser, dest, subframe):
    """Send a multiplex subframe and receive the response."""
    source = 0x80  # Bit 7 must be set for response
    header = bytes([dest, source, len(subframe), 0x00])
    ser.reset_input_buffer()
    ser.write(header + subframe)
    ser.flush()

    # Read 4-byte response header
    resp_hdr = ser.read(4)
    if len(resp_hdr) < 4:
        return None, None
    r_dest, r_src, r_len, r_flags = resp_hdr
    payload = ser.read(r_len) if r_len > 0 else b''
    return resp_hdr, payload


def cmd_stop(ser, dest):
    subframe = build_write_int8(REG_MODE, MODE_STOPPED)
    _, payload = send_command(ser, dest, subframe)
    print("  Motor stopped.")


def cmd_brake(ser, dest):
    subframe = build_write_int8(REG_MODE, MODE_BRAKE)
    _, payload = send_command(ser, dest, subframe)
    print("  Brake engaged.")


def cmd_query(ser, dest):
    subframe = (
        build_read_int8(REG_MODE, 1) +           # mode
        build_read_float(REG_POSITION, 3) +       # pos, vel, torque
        build_read_float(REG_VOLTAGE, 2) +        # voltage, temperature
        build_read_int8(REG_FAULT, 1)             # fault
    )
    resp_hdr, payload = send_command(ser, dest, subframe)
    if payload is None:
        print("  [ERROR] No response.")
        return
    if resp_hdr:
        print(f"  [RAW] hdr={resp_hdr.hex().upper()} payload={payload.hex().upper() if payload else '(empty)'}")
    regs = decode_response(payload)
    if regs:
        print(f"  [DECODED] {regs}")
    mode = regs.get(REG_MODE, '?')
    mode_name = MODE_NAMES.get(mode, f"unknown({mode})")
    print(f"  Mode:     {mode_name}")
    print(f"  Position: {regs.get(REG_POSITION, '?'):.4f} rev" if isinstance(regs.get(REG_POSITION), float) else f"  Position: ?")
    print(f"  Velocity: {regs.get(REG_VELOCITY, '?'):.4f} rev/s" if isinstance(regs.get(REG_VELOCITY), float) else f"  Velocity: ?")
    print(f"  Torque:   {regs.get(REG_TORQUE, '?'):.4f} Nm" if isinstance(regs.get(REG_TORQUE), float) else f"  Torque:   ?")
    print(f"  Voltage:  {regs.get(REG_VOLTAGE, '?'):.2f} V" if isinstance(regs.get(REG_VOLTAGE), float) else f"  Voltage:  ?")
    print(f"  Temp:     {regs.get(REG_TEMPERATURE, '?'):.1f} °C" if isinstance(regs.get(REG_TEMPERATURE), float) else f"  Temp:     ?")
    fault = regs.get(REG_FAULT, '?')
    print(f"  Fault:    {fault}" + (" (none)" if fault == 0 else ""))


def cmd_position(ser, dest, pos, vel=None, torque=None):
    subframe = build_write_int8(REG_MODE, MODE_POSITION)
    cmd_values = [pos]
    if vel is not None:
        cmd_values.append(vel)
        if torque is not None:
            cmd_values.append(torque)
    subframe += build_write_floats(REG_CMD_POSITION, cmd_values)
    # Also query status
    subframe += (
        build_read_int8(REG_MODE, 1) +
        build_read_float(REG_POSITION, 3)
    )
    _, payload = send_command(ser, dest, subframe)
    if payload is None:
        print("  [ERROR] No response.")
        return
    regs = decode_response(payload)
    mode = regs.get(REG_MODE, '?')
    mode_name = MODE_NAMES.get(mode, f"unknown({mode})")
    p = regs.get(REG_POSITION, '?')
    v = regs.get(REG_VELOCITY, '?')
    print(f"  Mode: {mode_name} | Pos: {p:.4f} | Vel: {v:.4f}" if isinstance(p, float) else f"  Mode: {mode_name}")


def cmd_zero(ser, dest, zero_pos=0.0):
    # Write the OutputNearest register (0x130) to set zero
    subframe = build_write_float(0x130, zero_pos)
    _, payload = send_command(ser, dest, subframe)
    print(f"  Set current position as {zero_pos:.2f}")


def cmd_raw(ser, dest, hex_str):
    try:
        subframe = bytes.fromhex(hex_str)
    except ValueError:
        print("  [ERROR] Invalid hex string.")
        return
    resp_hdr, payload = send_command(ser, dest, subframe)
    if resp_hdr is None:
        print("  [ERROR] No response.")
        return
    print(f"  Response header: {resp_hdr.hex().upper()}")
    if payload:
        print(f"  Response payload ({len(payload)}b): {payload.hex().upper()}")
        regs = decode_response(payload)
        if regs:
            print(f"  Decoded: {regs}")
    else:
        print(f"  (empty payload)")


def main():
    parser = argparse.ArgumentParser(description="Moteus UART Shim CLI")
    parser.add_argument("--port", "-p", default="COM3", help="Serial port")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--dest", "-d", type=int, default=1, help="Destination ID")
    args = parser.parse_args()

    print(f"[INFO] Opening {args.port} at {args.baud} bps (dest ID={args.dest})...")
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()

    print("Moteus UART Shim CLI. Type 'help' for commands.\n")

    while True:
        try:
            line = input(f"moteus[{args.dest}]> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ('quit', 'exit', 'q'):
            break
        elif cmd == 'help':
            print(__doc__)
        elif cmd == 'stop':
            cmd_stop(ser, args.dest)
        elif cmd == 'brake':
            cmd_brake(ser, args.dest)
        elif cmd == 'query' or cmd == 'q?' or cmd == 'status':
            cmd_query(ser, args.dest)
        elif cmd in ('pos', 'position'):
            if len(parts) < 2:
                print("  Usage: pos <position> [velocity] [torque]")
                continue
            pos = float(parts[1])
            vel = float(parts[2]) if len(parts) > 2 else None
            torque = float(parts[3]) if len(parts) > 3 else None
            cmd_position(ser, args.dest, pos, vel, torque)
        elif cmd == 'zero':
            zero_pos = float(parts[1]) if len(parts) > 1 else 0.0
            cmd_zero(ser, args.dest, zero_pos)
        elif cmd == 'raw':
            if len(parts) < 2:
                print("  Usage: raw <hex_bytes>")
                continue
            cmd_raw(ser, args.dest, parts[1])
        else:
            print(f"  Unknown command: {cmd}. Type 'help'.")

    ser.close()
    print("Bye.")

if __name__ == '__main__':
    main()
