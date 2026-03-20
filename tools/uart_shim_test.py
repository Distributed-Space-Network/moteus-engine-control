"""
Moteus UART Shim CLI — interactive motor controller over USART1.

Commands:
    query / q       - Read mode, position, velocity, torque, voltage, temp, fault
    stop            - Stop the motor
    brake           - Brake mode (shorts windings)
    pos <p> [v] [t] - Position command (pos=revs, vel=rev/s, torque=Nm)
    watch [hz]      - Continuous query at given rate (default 2 Hz, Ctrl+C to stop)
    fault-clear     - Clear fault state (transitions to stopped)
    raw <hex>       - Send raw multiplex subframe bytes
    help            - Show this help
    quit / exit     - Exit
"""
import argparse
import serial
import struct
import time
import math
import sys

# -- Moteus Register Map --
REG_MODE = 0x000
REG_POSITION = 0x001
REG_VELOCITY = 0x002
REG_TORQUE = 0x003
REG_Q_CURRENT = 0x004
REG_D_CURRENT = 0x005
REG_ABS_POSITION = 0x006
REG_POWER = 0x007
REG_MOTOR_TEMP = 0x00A
REG_VOLTAGE = 0x00D
REG_TEMPERATURE = 0x00E
REG_FAULT = 0x00F

REG_PWM_A = 0x010
REG_PWM_B = 0x011
REG_PWM_C = 0x012
REG_V_PHASE_A = 0x014
REG_V_PHASE_B = 0x015
REG_V_PHASE_C = 0x016
REG_VFOC_THETA = 0x018
REG_VFOC_VOLTAGE = 0x019
REG_V_DQ_D = 0x01A
REG_V_DQ_Q = 0x01B

REG_CMD_POSITION = 0x020
REG_CMD_VELOCITY = 0x021
REG_CMD_FEEDFORWARD_TORQUE = 0x022
REG_CMD_KP_SCALE = 0x023
REG_CMD_KD_SCALE = 0x024
REG_CMD_MAX_TORQUE = 0x025
REG_CMD_STOP_POSITION = 0x026
REG_CMD_TIMEOUT = 0x027

# Moteus modes
MODE_STOPPED = 0
MODE_FAULT = 1
MODE_PWM = 5
MODE_VOLTAGE = 6
MODE_VOLTAGE_FOC = 7
MODE_VOLTAGE_DQ = 8
MODE_CURRENT = 9
MODE_POSITION = 10
MODE_BRAKE = 15

MODE_NAMES = {
    0: "stopped", 1: "fault", 2: "enabling", 3: "calibrating",
    4: "calib_complete", 5: "pwm", 6: "voltage", 7: "voltage_foc",
    8: "voltage_dq", 9: "current", 10: "position", 11: "pos_timeout",
    12: "zero_velocity", 13: "stay_within", 14: "meas_inductance", 15: "brake",
}

# -- Multiplex Subframe Encoding --
# MicroServer encoding: type_length = (type * 4) + encoded_length
# encoded_length: 0 = read varuint count from stream, 1-3 = direct count
WRITE_INT8  = 0x00
WRITE_FLOAT = 0x0C
READ_INT8   = 0x10
READ_FLOAT  = 0x1C

# Tunnel subframes
CLIENT_TO_SERVER = 0x40
SERVER_TO_CLIENT = 0x41
CLIENT_POLL_SERVER = 0x42
TUNNEL_CHANNEL = 1  # Diagnostic console


def encode_varuint(value):
    result = bytearray()
    while value >= 0x80:
        result.append((value & 0x7F) | 0x80)
        value >>= 7
    result.append(value & 0x7F)
    return bytes(result)


def _build_rw(base, count, start_register, values=None):
    """Build a read or write subframe with correct encoded_length handling."""
    # encoded_length = count for 1-3, varuint form for 4+
    if 1 <= count <= 3:
        data = bytes([base + count]) + encode_varuint(start_register)
    else:
        data = bytes([base]) + encode_varuint(count) + encode_varuint(start_register)
    if values:
        data += values
    return data


def build_write_int8(register, value):
    return _build_rw(WRITE_INT8, 1, register, struct.pack('<b', value))


def build_write_floats(start_register, values):
    packed = b''.join(struct.pack('<f', v) for v in values)
    return _build_rw(WRITE_FLOAT, len(values), start_register, packed)


def build_read_int8(start_register, count=1):
    return _build_rw(READ_INT8, count, start_register)


def build_read_float(start_register, count=1):
    return _build_rw(READ_FLOAT, count, start_register)


def decode_response(payload):
    """Decode multiplex reply subframes."""
    results = {}
    i = 0
    while i < len(payload):
        cmd = payload[i]
        i += 1
        if cmd < 0x20 or cmd >= 0x30:
            break
        type_length = cmd - 0x20
        encoded_length = type_length % 4
        data_type = type_length // 4  # 0=int8, 1=int16, 2=int32, 3=float

        if encoded_length == 0:
            if i >= len(payload): break
            count = payload[i]; i += 1
        else:
            count = encoded_length

        if i >= len(payload): break
        start_reg = payload[i]; i += 1

        sizes = [1, 2, 4, 4]
        fmts = ['<b', '<h', '<i', '<f']
        sz = sizes[data_type]
        for j in range(count):
            if i + sz > len(payload): break
            if sz == 1:
                results[start_reg + j] = payload[i]
            else:
                results[start_reg + j] = struct.unpack(fmts[data_type], payload[i:i+sz])[0]
            i += sz
    return results


# -- Standard query subframe (reused by multiple commands) --
QUERY_SUBFRAME = (
    build_read_int8(REG_MODE, 1) +
    build_read_float(REG_POSITION, 3) +
    build_read_float(REG_VOLTAGE, 2) +
    build_read_int8(REG_FAULT, 1)
)


def send_command(ser, dest, subframe):
    """Send a multiplex subframe and receive the response."""
    header = bytes([dest, 0x80, len(subframe), 0x00])
    ser.reset_input_buffer()
    ser.write(header + subframe)
    ser.flush()
    resp_hdr = ser.read(4)
    if len(resp_hdr) < 4:
        return None, b''
    r_len = resp_hdr[2]
    payload = ser.read(r_len) if r_len > 0 else b''
    return resp_hdr, payload


def format_query(regs):
    """Format decoded registers into a readable status string."""
    mode = regs.get(REG_MODE, '?')
    mode_name = MODE_NAMES.get(mode, f"unknown({mode})")
    lines = [f"  Mode: {mode_name}"]

    def fval(reg, unit, fmt=".4f"):
        v = regs.get(reg)
        return f"{v:{fmt}} {unit}" if isinstance(v, float) else "?"

    lines.append(f"  Pos:  {fval(REG_POSITION, 'rev')}")
    lines.append(f"  Vel:  {fval(REG_VELOCITY, 'rev/s')}")
    lines.append(f"  Trq:  {fval(REG_TORQUE, 'Nm')}")
    lines.append(f"  Vbus: {fval(REG_VOLTAGE, 'V', '.2f')}")
    lines.append(f"  Temp: {fval(REG_TEMPERATURE, '°C', '.1f')}")

    fault = regs.get(REG_FAULT, '?')
    fault_str = f"{fault}" + (" (none)" if fault == 0 else " ⚠")
    lines.append(f"  Fault: {fault_str}")
    return "\n".join(lines)


def cmd_query(ser, dest, verbose=False):
    resp_hdr, payload = send_command(ser, dest, QUERY_SUBFRAME)
    if payload is None or resp_hdr is None:
        print("  [ERROR] No response.")
        return {}
    if verbose and payload:
        print(f"  [RAW] {payload.hex().upper()}")
    regs = decode_response(payload)
    print(format_query(regs))
    return regs


def cmd_stop(ser, dest):
    subframe = build_write_int8(REG_MODE, MODE_STOPPED) + QUERY_SUBFRAME
    _, payload = send_command(ser, dest, subframe)
    regs = decode_response(payload or b'')
    mode = MODE_NAMES.get(regs.get(REG_MODE, '?'), '?')
    print(f"  Stopped. Mode: {mode}")


def cmd_brake(ser, dest):
    subframe = build_write_int8(REG_MODE, MODE_BRAKE) + QUERY_SUBFRAME
    _, payload = send_command(ser, dest, subframe)
    regs = decode_response(payload or b'')
    mode = MODE_NAMES.get(regs.get(REG_MODE, '?'), '?')
    print(f"  Brake engaged. Mode: {mode}")


def cmd_fault_clear(ser, dest):
    # Writing mode=0 (stopped) clears faults
    subframe = build_write_int8(REG_MODE, MODE_STOPPED) + QUERY_SUBFRAME
    _, payload = send_command(ser, dest, subframe)
    regs = decode_response(payload or b'')
    mode = MODE_NAMES.get(regs.get(REG_MODE, '?'), '?')
    fault = regs.get(REG_FAULT, '?')
    print(f"  Fault cleared. Mode: {mode}, Fault: {fault}")


def cmd_voltage(ser, dest, d_v, q_v):
    """Set voltage_dq mode with d/q voltages. No encoder needed."""
    subframe = build_write_int8(REG_MODE, MODE_VOLTAGE_DQ)
    subframe += build_write_floats(REG_V_DQ_D, [d_v, q_v])
    subframe += QUERY_SUBFRAME
    _, payload = send_command(ser, dest, subframe)
    regs = decode_response(payload or b'')
    print(format_query(regs))


def cmd_vfoc(ser, dest, theta, voltage):
    """Set voltage_foc mode with theta (rad) and voltage magnitude."""
    subframe = build_write_int8(REG_MODE, MODE_VOLTAGE_FOC)
    subframe += build_write_floats(REG_VFOC_THETA, [theta, voltage])
    subframe += QUERY_SUBFRAME
    _, payload = send_command(ser, dest, subframe)
    regs = decode_response(payload or b'')
    print(format_query(regs))


def cmd_position(ser, dest, pos, vel=None, torque=None, kp=None, kd=None):
    subframe = build_write_int8(REG_MODE, MODE_POSITION)
    cmd_floats = [pos]
    if vel is not None:
        cmd_floats.append(vel)
        if torque is not None:
            cmd_floats.append(torque)
            if kp is not None:
                cmd_floats.append(kp)
                if kd is not None:
                    cmd_floats.append(kd)
    subframe += build_write_floats(REG_CMD_POSITION, cmd_floats)
    subframe += QUERY_SUBFRAME
    _, payload = send_command(ser, dest, subframe)
    regs = decode_response(payload or b'')
    print(format_query(regs))


def cmd_watch(ser, dest, hz=2):
    interval = 1.0 / hz
    print(f"  Watching at {hz} Hz (Ctrl+C to stop)...")
    try:
        while True:
            resp_hdr, payload = send_command(ser, dest, QUERY_SUBFRAME)
            regs = decode_response(payload or b'')
            mode = MODE_NAMES.get(regs.get(REG_MODE, '?'), '?')
            pos = regs.get(REG_POSITION, float('nan'))
            vel = regs.get(REG_VELOCITY, float('nan'))
            trq = regs.get(REG_TORQUE, float('nan'))
            vbus = regs.get(REG_VOLTAGE, float('nan'))
            temp = regs.get(REG_TEMPERATURE, float('nan'))
            fault = regs.get(REG_FAULT, '?')
            line = (f"\r  {mode:12s} "
                    f"pos={pos:+8.4f} vel={vel:+8.4f} trq={trq:+7.4f} "
                    f"V={vbus:5.1f} T={temp:4.1f}°C F={fault}")
            print(line, end='', flush=True)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n  Stopped watching.")


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
    print(f"  TX: {subframe.hex().upper()} ({len(subframe)}b)")
    print(f"  RX hdr: {resp_hdr.hex().upper()}")
    if payload:
        print(f"  RX payload ({len(payload)}b): {payload.hex().upper()}")
        regs = decode_response(payload)
        if regs:
            print(f"  Decoded: {regs}")
    else:
        print(f"  (empty payload)")


def send_tunnel(ser, dest, text):
    """Send text through multiplex tunnel 1 (diagnostic console) and read response."""
    data = (text + '\n').encode('ascii')
    # Build client-to-server tunnel subframe
    tunnel_sf = bytearray()
    tunnel_sf.append(CLIENT_TO_SERVER)
    tunnel_sf += encode_varuint(TUNNEL_CHANNEL)
    tunnel_sf += encode_varuint(len(data))
    tunnel_sf += data
    # Also add a poll to get the response
    tunnel_sf.append(CLIENT_POLL_SERVER)
    tunnel_sf += encode_varuint(TUNNEL_CHANNEL)
    tunnel_sf += encode_varuint(256)  # max bytes to receive

    resp_hdr, payload = send_command(ser, dest, bytes(tunnel_sf))
    response_text = ''

    # Parse server-to-client responses from payload
    if payload:
        response_text = _extract_tunnel_text(payload)

    # Keep polling for more response data
    for _ in range(20):
        poll_sf = bytearray()
        poll_sf.append(CLIENT_POLL_SERVER)
        poll_sf += encode_varuint(TUNNEL_CHANNEL)
        poll_sf += encode_varuint(256)
        resp_hdr, payload = send_command(ser, dest, bytes(poll_sf))
        if not payload:
            break
        chunk = _extract_tunnel_text(payload)
        if not chunk:
            break
        response_text += chunk

    return response_text


def _extract_tunnel_text(payload):
    """Extract text from server-to-client tunnel subframes in a response payload."""
    text = ''
    i = 0
    while i < len(payload):
        cmd = payload[i]
        i += 1
        if cmd == SERVER_TO_CLIENT:
            if i >= len(payload): break
            channel = payload[i]; i += 1
            if i >= len(payload): break
            data_len = payload[i]; i += 1
            if i + data_len > len(payload): break
            if channel == TUNNEL_CHANNEL:
                text += payload[i:i+data_len].decode('ascii', errors='replace')
            i += data_len
        else:
            break  # Not a tunnel subframe
    return text


def cmd_conf(ser, dest, conf_args):
    """Send a conf command through the diagnostic tunnel."""
    cmd_text = 'conf ' + conf_args
    response = send_tunnel(ser, dest, cmd_text)
    if response:
        print(response.rstrip())
    else:
        print("  (no response)")


def cmd_tel(ser, dest, tel_args):
    """Send a tel command through the diagnostic tunnel."""
    response = send_tunnel(ser, dest, 'tel ' + tel_args)
    if response:
        print(response.rstrip())
    else:
        print("  (no response)")


def cmd_diag(ser, dest, text):
    """Send arbitrary text through the diagnostic tunnel."""
    response = send_tunnel(ser, dest, text)
    if response:
        print(response.rstrip())
    else:
        print("  (no response)")


HELP_TEXT = """
Commands:
  query / q          Read motor status
  stop               Stop the motor
  brake              Brake mode (short windings)
  fault-clear        Clear fault → stopped
  pos P [V] [T]      Position cmd (revs, rev/s, Nm) — needs encoder
  voltage D_V Q_V    Voltage DQ mode (volts) — no encoder needed
  vfoc THETA VOLTAGE Voltage FOC mode (rad, volts)
  watch [Hz]         Continuous status (default 2 Hz)
  conf <args>        Send conf command (get/set/write/default/list/enumerate)
  tel <args>         Send tel command (list/get/stop)
  diag <text>        Send arbitrary diagnostic command
  raw <hex>          Send raw multiplex subframe
  help               This help
  quit / exit        Exit

Examples:
  conf get motor       Show motor config
  conf set motor.poles 14
  conf set servo.aux1_encoder.mode 1
  conf write           Save config to flash
  conf default         Reset config to defaults
  tel list             List telemetry channels
"""


def main():
    parser = argparse.ArgumentParser(description="Moteus UART Shim CLI")
    parser.add_argument("--port", "-p", default="COM3", help="Serial port")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate")
    parser.add_argument("--dest", "-d", type=int, default=1, help="Destination ID")
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud} (dest={args.dest})...")
    ser = serial.Serial(args.port, args.baud, timeout=20.0)

    # Capture boot diagnostic output (firmware prints HW info at startup)
    print("[BOOT] Waiting for boot output (20s)...")
    time.sleep(20.0)
    boot_data = ser.read(ser.in_waiting) if ser.in_waiting else b''
    if boot_data:
        # Try to decode as text, show hex for non-printable
        try:
            text = boot_data.decode('ascii', errors='replace')
            print(f"[BOOT] {text.strip()}")
        except:
            print(f"[BOOT] {boot_data.hex().upper()}")
    else:
        print("[BOOT] (no boot output)")
    ser.reset_input_buffer()
    ser.timeout = 1.0

    # Initial status
    cmd_query(ser, args.dest)
    print(f"\nType 'help' for commands.\n")

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

        try:
            if cmd in ('quit', 'exit'):
                break
            elif cmd == 'help':
                print(HELP_TEXT)
            elif cmd == 'stop':
                cmd_stop(ser, args.dest)
            elif cmd == 'brake':
                cmd_brake(ser, args.dest)
            elif cmd in ('query', 'q', 'status'):
                cmd_query(ser, args.dest, verbose=('-v' in parts))
            elif cmd == 'fault-clear':
                cmd_fault_clear(ser, args.dest)
            elif cmd in ('pos', 'position'):
                if len(parts) < 2:
                    print("  Usage: pos <position> [velocity] [torque] [kp_scale] [kd_scale]")
                    continue
                pos = float(parts[1])
                vel = float(parts[2]) if len(parts) > 2 else None
                torque = float(parts[3]) if len(parts) > 3 else None
                kp = float(parts[4]) if len(parts) > 4 else None
                kd = float(parts[5]) if len(parts) > 5 else None
                cmd_position(ser, args.dest, pos, vel, torque, kp, kd)
            elif cmd == 'voltage':
                if len(parts) < 3:
                    print("  Usage: voltage <d_V> <q_V>  (e.g. voltage 0 0.5)")
                    continue
                cmd_voltage(ser, args.dest, float(parts[1]), float(parts[2]))
            elif cmd == 'vfoc':
                if len(parts) < 3:
                    print("  Usage: vfoc <theta_rad> <voltage>  (e.g. vfoc 0 0.5)")
                    continue
                cmd_vfoc(ser, args.dest, float(parts[1]), float(parts[2]))
            elif cmd == 'watch':
                hz = float(parts[1]) if len(parts) > 1 else 2.0
                cmd_watch(ser, args.dest, hz)
            elif cmd == 'raw':
                if len(parts) < 2:
                    print("  Usage: raw <hex_bytes>")
                    continue
                cmd_raw(ser, args.dest, parts[1])
            elif cmd == 'conf':
                if len(parts) < 2:
                    print("  Usage: conf <get|set|write|default|list|enumerate> [args]")
                    continue
                cmd_conf(ser, args.dest, ' '.join(parts[1:]))
            elif cmd == 'tel':
                if len(parts) < 2:
                    print("  Usage: tel <list|get|stop> [args]")
                    continue
                cmd_tel(ser, args.dest, ' '.join(parts[1:]))
            elif cmd == 'diag':
                if len(parts) < 2:
                    print("  Usage: diag <command text>")
                    continue
                cmd_diag(ser, args.dest, ' '.join(parts[1:]))
            else:
                print(f"  Unknown: '{cmd}'. Type 'help'.")
        except Exception as e:
            print(f"  [ERROR] {e}")

    ser.close()
    print("Bye.")

if __name__ == '__main__':
    main()
