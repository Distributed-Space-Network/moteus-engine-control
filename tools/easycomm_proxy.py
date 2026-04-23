"""
Easycomm ↔ Moteus UART Proxy

TCP server that bridges Easycomm I/II rotator protocol commands
(from Gpredict, SatNOGS client, Hamlib rotctld, etc.) to moteus
motor controllers over UART multiplex protocol.

Usage:
    python easycomm_proxy.py -p COM5 --az-id 1 --el-id 2

Then point your tracking software at TCP localhost:4533.
"""

import argparse
import logging
import math
import select
import socket
import struct
import sys
import threading
import time

# Re-use moteus protocol primitives from uart_shim_test
from uart_shim_test import (
    REG_MODE, REG_POSITION, REG_VELOCITY, REG_TORQUE,
    REG_VOLTAGE, REG_TEMPERATURE, REG_FAULT,
    REG_CMD_POSITION, REG_CMD_VELOCITY,
    REG_CMD_FEEDFORWARD_TORQUE, REG_CMD_KP_SCALE, REG_CMD_KD_SCALE,
    REG_CMD_MAX_TORQUE, REG_CMD_STOP_POSITION, REG_CMD_TIMEOUT,
    MODE_STOPPED, MODE_POSITION, MODE_NAMES,
    QUERY_SUBFRAME,
    build_write_int8, build_write_floats, build_read_int8, build_read_float,
    send_command, decode_response, encode_varuint,
)

import serial

log = logging.getLogger("easycomm")

# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------

def degrees_to_revs(degrees, gear_ratio):
    """Convert degrees to motor revolutions.  revs = deg / 360 * gear_ratio"""
    return degrees / 360.0 * gear_ratio


def revs_to_degrees(revs, gear_ratio):
    """Convert motor revolutions to degrees.  deg = revs / gear_ratio * 360"""
    return revs / gear_ratio * 360.0


# ---------------------------------------------------------------------------
# Moteus helpers (thin wrappers around uart_shim primitives)
# ---------------------------------------------------------------------------

class MoteusAxis:
    """One moteus motor axis (AZ or EL)."""

    def __init__(self, ser, motor_id, gear_ratio, lock):
        self.ser = ser
        self.motor_id = motor_id
        self.gear_ratio = gear_ratio
        self._lock = lock
        self.cached_deg = 0.0  # last known position in degrees

    # -- low level --

    def _send(self, subframe):
        with self._lock:
            return send_command(self.ser, self.motor_id, subframe)

    # -- public API --

    def query(self):
        """Query motor registers and update cached position. Returns regs dict."""
        _, payload = self._send(QUERY_SUBFRAME)
        regs = decode_response(payload or b'')
        pos_rev = regs.get(REG_POSITION)
        if isinstance(pos_rev, float):
            self.cached_deg = revs_to_degrees(pos_rev, self.gear_ratio)
        return regs

    def set_position_deg(self, degrees, velocity_dps=None, max_torque=None):
        """Command motor to an absolute position in degrees."""
        revs = degrees_to_revs(degrees, self.gear_ratio)
        subframe = build_write_int8(REG_MODE, MODE_POSITION)

        cmd_floats = [revs]
        if velocity_dps is not None:
            cmd_floats.append(degrees_to_revs(velocity_dps, self.gear_ratio))
        if max_torque is not None:
            # need feedforward slot filled even if unused
            if len(cmd_floats) == 1:
                cmd_floats.append(math.nan)  # velocity = nan = no limit
            cmd_floats.append(0.0)  # feedforward torque
            cmd_floats.append(1.0)  # kp_scale
            cmd_floats.append(1.0)  # kd_scale
            cmd_floats.append(max_torque)

        subframe += build_write_floats(REG_CMD_POSITION, cmd_floats)
        subframe += QUERY_SUBFRAME
        _, payload = self._send(subframe)
        regs = decode_response(payload or b'')
        pos_rev = regs.get(REG_POSITION)
        if isinstance(pos_rev, float):
            self.cached_deg = revs_to_degrees(pos_rev, self.gear_ratio)
        return regs

    def stop(self):
        """Stop the motor."""
        subframe = build_write_int8(REG_MODE, MODE_STOPPED) + QUERY_SUBFRAME
        _, payload = self._send(subframe)
        regs = decode_response(payload or b'')
        return regs

    def get_position_deg(self):
        """Query and return current position in degrees."""
        self.query()
        return self.cached_deg


# ---------------------------------------------------------------------------
# Easycomm protocol parser
# ---------------------------------------------------------------------------

JOG_STEP_DEG = 1.0  # degrees per ML/MR/MU/MD press

VERSION_STRING = "VEeasycomm-moteus-proxy v1.0"


def parse_easycomm(line, az_axis, el_axis):
    """Parse one Easycomm command line. Returns response string or None."""
    line = line.strip()
    if not line:
        return None

    # Easycomm I: "AZaaa.a ELeee.e" on one line
    # Easycomm II: single commands, but can be space-separated on one line
    tokens = line.split()
    responses = []

    i = 0
    while i < len(tokens):
        token = tokens[i].upper()

        # --- AZ with value: set azimuth ---
        if token.startswith("AZ") and len(token) > 2:
            try:
                deg = float(token[2:])
                az_axis.set_position_deg(deg)
                log.info("AZ → %.1f°", deg)
            except ValueError:
                log.warning("Bad AZ value: %s", token)

        # --- AZ alone: query azimuth ---
        elif token == "AZ":
            deg = az_axis.get_position_deg()
            responses.append(f"AZ{deg:.1f}")

        # --- EL with value: set elevation ---
        elif token.startswith("EL") and len(token) > 2:
            try:
                deg = float(token[2:])
                el_axis.set_position_deg(deg)
                log.info("EL → %.1f°", deg)
            except ValueError:
                log.warning("Bad EL value: %s", token)

        # --- EL alone: query elevation ---
        elif token == "EL":
            deg = el_axis.get_position_deg()
            responses.append(f"EL{deg:.1f}")

        # --- Stop ---
        elif token == "SA":
            az_axis.stop()
            log.info("SA — stop azimuth")
        elif token == "SE":
            el_axis.stop()
            log.info("SE — stop elevation")

        # --- Jog ---
        elif token == "ML":
            az_axis.set_position_deg(az_axis.cached_deg - JOG_STEP_DEG)
            log.info("ML — jog left")
        elif token == "MR":
            az_axis.set_position_deg(az_axis.cached_deg + JOG_STEP_DEG)
            log.info("MR — jog right")
        elif token == "MU":
            el_axis.set_position_deg(el_axis.cached_deg + JOG_STEP_DEG)
            log.info("MU — jog up")
        elif token == "MD":
            el_axis.set_position_deg(el_axis.cached_deg - JOG_STEP_DEG)
            log.info("MD — jog down")

        # --- Version ---
        elif token == "VE":
            responses.append(VERSION_STRING)

        # --- AOS / LOS (acquisition/loss of signal) ---
        elif token == "AO":
            log.info("AOS — acquisition of signal")
        elif token == "LO":
            log.info("LOS — loss of signal")

        # --- Get status (Easycomm III) ---
        elif token == "GS":
            # 1=idle, 2=moving, 4=pointing, 8=error
            az_regs = az_axis.query()
            el_regs = el_axis.query()
            az_mode = az_regs.get(REG_MODE, 0)
            el_mode = el_regs.get(REG_MODE, 0)
            status = 1  # idle
            if az_mode == 10 or el_mode == 10:  # position mode = moving
                status = 2
            if az_regs.get(REG_FAULT, 0) or el_regs.get(REG_FAULT, 0):
                status |= 8
            responses.append(f"GS{status}")

        # --- Get error (Easycomm III) ---
        elif token == "GE":
            az_regs = az_axis.query()
            el_regs = el_axis.query()
            err = 0
            if az_regs.get(REG_FAULT, 0) or el_regs.get(REG_FAULT, 0):
                err = 1  # sensor error
            responses.append(f"GE{err}")

        # --- Ignored but valid commands ---
        elif token in ("UP", "DN", "DM", "UM", "DR", "UR", "ST",
                       "AN", "IP", "OP"):
            log.debug("Ignored command: %s", token)
            # consume the next token as value if present
            if i + 1 < len(tokens) and not tokens[i + 1][:2].isalpha():
                i += 1

        else:
            log.debug("Unknown command: %s", token)

        i += 1

    if responses:
        return "\n".join(responses) + "\n"
    return None


# ---------------------------------------------------------------------------
# Position polling thread
# ---------------------------------------------------------------------------

def poll_positions(az_axis, el_axis, interval):
    """Background thread: periodically query both axes to keep cache fresh."""
    while True:
        try:
            az_axis.query()
            el_axis.query()
        except Exception as e:
            log.warning("Poll error: %s", e)
        time.sleep(interval)


# ---------------------------------------------------------------------------
# TCP server
# ---------------------------------------------------------------------------

def handle_client(conn, addr, az_axis, el_axis):
    """Handle one Easycomm TCP client connection."""
    log.info("Client connected: %s", addr)
    buf = ""
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            buf += data.decode("ascii", errors="replace")

            # Process complete lines
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip("\r")
                if not line:
                    continue
                try:
                    resp = parse_easycomm(line, az_axis, el_axis)
                    if resp:
                        conn.sendall(resp.encode("ascii"))
                except Exception as e:
                    log.error("Command error [%s]: %s", line, e)
    except (ConnectionResetError, BrokenPipeError):
        pass
    except Exception as e:
        log.error("Client error: %s", e)
    finally:
        conn.close()
        log.info("Client disconnected: %s", addr)


def run_server(listen_addr, listen_port, az_axis, el_axis):
    """Accept Easycomm TCP connections."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((listen_addr, listen_port))
    srv.listen(2)
    log.info("Listening on %s:%d", listen_addr, listen_port)

    try:
        while True:
            conn, addr = srv.accept()
            t = threading.Thread(
                target=handle_client,
                args=(conn, addr, az_axis, el_axis),
                daemon=True,
            )
            t.start()
    except KeyboardInterrupt:
        log.info("Shutting down.")
    finally:
        srv.close()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(
        description="Easycomm ↔ Moteus UART proxy",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # Serial
    p.add_argument("-p", "--port", default="COM3", help="Serial port to moteus")
    p.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate")

    # Motor IDs
    p.add_argument("--az-id", type=int, default=1, help="Moteus ID for azimuth motor")
    p.add_argument("--el-id", type=int, default=2, help="Moteus ID for elevation motor")

    # Gear ratios (motor revs per output rev)
    p.add_argument("--az-gear-ratio", type=float, default=8.0,
                    help="Azimuth gear ratio (motor revs per output rev)")
    p.add_argument("--el-gear-ratio", type=float, default=8.0,
                    help="Elevation gear ratio (motor revs per output rev)")

    # TCP
    p.add_argument("--listen", default="0.0.0.0", help="TCP listen address")
    p.add_argument("--tcp-port", type=int, default=4533, help="TCP listen port")

    # Polling
    p.add_argument("--poll-hz", type=float, default=2.0,
                    help="Background position poll rate (Hz, 0=disable)")

    # Misc
    p.add_argument("--jog-step", type=float, default=1.0,
                    help="Jog step size in degrees (ML/MR/MU/MD)")
    p.add_argument("-v", "--verbose", action="store_true", help="Debug logging")

    args = p.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

    global JOG_STEP_DEG
    JOG_STEP_DEG = args.jog_step

    # Open serial
    log.info("Opening %s @ %d baud", args.port, args.baud)
    ser = serial.Serial(args.port, args.baud, timeout=1.0)
    time.sleep(0.5)
    ser.reset_input_buffer()

    # Shared lock for serial access (TCP threads + poll thread)
    serial_lock = threading.Lock()

    az = MoteusAxis(ser, args.az_id, args.az_gear_ratio, serial_lock)
    el = MoteusAxis(ser, args.el_id, args.el_gear_ratio, serial_lock)

    # Initial query
    log.info("Querying azimuth motor (ID=%d, gear=%.2f:1)...", args.az_id, args.az_gear_ratio)
    try:
        az.query()
        log.info("  AZ position: %.1f°", az.cached_deg)
    except Exception as e:
        log.warning("  AZ query failed: %s", e)

    log.info("Querying elevation motor (ID=%d, gear=%.2f:1)...", args.el_id, args.el_gear_ratio)
    try:
        el.query()
        log.info("  EL position: %.1f°", el.cached_deg)
    except Exception as e:
        log.warning("  EL query failed: %s", e)

    # Start poll thread
    if args.poll_hz > 0:
        poll_interval = 1.0 / args.poll_hz
        t = threading.Thread(
            target=poll_positions, args=(az, el, poll_interval), daemon=True
        )
        t.start()
        log.info("Position polling at %.1f Hz", args.poll_hz)

    # Run TCP server (blocks until Ctrl+C)
    run_server(args.listen, args.tcp_port, az, el)

    ser.close()
    log.info("Done.")


if __name__ == "__main__":
    main()
