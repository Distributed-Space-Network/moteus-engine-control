# Easycomm Proxy Guide — moteus UART ↔ Rotator Protocol

Bridge between satellite tracking software (Gpredict, SatNOGS, etc.) and moteus motor controllers over UART. Translates Easycomm I/II/III ASCII commands to moteus multiplex protocol.

## Overview

The proxy runs a TCP server that accepts Easycomm rotator commands and converts them to moteus position commands. Two motors are controlled independently — one for azimuth, one for elevation — each with its own gear ratio.

**Hardware**: Two moteus-c1 controllers (AZ + EL) on the same UART bus  
**Protocol**: Easycomm I/II/III over TCP (default port 4533)  
**Transport**: moteus multiplex protocol over UART at 115200 baud

## Architecture

```
Tracking Software (Gpredict / SatNOGS / rotctld)
    │  TCP :4533 (Easycomm ASCII)
    ▼
easycomm_proxy.py
    │  Serial UART @ 115200 (moteus multiplex binary)
    ▼
moteus-c1 (AZ, ID=1)  +  moteus-c1 (EL, ID=2)
    │                       │
    ▼                       ▼
  AZ motor               EL motor
```

## Quick Start

```bash
python easycomm_proxy.py -p COM5
```

Then configure your tracking software to connect to `localhost:4533` using Easycomm protocol.

## CLI Arguments

| Flag | Description | Default |
|------|-------------|---------|
| `-p`, `--port` | Serial port to moteus | COM3 |
| `-b`, `--baud` | Baud rate | 115200 |
| `--az-id` | Moteus ID for azimuth motor | 1 |
| `--el-id` | Moteus ID for elevation motor | 2 |
| `--az-gear-ratio` | AZ gear ratio (motor rev / output rev) | 8.0 |
| `--el-gear-ratio` | EL gear ratio (motor rev / output rev) | 8.0 |
| `--tcp-port` | TCP listen port | 4533 |
| `--listen` | TCP listen address | 0.0.0.0 |
| `--poll-hz` | Background position poll rate (0 = off) | 2.0 |
| `--jog-step` | Jog step size in degrees | 1.0 |
| `-v`, `--verbose` | Debug logging | off |

### Example: Different gear ratios

```bash
python easycomm_proxy.py -p COM5 --az-gear-ratio 8.0 --el-gear-ratio 5.0
```

### Example: Custom motor IDs and port

```bash
python easycomm_proxy.py -p COM5 --az-id 3 --el-id 4 --tcp-port 4534
```

## Coordinate Conversion

The proxy handles degree ↔ revolution conversion per axis:

```
motor_revs = degrees / 360 × gear_ratio
degrees    = motor_revs / gear_ratio × 360
```

| Gear Ratio | 1° output = | 360° output = |
|------------|-------------|---------------|
| 8:1 | 0.0222 motor rev | 8 motor rev |
| 5:1 | 0.0139 motor rev | 5 motor rev |
| 10:1 | 0.0278 motor rev | 10 motor rev |

## Supported Easycomm Commands

### Easycomm I

Single-line combined format:

| Command | Description | Example |
|---------|-------------|---------|
| `AZnnn.n ELeee.e` | Set both axes | `AZ120.5 EL45.0` |

### Easycomm II

| Command | Description | Example / Response |
|---------|-------------|-------------------|
| `AZnnn.n` | Set azimuth | `AZ180.0` |
| `ELeee.e` | Set elevation | `EL45.0` |
| `AZ` | Query azimuth | → `AZ180.0` |
| `EL` | Query elevation | → `EL45.0` |
| `SA` | Stop azimuth | |
| `SE` | Stop elevation | |
| `ML` | Jog left (AZ−) | |
| `MR` | Jog right (AZ+) | |
| `MU` | Jog up (EL+) | |
| `MD` | Jog down (EL−) | |
| `VE` | Version query | → `VEeasycomm-moteus-proxy v1.0` |
| `AO` | AOS (logged) | |
| `LO` | LOS (logged) | |

### Easycomm III

| Command | Description | Response |
|---------|-------------|----------|
| `GS` | Get status register | `GS1` (1=idle, 2=moving, 8=error) |
| `GE` | Get error register | `GE0` (0=ok, 1=sensor error) |

## Tracking Software Configuration

### Gpredict

1. **Edit → Preferences → Interfaces → Rotators**
2. Add new rotator:
   - **Host**: `localhost`
   - **Port**: `4533`
   - **Az type**: 0° – 360°
   - **El type**: 0° – 90°

### SatNOGS

Configure the rotator driver to use Easycomm on `localhost:4533`.

### Hamlib / rotctld

If your software expects a Hamlib rotctld interface, note that this proxy speaks Easycomm natively — the same protocol rotctld uses. Point directly at the proxy.

## Manual Testing

Use telnet or netcat to send commands directly:

```bash
telnet localhost 4533
```

```
AZ180.0 EL45.0     ← move to 180° az, 45° el
AZ                  ← query azimuth
EL                  ← query elevation
SA                  ← stop azimuth
SE                  ← stop elevation
MR                  ← jog right 1°
VE                  ← get version
```

## Prerequisites

- Python 3.7+
- `pyserial` (`pip install pyserial`)
- `uart_shim_test.py` in the same directory (protocol functions are imported from it)

## Troubleshooting

### No response from motors
1. Verify serial port: `python uart_shim_test.py -p COM5` and run `query`
2. Check motor IDs match `--az-id` and `--el-id`
3. Ensure motors are powered and not in fault state

### Position reads as 0.0 always
- Motors may need calibration first — see [UART_SHIM_GUIDE.md](UART_SHIM_GUIDE.md)
- Check encoder config: `conf get motor_position.sources.0.type`

### Tracking software can't connect
- Firewall may block port 4533
- Verify proxy is running: look for `Listening on 0.0.0.0:4533` in output
- Try `--listen 127.0.0.1` if binding to all interfaces fails

### Motors overshoot or oscillate
- Tune PID: `conf get servo.pid_position` via `uart_shim_test.py`
- Reduce velocity/acceleration limits (see UART_SHIM_GUIDE.md → Trapezoidal Motion)
