# UART Shim Guide — moteus-c1 via UART

Control the moteus-c1 motor controller through UART when CAN tools are unavailable.

## Overview

The UART shim replaces the moteus CAN bus interface with a direct UART connection.
Commands are tunneled through the multiplex protocol over serial at 115200 baud.

**Hardware**: GIM6010-8 motor (14 pole pairs, 8:1 planetary gear)  
**Board**: moteus-c1 (STM32G4, DRV8323, AS5047P encoder)

## Quick Start

```bash
python uart_shim_test.py -p COM5
```

### CLI Arguments

| Flag | Description | Default |
|------|-------------|---------|
| `-p`, `--port` | Serial port | COM3 |
| `-d`, `--dest` | Target motor ID | 1 |
| `-b`, `--baud` | Baud rate | 115200 |
| `-w`, `--boot-wait` | Boot wait seconds (0=skip) | 0 |

Boot output (only after power cycle/reflash):
```
FAM=2 ERR=0 SPI_MODE=0
```
`ERR=0` = encoder OK. If `ERR=1`, check `aux1.spi.mode` config.
Use `--no-boot-wait` to skip waiting for boot output.

## Motor ID

Each motor on the bus needs a unique ID (default: 1).

```
conf get id.id          # Read current ID
conf set id.id 2        # Set ID to 2
conf write              # Save to flash
```

After changing, reconnect with the new destination:
```bash
python uart_shim_test.py -p COM5 -d 2
```

## Initial Motor Setup

Run these once after a fresh `conf default` or first flash:

```
conf set motor.poles 28
conf set motor.Kv 11.54
conf set motor.resistance_ohm 0.22
conf set servo.max_current_A 5.0
conf set servo.max_voltage 24.0
conf set servo.default_timeout_s 20.0
conf set servo.fault_temperature 100.0
conf set servopos.position_min -100.0
conf set servopos.position_max 100.0
conf write
```

> **motor.poles = 28** — This is total poles (magnets), NOT pole pairs.
> The firmware formula uses `poles * 0.5`, so 28 total = 14 pole pairs.

Then run phase calibration:

```
calibrate
```

This sweeps the motor through one electrical revolution, measures the
encoder offset, and writes it to flash. The motor will move slightly.

## Commands Reference

### Status & Control

| Command | Description | Example |
|---------|-------------|---------|
| `query` / `q` | Read motor state | `query` |
| `stop` | Stop motor, clear mode | `stop` |
| `watch` | Live status at 2 Hz | `watch` (Ctrl+C to stop) |
| `brake` | Active brake mode | `brake` |
| `fault-clear` | Clear fault flags | `fault-clear` |

### Motor Movement

| Command | Description | Example |
|---------|-------------|---------|
| `move <rev> [V]` | Move by N motor revolutions | `move 8` (full output rev) |
| `voltage <d> <q>` | Raw d/q voltage | `voltage 0 2.0` |
| `vfoc <θ> <V>` | Voltage at electrical angle (rad) | `vfoc 1.57 2.0` |
| `pos <pos> <vel>` | Firmware position mode | `pos 0 0.1` |
| `calibrate [V]` | Phase offset calibration | `calibrate 3.0` |
| `move <rev> [V]` | vfoc sweep (reliable) | `move 8 2.0` (full output rev) |

### Configuration

| Command | Description | Example |
|---------|-------------|---------|
| `conf set <key> <val>` | Set config value | `conf set motor.poles 28` |
| `conf get <key>` | Read config value | `conf get motor.Kv` |
| `conf write` | Save config to flash | `conf write` |
| `conf default` | Reset all to defaults | `conf default` (⚠ power cycle after) |
| `conf list` | List config sections | `conf list` |
| `conf enumerate <sec>` | List keys in section | `conf enumerate servo` |
| `conf get id.id` | Read motor ID | `conf get id.id` |
| `conf set id.id <N>` | Set motor ID | `conf set id.id 2` |
| `raw <hex>` | Raw multiplex frame | `raw 1D06` |

### Common Configuration Keys

#### Motor Parameters
| Key | Description | Default | GIM6010-8 |
|-----|-------------|---------|-----------|
| `motor.poles` | Total pole count (NOT pairs) | 1 | 28 |
| `motor.Kv` | Speed constant (rpm/V) | 0 | 11.54 |
| `motor.resistance_ohm` | Phase resistance (Ω) | 0 | 0.22 |

#### Position PID (`servo.pid_position`)
| Key | Description | Example |
|-----|-------------|---------|
| `servo.pid_position.kp` | Proportional gain | `conf set servo.pid_position.kp 50.0` |
| `servo.pid_position.ki` | Integral gain | `conf set servo.pid_position.ki 0.0` |
| `servo.pid_position.kd` | Derivative gain | `conf set servo.pid_position.kd 1.0` |

#### Current PID (`servo.pid_dq`)
| Key | Description | Example |
|-----|-------------|---------|
| `servo.pid_dq.kp` | Current loop proportional | `conf set servo.pid_dq.kp 1.0` |
| `servo.pid_dq.ki` | Current loop integral | `conf set servo.pid_dq.ki 200.0` |

#### Servo Limits
| Key | Description | Default | Example |
|-----|-------------|---------|---------|
| `servo.max_current_A` | Max phase current | 20.0 (fam 2) | `conf set servo.max_current_A 5.0` |
| `servo.max_voltage` | Max output voltage | 24.0 | `conf set servo.max_voltage 24.0` |
| `servo.default_timeout_s` | Position cmd timeout | 0.1 | `conf set servo.default_timeout_s 20.0` |
| `servo.fault_temperature` | FET temp fault (°C) | 78.0 | `conf set servo.fault_temperature 100.0` |
| `servo.temperature_margin` | Derate before fault (°C) | 20.0 | `conf set servo.temperature_margin 20.0` |
| `servo.voltage_mode_control` | Voltage-based pos mode | 0 | `conf set servo.voltage_mode_control 1` |

#### Velocity & Acceleration Limits
| Key | Description | Default | Example |
|-----|-------------|---------|---------|
| `servo.max_velocity` | Max velocity (rev/s) | 500.0 | `conf set servo.max_velocity 50.0` |
| `servo.default_velocity_limit` | Pos mode vel limit | disabled | `conf set servo.default_velocity_limit 5.0` |
| `servo.default_accel_limit` | Pos mode accel limit | disabled | `conf set servo.default_accel_limit 10.0` |

#### Timeout Behavior
| Key | Description | Default | Example |
|-----|-------------|---------|---------|
| `servo.default_timeout_s` | Seconds before timeout | 0.1 | `conf set servo.default_timeout_s 20.0` |
| `servo.timeout_max_torque_Nm` | Torque limit in timeout | 5.0 | `conf set servo.timeout_max_torque_Nm 2.0` |
| `servo.timeout_mode` | 0=stop, 10=hold, 12=zero_vel, 15=brake | 12 | `conf set servo.timeout_mode 0` |

#### Feedforward
| Key | Description | Default |
|-----|-------------|---------|
| `servo.current_feedforward` | R*I feedforward | 1.0 |
| `servo.bemf_feedforward` | Back-EMF feedforward | 0.0 |
| `servo.inertia_feedforward` | Accel feedforward | 0.0 |

#### Motor Temperature
| Key | Description | Default | Example |
|-----|-------------|---------|---------|
| `servo.motor_fault_temperature` | Motor temp fault (°C) | disabled | `conf set servo.motor_fault_temperature 80.0` |
| `servo.motor_temperature_margin` | Derate margin (°C) | 20.0 | `conf set servo.motor_temperature_margin 10.0` |

#### Position Limits
| Key | Description | Default | Example |
|-----|-------------|---------|---------|
| `servopos.position_min` | Min position (rev) | -0.01 | `conf set servopos.position_min -100.0` |
| `servopos.position_max` | Max position (rev) | 0.01 | `conf set servopos.position_max 100.0` |

#### Encoder / Motor Position
| Key | Description | Example |
|-----|-------------|---------|
| `motor_position.sources.0.type` | Source type (1=SPI) | `conf get motor_position.sources.0.type` |
| `motor_position.sources.0.cpr` | Counts per rev | `conf get motor_position.sources.0.cpr` |
| `motor_position.sources.0.offset` | Phase offset (set by `calibrate`) | `conf get motor_position.sources.0.offset` |
| `motor_position.sources.0.sign` | Direction (1 or -1) | `conf get motor_position.sources.0.sign` |
| `motor_position.rotor_to_output_ratio` | Gear ratio | `conf get motor_position.rotor_to_output_ratio` |

#### SPI Encoder
| Key | Description | Example |
|-----|-------------|---------|
| `aux1.spi.mode` | SPI mode (0=AS5047) | `conf set aux1.spi.mode 0` |

### Diagnostics

| Command | Description | Example |
|---------|-------------|---------|
| `tel list` | List telemetry channels | `tel list` |
| `tel get <ch>` | Read telemetry channel | `tel get servo_stats` |
| `diag <cmd>` | Diagnostic command | `diag d rezero` |

## Movement Examples

### Small precise movement (1° output)
```
move 0.044
```
0.044 motor rev × 45° per motor rev ÷ 8 gear = 1° output

### 10° output rotation
```
move 0.44
```

### Full output revolution (360°)
```
move 8
```
8 motor revolutions ÷ 8:1 gear = 1 output revolution

### Reverse full revolution
```
move -8
```

### Gentle movement (lower voltage, less heat)
```
move 8 1.5
```

### Fast movement (higher voltage)
```
move 0.44 3.0
```

## Trapezoidal Motion Profiles

The firmware can generate trapezoidal velocity profiles (slow start → cruise → slow stop)
when using position mode with velocity and acceleration limits.

### Setup
```
conf set servo.default_velocity_limit 2.0   # Max speed (rev/s)
conf set servo.default_accel_limit 5.0      # Ramp rate (rev/s²)
conf write
```

### Usage
```
pos 0 2.0       # Move to position 0 at up to 2 rev/s
```

The firmware automatically:
1. **Accelerates** at `default_accel_limit` (5.0 rev/s²)
2. **Cruises** at `default_velocity_limit` (2.0 rev/s)
3. **Decelerates** at `default_accel_limit` to stop precisely at target

### Disable limits (no ramp, full speed)
```
conf set servo.default_velocity_limit 10000.0
conf set servo.default_accel_limit 10000.0
conf write
```
Setting very large values effectively disables the limits. The velocity and acceleration limits are **disabled by default**.

## Unit Conversions

| Unit | Value |
|------|-------|
| 1 motor revolution | 45° output (360°/8) |
| 1 output revolution | 8 motor revolutions |
| 1° output | 0.0222 motor rev |
| 1 electrical revolution | 0.0714 motor rev (1/14) |
| Position units | Motor revolutions |

## Common Faults

| Code | Name | Cause |
|------|------|-------|
| 0 | None | All OK |
| 39 | kStartOutsideLimit | Position outside min/max bounds |
| 98 | kLimitMaxVoltage | Voltage cap reached (info only) |
| 99 | kLimitMaxCurrent | Current cap reached (info only) |
| 100 | kLimitFetTemperature | FET temperature derate active |

Faults 96-103 are **soft limits** — the motor continues operating but with reduced output.

## Troubleshooting

### Motor doesn't move
1. Check temperature: `query` — if Temp > 58°C, wait for cooldown
2. Check fault: if F≠0, run `stop` then `fault-clear`
3. Verify config: `conf get motor.poles` should be 28

### ERR=1 on boot (SPI encoder error)
```
conf set aux1.spi.mode 0
conf write
```
Power cycle after.

### conf write has no response
Normal — flash write takes longer than serial poll timeout. The write succeeded.

### conf default breaks communication
Power cycle after `conf default`. It resets SPI config causing a temporary hang.

## Architecture

```
Python CLI (uart_shim_test.py)
    │
    ├── Serial UART @ 115200 baud
    │
    ├── Multiplex Protocol (binary frames)
    │   ├── Write registers (mode, position, voltage)
    │   ├── Read registers (query: pos, vel, torque, temp)
    │   └── Tunnel (text config/diagnostic commands)
    │
moteus-c1 firmware (moteus.cc)
    │
    ├── UART RX → multiplex parser
    ├── Register read/write handler
    ├── Config tunnel (conf set/get/write)
    └── BldcServo ISR (40 kHz motor control)
```
