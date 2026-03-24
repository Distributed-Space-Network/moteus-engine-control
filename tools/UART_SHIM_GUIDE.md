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

#### Motor Parameters (`motor.*`)
| Key | Description | Default | GIM6010-8 |
|-----|-------------|---------|-----------|
| `motor.poles` | Total pole count (NOT pairs) | 1 | 28 |
| `motor.Kv` | Speed constant (rpm/V) | 0 | 11.54 |
| `motor.resistance_ohm` | Phase resistance (Ω) | 0 | 0.22 |
| `motor.phase_invert` | Swap phase wiring | 0 | - |
| `motor.offset` | Commutation offset table (64 values) | all 0 | set by calibration |

#### Position PID (`servo.pid_position.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.pid_position.kp` | Proportional gain | 0 |
| `servo.pid_position.ki` | Integral gain | 0 |
| `servo.pid_position.kd` | Derivative gain | 0 |
| `servo.pid_position.ilimit` | Integral windup limit | 0 |

#### Current PID (`servo.pid_dq.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.pid_dq.kp` | Current loop proportional | 0 |
| `servo.pid_dq.ki` | Current loop integral | 0 |

#### PWM & Hardware (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.pwm_rate_hz` | PWM/control loop frequency (Hz) | 40000 |
| `servo.current_sense_ohm` | Sense resistor value (Ω) | hw-dependent |

#### Servo Limits (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.max_current_A` | Max phase current (A) | 20.0 (fam 2) |
| `servo.derate_current_A` | Current derate offset (A) | -3.0 (fam 2) |
| `servo.max_voltage` | Max output voltage (V) | hw-dependent |
| `servo.max_power_W` | Max power output (W) | disabled |
| `servo.max_velocity` | Max motor velocity (rev/s) | 500.0 |
| `servo.max_velocity_derate` | Velocity derate range (rev/s) | 2.0 |

#### Voltage & Current Modes (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.voltage_mode_control` | Use voltage instead of current PID | 0 (off) |
| `servo.fixed_voltage_mode` | Open-loop voltage mode (no encoder) | 0 (off) |
| `servo.fixed_voltage_control_V` | Voltage for fixed mode | 0.0 |

#### Velocity & Acceleration Limits (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.default_velocity_limit` | Position mode max speed (rev/s) | disabled |
| `servo.default_accel_limit` | Position mode max accel (rev/s²) | disabled |

#### Timeout Behavior (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.default_timeout_s` | Seconds before timeout | 0.1 |
| `servo.timeout_max_torque_Nm` | Torque limit during timeout | 5.0 |
| `servo.timeout_mode` | Timeout action: 0=stop, 10=hold, 12=zero_vel, 15=brake | 12 |

#### Feedforward (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.current_feedforward` | R×I voltage feedforward | 1.0 |
| `servo.bemf_feedforward` | Back-EMF velocity feedforward | 0.0 |
| `servo.inertia_feedforward` | Acceleration torque feedforward | 0.0 |

#### Slip Detection (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.max_position_slip` | Max position error before fault (rev) | disabled |
| `servo.max_velocity_slip` | Max velocity error before fault (rev/s) | disabled |

#### Temperature (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.fault_temperature` | FET temp fault threshold (°C) | 78.0 |
| `servo.temperature_margin` | Derate starts at fault−margin (°C) | 20.0 |
| `servo.enable_motor_temperature` | Enable motor thermistor | 0 (off) |
| `servo.motor_fault_temperature` | Motor temp fault (°C) | disabled |
| `servo.motor_temperature_margin` | Motor derate margin (°C) | 20.0 |

#### Flux Braking (`servo.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servo.flux_brake_margin_voltage` | Volts below max_voltage to start braking | 3.0 |
| `servo.flux_brake_resistance_ohm` | Brake resistor value (Ω) | 0.025 |

#### Position Limits (`servopos.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `servopos.position_min` | Min position (rev) | -0.01 |
| `servopos.position_max` | Max position (rev) | 0.01 |

#### Encoder / Motor Position (`motor_position.*`)
| Key | Description |
|-----|-------------|
| `motor_position.sources.0.type` | Source type (0=none, 1=SPI, 2=UART, 3=quad, 4=hall) |
| `motor_position.sources.0.aux_number` | AUX port (1 or 2) |
| `motor_position.sources.0.cpr` | Counts per revolution |
| `motor_position.sources.0.offset` | Phase offset (set by `calibrate`) |
| `motor_position.sources.0.sign` | Direction (1 or -1) |
| `motor_position.sources.0.pll_filter_hz` | PLL filter bandwidth (Hz) |
| `motor_position.commutation_source` | Which source for commutation (0-based) |
| `motor_position.output.source` | Which source for position output (0-based) |
| `motor_position.output.offset` | Output position offset |
| `motor_position.output.sign` | Output direction (1 or -1) |
| `motor_position.rotor_to_output_ratio` | Gear ratio (e.g. 0.125 for 8:1) |

#### SPI Encoder (`aux1.*`)
| Key | Description |
|-----|-------------|
| `aux1.spi.mode` | 0=AS5047, 1=disabled, 6=MA600, 7=board_default |

#### Device Identity (`id.*`)
| Key | Description | Default |
|-----|-------------|---------|
| `id.id` | Motor bus ID | 1 |

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
