# Helios Serial Protocol Specification

## Overview

The Helios serial protocol is a binary packet-based protocol for communicating with Helios Ignition Control Units (ICUs) over UART. The protocol provides command/control capabilities and real-time telemetry from the burner system.

**Transport Layer:**
- UART → LIN Translation IC
- Default Baud Rate: 115200
- Data Format: 8N1 (8 data bits, no parity, 1 stop bit)
- Flow Control: None

**Protocol Characteristics:**
- Binary packet format with framing
- CRC-16-CCITT for error detection
- Fixed maximum packet size: 64 bytes
- Bidirectional communication
- Periodic telemetry broadcast (100ms)

**Network Architecture:**
- **Master Device:** External controller/host
- **Slave Device:** Helios ICU (Ignition Control Unit)
- Master initiates commands, slave responds with telemetry and status
- Slave autonomously broadcasts periodic telemetry (100ms)

---

## Packet Format

All packets follow this structure:

```
+----------+--------+----------+---------+-----------+--------+
| START    | LENGTH | MSG_TYPE | PAYLOAD | CRC-16    | END    |
| 1 byte   | 1 byte | 1 byte   | N bytes | 2 bytes   | 1 byte |
+----------+--------+----------+---------+-----------+--------+
| 0x7E     | 0-58   | 0x00-FF  | ...     | MSB, LSB  | 0x7F   |
+----------+--------+----------+---------+-----------+--------+
```

### Field Descriptions

| Field | Size | Description |
|-------|------|-------------|
| **START** | 1 byte | Start delimiter: `0x7E` |
| **LENGTH** | 1 byte | Payload length (0-58 bytes, excludes framing/CRC) |
| **MSG_TYPE** | 1 byte | Message type identifier (see Message Types) |
| **PAYLOAD** | 0-58 bytes | Message-specific payload data |
| **CRC-16** | 2 bytes | CRC-16-CCITT (poly 0x1021, init 0xFFFF) over LENGTH + MSG_TYPE + PAYLOAD |
| **END** | 1 byte | End delimiter: `0x7F` |

**Total Packet Size:** 6 bytes (framing/CRC) + payload length = 6-64 bytes

### CRC Calculation

**Algorithm:** CRC-16-CCITT
- **Polynomial:** 0x1021
- **Initial Value:** 0xFFFF
- **XOR Out:** 0x0000
- **Reflect In:** False
- **Reflect Out:** False

**Zephyr Implementation:** Use `crc16_ccitt()` from `<zephyr/sys/crc.h>`
- See [Zephyr CRC API Documentation](https://docs.zephyrproject.org/latest/doxygen/html/group__crc.html)

**CRC Coverage:** LENGTH + MSG_TYPE + PAYLOAD fields
- CRC does NOT cover START or END delimiters
- CRC transmitted MSB first (big-endian)

**Example CRC Calculation (Python):**
```python
def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
```

### Byte Stuffing

To prevent confusion with START (0x7E) and END (0x7F) delimiters appearing in the payload or CRC:

**Escape Sequence:** Use `0x7D` as escape byte

| Original Byte | Escaped Sequence |
|---------------|------------------|
| 0x7E (START)  | 0x7D 0x5E       |
| 0x7F (END)    | 0x7D 0x5F       |
| 0x7D (ESC)    | 0x7D 0x5D       |

**Note:** Byte stuffing is applied AFTER CRC calculation and BEFORE framing.

---

## Message Types

### Command Messages (Host → Helios)

| MSG_TYPE | Name | Description | Payload Size |
|----------|------|-------------|--------------|
| 0x10 | STATE_COMMAND | Set system mode/state | 8 bytes |
| 0x11 | MOTOR_COMMAND | Set motor RPM | 8 bytes |
| 0x12 | PUMP_COMMAND | Set pump rate | 8 bytes |
| 0x13 | GLOW_COMMAND | Control glow plug | 8 bytes |
| 0x14 | TEMP_COMMAND | Temperature controller config | 20 bytes |
| 0x1F | PING_REQUEST | Heartbeat/connectivity check | 0 bytes |

### Data Messages (Helios → Host)

| MSG_TYPE | Name | Description | Payload Size | Send Rate |
|----------|------|-------------|--------------|-----------|
| 0x20 | STATE_DATA | System state and status | 16 bytes | 250 ms |
| 0x21 | MOTOR_DATA | Motor telemetry | 32 bytes | 100 ms |
| 0x22 | PUMP_DATA | Pump status | 16 bytes | On event |
| 0x23 | GLOW_DATA | Glow plug status | 12 bytes | On event |
| 0x24 | TEMP_DATA | Temperature readings | 32 bytes | 100 ms |
| 0x25 | TELEMETRY_BUNDLE | Consolidated telemetry | Variable: 37-61 bytes | 100 ms |
| 0x2F | PING_RESPONSE | Heartbeat response | 4 bytes | On request |

### Error Messages (Helios → Host)

| MSG_TYPE | Name | Description | Payload Size |
|----------|------|-------------|--------------|
| 0xE0 | ERROR_INVALID_MSG | Invalid message received | 4 bytes |
| 0xE1 | ERROR_CRC_FAIL | CRC validation failed | 4 bytes |
| 0xE2 | ERROR_INVALID_CMD | Command validation failed | 4 bytes |
| 0xE3 | ERROR_STATE_REJECT | Command rejected by state machine | 4 bytes |

---

## Command Message Formats

### 0x10 - STATE_COMMAND

Set system operating mode.

**Payload Structure (8 bytes):**
```
+------+----------+
| mode | argument |
+------+----------+
| u32  | i32      |
+------+----------+
```

**Fields:**
- **mode** (u32): Operating mode
  - 0 = IDLE_MODE
  - 1 = FAN_MODE
  - 2 = HEAT_MODE
  - 3 = EMERGENCY
- **argument** (i32): Mode-specific parameter
  - FAN_MODE: Target RPM (800-3400)
  - HEAT_MODE: Pump rate in milliseconds
  - IDLE_MODE/EMERGENCY: Ignored (set to 0)

**Example:** Enter fan mode at 2500 RPM
```
7E 08 10 00 00 00 01 00 00 09 C4 [CRC-H] [CRC-L] 7F
         ^^mode=1    ^^argument=2500
```

### 0x11 - MOTOR_COMMAND

Control motor (fan) speed.

**Payload Structure (8 bytes):**
```
+-------+------+
| motor | rpm  |
+-------+------+
| i32   | i32  |
+-------+------+
```

**Fields:**
- **motor** (i32): Motor index (0-9, typically 0)
- **rpm** (i32): Target RPM (0 = stop, 800-3400 = run)

**Validation:**
- RPM must be 0 OR within motor's min/max range
- Invalid RPM returns ERROR_INVALID_CMD

### 0x12 - PUMP_COMMAND

Control fuel pump rate.

**Payload Structure (8 bytes):**
```
+------+---------+
| pump | rate_ms |
+------+---------+
| i32  | i32     |
+------+---------+
```

**Fields:**
- **pump** (i32): Pump index (0-9, typically 0)
- **rate_ms** (i32): Pulse interval in milliseconds (0 = stop, ≥100 = run)

**Validation:**
- rate_ms must be 0 OR ≥ 100 (pulse duration + recovery time)

### 0x13 - GLOW_COMMAND

Control glow plug heating.

**Payload Structure (8 bytes):**
```
+------+----------+
| glow | duration |
+------+----------+
| i32  | i32      |
+------+----------+
```

**Fields:**
- **glow** (i32): Glow plug index (0-9, typically 0)
- **duration** (i32): Burn duration in milliseconds (0 = off, max = 300000)

**Validation:**
- duration must be 0-300000 ms (0-5 minutes)
- Cannot re-light already lit glow plug

### 0x14 - TEMP_COMMAND

Configure temperature controller.

**Payload Structure (20 bytes):**
```
+-------------+------+-------------+------------------+
| thermometer | type | motor_index | target_temp      |
+-------------+------+-------------+------------------+
| i32         | u32  | i32         | f64              |
+-------------+------+-------------+------------------+
```

**Fields:**
- **thermometer** (i32): Temperature controller index (0-9, typically 0)
- **type** (u32): Command type
  - 0 = WATCH_MOTOR (associate with motor)
  - 1 = UNWATCH_MOTOR (stop monitoring)
  - 2 = ENABLE_RPM_CONTROL (enable PID)
  - 3 = DISABLE_RPM_CONTROL (disable PID)
  - 4 = SET_TARGET_TEMP (set temperature target)
- **motor_index** (i32): Motor to control (used with WATCH_MOTOR)
- **target_temp** (f64): Target temperature in Celsius (used with SET_TARGET_TEMP)

**Note:** f64 is IEEE 754 double-precision, little-endian byte order

### 0x1F - PING_REQUEST

Connectivity check / heartbeat.

**Payload:** None (0 bytes)

**Response:** PING_RESPONSE (0x2F) with uptime

---

## Data Message Formats

### 0x20 - STATE_DATA

System state and error status.

**Payload Structure (16 bytes):**
```
+-------+------+-------+-----------+----------+
| error | code | state | timestamp | padding  |
+-------+------+-------+-----------+----------+
| u8    | i32  | u32   | u32       | 3 bytes  |
+-------+------+-------+-----------+----------+
```

**Fields:**
- **error** (u8): Error flag (0 = no error, 1 = error)
- **code** (i32): Error code (application-specific)
- **state** (u32): Current helios_state
  - 0 = INITIALIZING
  - 1 = IDLE
  - 2 = BLOWING
  - 3 = PREHEAT
  - 4 = PREHEAT_STAGE_2
  - 5 = HEATING
  - 6 = COOLING
  - 7 = ERROR
  - 8 = E_STOP
- **timestamp** (u32): Timestamp in microseconds (wraps at 2^32)
- **padding** (3 bytes): Reserved for alignment

**Send Rate:** Every 250 ms

### 0x21 - MOTOR_DATA

Motor telemetry including RPM and PWM feedback.

**Payload Structure (32 bytes):**
```
+-------+-----------+------+--------+---------+---------+------+---------+
| motor | timestamp | rpm  | target | max_rpm | min_rpm | pwm  | pwm_max |
+-------+-----------+------+--------+---------+---------+------+---------+
| i32   | u32       | i32  | i32    | i32     | i32     | i32  | i32     |
+-------+-----------+------+--------+---------+---------+------+---------+
```

**Fields:**
- **motor** (i32): Motor index
- **timestamp** (u32): Reading timestamp in microseconds
- **rpm** (i32): Current measured RPM
- **target** (i32): Target RPM setpoint
- **max_rpm** (i32): Maximum achievable RPM (typically 3400)
- **min_rpm** (i32): Minimum stable RPM (typically 800)
- **pwm** (i32): Current PWM pulse width in nanoseconds
- **pwm_max** (i32): PWM period in nanoseconds

**Send Rate:** Every 100 ms

### 0x22 - PUMP_DATA

Fuel pump status and events.

**Payload Structure (16 bytes):**
```
+------+-----------+------+------+
| pump | timestamp | type | rate |
+------+-----------+------+------+
| i32  | u32       | u32  | i32  |
+------+-----------+------+------+
```

**Fields:**
- **pump** (i32): Pump index
- **timestamp** (u32): Event timestamp in microseconds
- **type** (u32): Event type
  - 0 = INITIALIZING
  - 1 = READY
  - 2 = ERROR
  - 3 = CYCLE_START
  - 4 = PULSE_END
  - 5 = CYCLE_END
- **rate** (i32): Current pump rate in milliseconds

**Send Rate:** On event (state changes, cycle events)

### 0x23 - GLOW_DATA

Glow plug status.

**Payload Structure (12 bytes):**
```
+------+-----------+-----+---------+
| glow | timestamp | lit | padding |
+------+-----------+-----+---------+
| i32  | u32       | u8  | 3 bytes |
+------+-----------+-----+---------+
```

**Fields:**
- **glow** (i32): Glow plug index
- **timestamp** (u32): Status timestamp in microseconds
- **lit** (u8): Lit status (0 = off, 1 = lit)
- **padding** (3 bytes): Reserved for alignment

**Send Rate:** On event (on/off transitions)

### 0x24 - TEMP_DATA

Temperature sensor readings and PID control status.

**Payload Structure (32 bytes):**
```
+-------------+-----------+------+-------------+-------------------+
| thermometer | timestamp | temp | pid_enabled | rpm_ctrl_enabled  |
+-------------+-----------+------+-------------+-------------------+
| i32         | u32       | f64  | u8          | u8                |
+-------------+-----------+------+-------------+-------------------+

+---------------+-----------------+---------+
| watched_motor | target_temp     | padding |
+---------------+-----------------+---------+
| i32           | f64             | 2 bytes |
+---------------+-----------------+---------+
```

**Fields:**
- **thermometer** (i32): Thermometer index
- **timestamp** (u32): Reading timestamp in microseconds
- **temp** (f64): Temperature in Celsius
- **pid_enabled** (u8): PID active flag (0 = off, 1 = on)
- **rpm_ctrl_enabled** (u8): Motor RPM control flag (0 = off, 1 = on)
- **watched_motor** (i32): Motor being controlled (-1 = none)
- **target_temp** (f64): Target temperature for PID control
- **padding** (2 bytes): Reserved for alignment

**Send Rate:** Every 100 ms (after 60-sample warmup period)

### 0x25 - TELEMETRY_BUNDLE

Consolidated telemetry packet for efficient polling with support for multiple motors and temperature sensors.

**Payload Structure (Variable: 30-58 bytes):**
```
+-------+-------+-------------+------------+
| state | error | motor_count | temp_count |
+-------+-------+-------------+------------+
| u32   | u8    | u8          | u8         |
+-------+-------+-------------+------------+

+----------+--------+------+
| For each motor (motor_count × 12 bytes):
+----------+--------+------+
| rpm      | target | pwm  |
+----------+--------+------+
| i32      | i32    | i32  |
+----------+--------+------+

+------+
| For each temperature sensor (temp_count × 8 bytes):
+------+
| temp |
+------+
| f64  |
+------+

+-----------+-----------+--------------+----------------+
| pump_rate | glow_st   | timestamp    | pid_enabled    |
+-----------+-----------+--------------+----------------+
| i32       | u8        | u32          | u8             |
+-----------+-----------+--------------+----------------+

+----------------+---------+
| rpm_ctrl_en    | padding |
+----------------+---------+
| u8             | 1 byte  |
+----------------+---------+
```

**Fields:**
- **state** (u32): Current system state (see STATE_DATA)
- **error** (u8): Error flag
- **motor_count** (u8): Number of motors in this bundle (1-3)
- **temp_count** (u8): Number of temperature sensors in this bundle (1-3)
- **motors[motor_count]**: Array of motor telemetry entries
  - **rpm** (i32): Current motor RPM
  - **target** (i32): Target motor RPM
  - **pwm** (i32): Motor PWM duty in nanoseconds
- **temperatures[temp_count]**: Array of temperature readings
  - **temp** (f64): Temperature in Celsius
- **pump_rate** (i32): Current pump rate in ms
- **glow_status** (u8): Glow plug lit status
- **timestamp** (u32): Bundle timestamp in microseconds
- **pid_enabled** (u8): Temperature PID active
- **rpm_ctrl_enabled** (u8): Motor RPM control active
- **padding** (1 byte): Reserved for alignment

**Payload Size Calculation:**
```
Size = 7 (header) + (motor_count × 12) + (temp_count × 8) + 11 (footer)
```

**Common Configurations:**

| Motors | Temps | Payload Size | Status |
|--------|-------|--------------|--------|
| 1 | 1 | 7 + 12 + 8 + 11 = 38 bytes | ✓ Valid |
| 2 | 1 | 7 + 24 + 8 + 11 = 50 bytes | ✓ Valid |
| 1 | 2 | 7 + 12 + 16 + 11 = 46 bytes | ✓ Valid |
| 2 | 2 | 7 + 24 + 16 + 11 = 58 bytes | ✓ Valid (at limit) |
| 3 | 1 | 7 + 36 + 8 + 11 = 62 bytes | ✗ Exceeds limit |
| 1 | 3 | 7 + 12 + 24 + 11 = 54 bytes | ✓ Valid |
| 3 | 2 | 7 + 36 + 16 + 11 = 70 bytes | ✗ Exceeds limit |
| 3 | 3 | 7 + 36 + 24 + 11 = 78 bytes | ✗ Exceeds limit |

**Maximum Payload Constraint:** 58 bytes (to fit within 64-byte packet with framing)

**Purpose:** Single packet containing all critical telemetry for efficient monitoring. Supports variable number of motors and temperature sensors for different burner configurations.

**Send Rate:** Every 100 ms

**Notes:**
- Arrays are variable-length based on motor_count and temp_count fields
- Receivers must parse both count fields to determine array sizes
- Motor/temperature indices are implicit (array order: 0, 1, 2...)
- For configurations exceeding size limit, use individual messages (MOTOR_DATA, TEMP_DATA) instead
- Recommended: 2 motors + 2 temps or 1 motor + 3 temps for most applications

### 0x2F - PING_RESPONSE

Heartbeat response with system uptime.

**Payload Structure (4 bytes):**
```
+-----------+
| uptime_ms |
+-----------+
| u32       |
+-----------+
```

**Fields:**
- **uptime_ms** (u32): System uptime in milliseconds (wraps at 2^32)

**Send Rate:** On request (response to PING_REQUEST)

---

## Error Message Formats

All error messages share the same structure:

**Payload Structure (4 bytes):**
```
+------------+
| error_code |
+------------+
| u32        |
+------------+
```

### 0xE0 - ERROR_INVALID_MSG
Invalid message format or framing error.

**error_code:**
- 0 = Invalid START byte
- 1 = Invalid END byte
- 2 = Length exceeds maximum
- 3 = Timeout waiting for complete packet

### 0xE1 - ERROR_CRC_FAIL
CRC validation failed.

**error_code:**
- Expected CRC value (lower 16 bits)

### 0xE2 - ERROR_INVALID_CMD
Command validation failed.

**error_code:**
- 0 = Unknown message type
- 1 = Invalid parameter value
- 2 = Invalid device index

### 0xE3 - ERROR_STATE_REJECT
Command rejected by state machine.

**error_code:**
- Current state that rejected the command

---

## Communication Patterns

### 1. Command-Response

Host sends command, Helios may respond with error:

```
Host → Helios:  [STATE_COMMAND: Set HEAT mode]
Helios → Host:  (Success: no response)
                OR
                [ERROR_INVALID_CMD: Invalid parameter]
```

### 2. Periodic Telemetry

Helios broadcasts telemetry at fixed intervals:

```
Every 100ms:  TELEMETRY_BUNDLE (consolidated data)
              OR
              MOTOR_DATA + TEMP_DATA (individual messages)

Every 250ms:  STATE_DATA
```

### 3. Event-Driven Updates

Helios sends data messages on state changes:

```
PUMP_DATA:  Sent on pump cycle events
GLOW_DATA:  Sent when glow plug turns on/off
```

### 4. Heartbeat

Host can check connectivity:

```
Host → Helios:  PING_REQUEST
Helios → Host:  PING_RESPONSE (with uptime)
```

---

## Example Packets

### Example 1: Set Fan Mode to 2500 RPM

**Command:** STATE_COMMAND (0x10)
- mode = FAN_MODE (1)
- argument = 2500 RPM

```
Raw bytes (before stuffing):
7E              START
08              LENGTH = 8
10              MSG_TYPE = STATE_COMMAND
00 00 00 01     mode = 1 (little-endian u32)
00 00 09 C4     argument = 2500 (little-endian i32)
[CRC-H] [CRC-L] CRC-16 over bytes 08 10 00 00 00 01 00 00 09 C4
7F              END
```

**CRC Calculation:**
```
Data: 08 10 00 00 00 01 00 00 09 C4
CRC-16-CCITT: 0xXXXX (calculated)
```

### Example 2: Temperature Data

**Data:** TEMP_DATA (0x24)
- thermometer = 0
- timestamp = 123456789 μs
- temp = 225.5°C
- pid_enabled = 1
- rpm_ctrl_enabled = 1
- watched_motor = 0
- target_temp = 230.0°C

```
Raw bytes:
7E                          START
20                          LENGTH = 32
24                          MSG_TYPE = TEMP_DATA
00 00 00 00                 thermometer = 0
15 CD 5B 07                 timestamp = 123456789
00 00 00 00 00 28 6C 40     temp = 225.5 (f64, little-endian)
01                          pid_enabled = 1
01                          rpm_ctrl_enabled = 1
00 00 00 00                 watched_motor = 0
00 00 00 00 00 C0 6C 40     target_temp = 230.0 (f64)
00 00                       padding
[CRC-H] [CRC-L]             CRC-16
7F                          END
```

### Example 3: Telemetry Bundle

**Data:** TELEMETRY_BUNDLE (0x25)

Consolidated packet sent every 100ms with all critical telemetry.

---

## Data Type Encodings

All multi-byte integers use **little-endian** byte order.

| Type | Size | Format | Range |
|------|------|--------|-------|
| u8   | 1 byte | Unsigned integer | 0 to 255 |
| i32  | 4 bytes | Signed integer (LE) | -2^31 to 2^31-1 |
| u32  | 4 bytes | Unsigned integer (LE) | 0 to 2^32-1 |
| f64  | 8 bytes | IEEE 754 double (LE) | ±1.7E±308 |

**Float Encoding Example (225.5°C):**
```
IEEE 754 double: 0x406C280000000000
Little-endian:   00 00 00 00 00 28 6C 40
```

---

## Implementation Requirements

### UART Configuration

- **Baud Rate:** 115200
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
- **Flow Control:** None

### Buffer Requirements

**Receive Buffer:** Minimum 128 bytes (2x max packet size)
**Transmit Buffer:** Minimum 128 bytes

### Zephyr Configuration

Required Kconfig options:
```
CONFIG_SERIAL=y    # UART/serial driver support
CONFIG_CRC=y       # CRC library support
```

**Note:** `CONFIG_CRC16` is not required - the CRC-16 functions are available when `CONFIG_CRC=y` is enabled.

**Zephyr CRC Documentation:**
- [CRC API Reference](https://docs.zephyrproject.org/latest/doxygen/html/group__crc.html)
- [CRC Configuration Options](https://docs.zephyrproject.org/latest/kconfig.html#!CONFIG_CRC)

### Device Tree

UART node must be defined and aliased:
```dts
/ {
    aliases {
        helios-uart = &uart0;
    };
};

&uart0 {
    status = "okay";
    current-speed = <115200>;
};
```

---

## Error Handling

### Transmit Errors
- **Buffer Full:** Drop oldest packet or block until space available
- **UART Error:** Log error, attempt retransmit once

### Receive Errors
- **CRC Failure:** Send ERROR_CRC_FAIL, discard packet
- **Framing Error:** Send ERROR_INVALID_MSG, resync to next START byte
- **Timeout:** Discard partial packet after 100ms silence
- **Invalid Command:** Send ERROR_INVALID_CMD with error code

### Recovery
- On 3 consecutive CRC failures, suggest baud rate mismatch
- On persistent framing errors, suggest physical connection check

---

## Performance Characteristics

### Throughput

**100ms Telemetry Period:**
- TELEMETRY_BUNDLE: 58 bytes (after stuffing) = 580 bytes/sec
- Individual messages (MOTOR + TEMP + STATE): ~90 bytes = 900 bytes/sec

**At 115200 baud:**
- Effective throughput: ~11,520 bytes/sec
- Telemetry overhead: ~5-8% bandwidth utilization

### Latency

- **Command Processing:** < 5ms (zbus publish + state machine cycle)
- **Telemetry Delay:** 0-100ms (depends on timing within broadcast cycle)

### Reliability

- **CRC-16:** Detects all single-bit and double-bit errors
- **Byte Stuffing:** Prevents false START/END detection
- **Framing:** Robust resynchronization on errors

---

## Future Enhancements

1. **Compression:** Add optional payload compression for telemetry
2. **Sequencing:** Add sequence numbers to detect dropped packets
3. **Acknowledgments:** Add ACK/NAK for critical commands
4. **Batching:** Bundle multiple commands in single packet
5. **Firmware Update:** Add bootloader protocol for OTA updates
6. **Extended Addressing:** Support for multi-device bus (LIN network)

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-31 | Helios Team | Initial specification |

---

## References

- **CRC-16-CCITT:** ITU-T Recommendation V.41
- **IEEE 754:** IEEE Standard for Floating-Point Arithmetic
- **LIN Specification:** LIN Consortium, LIN 2.0 Protocol Specification
- **Zephyr RTOS:** https://docs.zephyrproject.org/
- **Zephyr CRC API:** https://docs.zephyrproject.org/latest/doxygen/html/group__crc.html
- **Zephyr Serial Driver:** https://docs.zephyrproject.org/latest/hardware/peripherals/uart.html
