# Helios Project - AI Assistant Guide

> **Note:** This file documents the Helios project specifically.
> Always read the [Thermoquad Organization CLAUDE.md](../../CLAUDE.md) first
> for organization-wide structure and conventions.

## Project Overview

**Helios** is a liquid fuel burner ignition control unit (ICU) firmware for diesel-like fuels including diesel, kerosene, used vegetable oil, and used motor oil. It runs on Zephyr RTOS and manages the complete combustion lifecycle from ignition to safe shutdown.

**Hardware:**
- **Development Board:** Raspberry Pi Pico 2 (RP2350a, ARM Cortex-M33)
- **Production Target:** Custom board with RP2354A (pending burn test completion)
- **Current Status:** Burn testing on Pico 2 development board

**Operating Temperature:** 220-230°C (normal), 275°C (emergency stop), 300°C (hardware damage threshold)

---

## Architecture

### Core Design Patterns

1. **Event-Driven Architecture**: Uses Zephyr's Zbus for inter-thread communication
2. **State Machine**: Zephyr SMF (State Machine Framework) manages burner lifecycle
3. **PID Control**: Temperature and motor speed control with configurable gains
4. **Thread-Per-Controller**: Dedicated threads for each hardware subsystem

### Threading Model

**All threads are defined in `src/main.c` using `K_THREAD_DEFINE`.**

| Thread ID | Entry Point | Implementation File | Purpose | Rate | Priority |
|-----------|-------------|-------------------|---------|------|----------|
| `helios_state_id` | `helios_state_runner()` | `src/state.c` | Main state machine | 5ms | 3 |
| `temperature_controller_id` | `temperature_controller()` | `src/controllers/temperature.c` | Temperature sensing & PID | 10ms | -1 |
| `motor_controller_id` | `motor_controller()` | `src/controllers/motor.c` | Motor/fan PWM control | 25ms | -1 |
| `pump_controller_id` | `pump_controller()` | `src/controllers/pump.c` | Fuel pump solenoid control | Variable | -1 |
| `glow_controller_id` | `glow_controller()` | `src/controllers/glow.c` | Glow plug heating control | Variable | -1 |
| `serial_rx_id` | `serial_rx_thread()` | `src/communications/serial_handler.c` | Serial RX & timeout checking | 1s | 5 |
| `serial_tx_id` | `serial_tx_thread()` | `src/communications/serial_handler.c` | Telemetry broadcasting | 100ms | 6 |

**Thread Entry Points:** All thread entry point functions are declared in `include/helios/threads.h`.

**Priority Notes:**
- Priority 3: State machine (higher priority for critical control)
- Priority 5: Serial RX (responsive to incoming commands)
- Priority 6: Serial TX (lower priority for telemetry)
- Priority -1: Cooperative scheduling (controllers run when ready)

### Communication: Zbus Message Bus

All inter-thread communication uses Zephyr Zbus with paired command/data channels:

**Channel Pairs:**
- `state_command_chan` / `state_data_chan` - System mode control
- `motor_command_chan` / `motor_data_chan` - Motor control & telemetry
- `pump_command_chan` / `pump_data_chan` - Pump control & events
- `glow_command_chan` / `glow_data_chan` - Glow plug control & status
- `temperature_command_chan` / `temperature_data_chan` - Temperature & PID control

**Message Structures:** Defined in `include/helios/zbus.h`

**Pattern:**
- Commands are validated before execution (validators prevent invalid states)
- Data messages include timestamps (microseconds)
- Controllers publish data at fixed rates or on events
- State machine subscribes to all data channels

---

## State Machine

**File:** `src/state.c`

**Framework:** Zephyr SMF (Hierarchical State Machine Framework)

**States:**

| State | Purpose | Entry Function | Exit Function |
|-------|---------|----------------|---------------|
| INITIALIZING | Startup, sensor validation | - | - |
| IDLE | At rest, all systems off | `idle_entry()` | - |
| BLOWING | Fan-only mode (no combustion) | - | `stop_blowing_helios()` |
| PREHEAT | Stage 1: Light glow plug, start fuel | `start_preheat()` | `end_preheat()` |
| PREHEAT_STAGE_2 | Stage 2: Stabilize combustion | - | - |
| HEATING | Normal operation with temp PID | `heating_entry()` | `heating_exit()` |
| COOLING | Safe shutdown with carbon prevention | `cooldown_entry()` | `cooldown_exit()` |
| ERROR | Fault state requiring user intervention | - | - |
| E_STOP | Emergency shutdown (275°C fault) | `emergency_stop_entry()` | - |

**Key Behaviors:**
- Automatic cooldown when shutting down from operating temperature (>120°C)
- Glow plug assists cooldown at 180-120°C to prevent carbon buildup
- Cooldown tracks return state (IDLE or ERROR) to persist errors
- Temperature PID control enabled/disabled via state entry/exit functions
- Fault temperature (275°C) triggers immediate E_STOP from any state

**Documentation:** See `docs/state_machine.md` for complete state machine specification

---

## Temperature Control

**File:** `src/controllers/temperature.c`

**Key Features:**
- 60-sample moving average for temperature stability (~3 seconds warmup)
- Inverted PID control: higher temperature → higher motor RPM (for cooling)
- Dynamic RPM limits learned from motor capabilities
- Publishes temperature data every 100ms (after warmup)

**PID Configuration:**
```c
#define DEFAULT_TEMP_KP 100.0
#define DEFAULT_TEMP_KI 10.0
#define DEFAULT_TEMP_KD 5.0
```

**Control Flow:**
1. State machine enables PID via `heating_entry()`
2. Temperature controller watches motor 0 for RPM data
3. Target temperature set to 230°C
4. PID output (target RPM) published to motor command channel
5. Motor controller adjusts fan speed to maintain temperature

**Temperature Command Types:**
- `TEMP_CMD_WATCH_MOTOR` - Associate with motor
- `TEMP_CMD_SET_TARGET_TEMP` - Set PID target
- `TEMP_CMD_ENABLE_RPM_CONTROL` - Enable PID
- `TEMP_CMD_DISABLE_RPM_CONTROL` - Disable PID
- `TEMP_CMD_UNWATCH_MOTOR` - Stop monitoring

---

## Motor Control

**File:** `src/controllers/motor.c`

**Key Features:**
- PWM-based motor control with tachometer feedback
- PID controller maintains target RPM
- Supports 800-3400 RPM range (configurable)
- Publishes motor data every 100ms

**PID Configuration:**
```c
#define DEFAULT_KP 4.0
#define DEFAULT_KI 12.0
#define DEFAULT_KD 0.1
```

**Hardware:**
- PWM output to motor driver
- Tachometer input for RPM sensing
- GPIO enable pin for motor driver

**Validation:**
- RPM must be 0 (stop) or within [min_rpm, max_rpm]
- Invalid commands rejected by validator before reaching controller

---

## Fuel Pump Control

**File:** `src/controllers/pump.c`

**Key Features:**
- Solenoid-based pulsed fuel delivery
- Configurable pulse rate (minimum 100ms)
- Self-check on initialization (continuity test)
- Event-driven status updates

**Pump Events:**
- `PUMP_INITIALIZING` - Startup self-test
- `PUMP_READY` - Ready for operation
- `PUMP_ERROR` - Hardware fault detected
- `PUMP_CYCLE_START` - Pulse cycle begins
- `PUMP_PULSE_END` - Solenoid closed
- `PUMP_CYCLE_END` - Cycle complete

**Rates Used:**
- Preheat Stage 1: 500ms
- Preheat Stage 2: 250ms
- Heating: User-specified

---

## Glow Plug Control

**File:** `src/controllers/glow.c`

**Key Features:**
- Timed heating with automatic shutoff
- Maximum duration: 5 minutes (safety timeout)
- Used during preheat and cooldown phases

**Usage Pattern:**
1. **Preheat:** Light for up to 5 minutes to ignite fuel
2. **Cooldown:** Light at 180-120°C to burn off residual fuel/carbon
3. **Safety:** Auto-shutoff prevents overheating

---

## Code Formatting & Style

**Formatter:** clang-format (config in `.clang-format` at project root)

**Conventions:**
- Use `snake_case` for functions and variables
- Use `UPPER_CASE` for constants and macros
- State machine functions follow pattern: `<state_name>_entry()`, `<state_name>_exit()`, `<state_name>()`
- Helper functions are `static` unless needed externally
- All multi-byte values use **little-endian** byte order

**Logging:**
- `LOG_ERR()` - Errors requiring attention
- `LOG_WRN()` - Warnings about unusual conditions
- `LOG_INF()` - Important state changes
- `LOG_DBG()` - Debug information
- `LOG_*_RATELIMIT()` - Rate-limited variants for high-frequency logs

**Comments:**
- Group code into logical blocks with comment headers
- Document non-obvious behavior and safety-critical logic
- Explain "why" not "what" for complex algorithms

---

## File Structure

```
helios/
├── CLAUDE.md                    # This file
├── README.md                    # Project overview
├── CMakeLists.txt              # Build configuration
├── prj.conf                    # Zephyr Kconfig
├── Taskfile.dist.yml           # Task runner commands
├── app.overlay                 # Device tree base
├── boards/
│   └── rpi_pico2_rp2350a_m33.overlay  # Board-specific device tree
├── include/helios/
│   ├── zbus.h                  # Zbus message definitions
│   ├── pid.h                   # PID controller interface
│   ├── threads.h               # Thread declarations
│   └── shell.h                 # Shell command interface
├── src/
│   ├── main.c                  # Application entry point
│   ├── state.c                 # State machine implementation
│   ├── shell.c                 # Interactive shell commands
│   ├── algorithms/
│   │   └── pid.c               # PID controller implementation
│   └── controllers/
│       ├── temperature.c       # Temperature sensing & PID
│       ├── motor.c             # Motor/fan control
│       ├── pump.c              # Fuel pump control
│       └── glow.c              # Glow plug control
└── docs/
    └── state_machine.md        # Complete state machine spec
```

---

## Important Constants

### Temperature Thresholds (src/state.c)

```c
#define FAULT_TEMP 275.0                // Emergency stop trigger
#define PREHEAT_STAGE_2_TEMP 190.0      // Stage 1 → Stage 2 transition
#define PREHEAT_SUCCESS_TEMP 210.0      // Stage 2 → Heating transition
#define HEATING_TARGET_TEMP 230.0       // PID control target
#define FLAME_OUT_TEMP 190.0            // Flame-out detection threshold
#define COOLDOWN_GLOW_START_TEMP 180.0  // Start glow during cooldown
#define COOLDOWN_COMPLETE_TEMP 120.0    // Cooldown complete threshold
```

### Motor RPM (src/state.c)

```c
#define PREHEAT_RPM 2500           // Stage 1 preheat fan speed
#define PREHEAT_STAGE_2_RPM 2800   // Stage 2 preheat fan speed
#define COOLDOWN_FAN_RPM 2500      // Cooldown fan speed
// HEATING: PID-controlled (800-3400 RPM range)
```

### Timing

```c
#define PREHEAT_PUMP_DELAY 30 * 1e6      // 30s wait before fuel pump starts
#define PREHEAT_GLOW_DURATION 5 * 60 * 1000  // 5min glow plug timeout
```

---

## Building & Development

### Build Commands (via Taskfile)

**IMPORTANT:** Always use the Taskfile commands for building. Never run `west build` directly.

```bash
task build-firmware    # Build firmware (ALWAYS use this)
task flash-firmware    # Flash to device (USER ONLY - see safety note below)
task rebuild-firmware  # Clean and rebuild in one command
task menuconfig        # Zephyr Kconfig menu
```

**SAFETY REQUIREMENT:** AI assistants must NEVER automatically execute `task flash-firmware`. See [Thermoquad Organization CLAUDE.md](../../CLAUDE.md) "Firmware Flashing Safety" section. After building firmware, always ask the user to manually flash it.

**Why use Taskfile:**
- Ensures consistent build environment
- Handles proper command sequencing
- Includes formatting and validation steps
- Used by CI/CD pipeline

### Manual Build (Not Recommended)

If you must build manually (not recommended):

```bash
west build -b rpi_pico2
west flash
```

### Serial Console

```bash
minicom -D /dev/ttyACM0 -b 115200
```

---

## Shell Commands

Interactive shell for testing and debugging (when connected via serial):

```
get_state              # Get current state
idle                   # Enter idle mode
fan <rpm>              # Set fan mode with RPM
heat <pump_rate>       # Start heating with pump rate
set_rpm <rpm>          # Set motor RPM directly
set_pump <rate_ms>     # Set pump rate directly
glow_burn <duration>   # Light glow plug for duration (ms)
fake_temp <temp>       # Inject fake temperature reading
```

---

## Serial Protocol

**Documentation:** `../../origin/docs/protocols/serial_protocol.md` (Fusain Protocol v2.0)

**Status:** ✅ Implemented

**Files:**
- `include/helios/communications/helios_serial.h` - Shared protocol library (header)
- `src/communications/helios_serial.c` - Shared protocol library (implementation)
- `include/helios/communications/serial_handler.h` - ICU-specific handler (header)
- `src/communications/serial_handler.c` - ICU-specific handler (implementation)

**Protocol:** Binary packet format with CRC-16-CCITT

**Transport:** UART1 (GPIO 4: TX, GPIO 5: RX) at 115200 baud

**Key Features:**
- CRC-16-CCITT error detection
- Byte stuffing for framing
- Variable-length telemetry bundles (supports 1-3 motors, 1-3 temperature sensors)
- Timeout mode (30s default) for safety - auto-transitions to IDLE on communication loss
- Master/slave architecture (controller is master, ICU is slave)

**Architecture:**
- **Shared Library:** Reusable encoding/decoding functions for both ICU and controller
- **ICU Handler:** UART integration, Zbus messaging, timeout mode
- **Threads:** RX and TX threads defined in `main.c`, implemented in `serial_handler.c`
  - `serial_rx_thread()`: Timeout checking (1s interval, priority 5)
  - `serial_tx_thread()`: Telemetry broadcasting (100ms interval, priority 6)

**Message Types:**
- Commands (master → ICU): SET_MODE, SET_PUMP_RATE, SET_TARGET_RPM, PING_REQUEST, etc.
- Data (ICU → master): TELEMETRY_BUNDLE, PING_RESPONSE, individual sensor data
- Errors (bidirectional): CRC errors, invalid commands, timeouts

**Timing:**
- 100ms telemetry broadcast rate (TX thread)
- 1s timeout check interval (RX thread)
- 10-15s recommended master ping interval

**Future:** Will use UART → LIN translation IC for production

---

## Safety Features

1. **Temperature Monitoring:**
   - Continuous monitoring with 60-sample averaging
   - Emergency stop at 275°C (25°C margin before hardware damage at 300°C)
   - Flame-out detection at <190°C during heating

2. **Cooldown Protection:**
   - Automatic cooldown when shutting down from >190°C
   - Glow plug assists at 180-120°C to prevent carbon buildup
   - Fan runs at 2500 RPM throughout cooldown
   - Only skipped during emergency stop

3. **Preheat Safety:**
   - 5-minute timeout via glow plug duration
   - Automatic error state if ignition fails
   - Prevents indefinite fuel pumping

4. **Error Handling:**
   - Safe cooldown even in error conditions
   - ERROR state persists after cooldown for user notification
   - Clear recovery path: refuel → idle command

5. **Hardware Protection:**
   - PTFE wires and seals rated to 300°C
   - 275°C emergency stop provides safety margin
   - Immediate shutdown in fault conditions

---

## Common Tasks for AI Assistants

### Adding a New Controller

1. Create controller file in `src/controllers/`
2. Define message structures in `include/helios/zbus.h`
3. Create command and data channels with validators
4. Add thread declaration in `include/helios/threads.h`
5. Register observers in channel definitions
6. Update state machine if needed

### Adding a New State

1. Add enum value to `helios_states` in `include/helios/zbus.h`
2. Add state name to `helios_state_names[]` in `src/state.c`
3. Create entry/run/exit functions in `src/state.c`
4. Add state to `helios_state_machine[]` array
5. Add transitions in relevant state functions
6. Update `docs/state_machine.md`

### Modifying PID Parameters

**Temperature PID:** `src/controllers/temperature.c`
```c
#define DEFAULT_TEMP_KP 100.0  // Proportional gain
#define DEFAULT_TEMP_KI 10.0   // Integral gain
#define DEFAULT_TEMP_KD 5.0    // Derivative gain
```

**Motor PID:** `src/controllers/motor.c`
```c
#define DEFAULT_KP 4.0   // Proportional gain
#define DEFAULT_KI 12.0  // Integral gain
#define DEFAULT_KD 0.1   // Derivative gain
```

### Adding New Hardware

1. Update device tree overlay in `boards/rpi_pico2_rp2350a_m33.overlay`
2. Add device alias in `app.overlay`
3. Update controller to use new hardware
4. Test with `task build-firmware`

---

## Testing Strategy

**Current Testing:** Manual testing via shell commands

**Recommended Test Sequence:**
1. Power on → verify INITIALIZING → IDLE transition
2. Test fan mode: `fan 2500` → verify RPM on target
3. Test heating sequence:
   - Command: `heat 250`
   - Verify: PREHEAT → PREHEAT_STAGE_2 → HEATING
   - Monitor: Temperature rises, PID engages
4. Test shutdown: `idle` → verify COOLING → IDLE
5. Test error recovery: Simulate fault → verify ERROR → COOLING → ERROR
6. Test emergency stop: Inject 275°C → verify E_STOP

**Future:** Add unit tests for PID controllers, state machine transitions

---

## Troubleshooting

### Build Errors

- **Missing CONFIG_***: Add to `prj.conf`
- **Device tree errors**: Check `app.overlay` and board overlay
- **Linking errors**: Verify thread definitions in `include/helios/threads.h`

### Runtime Issues

- **Temperature not reading**: Check thermometer device tree, verify sensor initialization
- **Motor not running**: Verify PWM configuration, check enabler GPIO
- **Pump not cycling**: Check solenoid continuity, verify pulse timing
- **State stuck**: Check validators, verify command message format
- **PID oscillation**: Tune gains, check sensor noise, verify time_divisor

### Common Mistakes

- Forgetting to update both command and data message structures
- Not adding observers to new channels
- Missing mutex locks when accessing shared state
- Incorrect byte order (must be little-endian)
- Not rate-limiting high-frequency logs

---

## Git Workflow

**IMPORTANT: Always get approval before committing changes!**

The developer manually tests firmware after flashing, so all changes must be reviewed before committing:

1. Show changes with `git diff` or `git diff --staged`
2. Explain what was modified and why
3. Show the proposed commit message
4. Wait for explicit approval
5. Only then run `git commit`

**Never commit without showing the changes first.**

---

**Commit Style:** Conventional Commits

Format:
```
<type>(<scope>): <subject>

<body>

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude Sonnet 4.5 <noreply@anthropic.com>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation only
- `style`: Code style/formatting
- `refactor`: Code refactoring
- `test`: Adding tests
- `chore`: Maintenance tasks

**Scopes:**
- `state`: State machine
- `temp`: Temperature controller
- `motor`: Motor controller
- `pump`: Pump controller
- `glow`: Glow plug controller
- `pid`: PID algorithm
- `serial`: Serial protocol
- `zbus`: Message bus

---

## Future Enhancements

**Planned:**
1. ✅ Complete state machine implementation (DONE)
2. ✅ Temperature PID control (DONE)
3. ✅ Automatic cooldown with carbon prevention (DONE)
4. ✅ Serial protocol implementation (DONE)
5. 🔲 Multi-motor support (2-3 motors) - protocol ready, hardware pending
6. 🔲 Multi-temperature sensor support - protocol ready, hardware pending
7. 🔲 Firmware update over serial
8. 🔲 Data logging and diagnostics
9. 🔲 Configurable PID gains via commands
10. 🔲 Advanced flame-out detection

**Considerations:**
- Watchdog timer for fault recovery
- Non-volatile storage for configuration
- CAN bus support as alternative to LIN
- Remote monitoring via WiFi/Bluetooth module

**Hardware Development:**
- 🔲 Custom RP2354A-based production board (awaiting burn test completion)
- 🔲 Board design optimized for burner integration
- 🔲 Production-ready connector layout for LIN/UART interface
- Current: Burn testing firmware on Raspberry Pi Pico 2 dev board

---

## References

- **Zephyr RTOS:** https://docs.zephyrproject.org/
- **State Machine Framework:** https://docs.zephyrproject.org/latest/services/smf/index.html
- **Zbus:** https://docs.zephyrproject.org/latest/services/zbus/index.html
- **RP2350 Datasheet:** https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf

---

## Project History

This project was developed with assistance from Claude (Anthropic). All significant implementations include:
- State machine with 9 states and automatic transitions
- Temperature-based PID control with inverted logic
- Automatic cooldown procedure with carbon prevention
- Complete serial protocol implementation with UART integration
- Timeout mode for safety (auto-idle on communication loss)
- Modular communication library for shared use between ICU and controller
- Safety features for emergency conditions

**Last Updated:** 2026-01-04

---

## Contact & Support

This is part of the Thermoquad project. For questions or issues, refer to the project repository.
