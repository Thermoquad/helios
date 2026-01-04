/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Helios Serial Handler - ICU Implementation
 *
 * Handles UART communication with master controller, processes commands,
 * sends telemetry data, and implements timeout mode for safety.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <fusain/fusain.h>
#include <helios/communications/serial_handler.h>
#include <helios/zbus.h>

LOG_MODULE_REGISTER(helios_serial_handler);

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

#define LOOP_SLEEP_MS 1
#define DEFAULT_TIMEOUT_INTERVAL_MS 30000
#define DEFAULT_TELEMETRY_INTERVAL_MS 100

//////////////////////////////////////////////////////////////
// Serial State
//////////////////////////////////////////////////////////////

struct serial_state {
  // Timeout tracking
  bool timeout_enabled;
  uint32_t timeout_interval_ms;
  uint64_t last_ping_time;

  // Telemetry configuration
  bool telemetry_enabled;
  uint32_t telemetry_interval_ms;
  uint32_t telemetry_mode; // 0 = bundled, 1 = individual
  uint64_t last_telemetry_time;
};

//////////////////////////////////////////////////////////////
// Static Variables
//////////////////////////////////////////////////////////////

/* UART Device */
static const struct device* uart_dev = DEVICE_DT_GET(DT_ALIAS(helios_uart));

/* Decoder State */
static helios_decoder_t decoder;

/* TX Buffer and State */
static uint8_t tx_buffer[HELIOS_MAX_PACKET_SIZE * 2]; // 2x for stuffing overhead
static size_t tx_index = 0;
static size_t tx_length = 0;
K_MUTEX_DEFINE(tx_mutex); // Protects TX buffer and state

/* TX Packet Queue - API pushes, thread pops and transmits */
K_MSGQ_DEFINE(tx_packet_queue, sizeof(helios_packet_t), 8, 4);

/* RX Packet Queue - ISR pushes, thread pops */
K_MSGQ_DEFINE(rx_packet_queue, sizeof(helios_packet_t), 8, 4);

/* Serial State */
static struct serial_state serial_state;

//////////////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////////////

static int serial_handler_init(void);
static void process_packet(const helios_packet_t* packet, struct serial_state* state,
    uint64_t current_micros);
static void send_packet(const helios_packet_t* packet);
static void fill_tx_buffer(const helios_packet_t* packet);
static void uart_isr(const struct device* dev, void* user_data);
static void process_rx_packets(struct serial_state* state, uint64_t current_micros);
static void process_tx_queue(void);
static void check_timeout(struct serial_state* state, uint64_t current_micros);
static void send_telemetry_bundle(struct serial_state* state, uint64_t current_micros);

//////////////////////////////////////////////////////////////
// Serial Thread
//////////////////////////////////////////////////////////////

/**
 * Serial Thread - Handles both TX and RX operations
 *
 * Main thread for serial communication. Processes received commands,
 * transmits responses and telemetry, and handles timeout mode.
 */
int serial_thread(void)
{
  LOG_DBG("Serial thread started");

  // Initialize serial handler
  int ret = serial_handler_init();
  if (ret < 0) {
    LOG_ERR("Failed to initialize serial handler: %d", ret);
    return ret;
  }

  // Initialize state
  serial_state.timeout_enabled = true;
  serial_state.timeout_interval_ms = DEFAULT_TIMEOUT_INTERVAL_MS;
  serial_state.last_ping_time = 0;
  serial_state.telemetry_enabled = false;
  serial_state.telemetry_interval_ms = DEFAULT_TELEMETRY_INTERVAL_MS;
  serial_state.telemetry_mode = 0;
  serial_state.last_telemetry_time = 0;

  while (1) {
    const uint64_t current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

    // Process all pending RX packets
    process_rx_packets(&serial_state, current_micros);

    // Check timeout mode
    check_timeout(&serial_state, current_micros);

    // Send telemetry bundle if enabled
    send_telemetry_bundle(&serial_state, current_micros);

    // Process one pending TX packet if available
    process_tx_queue();

    k_sleep(K_MSEC(LOOP_SLEEP_MS));
  }

  return 0;
}

//////////////////////////////////////////////////////////////
// UART Interrupt Service Routine
//////////////////////////////////////////////////////////////

/**
 * UART ISR - Handles both RX and TX interrupts
 *
 * RX: Decodes incoming bytes and queues complete packets
 * TX: Fills UART FIFO from buffer until transmission complete
 */
static void uart_isr(const struct device* dev, void* user_data)
{
  ARG_UNUSED(user_data);

  uart_irq_update(dev);

  // Handle RX
  if (uart_irq_rx_ready(dev)) {
    uint8_t byte;
    while (uart_fifo_read(dev, &byte, 1) == 1) {
      helios_packet_t packet;
      helios_decode_result_t result = helios_decode_byte(byte, &packet, &decoder);

      if (result == HELIOS_DECODE_OK) {
        // Packet complete - queue for processing in thread context
        int ret = k_msgq_put(&rx_packet_queue, &packet, K_NO_WAIT);
        if (ret != 0) {
          // Queue full - drop packet and log error
          LOG_ERR("RX queue full, dropping packet type 0x%02X", packet.msg_type);
        }
      } else if (result != HELIOS_DECODE_INCOMPLETE) {
        // Decode error - reset decoder and continue
        LOG_WRN("Decode error: %d", result);
        helios_reset_decoder(&decoder);
      }
    }
  }

  // Handle TX
  if (uart_irq_tx_ready(dev)) {
    if (tx_index < tx_length) {
      // Fill FIFO with as much data as possible
      size_t remaining = tx_length - tx_index;
      size_t sent = uart_fifo_fill(dev, &tx_buffer[tx_index], remaining);
      tx_index += sent;

      // Check if transmission complete
      if (tx_index >= tx_length) {
        uart_irq_tx_disable(dev);
        LOG_DBG("TX complete: %zu bytes sent", tx_length);
      }
    } else {
      // No data to send, disable TX interrupt
      uart_irq_tx_disable(dev);
    }
  }
}

//////////////////////////////////////////////////////////////
// Packet Processing
//////////////////////////////////////////////////////////////

/* Initialize Serial Handler */
int serial_handler_init(void)
{
  // Get UART device
  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
    return -1;
  }

  LOG_DBG("UART device ready: %s", uart_dev->name);

  // Initialize decoder BEFORE enabling interrupts
  helios_reset_decoder(&decoder);
  LOG_DBG("Decoder initialized");

  // Register ISR for both RX and TX
  uart_irq_callback_set(uart_dev, uart_isr);
  LOG_DBG("UART ISR registered");

  // Enable RX interrupt (after decoder initialized and FIFO flushed)
  uart_irq_rx_enable(uart_dev);
  LOG_DBG("UART RX interrupt enabled");

  // TX interrupt will be enabled on-demand during transmission
  LOG_INF("Serial handler initialized (interrupt-driven) on %s", uart_dev->name);

  return 0;
}

/**
 * Process Received Packet
 *
 * Called from serial thread to process packets queued by UART ISR.
 * Handles commands from master controller and updates state.
 */
static void process_packet(const helios_packet_t* packet, struct serial_state* state,
    uint64_t current_micros)
{
  LOG_DBG("RX packet type 0x%02X, length %d", packet->msg_type,
      packet->length);

  switch (packet->msg_type) {
  case HELIOS_MSG_SET_MODE: {
    if (packet->length != sizeof(helios_cmd_set_mode_t)) {
      LOG_WRN("Invalid SET_MODE length");
      return;
    }

    helios_cmd_set_mode_t* cmd = (helios_cmd_set_mode_t*)packet->payload;

    struct state_command_msg state_cmd;
    switch (cmd->mode) {
    case HELIOS_MODE_IDLE:
      state_cmd.mode = HELIOS_MODE_IDLE;
      state_cmd.argument = 0;
      break;
    case HELIOS_MODE_FAN:
      state_cmd.mode = HELIOS_MODE_FAN;
      state_cmd.argument = (int)cmd->parameter; // RPM
      break;
    case HELIOS_MODE_HEAT:
      state_cmd.mode = HELIOS_MODE_HEAT;
      state_cmd.argument = 0;
      break;
    case HELIOS_MODE_EMERGENCY:
      state_cmd.mode = HELIOS_MODE_EMERGENCY;
      state_cmd.argument = 0;
      break;
    default:
      LOG_WRN("Invalid mode: 0x%02X", cmd->mode);
      return;
    }

    LOG_INF("SET_MODE: mode=%d, arg=%d", state_cmd.mode,
        state_cmd.argument);
    zbus_chan_pub(&state_command_chan, &state_cmd, K_NO_WAIT);
    break;
  }

  case HELIOS_MSG_SET_PUMP_RATE: {
    if (packet->length != sizeof(helios_cmd_set_pump_rate_t)) {
      LOG_WRN("Invalid SET_PUMP_RATE length");
      return;
    }

    helios_cmd_set_pump_rate_t* cmd = (helios_cmd_set_pump_rate_t*)packet->payload;

    struct pump_command_msg pump_cmd = { .pump = 0,
      .rate_ms = (int)cmd->rate_ms };

    LOG_INF("SET_PUMP_RATE: %d ms", pump_cmd.rate_ms);
    zbus_chan_pub(&pump_command_chan, &pump_cmd, K_NO_WAIT);
    break;
  }

  case HELIOS_MSG_SET_TARGET_RPM: {
    if (packet->length != sizeof(helios_cmd_set_target_rpm_t)) {
      LOG_WRN("Invalid SET_TARGET_RPM length");
      return;
    }

    helios_cmd_set_target_rpm_t* cmd = (helios_cmd_set_target_rpm_t*)packet->payload;

    struct motor_command_msg motor_cmd = {
      .motor = 0, .rpm = (int)cmd->target_rpm
    };

    LOG_INF("SET_TARGET_RPM: %d", motor_cmd.rpm);
    zbus_chan_pub(&motor_command_chan, &motor_cmd, K_NO_WAIT);
    break;
  }

  case HELIOS_MSG_PING_REQUEST: {
    LOG_DBG("Ping request received");
    state->last_ping_time = current_micros; // Update timeout
    // Queue ping response for transmission
    serial_send_ping_response();
    break;
  }

  case HELIOS_MSG_SET_TIMEOUT_CONFIG: {
    if (packet->length != sizeof(helios_cmd_set_timeout_config_t)) {
      LOG_WRN("Invalid SET_TIMEOUT_CONFIG length");
      return;
    }

    helios_cmd_set_timeout_config_t* cmd = (helios_cmd_set_timeout_config_t*)packet->payload;

    state->timeout_enabled = (cmd->timeout_enabled != 0);
    state->timeout_interval_ms = cmd->timeout_ms;

    LOG_INF("Timeout config: enabled=%d, interval=%u ms",
        state->timeout_enabled, state->timeout_interval_ms);

    // Reset timeout timer
    state->last_ping_time = current_micros;
    break;
  }

  case HELIOS_MSG_EMERGENCY_STOP: {
    LOG_WRN("EMERGENCY_STOP received");

    struct state_command_msg state_cmd = { .mode = HELIOS_MODE_EMERGENCY,
      .argument = 0 };
    zbus_chan_pub(&state_command_chan, &state_cmd, K_NO_WAIT);
    break;
  }

  case HELIOS_MSG_TELEMETRY_CONFIG: {
    if (packet->length != sizeof(helios_cmd_telemetry_config_t)) {
      LOG_WRN("Invalid TELEMETRY_CONFIG length: got %d, expected %d",
          packet->length, sizeof(helios_cmd_telemetry_config_t));
      return;
    }

    helios_cmd_telemetry_config_t* cmd = (helios_cmd_telemetry_config_t*)packet->payload;

    LOG_INF("TELEMETRY_CONFIG received: raw enabled=%u, interval=%u, mode=%u",
        cmd->telemetry_enabled, cmd->interval_ms, cmd->telemetry_mode);

    state->telemetry_enabled = (cmd->telemetry_enabled != 0);
    state->telemetry_interval_ms = cmd->interval_ms;
    state->telemetry_mode = cmd->telemetry_mode;

    // Clamp interval to valid range (100-5000 ms)
    if (state->telemetry_interval_ms < 100) {
      state->telemetry_interval_ms = 100;
    } else if (state->telemetry_interval_ms > 5000) {
      state->telemetry_interval_ms = 5000;
    }

    LOG_INF("Telemetry config applied: enabled=%d, interval=%u ms, mode=%u",
        state->telemetry_enabled, state->telemetry_interval_ms, state->telemetry_mode);
    break;
  }

  default:
    LOG_WRN("Unknown message type: 0x%02X", packet->msg_type);
    break;
  }
}

//////////////////////////////////////////////////////////////
// Thread Helper Functions
//////////////////////////////////////////////////////////////

/**
 * Process RX Packets
 *
 * Drains RX packet queue and processes all pending packets.
 */
static void process_rx_packets(struct serial_state* state, uint64_t current_micros)
{
  helios_packet_t rx_packet;
  while (k_msgq_get(&rx_packet_queue, &rx_packet, K_NO_WAIT) == 0) {
    process_packet(&rx_packet, state, current_micros);
  }
}

/**
 * Process TX Queue
 *
 * Processes one packet from TX queue if available.
 */
static void process_tx_queue(void)
{
  helios_packet_t tx_packet;
  if (k_msgq_get(&tx_packet_queue, &tx_packet, K_NO_WAIT) == 0) {
    fill_tx_buffer(&tx_packet);
  }
}

/**
 * Check Timeout Mode
 *
 * Monitors communication timeout and transitions to IDLE if needed.
 */
static void check_timeout(struct serial_state* state, uint64_t current_micros)
{
  if (!state->timeout_enabled) {
    return;
  }

  // On first call (last_time == 0), initialize timer
  if (state->last_ping_time == 0) {
    state->last_ping_time = current_micros;
    return;
  }

  const uint64_t micros_since_ping = current_micros - state->last_ping_time;
  if (micros_since_ping < (state->timeout_interval_ms * 1000)) {
    return;
  }

  // Disable telemetry on timeout (protocol v1.2)
  if (state->telemetry_enabled) {
    LOG_WRN("Communication timeout - disabling telemetry");
    state->telemetry_enabled = false;
  }

  // Check current state before transitioning
  struct state_data_msg state_data;
  if (zbus_chan_read(&state_data_chan, &state_data, K_NO_WAIT) == 0) {
    // Only transition if not already in IDLE
    if (state_data.state != HELIOS_STATE_IDLE) {
      LOG_WRN("Communication timeout - transitioning to IDLE");

      struct state_command_msg cmd = { .mode = HELIOS_MODE_IDLE,
        .argument = 0 };
      zbus_chan_pub(&state_command_chan, &cmd, K_NO_WAIT);
    }
  }

  // Reset timeout
  state->last_ping_time = current_micros;
}

/**
 * Send Telemetry Bundle
 *
 * Sends periodic telemetry bundle if enabled.
 */
static void send_telemetry_bundle(struct serial_state* state, uint64_t current_micros)
{
  if (!state->telemetry_enabled) {
    return;
  }

  // On first call (last_time == 0), send immediately
  if (state->last_telemetry_time == 0) {
    serial_send_telemetry_bundle();
    state->last_telemetry_time = current_micros;
    return;
  }

  const uint64_t micros_since_telemetry = current_micros - state->last_telemetry_time;
  if (micros_since_telemetry < (state->telemetry_interval_ms * 1000)) {
    return;
  }

  serial_send_telemetry_bundle();
  state->last_telemetry_time = current_micros;
}

//////////////////////////////////////////////////////////////
// Packet Transmission
//////////////////////////////////////////////////////////////

/**
 * Queue Packet for Transmission
 *
 * Queues packet for transmission by serial thread.
 */
static void send_packet(const helios_packet_t* packet)
{
  int ret = k_msgq_put(&tx_packet_queue, packet, K_NO_WAIT);
  if (ret != 0) {
    LOG_ERR("TX queue full, dropping packet type 0x%02X", packet->msg_type);
  }
}

/* Transmit Packet - Actual UART transmission (called from TX thread) */
static void fill_tx_buffer(const helios_packet_t* packet)
{
  // Lock to prevent concurrent transmission attempts
  k_mutex_lock(&tx_mutex, K_FOREVER);

  // Encode packet to buffer
  int encoded_len = helios_encode_packet(packet, tx_buffer, sizeof(tx_buffer));

  if (encoded_len < 0) {
    LOG_ERR("Failed to encode packet: %d", encoded_len);
    k_mutex_unlock(&tx_mutex);
    return;
  }

  // Prepare TX state
  tx_index = 0;
  tx_length = (size_t)encoded_len;

  LOG_DBG("Starting TX: type=0x%02X, %zu bytes", packet->msg_type, tx_length);

  // Enable TX interrupt - this will trigger ISR to start sending
  uart_irq_tx_enable(uart_dev);

  k_mutex_unlock(&tx_mutex);
}

/**
 * Send Telemetry Bundle
 *
 * Called by send_telemetry_bundle() helper to actually send the bundle.
 * Public API for external callers (though primarily used internally).
 */
void serial_send_telemetry_bundle(void)
{
  // Check if telemetry is enabled (protocol v1.2)
  if (!serial_state.telemetry_enabled) {
    LOG_DBG_RATELIMIT("Telemetry disabled, not sending bundle");
    return;
  }

  LOG_DBG_RATELIMIT("Sending telemetry bundle (enabled=%d)", serial_state.telemetry_enabled);

  // Read current state from Zbus
  struct state_data_msg state_data;
  if (zbus_chan_read(&state_data_chan, &state_data, K_NO_WAIT) != 0) {
    return; // Channel not ready
  }

  struct motor_data_msg motor_data;
  if (zbus_chan_read(&motor_data_chan, &motor_data, K_NO_WAIT) != 0) {
    return; // Channel not ready
  }

  struct temperature_data_msg temp_data;
  if (zbus_chan_read(&temperature_data_chan, &temp_data, K_NO_WAIT) != 0) {
    return; // Channel not ready
  }

  // Build telemetry data
  helios_telemetry_motor_t motor = { .rpm = motor_data.rpm,
    .target_rpm = motor_data.target,
    .pwm_duty = motor_data.pwm,
    .pwm_period = motor_data.pwm_max };

  helios_telemetry_temperature_t temperature = {
    .temperature = temp_data.temperature
  };

  // Map state machine state to serial protocol state
  helios_state_t serial_state;
  switch (state_data.state) {
  case HELIOS_STATE_INITIALIZING:
    serial_state = HELIOS_STATE_INITIALIZING;
    break;
  case HELIOS_STATE_IDLE:
    serial_state = HELIOS_STATE_IDLE;
    break;
  case HELIOS_STATE_BLOWING:
    serial_state = HELIOS_STATE_BLOWING;
    break;
  case HELIOS_STATE_PREHEAT:
    serial_state = HELIOS_STATE_PREHEAT;
    break;
  case HELIOS_STATE_PREHEAT_STAGE_2:
    serial_state = HELIOS_STATE_PREHEAT_STAGE_2;
    break;
  case HELIOS_STATE_HEATING:
    serial_state = HELIOS_STATE_HEATING;
    break;
  case HELIOS_STATE_COOLING:
    serial_state = HELIOS_STATE_COOLING;
    break;
  case HELIOS_STATE_ERROR:
    serial_state = HELIOS_STATE_ERROR;
    break;
  case HELIOS_STATE_E_STOP:
    serial_state = HELIOS_STATE_E_STOP;
    break;
  default:
    serial_state = HELIOS_STATE_ERROR;
    break;
  }

  // Map error code
  helios_error_t error = state_data.error ? (helios_error_t)state_data.code : HELIOS_ERROR_NONE;

  // Create and send packet
  helios_packet_t packet;
  if (helios_create_telemetry_bundle(&packet, serial_state, error,
          &motor, 1, &temperature, 1)
      == 0) {
    send_packet(&packet);
  }
}

/* Send Ping Response */
void serial_send_ping_response(void)
{
  helios_packet_t packet;
  uint64_t uptime = k_uptime_get();

  helios_create_ping_response(&packet, uptime);
  LOG_DBG_RATELIMIT("Sending ping response (uptime=%llu ms)", uptime);
  send_packet(&packet);
}

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

/**
 * Get Timeout Configuration
 *
 * Returns current timeout mode configuration.
 */
void serial_get_timeout_config(bool* enabled, uint32_t* timeout_ms)
{
  if (enabled) {
    *enabled = serial_state.timeout_enabled;
  }
  if (timeout_ms) {
    *timeout_ms = serial_state.timeout_interval_ms;
  }
}

/**
 * Set Timeout Enabled
 *
 * Enable or disable timeout mode and reset timer.
 */
void serial_set_timeout_enabled(bool enabled)
{
  serial_state.timeout_enabled = enabled;
  // Reset timeout timer when changing state
  serial_state.last_ping_time = k_cyc_to_us_floor64(k_cycle_get_64());
}
