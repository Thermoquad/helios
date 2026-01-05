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

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(helios_serial_handler);

#define LOOP_SLEEP_US 500  // UART FIFO (32 bytes) fills in 2780us at 115200 baud
#define DEFAULT_TIMEOUT_INTERVAL_MS 30000
#define DEFAULT_TELEMETRY_INTERVAL_MS 100

//////////////////////////////////////////////////////////////
// State Struct Definition
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
static void poll_uart_rx(void);
static void poll_uart_tx(void);
static void process_rx_packets(struct serial_state* state, uint64_t current_micros);
static void process_tx_queue(void);
static void check_timeout(struct serial_state* state, uint64_t current_micros);
static void send_telemetry_bundle(struct serial_state* state, uint64_t current_micros);

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

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

/**
 * Send Ping Response
 */
void serial_send_ping_response(void)
{
  helios_packet_t packet;
  uint64_t uptime = k_uptime_get();

  helios_create_ping_response(&packet, uptime);
  LOG_DBG_RATELIMIT("Sending ping response (uptime=%llu ms)", uptime);
  send_packet(&packet);
}

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

//////////////////////////////////////////////////////////////
// Thread Functions
//////////////////////////////////////////////////////////////

/**
 * Serial RX Thread - High-priority UART reception only
 *
 * Dedicated thread for receiving bytes and decoding packets.
 * Runs at highest priority with fast polling to prevent FIFO overflow.
 * Only polls UART and queues packets - does not process them.
 */
int serial_rx_thread(void)
{
  LOG_DBG("Serial RX thread started");

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
    // Poll UART for incoming data and decode (time-critical)
    poll_uart_rx();

    k_sleep(K_USEC(LOOP_SLEEP_US));
  }

  return 0;
}

/**
 * Serial TX Thread - UART transmission only
 *
 * Handles UART TX polling and TX queue processing.
 * Pops packets from queue and fills TX buffer, then polls UART to send bytes.
 */
int serial_tx_thread(void)
{
  LOG_DBG("Serial TX thread started");

  // Wait for RX thread to initialize serial
  k_sleep(K_MSEC(100));

  while (1) {
    // Poll UART for TX readiness and send queued data
    poll_uart_tx();

    // Process one pending TX packet if available (pops from queue, fills buffer)
    process_tx_queue();

    k_sleep(K_MSEC(1));  // TX thread can run slower (1ms)
  }

  return 0;
}

/**
 * Serial Processing Thread - Protocol logic and packet processing
 *
 * Handles all higher-level protocol logic:
 * - Processing received packets from RX queue
 * - Timeout checking and auto-IDLE transition
 * - Periodic telemetry transmission
 *
 * Runs at moderate priority between RX and TX.
 */
int serial_processing_thread(void)
{
  LOG_DBG("Serial processing thread started");

  // Wait for RX thread to initialize serial and state
  k_sleep(K_MSEC(200));

  while (1) {
    const uint64_t current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

    // Process all pending RX packets (pops from queue, processes commands)
    process_rx_packets(&serial_state, current_micros);

    // Check timeout mode (30s default - auto-transition to IDLE)
    check_timeout(&serial_state, current_micros);

    // Send telemetry bundle if enabled (periodic broadcast)
    send_telemetry_bundle(&serial_state, current_micros);

    k_sleep(K_MSEC(10));  // Processing thread runs at 10ms
  }

  return 0;
}

//////////////////////////////////////////////////////////////
// Init Functions
//////////////////////////////////////////////////////////////

/**
 * Initialize Serial Handler
 */
static int serial_handler_init(void)
{
  // Get UART device
  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
    return -1;
  }

  LOG_DBG("UART device ready: %s", uart_dev->name);

  // Initialize decoder
  helios_reset_decoder(&decoder);
  LOG_DBG("Decoder initialized");

  LOG_INF("Serial handler initialized (polling mode) on %s", uart_dev->name);

  return 0;
}

//////////////////////////////////////////////////////////////
// Hardware Functions
//////////////////////////////////////////////////////////////

/**
 * Poll UART RX - Reads available bytes and decodes packets
 *
 * Processes up to 32 bytes per call to keep iterations short.
 * Queues complete packets for processing in thread context.
 */
static void poll_uart_rx(void)
{
  uint8_t byte;
  int max_bytes = 32; // Limit bytes per poll

  while (max_bytes-- > 0 && uart_poll_in(uart_dev, &byte) == 0) {
    helios_packet_t packet;

    // Save state BEFORE decoding for diagnostics
    uint8_t prev_state = decoder.state;
    size_t prev_index = decoder.buffer_index;

    helios_decode_result_t result = helios_decode_byte(byte, &packet, &decoder);

    if (result == HELIOS_DECODE_OK) {
      // Packet complete - queue for processing
      int ret = k_msgq_put(&rx_packet_queue, &packet, K_NO_WAIT);
      if (ret != 0) {
        LOG_ERR("RX queue full, dropping packet type 0x%02X", packet.msg_type);
      }
    } else if (result != HELIOS_DECODE_INCOMPLETE) {
      // Decode error - reset decoder and continue
      LOG_ERR("DECODE ERROR: result=%d, last_byte=0x%02X",
              result, byte);
      LOG_ERR("  State: prev=%u → curr=%u, Index: prev=%zu → curr=%zu",
              prev_state, decoder.state, prev_index, decoder.buffer_index);
      LOG_ERR("  escape_next=%d", decoder.escape_next);

      // Log last few bytes in decoder buffer
      if (decoder.buffer_index > 0) {
        size_t log_start = (decoder.buffer_index > 16) ? decoder.buffer_index - 16 : 0;
        LOG_ERR("  Last bytes in buffer (from [%zu]):", log_start);
        for (size_t i = log_start; i < decoder.buffer_index && i < log_start + 16; i += 8) {
          size_t bytes_left = (decoder.buffer_index - i < 8) ? decoder.buffer_index - i : 8;
          if (bytes_left >= 8) {
            LOG_ERR("    [%02zu]: %02X %02X %02X %02X %02X %02X %02X %02X", i,
                    decoder.buffer[i+0], decoder.buffer[i+1], decoder.buffer[i+2], decoder.buffer[i+3],
                    decoder.buffer[i+4], decoder.buffer[i+5], decoder.buffer[i+6], decoder.buffer[i+7]);
          } else {
            LOG_ERR("    [%02zu]: partial (%zu bytes)", i, bytes_left);
          }
        }
      }

      helios_reset_decoder(&decoder);
    }
  }
}

/**
 * Poll UART TX - Sends buffered data as fast as UART can accept
 *
 * Fills UART FIFO completely each poll to maximize throughput.
 */
static void poll_uart_tx(void)
{
  if (tx_index < tx_length) {
    // Send as many bytes as possible while UART is ready
    while (tx_index < tx_length) {
      uart_poll_out(uart_dev, tx_buffer[tx_index]);
      tx_index++;
    }

    // Transmission complete
    if (tx_index >= tx_length) {
      LOG_DBG("TX complete: %zu bytes sent", tx_length);
    }
  }
}

//////////////////////////////////////////////////////////////
// Helper Functions - Packet Processing
//////////////////////////////////////////////////////////////

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
  case HELIOS_MSG_STATE_COMMAND: {
    if (packet->length != sizeof(helios_cmd_set_mode_t)) {
      LOG_WRN("Invalid STATE_COMMAND length");
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

  case HELIOS_MSG_PUMP_COMMAND: {
    if (packet->length != sizeof(helios_cmd_set_pump_rate_t)) {
      LOG_WRN("Invalid PUMP_COMMAND length");
      return;
    }

    helios_cmd_set_pump_rate_t* cmd = (helios_cmd_set_pump_rate_t*)packet->payload;

    struct pump_command_msg pump_cmd = { .pump = 0,
      .rate_ms = (int)cmd->rate_ms };

    LOG_INF("SET_PUMP_RATE: %d ms", pump_cmd.rate_ms);
    zbus_chan_pub(&pump_command_chan, &pump_cmd, K_NO_WAIT);
    break;
  }

  case HELIOS_MSG_MOTOR_COMMAND: {
    if (packet->length != sizeof(helios_cmd_set_target_rpm_t)) {
      LOG_WRN("Invalid MOTOR_COMMAND length");
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

  case HELIOS_MSG_GLOW_COMMAND: {
    if (packet->length != sizeof(helios_cmd_glow_t)) {
      LOG_WRN("Invalid GLOW_COMMAND length");
      return;
    }

    helios_cmd_glow_t* cmd = (helios_cmd_glow_t*)packet->payload;

    // Validate duration (0-300000 ms)
    if (cmd->duration < 0 || cmd->duration > 300000) {
      LOG_WRN("Invalid glow duration: %d ms", cmd->duration);
      return;
    }

    // Send to glow controller via zbus
    struct glow_command_msg glow_cmd = {
      .glow = cmd->glow,
      .duration = cmd->duration
    };
    zbus_chan_pub(&glow_command_chan, &glow_cmd, K_NO_WAIT);
    LOG_DBG("Glow command: glow=%d, duration=%d ms", cmd->glow, cmd->duration);
    break;
  }

  case HELIOS_MSG_PING_REQUEST: {
    LOG_DBG("Ping request received");
    state->last_ping_time = current_micros; // Update timeout
    serial_send_ping_response();
    break;
  }

  case HELIOS_MSG_TELEMETRY_CONFIG: {
    if (packet->length != sizeof(helios_cmd_telemetry_config_t)) {
      LOG_WRN("Invalid TELEMETRY_CONFIG length: got %d, expected %d",
          packet->length, sizeof(helios_cmd_telemetry_config_t));
      return;
    }

    helios_cmd_telemetry_config_t* cmd = (helios_cmd_telemetry_config_t*)packet->payload;

    LOG_DBG("TELEMETRY_CONFIG received: raw enabled=%u, interval=%u, mode=%u",
        cmd->telemetry_enabled, cmd->interval_ms, cmd->telemetry_mode);

    bool was_enabled = state->telemetry_enabled;
    state->telemetry_enabled = (cmd->telemetry_enabled != 0);
    state->telemetry_interval_ms = cmd->interval_ms;
    state->telemetry_mode = cmd->telemetry_mode;

    // Clamp interval to valid range (100-5000 ms)
    if (state->telemetry_interval_ms < 100) {
      state->telemetry_interval_ms = 100;
    } else if (state->telemetry_interval_ms > 5000) {
      state->telemetry_interval_ms = 5000;
    }

    // Log INF only when telemetry is first enabled
    if (state->telemetry_enabled && !was_enabled) {
      LOG_INF("Telemetry enabled: interval=%u ms, mode=%u",
          state->telemetry_interval_ms, state->telemetry_mode);
    } else {
      LOG_DBG("Telemetry config applied: enabled=%d, interval=%u ms, mode=%u",
          state->telemetry_enabled, state->telemetry_interval_ms, state->telemetry_mode);
    }
    break;
  }

  default:
    LOG_WRN("Unknown message type: 0x%02X", packet->msg_type);
    break;
  }
}

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
 * Only dequeues if no transmission is currently in progress.
 */
static void process_tx_queue(void)
{
  // Don't dequeue if transmission in progress
  if (tx_index < tx_length) {
    return;
  }

  helios_packet_t tx_packet;
  if (k_msgq_get(&tx_packet_queue, &tx_packet, K_NO_WAIT) == 0) {
    fill_tx_buffer(&tx_packet);
  }
}

/**
 * Fill TX Buffer - Encodes packet and prepares for transmission
 *
 * Called from TX thread to prepare packet for UART transmission.
 */
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

  // TX will be sent via polling in main loop

  k_mutex_unlock(&tx_mutex);
}

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

//////////////////////////////////////////////////////////////
// Helper Functions - Protocol Handlers
//////////////////////////////////////////////////////////////

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
