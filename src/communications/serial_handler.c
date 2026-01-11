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
#include <fusain_cbor_decode.h>
#include <fusain_cbor_types.h>
#include <helios/communications/serial_handler.h>
#include <helios/zbus.h>

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(helios_serial_handler);

#define LOOP_SLEEP_US 500 // UART FIFO (32 bytes) fills in 2780us at 115200 baud
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
  uint64_t last_telemetry_time;
};

//////////////////////////////////////////////////////////////
// Static Variables
//////////////////////////////////////////////////////////////

/* UART Device */
static const struct device* uart_dev = DEVICE_DT_GET(DT_ALIAS(helios_uart));

/* Decoder State */
static fusain_decoder_t decoder;

/* TX Buffer and State */
static uint8_t tx_buffer[FUSAIN_MAX_PACKET_SIZE * 2]; // 2x for stuffing overhead
static size_t tx_index = 0;
static size_t tx_length = 0;

/* TX Packet Queue - API pushes, thread pops */
K_MSGQ_DEFINE(tx_packet_queue, sizeof(fusain_packet_t), 8, 4);

/* RX Packet Queue - ISR pushes, thread pops */
K_MSGQ_DEFINE(rx_packet_queue, sizeof(fusain_packet_t), 8, 4);

/* Serial State */
static struct serial_state serial_state;

//////////////////////////////////////////////////////////////
// CBOR Helper
//////////////////////////////////////////////////////////////

/**
 * Get CBOR message header length based on msg_type
 *
 * CBOR wire format: [0x82, msg_type, payload_map]
 * - msg_type 0x00-0x17: header is [0x82, type] = 2 bytes
 * - msg_type 0x18-0xFF: header is [0x82, 0x18, type] = 3 bytes
 */
static inline size_t cbor_header_len(uint8_t msg_type)
{
  return (msg_type <= 0x17) ? 2 : 3;
}

//////////////////////////////////////////////////////////////
// Forward Declarations
//////////////////////////////////////////////////////////////

static int serial_handler_init(void);
static void process_packet(const fusain_packet_t* packet, struct serial_state* state,
    uint64_t current_micros);
static void send_packet(const fusain_packet_t* packet);
static void fill_tx_buffer(const fusain_packet_t* packet);
static void poll_uart_rx(void);
static void poll_uart_tx(void);
static void process_rx_packets(struct serial_state* state, uint64_t current_micros);
static void process_tx_queue(void);
static void check_timeout(struct serial_state* state, uint64_t current_micros);
static void send_telemetry_periodic(struct serial_state* state, uint64_t current_micros);

//////////////////////////////////////////////////////////////
// Public API
//////////////////////////////////////////////////////////////

/**
 * Send Telemetry Messages
 *
 * Sends individual telemetry messages: STATE_DATA, MOTOR_DATA, TEMP_DATA.
 * Called periodically when telemetry is enabled.
 */
void serial_send_telemetry(void)
{
  // Check if telemetry is enabled
  if (!serial_state.telemetry_enabled) {
    LOG_DBG_RATELIMIT("Telemetry disabled, not sending");
    return;
  }

  LOG_DBG_RATELIMIT("Sending telemetry (enabled=%d)", serial_state.telemetry_enabled);

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

  // Map error code
  fusain_error_t error = state_data.error ? (fusain_error_t)state_data.code : FUSAIN_ERROR_NONE;

  // Send STATE_DATA
  fusain_packet_t packet;
  fusain_create_state_data(&packet, 0, state_data.error ? 1 : 0, error,
      state_data.state, state_data.timestamp);
  send_packet(&packet);

  // Send MOTOR_DATA
  fusain_create_motor_data(&packet, 0, motor_data.motor, motor_data.timestamp,
      motor_data.rpm, motor_data.target);
  send_packet(&packet);

  // Send TEMP_DATA
  fusain_create_temp_data(&packet, 0, temp_data.thermometer, temp_data.timestamp,
      temp_data.temperature);
  send_packet(&packet);
}

/**
 * Send Ping Response
 */
void serial_send_ping_response(void)
{
  fusain_packet_t packet;
  uint32_t uptime = (uint32_t)k_uptime_get();

  fusain_create_ping_response(&packet, 0, uptime);
  LOG_DBG_RATELIMIT("Sending ping response (uptime=%u ms)", uptime);
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

    k_sleep(K_MSEC(1)); // TX thread can run slower (1ms)
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

    // Send telemetry if enabled (periodic broadcast)
    send_telemetry_periodic(&serial_state, current_micros);

    k_sleep(K_MSEC(10)); // Processing thread runs at 10ms
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
  fusain_reset_decoder(&decoder);
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
    fusain_packet_t packet;

    // Save state BEFORE decoding for diagnostics
    uint8_t prev_state = decoder.state;
    size_t prev_index = decoder.buffer_index;

    fusain_decode_result_t result = fusain_decode_byte(byte, &packet, &decoder);

    if (result == FUSAIN_DECODE_OK) {
      // Packet complete - queue for processing
      int ret = k_msgq_put(&rx_packet_queue, &packet, K_NO_WAIT);
      if (ret != 0) {
        LOG_ERR("RX queue full, dropping packet type 0x%02X", packet.msg_type);
      }
    } else if (result != FUSAIN_DECODE_INCOMPLETE) {
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
                decoder.buffer[i + 0], decoder.buffer[i + 1], decoder.buffer[i + 2], decoder.buffer[i + 3],
                decoder.buffer[i + 4], decoder.buffer[i + 5], decoder.buffer[i + 6], decoder.buffer[i + 7]);
          } else {
            LOG_ERR("    [%02zu]: partial (%zu bytes)", i, bytes_left);
          }
        }
      }

      fusain_reset_decoder(&decoder);
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
static void process_packet(const fusain_packet_t* packet, struct serial_state* state,
    uint64_t current_micros)
{
  LOG_DBG("RX packet type 0x%02X, length %d", packet->msg_type,
      packet->length);

  switch (packet->msg_type) {
  case FUSAIN_MSG_STATE_COMMAND: {
    struct state_command_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_state_command_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode STATE_COMMAND: %d", ret);
      return;
    }

    struct state_command_msg state_cmd;
    switch (decoded.state_command_payload_mode_m) {
    case FUSAIN_MODE_IDLE:
      state_cmd.mode = FUSAIN_MODE_IDLE;
      state_cmd.argument = 0;
      break;
    case FUSAIN_MODE_FAN:
      state_cmd.mode = FUSAIN_MODE_FAN;
      // Argument is optional in CBOR
      state_cmd.argument = decoded.state_command_payload_uint1int_present
          ? decoded.state_command_payload_uint1int.state_command_payload_uint1int
          : 0;
      break;
    case FUSAIN_MODE_HEAT:
      state_cmd.mode = FUSAIN_MODE_HEAT;
      state_cmd.argument = 0;
      break;
    case FUSAIN_MODE_EMERGENCY:
      state_cmd.mode = FUSAIN_MODE_EMERGENCY;
      state_cmd.argument = 0;
      break;
    default:
      LOG_WRN("Invalid mode: 0x%02X", decoded.state_command_payload_mode_m);
      return;
    }

    LOG_INF("SET_MODE: mode=%d, arg=%d", state_cmd.mode,
        state_cmd.argument);
    zbus_chan_pub(&state_command_chan, &state_cmd, K_NO_WAIT);
    break;
  }

  case FUSAIN_MSG_PUMP_COMMAND: {
    struct pump_command_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_pump_command_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode PUMP_COMMAND: %d", ret);
      return;
    }

    struct pump_command_msg pump_cmd = {
      .pump = (int)decoded.pump_command_payload_pump_index_m,
      .rate_ms = decoded.pump_command_payload_uint1int
    };

    LOG_INF("SET_PUMP_RATE: %d ms", pump_cmd.rate_ms);
    zbus_chan_pub(&pump_command_chan, &pump_cmd, K_NO_WAIT);
    break;
  }

  case FUSAIN_MSG_MOTOR_COMMAND: {
    struct motor_command_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_motor_command_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode MOTOR_COMMAND: %d", ret);
      return;
    }

    struct motor_command_msg motor_cmd = {
      .motor = (int)decoded.motor_command_payload_motor_index_m,
      .rpm = decoded.motor_command_payload_uint1int
    };

    LOG_INF("SET_TARGET_RPM: %d", motor_cmd.rpm);
    zbus_chan_pub(&motor_command_chan, &motor_cmd, K_NO_WAIT);
    break;
  }

  case FUSAIN_MSG_GLOW_COMMAND: {
    struct glow_command_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_glow_command_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode GLOW_COMMAND: %d", ret);
      return;
    }

    int32_t duration = decoded.glow_command_payload_uint1int;

    // Validate duration (0-300000 ms)
    if (duration < 0 || duration > 300000) {
      LOG_WRN("Invalid glow duration: %d ms", duration);
      return;
    }

    // Send to glow controller via zbus
    struct glow_command_msg glow_cmd = {
      .glow = (int)decoded.glow_command_payload_glow_index_m,
      .duration = duration
    };
    zbus_chan_pub(&glow_command_chan, &glow_cmd, K_NO_WAIT);
    LOG_DBG("Glow command: glow=%d, duration=%d ms", glow_cmd.glow, duration);
    break;
  }

  case FUSAIN_MSG_PING_REQUEST: {
    LOG_DBG("Ping request received");
    state->last_ping_time = current_micros; // Update timeout
    serial_send_ping_response();
    break;
  }

  case FUSAIN_MSG_TELEMETRY_CONFIG: {
    struct telemetry_config_payload decoded;
    size_t decoded_len;
    size_t hdr_len = cbor_header_len(packet->msg_type);
    int ret = cbor_decode_telemetry_config_payload(packet->payload + hdr_len,
        packet->length - hdr_len, &decoded, &decoded_len);
    if (ret != 0) {
      LOG_WRN("Failed to decode TELEMETRY_CONFIG: %d", ret);
      return;
    }

    LOG_DBG("TELEMETRY_CONFIG received: enabled=%d, interval=%u",
        decoded.telemetry_config_payload_uint0bool,
        decoded.telemetry_config_payload_uint1uint);

    bool was_enabled = state->telemetry_enabled;
    state->telemetry_enabled = decoded.telemetry_config_payload_uint0bool;
    state->telemetry_interval_ms = decoded.telemetry_config_payload_uint1uint;

    // Clamp interval to valid range (100-5000 ms)
    if (state->telemetry_interval_ms < 100) {
      state->telemetry_interval_ms = 100;
    } else if (state->telemetry_interval_ms > 5000) {
      state->telemetry_interval_ms = 5000;
    }

    // Log INF only when telemetry is first enabled
    if (state->telemetry_enabled && !was_enabled) {
      LOG_INF("Telemetry enabled: interval=%u ms",
          state->telemetry_interval_ms);
    } else {
      LOG_DBG("Telemetry config applied: enabled=%d, interval=%u ms",
          state->telemetry_enabled, state->telemetry_interval_ms);
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
  fusain_packet_t rx_packet;
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

  fusain_packet_t tx_packet;
  if (k_msgq_get(&tx_packet_queue, &tx_packet, K_NO_WAIT) == 0) {
    fill_tx_buffer(&tx_packet);
  }
}

/**
 * Fill TX Buffer - Encodes packet and prepares for transmission
 *
 * Called from TX thread to prepare packet for UART transmission.
 */
static void fill_tx_buffer(const fusain_packet_t* packet)
{
  // Encode packet to buffer
  int encoded_len = fusain_encode_packet(packet, tx_buffer, sizeof(tx_buffer));

  if (encoded_len < 0) {
    LOG_ERR("Failed to encode packet: %d", encoded_len);
    return;
  }

  // Prepare TX state
  tx_index = 0;
  tx_length = (size_t)encoded_len;

  LOG_DBG("Starting TX: type=0x%02X, %zu bytes", packet->msg_type, tx_length);
}

/**
 * Queue Packet for Transmission
 *
 * Queues packet for transmission by serial thread.
 */
static void send_packet(const fusain_packet_t* packet)
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
    if (state_data.state != FUSAIN_STATE_IDLE) {
      LOG_WRN("Communication timeout - transitioning to IDLE");

      struct state_command_msg cmd = { .mode = FUSAIN_MODE_IDLE,
        .argument = 0 };
      zbus_chan_pub(&state_command_chan, &cmd, K_NO_WAIT);
    }
  }

  // Reset timeout
  state->last_ping_time = current_micros;
}

/**
 * Send Telemetry
 *
 * Sends periodic telemetry if enabled.
 */
static void send_telemetry_periodic(struct serial_state* state, uint64_t current_micros)
{
  if (!state->telemetry_enabled) {
    return;
  }

  // On first call (last_time == 0), send immediately
  if (state->last_telemetry_time == 0) {
    serial_send_telemetry();
    state->last_telemetry_time = current_micros;
    return;
  }

  const uint64_t micros_since_telemetry = current_micros - state->last_telemetry_time;
  if (micros_since_telemetry < (state->telemetry_interval_ms * 1000)) {
    return;
  }

  serial_send_telemetry();
  state->last_telemetry_time = current_micros;
}
