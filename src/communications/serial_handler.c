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

#include <helios/communications/serial_handler.h>
#include <helios/zbus.h>
#include <fusain/fusain.h>

LOG_MODULE_REGISTER(helios_serial_handler);

/* UART Device */
static const struct device* uart_dev = DEVICE_DT_GET(DT_ALIAS(helios_uart));

/* Decoder State */
static helios_decoder_t decoder;

/* TX Buffer and State */
static uint8_t tx_buffer[HELIOS_MAX_PACKET_SIZE * 2]; // 2x for stuffing overhead
static size_t tx_index = 0;
static size_t tx_length = 0;
K_MUTEX_DEFINE(tx_mutex); // Protects TX buffer and state

/* TX Packet Queue - API pushes, TX thread pops and transmits */
K_MSGQ_DEFINE(tx_packet_queue, sizeof(helios_packet_t), 8, 4);

/* RX Packet Queue - ISR pushes, thread pops */
K_MSGQ_DEFINE(rx_packet_queue, sizeof(helios_packet_t), 8, 4);

/* Timeout Mode Configuration */
static bool timeout_enabled = true; // Enabled by default
static uint32_t timeout_interval_ms = 30000; // 30 seconds default
static int64_t last_ping_time = 0;

/* Telemetry Configuration */
static bool telemetry_enabled = false; // Disabled by default (protocol v1.2)
static uint32_t telemetry_interval_ms = 100; // Default 100ms
static uint32_t telemetry_mode = 0; // 0 = bundled (default), 1 = individual

/* Forward Declarations */
static void recieve_packet(const helios_packet_t* packet);
static void send_packet(const helios_packet_t* packet);
static void fill_tx_buffer(const helios_packet_t* packet);
static void check_timeout(void);
static void uart_isr(const struct device* dev, void* user_data);

/* UART ISR - Handles both RX and TX interrupts */
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

  // Initialize timeout tracking
  last_ping_time = k_uptime_get();

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

/* RX Thread - Dequeues and processes received packets, checks timeout */
int serial_rx_thread(void)
{
  int ret = 0;
  // Initialize serial communication handler
  ret = serial_handler_init();
  if (ret < 0) {
    LOG_ERR("Failed to initialize serial handler: %d", ret);
    return ret;
  }

  while (1) {
    helios_packet_t packet;

    // Wait for packet from queue (with timeout for periodic checks)
    ret = k_msgq_get(&rx_packet_queue, &packet, K_MSEC(1000));
    if (ret == 0) {
      // Process packet in thread context (safe for logging, zbus, etc.)
      recieve_packet(&packet);
    }

    // Check timeout every iteration (at least once per second)
    check_timeout();
  }

  return ret;
}

/* TX Thread - Dequeues and transmits packets, sends periodic telemetry */
int serial_tx_thread(void)
{
  LOG_DBG("Serial TX thread started");

  while (1) {
    helios_packet_t packet;

    // Try to dequeue a packet with timeout (telemetry interval)
    int ret = k_msgq_get(&tx_packet_queue, &packet, K_MSEC(telemetry_interval_ms));

    if (ret == 0) {
      // Packet available - transmit it
      fill_tx_buffer(&packet);
    } else {
      // Timeout - send periodic telemetry (if enabled)
      serial_send_telemetry_bundle();
    }
  }

  return 0;
}

/* Check Timeout Mode */
static void check_timeout(void)
{
  if (!timeout_enabled) {
    return;
  }

  int64_t now = k_uptime_get();
  int64_t elapsed = now - last_ping_time;

  if (elapsed > timeout_interval_ms) {
    // Disable telemetry on timeout (protocol v1.2)
    if (telemetry_enabled) {
      LOG_WRN("Communication timeout - disabling telemetry");
      telemetry_enabled = false;
    }

    // Check current state before transitioning
    struct state_data_msg state_data;
    if (zbus_chan_read(&state_data_chan, &state_data, K_NO_WAIT) == 0) {
      // Only transition if not already in IDLE
      if (state_data.state != HELIOS_IDLE) {
        LOG_WRN("Communication timeout - transitioning to IDLE");

        struct state_command_msg cmd = { .mode = HELIOS_IDLE_MODE,
          .argument = 0 };
        zbus_chan_pub(&state_command_chan, &cmd, K_NO_WAIT);
      }
    }

    // Reset timeout
    last_ping_time = now;
  }
}

/* Process Received Packet (called from RX thread) */
static void recieve_packet(const helios_packet_t* packet)
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
      state_cmd.mode = HELIOS_IDLE_MODE;
      state_cmd.argument = 0;
      break;
    case HELIOS_MODE_FAN:
      state_cmd.mode = HELIOS_FAN_MODE;
      state_cmd.argument = (int)cmd->parameter; // RPM
      break;
    case HELIOS_MODE_HEAT:
      state_cmd.mode = HELIOS_HEAT_MODE;
      state_cmd.argument = 0;
      break;
    case HELIOS_MODE_EMERGENCY:
      state_cmd.mode = HELIOS_EMERGENCY;
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
    last_ping_time = k_uptime_get(); // Update timeout
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

    timeout_enabled = (cmd->timeout_enabled != 0);
    timeout_interval_ms = cmd->timeout_ms;

    LOG_INF("Timeout config: enabled=%d, interval=%u ms",
        timeout_enabled, timeout_interval_ms);

    // Reset timeout timer
    last_ping_time = k_uptime_get();
    break;
  }

  case HELIOS_MSG_EMERGENCY_STOP: {
    LOG_WRN("EMERGENCY_STOP received");

    struct state_command_msg state_cmd = { .mode = HELIOS_EMERGENCY,
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

    telemetry_enabled = (cmd->telemetry_enabled != 0);
    telemetry_interval_ms = cmd->interval_ms;
    telemetry_mode = cmd->telemetry_mode;

    // Clamp interval to valid range (100-5000 ms)
    if (telemetry_interval_ms < 100) {
      telemetry_interval_ms = 100;
    } else if (telemetry_interval_ms > 5000) {
      telemetry_interval_ms = 5000;
    }

    LOG_INF("Telemetry config applied: enabled=%d, interval=%u ms, mode=%u",
        telemetry_enabled, telemetry_interval_ms, telemetry_mode);
    break;
  }

  default:
    LOG_WRN("Unknown message type: 0x%02X", packet->msg_type);
    break;
  }
}

/* Queue Packet for Transmission */
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

/* Send Telemetry Bundle */
void serial_send_telemetry_bundle(void)
{
  // Check if telemetry is enabled (protocol v1.2)
  if (!telemetry_enabled) {
    LOG_DBG_RATELIMIT("Telemetry disabled, not sending bundle");
    return;
  }

  LOG_DBG_RATELIMIT("Sending telemetry bundle (enabled=%d)", telemetry_enabled);

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
  case HELIOS_INITIALIZING:
    serial_state = HELIOS_STATE_INITIALIZING;
    break;
  case HELIOS_IDLE:
    serial_state = HELIOS_STATE_IDLE;
    break;
  case HELIOS_BLOWING:
    serial_state = HELIOS_STATE_BLOWING;
    break;
  case HELIOS_PREHEAT:
    serial_state = HELIOS_STATE_PREHEAT;
    break;
  case HELIOS_PREHEAT_STAGE_2:
    serial_state = HELIOS_STATE_PREHEAT_STAGE_2;
    break;
  case HELIOS_HEATING:
    serial_state = HELIOS_STATE_HEATING;
    break;
  case HELIOS_COOLING:
    serial_state = HELIOS_STATE_COOLING;
    break;
  case HELIOS_ERROR:
    serial_state = HELIOS_STATE_ERROR;
    break;
  case HELIOS_E_STOP:
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

/* Get Timeout Configuration */
void serial_get_timeout_config(bool* enabled, uint32_t* timeout_ms)
{
  if (enabled) {
    *enabled = timeout_enabled;
  }
  if (timeout_ms) {
    *timeout_ms = timeout_interval_ms;
  }
}

/* Set Timeout Enabled */
void serial_set_timeout_enabled(bool enabled)
{
  timeout_enabled = enabled;
  // Reset timeout timer when changing state
  last_ping_time = k_uptime_get();
}
