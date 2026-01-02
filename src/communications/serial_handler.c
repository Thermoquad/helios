/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Helios Serial Handler - ICU Implementation
 *
 * Handles UART communication with master controller, processes commands,
 * sends telemetry data, and implements timeout mode for safety.
 */

#include <helios/communications/serial_handler.h>
#include <helios/zbus.h>
#include <helios_serial/helios_serial.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(helios_serial_handler);

/* UART Device */
static const struct device* uart_dev;

/* Decoder State */
static uint8_t decoder_state = 0;
static uint8_t decode_buffer[HELIOS_MAX_PACKET_SIZE];
static size_t decode_buffer_index = 0;
static bool decode_escape_next = false;

/* TX Buffer and State */
static uint8_t tx_buffer[HELIOS_MAX_PACKET_SIZE * 2]; // 2x for stuffing overhead
static size_t tx_index = 0;
static size_t tx_length = 0;
static bool tx_in_progress = false;
K_MUTEX_DEFINE(tx_mutex); // Protects TX buffer and state

/* RX Packet Queue - ISR pushes, thread pops */
K_MSGQ_DEFINE(rx_packet_queue, sizeof(helios_packet_t), 8, 4);

/* Work queue for deferred packet sending from ISR */
static struct k_work ping_response_work;
static void ping_response_work_handler(struct k_work* work);

/* Timeout Mode Configuration */
static bool timeout_enabled = true; // Enabled by default
static uint32_t timeout_interval_ms = 30000; // 30 seconds default
static int64_t last_ping_time = 0;

/* Forward Declarations */
static void process_packet(const helios_packet_t* packet);
static void send_packet(const helios_packet_t* packet);
static void check_timeout(void);
static void uart_isr(const struct device* dev, void* user_data);

/* Work handler for sending ping response from thread context */
static void ping_response_work_handler(struct k_work* work)
{
  ARG_UNUSED(work);
  serial_send_ping_response();
}

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
      helios_decode_result_t result = helios_decode_byte(
          byte, &packet, &decoder_state, decode_buffer,
          &decode_buffer_index, &decode_escape_next);

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
        helios_reset_decoder(&decoder_state,
            &decode_buffer_index,
            &decode_escape_next);
      }
    }
  }

  // Handle TX
  if (uart_irq_tx_ready(dev)) {
    if (tx_in_progress && tx_index < tx_length) {
      // Fill FIFO with as much data as possible
      size_t remaining = tx_length - tx_index;
      size_t sent = uart_fifo_fill(dev, &tx_buffer[tx_index], remaining);
      tx_index += sent;

      // Check if transmission complete
      if (tx_index >= tx_length) {
        tx_in_progress = false;
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
  uart_dev = DEVICE_DT_GET(DT_ALIAS(helios_uart));
  if (!device_is_ready(uart_dev)) {
    LOG_ERR("UART device not ready");
    return -1;
  }

  LOG_DBG("UART device ready: %s", uart_dev->name);

  // Initialize decoder BEFORE enabling interrupts
  helios_reset_decoder(&decoder_state, &decode_buffer_index,
      &decode_escape_next);
  LOG_DBG("Decoder initialized");

  // Flush RX FIFO to discard any garbage bytes
  uint8_t discard;
  int flushed = 0;
  while (uart_fifo_read(uart_dev, &discard, 1) == 1) {
    flushed++;
  }
  if (flushed > 0) {
    LOG_DBG("Flushed %d bytes from RX FIFO", flushed);
  }

  // Initialize work queue for deferred sends from ISR
  k_work_init(&ping_response_work, ping_response_work_handler);
  LOG_DBG("Ping response work queue initialized");

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
  while (1) {
    helios_packet_t packet;

    // Wait for packet from queue (with timeout for periodic checks)
    int ret = k_msgq_get(&rx_packet_queue, &packet, K_MSEC(1000));
    if (ret == 0) {
      // Process packet in thread context (safe for logging, zbus, etc.)
      process_packet(&packet);
    }

    // Check timeout every iteration (at least once per second)
    check_timeout();
  }

  return 0;
}

/* TX Thread - Sends periodic telemetry */
int serial_tx_thread(void)
{
  while (1) {
    // Send telemetry bundle every 100ms
    serial_send_telemetry_bundle();
    k_sleep(K_MSEC(100));
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
static void process_packet(const helios_packet_t* packet)
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
    // Submit work to send response from thread context (not ISR)
    k_work_submit(&ping_response_work);
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

  default:
    LOG_WRN("Unknown message type: 0x%02X", packet->msg_type);
    break;
  }
}

/* Send Packet via UART using interrupt-driven TX */
static void send_packet(const helios_packet_t* packet)
{
  // Lock to prevent concurrent transmission attempts
  k_mutex_lock(&tx_mutex, K_FOREVER);

  // Wait for any previous transmission to complete
  while (tx_in_progress) {
    k_yield();
  }

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
  tx_in_progress = true;

  LOG_DBG("Starting TX: type=0x%02X, %zu bytes", packet->msg_type, tx_length);

  // Enable TX interrupt - this will trigger ISR to start sending
  uart_irq_tx_enable(uart_dev);

  k_mutex_unlock(&tx_mutex);
}

/* Send Telemetry Bundle */
void serial_send_telemetry_bundle(void)
{
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
    .pwm_duty = motor_data.pwm };

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
  LOG_DBG("Sending ping response (uptime=%llu ms)", uptime);
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
