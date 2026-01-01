/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Helios Serial Handler - ICU Implementation
 *
 * Handles UART communication with master controller, processes commands,
 * sends telemetry data, and implements timeout mode for safety.
 */

#include <helios/communications/helios_serial.h>
#include <helios/communications/serial_handler.h>
#include <helios/zbus.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(serial_handler, LOG_LEVEL_INF);

/* UART Device */
static const struct device* uart_dev;

/* Decoder State */
static uint8_t decoder_state = 0;
static uint8_t decode_buffer[HELIOS_MAX_PACKET_SIZE];
static size_t decode_buffer_index = 0;
static bool decode_escape_next = false;

/* TX Buffer */
static uint8_t tx_buffer[HELIOS_MAX_PACKET_SIZE * 2]; // 2x for stuffing overhead

/* Timeout Mode Configuration */
static bool timeout_enabled = true; // Enabled by default
static uint32_t timeout_interval_ms = 30000; // 30 seconds default
static int64_t last_ping_time = 0;

/* Thread Stacks and Structures */
#define RX_THREAD_STACK_SIZE 2048
#define TX_THREAD_STACK_SIZE 2048
#define RX_THREAD_PRIORITY 5
#define TX_THREAD_PRIORITY 6

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);

static struct k_thread rx_thread_data;
static struct k_thread tx_thread_data;

/* Forward Declarations */
static void rx_thread(void* p1, void* p2, void* p3);
static void tx_thread(void* p1, void* p2, void* p3);
static void process_packet(const helios_packet_t* packet);
static void send_packet(const helios_packet_t* packet);
static void check_timeout(void);

/* UART Callback */
static void uart_rx_callback(const struct device* dev, void* user_data)
{
  ARG_UNUSED(user_data);

  uint8_t byte;
  while (uart_poll_in(dev, &byte) == 0) {
    helios_packet_t packet;
    helios_decode_result_t result = helios_decode_byte(
        byte, &packet, &decoder_state, decode_buffer,
        &decode_buffer_index, &decode_escape_next);

    if (result == HELIOS_DECODE_OK) {
      // Packet received successfully
      process_packet(&packet);
    } else if (result != HELIOS_DECODE_INCOMPLETE) {
      // Decode error
      LOG_WRN("Decode error: %d", result);
      helios_reset_decoder(&decoder_state,
          &decode_buffer_index,
          &decode_escape_next);
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

  // Configure UART for interrupt-driven reception
  uart_irq_callback_set(uart_dev, uart_rx_callback);
  uart_irq_rx_enable(uart_dev);

  // Initialize decoder
  helios_reset_decoder(&decoder_state, &decode_buffer_index,
      &decode_escape_next);

  // Initialize timeout tracking
  last_ping_time = k_uptime_get();

  // Start RX thread
  k_thread_create(&rx_thread_data, rx_thread_stack,
      K_THREAD_STACK_SIZEOF(rx_thread_stack), rx_thread,
      NULL, NULL, NULL, RX_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&rx_thread_data, "serial_rx");

  // Start TX thread
  k_thread_create(&tx_thread_data, tx_thread_stack,
      K_THREAD_STACK_SIZEOF(tx_thread_stack), tx_thread,
      NULL, NULL, NULL, TX_THREAD_PRIORITY, 0, K_NO_WAIT);
  k_thread_name_set(&tx_thread_data, "serial_tx");

  LOG_INF("Serial handler initialized on %s", uart_dev->name);

  return 0;
}

/* RX Thread - Handles timeout checking */
static void rx_thread(void* p1, void* p2, void* p3)
{
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    check_timeout();
    k_sleep(K_MSEC(1000)); // Check every second
  }
}

/* TX Thread - Sends periodic telemetry */
static void tx_thread(void* p1, void* p2, void* p3)
{
  ARG_UNUSED(p1);
  ARG_UNUSED(p2);
  ARG_UNUSED(p3);

  while (1) {
    // Send telemetry bundle every 100ms
    serial_send_telemetry_bundle();
    k_sleep(K_MSEC(100));
  }
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
    LOG_WRN("Communication timeout - transitioning to IDLE");

    // Send IDLE command to state machine
    struct state_command_msg cmd = { .mode = HELIOS_IDLE_MODE,
      .argument = 0 };
    zbus_chan_pub(&state_command_chan, &cmd, K_NO_WAIT);

    // Reset timeout
    last_ping_time = now;
  }
}

/* Process Received Packet */
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
    LOG_DBG("PING_REQUEST received");
    last_ping_time = k_uptime_get(); // Update timeout
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

  default:
    LOG_WRN("Unknown message type: 0x%02X", packet->msg_type);
    break;
  }
}

/* Send Packet via UART */
static void send_packet(const helios_packet_t* packet)
{
  int encoded_len = helios_encode_packet(packet, tx_buffer, sizeof(tx_buffer));

  if (encoded_len < 0) {
    LOG_ERR("Failed to encode packet: %d", encoded_len);
    return;
  }

  // Send via UART
  for (int i = 0; i < encoded_len; i++) {
    uart_poll_out(uart_dev, tx_buffer[i]);
  }

  LOG_DBG("TX packet type 0x%02X, encoded %d bytes", packet->msg_type,
      encoded_len);
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
