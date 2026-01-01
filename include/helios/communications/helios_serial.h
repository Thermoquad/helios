/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Helios Serial Protocol - Shared Library
 *
 * This library provides reusable functions for encoding/decoding Helios serial
 * protocol packets. Can be used by both ICU (slave) and controller (master).
 */

#ifndef HELIOS_SERIAL_H
#define HELIOS_SERIAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Protocol Constants */
#define HELIOS_START_BYTE 0x7E
#define HELIOS_END_BYTE 0x7F
#define HELIOS_ESC_BYTE 0x7D
#define HELIOS_ESC_XOR 0x20

#define HELIOS_MAX_PACKET_SIZE 64
#define HELIOS_MAX_PAYLOAD_SIZE 58
#define HELIOS_MIN_PACKET_SIZE 6 // START + LEN + TYPE + CRC(2) + END

/* Message Type Definitions */
typedef enum {
  /* Command Messages (Master → ICU) */
  HELIOS_MSG_SET_MODE = 0x10,
  HELIOS_MSG_SET_PUMP_RATE = 0x11,
  HELIOS_MSG_SET_TARGET_RPM = 0x12,
  HELIOS_MSG_PING_REQUEST = 0x13,
  HELIOS_MSG_SET_TIMEOUT_CONFIG = 0x14,
  HELIOS_MSG_EMERGENCY_STOP = 0x15,

  /* Data Messages (ICU → Master) */
  HELIOS_MSG_STATE_DATA = 0x20,
  HELIOS_MSG_MOTOR_DATA = 0x21,
  HELIOS_MSG_TEMPERATURE_DATA = 0x22,
  HELIOS_MSG_PUMP_DATA = 0x23,
  HELIOS_MSG_GLOW_DATA = 0x24,
  HELIOS_MSG_TELEMETRY_BUNDLE = 0x25,
  HELIOS_MSG_PING_RESPONSE = 0x26,

  /* Error Messages (Bidirectional) */
  HELIOS_MSG_ERROR_INVALID_COMMAND = 0xE0,
  HELIOS_MSG_ERROR_INVALID_CRC = 0xE1,
  HELIOS_MSG_ERROR_INVALID_LENGTH = 0xE2,
  HELIOS_MSG_ERROR_TIMEOUT = 0xE3,
} helios_msg_type_t;

/* Operating Modes */
typedef enum {
  HELIOS_MODE_IDLE = 0x00,
  HELIOS_MODE_FAN = 0x01,
  HELIOS_MODE_HEAT = 0x02,
  HELIOS_MODE_EMERGENCY = 0xFF,
} helios_mode_t;

/* State Values (from state machine) */
typedef enum {
  HELIOS_STATE_INITIALIZING = 0x00,
  HELIOS_STATE_IDLE = 0x01,
  HELIOS_STATE_BLOWING = 0x02,
  HELIOS_STATE_PREHEAT = 0x03,
  HELIOS_STATE_PREHEAT_STAGE_2 = 0x04,
  HELIOS_STATE_HEATING = 0x05,
  HELIOS_STATE_COOLING = 0x06,
  HELIOS_STATE_ERROR = 0x07,
  HELIOS_STATE_E_STOP = 0x08,
} helios_state_t;

/* Error Codes */
typedef enum {
  HELIOS_ERROR_NONE = 0x00,
  HELIOS_ERROR_PREHEAT_FAILED = 0x01,
  HELIOS_ERROR_FLAME_OUT = 0x02,
  HELIOS_ERROR_OVERHEAT = 0x03,
  HELIOS_ERROR_PUMP_FAULT = 0x04,
  HELIOS_ERROR_TIMEOUT = 0x05,
} helios_error_t;

/* Command Payloads */
typedef struct __attribute__((packed)) {
  uint8_t mode; // helios_mode_t
  uint32_t parameter; // RPM for FAN mode, 0 for others
} helios_cmd_set_mode_t;

typedef struct __attribute__((packed)) {
  uint32_t rate_ms; // Pump interval in milliseconds
} helios_cmd_set_pump_rate_t;

typedef struct __attribute__((packed)) {
  uint32_t target_rpm;
} helios_cmd_set_target_rpm_t;

typedef struct __attribute__((packed)) {
  uint8_t timeout_enabled; // 0 = disabled, 1 = enabled
  uint32_t timeout_ms; // Timeout interval in milliseconds
} helios_cmd_set_timeout_config_t;

/* Data Payloads */
typedef struct __attribute__((packed)) {
  uint32_t state; // helios_state_t
  uint8_t error; // helios_error_t
} helios_data_state_t;

typedef struct __attribute__((packed)) {
  int32_t rpm;
  int32_t target_rpm;
  int32_t pwm_duty;
  int32_t min_rpm;
  int32_t max_rpm;
} helios_data_motor_t;

typedef struct __attribute__((packed)) {
  double temperature;
  uint8_t pid_enabled;
  double pid_setpoint;
  int32_t pid_output_rpm;
} helios_data_temperature_t;

typedef struct __attribute__((packed)) {
  uint8_t enabled;
  uint32_t rate_ms;
  uint64_t pulse_count;
} helios_data_pump_t;

typedef struct __attribute__((packed)) {
  uint8_t lit;
  uint64_t lit_timestamp;
  uint64_t total_burn_time;
} helios_data_glow_t;

typedef struct __attribute__((packed)) {
  uint64_t uptime_ms;
} helios_data_ping_response_t;

/* Variable-length telemetry bundle */
#define HELIOS_MAX_MOTORS 3
#define HELIOS_MAX_TEMPERATURES 3

typedef struct __attribute__((packed)) {
  int32_t rpm;
  int32_t target_rpm;
  int32_t pwm_duty;
} helios_telemetry_motor_t;

typedef struct __attribute__((packed)) {
  double temperature;
} helios_telemetry_temperature_t;

typedef struct __attribute__((packed)) {
  uint32_t state; // helios_state_t
  uint8_t error; // helios_error_t
  uint8_t motor_count; // Number of motors (1-3)
  uint8_t temp_count; // Number of temperature sensors (1-3)
  // Followed by:
  // - motor_count × helios_telemetry_motor_t
  // - temp_count × helios_telemetry_temperature_t
} helios_data_telemetry_bundle_t;

/* Error Message Payloads */
typedef struct __attribute__((packed)) {
  uint8_t invalid_command;
} helios_error_invalid_command_t;

typedef struct __attribute__((packed)) {
  uint16_t received_crc;
  uint16_t calculated_crc;
} helios_error_invalid_crc_t;

typedef struct __attribute__((packed)) {
  uint8_t received_length;
  uint8_t expected_length;
} helios_error_invalid_length_t;

/* Packet Structure */
typedef struct {
  uint8_t start; // HELIOS_START_BYTE
  uint8_t length; // Payload length (0-58)
  uint8_t msg_type; // helios_msg_type_t
  uint8_t payload[HELIOS_MAX_PAYLOAD_SIZE];
  uint16_t crc; // CRC-16-CCITT
  uint8_t end; // HELIOS_END_BYTE
} helios_packet_t;

/* Decode Result */
typedef enum {
  HELIOS_DECODE_OK = 0,
  HELIOS_DECODE_INCOMPLETE = 1,
  HELIOS_DECODE_INVALID_START = 2,
  HELIOS_DECODE_INVALID_CRC = 3,
  HELIOS_DECODE_INVALID_LENGTH = 4,
  HELIOS_DECODE_BUFFER_OVERFLOW = 5,
} helios_decode_result_t;

/* Function Declarations */

/**
 * Calculate CRC-16-CCITT (polynomial 0x1021, initial 0xFFFF)
 *
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @return CRC-16 value
 */
uint16_t helios_crc16(const uint8_t* data, size_t length);

/**
 * Encode a packet into a byte buffer with byte stuffing
 *
 * @param packet Packet structure to encode
 * @param buffer Output buffer for encoded bytes
 * @param buffer_size Size of output buffer
 * @return Number of bytes written, or negative error code
 */
int helios_encode_packet(const helios_packet_t* packet, uint8_t* buffer,
    size_t buffer_size);

/**
 * Decode received bytes into a packet structure
 *
 * This function handles byte unstuffing and CRC validation.
 * Call repeatedly as bytes are received until HELIOS_DECODE_OK or error.
 *
 * @param rx_byte Received byte to process
 * @param packet Output packet structure
 * @param state Decoder state (initialize to 0)
 * @param buffer Internal decode buffer (must be HELIOS_MAX_PACKET_SIZE)
 * @param buffer_index Current buffer index (initialize to 0)
 * @param escape_next Escape flag (initialize to false)
 * @return Decode result status
 */
helios_decode_result_t helios_decode_byte(uint8_t rx_byte,
    helios_packet_t* packet,
    uint8_t* state, uint8_t* buffer,
    size_t* buffer_index,
    bool* escape_next);

/**
 * Reset decoder state
 *
 * @param state Decoder state to reset
 * @param buffer_index Buffer index to reset
 * @param escape_next Escape flag to reset
 */
void helios_reset_decoder(uint8_t* state, size_t* buffer_index,
    bool* escape_next);

/**
 * Create a SET_MODE command packet
 *
 * @param packet Output packet
 * @param mode Operating mode
 * @param parameter Mode parameter (RPM for FAN mode)
 */
void helios_create_set_mode(helios_packet_t* packet, helios_mode_t mode,
    uint32_t parameter);

/**
 * Create a SET_PUMP_RATE command packet
 *
 * @param packet Output packet
 * @param rate_ms Pump rate in milliseconds
 */
void helios_create_set_pump_rate(helios_packet_t* packet, uint32_t rate_ms);

/**
 * Create a SET_TARGET_RPM command packet
 *
 * @param packet Output packet
 * @param target_rpm Target RPM
 */
void helios_create_set_target_rpm(helios_packet_t* packet, uint32_t target_rpm);

/**
 * Create a PING_REQUEST packet
 *
 * @param packet Output packet
 */
void helios_create_ping_request(helios_packet_t* packet);

/**
 * Create a SET_TIMEOUT_CONFIG packet
 *
 * @param packet Output packet
 * @param enabled Timeout mode enabled
 * @param timeout_ms Timeout interval in milliseconds
 */
void helios_create_set_timeout_config(helios_packet_t* packet, bool enabled,
    uint32_t timeout_ms);

/**
 * Create an EMERGENCY_STOP packet
 *
 * @param packet Output packet
 */
void helios_create_emergency_stop(helios_packet_t* packet);

/**
 * Create a STATE_DATA packet
 *
 * @param packet Output packet
 * @param state Current state
 * @param error Error code
 */
void helios_create_state_data(helios_packet_t* packet, helios_state_t state,
    helios_error_t error);

/**
 * Create a PING_RESPONSE packet
 *
 * @param packet Output packet
 * @param uptime_ms System uptime in milliseconds
 */
void helios_create_ping_response(helios_packet_t* packet, uint64_t uptime_ms);

/**
 * Create a TELEMETRY_BUNDLE packet
 *
 * @param packet Output packet
 * @param state Current state
 * @param error Error code
 * @param motors Array of motor data
 * @param motor_count Number of motors (1-3)
 * @param temperatures Array of temperature data
 * @param temp_count Number of temperatures (1-3)
 * @return 0 on success, negative on error
 */
int helios_create_telemetry_bundle(helios_packet_t* packet,
    helios_state_t state, helios_error_t error,
    const helios_telemetry_motor_t* motors,
    uint8_t motor_count,
    const helios_telemetry_temperature_t* temperatures,
    uint8_t temp_count);

#endif /* HELIOS_SERIAL_H */
