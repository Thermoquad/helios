// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef HELIOS_MESSAGES_H
#define HELIOS_MESSAGES_H

#include <fusain/fusain.h>
#include <stdbool.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////
// State
//////////////////////////////////////////////////////////////

struct state_command_msg {
  fusain_mode_t mode;
  int32_t argument;
};

struct state_data_msg {
  bool error;
  uint8_t code;
  fusain_state_t state;
  uint32_t timestamp;
};

//////////////////////////////////////////////////////////////
// Glow plugs
//////////////////////////////////////////////////////////////

struct glow_command_msg {
  uint8_t glow;
  int32_t duration;
};

struct glow_data_msg {
  uint8_t glow;
  uint32_t timestamp;
  bool lit;
};

//////////////////////////////////////////////////////////////
// Pumps
//////////////////////////////////////////////////////////////

struct pump_command_msg {
  uint8_t pump;
  int32_t rate_ms;
};

enum pump_data_msg_type {
  PUMP_INITIALIZING,
  PUMP_READY,
  PUMP_ERROR,
  PUMP_CYCLE_START,
  PUMP_PULSE_END,
  PUMP_CYCLE_END
};

struct pump_data_msg {
  uint8_t pump;
  uint32_t timestamp;
  enum pump_data_msg_type type;
  int32_t rate;
};

//////////////////////////////////////////////////////////////
// Motors
//////////////////////////////////////////////////////////////

struct motor_command_msg {
  uint8_t motor;
  int32_t rpm;
};

struct motor_data_msg {
  uint8_t motor;
  uint32_t timestamp;
  int32_t rpm;
  int32_t target;
  int32_t max_rpm;
  int32_t min_rpm;
  uint32_t pwm;
  uint32_t pwm_max;
};

//////////////////////////////////////////////////////////////
// Temperature
//////////////////////////////////////////////////////////////

/**
 * Temperature controller command types for motor RPM control
 */
enum temperature_command_type {
  TEMP_CMD_WATCH_MOTOR, // Associate temp controller with a motor to monitor RPM
  TEMP_CMD_UNWATCH_MOTOR, // Stop watching motor RPM data
  TEMP_CMD_ENABLE_RPM_CONTROL, // Enable inverted PID control of motor RPM
  TEMP_CMD_DISABLE_RPM_CONTROL, // Disable PID control of motor RPM
  TEMP_CMD_SET_TARGET_TEMP, // Set target temperature for PID control
};

/**
 * Temperature controller command message
 */
struct temperature_command_msg {
  uint8_t thermometer; // Temperature controller index
  enum temperature_command_type type; // Command type to execute
  uint8_t motor_index; // Motor index (used with WATCH_MOTOR)
  float target_temperature; // Target temperature (used with SET_TARGET_TEMP)
};

/**
 * Temperature data message with PID control status
 */
struct temperature_data_msg {
  uint8_t thermometer; // Temperature controller index
  uint32_t timestamp; // Reading timestamp in microseconds
  float reading; // Current temperature in Celsius
  bool pid_enabled; // PID controller active (internal use, no Fusain equivalent)
  bool temperature_rpm_control; // Motor RPM control active
  int32_t watched_motor; // Motor being monitored (-1 if none)
  float target_temperature; // Target temperature for PID control
};

//////////////////////////////////////////////////////////////
// Configuration Commands
//////////////////////////////////////////////////////////////

struct motor_config_msg {
  uint8_t motor;
  bool pwm_period_present;
  uint32_t pwm_period;
  bool pid_kp_present;
  double pid_kp;
  bool pid_ki_present;
  double pid_ki;
  bool pid_kd_present;
  double pid_kd;
  bool max_rpm_present;
  int32_t max_rpm;
  bool min_rpm_present;
  int32_t min_rpm;
  bool min_pwm_duty_present;
  uint32_t min_pwm_duty;
};

struct pump_config_msg {
  uint8_t pump;
  bool pulse_ms_present;
  uint32_t pulse_ms;
  bool recovery_ms_present;
  uint32_t recovery_ms;
};

struct temp_config_msg {
  uint8_t thermometer;
  bool pid_kp_present;
  double pid_kp;
  bool pid_ki_present;
  double pid_ki;
  bool pid_kd_present;
  double pid_kd;
};

struct glow_config_msg {
  uint8_t glow;
  bool max_duration_present;
  uint32_t max_duration;
};

struct telemetry_config_msg {
  bool enabled;
  uint32_t interval_ms;
};

struct timeout_config_msg {
  bool enabled;
  uint32_t timeout_ms;
};

//////////////////////////////////////////////////////////////
// Data Messages
//////////////////////////////////////////////////////////////

struct device_announce_msg {
  uint8_t motor_count;
  uint8_t thermometer_count;
  uint8_t pump_count;
  uint8_t glow_count;
};

struct send_telemetry_msg {
  uint8_t telemetry_type;
  bool index_present;
  uint8_t index;
};

//////////////////////////////////////////////////////////////
// Error Messages
//////////////////////////////////////////////////////////////

struct error_invalid_cmd_msg {
  int32_t error_code;
};

struct error_state_reject_msg {
  uint8_t current_state;
};

#endif
