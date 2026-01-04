// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef HELIOS_MESSAGES_H
#define HELIOS_MESSAGES_H

#include <fusain/fusain.h>
#include <stdbool.h>

//////////////////////////////////////////////////////////////
// State
//////////////////////////////////////////////////////////////

struct state_command_msg {
  helios_mode_t mode;
  int argument;
};

struct state_data_msg {
  bool error;
  int code;
  helios_state_t state;
  unsigned timestamp;
};

//////////////////////////////////////////////////////////////
// Glow plugs
//////////////////////////////////////////////////////////////

struct glow_command_msg {
  int glow;
  int duration;
};

struct glow_data_msg {
  int glow;
  unsigned timestamp;
  bool lit;
};

//////////////////////////////////////////////////////////////
// Pumps
//////////////////////////////////////////////////////////////

struct pump_command_msg {
  int pump;
  int rate_ms;
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
  int pump;
  unsigned timestamp;
  enum pump_data_msg_type type;
  int rate;
};

//////////////////////////////////////////////////////////////
// Motors
//////////////////////////////////////////////////////////////

struct motor_command_msg {
  int motor;
  int rpm;
};

struct motor_data_msg {
  int motor;
  unsigned timestamp;
  int rpm;
  int target;
  int max_rpm;
  int min_rpm;
  int pwm;
  int pwm_max;
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
  int thermometer; // Temperature controller index
  enum temperature_command_type type; // Command type to execute
  int motor_index; // Motor index (used with WATCH_MOTOR)
  double target_temperature; // Target temperature (used with SET_TARGET_TEMP)
};

/**
 * Temperature data message with PID control status
 */
struct temperature_data_msg {
  int thermometer; // Temperature controller index
  unsigned timestamp; // Reading timestamp in microseconds
  double temperature; // Current temperature in Celsius
  bool pid_enabled; // PID controller active
  bool rpm_control_enabled; // Motor RPM control active
  int watched_motor; // Motor being monitored (-1 if none)
  double target_temperature; // Target temperature for PID control
};

#endif
