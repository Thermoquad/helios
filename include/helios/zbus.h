#ifndef HELIOS_ZBUS_H
#define HELIOS_ZBUS_H
#include <zephyr/zbus/zbus.h>

//////////////////////////////////////////////////////////////
// Channels
//////////////////////////////////////////////////////////////

ZBUS_CHAN_DECLARE(
    state_command_chan,
    state_data_chan,
    glow_command_chan,
    glow_data_chan,
    motor_command_chan,
    motor_data_chan,
    pump_command_chan,
    pump_data_chan,
    temperature_data_chan);

ZBUS_OBS_DECLARE(state_data_listener);

//////////////////////////////////////////////////////////////
// State
//////////////////////////////////////////////////////////////

enum helios_states {
  HELIOS_INITIALIZING,
  HELIOS_IDLE,
  HELIOS_BLOWING,
  HELIOS_PREHEAT,
  HELIOS_PREHEAT_STAGE_2,
  HELIOS_HEATING,
  HELIOS_COOLING,
  HELIOS_ERROR,
  HELIOS_E_STOP,
};

enum helios_modes {
  HELIOS_FAN_MODE,
  HELIOS_HEAT_MODE,
  HELIOS_IDLE_MODE,
  HELIOS_EMERGENCY,
};

struct state_command_msg {
  enum helios_modes mode;
  int argument;
};

struct state_data_msg {
  bool error;
  int code;
  enum helios_states state;
  unsigned timestamp;
};

extern const char helios_state_names[9][18];
extern const char helios_mode_names[4][12];

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

struct temperature_data_msg {
  int thermometer;
  unsigned timestamp;
  double temperature;
};

#endif
