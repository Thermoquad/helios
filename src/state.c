#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/smf.h>

#include <helios/zbus.h>

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(helios_state);

#define LOOP_SLEEP K_MSEC(5U)
#define MUTEX_WAIT K_MSEC(100U)
#define PUB_TIMEOUT K_MSEC(10U)
#define REPORT_RATE_MS 250

#define FAULT_TEMP 275.0

#define RPM_MARGIN 0.05
#define PREHEAT_PUMP_RATE 500
#define PREHEAT_PUMP_DELAY 30 * 1e6
#define PREHEAT_RPM 2500
#define PREHEAT_GLOW_DURATION 5 * 60 * 1000
#define PREHEAT_SUCCESS_TEMP 210.0
#define PREHEAT_STAGE_2_RPM 2800
#define PREHEAT_STAGE_2_TEMP 190.0
#define PREHEAT_STAGE_2_PUMP_RATE 250

#define BURN_TEMP 220.0
#define FLAME_OUT_TEMP PREHEAT_STAGE_2_TEMP

// Temperature PID control configuration
#define TEMP_CONTROLLER_INDEX 0  // Which thermometer to use
#define TEMP_MOTOR_INDEX 0       // Which motor to control
#define HEATING_TARGET_TEMP 230.0  // Target temperature during heating

// Cooldown configuration
#define COOLDOWN_THRESHOLD_TEMP 190.0  // Temperature that requires cooldown
#define COOLDOWN_GLOW_START_TEMP 180.0  // Temperature to light glow plug
#define COOLDOWN_COMPLETE_TEMP 120.0   // Temperature to end cooldown
#define COOLDOWN_FAN_RPM 2500          // Fan speed during cooldown

//////////////////////////////////////////////////////////////
// State names
//////////////////////////////////////////////////////////////

const char helios_state_names[9][18] = {
  [HELIOS_INITIALIZING] = "initializing",
  [HELIOS_IDLE] = "idle",
  [HELIOS_BLOWING] = "fan mode",
  [HELIOS_PREHEAT] = "preheat stage 1",
  [HELIOS_PREHEAT_STAGE_2] = "preheat stage 2",
  [HELIOS_HEATING] = "heat mode",
  [HELIOS_COOLING] = "cooling",
  [HELIOS_ERROR] = "error",
  [HELIOS_E_STOP] = "emergency stop",
};

const char helios_mode_names[4][12] = {
  [HELIOS_IDLE_MODE] = "idle",
  [HELIOS_FAN_MODE] = "fan",
  [HELIOS_HEAT_MODE] = "heat",
  [HELIOS_EMERGENCY] = "emergency",
};

//////////////////////////////////////////////////////////////
// State variables
//////////////////////////////////////////////////////////////

K_MUTEX_DEFINE(state_machine_mutex);
static enum helios_states current_state;

static int current_rpm;
static int max_rpm;
static int min_rpm;
static int target_rpm;

static double temperature;
static bool glowing = false;
static unsigned glow_lit_at;

static bool pump_ready = false;
static unsigned last_pump_at;

static unsigned last_report_at;
static unsigned preheat_started_at;

static int user_pump_rate_target;
static int current_pump_rate;

// Temperature PID control tracking
static bool temp_pid_enabled = false;
static int temp_controller_index = TEMP_CONTROLLER_INDEX;
static double target_temperature = HEATING_TARGET_TEMP;

// Cooldown tracking
static bool reached_high_temp = false;

//////////////////////////////////////////////////////////////
// State machine framework variables
//////////////////////////////////////////////////////////////

static const struct smf_state helios_state_machine[];

static struct s_object {
  struct smf_ctx ctx;
} helios_state_ctx;

//////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////

static void zbus_publish_state(enum helios_states state, int code, bool error)
{
  const struct state_data_msg msg = {
    .timestamp = k_cyc_to_us_floor64(k_cycle_get_64()),
    .state = state,
    .code = code,
    .error = error,
  };
  zbus_chan_pub(&state_data_chan, &msg, PUB_TIMEOUT);
}

static void report_state()
{
  const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());
  if (last_report_at == 0 || current_micros >= last_report_at + (REPORT_RATE_MS * 1000)) {
    zbus_publish_state(current_state, 0, false);
    last_report_at = current_micros;
  }
}

static int set_rpm(int new_target_rpm)
{
  if (new_target_rpm == target_rpm) {
    return 0;
  }
  struct motor_command_msg msg = { .motor = 0, .rpm = new_target_rpm };
  LOG_DBG("setting motor %d RPM to %d", 0, new_target_rpm);
  return zbus_chan_pub(&motor_command_chan, &msg, PUB_TIMEOUT);
}

static bool rpm_on_target()
{
  const int rpm_margin = current_rpm * RPM_MARGIN;
  const int min_target_rpm = target_rpm - rpm_margin;
  const int max_target_rpm = target_rpm + rpm_margin;

  if (current_rpm > min_target_rpm && current_rpm < max_target_rpm) {
    return true;
  } else {
    return false;
  }
}

static int start_glow_burn()
{
  if (glowing) {
    return 0;
  }
  struct glow_command_msg cmd = { .glow = 0, .duration = PREHEAT_GLOW_DURATION };
  LOG_DBG("lighting glow plug %d", 0);
  return zbus_chan_pub(&glow_command_chan, &cmd, PUB_TIMEOUT);
}

static int stop_glow_burn()
{
  if (!glowing) {
    return 0;
  }
  struct glow_command_msg cmd = { .glow = 0, .duration = 0 };
  LOG_DBG("extinguishing glow plug %d", 0);
  return zbus_chan_pub(&glow_command_chan, &cmd, PUB_TIMEOUT);
}

static int set_pump_rate(int rate)
{
  if (current_pump_rate == rate) {
    return 0;
  }
  struct pump_command_msg cmd = { .pump = 0, .rate_ms = rate };
  LOG_DBG("setting pump rate to %d", rate);
  return zbus_chan_pub(&pump_command_chan, &cmd, PUB_TIMEOUT);
}

static bool preheat_failed()
{
  if (glowing) {
    return false;
  }

  if (temperature >= PREHEAT_SUCCESS_TEMP) {
    return false;
  }

  if (temperature >= PREHEAT_STAGE_2_TEMP) {
    return false;
  }

  if (!glowing && glow_lit_at == 0) {
    return false;
  }

  LOG_ERR("preheat failed");
  return true;
}

static int enable_temp_pid_control()
{
  if (temp_pid_enabled) {
    return 0;
  }

  int ret = 0;

  // Step 1: Tell temperature controller to watch the motor
  struct temperature_command_msg watch_cmd = {
    .thermometer = temp_controller_index,
    .type = TEMP_CMD_WATCH_MOTOR,
    .motor_index = TEMP_MOTOR_INDEX,
    .target_temperature = 0.0
  };
  ret = zbus_chan_pub(&temperature_command_chan, &watch_cmd, PUB_TIMEOUT);
  if (ret) {
    LOG_ERR("Failed to send watch motor command");
    return ret;
  }
  LOG_DBG("Temperature controller %d watching motor %d", temp_controller_index, TEMP_MOTOR_INDEX);

  // Step 2: Set target temperature
  struct temperature_command_msg temp_cmd = {
    .thermometer = temp_controller_index,
    .type = TEMP_CMD_SET_TARGET_TEMP,
    .motor_index = 0,
    .target_temperature = target_temperature
  };
  ret = zbus_chan_pub(&temperature_command_chan, &temp_cmd, PUB_TIMEOUT);
  if (ret) {
    LOG_ERR("Failed to send set target temp command");
    return ret;
  }
  LOG_DBG("Temperature controller %d target set to %.2f", temp_controller_index, target_temperature);

  // Step 3: Enable RPM control
  struct temperature_command_msg enable_cmd = {
    .thermometer = temp_controller_index,
    .type = TEMP_CMD_ENABLE_RPM_CONTROL,
    .motor_index = 0,
    .target_temperature = 0.0
  };
  ret = zbus_chan_pub(&temperature_command_chan, &enable_cmd, PUB_TIMEOUT);
  if (ret) {
    LOG_ERR("Failed to send enable RPM control command");
    return ret;
  }

  temp_pid_enabled = true;
  LOG_INF("Temperature PID control enabled on thermometer %d", temp_controller_index);
  return ret;
}

static int disable_temp_pid_control()
{
  if (!temp_pid_enabled) {
    return 0;
  }

  int ret = 0;

  // Disable RPM control
  struct temperature_command_msg disable_cmd = {
    .thermometer = temp_controller_index,
    .type = TEMP_CMD_DISABLE_RPM_CONTROL,
    .motor_index = 0,
    .target_temperature = 0.0
  };
  ret = zbus_chan_pub(&temperature_command_chan, &disable_cmd, PUB_TIMEOUT);
  if (ret) {
    LOG_ERR("Failed to send disable RPM control command");
    return ret;
  }

  // Stop watching motor
  struct temperature_command_msg unwatch_cmd = {
    .thermometer = temp_controller_index,
    .type = TEMP_CMD_UNWATCH_MOTOR,
    .motor_index = 0,
    .target_temperature = 0.0
  };
  ret = zbus_chan_pub(&temperature_command_chan, &unwatch_cmd, PUB_TIMEOUT);
  if (ret) {
    LOG_ERR("Failed to send unwatch motor command");
    return ret;
  }

  temp_pid_enabled = false;
  LOG_INF("Temperature PID control disabled on thermometer %d", temp_controller_index);
  return ret;
}

//////////////////////////////////////////////////////////////
// Thread runner
//////////////////////////////////////////////////////////////

int helios_state_runner(void)
{
  int res = 0;
  k_mutex_init(&state_machine_mutex);
  smf_set_initial(SMF_CTX(&helios_state_ctx),
      &helios_state_machine[HELIOS_INITIALIZING]);

  while (res == 0) {
    k_mutex_lock(&state_machine_mutex, MUTEX_WAIT);
    res = smf_run_state(SMF_CTX(&helios_state_ctx));
    report_state();
    k_mutex_unlock(&state_machine_mutex);
    k_sleep(LOOP_SLEEP);
  }

  return res;
}

//////////////////////////////////////////////////////////////
// State machine functions
//////////////////////////////////////////////////////////////

static enum smf_state_result initialize_helios(void* o)
{
  const enum smf_state_result res = SMF_EVENT_HANDLED;
  LOG_DBG_RATELIMIT("Initializing helios state");

  if (temperature == 0.0) {
    LOG_DBG_RATELIMIT("Helios initialization waiting for temperature");
    return res;
  }

  if (!pump_ready) {
    LOG_DBG_RATELIMIT("Helios initialization waiting for pump to be ready");
    return res;
  }

  if (temperature >= FAULT_TEMP) {
    LOG_ERR("FATAL: helios temperature in fault mode");
    zbus_publish_state(HELIOS_INITIALIZING, 1, true);
    return res;
  }

  zbus_publish_state(HELIOS_INITIALIZING, 1, false);
  smf_set_state(SMF_CTX(&helios_state_ctx), &helios_state_machine[HELIOS_IDLE]);
  LOG_DBG("Helios initialized");
  return res;
}

static enum smf_state_result idle(void* o)
{
  current_state = HELIOS_IDLE;
  return SMF_EVENT_HANDLED;
}

static enum smf_state_result blowing(void* o)
{
  current_state = HELIOS_BLOWING;
  return SMF_EVENT_HANDLED;
}

static void stop_blowing_helios(void* o) { set_rpm(0); }

static enum smf_state_result preheat_helios(void* o)
{
  current_state = HELIOS_PREHEAT;
  const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

  if (preheat_failed()) {
    smf_set_state(SMF_CTX(&helios_state_ctx), &helios_state_machine[HELIOS_ERROR]);
    return SMF_EVENT_HANDLED;
  }

  if (!rpm_on_target()) {
    LOG_DBG_RATELIMIT("preheat waiting for RPMs to be on target");
    return SMF_EVENT_HANDLED;
  }

  if (!glowing && glow_lit_at == 0) {
    LOG_DBG("lighting glow plug");
    start_glow_burn();
  }

  if (glowing && current_micros >= glow_lit_at + PREHEAT_PUMP_DELAY) {
    set_pump_rate(PREHEAT_PUMP_RATE);
  }

  if (temperature >= PREHEAT_STAGE_2_TEMP) {
    LOG_DBG("preheat stage 1 complete");
    smf_set_state(SMF_CTX(&helios_state_ctx), &helios_state_machine[HELIOS_PREHEAT_STAGE_2]);
  }

  return SMF_EVENT_HANDLED;
}

static void start_preheat(void* o)
{
  LOG_DBG("starting preheat stage 1");
  preheat_started_at = k_cyc_to_us_floor64(k_cycle_get_64());
  set_rpm(PREHEAT_RPM);
}

static void end_preheat(void* o)
{
  LOG_DBG("ending preheat");
  preheat_started_at = 0;
  glow_lit_at = 0;
}

static enum smf_state_result preheat_stage_2(void* o)
{
  current_state = HELIOS_PREHEAT_STAGE_2;

  // Track that we've reached high temperature
  if (temperature >= COOLDOWN_THRESHOLD_TEMP) {
    reached_high_temp = true;
  }

  if (preheat_failed()) {
    LOG_DBG("stage 2 preheat failed");
    smf_set_state(SMF_CTX(&helios_state_ctx), &helios_state_machine[HELIOS_ERROR]);
    return SMF_EVENT_HANDLED;
  }

  if (glowing && temperature >= PREHEAT_SUCCESS_TEMP) {
    LOG_DBG("stage 2 preheat complete");
    smf_set_state(SMF_CTX(&helios_state_ctx), &helios_state_machine[HELIOS_HEATING]);
    stop_glow_burn();
  }

  set_pump_rate(PREHEAT_STAGE_2_PUMP_RATE);
  set_rpm(PREHEAT_STAGE_2_RPM);
  return SMF_EVENT_HANDLED;
}

static enum smf_state_result heating_helios(void* o)
{
  current_state = HELIOS_HEATING;

  // Track that we've reached high temperature
  if (temperature >= COOLDOWN_THRESHOLD_TEMP) {
    reached_high_temp = true;
  }

  set_pump_rate(user_pump_rate_target);
  return SMF_EVENT_HANDLED;
}

static enum smf_state_result cooldown_helios(void* o)
{
  current_state = HELIOS_COOLING;

  // Check if cooldown is complete
  if (temperature <= COOLDOWN_COMPLETE_TEMP) {
    LOG_INF("Cooldown complete at %.2f°C, transitioning to idle", temperature);
    reached_high_temp = false;
    smf_set_state(SMF_CTX(&helios_state_ctx), &helios_state_machine[HELIOS_IDLE]);
    return SMF_EVENT_HANDLED;
  }

  // Start glow plug when temperature drops to threshold
  if (temperature <= COOLDOWN_GLOW_START_TEMP) {
    if (!glowing) {
      LOG_INF("Starting glow plug for cooldown at %.2f°C", temperature);
      start_glow_burn();
    }
    // Maintain fan speed during glow-assisted cooldown
    set_rpm(COOLDOWN_FAN_RPM);
  } else {
    // Above glow start temp, just run fan
    set_rpm(COOLDOWN_FAN_RPM);
    if (glowing) {
      LOG_DBG("Extinguishing glow plug - temperature above cooldown glow threshold");
      stop_glow_burn();
    }
  }

  // Turn off pump during cooldown
  set_pump_rate(0);

  LOG_DBG_RATELIMIT("Cooldown in progress: %.2f°C (target: %.2f°C)",
      temperature, COOLDOWN_COMPLETE_TEMP);

  return SMF_EVENT_HANDLED;
}

static void idle_entry(void* o)
{
  LOG_DBG("helios entering idle state");
  set_pump_rate(0);
  set_rpm(0);
  stop_glow_burn();
}

static void heating_entry(void* o)
{
  LOG_INF("Helios entering heating state - enabling temperature PID control");
  enable_temp_pid_control();
}

static void heating_exit(void* o)
{
  LOG_INF("Helios exiting heating state - disabling temperature PID control");
  disable_temp_pid_control();
}

static void cooldown_entry(void* o)
{
  LOG_INF("Helios entering cooldown state - temperature: %.2f°C", temperature);
  // Stop pump and disable temperature PID control
  set_pump_rate(0);
  disable_temp_pid_control();
}

static void cooldown_exit(void* o)
{
  LOG_INF("Helios exiting cooldown state");
  // Ensure everything is stopped
  stop_glow_burn();
  set_rpm(0);
  set_pump_rate(0);
}

//////////////////////////////////////////////////////////////
// State machine definition
//////////////////////////////////////////////////////////////

static const struct smf_state helios_state_machine[] = {
  [HELIOS_INITIALIZING] = SMF_CREATE_STATE(NULL, initialize_helios, NULL, NULL, NULL),
  [HELIOS_IDLE] = SMF_CREATE_STATE(idle_entry, idle, NULL, NULL, NULL),
  [HELIOS_BLOWING] = SMF_CREATE_STATE(NULL, blowing, stop_blowing_helios, NULL, NULL),
  [HELIOS_PREHEAT] = SMF_CREATE_STATE(start_preheat, preheat_helios,
      end_preheat, NULL, NULL),
  [HELIOS_PREHEAT_STAGE_2] = SMF_CREATE_STATE(NULL, preheat_stage_2,
      NULL, NULL, NULL),
  [HELIOS_HEATING] = SMF_CREATE_STATE(heating_entry, heating_helios,
      heating_exit, NULL, NULL),
  [HELIOS_COOLING] = SMF_CREATE_STATE(cooldown_entry, cooldown_helios,
      cooldown_exit, NULL, NULL),
  [HELIOS_ERROR] = SMF_CREATE_STATE(NULL, NULL, NULL, NULL, NULL),
};

//////////////////////////////////////////////////////////////
// Zbus
//////////////////////////////////////////////////////////////

static void state_machine_data_callback(const struct zbus_channel* chan)
{
  // TODO: Support multiple inputs from each channel.
  k_mutex_lock(&state_machine_mutex, MUTEX_WAIT);

  if (&motor_data_chan == chan) {
    const struct motor_data_msg* msg = zbus_chan_const_msg(chan);
    current_rpm = msg->rpm;
    max_rpm = msg->max_rpm;
    min_rpm = msg->min_rpm;
    target_rpm = msg->target;

  } else if (&glow_data_chan == chan) {
    const struct glow_data_msg* msg = zbus_chan_const_msg(chan);
    glowing = msg->lit;
    if (glowing && glow_lit_at == 0) {
      glow_lit_at = msg->timestamp;
    }

  } else if (&pump_data_chan == chan) {
    const struct pump_data_msg* msg = zbus_chan_const_msg(chan);
    current_pump_rate = msg->rate;

    if (msg->type == PUMP_READY) {
      pump_ready = true;
    }

    if (msg->type == PUMP_CYCLE_START) {
      last_pump_at = msg->timestamp;
    }

  } else if (&temperature_data_chan == chan) {
    const struct temperature_data_msg* msg = zbus_chan_const_msg(chan);
    temperature = msg->temperature;

  } else {
    LOG_WRN_RATELIMIT("state machine got unhandled data over zbus channel %s",
        chan->name);
  }

  k_mutex_unlock(&state_machine_mutex);
}

static bool state_command_validator(const void* msg, size_t msg_size)
{
  const struct state_command_msg* cmd = msg;
  LOG_DBG("got state command %s - %d", helios_mode_names[cmd->mode], cmd->argument);
  if (cmd->mode == HELIOS_FAN_MODE) {
    if (cmd->argument != 0 && (cmd->argument > max_rpm || cmd->argument < min_rpm)) {
      LOG_DBG("can't set fan RPM, value too high or too low");
      return false;
    }
  }

  if (cmd->mode == HELIOS_HEAT_MODE && current_state != HELIOS_IDLE) {
    LOG_DBG("can't start heat mode, helios is not idle");
    return false;
  }

  return true;
}

static void state_command_callback(const struct zbus_channel* chan)
{
  const struct state_command_msg* msg = zbus_chan_const_msg(chan);
  k_mutex_lock(&state_machine_mutex, MUTEX_WAIT);

  if (msg->mode == HELIOS_FAN_MODE) {
    if (msg->argument == 0) {
      smf_set_state(SMF_CTX(&helios_state_ctx),
          &helios_state_machine[HELIOS_IDLE]);
    } else {
      if (current_state != HELIOS_BLOWING) {
        smf_set_state(SMF_CTX(&helios_state_ctx),
            &helios_state_machine[HELIOS_BLOWING]);
      }
      set_rpm(msg->argument);
    }
  }

  if (msg->mode == HELIOS_IDLE_MODE) {
    // Check if cooldown is required before going to idle
    if (reached_high_temp && temperature > COOLDOWN_COMPLETE_TEMP) {
      LOG_INF("Temperature is %.2f°C - initiating cooldown procedure", temperature);
      smf_set_state(SMF_CTX(&helios_state_ctx),
          &helios_state_machine[HELIOS_COOLING]);
    } else {
      smf_set_state(SMF_CTX(&helios_state_ctx),
          &helios_state_machine[HELIOS_IDLE]);
    }
  }

  if (msg->mode == HELIOS_HEAT_MODE) {
    user_pump_rate_target = msg->argument;
    smf_set_state(SMF_CTX(&helios_state_ctx),
        &helios_state_machine[HELIOS_PREHEAT]);
  }

  k_mutex_unlock(&state_machine_mutex);
}

ZBUS_LISTENER_DEFINE(state_data_listener, state_machine_data_callback);
ZBUS_LISTENER_DEFINE(state_command_listener, state_command_callback);

ZBUS_CHAN_DEFINE(state_command_chan, /* Name */
    struct state_command_msg, /* Message type */
    state_command_validator, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(state_command_listener), /* Observers */
    ZBUS_MSG_INIT(.mode = HELIOS_IDLE_MODE,
        .argument = 0) /* Initial value */
);

ZBUS_CHAN_DEFINE(state_data_chan, /* Name */
    struct state_data_msg, /* Message type */
    NULL, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS_EMPTY, /* Observers */
    ZBUS_MSG_INIT(.code = 0, .error = false,
        .state = HELIOS_INITIALIZING,
        .timestamp = 0) /* Initial value */
);
