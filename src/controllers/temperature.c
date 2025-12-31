#include <stdbool.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <helios/pid.h>
#include <helios/zbus.h>

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(temperature_controller);

#define LOOP_SLEEP K_MSEC(10U)
#define PUB_TIMEOUT K_MSEC(10U)
#define MUTEX_WAIT K_MSEC(100U)
#define READ_RATE_MS 50
#define ANNOUNCE_RATE_MS 100

#define SAMPLE_COUNT 60

// Default PID gains for temperature->RPM control
#define DEFAULT_TEMP_KP 100.0
#define DEFAULT_TEMP_KI 10.0
#define DEFAULT_TEMP_KD 5.0

//////////////////////////////////////////////////////////////
// Hardware setup
//////////////////////////////////////////////////////////////

#define TEMP_ALIASES(i) DT_ALIAS(_CONCAT(helios_temp, i))
#define TEMPS(i, _)                           \
  IF_ENABLED(DT_NODE_EXISTS(TEMP_ALIASES(i)), \
      (DEVICE_DT_GET(TEMP_ALIASES(i)), ))
static const struct device* thermometers[] = { LISTIFY(10, TEMPS, ()) };

//////////////////////////////////////////////////////////////
// State struct
//////////////////////////////////////////////////////////////

struct temperature_state {
  int index;
  unsigned last_announce;
  unsigned last_read;

  const struct device* thermometer;
  double current_temperature;
  int sample_itterator;
  double samples[SAMPLE_COUNT];
  bool samples_ready;

  bool pid_enabled;
  struct pid_controller pid;

  // Motor RPM control fields
  int watched_motor_index;
  int current_motor_rpm;
  int motor_min_rpm;
  int motor_max_rpm;
  bool motor_rpm_control_enabled;
  double target_temperature;
};

//////////////////////////////////////////////////////////////
// State variables
//////////////////////////////////////////////////////////////

K_MUTEX_DEFINE(temperature_mutex);
static struct temperature_state temperature_states[ARRAY_SIZE(thermometers)];

//////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////

static int read_temperature(struct temperature_state* state, unsigned current_micros)
{
  int ret = 0;
  if (current_micros < state->last_read + READ_RATE_MS * 1e3) {
    return ret;
  }

  struct sensor_value pdata;
  ret = sensor_sample_fetch(state->thermometer);
  if (ret) {
    LOG_ERR_RATELIMIT("Failed to fetch sample for thermometer %d", state->index);
    return ret;
  }
  ret = sensor_channel_get(state->thermometer, SENSOR_CHAN_AMBIENT_TEMP, &pdata);
  if (ret) {
    LOG_ERR_RATELIMIT("Failed to get reading for thermometer %d", state->index);
    return ret;
  }

  state->last_read = current_micros;
  state->samples[state->sample_itterator] = sensor_value_to_double(&pdata);
  state->sample_itterator++;
  if (state->sample_itterator > SAMPLE_COUNT - 1) {
    state->sample_itterator = 0;
    state->samples_ready = true;
  }

  if (!state->samples_ready) {
    return ret;
  }

  double sum = 0.0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += state->samples[i];
  }
  state->current_temperature = sum / SAMPLE_COUNT;

  LOG_DBG_RATELIMIT("read temperature for temp %d, %f", state->index,
      state->current_temperature);
  return ret;
}

static void announce_temperature(struct temperature_state* state, unsigned current_micros)
{
  // Rate limit announcements
  if (!state->samples_ready || current_micros < state->last_announce + ANNOUNCE_RATE_MS * 1e3) {
    return;
  }

  // Publish temperature reading with PID control status
  struct temperature_data_msg msg = {
    .thermometer = state->index,
    .temperature = state->current_temperature,
    .timestamp = current_micros,
    .pid_enabled = state->pid_enabled,
    .rpm_control_enabled = state->motor_rpm_control_enabled,
    .watched_motor = state->watched_motor_index,
    .target_temperature = state->target_temperature
  };

  zbus_chan_pub(&temperature_data_chan, &msg, PUB_TIMEOUT);
  state->last_announce = current_micros;
  LOG_DBG_RATELIMIT("announcing temperature for temp %d, %f", state->index,
      state->current_temperature);
}

/**
 * Temperature-based PID control of motor RPM
 * Uses inverted PID: higher temperature -> higher RPM to increase cooling
 */
static void pid_control(struct temperature_state* state, unsigned current_micros)
{
  // Validate PID control is enabled and configured
  if (!state->samples_ready || !state->pid_enabled || !state->motor_rpm_control_enabled) {
    return;
  }

  if (state->watched_motor_index < 0) {
    LOG_WRN_RATELIMIT("PID enabled but no motor being watched");
    return;
  }

  // Ensure motor RPM limits have been received from motor data before running PID
  if (state->motor_min_rpm == 0 || state->motor_max_rpm == 0) {
    LOG_WRN_RATELIMIT("PID enabled but motor RPM limits not yet received");
    return;
  }

  // Set PID parameters from current and target temperatures
  state->pid.input = state->current_temperature;
  state->pid.target = state->target_temperature;

  // Run PID cycle - output will be target RPM for motor
  run_pid_cycle(&state->pid, current_micros);

  // Publish motor command with calculated target RPM
  int target_rpm = (int)state->pid.output;
  struct motor_command_msg motor_cmd = {
    .motor = state->watched_motor_index,
    .rpm = target_rpm
  };

  int ret = zbus_chan_pub(&motor_command_chan, &motor_cmd, PUB_TIMEOUT);
  if (ret) {
    LOG_ERR_RATELIMIT("Failed to publish motor command for temp controller %d", state->index);
  } else {
    LOG_DBG_RATELIMIT("Temp controller %d: temp=%.2f target=%.2f -> motor RPM=%d",
        state->index, state->current_temperature, state->target_temperature, target_rpm);
  }
}

//////////////////////////////////////////////////////////////
// Controller thread functions
//////////////////////////////////////////////////////////////

static int initialize_temp_controllers()
{
  for (int i = 0; i < ARRAY_SIZE(temperature_states); i++) {
    struct temperature_state controller;
    controller.index = i;
    controller.thermometer = thermometers[i];
    controller.last_announce = 0;
    controller.last_read = 0;
    controller.sample_itterator = 0;
    controller.samples_ready = false;

    struct pid_controller pid;
    pid.inverted = true;
    pid.time_divisor = 1.0e6;
    pid.p_gain = DEFAULT_TEMP_KP;
    pid.i_gain = DEFAULT_TEMP_KI;
    pid.d_gain = DEFAULT_TEMP_KD;
    pid.output_min_limit = 0.0;
    pid.output_max_limit = 0.0;
    reset_pid(&pid);
    controller.pid_enabled = false;
    controller.pid = pid;

    // Initialize motor RPM control fields
    controller.watched_motor_index = -1;
    controller.current_motor_rpm = 0;
    controller.motor_min_rpm = 0; // Will be set from motor data
    controller.motor_max_rpm = 0; // Will be set from motor data
    controller.motor_rpm_control_enabled = false;
    controller.target_temperature = 0.0;

    temperature_states[i] = controller;
  }
  return 0;
}

int temperature_controller(void)
{
  int ret = 0;
  k_mutex_init(&temperature_mutex);
  ret = initialize_temp_controllers();

  while (ret == 0) {
    k_mutex_lock(&temperature_mutex, MUTEX_WAIT);
    for (int i = 0; i < ARRAY_SIZE(temperature_states); i++) {
      const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());
      struct temperature_state* state = &temperature_states[i];
      ret = read_temperature(state, current_micros);
      pid_control(state, current_micros);
      announce_temperature(state, current_micros);
    }
    k_mutex_unlock(&temperature_mutex);
    k_sleep(LOOP_SLEEP);
  }
  return ret;
}

//////////////////////////////////////////////////////////////
// Zbus callbacks
//////////////////////////////////////////////////////////////

/**
 * Motor data listener callback
 * Receives motor RPM updates and dynamically adjusts PID output limits
 */
void motor_data_callback(const struct zbus_channel* chan)
{
  const struct motor_data_msg* motor_data = zbus_chan_const_msg(chan);
  LOG_DBG("Temperature controller received motor data for motor %d, RPM %d",
      motor_data->motor, motor_data->rpm);

  k_mutex_lock(&temperature_mutex, MUTEX_WAIT);

  // Update any temperature controllers watching this motor
  for (int i = 0; i < ARRAY_SIZE(temperature_states); i++) {
    struct temperature_state* state = &temperature_states[i];
    if (state->watched_motor_index == motor_data->motor) {
      // Update current RPM reading
      state->current_motor_rpm = motor_data->rpm;

      // Update motor min/max RPM limits from motor data
      bool limits_changed = false;
      if (state->motor_min_rpm != motor_data->min_rpm) {
        state->motor_min_rpm = motor_data->min_rpm;
        limits_changed = true;
      }
      if (state->motor_max_rpm != motor_data->max_rpm) {
        state->motor_max_rpm = motor_data->max_rpm;
        limits_changed = true;
      }

      // Update PID output limits to match motor capabilities
      if (limits_changed) {
        state->pid.output_min_limit = state->motor_min_rpm;
        state->pid.output_max_limit = state->motor_max_rpm;
        LOG_DBG("Updated temp controller %d PID limits: %d - %d RPM",
            i, state->motor_min_rpm, state->motor_max_rpm);
      }

      LOG_DBG("Updated temp controller %d with motor %d RPM: %d",
          i, motor_data->motor, motor_data->rpm);
    }
  }
  k_mutex_unlock(&temperature_mutex);
}

bool temperature_command_validator(const void* msg, size_t msg_size)
{
  const struct temperature_command_msg* cmd = msg;
  if (cmd->thermometer < 0 || cmd->thermometer > ARRAY_SIZE(temperature_states) - 1) {
    LOG_ERR("Invalid thermometer index: %d", cmd->thermometer);
    return false;
  }

  // Validate motor index for WATCH_MOTOR command
  if (cmd->type == TEMP_CMD_WATCH_MOTOR) {
    // Note: We can't easily validate motor count here, but the command will be ignored
    // if the motor doesn't exist
    if (cmd->motor_index < 0) {
      LOG_ERR("Invalid motor index: %d", cmd->motor_index);
      return false;
    }
  }

  return true;
}

/**
 * Temperature command listener callback
 * Handles commands to configure motor RPM monitoring and PID control
 */
void temperature_command_callback(const struct zbus_channel* chan)
{
  const struct temperature_command_msg* cmd = zbus_chan_const_msg(chan);
  LOG_DBG("Got temperature command for thermometer %d, type %d",
      cmd->thermometer, cmd->type);

  k_mutex_lock(&temperature_mutex, MUTEX_WAIT);
  struct temperature_state* state = &temperature_states[cmd->thermometer];

  switch (cmd->type) {
  case TEMP_CMD_WATCH_MOTOR:
    // Associate this temperature controller with a motor to monitor
    state->watched_motor_index = cmd->motor_index;
    state->current_motor_rpm = 0;
    LOG_INF("Temp controller %d now watching motor %d",
        cmd->thermometer, cmd->motor_index);
    break;

  case TEMP_CMD_UNWATCH_MOTOR:
    // Stop monitoring motor and disable RPM control
    LOG_INF("Temp controller %d stopped watching motor %d",
        cmd->thermometer, state->watched_motor_index);
    state->watched_motor_index = -1;
    state->current_motor_rpm = 0;
    state->motor_rpm_control_enabled = false;
    break;

  case TEMP_CMD_ENABLE_RPM_CONTROL:
    // Enable inverted PID control of motor RPM based on temperature
    if (state->watched_motor_index < 0) {
      LOG_WRN("Cannot enable RPM control: no motor being watched");
      break;
    }
    state->motor_rpm_control_enabled = true;
    state->pid_enabled = true;
    reset_pid(&state->pid);
    LOG_INF("Temp controller %d enabled RPM control for motor %d",
        cmd->thermometer, state->watched_motor_index);
    break;

  case TEMP_CMD_DISABLE_RPM_CONTROL:
    // Disable PID control of motor RPM
    state->motor_rpm_control_enabled = false;
    state->pid_enabled = false;
    LOG_INF("Temp controller %d disabled RPM control", cmd->thermometer);
    break;

  case TEMP_CMD_SET_TARGET_TEMP:
    // Set target temperature for PID controller
    state->target_temperature = cmd->target_temperature;
    LOG_INF("Temp controller %d target temperature set to %.2f",
        cmd->thermometer, cmd->target_temperature);
    break;

  default:
    LOG_ERR("Unknown temperature command type: %d", cmd->type);
    break;
  }

  k_mutex_unlock(&temperature_mutex);
}

ZBUS_LISTENER_DEFINE(temperature_command_listener, temperature_command_callback);
ZBUS_LISTENER_DEFINE(temperature_motor_data_listener, motor_data_callback);

//////////////////////////////////////////////////////////////
// Zbus channels
//////////////////////////////////////////////////////////////

ZBUS_CHAN_DEFINE(temperature_command_chan, /* Name */
    struct temperature_command_msg, /* Message type */
    temperature_command_validator, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(temperature_command_listener), /* Observers */
    ZBUS_MSG_INIT(.thermometer = 0, .type = TEMP_CMD_WATCH_MOTOR,
        .motor_index = 0, .target_temperature = 0.0) /* Initial value */
);

ZBUS_CHAN_DEFINE(temperature_data_chan, /* Name */
    struct temperature_data_msg, /* Message type */
    NULL, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(state_data_listener), /* Observers */
    ZBUS_MSG_INIT(.thermometer = 0, .timestamp = 0, .temperature = 0.0,
        .pid_enabled = false, .rpm_control_enabled = false,
        .watched_motor = -1, .target_temperature = 0.0) /* Initial value */
);
