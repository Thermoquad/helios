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
  if (!state->samples_ready || current_micros < state->last_announce + ANNOUNCE_RATE_MS * 1e3) {
    return;
  }

  struct temperature_data_msg msg = { .thermometer = state->index,
    .temperature = state->current_temperature,
    .timestamp = current_micros };

  zbus_chan_pub(&temperature_data_chan, &msg, PUB_TIMEOUT);
  state->last_announce = current_micros;
  LOG_DBG_RATELIMIT("announcing temperature for temp %d, %f", state->index,
      state->current_temperature);
}

static void pid_control(struct temperature_state* state, unsigned current_micros)
{
  if (!state->samples_ready || !state->pid_enabled) {
    return;
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
    reset_pid(&pid);
    controller.pid_enabled = false;
    controller.pid = pid;

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
// Zbus
//////////////////////////////////////////////////////////////

ZBUS_CHAN_DEFINE(temperature_data_chan, /* Name */
    struct temperature_data_msg, /* Message type */
    NULL, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(state_data_listener), /* Observers */
    ZBUS_MSG_INIT(.thermometer = 0, .timestamp = 0,
        .temperature = 0.0) /* Initial value */
);
