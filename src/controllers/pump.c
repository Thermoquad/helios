// SPDX-License-Identifier: GPL-2.0-or-later

#include <stdbool.h>
#include <sys/_intsup.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <helios/zbus.h>

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(pump_controller);

#define LOOP_SLEEP K_USEC(500U)
#define MUTEX_WAIT K_MSEC(10U)
#define DEFAULT_PULSE_MS 50
#define DEFAULT_RECOVERY_MS 50
#define PUB_TIMEOUT K_MSEC(1U)

//////////////////////////////////////////////////////////////
// Hardware setup
//////////////////////////////////////////////////////////////

#define PUMP_ALIASES(i) DT_ALIAS(_CONCAT(helios_pump, i))
#define PUMPS(i, _)                           \
  IF_ENABLED(DT_NODE_EXISTS(PUMP_ALIASES(i)), \
      (GPIO_DT_SPEC_GET(PUMP_ALIASES(i), gpios), ))

static struct gpio_dt_spec pump_pins[] = { LISTIFY(10, PUMPS, ()) };

//////////////////////////////////////////////////////////////
// Pump struct
//////////////////////////////////////////////////////////////

struct pump_state {
  int index;
  int rate_ms;
  struct gpio_dt_spec gpio;
  unsigned cycle_start;
  int pulse_ms;
  int recovery_ms;
};

//////////////////////////////////////////////////////////////
// State variables
//////////////////////////////////////////////////////////////

K_MUTEX_DEFINE(pump_mutex);
static struct pump_state pumps[ARRAY_SIZE(pump_pins)];

//////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////

int publish_pump_data(struct pump_state* pump, unsigned current_micros, enum pump_data_msg_type type)
{
  struct pump_data_msg msg = {
    .pump = pump->index, .type = type, .rate = pump->rate_ms, .timestamp = current_micros
  };
  return zbus_chan_pub(&pump_data_chan, &msg, PUB_TIMEOUT);
}

void publish_pump_error(struct pump_state* pump, unsigned current_micros)
{
  publish_pump_data(pump, current_micros, PUMP_ERROR);
}

void publish_pump_ready(struct pump_state* pump, unsigned current_micros)
{
  publish_pump_data(pump, current_micros, PUMP_READY);
}

void start_pump_cycle(struct pump_state* pump, unsigned current_micros)
{
  pump->cycle_start = current_micros;
  gpio_pin_set_dt(&pump->gpio, 1);
  publish_pump_data(pump, current_micros, PUMP_CYCLE_START);
}

void end_pump_pulse(struct pump_state* pump, unsigned current_micros)
{
  gpio_pin_set_dt(&pump->gpio, 0);
  publish_pump_data(pump, current_micros, PUMP_PULSE_END);
}

void end_pump_cycle(struct pump_state* pump, unsigned current_micros)
{
  pump->cycle_start = 0;
  publish_pump_data(pump, current_micros, PUMP_CYCLE_END);
}

//////////////////////////////////////////////////////////////
// Controller thread functions
//////////////////////////////////////////////////////////////

int initialize_pumps()
{
  for (int i = 0; i < ARRAY_SIZE(pumps); i++) {
    int ret;

    struct pump_state pump;
    pump.index = i;
    pump.gpio = pump_pins[i];
    pump.rate_ms = 0;
    pump.cycle_start = 0;
    pump.pulse_ms = DEFAULT_PULSE_MS;
    pump.recovery_ms = DEFAULT_RECOVERY_MS;
    pumps[i] = pump;
    const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

    if (!gpio_is_ready_dt(&pump.gpio)) {
      LOG_ERR("Error: device %s is not ready\n", pump.gpio.port->name);
      publish_pump_error(&pump, current_micros);
      return 1;
    }

    ret = gpio_pin_configure_dt(&pump.gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
      LOG_ERR("Error %d: failed to configure %s pin %d\n", ret,
          pump.gpio.port->name, pump.gpio.pin);
      publish_pump_error(&pump, current_micros);
      return ret;
    }

    publish_pump_ready(&pump, current_micros);
  }

  return 0;
}

int pump_controller(void)
{
  int ret = 0;
  k_mutex_init(&pump_mutex);
  ret = initialize_pumps();
  if (ret != 0) {
    return ret;
  }

  while (true) {
    for (int i = 0; i < ARRAY_SIZE(pumps); i++) {
      k_mutex_lock(&pump_mutex, MUTEX_WAIT);
      const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());
      struct pump_state* pump = &pumps[i];
      const unsigned pulse_micros = pump->pulse_ms * 1000;
      const unsigned rate_micros = pump->rate_ms * 1000;
      const bool cycle_at_zero = pump->cycle_start == 0;

      if (cycle_at_zero && pump->rate_ms != 0) {
        start_pump_cycle(pump, current_micros);
      }

      if (!cycle_at_zero) {
        if (current_micros >= pump->cycle_start + pulse_micros) {
          end_pump_pulse(pump, current_micros);
        }

        if (rate_micros != 0 && current_micros >= pump->cycle_start + rate_micros) {
          end_pump_cycle(pump, current_micros);
        }
      }

      k_mutex_unlock(&pump_mutex);
    }
    k_sleep(LOOP_SLEEP);
  }

  return ret;
}

//////////////////////////////////////////////////////////////
// Zbus
//////////////////////////////////////////////////////////////

bool pump_command_validator(const void* msg, size_t msg_size)
{
  const struct pump_command_msg* cmd = msg;
  if (cmd->pump < 0 || cmd->pump > ARRAY_SIZE(pumps) - 1) {
    return false;
  }

  k_mutex_lock(&pump_mutex, MUTEX_WAIT);
  const struct pump_state* pump = &pumps[cmd->pump];
  const int pulse_ms = pump->pulse_ms;
  const int recovery_ms = pump->recovery_ms;
  k_mutex_unlock(&pump_mutex);

  if (cmd->rate_ms != 0 && cmd->rate_ms < pulse_ms + recovery_ms) {
    return false;
  }

  return true;
}

void pump_command_callback(const struct zbus_channel* chan)
{
  const struct pump_command_msg* msg = zbus_chan_const_msg(chan);
  LOG_DBG("Got command for pump %d, rate %d", msg->pump, msg->rate_ms);

  k_mutex_lock(&pump_mutex, MUTEX_WAIT);
  struct pump_state* pump = &pumps[msg->pump];
  pump->rate_ms = msg->rate_ms;
  k_mutex_unlock(&pump_mutex);
}

ZBUS_LISTENER_DEFINE(pump_controller_listener, pump_command_callback);

bool pump_config_validator(const void* msg, size_t msg_size)
{
  const struct pump_config_msg* cfg = msg;
  if (cfg->pump > ARRAY_SIZE(pumps) - 1) {
    LOG_ERR("Invalid pump index: %d", cfg->pump);
    return false;
  }
  return true;
}

void pump_config_callback(const struct zbus_channel* chan)
{
  const struct pump_config_msg* cfg = zbus_chan_const_msg(chan);
  LOG_DBG("Got config for pump %d", cfg->pump);

  k_mutex_lock(&pump_mutex, MUTEX_WAIT);
  struct pump_state* pump = &pumps[cfg->pump];

  if (cfg->pulse_ms_present && cfg->pulse_ms > 0) {
    pump->pulse_ms = cfg->pulse_ms;
    LOG_INF("Pump %d: pulse_ms set to %u", cfg->pump, cfg->pulse_ms);
  }

  if (cfg->recovery_ms_present) {
    pump->recovery_ms = cfg->recovery_ms;
    LOG_INF("Pump %d: recovery_ms set to %u", cfg->pump, cfg->recovery_ms);
  }

  k_mutex_unlock(&pump_mutex);
}

ZBUS_LISTENER_DEFINE(pump_config_listener, pump_config_callback);

ZBUS_CHAN_DEFINE(pump_config_chan, /* Name */
    struct pump_config_msg, /* Message type */
    pump_config_validator, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(pump_config_listener), /* Observers */
    ZBUS_MSG_INIT(.pump = 0) /* Initial value */
);

ZBUS_CHAN_DEFINE(pump_command_chan, /* Name */
    struct pump_command_msg, /* Message type */
    pump_command_validator, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(pump_controller_listener), /* Observers */
    ZBUS_MSG_INIT(.pump = 0, .rate_ms = 0) /* Initial value */
);

ZBUS_CHAN_DEFINE(pump_data_chan, /* Name */
    struct pump_data_msg, /* Message type */
    NULL, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(state_data_listener), /* Observers */
    ZBUS_MSG_INIT(.pump = 0, .type = PUMP_INITIALIZING, .rate = 0,
        .timestamp = 0) /* Initial value */
);
