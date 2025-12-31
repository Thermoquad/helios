#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <helios/zbus.h>

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(glow_controller);

#define LOOP_SLEEP K_USEC(500U)
#define MUTEX_WAIT K_MSEC(5U)
#define DEFAULT_MAX_DURATION 5 * 60 * 1000
#define PUB_TIMEOUT K_MSEC(1U)

//////////////////////////////////////////////////////////////
// Hardware setup
//////////////////////////////////////////////////////////////

#define GLOW_ALIASES(i) DT_ALIAS(_CONCAT(helios_glow, i))
#define GLOWS(i, _)                           \
  IF_ENABLED(DT_NODE_EXISTS(GLOW_ALIASES(i)), \
      (GPIO_DT_SPEC_GET(GLOW_ALIASES(i), gpios), ))
static struct gpio_dt_spec glow_pins[] = { LISTIFY(10, GLOWS, ()) };

//////////////////////////////////////////////////////////////
// Glow struct
//////////////////////////////////////////////////////////////

struct glow_state {
  int index;
  struct gpio_dt_spec gpio;
  unsigned cycle_start;
  int duration;
  int max_duration;
};

//////////////////////////////////////////////////////////////
// State variables
//////////////////////////////////////////////////////////////

K_MUTEX_DEFINE(glow_mutex);
static struct glow_state glows[ARRAY_SIZE(glow_pins)];

//////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////

void light_glow(struct glow_state* glow, unsigned current_micros)
{
  glow->cycle_start = current_micros;
  gpio_pin_set_dt(&glow->gpio, 1);

  struct glow_data_msg msg = {
    .glow = glow->index, .lit = true, .timestamp = current_micros
  };
  zbus_chan_pub(&glow_data_chan, &msg, PUB_TIMEOUT);

  LOG_DBG("lit glow %d", glow->index);
}

void extinguish_glow(struct glow_state* glow, unsigned current_micros)
{
  glow->duration = 0;
  glow->cycle_start = 0;
  gpio_pin_set_dt(&glow->gpio, 0);

  struct glow_data_msg msg = {
    .glow = glow->index, .lit = false, .timestamp = current_micros
  };
  zbus_chan_pub(&glow_data_chan, &msg, PUB_TIMEOUT);

  LOG_DBG("extinguished glow %d", glow->index);
}

//////////////////////////////////////////////////////////////
// Controller thread functions
//////////////////////////////////////////////////////////////

int initialize_glows()
{
  for (int i = 0; i < ARRAY_SIZE(glows); i++) {
    int ret;

    struct glow_state glow;
    glow.index = i;
    glow.gpio = glow_pins[i];
    glow.cycle_start = 0;
    glow.duration = 0;
    glow.max_duration = DEFAULT_MAX_DURATION;
    glows[i] = glow;

    if (!gpio_is_ready_dt(&glow.gpio)) {
      LOG_ERR("Error: device %s is not ready\n", glow.gpio.port->name);
      return 1;
    }
    ret = gpio_pin_configure_dt(&glow.gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
      LOG_ERR("Error %d: failed to configure %s pin %d\n", ret,
          glow.gpio.port->name, glow.gpio.pin);
      return ret;
    }
  }
  return 0;
}

int glow_controller(void)
{
  int ret = 0;
  k_mutex_init(&glow_mutex);
  ret = initialize_glows();
  if (ret != 0) {
    return ret;
  }
  while (true) {
    for (int i = 0; i < ARRAY_SIZE(glows); i++) {
      k_mutex_lock(&glow_mutex, MUTEX_WAIT);
      const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());
      struct glow_state* glow = &glows[i];
      const unsigned lit_micros = glow->duration * 1000;

      if (glow->cycle_start == 0 && glow->duration != 0) {
        light_glow(glow, current_micros);
      }

      if (glow->cycle_start != 0 && current_micros >= glow->cycle_start + lit_micros) {
        extinguish_glow(glow, current_micros);
      }

      k_mutex_unlock(&glow_mutex);
    }
    k_sleep(LOOP_SLEEP);
  }
  return ret;
}

//////////////////////////////////////////////////////////////
// Zbus
//////////////////////////////////////////////////////////////

bool glow_command_validator(const void* msg, size_t msg_size)
{
  const struct glow_command_msg* cmd = msg;
  if (cmd->glow < 0 || cmd->glow > ARRAY_SIZE(glows) - 1) {
    return false;
  }

  if (cmd->duration < 0) {
    return false;
  }

  k_mutex_lock(&glow_mutex, MUTEX_WAIT);
  const struct glow_state* glow = &glows[cmd->glow];
  const int max_duration = glow->max_duration;
  const bool lit = glow->cycle_start != 0;
  k_mutex_unlock(&glow_mutex);

  if (lit && cmd->duration != 0) {
    return false;
  }

  if (cmd->duration != 0 && cmd->duration > max_duration) {
    return false;
  }

  return true;
}

void glow_command_callback(const struct zbus_channel* chan)
{
  const struct glow_command_msg* cmd = zbus_chan_const_msg(chan);
  LOG_DBG("Got command for glow %d for %d ms", cmd->glow, cmd->duration);

  k_mutex_lock(&glow_mutex, MUTEX_WAIT);
  const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

  struct glow_state* glow = &glows[cmd->glow];
  if (cmd->duration == 0) {
    extinguish_glow(glow, current_micros);
  } else {
    glow->duration = cmd->duration;
    light_glow(glow, current_micros);
  }

  k_mutex_unlock(&glow_mutex);
}

ZBUS_LISTENER_DEFINE(glow_command_listener, glow_command_callback);

ZBUS_CHAN_DEFINE(glow_command_chan, /* Name */
    struct glow_command_msg, /* Message type */
    glow_command_validator, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(glow_command_listener), /* Observers */
    ZBUS_MSG_INIT(.glow = 0, .duration = 0) /* Initial value */
);

ZBUS_CHAN_DEFINE(glow_data_chan, /* Name */
    struct glow_data_msg, /* Message type */
    NULL, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(state_data_listener), /* Observers */
    ZBUS_MSG_INIT(.glow = 0, .timestamp = 0,
        .lit = false) /* Initial value */
);
