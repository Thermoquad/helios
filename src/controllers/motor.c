#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <helios/pid.h>
#include <helios/zbus.h>

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(motor_controller);

#define DEFAULT_KP 4.0
#define DEFAULT_KI 12.0
#define DEFAULT_KD 0.1

#define LOW_CUT_OFF PWM_USEC(10U)
#define LOOP_SLEEP K_MSEC(25U)
#define MUTEX_WAIT K_MSEC(100U)
#define RPM_MAX 3400
#define RPM_MIN 800
#define USE_MOTOR_ENABLERS true
#define ANNOUNCE_RATE_MS 100

//////////////////////////////////////////////////////////////
// Hardware setup
//////////////////////////////////////////////////////////////

#define MOTOR_ALIASES(i) DT_ALIAS(_CONCAT(helios_motor, i))
#define MOTORS(i, _)                           \
  IF_ENABLED(DT_NODE_EXISTS(MOTOR_ALIASES(i)), \
      (PWM_DT_SPEC_GET(MOTOR_ALIASES(i)), ))
static struct pwm_dt_spec motor_pins[] = { LISTIFY(10, MOTORS, ()) };

#define TACH_ALIASES(i) DT_ALIAS(_CONCAT(helios_tach, i))
#define TACHS(i, _)                           \
  IF_ENABLED(DT_NODE_EXISTS(TACH_ALIASES(i)), \
      (DEVICE_DT_GET(TACH_ALIASES(i)), ))
static const struct device* tachs[] = { LISTIFY(10, TACHS, ()) };

#if USE_MOTOR_ENABLERS
#define MOTOR_ENABLE_ALIASES(i) DT_ALIAS(_CONCAT(helios_enable_motor, i))
#define MOTOR_ENABLERS(i, _)                          \
  IF_ENABLED(DT_NODE_EXISTS(MOTOR_ENABLE_ALIASES(i)), \
      (GPIO_DT_SPEC_GET(MOTOR_ENABLE_ALIASES(i), gpios), ))
static struct gpio_dt_spec enablers[] = { LISTIFY(10, MOTOR_ENABLERS, ()) };
#endif /* if USE_MOTOR_ENABLERS */

//////////////////////////////////////////////////////////////
// Motor struct
//////////////////////////////////////////////////////////////

struct motor_state {
  int index;
  unsigned last_announce;

  const struct device* tach;
  struct pid_controller pid;
  int current_rpm;
  int target_rpm;
  int max_rpm;
  int min_rpm;

  struct pwm_dt_spec pwm;
  int pulse;
  int pulse_cycle;
  int pulse_min;

#if USE_MOTOR_ENABLERS
  struct gpio_dt_spec enabler;
#endif
};

//////////////////////////////////////////////////////////////
// State variables
//////////////////////////////////////////////////////////////

K_MUTEX_DEFINE(motor_mutex);
static struct motor_state motors[ARRAY_SIZE(motor_pins)];

//////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////

void pid_control(struct motor_state* motor, unsigned current_micros)
{
  LOG_DBG_RATELIMIT("running pid cycle for motor %d", motor->index);
  motor->pid.input = motor->current_rpm;
  motor->pid.target = motor->target_rpm;
  run_pid_cycle(&motor->pid, current_micros);
  motor->pulse = motor->pid.output;
}

void get_rpm(struct motor_state* motor)
{
  struct sensor_value pdata;
  if (sensor_sample_fetch(motor->tach)) {
    LOG_DBG_RATELIMIT("Failed to get RPM from tach for motor %d", motor->index);
    motor->current_rpm = 0;
    return;
  }

  sensor_channel_get(motor->tach, SENSOR_CHAN_RPM, &pdata);

  // filter out bad values from the tach
  if (pdata.val1 <= motor->max_rpm * 2 && pdata.val1 > 0) {
    motor->current_rpm = pdata.val1;
  }
}

void publish_motor_data(struct motor_state* motor, unsigned current_micros)
{
  const unsigned current_millis = current_micros / 1e3;
  const unsigned millis_since_last_publish = current_millis - motor->last_announce;
  if (millis_since_last_publish < ANNOUNCE_RATE_MS) {
    return;
  }

  struct motor_data_msg msg = { .motor = motor->index,
    .rpm = motor->current_rpm,
    .target = motor->target_rpm,
    .timestamp = current_micros,
    .max_rpm = motor->max_rpm,
    .min_rpm = motor->min_rpm,
    .pwm = motor->pulse,
    .pwm_max = motor->pulse_cycle };
  zbus_chan_pub(&motor_data_chan, &msg, K_MSEC(1U));
  motor->last_announce = current_millis;
  LOG_DBG_RATELIMIT("Got %d/%d RPM for motor %d", motor->current_rpm,
      motor->target_rpm, motor->index);
  // LOG_DBG_RATELIMIT("announcing rpm for motor %d", motor->index);
}

void set_motor_pwm(struct motor_state* motor)
{
#if USE_MOTOR_ENABLERS
  gpio_pin_set_dt(&motor->enabler, (int)motor->pulse != 0);
#endif /* if USE_MOTOR_ENABLERS */
  pwm_set_dt(&motor->pwm, motor->pulse_cycle, motor->pulse);
  LOG_DBG_RATELIMIT("set motor %d PWM to %d/%d", motor->index, motor->pulse,
      motor->pulse_cycle);
}

void stop_motor(struct motor_state* motor)
{
  motor->pulse = 0;
  set_motor_pwm(motor);
  motor->target_rpm = 0;
  motor->last_announce = 0;
  reset_pid(&motor->pid);
  LOG_DBG("stopped motor %d", motor->index);
}

//////////////////////////////////////////////////////////////
// Controller thread functions
//////////////////////////////////////////////////////////////

int initialize_motors()
{
  int ret = 0;

  for (int i = 0; i < ARRAY_SIZE(motors); i++) {
#if USE_MOTOR_ENABLERS
    const struct gpio_dt_spec enabler = enablers[i];
    if (gpio_pin_configure_dt(&enabler, GPIO_OUTPUT_INACTIVE) != 0) {
      LOG_ERR("Fatal: failed to configure %s pin %d\n", enabler.port->name,
          enabler.pin);
      ret = 3;
      break;
    }
#endif /* if USE_MOTOR_ENABLERS */

    const struct pwm_dt_spec motor_pin = motor_pins[i];

    if (!pwm_is_ready_dt(&motor_pin)) {
      LOG_ERR("Fatal: PWM device %s is not ready\n", motor_pin.dev->name);
      ret = 2;
      break;
    }

    struct motor_state state;

    state.index = i;
    state.last_announce = 0;

    state.tach = tachs[i];
    state.target_rpm = 0;
    state.current_rpm = 0;
    state.max_rpm = RPM_MAX;
    state.min_rpm = RPM_MIN;

    state.pulse = 0;
    state.pulse_cycle = motor_pin.period;
    state.pulse_min = LOW_CUT_OFF;
    state.pwm = motor_pin;

    struct pid_controller pid;
    pid.p_gain = DEFAULT_KP;
    pid.i_gain = DEFAULT_KI;
    pid.d_gain = DEFAULT_KD;
    reset_pid(&pid);
    pid.output_max_limit = state.pulse_cycle;
    pid.output_min_limit = state.pulse_min;
    pid.time_divisor = 1.0e6;
    pid.inverted = false;
    state.pid = pid;

#if USE_MOTOR_ENABLERS
    state.enabler = enabler;
#endif
    motors[i] = state;
  }

  return ret;
}

int motor_controller(void)
{
  int ret = 0;
  k_mutex_init(&motor_mutex);
  if (ARRAY_SIZE(motors) != ARRAY_SIZE(tachs)) {
    LOG_ERR("fatal: number of motors and tachs don't match");
    ret = 1;
    return ret;
  }

  ret = initialize_motors();
  if (ret) {
    LOG_ERR("fatal: unable to initialize motor hardware");
    return ret;
  }

  while (true) {
    k_mutex_lock(&motor_mutex, MUTEX_WAIT);

    for (int i = 0; i < ARRAY_SIZE(motors); i++) {
      struct motor_state* motor = &motors[i];
      const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

      get_rpm(motor);
      if (motor->target_rpm != 0) {
        pid_control(motor, current_micros);
        set_motor_pwm(motor);
      }
      publish_motor_data(motor, current_micros);
    }

    k_mutex_unlock(&motor_mutex);
    k_sleep(LOOP_SLEEP);
  }

  return 0;
}

//////////////////////////////////////////////////////////////
// Zbus
//////////////////////////////////////////////////////////////

bool motor_command_validator(const void* msg, size_t msg_size)
{
  const struct motor_command_msg* cmd = msg;
  if (cmd->motor < 0 || cmd->motor > ARRAY_SIZE(motors) - 1) {
    return false;
  }

  if (cmd->rpm == 0) {
    return true;
  }

  bool ret = true;
  k_mutex_lock(&motor_mutex, MUTEX_WAIT);
  const struct motor_state* motor = &motors[cmd->motor];
  if (cmd->rpm > motor->max_rpm || cmd->rpm < motor->min_rpm) {
    ret = false;
  }
  k_mutex_unlock(&motor_mutex);

  return ret;
}

void motor_command_callback(const struct zbus_channel* chan)
{
  const struct motor_command_msg* cmd = zbus_chan_const_msg(chan);
  LOG_DBG("Got command for motor %d, rpm %d", cmd->motor, cmd->rpm);
  k_mutex_lock(&motor_mutex, MUTEX_WAIT);

  struct motor_state* motor = &motors[cmd->motor];
  if (cmd->rpm == 0) {
    stop_motor(motor);
  } else {
    motor->target_rpm = cmd->rpm;
  }

  k_mutex_unlock(&motor_mutex);
}

ZBUS_LISTENER_DEFINE(motor_command_listener, motor_command_callback);

ZBUS_CHAN_DEFINE(motor_command_chan, /* Name */
    struct motor_command_msg, /* Message type */
    motor_command_validator, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(motor_command_listener), /* Observers */
    ZBUS_MSG_INIT(.motor = 0, .rpm = 0) /* Initial value */
);

ZBUS_CHAN_DEFINE(motor_data_chan, /* Name */
    struct motor_data_msg, /* Message type */
    NULL, /* Validator */
    NULL, /* User Data */
    ZBUS_OBSERVERS(state_data_listener), /* Observers */
    ZBUS_MSG_INIT(.motor = 0, .rpm = 0, .target = 0,
        .timestamp = 0) /* Initial value */
);
