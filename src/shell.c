// SPDX-License-Identifier: GPL-2.0-or-later

#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>
#include <sys/errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include <helios/communications/serial_handler.h>
#include <helios/zbus.h>

//////////////////////////////////////////////////////////////
// Config
//////////////////////////////////////////////////////////////

LOG_MODULE_REGISTER(helios_shell);

#define PUB_TIMEOUT K_SECONDS(1U)

//////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////

int send_state_command(const struct shell* sh, struct state_command_msg* cmd)
{
  int res = zbus_chan_pub(&state_command_chan, cmd, PUB_TIMEOUT);
  if (res != 0) {
    if (res == -ENOMSG) {
      shell_print(sh, "invalid state command");
    } else {
      LOG_WRN("got zbus error: %d", res);
      shell_print(sh, "Unable to send state command to Helios");
    }
  }
  shell_print(sh, "sent state %s command %d", helios_mode_names[cmd->mode], cmd->argument);
  return res;
}

//////////////////////////////////////////////////////////////
// Shell functions
//////////////////////////////////////////////////////////////

int cmd_helios(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  shell_print(sh, "Helios awaits your command");
  return 0;
}

int cmd_fake_temp(const struct shell* sh, size_t argc, char** argv)
{
  if (argc != 3) {
    shell_print(sh, "wrong number of args");
    return 1;
  }

  const int i = atoi(argv[1]);
  const double current_temperature = atof(argv[2]);
  const unsigned current_micros = k_cyc_to_us_floor64(k_cycle_get_64());

  struct temperature_data_msg msg = { .thermometer = i,
    .temperature = current_temperature,
    .timestamp = current_micros };
  int res = zbus_chan_pub(&temperature_data_chan, &msg, PUB_TIMEOUT);
  if (res != 0) {
    if (res == -ENOMSG) {
      shell_print(sh, "invalid state command");
    } else {
      LOG_WRN("got zbus error: %d", res);
      shell_print(sh, "Unable to send fake data to Helios");
    }
  }
  shell_print(sh, "sending fake data for temperature for temp %d, %f", i,
      current_temperature);
  return res;
}

int cmd_get_state(const struct shell* sh, size_t argc, char** argv)
{
  struct state_data_msg msg;
  int ret;
  ret = zbus_chan_read(&state_data_chan, &msg, PUB_TIMEOUT);
  if (ret == 0) {
    shell_print(sh, "%d - %s", msg.state, helios_state_names[msg.state]);
  } else {
    shell_print(sh, "Unable to get Helios state");
  }
  return ret;
}

int cmd_set_idle(const struct shell* sh, size_t argc, char** argv)
{
  struct state_command_msg cmd;
  cmd.mode = HELIOS_IDLE_MODE;
  return send_state_command(sh, &cmd);
}

int cmd_set_fan(const struct shell* sh, size_t argc, char** argv)
{
  struct state_command_msg cmd;
  cmd.mode = HELIOS_FAN_MODE;
  const int arg = atoi(argv[1]);
  cmd.argument = arg;
  return send_state_command(sh, &cmd);
}

int cmd_set_heat(const struct shell* sh, size_t argc, char** argv)
{
  struct state_command_msg cmd;
  cmd.mode = HELIOS_HEAT_MODE;
  const int arg = atoi(argv[1]);
  cmd.argument = arg;
  return send_state_command(sh, &cmd);
}

int cmd_set_rpm(const struct shell* sh, size_t argc, char** argv)
{
  if (argc != 3) {
    shell_print(sh, "wrong number of args");
    return 1;
  }
  const int motor_index = atoi(argv[1]);
  const int rpm = atoi(argv[2]);
  struct motor_command_msg msg = { .motor = motor_index, .rpm = rpm };
  zbus_chan_pub(&motor_command_chan, &msg, PUB_TIMEOUT);
  return 0;
}

int cmd_set_pump_rate(const struct shell* sh, size_t argc, char** argv)
{
  if (argc != 3) {
    shell_print(sh, "wrong number of args");
    return 1;
  }
  const int pump_index = atoi(argv[1]);
  const int rate_ms = atoi(argv[2]);
  struct pump_command_msg msg = { .pump = pump_index, .rate_ms = rate_ms };
  zbus_chan_pub(&pump_command_chan, &msg, PUB_TIMEOUT);
  return 0;
}

int cmd_set_glow_burn(const struct shell* sh, size_t argc, char** argv)
{
  if (argc != 3) {
    shell_print(sh, "wrong number of args");
    return 1;
  }
  const int glow_index = atoi(argv[1]);
  const int burn_length = atoi(argv[2]);
  struct glow_command_msg msg = { .glow = glow_index, .duration = burn_length };
  zbus_chan_pub(&glow_command_chan, &msg, PUB_TIMEOUT);
  return 0;
}

int cmd_serial_timeout(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  // Display current timeout configuration
  bool enabled;
  uint32_t timeout_ms;
  serial_get_timeout_config(&enabled, &timeout_ms);

  shell_print(sh, "Serial Timeout Configuration:");
  shell_print(sh, "  Status: %s", enabled ? "ENABLED" : "DISABLED");
  shell_print(sh, "  Interval: %u ms (%u seconds)", timeout_ms,
      timeout_ms / 1000);

  if (enabled) {
    shell_print(sh, "");
    shell_print(sh,
        "Timeout will trigger IDLE transition if no PING received");
    shell_print(sh, "Recommended master ping interval: 10-15 seconds");
  }

  return 0;
}

int cmd_serial_timeout_enable(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  serial_set_timeout_enabled(true);
  shell_print(sh, "Serial timeout ENABLED");
  return 0;
}

int cmd_serial_timeout_disable(const struct shell* sh, size_t argc, char** argv)
{
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);

  serial_set_timeout_enabled(false);
  shell_print(sh, "Serial timeout DISABLED");
  return 0;
}
