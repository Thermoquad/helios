#ifndef HELIOS_SHELL_H
#define HELIOS_SHELL_H
#include <stdio.h>
#include <zephyr/shell/shell.h>

int cmd_helios(const struct shell* sh, size_t argc, char** argv);

int cmd_set_rpm(const struct shell* sh, size_t argc, char** argv);
int cmd_set_pump_rate(const struct shell* sh, size_t argc, char** argv);
int cmd_set_glow_burn(const struct shell* sh, size_t argc, char** argv);

int cmd_get_state(const struct shell* sh, size_t argc, char** argv);
int cmd_set_fan(const struct shell* sh, size_t argc, char** argv);
int cmd_set_idle(const struct shell* sh, size_t argc, char** argv);
int cmd_set_heat(const struct shell* sh, size_t argc, char** argv);

int cmd_fake_temp(const struct shell* sh, size_t argc, char** argv);

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_helios_state,
    SHELL_CMD(fan, NULL, "Set fan RPM", cmd_set_fan),
    SHELL_CMD(idle, NULL, "Shutdown heating or fan mode", cmd_set_idle),
    SHELL_CMD(heat, NULL, "Set heat level", cmd_set_heat),
    SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_helios_fake,
    SHELL_CMD(temperature, NULL, "Send fake temperature data to Helios", cmd_fake_temp),
    SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(
    sub_helios,
    SHELL_CMD(fake, &sub_helios_fake, "Set motor RPM", NULL),
    SHELL_CMD(set_rpm, NULL, "Set motor RPM", cmd_set_rpm),
    SHELL_CMD(state, &sub_helios_state, "Set Helios' state", cmd_get_state),
    SHELL_CMD(set_pump_rate, NULL, "Set pump rate in milliseconds",
        cmd_set_pump_rate),
    SHELL_CMD(set_glow_burn, NULL, "Light glow plug for x milliseconds",
        cmd_set_glow_burn),
    SHELL_SUBCMD_SET_END);

#endif
