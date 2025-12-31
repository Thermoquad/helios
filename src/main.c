// SPDX-License-Identifier: Apache-2.0
#include <zephyr/kernel.h>

#include <helios/shell.h>
#include <helios/threads.h>

SHELL_CMD_ARG_REGISTER(helios, &sub_helios, "Operate the burner", cmd_helios, 1,
    0);

K_THREAD_DEFINE(glow_controller_id, CONFIG_MAIN_STACK_SIZE, glow_controller,
    NULL, NULL, NULL, -1, 0, 0);

K_THREAD_DEFINE(pump_controller_id, CONFIG_MAIN_STACK_SIZE, pump_controller,
    NULL, NULL, NULL, -1, 0, 0);

K_THREAD_DEFINE(motor_controller_id, CONFIG_MAIN_STACK_SIZE, motor_controller,
    NULL, NULL, NULL, -1, 0, 0);

K_THREAD_DEFINE(temperature_controller_id, CONFIG_MAIN_STACK_SIZE,
    temperature_controller, NULL, NULL, NULL, -1, 0, 0);

K_THREAD_DEFINE(helios_state_id, CONFIG_MAIN_STACK_SIZE, helios_state_runner,
    NULL, NULL, NULL, 3, 0, 0);

int main(void) { return 0; }
