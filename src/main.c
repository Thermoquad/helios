// SPDX-License-Identifier: GPL-2.0-or-later
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <helios/communications/serial_handler.h>
#include <helios/shell.h>
#include <helios/threads.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

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

K_THREAD_DEFINE(serial_rx_id, CONFIG_MAIN_STACK_SIZE, serial_rx_thread, NULL, NULL, NULL, 5, 0,
    0);

K_THREAD_DEFINE(serial_tx_id, CONFIG_MAIN_STACK_SIZE, serial_tx_thread, NULL, NULL, NULL, 6, 0,
    0);

int main(void)
{
  LOG_INF("Helios ICU starting...");
  return 0;
}
