// SPDX-License-Identifier: GPL-2.0-or-later
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <helios/communications/fusain.h>
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

K_THREAD_DEFINE(thermometer_controller_id, CONFIG_MAIN_STACK_SIZE * 2,
    thermometer_controller, NULL, NULL, NULL, -1, 0, 0);

K_THREAD_DEFINE(helios_state_id, CONFIG_MAIN_STACK_SIZE, helios_state_runner,
    NULL, NULL, NULL, 4, 0, 0); // State machine - important but below serial

K_THREAD_DEFINE(serial_rx_id, CONFIG_MAIN_STACK_SIZE * 2, serial_rx_thread, NULL, NULL, NULL, -2, 0, 0); // RX: Highest priority - time-critical UART RX polling
K_THREAD_DEFINE(serial_tx_id, CONFIG_MAIN_STACK_SIZE, serial_tx_thread, NULL, NULL, NULL, 0, 0, 0);  // TX: Priority 0 - UART TX polling and queue processing
K_THREAD_DEFINE(serial_processing_id, CONFIG_MAIN_STACK_SIZE * 4, serial_processing_thread, NULL, NULL, NULL, 1, 0, 0); // Processing: Larger stack for CBOR encoding

int main(void)
{
  LOG_INF("Helios ICU starting...");
  return 0;
}
