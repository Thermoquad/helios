// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef HELIOS_THREADS_H
#define HELIOS_THREADS_H

int helios_state_runner(void);

int glow_controller(void);
int pump_controller(void);
int motor_controller(void);
int temperature_controller(void);

int serial_thread(void);

#endif
