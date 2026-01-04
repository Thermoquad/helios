// SPDX-License-Identifier: GPL-2.0-or-later

#ifndef HELIOS_ZBUS_H
#define HELIOS_ZBUS_H
#include <fusain/fusain.h>
#include <helios/messages.h>
#include <zephyr/zbus/zbus.h>

//////////////////////////////////////////////////////////////
// Channels
//////////////////////////////////////////////////////////////

ZBUS_CHAN_DECLARE(
    state_command_chan,
    state_data_chan,
    glow_command_chan,
    glow_data_chan,
    motor_command_chan,
    motor_data_chan,
    pump_command_chan,
    pump_data_chan,
    temperature_command_chan,
    temperature_data_chan);

ZBUS_OBS_DECLARE(state_data_listener);

#endif
