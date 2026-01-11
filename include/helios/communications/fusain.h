/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Helios Fusain Handler - ICU-specific implementation
 */

#ifndef HELIOS_FUSAIN_H
#define HELIOS_FUSAIN_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Send a telemetry bundle packet
 *
 * Called by telemetry thread to broadcast system state.
 */
void serial_send_telemetry_bundle(void);

/**
 * Send a ping response packet
 *
 * Called when PING_REQUEST is received.
 */
void serial_send_ping_response(void);

/**
 * Get timeout mode configuration
 *
 * @param enabled Output: timeout mode enabled
 * @param timeout_ms Output: timeout interval in ms
 */
void serial_get_timeout_config(bool* enabled, uint32_t* timeout_ms);

/**
 * Set timeout mode enabled/disabled
 *
 * @param enabled Timeout mode enabled
 */
void serial_set_timeout_enabled(bool enabled);

#endif /* HELIOS_FUSAIN_H */
