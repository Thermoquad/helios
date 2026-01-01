/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Helios Serial Handler - ICU-specific implementation
 */

#ifndef HELIOS_SERIAL_HANDLER_H
#define HELIOS_SERIAL_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize serial communication handler
 *
 * Sets up UART device, decoder state, and communication threads.
 *
 * @return 0 on success, negative error code on failure
 */
int serial_handler_init(void);

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

#endif /* HELIOS_SERIAL_HANDLER_H */
