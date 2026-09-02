#pragma once

#include <stdbool.h>

typedef enum {
	ESC_OWNER_NONE = 0,
	ESC_OWNER_WIFI,
	ESC_OWNER_USB,
} esc_owner_t;

/*
 * Acquire the shared ESC UART for one host transport.
 *
 * If the UART is idle, ownership is assigned to `owner` and *new_session is
 * set to true (when non-NULL). Re-acquiring by the same owner succeeds and
 * reports new_session=false. Acquisition by the other owner fails.
 */
bool esc_owner_try_acquire(esc_owner_t owner, bool *new_session);

/* Release only if `owner` is the current owner. */
void esc_owner_release(esc_owner_t owner);

/* Read the current owner atomically. */
esc_owner_t esc_owner_get(void);

/* Start the ESP32-C3 USB Serial/JTAG <-> ESC UART transparent bridge. */
void usb_bridge_start(void);
