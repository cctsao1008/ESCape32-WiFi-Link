#include "sdkconfig.h"
#include "usb_bridge.h"

#include <stdint.h>
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#ifdef CONFIG_USB_BRIDGE
#include "driver/usb_serial_jtag.h"
#endif

static portMUX_TYPE owner_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile esc_owner_t owner = ESC_OWNER_NONE;

bool esc_owner_try_acquire(esc_owner_t request, bool *new_session) {
	bool ok = false;
	bool fresh = false;

	if (request == ESC_OWNER_NONE) return false;

	portENTER_CRITICAL(&owner_lock);
	if (owner == ESC_OWNER_NONE) {
		owner = request;
		fresh = true;
		ok = true;
	} else if (owner == request) {
		ok = true;
	}
	portEXIT_CRITICAL(&owner_lock);

	if (new_session) *new_session = fresh;
	return ok;
}

void esc_owner_release(esc_owner_t request) {
	if (request == ESC_OWNER_NONE) return;

	portENTER_CRITICAL(&owner_lock);
	if (owner == request) owner = ESC_OWNER_NONE;
	portEXIT_CRITICAL(&owner_lock);
}

esc_owner_t esc_owner_get(void) {
	esc_owner_t current;

	portENTER_CRITICAL(&owner_lock);
	current = owner;
	portEXIT_CRITICAL(&owner_lock);

	return current;
}

#ifdef CONFIG_USB_BRIDGE

#define USB_BUF_SIZE 256
#define USB_DRIVER_BUF_SIZE 2048

static TickType_t usb_last_activity;

static void usb_touch(void) {
	usb_last_activity = xTaskGetTickCount();
}

static void usb_write_all(const uint8_t *buf, int len) {
	int pos = 0;

	while (pos < len) {
		int n = usb_serial_jtag_write_bytes(
			buf + pos,
			len - pos,
			pdMS_TO_TICKS(100)
		);
		if (n <= 0) return;
		pos += n;
	}
}

static void usb_bridge_task(void *arg) {
	uint8_t usb_buf[USB_BUF_SIZE];
	uint8_t uart_buf[USB_BUF_SIZE];
	const TickType_t idle_ticks = pdMS_TO_TICKS(CONFIG_USB_BRIDGE_IDLE_MS);

	(void)arg;

	for (;;) {
		bool did_work = false;

		/*
		 * Host -> ESP32-C3. The USB Serial/JTAG controller applies
		 * back-pressure when the application does not consume RX bytes, so
		 * this read is the key operation that lets Windows WriteFile()
		 * complete even when no ESC is connected.
		 */
		int n = usb_serial_jtag_read_bytes(usb_buf, sizeof usb_buf, 1);
		if (n > 0) {
			bool fresh = false;

			/*
			 * Never interleave raw USB traffic with a WiFi/WebSocket ESC
			 * transaction. Keep the received USB bytes locally until the
			 * WiFi owner releases the UART.
			 */
			while (!esc_owner_try_acquire(ESC_OWNER_USB, &fresh)) {
				vTaskDelay(1);
			}

			if (fresh) {
				/*
				 * Establish one clean USB session. This is deliberately done
				 * only once on ownership transition, NOT once per USB packet.
				 * Per-chunk flushing can destroy a valid ESC response.
				 */
				uart_flush(CONFIG_UART_NUM);
			}

			usb_touch();

			int pos = 0;
			while (pos < n) {
				int written = uart_write_bytes(CONFIG_UART_NUM, usb_buf + pos, n - pos);
				if (written <= 0) break;
				pos += written;
			}

			did_work = true;
		}

		/*
		 * ESC -> host. Only the USB owner may consume UART RX bytes. The
		 * existing WiFi recvbuf() remains the sole UART reader while WiFi
		 * owns the link.
		 */
		if (esc_owner_get() == ESC_OWNER_USB) {
			int m = uart_read_bytes(CONFIG_UART_NUM, uart_buf, sizeof uart_buf, 0);
			if (m > 0) {
				usb_touch();
				usb_write_all(uart_buf, m);
				did_work = true;
			}

			/*
			 * USB has no explicit open/close callback in this design. An
			 * inactivity lease returns ownership to WiFi after the host
			 * stops talking. Both host TX and ESC RX refresh the lease.
			 */
			if ((TickType_t)(xTaskGetTickCount() - usb_last_activity) >= idle_ticks) {
				esc_owner_release(ESC_OWNER_USB);
			}
		}

		if (!did_work) vTaskDelay(1);
	}
}

#endif /* CONFIG_USB_BRIDGE */

void usb_bridge_start(void) {
#ifdef CONFIG_USB_BRIDGE
	usb_serial_jtag_driver_config_t cfg = {
		.tx_buffer_size = USB_DRIVER_BUF_SIZE,
		.rx_buffer_size = USB_DRIVER_BUF_SIZE,
	};

	ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));

	BaseType_t res = xTaskCreate(
		usb_bridge_task,
		"usb-esc-bridge",
		4096,
		NULL,
		10,
		NULL
	);
	ESP_ERROR_CHECK(res == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
#endif
}
