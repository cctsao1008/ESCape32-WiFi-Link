/*
** Copyright (C) 2023 Arseny Vakhrushev <arseny.vakhrushev@me.com>
**
** USB-only transport adaptation for E61.
**
** This firmware is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
*/

#include <stdint.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"

#define BRIDGE_CHUNK_SIZE       256
#define USB_DRIVER_BUFFER_SIZE  2048
#define UART_RX_BUFFER_SIZE     2048
#define BRIDGE_TASK_STACK_SIZE  4096
#define BRIDGE_TASK_PRIORITY    10

static void setled(int on)
{
#ifdef CONFIG_LED_INV
	on = !on;
#endif
	gpio_set_level(CONFIG_LED_PIN, on);
}

static void uart_write_all(const uint8_t *buf, int len)
{
	int pos = 0;

	while (pos < len) {
		int n = uart_write_bytes(CONFIG_UART_NUM, buf + pos, len - pos);
		if (n <= 0) {
			continue;
		}
		pos += n;
	}

	/*
	 * Keep the request/response boundary deterministic on the single-wire
	 * ESC link.  In RS485 half-duplex mode the driver releases TX after the
	 * final bit has left the UART.
	 */
	ESP_ERROR_CHECK(uart_wait_tx_done(CONFIG_UART_NUM, portMAX_DELAY));
}

static void usb_write_all(const uint8_t *buf, int len)
{
	int pos = 0;

	while (pos < len) {
		int n = usb_serial_jtag_write_bytes(
			buf + pos,
			len - pos,
			portMAX_DELAY
		);
		if (n <= 0) {
			continue;
		}
		pos += n;
	}
}

static void usb_to_uart_task(void *arg)
{
	uint8_t buf[BRIDGE_CHUNK_SIZE];
	(void)arg;

	for (;;) {
		/*
		 * Continuously consume native USB Serial/JTAG RX data.  This is
		 * essential because the peripheral applies back-pressure when its RX
		 * path is not serviced; on Windows that otherwise appears as a serial
		 * write timeout.
		 */
		int n = usb_serial_jtag_read_bytes(buf, sizeof buf, portMAX_DELAY);
		if (n <= 0) {
			continue;
		}

		setled(1);
		uart_write_all(buf, n);
		setled(0);
	}
}

static void uart_to_usb_task(void *arg)
{
	uint8_t buf[BRIDGE_CHUNK_SIZE];
	(void)arg;

	for (;;) {
		int n = uart_read_bytes(
			CONFIG_UART_NUM,
			buf,
			sizeof buf,
			portMAX_DELAY
		);
		if (n <= 0) {
			continue;
		}

		setled(1);
		usb_write_all(buf, n);
		setled(0);
	}
}

void app_main(void)
{
	gpio_set_direction(CONFIG_LED_PIN, GPIO_MODE_OUTPUT);
	setled(1);

	/* ESCape32 single-wire transport: 38400 baud, 8N1. */
	uart_config_t uart_cfg = {
		.baud_rate = 38400,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	ESP_ERROR_CHECK(uart_driver_install(
		CONFIG_UART_NUM,
		UART_RX_BUFFER_SIZE,
		0,
		0,
		NULL,
		0
	));
	ESP_ERROR_CHECK(uart_param_config(CONFIG_UART_NUM, &uart_cfg));
	ESP_ERROR_CHECK(uart_set_pin(
		CONFIG_UART_NUM,
		CONFIG_UART_TX,
		CONFIG_UART_RX,
		UART_PIN_NO_CHANGE,
		UART_PIN_NO_CHANGE
	));
	ESP_ERROR_CHECK(uart_set_mode(CONFIG_UART_NUM, UART_MODE_RS485_HALF_DUPLEX));
	ESP_ERROR_CHECK(uart_flush_input(CONFIG_UART_NUM));

	usb_serial_jtag_driver_config_t usb_cfg = {
		.rx_buffer_size = USB_DRIVER_BUFFER_SIZE,
		.tx_buffer_size = USB_DRIVER_BUFFER_SIZE,
	};
	ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

	BaseType_t res;
	res = xTaskCreate(
		usb_to_uart_task,
		"usb-to-uart",
		BRIDGE_TASK_STACK_SIZE,
		NULL,
		BRIDGE_TASK_PRIORITY,
		NULL
	);
	ESP_ERROR_CHECK(res == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

	res = xTaskCreate(
		uart_to_usb_task,
		"uart-to-usb",
		BRIDGE_TASK_STACK_SIZE,
		NULL,
		BRIDGE_TASK_PRIORITY,
		NULL
	);
	ESP_ERROR_CHECK(res == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

	setled(0);
}
