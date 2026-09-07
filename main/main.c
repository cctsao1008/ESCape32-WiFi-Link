/*
** Copyright (C) 2023 Arseny Vakhrushev <arseny.vakhrushev@me.com>
**
** USB + optional Wi-Fi transport integration.
**
** This firmware is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_crc.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "mdns.h"
#include "build_defs.h"

#define SSID "ESCape32-WiFi-Link"
#define HOSTNAME "escape32"

#define LINK_NVS_NAMESPACE "escape32_link"
#define LINK_NVS_WIFI_KEY  "wifi_enabled"

#define CMD_PROBE  0
#define CMD_INFO   1
#define CMD_READ   2
#define CMD_WRITE  3
#define CMD_UPDATE 4
#define CMD_SETWRP 5

#define BRIDGE_CHUNK_SIZE       256
#define USB_DRIVER_BUFFER_SIZE  2048
#define UART_RX_BUFFER_SIZE     2048
#define BRIDGE_TASK_STACK_SIZE  4096
#define BRIDGE_TASK_PRIORITY    10

#define LINK_OWNER_LEASE_MS     10000

#define ADAPTER_PROTOCOL_VERSION 1
#define ADAPTER_MAX_PAYLOAD      64
#define ADAPTER_CMD_INFO         1
#define ADAPTER_CMD_WIFI_GET     2
#define ADAPTER_CMD_WIFI_SET     3
#define ADAPTER_CMD_REBOOT       4
#define ADAPTER_CMD_ACQUIRE_ESC  5
#define ADAPTER_CMD_RELEASE_ESC  6

#define ADAPTER_STATUS_OK         0
#define ADAPTER_STATUS_BUSY       1
#define ADAPTER_STATUS_BAD_ARG    2
#define ADAPTER_STATUS_NVS_ERROR  3
#define ADAPTER_STATUS_INTERNAL   4

#define ADAPTER_FW_MAJOR 1
#define ADAPTER_FW_MINOR 0
#define ADAPTER_FW_PATCH 0

typedef enum {
	LINK_OWNER_NONE = 0,
	LINK_OWNER_USB = 1,
	LINK_OWNER_WIFI = 2,
} link_owner_t;

typedef struct __attribute__((__packed__)) {
	uint16_t xid;
	uint16_t flags;
	uint16_t qucnt;
	uint16_t ancnt;
	uint16_t nscnt;
	uint16_t arcnt;
} DNSHeader;

typedef struct __attribute__((__packed__)) {
	uint16_t name;
	uint16_t type;
	uint16_t class;
	uint32_t ttl;
	uint16_t len;
	uint32_t addr;
} DNSAnswer;

typedef struct __attribute__((__packed__)) {
	uint8_t version;
	uint8_t command;
	uint16_t length;
	uint32_t sequence;
} AdapterHeader;

typedef struct __attribute__((__packed__)) {
	uint8_t status;
	uint8_t fw_major;
	uint8_t fw_minor;
	uint8_t fw_patch;
	uint8_t wifi_configured;
	uint8_t wifi_active;
	uint8_t reboot_required;
	uint8_t owner;
	uint32_t uart_baud;
	uint8_t uart_rx;
	uint8_t uart_tx;
} AdapterInfoPayload;

typedef struct {
	uint8_t buf[8 + sizeof(AdapterHeader) + ADAPTER_MAX_PAYLOAD + 4];
	size_t len;
	size_t expected;
} AdapterParser;

static const uint8_t adapter_request_magic[8] = {
	0xA5, 0x5A, 'E', 'S', 'C', '3', '2', '!'
};

static const uint8_t adapter_response_magic[8] = {
	0x5A, 0xA5, 'E', 'S', 'C', '3', '2', '!'
};

extern const char _binary_root_html_gz_start[], _binary_root_html_gz_end[];
#define XX(lang) \
extern const char _binary_root_##lang##_json_gz_start[], _binary_root_##lang##_json_gz_end[];
LANG_LIST(XX)
#undef XX

static httpd_handle_t server;
static QueueHandle_t queue;
static SemaphoreHandle_t owner_mutex;
static SemaphoreHandle_t uart_mutex;
static SemaphoreHandle_t usb_tx_mutex;

static link_owner_t link_owner;
static TickType_t link_owner_touch_tick;

static bool wifi_configured;
static bool wifi_active;

static int update_size;
static int update_boot;
static int update_wrp;
static int update_ofs;
static int update_idx;
static bool update_active;

static inline int min(int a, int b)
{
	return a < b ? a : b;
}

static void setled(int x)
{
#ifdef CONFIG_LED_INV
	x = !x;
#endif
	gpio_set_level(CONFIG_LED_PIN, x);
}

static bool link_owner_expired_locked(void)
{
	if (link_owner == LINK_OWNER_NONE) return false;
	TickType_t now = xTaskGetTickCount();
	return (TickType_t)(now - link_owner_touch_tick) >= pdMS_TO_TICKS(LINK_OWNER_LEASE_MS);
}

static link_owner_t link_owner_get(void)
{
	link_owner_t owner;
	xSemaphoreTake(owner_mutex, portMAX_DELAY);
	if (link_owner_expired_locked()) link_owner = LINK_OWNER_NONE;
	owner = link_owner;
	xSemaphoreGive(owner_mutex);
	return owner;
}

static bool link_owner_acquire(link_owner_t owner)
{
	bool ok = false;
	xSemaphoreTake(owner_mutex, portMAX_DELAY);
	if (link_owner_expired_locked()) link_owner = LINK_OWNER_NONE;
	if (link_owner == LINK_OWNER_NONE || link_owner == owner) {
		link_owner = owner;
		link_owner_touch_tick = xTaskGetTickCount();
		ok = true;
	}
	xSemaphoreGive(owner_mutex);
	return ok;
}

static void link_owner_touch(link_owner_t owner)
{
	xSemaphoreTake(owner_mutex, portMAX_DELAY);
	if (link_owner == owner) link_owner_touch_tick = xTaskGetTickCount();
	xSemaphoreGive(owner_mutex);
}

static void link_owner_release(link_owner_t owner)
{
	xSemaphoreTake(owner_mutex, portMAX_DELAY);
	if (link_owner == owner) link_owner = LINK_OWNER_NONE;
	xSemaphoreGive(owner_mutex);
}

static esp_err_t link_config_init(void)
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	if (err != ESP_OK) return err;

	nvs_handle_t handle;
	err = nvs_open(LINK_NVS_NAMESPACE, NVS_READWRITE, &handle);
	if (err != ESP_OK) return err;

	uint8_t value = 0;
	err = nvs_get_u8(handle, LINK_NVS_WIFI_KEY, &value);
	if (err == ESP_ERR_NVS_NOT_FOUND) {
		wifi_configured = false;
		err = ESP_OK;
	} else if (err == ESP_OK) {
		wifi_configured = value != 0;
	}
	nvs_close(handle);
	return err;
}

static esp_err_t link_config_set_wifi(bool enabled)
{
	nvs_handle_t handle;
	esp_err_t err = nvs_open(LINK_NVS_NAMESPACE, NVS_READWRITE, &handle);
	if (err != ESP_OK) return err;
	err = nvs_set_u8(handle, LINK_NVS_WIFI_KEY, enabled ? 1 : 0);
	if (err == ESP_OK) err = nvs_commit(handle);
	nvs_close(handle);
	if (err == ESP_OK) wifi_configured = enabled;
	return err;
}

static void usb_write_all(const uint8_t *buf, int len)
{
	int pos = 0;
	xSemaphoreTake(usb_tx_mutex, portMAX_DELAY);
	while (pos < len) {
		int n = usb_serial_jtag_write_bytes(buf + pos, len - pos, portMAX_DELAY);
		if (n <= 0) continue;
		pos += n;
	}
	xSemaphoreGive(usb_tx_mutex);
}

static void uart_write_all(const uint8_t *buf, int len)
{
	int pos = 0;
	while (pos < len) {
		int n = uart_write_bytes(CONFIG_UART_NUM, buf + pos, len - pos);
		if (n <= 0) continue;
		pos += n;
	}
}

static void usb_forward_raw(const uint8_t *buf, int len)
{
	if (!len) return;

	/*
	 * Always consume USB RX even when Wi-Fi owns the ESC.  Dropping a raw USB
	 * packet while busy is preferable to propagating USB back-pressure to the
	 * host.  A normal USB transaction automatically acquires a short lease.
	 */
	if (!link_owner_acquire(LINK_OWNER_USB)) return;

	if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) != pdTRUE) return;
	if (link_owner_get() == LINK_OWNER_USB) {
		setled(1);
		uart_write_all(buf, len);
		link_owner_touch(LINK_OWNER_USB);
		setled(0);
	}
	xSemaphoreGive(uart_mutex);
}

static void adapter_fill_info(AdapterInfoPayload *info, uint8_t status)
{
	memset(info, 0, sizeof *info);
	info->status = status;
	info->fw_major = ADAPTER_FW_MAJOR;
	info->fw_minor = ADAPTER_FW_MINOR;
	info->fw_patch = ADAPTER_FW_PATCH;
	info->wifi_configured = wifi_configured ? 1 : 0;
	info->wifi_active = wifi_active ? 1 : 0;
	info->reboot_required = wifi_configured != wifi_active;
	info->owner = (uint8_t)link_owner_get();
	info->uart_baud = 38400;
	info->uart_rx = (uint8_t)CONFIG_UART_RX;
	info->uart_tx = (uint8_t)CONFIG_UART_TX;
}

static void adapter_send_response(uint8_t command, uint32_t sequence, uint8_t status)
{
	uint8_t frame[8 + sizeof(AdapterHeader) + sizeof(AdapterInfoPayload) + 4];
	AdapterHeader header = {
		.version = ADAPTER_PROTOCOL_VERSION,
		.command = command | 0x80,
		.length = sizeof(AdapterInfoPayload),
		.sequence = sequence,
	};
	AdapterInfoPayload payload;
	adapter_fill_info(&payload, status);

	int pos = 0;
	memcpy(frame + pos, adapter_response_magic, sizeof adapter_response_magic);
	pos += sizeof adapter_response_magic;
	memcpy(frame + pos, &header, sizeof header);
	pos += sizeof header;
	memcpy(frame + pos, &payload, sizeof payload);
	pos += sizeof payload;

	uint32_t crc = esp_crc32_le(
		0,
		frame + sizeof adapter_response_magic,
		sizeof header + sizeof payload
	);
	memcpy(frame + pos, &crc, sizeof crc);
	pos += sizeof crc;

	usb_write_all(frame, pos);
}

static void reboot_task(void *arg)
{
	(void)arg;
	vTaskDelay(pdMS_TO_TICKS(150));
	esp_restart();
}

static void adapter_process_frame(const AdapterHeader *header, const uint8_t *payload)
{
	uint8_t status = ADAPTER_STATUS_OK;

	switch (header->command) {
		case ADAPTER_CMD_INFO:
		case ADAPTER_CMD_WIFI_GET:
			if (header->length != 0) status = ADAPTER_STATUS_BAD_ARG;
			break;

		case ADAPTER_CMD_WIFI_SET:
			if (header->length != 1 || payload[0] > 1) {
				status = ADAPTER_STATUS_BAD_ARG;
			} else if (link_config_set_wifi(payload[0] != 0) != ESP_OK) {
				status = ADAPTER_STATUS_NVS_ERROR;
			}
			break;

		case ADAPTER_CMD_REBOOT:
			if (header->length != 0) status = ADAPTER_STATUS_BAD_ARG;
			break;

		case ADAPTER_CMD_ACQUIRE_ESC:
			if (header->length != 0) {
				status = ADAPTER_STATUS_BAD_ARG;
			} else if (!link_owner_acquire(LINK_OWNER_USB)) {
				status = ADAPTER_STATUS_BUSY;
			}
			break;

		case ADAPTER_CMD_RELEASE_ESC:
			if (header->length != 0) {
				status = ADAPTER_STATUS_BAD_ARG;
			} else {
				link_owner_release(LINK_OWNER_USB);
			}
			break;

		default:
			status = ADAPTER_STATUS_BAD_ARG;
			break;
	}

	adapter_send_response(header->command, header->sequence, status);

	if (header->command == ADAPTER_CMD_REBOOT && status == ADAPTER_STATUS_OK) {
		xTaskCreate(reboot_task, "link-reboot", 2048, NULL, BRIDGE_TASK_PRIORITY, NULL);
	}
}

static void forward_append(uint8_t *raw, int *raw_len, const uint8_t *data, int len)
{
	while (len > 0) {
		int room = BRIDGE_CHUNK_SIZE - *raw_len;
		int n = len < room ? len : room;
		memcpy(raw + *raw_len, data, n);
		*raw_len += n;
		data += n;
		len -= n;
		if (*raw_len == BRIDGE_CHUNK_SIZE) {
			usb_forward_raw(raw, *raw_len);
			*raw_len = 0;
		}
	}
}

static void adapter_parser_feed(AdapterParser *parser, uint8_t byte, uint8_t *raw, int *raw_len)
{
	/* Searching for the request magic. */
	if (parser->len < sizeof adapter_request_magic) {
		if (byte == adapter_request_magic[parser->len]) {
			parser->buf[parser->len++] = byte;
			return;
		}

		if (parser->len) {
			forward_append(raw, raw_len, parser->buf, parser->len);
			parser->len = 0;
			parser->expected = 0;
			if (byte == adapter_request_magic[0]) {
				parser->buf[parser->len++] = byte;
				return;
			}
		}

		forward_append(raw, raw_len, &byte, 1);
		return;
	}

	/* Header follows the 8-byte magic. */
	parser->buf[parser->len++] = byte;
	if (parser->len == sizeof adapter_request_magic + sizeof(AdapterHeader)) {
		AdapterHeader header;
		memcpy(&header, parser->buf + sizeof adapter_request_magic, sizeof header);
		if (
			header.version != ADAPTER_PROTOCOL_VERSION ||
			header.length > ADAPTER_MAX_PAYLOAD
		) {
			forward_append(raw, raw_len, parser->buf, parser->len);
			parser->len = 0;
			parser->expected = 0;
			return;
		}
		parser->expected =
			sizeof adapter_request_magic +
			sizeof(AdapterHeader) +
			header.length +
			sizeof(uint32_t);
		return;
	}

	if (!parser->expected || parser->len < parser->expected) return;

	AdapterHeader header;
	memcpy(&header, parser->buf + sizeof adapter_request_magic, sizeof header);
	const uint8_t *payload =
		parser->buf + sizeof adapter_request_magic + sizeof(AdapterHeader);

	uint32_t crc_wire;
	memcpy(&crc_wire, parser->buf + parser->expected - sizeof crc_wire, sizeof crc_wire);

	uint32_t crc_calc = esp_crc32_le(
		0,
		parser->buf + sizeof adapter_request_magic,
		sizeof(AdapterHeader) + header.length
	);

	if (crc_wire == crc_calc) {
		adapter_process_frame(&header, payload);
	} else {
		forward_append(raw, raw_len, parser->buf, parser->len);
	}

	parser->len = 0;
	parser->expected = 0;
}

static void usb_to_uart_task(void *arg)
{
	uint8_t buf[BRIDGE_CHUNK_SIZE];
	uint8_t raw[BRIDGE_CHUNK_SIZE];
	int raw_len = 0;
	AdapterParser parser = {0};
	(void)arg;

	for (;;) {
		int n = usb_serial_jtag_read_bytes(buf, sizeof buf, portMAX_DELAY);
		if (n <= 0) continue;

		for (int i = 0; i < n; ++i) {
			adapter_parser_feed(&parser, buf[i], raw, &raw_len);
		}
		if (raw_len) {
			usb_forward_raw(raw, raw_len);
			raw_len = 0;
		}
	}
}

static void uart_to_usb_task(void *arg)
{
	uint8_t buf[BRIDGE_CHUNK_SIZE];
	(void)arg;

	for (;;) {
		if (link_owner_get() != LINK_OWNER_USB) {
			vTaskDelay(pdMS_TO_TICKS(5));
			continue;
		}

		if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(20)) != pdTRUE) continue;
		if (link_owner_get() != LINK_OWNER_USB) {
			xSemaphoreGive(uart_mutex);
			continue;
		}

		int n = uart_read_bytes(
			CONFIG_UART_NUM,
			buf,
			sizeof buf,
			pdMS_TO_TICKS(10)
		);
		xSemaphoreGive(uart_mutex);

		if (n <= 0) continue;
		link_owner_touch(LINK_OWNER_USB);
		setled(1);
		usb_write_all(buf, n);
		setled(0);
	}
}

/* ---------------- Original Wi-Fi Link services ---------------- */

static int processdns(uint8_t *buf, int len)
{
	if (len < (int)sizeof(DNSHeader)) return 0;
	DNSHeader *header = (DNSHeader *)buf;
	int flags = ntohs(header->flags);
	if (flags & 0x7800) return 0;
	uint8_t *cur = buf + sizeof *header;
	uint8_t *end = buf + len;
	uint8_t *pos = end;
	int cnt = 0;
	for (int i = 0, n = ntohs(header->qucnt); i < n; ++i) {
		uint8_t *name = cur;
		for (int s; cur < end && (s = *cur++); cur += s);
		if (cur + 4 > end) return 0;
		int type = ntohs(*(uint16_t *)cur);
		int class = ntohs(*(uint16_t *)(cur + 2));
		cur += 4;
		if (type != 1 || class != 1) continue;
		DNSAnswer *answer = (DNSAnswer *)pos;
		pos += sizeof *answer;
		if (pos - buf > 512) return 0;
		answer->name = htons(0xc000 | (name - buf));
		answer->type = htons(type);
		answer->class = htons(class);
		answer->ttl = htonl(60);
		answer->len = htons(4);
		answer->addr = htonl(0xc0a80401);
		++cnt;
	}
	memmove(cur, end, pos - end);
	header->flags = htons(flags | 0x8000);
	header->ancnt = htons(cnt);
	header->nscnt = 0;
	header->arcnt = 0;
	return pos - end + cur - buf;
}

static int recvbuf(uint8_t *buf, int len, int all)
{
	int pos = 0;
	while (len) {
		size_t size;
		uart_get_buffered_data_len(CONFIG_UART_NUM, &size);
		if (!size) {
			setled(1);
			uart_event_t event;
			if (!xQueueReceive(queue, &event, pdMS_TO_TICKS(200)) || event.type != UART_DATA) {
				setled(0);
				return 0;
			}
			size = event.size;
			setled(0);
		}
		if (size > (size_t)len) size = len;
		uart_read_bytes(CONFIG_UART_NUM, buf, size, portMAX_DELAY);
		buf += size;
		pos += size;
		len -= size;
		if (all) continue;
		if (pos >= 3 && !memcmp(buf - 3, "OK\n", 3)) break;
		if (pos >= 6 && !memcmp(buf - 6, "ERROR\n", 6)) break;
	}
	return pos;
}

static void sendbuf(const uint8_t *buf, int len)
{
	xQueueReset(queue);
	uart_flush(CONFIG_UART_NUM);
	uart_write_bytes(CONFIG_UART_NUM, buf, len);
	link_owner_touch(LINK_OWNER_WIFI);
}

static int recvval(void)
{
	uint8_t buf[2];
	return recvbuf(buf, 2, 1) && (buf[0] ^ buf[1]) == 0xff ? buf[0] : -1;
}

static void sendval(int val)
{
	uint8_t buf[2] = {val, ~val};
	sendbuf(buf, 2);
}

static int recvdata(uint8_t *buf)
{
	int cnt = recvval();
	if (cnt == -1) return -1;
	int len = (cnt + 1) << 2;
	uint32_t crc;
	return recvbuf(buf, len, 1) &&
		recvbuf((uint8_t *)&crc, 4, 1) &&
		esp_crc32_le(0, buf, len) == crc ? len : -1;
}

static void senddata(const uint8_t *buf, int len)
{
	uint32_t crc = esp_crc32_le(0, buf, len);
	sendval((len >> 2) - 1);
	sendbuf(buf, len);
	sendbuf((uint8_t *)&crc, 4);
}

static char *checkcmd(uint8_t *buf, int len, const char *cmd)
{
	int n = strlen(cmd);
	return len < n || memcmp(buf, cmd, n) ||
		(buf[n] != ' ' && buf[n] != '\n') ? 0 : (char *)buf + n;
}

static void notify(httpd_req_t *req, const char *key, int val)
{
	char buf[32];
	httpd_ws_frame_t frame = {
		.type = HTTPD_WS_TYPE_TEXT,
		.payload = (uint8_t *)buf,
		.len = sprintf(buf, "%s %d\n", key, val),
	};
	httpd_ws_send_frame(req, &frame);
}

static esp_err_t http404handler(httpd_req_t *req, httpd_err_code_t err)
{
	(void)err;
	httpd_resp_set_status(req, "302 Temporary Redirect");
	httpd_resp_set_hdr(req, "Location", "/");
	httpd_resp_send(req, "Redirect", 8);
	return ESP_OK;
}

static esp_err_t roothandler(httpd_req_t *req)
{
	const char *buf = req->uri;
	ssize_t len;
	if (!strcmp(buf, "/")) {
		buf = _binary_root_html_gz_start;
		len = _binary_root_html_gz_end - _binary_root_html_gz_start;
	}
#define XX(lang) \
	else if (!strcmp(buf, "/?" #lang)) { \
		buf = _binary_root_##lang##_json_gz_start; \
		len = _binary_root_##lang##_json_gz_end - _binary_root_##lang##_json_gz_start; \
	}
LANG_LIST(XX)
#undef XX
	else {
		httpd_resp_set_status(req, "400 Bad Request");
		httpd_resp_send(req, 0, 0);
		return ESP_OK;
	}
	httpd_resp_set_type(req, buf == _binary_root_html_gz_start ? "text/html" : "text/json");
	httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
	httpd_resp_send(req, buf, len);
	return ESP_OK;
}

static bool wifi_esc_lock(void)
{
	if (!link_owner_acquire(LINK_OWNER_WIFI)) return false;
	if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
		link_owner_release(LINK_OWNER_WIFI);
		return false;
	}
	link_owner_touch(LINK_OWNER_WIFI);
	return true;
}

static void wifi_esc_unlock(bool release_owner)
{
	xSemaphoreGive(uart_mutex);
	if (release_owner) link_owner_release(LINK_OWNER_WIFI);
	else link_owner_touch(LINK_OWNER_WIFI);
}

static esp_err_t wshandler(httpd_req_t *req)
{
	if (req->method == HTTP_GET) return ESP_OK;

	httpd_ws_frame_t frame = {0};
	if (httpd_ws_recv_frame(req, &frame, 0)) return ESP_FAIL;
	int len = frame.len;
	int res = -1;

	switch (frame.type) {
		case HTTPD_WS_TYPE_TEXT: {
			uint8_t buf[1200];
			frame.payload = buf;
			if (httpd_ws_recv_frame(req, &frame, sizeof buf)) return ESP_FAIL;
			char *arg;

			if ((arg = checkcmd(buf, len, "_update"))) {
				if (*arg == ' ') ++arg;
				update_size = strtol(arg, &arg, 0);
				update_boot = strtol(arg, &arg, 0);
				update_wrp = strtol(arg, &arg, 0);
				if (*arg != '\n') goto text_done;
				if (!link_owner_acquire(LINK_OWNER_WIFI)) {
					res = -2001;
					goto text_done;
				}
				update_ofs = 0;
				update_idx = 0;
				update_active = true;
				link_owner_touch(LINK_OWNER_WIFI);
				res = 0;
				goto text_done;
			}

			if (!wifi_esc_lock()) {
				res = -2001;
				goto text_done;
			}

			bool locked = true;

			if ((arg = checkcmd(buf, len, "_probe"))) {
				if (*arg != '\n') goto text_unlock_done;
				sendval(CMD_PROBE);
				res = recvval();
				goto text_unlock_done;
			}

			if ((arg = checkcmd(buf, len, "_info"))) {
				if (*arg != '\n') goto text_unlock_done;
				uint8_t info[52];
				sendval(CMD_INFO);
				if (recvdata(info) != 32) goto text_unlock_done;
				sendval(CMD_READ);
				sendval(0);
				sendval(4);
				if (recvdata(info + 32) != 20) goto text_unlock_done;
				len += sprintf(
					(char *)buf + len,
					"%d %d %X\n",
					info[0],
					info[1],
					info[2] | info[3] << 8 | info[4] << 16 | info[5] << 24
				);
				len += sprintf(
					(char *)buf + len,
					info[32] == 0xea && info[33] == 0x32 ? "%d\n%s\n" : "\n\n",
					info[34],
					info + 36
				);
				res = 0;
				goto text_unlock_done;
			}

			if ((arg = checkcmd(buf, len, "_setwrp"))) {
				int val = strtol(arg, &arg, 0);
				if (*arg != '\n') goto text_unlock_done;
				sendval(CMD_SETWRP);
				sendval(val);
				res = recvval();
				goto text_unlock_done;
			}

			sendbuf(buf, len);
			if (checkcmd(buf, len, "play")) {
				wifi_esc_unlock(true);
				return ESP_OK;
			}
			int pos = recvbuf(buf + len, sizeof buf - len, 0);
			if (!pos) {
				wifi_esc_unlock(true);
				return ESP_FAIL;
			}
			frame.len += pos;
			wifi_esc_unlock(true);
			return httpd_ws_send_frame(req, &frame);

text_unlock_done:
			if (locked) wifi_esc_unlock(true);

text_done:
			len += sprintf((char *)buf + len, res ? "ERROR\n" : "OK\n");
			frame.len = len;
			return httpd_ws_send_frame(req, &frame);
		}

		case HTTPD_WS_TYPE_BINARY: {
			ESP_LOGI(
				"httpd_ws",
				"Updating... [size %d, boot %d, wrp 0x%02x, ofs %d, len %d]",
				update_size,
				update_boot,
				update_wrp,
				update_ofs,
				len
			);

			if (!update_active || !link_owner_acquire(LINK_OWNER_WIFI)) {
				notify(req, "_result", -2001);
				return ESP_OK;
			}

			if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
				notify(req, "_result", -2001);
				link_owner_release(LINK_OWNER_WIFI);
				update_active = false;
				return ESP_OK;
			}

			uint8_t *buf = 0;
			int frame_len = len;
			int transfer_len = (frame_len + 3) & ~3;
			if (
				update_ofs + frame_len > update_size ||
				(update_boot && frame_len > 4096) ||
				!transfer_len
			) {
				res = -1001;
				goto binary_error;
			}
			if (
				update_boot &&
				!(transfer_len & 1023) &&
				transfer_len != 4096
			) {
				transfer_len += 4;
			}

			if (!(buf = malloc(transfer_len))) {
				res = -1002;
				goto binary_error;
			}
			frame.payload = memset(buf, 0xff, transfer_len);
			if (httpd_ws_recv_frame(req, &frame, transfer_len)) {
				res = -1003;
				goto binary_error;
			}
			len = transfer_len;

			if (update_boot) {
				update_size = 0;
				sendval(CMD_UPDATE);
				for (int pos = 0; pos < len; pos += 1024) {
					notify(req, "_status", pos * 100 / len);
					senddata(buf + pos, min(len - pos, 1024));
					if ((res = recvval())) goto binary_error;
				}
				if ((res = recvval())) goto binary_error;
			} else {
				static uint8_t sig[2048];
				int pos = 0;
				int cnt = (update_size + 1023) >> 10;
				if (cnt > 2 && !update_ofs) {
					static const uint8_t dummy[] = {
						0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
					};
					for (int num = 0; num < 2; ++num) {
						sendval(CMD_WRITE);
						sendval(num);
						senddata(dummy, sizeof dummy);
						if ((res = recvval())) goto binary_error;
					}
					memcpy(sig, buf, 2048);
					pos = 2048;
					update_ofs = 2048;
				}
				for (
					;
					pos < len;
					pos += 1024, update_ofs += 1024, ++update_idx
				) {
					notify(req, "_status", update_idx * 100 / cnt);
					sendval(CMD_WRITE);
					sendval(update_ofs >> 10);
					senddata(buf + pos, min(len - pos, 1024));
					if ((res = recvval())) goto binary_error;
				}
				if (update_ofs < update_size) {
					free(buf);
					wifi_esc_unlock(false);
					return ESP_OK;
				}
				if (cnt > 2) {
					for (pos = 0; pos < 2048; pos += 1024, ++update_idx) {
						notify(req, "_status", update_idx * 100 / cnt);
						sendval(CMD_WRITE);
						sendval(pos >> 10);
						senddata(sig + pos, 1024);
						if ((res = recvval())) goto binary_error;
					}
				}
			}

			if (update_wrp) {
				sendval(CMD_SETWRP);
				sendval(update_wrp);
				if ((res = recvval())) goto binary_error;
			}

			notify(req, "_status", 100);
			res = 0;

binary_error:
			notify(req, "_result", res);
			free(buf);
			update_active = false;
			wifi_esc_unlock(true);
			ESP_LOGI("httpd_ws", "Update completed with result %d", res);
			return ESP_OK;
		}

		default:
			ESP_LOGE("httpd_ws", "Unrecognized frame type %d", frame.type);
			return ESP_FAIL;
	}
}

static void addhandler(const char *path, esp_err_t (*handler)(httpd_req_t *))
{
	const httpd_uri_t uri = {
		.uri = path,
		.method = HTTP_GET,
		.handler = handler,
		.is_websocket = handler == wshandler,
	};
	ESP_ERROR_CHECK(httpd_register_uri_handler(server, &uri));
}

static void connhandler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
	(void)arg;
	(void)base;
	(void)id;
	ESP_LOGI("httpd", "Socket %d connected", *(int *)data);
}

static void disconnhandler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
	(void)arg;
	(void)base;
	(void)id;
	ESP_LOGI("httpd", "Socket %d disconnected", *(int *)data);
	update_active = false;
	link_owner_release(LINK_OWNER_WIFI);
}

static void dns_task(void *arg)
{
	(void)arg;
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	if (fd == -1) {
		ESP_LOGE("dns", "socket() failed: %s", strerror(errno));
		vTaskDelete(NULL);
		return;
	}

	struct sockaddr_in sa = {
		.sin_family = AF_INET,
		.sin_port = htons(53),
	};
	socklen_t sl = sizeof sa;

	if (bind(fd, (struct sockaddr *)&sa, sl) == -1) {
		ESP_LOGE("dns", "bind() failed: %s", strerror(errno));
		close(fd);
		vTaskDelete(NULL);
		return;
	}

	for (;;) {
		uint8_t buf[512];
		int len1 = recvfrom(fd, buf, sizeof buf - 1, 0, (struct sockaddr *)&sa, &sl);
		if (len1 == -1) {
			ESP_LOGE("dns", "recvfrom() failed: %s", strerror(errno));
			break;
		}
		int len2 = processdns(buf, len1);
		if (!len2) continue;
		if (sendto(fd, buf, len2, 0, (struct sockaddr *)&sa, sl) == -1) {
			ESP_LOGE("dns", "sendto() failed: %s", strerror(errno));
		}
	}
	close(fd);
	vTaskDelete(NULL);
}

static void start_wifi_services(void)
{
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_ap();

	wifi_init_config_t wicfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wicfg));

	wifi_config_t wcfg = {
		.ap = {
			.ssid = SSID,
			.ssid_len = sizeof SSID - 1,
			.max_connection = 1,
			.authmode = WIFI_AUTH_OPEN,
		},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wcfg));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_ERROR_CHECK(mdns_init());
	ESP_ERROR_CHECK(mdns_hostname_set(HOSTNAME));

	httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
	hcfg.max_open_sockets = CONFIG_LWIP_MAX_SOCKETS - 3;
	hcfg.lru_purge_enable = true;
	ESP_ERROR_CHECK(httpd_start(&server, &hcfg));
	ESP_ERROR_CHECK(httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http404handler));
	addhandler("/", roothandler);
	addhandler("/ws", wshandler);

	ESP_ERROR_CHECK(
		esp_event_handler_register(
			ESP_HTTP_SERVER_EVENT,
			HTTP_SERVER_EVENT_ON_CONNECTED,
			&connhandler,
			0
		)
	);
	ESP_ERROR_CHECK(
		esp_event_handler_register(
			ESP_HTTP_SERVER_EVENT,
			HTTP_SERVER_EVENT_DISCONNECTED,
			&disconnhandler,
			0
		)
	);

	wifi_active = true;
	xTaskCreate(dns_task, "dns", 4096, NULL, 5, NULL);
}

void app_main(void)
{
	gpio_set_direction(CONFIG_LED_PIN, GPIO_MODE_OUTPUT);
	setled(1);

	esp_log_level_set("httpd_uri", ESP_LOG_ERROR);
	esp_log_level_set("httpd_txrx", ESP_LOG_ERROR);
	esp_log_level_set("httpd_parse", ESP_LOG_ERROR);

	owner_mutex = xSemaphoreCreateMutex();
	uart_mutex = xSemaphoreCreateMutex();
	usb_tx_mutex = xSemaphoreCreateMutex();
	ESP_ERROR_CHECK(
		owner_mutex && uart_mutex && usb_tx_mutex ? ESP_OK : ESP_ERR_NO_MEM
	);

	ESP_ERROR_CHECK(link_config_init());

	uart_config_t uart_cfg = {
		.baud_rate = 38400,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_DEFAULT,
	};

	ESP_ERROR_CHECK(
		uart_driver_install(
			CONFIG_UART_NUM,
			UART_RX_BUFFER_SIZE,
			0,
			10,
			&queue,
			0
		)
	);
	ESP_ERROR_CHECK(uart_param_config(CONFIG_UART_NUM, &uart_cfg));
	ESP_ERROR_CHECK(
		uart_set_pin(
			CONFIG_UART_NUM,
			CONFIG_UART_TX,
			CONFIG_UART_RX,
			UART_PIN_NO_CHANGE,
			UART_PIN_NO_CHANGE
		)
	);
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

	if (wifi_configured) start_wifi_services();

	setled(0);
}
