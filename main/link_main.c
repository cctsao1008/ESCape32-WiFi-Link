/*
 * ESCape32 Link integration entry point.
 *
 * The upstream-derived main.c remains the Wi-Fi/USB/UART service baseline.
 * This wrapper extends the USB adapter control plane with PWM/DShot signal
 * generation while keeping the existing Wi-Fi Web UI and ESCape32 transport.
 */

#include "driver/gpio.h"

/*
 * main.c contains legacy activity LED writes.  ESCape32 Link owns the
 * physical status LED through link_led_task(), so suppress those writes while
 * including the legacy implementation.  main.c has no other direct
 * gpio_set_level() use.
 */
static esp_err_t link_legacy_led_noop(gpio_num_t gpio_num, uint32_t level)
{
    (void)gpio_num;
    (void)level;
    return ESP_OK;
}

#define gpio_set_level link_legacy_led_noop
#define app_main legacy_app_main
#define usb_to_uart_task legacy_usb_to_uart_task
#define uart_to_usb_task legacy_uart_to_usb_task
#include "main.c"
#undef uart_to_usb_task
#undef usb_to_uart_task
#undef app_main
#undef gpio_set_level

#include "signal_generator.h"

#define ADAPTER_CMD_SIGNAL_GET       7
#define ADAPTER_CMD_SIGNAL_PWM       8
#define ADAPTER_CMD_SIGNAL_DSHOT     9
#define ADAPTER_CMD_SIGNAL_STOP      10
#define ADAPTER_CMD_SIGNAL_KEEPALIVE 11
#define ADAPTER_CMD_GPIO_GET         12
#define ADAPTER_CMD_GPIO_SET         13
#define ADAPTER_CMD_GPIO_RELEASE     14
#define ADAPTER_CMD_SIGNAL_DSHOT_COMMAND 15

#define SIGNAL_STATUS_UART   0
#define SIGNAL_STATUS_PWM    1
#define SIGNAL_STATUS_DSHOT  2

#define LINK_LED_TICK_MS       125
#define LINK_LED_TASK_STACK    2048
#define LINK_LED_TASK_PRIORITY 5
#define LINK_LED_ACTIVITY_TICKS 2

typedef struct __attribute__((__packed__)) {
    uint16_t freq_hz;
    uint16_t pulse_us;
    uint16_t watchdog_ms;
} SignalPwmRequest;

typedef struct __attribute__((__packed__)) {
    uint16_t speed;
    uint16_t value;
    uint16_t rate_hz;
    uint16_t watchdog_ms;
    uint8_t telemetry;
} SignalDshotRequest;

typedef struct __attribute__((__packed__)) {
    uint16_t speed;
    uint8_t command;
} SignalDshotCommandRequest;

typedef struct __attribute__((__packed__)) {
    uint8_t status;
    uint8_t mode;
    uint8_t gpio;
    uint8_t telemetry;
    uint16_t pwm_freq_hz;
    uint16_t pwm_pulse_us;
    uint16_t dshot_speed;
    uint16_t dshot_value;
    uint16_t dshot_rate_hz;
    uint16_t watchdog_ms;
} SignalInfoPayload;

typedef struct __attribute__((__packed__)) {
    uint8_t status;
    uint8_t gpio;
    uint8_t level;
    uint8_t override_active;
} GpioInfoPayload;

typedef struct {
    uint8_t buf[8 + sizeof(AdapterHeader) + ADAPTER_MAX_PAYLOAD + 4];
    size_t len;
    size_t expected;
} LinkAdapterParser;

static volatile uint8_t link_led_activity_ticks;
static volatile bool link_gpio_override_active;
static volatile uint8_t link_gpio_override_pin;

static void link_led_write(bool on)
{
    int level = on ? 1 : 0;
#ifdef CONFIG_LED_INV
    level = !level;
#endif
    gpio_set_level(CONFIG_LED_PIN, level);
}

static void link_led_note_activity(void)
{
    link_led_activity_ticks = LINK_LED_ACTIVITY_TICKS;
}

static void link_led_task(void *arg)
{
    (void)arg;
    uint32_t phase = 0;

    for (;;) {
        signal_generator_status_t sig;
        signal_generator_get_status(&sig);

        if (
            link_gpio_override_active &&
            link_gpio_override_pin == (uint8_t)CONFIG_LED_PIN
        ) {
            ++phase;
            vTaskDelay(pdMS_TO_TICKS(LINK_LED_TICK_MS));
            continue;
        }

        bool on = false;
        if (wifi_configured != wifi_active) {
            /* Reboot/config transition: double blink every 2 seconds. */
            uint32_t p = phase % 16U;
            on = p == 0U || p == 2U;
        } else if (sig.mode == SIGNAL_MODE_PWM) {
            /* PWM generator: 2 Hz, 50% duty. */
            on = (phase % 4U) < 2U;
        } else if (sig.mode == SIGNAL_MODE_DSHOT) {
            /* DShot generator: 4 Hz, 50% duty. */
            on = (phase % 2U) == 0U;
        } else if (link_led_activity_ticks > 0) {
            /* UART/USB activity: visible ~250 ms pulse. */
            on = true;
            --link_led_activity_ticks;
        } else if (wifi_active) {
            /* Wi-Fi active and otherwise idle: short heartbeat every 2 seconds. */
            on = (phase % 16U) == 0U;
        }

        link_led_write(on);
        ++phase;
        vTaskDelay(pdMS_TO_TICKS(LINK_LED_TICK_MS));
    }
}

static void link_fill_signal_info(SignalInfoPayload *payload, uint8_t status)
{
    signal_generator_status_t sig;
    signal_generator_get_status(&sig);
    memset(payload, 0, sizeof *payload);
    payload->status = status;
    payload->mode = (uint8_t)sig.mode;
    payload->gpio = sig.gpio;
    payload->telemetry = sig.dshot_telemetry ? 1 : 0;
    payload->pwm_freq_hz = sig.pwm_freq_hz;
    payload->pwm_pulse_us = sig.pwm_pulse_us;
    payload->dshot_speed = sig.dshot_speed;
    payload->dshot_value = sig.dshot_value;
    payload->dshot_rate_hz = sig.dshot_rate_hz;
    payload->watchdog_ms = sig.watchdog_ms;
}

static void link_send_signal_response(
    uint8_t command,
    uint32_t sequence,
    uint8_t status
)
{
    uint8_t frame[8 + sizeof(AdapterHeader) + sizeof(SignalInfoPayload) + 4];
    AdapterHeader header = {
        .version = ADAPTER_PROTOCOL_VERSION,
        .command = command | 0x80,
        .length = sizeof(SignalInfoPayload),
        .sequence = sequence,
    };
    SignalInfoPayload payload;
    link_fill_signal_info(&payload, status);

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

static uint8_t link_signal_start_pwm(const SignalPwmRequest *req)
{
    if (!link_owner_acquire(LINK_OWNER_USB)) return ADAPTER_STATUS_BUSY;
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        link_owner_release(LINK_OWNER_USB);
        return ADAPTER_STATUS_BUSY;
    }

    esp_err_t err = signal_generator_start_pwm(
        req->freq_hz,
        req->pulse_us,
        req->watchdog_ms
    );
    xSemaphoreGive(uart_mutex);

    if (err != ESP_OK) {
        link_owner_release(LINK_OWNER_USB);
        return err == ESP_ERR_INVALID_ARG ?
            ADAPTER_STATUS_BAD_ARG : ADAPTER_STATUS_INTERNAL;
    }

    link_owner_touch(LINK_OWNER_USB);
    return ADAPTER_STATUS_OK;
}

static uint8_t link_signal_start_dshot(const SignalDshotRequest *req)
{
    if (req->telemetry > 1) return ADAPTER_STATUS_BAD_ARG;
    if (!link_owner_acquire(LINK_OWNER_USB)) return ADAPTER_STATUS_BUSY;
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        link_owner_release(LINK_OWNER_USB);
        return ADAPTER_STATUS_BUSY;
    }

    esp_err_t err = signal_generator_start_dshot(
        req->speed,
        req->value,
        req->rate_hz,
        req->telemetry != 0,
        req->watchdog_ms
    );
    xSemaphoreGive(uart_mutex);

    if (err != ESP_OK) {
        link_owner_release(LINK_OWNER_USB);
        return err == ESP_ERR_INVALID_ARG ?
            ADAPTER_STATUS_BAD_ARG : ADAPTER_STATUS_INTERNAL;
    }

    link_owner_touch(LINK_OWNER_USB);
    return ADAPTER_STATUS_OK;
}

static uint8_t link_signal_send_dshot_command(const SignalDshotCommandRequest *req)
{
    if (req->command < 1 || req->command > 47) return ADAPTER_STATUS_BAD_ARG;
    if (!link_owner_acquire(LINK_OWNER_USB)) return ADAPTER_STATUS_BUSY;
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        link_owner_release(LINK_OWNER_USB);
        return ADAPTER_STATUS_BUSY;
    }
    esp_err_t err = signal_generator_send_dshot_command(req->speed, req->command);
    xSemaphoreGive(uart_mutex);
    link_owner_release(LINK_OWNER_USB);
    if (err == ESP_OK) return ADAPTER_STATUS_OK;
    return err == ESP_ERR_INVALID_ARG ? ADAPTER_STATUS_BAD_ARG : ADAPTER_STATUS_INTERNAL;
}

static uint8_t link_signal_stop(void)
{
    if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(250)) != pdTRUE) {
        return ADAPTER_STATUS_BUSY;
    }
    esp_err_t err = signal_generator_stop();
    xSemaphoreGive(uart_mutex);
    link_owner_release(LINK_OWNER_USB);
    return err == ESP_OK ? ADAPTER_STATUS_OK : ADAPTER_STATUS_INTERNAL;
}

static uint8_t link_signal_keepalive(void)
{
    if (signal_generator_uart_mode()) return ADAPTER_STATUS_BAD_ARG;
    if (!link_owner_acquire(LINK_OWNER_USB)) return ADAPTER_STATUS_BUSY;
    signal_generator_keepalive();
    link_owner_touch(LINK_OWNER_USB);
    return ADAPTER_STATUS_OK;
}

static bool link_gpio_allowed(uint8_t gpio)
{
    /* Keep diagnostic writes away from UART, SIG and native USB pins. */
    return gpio == (uint8_t)CONFIG_LED_PIN;
}

static void link_fill_gpio_info(
    GpioInfoPayload *payload,
    uint8_t status,
    uint8_t gpio
)
{
    memset(payload, 0, sizeof *payload);
    payload->status = status;
    payload->gpio = gpio;
    if (link_gpio_allowed(gpio)) {
        payload->level = gpio_get_level((gpio_num_t)gpio) ? 1 : 0;
        payload->override_active =
            link_gpio_override_active && link_gpio_override_pin == gpio ? 1 : 0;
    }
}

static void link_send_gpio_response(
    uint8_t command,
    uint32_t sequence,
    uint8_t status,
    uint8_t gpio
)
{
    uint8_t frame[8 + sizeof(AdapterHeader) + sizeof(GpioInfoPayload) + 4];
    AdapterHeader header = {
        .version = ADAPTER_PROTOCOL_VERSION,
        .command = command | 0x80,
        .length = sizeof(GpioInfoPayload),
        .sequence = sequence,
    };
    GpioInfoPayload payload;
    link_fill_gpio_info(&payload, status, gpio);

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

static void link_process_gpio_frame(
    const AdapterHeader *header,
    const uint8_t *payload
)
{
    uint8_t status = ADAPTER_STATUS_OK;
    uint8_t gpio = header->length > 0 ? payload[0] : 0xffU;

    switch (header->command) {
        case ADAPTER_CMD_GPIO_GET:
            if (header->length != 1 || !link_gpio_allowed(gpio)) {
                status = ADAPTER_STATUS_BAD_ARG;
            }
            break;

        case ADAPTER_CMD_GPIO_SET:
            if (
                header->length != 2 ||
                !link_gpio_allowed(gpio) ||
                payload[1] > 1
            ) {
                status = ADAPTER_STATUS_BAD_ARG;
                break;
            }
            link_gpio_override_pin = gpio;
            link_gpio_override_active = true;
            gpio_set_direction((gpio_num_t)gpio, GPIO_MODE_INPUT_OUTPUT);
            gpio_set_level((gpio_num_t)gpio, payload[1]);
            break;

        case ADAPTER_CMD_GPIO_RELEASE:
            if (header->length != 1 || !link_gpio_allowed(gpio)) {
                status = ADAPTER_STATUS_BAD_ARG;
                break;
            }
            if (link_gpio_override_active && link_gpio_override_pin == gpio) {
                link_gpio_override_active = false;
            }
            if (gpio == (uint8_t)CONFIG_LED_PIN) {
                gpio_set_direction(CONFIG_LED_PIN, GPIO_MODE_INPUT_OUTPUT);
            }
            break;

        default:
            status = ADAPTER_STATUS_BAD_ARG;
            break;
    }

    link_send_gpio_response(header->command, header->sequence, status, gpio);
}

static bool link_is_gpio_command(uint8_t command)
{
    return command >= ADAPTER_CMD_GPIO_GET &&
        command <= ADAPTER_CMD_GPIO_RELEASE;
}

static void link_process_signal_frame(
    const AdapterHeader *header,
    const uint8_t *payload
)
{
    uint8_t status = ADAPTER_STATUS_OK;

    switch (header->command) {
        case ADAPTER_CMD_SIGNAL_GET:
            if (header->length != 0) status = ADAPTER_STATUS_BAD_ARG;
            break;

        case ADAPTER_CMD_SIGNAL_PWM: {
            if (header->length != sizeof(SignalPwmRequest)) {
                status = ADAPTER_STATUS_BAD_ARG;
                break;
            }
            SignalPwmRequest req;
            memcpy(&req, payload, sizeof req);
            status = link_signal_start_pwm(&req);
            break;
        }

        case ADAPTER_CMD_SIGNAL_DSHOT: {
            if (header->length != sizeof(SignalDshotRequest)) {
                status = ADAPTER_STATUS_BAD_ARG;
                break;
            }
            SignalDshotRequest req;
            memcpy(&req, payload, sizeof req);
            status = link_signal_start_dshot(&req);
            break;
        }

        case ADAPTER_CMD_SIGNAL_DSHOT_COMMAND: {
            if (header->length != sizeof(SignalDshotCommandRequest)) {
                status = ADAPTER_STATUS_BAD_ARG;
                break;
            }
            SignalDshotCommandRequest req;
            memcpy(&req, payload, sizeof req);
            status = link_signal_send_dshot_command(&req);
            break;
        }

        case ADAPTER_CMD_SIGNAL_STOP:
            if (header->length != 0) status = ADAPTER_STATUS_BAD_ARG;
            else status = link_signal_stop();
            break;

        case ADAPTER_CMD_SIGNAL_KEEPALIVE:
            if (header->length != 0) status = ADAPTER_STATUS_BAD_ARG;
            else status = link_signal_keepalive();
            break;

        default:
            status = ADAPTER_STATUS_BAD_ARG;
            break;
    }

    link_send_signal_response(header->command, header->sequence, status);
}

static bool link_is_signal_command(uint8_t command)
{
    return (
        command >= ADAPTER_CMD_SIGNAL_GET &&
        command <= ADAPTER_CMD_SIGNAL_KEEPALIVE
    ) || command == ADAPTER_CMD_SIGNAL_DSHOT_COMMAND;
}

static void link_forward_raw(const uint8_t *buf, int len)
{
    if (!signal_generator_uart_mode()) return;
    link_led_note_activity();
    usb_forward_raw(buf, len);
}

static void link_forward_append(
    uint8_t *raw,
    int *raw_len,
    const uint8_t *data,
    int len
)
{
    while (len > 0) {
        int room = BRIDGE_CHUNK_SIZE - *raw_len;
        int n = len < room ? len : room;
        memcpy(raw + *raw_len, data, n);
        *raw_len += n;
        data += n;
        len -= n;
        if (*raw_len == BRIDGE_CHUNK_SIZE) {
            link_forward_raw(raw, *raw_len);
            *raw_len = 0;
        }
    }
}

static void link_adapter_parser_feed(
    LinkAdapterParser *parser,
    uint8_t byte,
    uint8_t *raw,
    int *raw_len
)
{
    if (parser->len < sizeof adapter_request_magic) {
        if (byte == adapter_request_magic[parser->len]) {
            parser->buf[parser->len++] = byte;
            return;
        }

        if (parser->len) {
            link_forward_append(raw, raw_len, parser->buf, parser->len);
            parser->len = 0;
            parser->expected = 0;
            if (byte == adapter_request_magic[0]) {
                parser->buf[parser->len++] = byte;
                return;
            }
        }

        link_forward_append(raw, raw_len, &byte, 1);
        return;
    }

    parser->buf[parser->len++] = byte;
    if (parser->len == sizeof adapter_request_magic + sizeof(AdapterHeader)) {
        AdapterHeader header;
        memcpy(&header, parser->buf + sizeof adapter_request_magic, sizeof header);
        if (
            header.version != ADAPTER_PROTOCOL_VERSION ||
            header.length > ADAPTER_MAX_PAYLOAD
        ) {
            link_forward_append(raw, raw_len, parser->buf, parser->len);
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
    memcpy(
        &crc_wire,
        parser->buf + parser->expected - sizeof crc_wire,
        sizeof crc_wire
    );
    uint32_t crc_calc = esp_crc32_le(
        0,
        parser->buf + sizeof adapter_request_magic,
        sizeof(AdapterHeader) + header.length
    );

    if (crc_wire == crc_calc) {
        if (link_is_signal_command(header.command)) {
            link_process_signal_frame(&header, payload);
        } else if (link_is_gpio_command(header.command)) {
            link_process_gpio_frame(&header, payload);
        } else {
            adapter_process_frame(&header, payload);
        }
    } else {
        link_forward_append(raw, raw_len, parser->buf, parser->len);
    }

    parser->len = 0;
    parser->expected = 0;
}

static void link_usb_to_uart_task(void *arg)
{
    uint8_t buf[BRIDGE_CHUNK_SIZE];
    uint8_t raw[BRIDGE_CHUNK_SIZE];
    int raw_len = 0;
    LinkAdapterParser parser = {0};
    (void)arg;

    for (;;) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof buf, portMAX_DELAY);
        if (n <= 0) continue;

        for (int i = 0; i < n; ++i) {
            link_adapter_parser_feed(&parser, buf[i], raw, &raw_len);
        }
        if (raw_len) {
            link_forward_raw(raw, raw_len);
            raw_len = 0;
        }
    }
}

static void link_uart_to_usb_task(void *arg)
{
    uint8_t buf[BRIDGE_CHUNK_SIZE];
    (void)arg;

    for (;;) {
        if (!signal_generator_uart_mode()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (link_owner_get() != LINK_OWNER_USB) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(20)) != pdTRUE) continue;
        if (!signal_generator_uart_mode() || link_owner_get() != LINK_OWNER_USB) {
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
        link_led_note_activity();
        usb_write_all(buf, n);
    }
}

void app_main(void)
{
    gpio_set_direction(CONFIG_LED_PIN, GPIO_MODE_INPUT_OUTPUT);
    link_led_write(true);

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

    ESP_ERROR_CHECK(
        signal_generator_init(
            CONFIG_UART_NUM,
            CONFIG_UART_TX,
            CONFIG_UART_RX,
            CONFIG_UART_RX
        )
    );

    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = USB_DRIVER_BUFFER_SIZE,
        .tx_buffer_size = USB_DRIVER_BUFFER_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    BaseType_t res;
    res = xTaskCreate(
        link_usb_to_uart_task,
        "usb-to-link",
        BRIDGE_TASK_STACK_SIZE,
        NULL,
        BRIDGE_TASK_PRIORITY,
        NULL
    );
    ESP_ERROR_CHECK(res == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    res = xTaskCreate(
        link_uart_to_usb_task,
        "link-to-usb",
        BRIDGE_TASK_STACK_SIZE,
        NULL,
        BRIDGE_TASK_PRIORITY,
        NULL
    );
    ESP_ERROR_CHECK(res == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    if (wifi_configured) start_wifi_services();

    /* Boot indication ends here; the status task owns the LED from now on. */
    link_led_write(false);
    res = xTaskCreate(
        link_led_task,
        "link-led",
        LINK_LED_TASK_STACK,
        NULL,
        LINK_LED_TASK_PRIORITY,
        NULL
    );
    ESP_ERROR_CHECK(res == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
