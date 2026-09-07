#include "signal_generator.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define PWM_MIN_FREQ_HZ 50
#define PWM_MAX_FREQ_HZ 490
#define PWM_MIN_PULSE_US 500
#define PWM_MAX_PULSE_US 2500
#define PWM_DUTY_BITS 14
#define PWM_DUTY_MAX (1U << PWM_DUTY_BITS)

#define DSHOT_RMT_RESOLUTION_HZ 20000000U
#define DSHOT_SYMBOL_COUNT 17
#define DSHOT_MIN_RATE_HZ 50
#define DSHOT_MAX_RATE_HZ 4000

#define SIGNAL_WATCHDOG_TASK_STACK 3072
#define SIGNAL_DSHOT_TASK_STACK 3072
#define SIGNAL_TASK_PRIORITY 9

static SemaphoreHandle_t signal_mutex;
static TaskHandle_t dshot_task_handle;
static esp_timer_handle_t dshot_timer;

static int uart_num_cfg;
static int uart_tx_gpio_cfg;
static int uart_rx_gpio_cfg;
static int signal_gpio_cfg;

static signal_generator_status_t current_status;
static int64_t last_keepalive_us;

static rmt_channel_handle_t rmt_channel;
static rmt_encoder_handle_t rmt_encoder;
static rmt_symbol_word_t dshot_symbols[DSHOT_SYMBOL_COUNT];

static void restore_uart_locked(void)
{
    gpio_reset_pin(signal_gpio_cfg);
    gpio_set_direction(uart_tx_gpio_cfg, GPIO_MODE_INPUT);
    uart_set_pin(
        uart_num_cfg,
        uart_tx_gpio_cfg,
        uart_rx_gpio_cfg,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
    uart_flush_input(uart_num_cfg);
}

static void stop_locked(void)
{
    if (dshot_timer && esp_timer_is_active(dshot_timer)) {
        esp_timer_stop(dshot_timer);
    }

    if (current_status.mode == SIGNAL_MODE_PWM) {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    }

    if (rmt_channel) {
        rmt_disable(rmt_channel);
        if (rmt_encoder) {
            rmt_del_encoder(rmt_encoder);
            rmt_encoder = NULL;
        }
        rmt_del_channel(rmt_channel);
        rmt_channel = NULL;
    }

    memset(&current_status, 0, sizeof current_status);
    current_status.mode = SIGNAL_MODE_UART;
    current_status.gpio = (uint8_t)signal_gpio_cfg;
    last_keepalive_us = 0;
    restore_uart_locked();
}

static void prepare_generator_gpio_locked(void)
{
    uart_flush_input(uart_num_cfg);
    gpio_set_direction(uart_tx_gpio_cfg, GPIO_MODE_INPUT);
    gpio_set_pull_mode(uart_tx_gpio_cfg, GPIO_FLOATING);
}

static uint16_t dshot_make_frame(uint16_t value, bool telemetry)
{
    uint16_t packet = (uint16_t)((value & 0x07ffU) << 1) | (telemetry ? 1U : 0U);
    uint16_t csum_data = packet;
    uint16_t csum = 0;
    for (int i = 0; i < 3; ++i) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0x0fU;
    return (uint16_t)((packet << 4) | csum);
}

static void dshot_build_symbols(uint16_t speed, uint16_t value, bool telemetry)
{
    /* speed is expressed as DShot150/300/600, i.e. in kbit/s. */
    uint32_t bit_rate_hz = (uint32_t)speed * 1000U;
    uint32_t bit_ticks =
        (DSHOT_RMT_RESOLUTION_HZ + bit_rate_hz / 2U) / bit_rate_hz;
    uint32_t high_one = (bit_ticks * 3U + 2U) / 4U;
    uint32_t high_zero = (bit_ticks * 3U + 4U) / 8U;
    uint16_t frame = dshot_make_frame(value, telemetry);

    for (int i = 0; i < 16; ++i) {
        bool one = (frame & (1U << (15 - i))) != 0;
        uint32_t high = one ? high_one : high_zero;
        dshot_symbols[i].level0 = 1;
        dshot_symbols[i].duration0 = high;
        dshot_symbols[i].level1 = 0;
        dshot_symbols[i].duration1 = bit_ticks - high;
    }

    /* Explicit low inter-frame gap: 20 us. */
    dshot_symbols[16].level0 = 0;
    dshot_symbols[16].duration0 = DSHOT_RMT_RESOLUTION_HZ / 100000U;
    dshot_symbols[16].level1 = 0;
    dshot_symbols[16].duration1 = DSHOT_RMT_RESOLUTION_HZ / 100000U;
}

static void dshot_timer_cb(void *arg)
{
    (void)arg;
    if (dshot_task_handle) xTaskNotifyGive(dshot_task_handle);
}

static void dshot_task(void *arg)
{
    (void)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        xSemaphoreTake(signal_mutex, portMAX_DELAY);
        if (current_status.mode == SIGNAL_MODE_DSHOT && rmt_channel && rmt_encoder) {
            rmt_transmit_config_t tx_cfg = {0};
            esp_err_t err = rmt_transmit(
                rmt_channel,
                rmt_encoder,
                dshot_symbols,
                sizeof dshot_symbols,
                &tx_cfg
            );
            if (err == ESP_OK) {
                rmt_tx_wait_all_done(rmt_channel, 10);
            }
        }
        xSemaphoreGive(signal_mutex);
    }
}

static void watchdog_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        xSemaphoreTake(signal_mutex, portMAX_DELAY);
        if (
            current_status.mode != SIGNAL_MODE_UART &&
            current_status.watchdog_ms > 0 &&
            last_keepalive_us > 0 &&
            esp_timer_get_time() - last_keepalive_us >=
                (int64_t)current_status.watchdog_ms * 1000
        ) {
            stop_locked();
        }
        xSemaphoreGive(signal_mutex);
    }
}

esp_err_t signal_generator_init(
    int uart_num,
    int uart_tx_gpio,
    int uart_rx_gpio,
    int signal_gpio
)
{
    uart_num_cfg = uart_num;
    uart_tx_gpio_cfg = uart_tx_gpio;
    uart_rx_gpio_cfg = uart_rx_gpio;
    signal_gpio_cfg = signal_gpio;

    signal_mutex = xSemaphoreCreateMutex();
    if (!signal_mutex) return ESP_ERR_NO_MEM;

    memset(&current_status, 0, sizeof current_status);
    current_status.mode = SIGNAL_MODE_UART;
    current_status.gpio = (uint8_t)signal_gpio_cfg;

    esp_timer_create_args_t timer_args = {
        .callback = dshot_timer_cb,
        .name = "dshot-periodic",
    };
    esp_err_t err = esp_timer_create(&timer_args, &dshot_timer);
    if (err != ESP_OK) return err;

    if (
        xTaskCreate(
            dshot_task,
            "dshot-tx",
            SIGNAL_DSHOT_TASK_STACK,
            NULL,
            SIGNAL_TASK_PRIORITY,
            &dshot_task_handle
        ) != pdPASS
    ) {
        return ESP_ERR_NO_MEM;
    }

    if (
        xTaskCreate(
            watchdog_task,
            "signal-watchdog",
            SIGNAL_WATCHDOG_TASK_STACK,
            NULL,
            SIGNAL_TASK_PRIORITY,
            NULL
        ) != pdPASS
    ) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t signal_generator_start_pwm(
    uint16_t freq_hz,
    uint16_t pulse_us,
    uint16_t watchdog_ms
)
{
    if (freq_hz < PWM_MIN_FREQ_HZ || freq_hz > PWM_MAX_FREQ_HZ) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pulse_us < PWM_MIN_PULSE_US || pulse_us > PWM_MAX_PULSE_US) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((uint32_t)pulse_us * freq_hz >= 1000000U) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(signal_mutex, portMAX_DELAY);
    stop_locked();
    prepare_generator_gpio_locked();

    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = freq_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        restore_uart_locked();
        xSemaphoreGive(signal_mutex);
        return err;
    }

    uint32_t duty = (uint32_t)(
        ((uint64_t)pulse_us * freq_hz * PWM_DUTY_MAX + 500000ULL) /
        1000000ULL
    );
    if (duty >= PWM_DUTY_MAX) duty = PWM_DUTY_MAX - 1;

    ledc_channel_config_t channel_cfg = {
        .gpio_num = signal_gpio_cfg,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = duty,
        .hpoint = 0,
    };
    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK) {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        restore_uart_locked();
        xSemaphoreGive(signal_mutex);
        return err;
    }

    memset(&current_status, 0, sizeof current_status);
    current_status.mode = SIGNAL_MODE_PWM;
    current_status.gpio = (uint8_t)signal_gpio_cfg;
    current_status.pwm_freq_hz = freq_hz;
    current_status.pwm_pulse_us = pulse_us;
    current_status.watchdog_ms = watchdog_ms;
    last_keepalive_us = esp_timer_get_time();

    xSemaphoreGive(signal_mutex);
    return ESP_OK;
}

esp_err_t signal_generator_start_dshot(
    uint16_t speed,
    uint16_t value,
    uint16_t rate_hz,
    bool telemetry,
    uint16_t watchdog_ms
)
{
    if (speed != 150 && speed != 300 && speed != 600) {
        return ESP_ERR_INVALID_ARG;
    }
    if (value > 2047) return ESP_ERR_INVALID_ARG;
    if (rate_hz < DSHOT_MIN_RATE_HZ || rate_hz > DSHOT_MAX_RATE_HZ) {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(signal_mutex, portMAX_DELAY);
    stop_locked();
    prepare_generator_gpio_locked();

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = signal_gpio_cfg,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 2,
    };
    esp_err_t err = rmt_new_tx_channel(&tx_cfg, &rmt_channel);
    if (err != ESP_OK) {
        restore_uart_locked();
        xSemaphoreGive(signal_mutex);
        return err;
    }

    rmt_copy_encoder_config_t encoder_cfg = {};
    err = rmt_new_copy_encoder(&encoder_cfg, &rmt_encoder);
    if (err != ESP_OK) {
        rmt_del_channel(rmt_channel);
        rmt_channel = NULL;
        restore_uart_locked();
        xSemaphoreGive(signal_mutex);
        return err;
    }

    err = rmt_enable(rmt_channel);
    if (err != ESP_OK) {
        rmt_del_encoder(rmt_encoder);
        rmt_encoder = NULL;
        rmt_del_channel(rmt_channel);
        rmt_channel = NULL;
        restore_uart_locked();
        xSemaphoreGive(signal_mutex);
        return err;
    }

    dshot_build_symbols(speed, value, telemetry);

    memset(&current_status, 0, sizeof current_status);
    current_status.mode = SIGNAL_MODE_DSHOT;
    current_status.gpio = (uint8_t)signal_gpio_cfg;
    current_status.dshot_speed = speed;
    current_status.dshot_value = value;
    current_status.dshot_rate_hz = rate_hz;
    current_status.dshot_telemetry = telemetry;
    current_status.watchdog_ms = watchdog_ms;
    last_keepalive_us = esp_timer_get_time();

    err = esp_timer_start_periodic(dshot_timer, 1000000ULL / rate_hz);
    if (err != ESP_OK) {
        stop_locked();
        xSemaphoreGive(signal_mutex);
        return err;
    }

    xSemaphoreGive(signal_mutex);
    return ESP_OK;
}

esp_err_t signal_generator_stop(void)
{
    if (!signal_mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(signal_mutex, portMAX_DELAY);
    stop_locked();
    xSemaphoreGive(signal_mutex);
    return ESP_OK;
}

void signal_generator_keepalive(void)
{
    if (!signal_mutex) return;
    xSemaphoreTake(signal_mutex, portMAX_DELAY);
    if (current_status.mode != SIGNAL_MODE_UART) {
        last_keepalive_us = esp_timer_get_time();
    }
    xSemaphoreGive(signal_mutex);
}

void signal_generator_get_status(signal_generator_status_t *status)
{
    if (!status) return;
    if (!signal_mutex) {
        memset(status, 0, sizeof *status);
        status->mode = SIGNAL_MODE_UART;
        return;
    }
    xSemaphoreTake(signal_mutex, portMAX_DELAY);
    *status = current_status;
    xSemaphoreGive(signal_mutex);
}

bool signal_generator_uart_mode(void)
{
    signal_generator_status_t status;
    signal_generator_get_status(&status);
    return status.mode == SIGNAL_MODE_UART;
}
