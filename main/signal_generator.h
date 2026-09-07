#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef enum {
    SIGNAL_MODE_UART = 0,
    SIGNAL_MODE_PWM = 1,
    SIGNAL_MODE_DSHOT = 2,
} signal_mode_t;

typedef struct {
    signal_mode_t mode;
    uint8_t gpio;
    uint16_t pwm_freq_hz;
    uint16_t pwm_pulse_us;
    uint16_t dshot_speed;
    uint16_t dshot_value;
    uint16_t dshot_rate_hz;
    bool dshot_telemetry;
    uint16_t watchdog_ms;
} signal_generator_status_t;

esp_err_t signal_generator_init(int uart_num, int uart_tx_gpio, int uart_rx_gpio, int signal_gpio);
esp_err_t signal_generator_start_pwm(uint16_t freq_hz, uint16_t pulse_us, uint16_t watchdog_ms);
esp_err_t signal_generator_start_dshot(uint16_t speed, uint16_t value, uint16_t rate_hz, bool telemetry, uint16_t watchdog_ms);
esp_err_t signal_generator_stop(void);
void signal_generator_keepalive(void);
void signal_generator_get_status(signal_generator_status_t *status);
bool signal_generator_uart_mode(void);
