#pragma once

#include <stdint.h>

#include <esp_err.h>
#include <uart_driver.h>
#include <driver/uart.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct uart_driver_config_t {
    uart_port_t num;
    uint32_t rx_pin;
    uint32_t tx_pin;
} uart_driver_config_t;

typedef void *uart_driver_handle_t;

uart_driver_handle_t uart_driver_init(uart_driver_config_t *config);
esp_err_t uart_driver_port_config(uart_driver_handle_t handle, uart_config_t *port_config);
int uart_driver_read(uart_driver_handle_t handle, void *buf, uint32_t buflen, uint32_t ms_to_wait);
int uart_driver_read_non_blocking(uart_driver_handle_t handle, void *buf, uint32_t buflen);
int uart_driver_write(uart_driver_handle_t handle, const void *buf, size_t size);

#ifdef __cplusplus
}
#endif