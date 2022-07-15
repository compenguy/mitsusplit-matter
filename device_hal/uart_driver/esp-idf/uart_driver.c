// Copyright 2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License

#include <driver/uart.h>
#include <esp_log.h>

#include <uart_driver.h>

#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

static const char *TAG = "uart_driver_esp_idf";
static uart_port_t uart_num = UART_NUM_0;
// UART_RX/TX_SIZE is how many bits are needed to address the number of bytes in the FIFO
// e.g. 7 bits allows for 128 addressable bytes
// We'll set the rx/tx ring buffers to be 2x the size of the fifo, or 64 bytes, whichever is bigger
const int UART_TX_BUFSIZE = max(64, 2 * (1 << (UART_TX_SIZE + 1)));
const int UART_RX_BUFSIZE = max(64, 2 * (1 << (UART_RX_SIZE + 1)));

// handle is uart number + 1 to ensure non-zero result, because 0/NULL is reserved for error/uninitialized
static uart_port_t handle_to_uart_num(uart_driver_handle_t handle) {
    return (uart_port_t)handle - 1;
}

static uart_driver_handle_t uart_num_to_handle(uart_port_t uart_num) {
    return (uart_driver_handle_t)(uart_num + 1);
}

uart_driver_handle_t uart_driver_init(uart_driver_config_t *config)
{
    esp_err_t err = ESP_OK;

    uart_num = config->num;
    ESP_LOGI(TAG, "Initializing uart %d driver", uart_num);

    err = uart_driver_install(uart_num, UART_RX_BUFSIZE, UART_TX_BUFSIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed");
        return NULL;
    }

    ESP_LOGI(TAG, "Setting uart %d to use pins %d and %d", uart_num, config->tx_pin, config->rx_pin);
    err = uart_set_pin(uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return NULL;
    }

    return uart_num_to_handle(config->num);
}

esp_err_t uart_driver_port_config(uart_driver_handle_t handle, uart_config_t *config)
{
    uart_port_t uart_num = handle_to_uart_num(handle);
    esp_err_t err = ESP_OK;
    err = uart_param_config(uart_num, config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        return err;
    }

    return err;
}

// Block until the requested number of bytes have been read
int uart_driver_read(uart_driver_handle_t handle, void *buf, uint32_t buflen, uint32_t ms_to_wait)
{
    uart_port_t uart_num = handle_to_uart_num(handle);
    return uart_read_bytes(uart_num, buf, buflen, ms_to_wait / portTICK_PERIOD_MS);
}

// Read as many bytes as are already available, up to buflen bytes
int uart_driver_read_non_blocking(uart_driver_handle_t handle, void *buf, uint32_t buflen)
{
    uart_port_t uart_num = handle_to_uart_num(handle);
    size_t uart_buf_bytes = 0;
    esp_err_t err = uart_get_buffered_data_len(uart_num, &uart_buf_bytes);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_get_buffered_data_len failed");
        return -1;
    }
    return uart_driver_read(handle, buf, min(buflen, uart_buf_bytes), 0);
}

// Block until the requested number of bytes have been written
int uart_driver_write(uart_driver_handle_t handle, const void *buf, size_t size)
{
    uart_port_t uart_num = handle_to_uart_num(handle);
    ESP_LOGI(TAG, "writing %d bytes to uart %d", size, uart_num);
    return uart_write_bytes(uart_num, buf, size);
}

