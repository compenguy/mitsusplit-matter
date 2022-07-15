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

#include <esp_log.h>
#include <iot_button.h>
#include <led_driver.h>
#include <uart_driver.h>

#define LED_RED_PIN GPIO_NUM_3
#define LED_GRN_PIN GPIO_NUM_4
#define LED_BLU_PIN GPIO_NUM_5
#define LED_CLW_PIN GPIO_NUM_19 /* Cool White */
#define LED_WRW_PIN GPIO_NUM_18 /* Warm White */

#define LED_BASE_CHANNEL 0 /* Channel per LED, LEDC_CHANNEL_0 through LEDC_CHANNEL_4 */

#define BUTTON_RST_GPIO_PIN GPIO_NUM_0
#define BUTTON_PROG_GPIO_PIN GPIO_NUM_9

/* UART_NUM_0/GPIO_NUM_20/GPIO_NUM_21 are tied up in the USB uart */
#if 0
#define UART_NUM UART_NUM_0
#define UART_RX_PIN GPIO_NUM_20
#define UART_TX_PIN GPIO_NUM_21
#endif

#define UART_NUM UART_NUM_1
#define UART_RX_PIN GPIO_NUM_0
#define UART_TX_PIN GPIO_NUM_1

static const char *TAG = "device";

led_driver_config_t led_driver_get_config()
{
    // .gpio is an int type, which means we've got 32 bits to give the gpio
    // numbers for 3 separate LEDs
    // We'll allocate 8 bits for each gpio, addressing gpios in the range of 0-31
    // with 0xFF reserved for disabled
    led_driver_config_t config = {
        .gpio = ((LED_BLU_PIN & 0xFF) << 16) |
                ((LED_GRN_PIN & 0xFF) <<  8) |
                ((LED_RED_PIN & 0xFF) <<  0),
        .channel = LED_BASE_CHANNEL,
    };
    return config;
}

button_config_t button_driver_get_config()
{
    button_config_t config = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = BUTTON_PROG_GPIO_PIN,
            .active_level = 0,
        }
    };
    return config;
}

uart_driver_config_t uart_driver_get_config()
{
    uart_driver_config_t config = {
        .num = UART_NUM,
        .rx_pin = UART_RX_PIN,
        .tx_pin = UART_TX_PIN
    };
    return config;
}
