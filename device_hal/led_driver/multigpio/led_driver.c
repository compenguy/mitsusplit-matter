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

#include <string.h>
#include <color_format.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <hal/ledc_types.h>
#include <led_driver.h>

#define CHANNEL_COUNT 3

static const char *TAG = "led_driver_multigpio";
static bool current_power = false;
static uint8_t current_brightness = 0;
static uint32_t current_temp = 6600;
static HS_color_t current_HS = {0, 0};
static RGB_color_t mRGB = {0};
static ledc_channel_config_t led_channel_defaults[CHANNEL_COUNT] = {
    // Red
    {
        .gpio_num = 0xFF,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    },
    // Green
    {
        .gpio_num = 0xFF,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    },
    // Blue
    {
        .gpio_num = 0xFF,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    },
    // White/Cool White
    {
        .gpio_num = 0xFF,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    },
    // Warm White
    {
        .gpio_num = 0xFF,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    }
};

led_driver_handle_t led_driver_init(led_driver_config_t *config)
{
    ESP_LOGI(TAG, "Initializing light driver");
    esp_err_t err = ESP_OK;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .timer_num = LEDC_TIMER_1, // timer index
        .freq_hz = 5000, // frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK, // Auto select the source clock
    };
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_timerc_config failed");
        return NULL;
    }

    ledc_channel_config_t *led_channels = malloc(sizeof(led_channels));
    memcpy(led_channels, led_channel_defaults, sizeof(led_channel_defaults));
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        int gpio = (config->gpio >> (i * 8)) & 0xFF;
        if (gpio == 0xFF) {
            // Color channel is disabled - skip
            continue;
        }
        led_channels[i].gpio_num = gpio;
        led_channels[i].channel = i;
        err = ledc_channel_config(&led_channels[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed");
            return NULL;
        }
    }
    return (led_driver_handle_t)led_channels;
}

esp_err_t led_driver_set_power(led_driver_handle_t handle, bool power)
{
    current_power = power;
    return led_driver_set_brightness(handle, current_brightness);
}

static esp_err_t led_driver_set_channel(int channel, uint8_t intensity)
{
    esp_err_t = ESP_OK;
    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, intensity);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_set_duty failed");
    }

    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_update_duty failed");
    }
    return err;
}

esp_err_t led_driver_set_RGB(led_driver_handle_t handle)
{
    esp_err_t err = ESP_OK;
    ledc_channel_config_t *led_channels = (ledc_channel_config_t*)handle;

    RGB_color_t scaledRGB = {0};

    if (led_channels[0].gpio_num != 0xFF) {
        scaledRGB.red = (uint8_t)(((int)mRGB.red * (int)current_brightness) / 256);
        err = led_driver_set_channel(LEDC_CHANNEL_0, scaledRGB.red);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "led_driver_set_RGB failed to set RED channel");
        }
    }
    if (led_channels[1].gpio_num != 0xFF) {
        scaledRGB.green = (uint8_t)(((int)mRGB.green * (int)current_brightness) / 256);
        err = led_driver_set_channel(LEDC_CHANNEL_1, scaledRGB.green);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "led_driver_set_RGB failed to set GREEN channel");
        }
    }
    if (led_channels[2].gpio_num != 0xFF) {
        scaledRGB.blue = (uint8_t)(((int)mRGB.blue * (int)current_brightness) / 256);
        err = led_driver_set_channel(LEDC_CHANNEL_2, scaledRGB.green);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "led_driver_set_RGB failed to set BLUE channel");
        }
    }

    ESP_LOGI(TAG, "led set r:%d, g:%d, b:%d", mRGB.red, mRGB.green, mRGB.blue);
    return err;
}

esp_err_t led_driver_set_brightness(led_driver_handle_t handle, uint8_t brightness)
{
    if (brightness != 0) {
        current_brightness = brightness;
    }
    if (!current_power) {
        brightness = 0;
    }
    hsv_to_rgb(current_HS, brightness, &mRGB);
    return led_driver_set_RGB(handle);
}

esp_err_t led_driver_set_hue(led_driver_handle_t handle, uint16_t hue)
{
    uint8_t brightness = current_power ? current_brightness : 0;
    current_HS.hue = hue;
    hsv_to_rgb(current_HS, brightness, &mRGB);
    return led_driver_set_RGB(handle);
}

esp_err_t led_driver_set_saturation(led_driver_handle_t handle, uint8_t saturation)
{
    uint8_t brightness = current_power ? current_brightness : 0;
    current_HS.saturation = saturation;
    hsv_to_rgb(current_HS, brightness, &mRGB);
    return led_driver_set_RGB(handle);
}

esp_err_t led_driver_set_temperature(led_driver_handle_t handle, uint32_t temperature)
{
    uint8_t brightness = current_power ? current_brightness : 0;
    current_temp = temperature;
    temp_to_hs(current_temp, &current_HS);
    hsv_to_rgb(current_HS, brightness, &mRGB);
    return led_driver_set_RGB(handle);
}

bool led_driver_get_power()
{
    return current_power;
}

uint8_t led_driver_get_brightness()
{
    return current_brightness;
}

uint16_t led_driver_get_hue()
{
    return current_HS.hue;
}

uint8_t led_driver_get_saturation()
{
    return current_HS.saturation;
}

uint32_t led_driver_get_temperature()
{
    return current_temp;
}
