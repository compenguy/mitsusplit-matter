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

#include <color_format.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <hal/ledc_types.h>
#include <led_driver.h>

static const char *TAG = "led_driver_multigpio";
static bool current_power = false;
static uint8_t current_brightness = 0;
static uint32_t current_temp = 6600;
static HS_color_t current_HS = {0, 0};
static RGB_color_t mRGB = {0};
static ledc_channel_config_t led_channels[] = {
    // Red
    {
        .gpio_num = 0xFF,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    },
    // Green
    {
        .gpio_num = 0xFF,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    },
    // Blue
    {
        .gpio_num = 0xFF,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = 0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    }
};

esp_err_t led_driver_init(led_driver_config_t *config)
{
    ESP_LOGI(TAG, "Initializing light driver");
    esp_err_t err = ESP_OK;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, // timer mode
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .timer_num = LEDC_TIMER_0, // timer index
        .freq_hz = 25000, // frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK, // Auto select the source clock
    };
    err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_timerc_config failed");
        return err;
    }

    int channel_count = sizeof(led_channels)/sizeof(ledc_channel_config_t);
    for (int i = 0; i < channel_count; i++) {
        int gpio = (config->gpio >> (i * 8)) & 0xFF;
        if (gpio == 0xFF) {
            // Color channel is disabled - skip
            continue;
        }
        led_channels[i].gpio_num = gpio;
        led_channels[i].channel = i;
        ESP_LOGI(TAG, "ledc_channel_config(gpio: %d, channel: %d)", led_channels[i].gpio_num, led_channels[i].channel);
        err = ledc_channel_config(&led_channels[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed");
            return err;
        }
    }
    return err;
}

esp_err_t led_driver_set_power(bool power)
{
    current_power = power;
    return led_driver_set_brightness(current_brightness);
}

static esp_err_t led_driver_set_channel(int channel, uint8_t intensity)
{
    esp_err_t err = ESP_OK;
    ESP_LOGI(TAG, "ledc_set_duty(channel: %d, duty: %d)", channel, intensity);
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

esp_err_t led_driver_set_RGB()
{
    esp_err_t err = ESP_OK;

    RGB_color_t scaledRGB = {0};
    ESP_LOGI(TAG, "led set r:%d, g:%d, b:%d (unscaled)", mRGB.red, mRGB.green, mRGB.blue);

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

    ESP_LOGI(TAG, "led set r:%d, g:%d, b:%d (scaled)", scaledRGB.red, scaledRGB.green, scaledRGB.blue);
    return err;
}

esp_err_t led_driver_set_brightness(uint8_t brightness)
{
    if (brightness != 0) {
        current_brightness = brightness;
    }
    if (!current_power) {
        brightness = 0;
    }

    hsv_to_rgb(current_HS, brightness, &mRGB);
    return led_driver_set_RGB();
}

esp_err_t led_driver_set_hue(uint16_t hue)
{
    uint8_t brightness = current_power ? current_brightness : 0;
    current_HS.hue = hue;
    hsv_to_rgb(current_HS, brightness, &mRGB);
    return led_driver_set_RGB();
}

esp_err_t led_driver_set_saturation(uint8_t saturation)
{
    uint8_t brightness = current_power ? current_brightness : 0;
    current_HS.saturation = saturation;
    hsv_to_rgb(current_HS, brightness, &mRGB);
    return led_driver_set_RGB();
}

esp_err_t led_driver_set_temperature(uint32_t temperature)
{
    uint8_t brightness = current_power ? current_brightness : 0;
    current_temp = temperature;
    temp_to_hs(current_temp, &current_HS);
    hsv_to_rgb(current_HS, brightness, &mRGB);
    return led_driver_set_RGB();
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
