#include <app_priv.h>

static const char *TAG = "app_led_helper";

static app_driver_handle_t handle = NULL;

esp_err_t init_status_led(app_driver_handle_t led_handle)
{
    handle = led_handle;
    ESP_LOGI(TAG, "Initializing Status LED...");
    return set_status_led_off();
}

esp_err_t set_status_led_off()
{
    ESP_LOGI(TAG, "Status LED: OFF");
    return led_driver_set_power(handle, false);
}

esp_err_t set_status_led_state(u8_t brightness, u16_t hue, u8_t saturation, u16_t temperature)
{
    esp_err_t ret = ESP_OK;
    if (brightness == 0) {
        return led_driver_set_power(handle, false);
    }
    if ((ret = led_driver_set_power(handle, true)) != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "Status LED brightness: %d", brightness);
    if ((ret = led_driver_set_brightness(handle, brightness)) != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "Status LED hue: %d", hue);
    if ((ret = led_driver_set_hue(handle, hue)) != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "Status LED saturation: %d", saturation);
    if ((ret = led_driver_set_saturation(handle, saturation)) != ESP_OK) {
        return ret;
    }
    /*
    if ((ret = led_driver_set_temperature(handle, temperature)) != ESP_OK) {
        return ret;
    }
    */
    ESP_LOGI(TAG, "Status LED update result: %d", ret);
    return ret;
}

esp_err_t set_status_led_yellow(u8_t brightness)
{
    // 60 hue, 100% saturation
    ESP_LOGI(TAG, "Status LED: YELLOW");
    return set_status_led_state(brightness, 60, 100, 0);
}

esp_err_t set_status_led_blue(u8_t brightness)
{
    // 240 hue, 100% saturation
    // Verified
    ESP_LOGI(TAG, "Status LED: BLUE");
    return set_status_led_state(brightness, 240, 100, 0);
}

esp_err_t set_status_led_green(u8_t brightness)
{
    // 120 hue, 100% saturation
    // Verified
    ESP_LOGI(TAG, "Status LED: GREEN");
    //return set_status_led_state(brightness, 120, 100, 0);
    return set_status_led_state(brightness, 120, 100, 0);
}

esp_err_t set_status_led_red(u8_t brightness)
{
    // 0 hue, 100% saturation
    ESP_LOGI(TAG, "Status LED: RED");
    return set_status_led_state(brightness, 0, 100, 0);
}
