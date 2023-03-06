/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#include <esp_err.h>
#include <esp_matter.h>
#include <led_driver.h>

// all-clusters-app.matter, enum ThermostatSystemMode : ENUM8
// except for dehumidify, which appears to be in the equivalent zigbee clusters, but not (yet?) in matter
enum thermostat_system_mode : uint8_t {
    thermostat_system_mode_off = 0,
    thermostat_system_mode_auto = 1,
    thermostat_system_mode_cool = 3,
    thermostat_system_mode_heat = 4,
    thermostat_system_mode_emergency_heat = 5,
    thermostat_system_mode_precool = 6,
    thermostat_system_mode_fanonly = 7,
    thermostat_system_mode_dehumidify = 8,
};
#define DEFAULT_SYSTEM_MODE thermostat_system_mode_off

// all-clusters-app.matter, enum ThermostatControlSequence : ENUM8
enum thermostat_control_sequence : uint8_t {
    thermostat_control_coolingonly = 0,
    thermostat_control_coolingwithreheat = 1,
    thermostat_control_heatingonly = 2,
    thermostat_control_heatingwithreheat = 3,
    thermostat_control_coolingandheating = 4,
    thermostat_control_coolingandheatingwithreheat = 5
};
#define DEFAULT_CONTROL_SEQUENCE thermostat_control_coolingandheating
// setpoints are in hundredths of a degree C
#define DEFAULT_HEATING_SETPOINT 2000
#define DEFAULT_COOLING_SETPOINT 2600


// all-clusters-app.matter, enum FanModeType : ENUM8
enum fan_mode_type : uint8_t {
    fan_mode_off = 0,
    fan_mode_low = 1,
    fan_mode_medium = 2,
    fan_mode_high = 3,
    fan_mode_on = 4,
    fan_mode_auto = 5,
    fan_mode_smart = 6
};
#define DEFAULT_FAN_MODE fan_mode_auto

// all-clusters-app.matter, enum FanModeSequenceType : ENUM8
enum fan_mode_sequence : uint8_t {
    fan_mode_sequence_lowmedhigh = 0,
    fan_mode_sequence_lowhigh = 1,
    fan_mode_sequence_lowmedhighauto = 2,
    fan_mode_sequence_lowhighauto = 3,
    fan_mode_sequence_offonauto = 4,
    fan_mode_sequence_offon = 5
};
#define DEFAULT_FAN_MODE_SEQUENCE fan_mode_sequence_lowmedhighauto

typedef void *app_driver_handle_t;

/** Initialize the light driver
 *
 * This initializes the light driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_light_init();

/** Initialize the button driver
 *
 * This initializes the button driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_button_init();

/** Initialize the heatpump driver
 *
 * This initializes the heatpump driver associated with the selected board.
 *
 * @return Handle on success.
 * @return NULL in case of failure.
 */
app_driver_handle_t app_driver_heatpump_init();

/** Driver Update
 *
 * This API should be called to update the driver for the attribute being updated.
 * This is usually called from the common `app_attribute_update_cb()`.
 *
 * @param[in] endpoint_id Endpoint ID of the attribute.
 * @param[in] cluster_id Cluster ID of the attribute.
 * @param[in] attribute_id Attribute ID of the attribute.
 * @param[in] val Pointer to `esp_matter_attr_val_t`. Use appropriate elements as per the value type.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val);

/** Set defaults for heatpump driver
 *
 * Set the attribute drivers to their default values from the created data model.
 *
 * @param[in] endpoint_id Endpoint ID of the driver.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t app_driver_heatpump_set_defaults(uint16_t endpoint_id);

/** Initialize Status LED
 *
 * Specify the handle to be used for the status LED, and initialize it to OFF.
 *
 * @param[in] led_handle The driver handle to the status LED.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t init_status_led(app_driver_handle_t led_handle);

/** Turn the Status LED off
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t set_status_led_off();

/** Fully specify the Status LED state
 *
 * Allows precisely controlling all attributes of the LED state.
 *
 * @param[in] brightness Specify the LED brightness in the range of 0-255.
 * @param[in] hue Specify the LED color hue in the range of 0-255.
 * @param[in] saturation Specify the LED color saturation in the range of 0-255.
 * @param[in] temperature Specify the LED color temperature in (kelvin?).
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t set_status_led_state(u8_t brightness, u16_t hue, u8_t saturation, u16_t temperature);

/** Set the Status LED to Yellow at the indicated brightness
 *
 * @param[in] brightness Specify the LED brightness in the range of 0-255.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t set_status_led_yellow(u8_t brightness);

/** Set the Status LED to Yellow at the indicated brightness
 *
 * @param[in] brightness Specify the LED brightness in the range of 0-255.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t set_status_led_blue(u8_t brightness);

/** Set the Status LED to Blue at the indicated brightness
 *
 * @param[in] brightness Specify the LED brightness in the range of 0-255.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t set_status_led_green(u8_t brightness);

/** Set the Status LED to Green at the indicated brightness
 *
 * @param[in] brightness Specify the LED brightness in the range of 0-255.
 *
 * @return ESP_OK on success.
 * @return error in case of failure.
 */
esp_err_t set_status_led_red(u8_t brightness);
