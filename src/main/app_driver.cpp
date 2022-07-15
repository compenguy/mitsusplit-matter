/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_log.h>
#include <stdlib.h>
#include <string.h>

#include <device.h>
#include <esp_matter.h>
#include <uart_driver.h>

#include <app_priv.h>
#include <mitsubishi_msz_driver.h>

// Defines cluster value enums
#include <app-common/zap-generated/cluster-enums.h>

using namespace chip::app::Clusters;
using namespace esp_matter;

static const char *TAG = "app_driver";
extern uint16_t thermostat_endpoint_id;
extern uint16_t fan_endpoint_id;

/*****************************
 * Heatpump/Thermostat Control
 ****************************/

static esp_err_t app_driver_thermostat_set_cooling_setpoint(mitsubishi_msz_handle_t handle, esp_matter_attr_val_t *val)
{
    return mitsubishi_msz_driver_set_cooling_setpoint(handle, val->val.u16);
}

static esp_err_t app_driver_thermostat_set_heating_setpoint(mitsubishi_msz_handle_t handle, esp_matter_attr_val_t *val)
{
    return mitsubishi_msz_driver_set_heating_setpoint(handle, val->val.u16);
}

static esp_err_t app_driver_thermostat_set_mode(mitsubishi_msz_handle_t handle, esp_matter_attr_val_t *val)
{
    //Thermostat::ThermostatSystemMode matter_mode = Thermostat::ThermostatSystemMode::kOff;
    msz_setting_power_t msz_power = msz_power_off;
    msz_setting_mode_t msz_mode = msz_mode_auto;

    switch(static_cast<Thermostat::ThermostatSystemMode>(val->val.u8)) {
        case Thermostat::ThermostatSystemMode::kOff:
            msz_power = msz_power_off;
            msz_mode = msz_mode_auto;
            break;
        case Thermostat::ThermostatSystemMode::kAuto:
            msz_power = msz_power_on;
            msz_mode = msz_mode_auto;
            break;
        case Thermostat::ThermostatSystemMode::kCool:
            msz_power = msz_power_on;
            msz_mode = msz_mode_cool;
            break;
        case Thermostat::ThermostatSystemMode::kHeat:
            msz_power = msz_power_on;
            msz_mode = msz_mode_heat;
            break;
        case Thermostat::ThermostatSystemMode::kEmergencyHeating:
            msz_power = msz_power_on;
            msz_mode = msz_mode_heat;
            // TODO: set heat setpoint to emergency safety level
            break;
        case Thermostat::ThermostatSystemMode::kFanOnly:
            msz_power = msz_power_on;
            msz_mode = msz_mode_fan;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
            break;
    }
    // the matter thermostat cluster doesn't seem to support dehumidify mode (msz_mode_dry)
    // the mitsubishi msz heatpump doesn't support a a precool mode (kPrecooling)
    esp_err_t err = mitsubishi_msz_driver_set_power(handle, msz_power);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "thermostat power state change failed: %d", err);
        return err;
    }
    err = mitsubishi_msz_driver_set_mode(handle, msz_mode);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "thermostat system mode state change failed: %d", err);
        return err;
    }
    return ESP_OK;
}

static esp_err_t app_driver_fan_set_mode(mitsubishi_msz_handle_t handle, esp_matter_attr_val_t *val)
{
    //FanControl::FanModeType matter_mode = FanModeType::kOff;
    msz_setting_power_t msz_power = msz_power_off;
    msz_setting_fan_t msz_fan = msz_fan_quiet;

    switch(static_cast<FanControl::FanModeType>(val->val.u8)) {
        case FanControl::FanModeType::kOff:
            msz_power = msz_power_off;
            msz_fan = msz_fan_quiet;
            break;
        case FanControl::FanModeType::kLow:
            msz_power = msz_power_on;
            msz_fan = msz_fan_quiet;
            break;
        case FanControl::FanModeType::kMedium:
            msz_power = msz_power_on;
            msz_fan = msz_fan_med;
            break;
        case FanControl::FanModeType::kHigh:
            msz_power = msz_power_on;
            msz_fan = msz_fan_full;
            break;
        case FanControl::FanModeType::kOn:
            msz_power = msz_power_on;
            msz_fan = msz_fan_med;
            break;
        case FanControl::FanModeType::kAuto:
        case FanControl::FanModeType::kSmart:
            msz_power = msz_power_on;
            msz_fan = msz_fan_auto;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
            break;
    }
    esp_err_t err = mitsubishi_msz_driver_set_power(handle, msz_power);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "thermostat power state change failed: %d", err);
        return err;
    }
    err = mitsubishi_msz_driver_set_fan(handle, msz_fan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "fan control state change failed: %d", err);
        return err;
    }
    return ESP_OK;
}

esp_err_t app_driver_attribute_update(app_driver_handle_t driver_handle, uint16_t endpoint_id, uint32_t cluster_id,
                                      uint32_t attribute_id, esp_matter_attr_val_t *val)
{
    esp_err_t err = ESP_OK;
    if (endpoint_id == thermostat_endpoint_id && cluster_id == Thermostat::Id) {
        switch(attribute_id) {
            case Thermostat::Attributes::OccupiedCoolingSetpoint::Id:
                err = app_driver_thermostat_set_cooling_setpoint(driver_handle, val);
                break;
            case Thermostat::Attributes::OccupiedHeatingSetpoint::Id:
                err = app_driver_thermostat_set_heating_setpoint(driver_handle, val);
                break;
            case Thermostat::Attributes::SystemMode::Id:
                err = app_driver_thermostat_set_mode(driver_handle, val);
                break;
            default:
                break;
        }
    } else if (endpoint_id == fan_endpoint_id && cluster_id == FanControl::Id) {
        if (attribute_id == FanControl::Attributes::FanMode::Id) {
            err = app_driver_fan_set_mode(driver_handle, val);
        }
    }
    return err;
}

app_driver_handle_t app_driver_heatpump_init()
{
    uart_driver_config_t uart_config = uart_driver_get_config();
    mitsubishi_msz_handle_t handle = mitsubishi_msz_driver_init(&uart_config);
    if (handle == NULL) {
        ESP_LOGE(TAG, "mitsubishi msz driver initialization failed");
        return NULL;
    }

    return (app_driver_handle_t)handle;
}

esp_err_t app_driver_heatpump_set_defaults(uint16_t endpoint_id)
{
    esp_err_t err = ESP_OK;
    void *priv_data = endpoint::get_priv_data(endpoint_id);
    app_driver_handle_t handle = (app_driver_handle_t)priv_data;
    node_t *node = node::get();
    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
    cluster_t *cluster = NULL;
    attribute_t *attribute = NULL;
    esp_matter_attr_val_t val = esp_matter_invalid(NULL);

    if (endpoint_id == thermostat_endpoint_id) {
        /* Setting thermostat heating and cooling points */
        cluster = cluster::get(endpoint, Thermostat::Id);
        attribute = attribute::get(cluster, Thermostat::Attributes::OccupiedCoolingSetpoint::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_thermostat_set_cooling_setpoint(handle, &val);

        attribute = attribute::get(cluster, Thermostat::Attributes::OccupiedHeatingSetpoint::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_thermostat_set_heating_setpoint(handle, &val);

        attribute = attribute::get(cluster, Thermostat::Attributes::SystemMode::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_thermostat_set_mode(handle, &val);
    } else if (endpoint_id == fan_endpoint_id) {
        /* Setting fan mode */
        cluster = cluster::get(endpoint, FanControl::Id);
        attribute = attribute::get(cluster, FanControl::Attributes::FanMode::Id);
        attribute::get_val(attribute, &val);
        err |= app_driver_fan_set_mode(handle, &val);
    }

    return err;
}

app_driver_handle_t app_driver_light_init()
{
    /* Initialize led */
    led_driver_config_t config = led_driver_get_config();
    led_driver_handle_t handle = led_driver_init(&config);
    if (handle == NULL) {
        ESP_LOGE(TAG, "led driver initialization failed");
        return NULL;
    }
    return (app_driver_handle_t)handle;
}

static void app_driver_button_toggle_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Toggle button pressed");
    uint16_t endpoint_id = thermostat_endpoint_id;
    uint32_t cluster_id = Thermostat::Id;
    uint32_t attribute_id = Thermostat::Attributes::SystemMode::Id;

    node_t *node = node::get();
    endpoint_t *endpoint = endpoint::get(node, endpoint_id);
    cluster_t *cluster = cluster::get(endpoint, cluster_id);
    attribute_t *attribute = attribute::get(cluster, attribute_id);

    esp_matter_attr_val_t val = esp_matter_invalid(NULL);
    attribute::get_val(attribute, &val);
    val.val.b = !val.val.b;
    attribute::update(endpoint_id, cluster_id, attribute_id, &val);
}

app_driver_handle_t app_driver_button_init()
{
    /* Initialize button */
    button_config_t config = button_driver_get_config();
    button_handle_t handle = iot_button_create(&config);
    iot_button_register_cb(handle, BUTTON_PRESS_DOWN, app_driver_button_toggle_cb, NULL);
    return (app_driver_handle_t)handle;
}

