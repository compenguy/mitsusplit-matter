/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <app_priv.h>
#include <app_reset.h>

static const char *TAG = "app_main";

uint16_t thermostat_endpoint_id = 0;
uint16_t fan_endpoint_id = 0;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    default:
        break;
    }
}

static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %d, effect: %d", type, effect_id);
    return ESP_OK;
}

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) {
        /* Driver update */
        app_driver_handle_t driver_handle = (app_driver_handle_t)priv_data;
        err = app_driver_attribute_update(driver_handle, endpoint_id, cluster_id, attribute_id, val);
    }

    return err;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Initialize driver */
    app_driver_handle_t light_handle = app_driver_light_init();
    (void)light_handle;
    app_driver_handle_t button_handle = app_driver_button_init();
    (void)button_handle;
    //app_reset_button_register(button_handle);
    app_driver_handle_t heatpump_handle = app_driver_heatpump_init();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ESP_LOGI(TAG, "Root node initialized");

    /* Initializing thermostat endpoint */
    ESP_LOGI(TAG, "Initializing thermostat endpoint...");
    thermostat::config_t thermostat_config;
    thermostat_config.thermostat.system_mode = DEFAULT_SYSTEM_MODE;
    thermostat_config.thermostat.control_sequence_of_operation = DEFAULT_CONTROL_SEQUENCE;
    endpoint_t *thermostat_endpoint = thermostat::create(node, &thermostat_config, ENDPOINT_FLAG_NONE, heatpump_handle);

    /* Initializing fan control endpoint */
    ESP_LOGI(TAG, "Initializing fan endpoint...");
    fan::config_t fan_config;
    fan_config.fan_control.fan_mode = DEFAULT_FAN_MODE;
    fan_config.fan_control.fan_mode_sequence = DEFAULT_FAN_MODE_SEQUENCE;
    endpoint_t *fan_endpoint = fan::create(node, &fan_config, ENDPOINT_FLAG_NONE, heatpump_handle);

    /* These node and endpoint handles can be used to create/add other endpoints and clusters. */
    if (!node || !thermostat_endpoint || !fan_endpoint) {
        ESP_LOGE(TAG, "Matter node creation failed");
    }

    ESP_LOGI(TAG, "Getting endpoint handles...");
    thermostat_endpoint_id = endpoint::get_id(thermostat_endpoint);
    ESP_LOGI(TAG, "Thermostat created with endpoint_id %d", thermostat_endpoint_id);
    fan_endpoint_id = endpoint::get_id(fan_endpoint);
    ESP_LOGI(TAG, "Fan created with endpoint_id %d", fan_endpoint_id);

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Matter start failed: %d", err);
    }

    /* Starting driver with default values */
    app_driver_heatpump_set_defaults(thermostat_endpoint_id);
    app_driver_heatpump_set_defaults(fan_endpoint_id);
    ESP_LOGI(TAG, "Initialized thermostat with defaults...");

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::init();
#endif
}
