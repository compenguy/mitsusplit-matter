
#include <string.h>
#include <endian.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <uart_driver.h>

#include <mitsubishi_msz_driver.h>

/* HEADER
 * offset	0		1		2		3		4
 *              Sentinel	Packet type	Sync word --------------------- Payload length
 *              0xFC				0x01		0x30		0x10
 *
 * PAYLOAD
 * offset	5		6		7		8
 *              Command		Data/req flags	Data/req flags	Data
 * offset	9		10		11		12
 *              Data		Data		Data		Data
 * offset	13		14		15		16
 *              Data		Data		Data		Data
 * offset	17		18		19		20
 *              Data		Data		Data		Data
 *
 * CHECKSUM
 * offset	21
 *              checksum
 */

static const char *TAG = "mitsubishi_msz_driver";
static const uint32_t MSZ_DRIVER_READ_TASK_STACK = 4096;
typedef enum msz_packet_type: uint8_t {
    msz_packet_snd_set = 0x41,
    msz_packet_snd_info = 0x42,
    msz_packet_rcv_send_ack = 0x61,
    msz_packet_rcv_update = 0x62,
    msz_packet_rcv_conn_ack = 0x7a
} msz_packet_type_t;

typedef enum msz_command: uint8_t {
    msz_command_set_settings = 0x01,
    msz_command_settings = 0x02,
    msz_command_roomtemp = 0x03,
    msz_command_timer = 0x05,
    msz_command_status = 0x06,
    msz_command_set_remote_temp = 0x07,
    msz_command_functions1 = 0x20,
    msz_command_functions2 = 0x22
} msz_command_t;

typedef enum {
    msz_control_power_mask = 0x01,
    msz_control_mode_mask  = 0x02,
    msz_control_temp_mask  = 0x04,
    msz_control_fan_mask   = 0x08,
    msz_control_vane_mask  = 0x10
} msz_control1_mask_t;

typedef enum {
    msz_control_widevane_mask = 0x01
} msz_control2_mask_t;

typedef struct __attribute__((__packed__)) msz_packet {
    uint8_t sentinel;
    msz_packet_type_t packet_type;
    uint16_t sync_word;
    uint8_t payload_len;
    uint8_t payload[17];
} msz_packet_t;

/* TODO
typedef enum msz_reqmode {
    msz_reqmode_settings = 0x02,
    msz_reqmode_roomtemp
} msz_reqmode_t;
*/

const uint8_t set_temp_map_C[16] = {
    31, 30, 29, 28,
    27, 26, 25, 24,
    23, 22, 21, 20,
    19, 18, 17, 16
};

const uint8_t room_temp_map_C[32] = {
    10, 11, 12, 13,
    14, 15, 16, 17,
    18, 19, 20, 21,
    22, 23, 24, 25,
    26, 27, 28, 29,
    30, 31, 32, 33,
    34, 35, 36, 37,
    38, 39, 40, 41
};

static const uint8_t SENTINEL = 0xFC;
static const uint16_t SYNC_WORD = 0x0130;
static const uint16_t TIMER_STEP_MINUTES = 10;

static const unsigned int PACKET_HEADER_LEN = 5;
static const unsigned int MAX_PACKET_LEN = 22;
static uint8_t packet_buf[MAX_PACKET_LEN * 2] = {0};
static uint8_t packet_buf_start = 0;
static uint8_t packet_buf_end = 0;

static const uint8_t CONNECT[] = { 0xFC, 0x5A, 0x01, 0x30, 0x02, 0xCA, 0x01, 0xA8 };

static const uint8_t MAX_FUNCTION_CODE_COUNT = 30;
// If we ever start using this, we'll probably need to protect it with a synchronization primitive
static uint8_t function_codes[MAX_FUNCTION_CODE_COUNT] = {0};

static const uint32_t THREAD_READ_TIMEOUT_MS = 60000;
static const uint32_t LOCK_TIMEOUT_MS = 20;

SemaphoreHandle_t xCurrentLock = NULL;
msz_settings_t current_settings = {};
SemaphoreHandle_t xRequestedLock = NULL;
msz_settings_t requested_settings = {};
bool connect_pending = false;
bool connected = false;
bool update_pending = false;

uint8_t calculate_checksum(const uint8_t *buf, uint8_t buf_len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < buf_len; i++) {
        sum += buf[i];
    }
    return (SENTINEL - sum) & 0xFF;
}

/********* PACKET SEND **********/
static esp_err_t write_packet(mitsubishi_msz_handle_t handle, msz_packet_t *packet) {
    if (uart_driver_write(handle, (const uint8_t*)packet, sizeof(msz_packet_t)) < 0) {
        ESP_LOGE(TAG, "write_packet() failed writing bytes to uart driver");
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void init_set_packet(msz_packet_t *packet) {
    packet->sentinel = SENTINEL;
    packet->packet_type = msz_packet_snd_set;
    packet->sync_word = htobe16(SYNC_WORD);
    packet->payload_len = 0x10;
}

static int8_t index_of(const uint8_t *haystack, uint8_t haystack_len, uint8_t needle) {
    for (uint8_t i = 0; i < haystack_len; i++) {
        if (haystack[i] == needle) {
            return i;
        }
    }
    return -1;
}

// I *think* this allows supplying an external temperature sensor to bring up to the target temperature
esp_err_t send_packet_set_remote_temp(mitsubishi_msz_handle_t handle, int16_t remote_temp_centi_C) {
    if (!connected) {
        ESP_LOGW(TAG, "[%s] Deferring send of remote temp packet, not yet connected.", __FUNCTION__);
        return ESP_OK;
    }
    msz_packet_t packet = {};
    init_set_packet(&packet);
    packet.payload[0] = msz_command_set_remote_temp;
    if (remote_temp_centi_C > 0) {
        packet.payload[1] = 0x01;
        int8_t temp_C = remote_temp_centi_C / 100;
        packet.payload[2] = 3 + ((temp_C - 10) * 2);
        packet.payload[3] = (temp_C * 2) + 128;
    } else {
        packet.payload[1] = 0x00;
        packet.payload[3] = 0x80;
    }

    packet.payload[16] = calculate_checksum((const uint8_t *)(&packet), sizeof(packet));
    return write_packet(handle, &packet);
}

esp_err_t send_packet_updated_settings(mitsubishi_msz_handle_t handle) {
    if (!connected) {
        ESP_LOGW(TAG, "[%s] Deferring send of settings update packet, not yet connected.", __FUNCTION__);
        return ESP_OK;
    }
    if (update_pending) {
        ESP_LOGW(TAG, "[%s] Deferring send of settings update packet, a prior send has not yet been acknowledged.", __FUNCTION__);
        return ESP_OK;
    }
    msz_packet_t packet = {};
    init_set_packet(&packet);
    packet.payload[0] = msz_command_set_settings;

    if (xSemaphoreTake(xRequestedLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        return ESP_FAIL;
    }
    if (xSemaphoreTake(xCurrentLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        xSemaphoreGive(xRequestedLock);
        return ESP_FAIL;
    }

    if (requested_settings.power != current_settings.power) {
        packet.payload[3] = requested_settings.power;
        packet.payload[1] |= msz_control_power_mask;
    }

    // We track separate heating and cooling set temps, but the air handler doesn't
    // so if there's a mode change, we need to force a temp change
    bool temp_updated = false;
    if (requested_settings.mode != current_settings.mode) {
        packet.payload[4] = requested_settings.mode;
        packet.payload[1] |= msz_control_mode_mask;
        temp_updated = true;
    }

    int8_t requested_temp_idx = -1;
    int8_t requested_temp_precise = 0;
    if (requested_settings.mode == msz_mode_cool || requested_settings.mode == msz_mode_auto) {
        if (temp_updated || requested_settings.cool_temp_centi_C != current_settings.cool_temp_centi_C) {
            requested_temp_idx = index_of(set_temp_map_C, sizeof(set_temp_map_C), requested_settings.cool_temp_centi_C / 100);
            requested_temp_precise = (((int16_t)requested_settings.cool_temp_centi_C * 2) / 100) + 128;
            temp_updated = true;
            ESP_LOGD(TAG, "[%s] Cooling/Auto mode, set temp update required: %04x (%dC)", __FUNCTION__, requested_temp_precise, requested_settings.cool_temp_centi_C / 100);
        }
    } else if (requested_settings.mode == msz_mode_heat) {
        if (temp_updated || requested_settings.heat_temp_centi_C != current_settings.heat_temp_centi_C) {
            requested_temp_idx = index_of(set_temp_map_C, sizeof(set_temp_map_C), requested_settings.heat_temp_centi_C / 100);
            requested_temp_precise = (((int16_t)requested_settings.heat_temp_centi_C * 2) / 100) + 128;
            temp_updated = true;
            ESP_LOGD(TAG, "[%s] Heating mode, set temp update required: %04x (%dC)", __FUNCTION__, requested_temp_precise, requested_settings.heat_temp_centi_C / 100);
        }
    }
    if (temp_updated) {
        // If the temperature appears in the lookup table, set the temp index
        // Otherwise encode it for the long-format temperature field
        if (requested_temp_idx >= 0) {
            packet.payload[5] = requested_temp_idx;
        } else {
            packet.payload[14] = requested_temp_precise;
        }
        packet.payload[1] |= msz_control_temp_mask;
    }
    if (requested_settings.fan != current_settings.fan) {
        packet.payload[6] = requested_settings.fan;
        packet.payload[1] |= msz_control_fan_mask;
    }
    if (requested_settings.vane != current_settings.vane) {
        packet.payload[7] = requested_settings.vane;
        packet.payload[1] |= msz_control_vane_mask;
    }
    if (requested_settings.widevane != current_settings.widevane) {
        packet.payload[13] = requested_settings.widevane | (current_settings.widevane_adj ? 0x80: 0x00);
        packet.payload[2] |= msz_control_widevane_mask;
    }
    xSemaphoreGive(xCurrentLock);
    xSemaphoreGive(xRequestedLock);

    packet.payload[16] = calculate_checksum((const uint8_t *)(&packet), sizeof(packet));
    esp_err_t err = write_packet(handle, &packet);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[%s] Update packet sent, waiting for acknowledgment.", __FUNCTION__);
        update_pending = true;
    }
    return err;
}

esp_err_t send_sync(mitsubishi_msz_handle_t handle) {
    if (!connected) {
        ESP_LOGW(TAG, "[%s] Deferring send of settings update packet, not yet connected.", __FUNCTION__);
        return ESP_OK;
    }
    if (update_pending) {
        ESP_LOGW(TAG, "[%s] Deferring send of settings update packet, a prior send has not yet been acknowledged.", __FUNCTION__);
        return ESP_OK;
    }
    if (xSemaphoreTake(xRequestedLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        return ESP_FAIL;
    }
    if (xSemaphoreTake(xCurrentLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for current_settings", __FUNCTION__);
        xSemaphoreGive(xRequestedLock);
        return ESP_FAIL;
    }
    bool sync_required = false;
    sync_required |= requested_settings.power != current_settings.power;
    sync_required |= requested_settings.mode != current_settings.mode;
    sync_required |= requested_settings.cool_temp_centi_C != current_settings.cool_temp_centi_C;
    sync_required |= requested_settings.heat_temp_centi_C != current_settings.heat_temp_centi_C;
    sync_required |= requested_settings.fan != current_settings.fan;
    sync_required |= requested_settings.vane != current_settings.vane;
    sync_required |= requested_settings.widevane != current_settings.widevane;
    xSemaphoreGive(xCurrentLock);
    xSemaphoreGive(xRequestedLock);

    if (sync_required) {
        return send_packet_updated_settings(handle);
    }
    return ESP_OK;
}

esp_err_t send_packet_connect(mitsubishi_msz_handle_t handle) {
    if (uart_driver_write(handle, CONNECT, sizeof(CONNECT)) < 0) {
        ESP_LOGE(TAG, "send_packet_connect() failed writing bytes to uart driver");
        return ESP_FAIL;
    }
    connect_pending = true;
    connected = false;
    return ESP_OK;
}

/********* PACKET RECV **********/
static esp_err_t validate_packet(msz_packet_t *packet) {
    if (packet->sentinel != SENTINEL) { return ESP_ERR_INVALID_RESPONSE; }

    switch(packet->packet_type) {
        case msz_packet_rcv_send_ack:
        case msz_packet_rcv_update:
        case msz_packet_rcv_conn_ack:
            // recognized packet type
            break;
        default:
            ESP_LOGW(TAG, "[%s] unrecognized packet type %02x", __FUNCTION__, packet->packet_type);
            break;
    }

    if (packet->sync_word != htobe16(SYNC_WORD)) {
        ESP_LOGW(TAG, "[%s] invalid packet sync word %04x", __FUNCTION__, packet->sync_word);
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (packet->payload_len == 0x10) {
        switch(packet->payload[1]) {
            case msz_command_settings:
            case msz_command_roomtemp:
            case msz_command_timer:
            case msz_command_status:
            case msz_command_functions1:
            case msz_command_functions2:
                // recognized command
                break;
            default:
                ESP_LOGW(TAG, "[%s] unrecognized command %02x", __FUNCTION__, packet->payload[0]);
                break;
        }
        // uint8_t payload[16], nothing much we can really do to validate this
    } else if (packet->payload_len == 0x00) {
        // Nothing to do here
    } else {
        ESP_LOGW(TAG, "[%s] unexpected payload length %02x", __FUNCTION__, packet->payload_len);
        return ESP_ERR_INVALID_RESPONSE;
    }

    // verify packet checksum
    uint8_t checksum = calculate_checksum((const uint8_t *)packet, PACKET_HEADER_LEN + packet->payload_len - 1);
    if (packet->payload[packet->payload_len] != checksum) {
        ESP_LOGE(TAG, "[%s] invalid packet checksum %02x", __FUNCTION__, packet->payload[packet->payload_len]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static void record_settings_packet(msz_packet_t *packet) {
    static const uint8_t zero_setting[sizeof(requested_settings)] = {0};
    int16_t set_temp_centi_C = 0;

    assert(packet->payload[0] == msz_command_settings);

    msz_setting_power_t power = (msz_setting_power_t)packet->payload[3];
    bool iSee = packet->payload[4] & 0xF8 ? true : false;
    msz_setting_mode_t mode = (msz_setting_mode_t)((packet->payload[4] & 0x07) - 1);
    if (packet->payload[11] == 0x00) {
        int16_t set_temp_C = set_temp_map_C[packet->payload[5]];
        set_temp_centi_C = set_temp_C * 100;
    } else {
        // temp C, accurate to 0.5 degrees, where -64.0C = 0, +0.0C = 128, and +63.5C = 255
        int16_t set_temp_C_x2 = (packet->payload[11] - 128);
        set_temp_centi_C = set_temp_C_x2 * (100 / 2);
    }
    msz_setting_fan_t fan = (msz_setting_fan_t)packet->payload[6];
    msz_setting_vane_t vane = (msz_setting_vane_t)packet->payload[8];
    msz_setting_widevane_t widevane = (msz_setting_widevane_t)(packet->payload[10] & 0x0F);
    bool widevane_adj = (packet->payload[10] & 0xF0) == 0x80;

    if (xSemaphoreTake(xCurrentLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for current_settings", __FUNCTION__);
        return;
    }
    if (current_settings.power != power) {
        ESP_LOGD(TAG, "[%s] power state was %d now %d", __FUNCTION__, current_settings.power, power);
        current_settings.power = power;
    }
    if (current_settings.iSee != iSee) {
        ESP_LOGD(TAG, "[%s] iSee state was %d now %d", __FUNCTION__, current_settings.iSee, iSee);
        current_settings.iSee = iSee;
    }
    if (current_settings.mode != mode) {
        ESP_LOGD(TAG, "[%s] mode state was %d now %d", __FUNCTION__, current_settings.mode, mode);
        current_settings.mode = mode;
    }
    if (current_settings.mode == msz_mode_cool || current_settings.mode == msz_mode_auto) {
        if (current_settings.cool_temp_centi_C != set_temp_centi_C) {
            ESP_LOGD(TAG, "[%s] Cooling/Auto mode, set temp was %dC now %dC", __FUNCTION__, current_settings.cool_temp_centi_C, set_temp_centi_C);
            current_settings.cool_temp_centi_C = set_temp_centi_C;
        }
    } else if (current_settings.mode == msz_mode_heat) {
        if (current_settings.heat_temp_centi_C != set_temp_centi_C) {
            ESP_LOGD(TAG, "[%s] Heating mode, set temp was %dC now %dC", __FUNCTION__, current_settings.cool_temp_centi_C, set_temp_centi_C);
            current_settings.heat_temp_centi_C = set_temp_centi_C;
        }
    }
    if (current_settings.fan != fan) {
        ESP_LOGD(TAG, "[%s] fan state was %d now %d", __FUNCTION__, current_settings.fan, fan);
        current_settings.fan = fan;
    }
    if (current_settings.vane != vane) {
        ESP_LOGD(TAG, "[%s] vane state was %d now %d", __FUNCTION__, current_settings.vane, vane);
        current_settings.vane = vane;
    }
    if (current_settings.widevane != widevane) {
        ESP_LOGD(TAG, "[%s] widevane state was %d now %d", __FUNCTION__, current_settings.widevane, widevane);
        current_settings.widevane = widevane;
    }
    if (current_settings.widevane_adj != widevane_adj) {
        ESP_LOGD(TAG, "[%s] widevane adj state was %d now %d", __FUNCTION__, current_settings.widevane_adj, widevane_adj);
        current_settings.widevane_adj = widevane_adj;
    }

    // *** NESTED LOCKS ***
    // If requested_settings is uninitialized, initialize it from current_settings
    if (xSemaphoreTake(xRequestedLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        xSemaphoreGive(xCurrentLock);
        return;
    }
    if (memcmp(&requested_settings, zero_setting, sizeof(requested_settings)) == 0) {
        ESP_LOGD(TAG, "[%s] Initialized requested_settings from current_settings.", __FUNCTION__);
        requested_settings = current_settings;
    }
    xSemaphoreGive(xRequestedLock);
    xSemaphoreGive(xCurrentLock);
    update_pending = false;
}

static void record_roomtemp_packet(msz_packet_t *packet) {
    assert(packet->payload[0] == msz_command_roomtemp);
    int16_t room_temp_centi_C = 0;
    if (packet->payload[6] == 0x00) {
        int16_t room_temp_C = room_temp_map_C[packet->payload[3]];
        room_temp_centi_C = room_temp_C * 100;
    } else {
        // temp C, accurate to 0.5 degrees, where -64.0C = 0, +0.0C = 128, and +63.5C = 255
        int16_t room_temp_C_x2 = (packet->payload[6] - 128);
        room_temp_centi_C = room_temp_C_x2 * (100 / 2);
    }

    if (xSemaphoreTake(xCurrentLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for current_settings", __FUNCTION__);
        return;
    }
    if (current_settings.room_temp_centi_C != room_temp_centi_C) {
        ESP_LOGD(TAG, "[%s] roomtemp state was %d now %d", __FUNCTION__, current_settings.room_temp_centi_C, room_temp_centi_C);
        current_settings.room_temp_centi_C = room_temp_centi_C;
    }
    xSemaphoreGive(xCurrentLock);
}

static void record_timer_packet(msz_packet_t *packet) {
    assert(packet->payload[0] == msz_command_timer);

    msz_setting_timermode_t timermode = (msz_setting_timermode_t)packet->payload[3];
    uint16_t timer_on_at_minutes = packet->payload[4] * TIMER_STEP_MINUTES;
    uint16_t timer_off_at_minutes = packet->payload[5] * TIMER_STEP_MINUTES;
    uint16_t timer_on_remaining_minutes = packet->payload[6] * TIMER_STEP_MINUTES;
    uint16_t timer_off_remaining_minutes = packet->payload[7] * TIMER_STEP_MINUTES;

    if (xSemaphoreTake(xCurrentLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for current_settings", __FUNCTION__);
        return;
    }
    if (current_settings.timermode != timermode) {
        ESP_LOGD(TAG, "[%s] timermode state was %d now %d", __FUNCTION__, current_settings.timermode, timermode);
        current_settings.timermode = timermode;
    }
    if (current_settings.timer_on_at_minutes != timer_on_at_minutes) {
        ESP_LOGD(TAG, "[%s] timer_on_at_minutes state was %d now %d", __FUNCTION__, current_settings.timer_on_at_minutes, timer_on_at_minutes);
        current_settings.timer_on_at_minutes = timer_on_at_minutes;
    }
    if (current_settings.timer_off_at_minutes != timer_off_at_minutes) {
        ESP_LOGD(TAG, "[%s] timer_off_at_minutes state was %d now %d", __FUNCTION__, current_settings.timer_off_at_minutes, timer_off_at_minutes);
        current_settings.timer_off_at_minutes = timer_off_at_minutes;
    }
    if (current_settings.timer_on_remaining_minutes != timer_on_remaining_minutes) {
        ESP_LOGD(TAG, "[%s] timer_on_remaining_minutes state was %d now %d", __FUNCTION__, current_settings.timer_on_remaining_minutes, timer_on_remaining_minutes);
        current_settings.timer_on_remaining_minutes = timer_on_remaining_minutes;
    }
    if (current_settings.timer_off_remaining_minutes != timer_off_remaining_minutes) {
        ESP_LOGD(TAG, "[%s] timer_off_remaining_minutes state was %d now %d", __FUNCTION__, current_settings.timer_off_remaining_minutes, timer_off_remaining_minutes);
        current_settings.timer_off_remaining_minutes = timer_off_remaining_minutes;
    }
    xSemaphoreGive(xCurrentLock);
}

static void record_status_packet(msz_packet_t *packet) {
    assert(packet->payload[0] == msz_command_timer);
    // payload[4] says whether the compressor is running, payload[3] has the compressor frequency
    uint32_t compressor_frequency = packet->payload[4] ? packet->payload[3] : 0;

    if (xSemaphoreTake(xCurrentLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for current_settings", __FUNCTION__);
        return;
    }
    if (current_settings.compressor_frequency != compressor_frequency) {
        ESP_LOGD(TAG, "[%s] compressor_frequency state was %d now %d", __FUNCTION__, current_settings.compressor_frequency, compressor_frequency);
        current_settings.compressor_frequency = compressor_frequency;
    }
    xSemaphoreGive(xCurrentLock);
}

static const uint32_t FUNCTIONS_DATA_LEN = 15;
static void record_functions1_packet(msz_packet_t *packet) {
    assert(packet->payload[0] == msz_command_functions1);
    if (packet->payload_len == 0x10) {
        memcpy(function_codes, &packet->payload[1], FUNCTIONS_DATA_LEN);
    } else {
        ESP_LOGW(TAG, "[%s] unexpected payload length (%d) for a function command", __FUNCTION__, packet->payload_len);
    }
}

static void record_functions2_packet(msz_packet_t *packet) {
    assert(packet->payload[0] == msz_command_functions2);
    if (packet->payload_len == 0x10) {
        memcpy(function_codes + FUNCTIONS_DATA_LEN, &packet->payload[1], FUNCTIONS_DATA_LEN);
    } else {
        ESP_LOGW(TAG, "[%s] unexpected payload length (%d) for a function command", __FUNCTION__, packet->payload_len);
    }
}

static void record_packet(mitsubishi_msz_handle_t handle, msz_packet_t *packet) {
    if (packet->packet_type == msz_packet_rcv_update) {
        switch(packet->payload[0]) {
            case msz_command_settings:
                record_settings_packet(packet);
                break;
            case msz_command_roomtemp:
                record_roomtemp_packet(packet);
                break;
            case msz_command_timer:
                record_timer_packet(packet);
                break;
            case msz_command_status:
                record_status_packet(packet);
                break;
            case msz_command_functions1:
                record_functions1_packet(packet);
                break;
            case msz_command_functions2:
                record_functions2_packet(packet);
                break;
            default:
                ESP_LOGW(TAG, "[%s] unrecognized packet command %02x", __FUNCTION__, packet->payload[0]);
                break;
        }
    } else if (packet->packet_type == msz_packet_rcv_send_ack) {
        // acknowledged update
        ESP_LOGW(TAG, "[%s] remote acknowledged update", __FUNCTION__);
        if (xSemaphoreTake(xCurrentLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
            ESP_LOGE(TAG, "[%s] Failed to take lock for current_settings", __FUNCTION__);
            return;
        }
        update_pending = false;
        xSemaphoreGive(xCurrentLock);
    } else if (packet->packet_type == msz_packet_rcv_conn_ack) {
        // acknowledged connect
        ESP_LOGI(TAG, "[%s] remote acknowledged connect", __FUNCTION__);
        connect_pending = false;
        connected = true;
        ESP_LOGI(TAG, "[%s] Deferred send of updated settings packet", __FUNCTION__);
        send_packet_updated_settings(handle);
    }
}

esp_err_t read_packet(mitsubishi_msz_handle_t handle, uint32_t timeout_in_ms) {
    if (!connected && !connect_pending) {
        ESP_LOGI(TAG, "Connecting to mitsubishi msz controller on serial bus...");
        esp_err_t err = send_packet_connect(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "not connected in read_packet(), and failed sending CONNECT packet");
            return err;
        }
    }

    uint8_t packet_buf_bytes = packet_buf_end - packet_buf_start;
    ESP_LOGD(TAG, "[%s] %d bytes in the packet buffer", __FUNCTION__, packet_buf_bytes);

    // See if we already have a packet sentinel byte
    ESP_LOGD(TAG, "[%s] searching for packet sentinel character", __FUNCTION__);
    while (packet_buf_start < packet_buf_end && packet_buf[packet_buf_start] != SENTINEL) {
        packet_buf_start += 1;
        packet_buf_bytes -= 1;
    }

    // Ensure that there's enough room in the packet_buf to receive a whole packet
    if (packet_buf_bytes == 0) {
        packet_buf_start = packet_buf_end = 0;
        ESP_LOGD(TAG, "[%s] packet_buf is empty", __FUNCTION__);
    } else if (packet_buf_start >= MAX_PACKET_LEN) {
        memmove(&packet_buf[0], &packet_buf[packet_buf_start], packet_buf_bytes);
        packet_buf_start = 0;
        packet_buf_end = packet_buf_bytes;
        ESP_LOGD(TAG, "[%s] relocated packet_buf contents to beginning of buffer", __FUNCTION__);
    }

    // if there's bytes in the buffer, the first one should be 0xFC
    // if we haven't received a full header, fetch enough to finish it
    // if we have the header, we know the payload size, and can fetch enough to finish the packet
    int bytes_left_in_packet = 0;
    if (packet_buf_bytes < PACKET_HEADER_LEN) {
        bytes_left_in_packet = PACKET_HEADER_LEN - packet_buf_bytes;
    } else if (packet_buf_bytes < (PACKET_HEADER_LEN + packet_buf[packet_buf_start+4] + 1)) {
        bytes_left_in_packet = (PACKET_HEADER_LEN + packet_buf[packet_buf_start+4] + 1) - packet_buf_bytes;
    } else {
        bytes_left_in_packet = 0;
        ESP_LOGI(TAG, "[%s] received complete packet.", __FUNCTION__);
    }

    // This should grow the buffer enough to contain a whole packet
    ESP_LOGD(TAG, "[%s] attempting to grow packet buffer by %d bytes", __FUNCTION__, bytes_left_in_packet);
    int read_length = uart_driver_read(handle, &packet_buf[packet_buf_end], bytes_left_in_packet, THREAD_READ_TIMEOUT_MS);
    if (read_length < 0) {
        // Timeout waiting for packet data. If a connect was pending, restart connection process.
        if (connect_pending) {
            ESP_LOGW(TAG, "[%s] timed out waiting for connection acknowledgment", __FUNCTION__);
            connect_pending = false;
            connected = false;
        }
        ESP_LOGE(TAG, "[%s] failed reading more bytes from uart: %d", __FUNCTION__, read_length);
        return ESP_ERR_INVALID_ARG;
    }
    packet_buf_end += read_length;
    packet_buf_bytes += read_length;
    ESP_LOGD(TAG, "[%s] %d bytes in the packet buffer after uart read", __FUNCTION__, packet_buf_bytes);

    if (packet_buf_bytes == 0) {
        ESP_LOGD(TAG, "[%s] no packet data", __FUNCTION__);
        return ESP_OK;
    } else if (packet_buf[packet_buf_start] != SENTINEL) {
        ESP_LOGD(TAG, "[%s] no packet sentinel found (start char = %02x, len = %d)", __FUNCTION__, packet_buf[packet_buf_start], packet_buf_bytes);
        return ESP_OK;
    } else if (packet_buf_bytes < PACKET_HEADER_LEN) {
        ESP_LOGD(TAG, "[%s] waiting for complete packet header (start char = %02x, len = %d)", __FUNCTION__, packet_buf[packet_buf_start], packet_buf_bytes);
        return ESP_OK;
    }

    uint8_t packet_len = PACKET_HEADER_LEN + packet_buf[packet_buf_start+4] + 1;

    if (packet_buf_bytes < packet_len) {
        ESP_LOGD(TAG, "[%s] waiting for complete packet payload: start char = %02x, len = %d", __FUNCTION__, packet_buf[packet_buf_start], packet_buf_bytes);
        return ESP_OK;
    }

    // ensure we have a valid packet, and if not, we'll pop the first character off the buffer
    // and jump to the next loop iteration and keep scanning
    msz_packet_t *packet = (msz_packet_t*)&packet_buf[packet_buf_start];
    if (validate_packet(packet) != ESP_OK) {
        packet_buf_start += 1;
        packet_buf_bytes -= 1;
        ESP_LOGW(TAG, "[%s] packet sentinel found and advertised payload received, but packet contents weren't valid. Discarding...", __FUNCTION__);
        return ESP_OK;
    }

    // packet is valid
    ESP_LOGI(TAG, "[%s] valid packet found", __FUNCTION__);
    record_packet(handle, packet);
    packet_buf_start += packet_len;
    packet_buf_bytes -= packet_len;

    ESP_LOGD(TAG, "[%s] recorded valid packet", __FUNCTION__);

    // Check if current_settings differs from requested_settings, and
    // whether a send is already pending, and re-send update
    return send_sync(handle);
}

void read_packet_task(void *args) {
    mitsubishi_msz_handle_t handle = (mitsubishi_msz_handle_t)args;

    while(1) {
        if (read_packet(handle, THREAD_READ_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGE(TAG, "read_packet_task failed reading packet");
        }
    }
}

mitsubishi_msz_handle_t mitsubishi_msz_driver_init(uart_driver_config_t *config) {
    esp_err_t err = ESP_OK;
    uart_driver_handle_t uart_handle = uart_driver_init(config);

    uart_config_t port_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
        .source_clk = UART_SCLK_REF_TICK,
#else
        .source_clk = UART_SCLK_XTAL,
#endif
    };

    err = uart_driver_port_config(uart_handle, &port_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mitsubishi_msz_driver_init could not configure serial port");
        return NULL;
    }

    // Locks to protect data structures that are accessed by both the main thread
    // and the read_packet_task thread
    // BUG: This code block causes, at the very least, the USB debug monitor to lock up
    // it might lock up the whole CPU
#if 0
    if (xCurrentLock == NULL) {
        xCurrentLock = xSemaphoreCreateMutex();
    }
    if (xRequestedLock == NULL) {
        xRequestedLock = xSemaphoreCreateMutex();
    }

    if (xSemaphoreTake(xCurrentLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for current_settings", __FUNCTION__);
        return NULL;
    }

    connected = false;
    update_pending = false;
    xSemaphoreGive(xCurrentLock);

    ESP_LOGD(TAG, "Spawning read_packet_task()...");
    if(xTaskCreate(&read_packet_task, "read_packet_task", MSZ_DRIVER_READ_TASK_STACK, uart_handle, tskIDLE_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Couldn't create packet read task");
    }
#endif

    return (mitsubishi_msz_handle_t)uart_handle;
}

esp_err_t mitsubishi_msz_driver_set_cooling_setpoint(mitsubishi_msz_handle_t handle, uint16_t temp_in_centi_C) {
    if (xSemaphoreTake(xRequestedLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        return ESP_FAIL;
    }
    requested_settings.cool_temp_centi_C = temp_in_centi_C;
    xSemaphoreGive(xRequestedLock);
    ESP_LOGI(TAG, "[%s] matter called for new cooling setpoint: %dC", __FUNCTION__, temp_in_centi_C / 100);
    return send_packet_updated_settings(handle);
}

esp_err_t mitsubishi_msz_driver_set_heating_setpoint(mitsubishi_msz_handle_t handle, uint16_t temp_in_centi_C) {
    if (xSemaphoreTake(xRequestedLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        return ESP_FAIL;
    }
    requested_settings.heat_temp_centi_C = temp_in_centi_C;
    xSemaphoreGive(xRequestedLock);
    ESP_LOGI(TAG, "[%s] matter called for new heating setpoint: %dC", __FUNCTION__, temp_in_centi_C / 100);
    return send_packet_updated_settings(handle);
}

esp_err_t mitsubishi_msz_driver_set_power(mitsubishi_msz_handle_t handle, msz_setting_power_t power) {
    if (xSemaphoreTake(xRequestedLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        return ESP_FAIL;
    }
    requested_settings.power = power;
    xSemaphoreGive(xRequestedLock);
    ESP_LOGI(TAG, "[%s] matter called for new power state: %d", __FUNCTION__, power);
    return send_packet_updated_settings(handle);
}

esp_err_t mitsubishi_msz_driver_set_mode(mitsubishi_msz_handle_t handle, msz_setting_mode_t mode) {
    if (xSemaphoreTake(xRequestedLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        return ESP_FAIL;
    }
    requested_settings.mode = mode;
    xSemaphoreGive(xRequestedLock);
    ESP_LOGI(TAG, "[%s] matter called for new mode state: %d", __FUNCTION__, mode);
    return send_packet_updated_settings(handle);
}

esp_err_t mitsubishi_msz_driver_set_fan(mitsubishi_msz_handle_t handle, msz_setting_fan_t fan) {
    if (xSemaphoreTake(xRequestedLock, LOCK_TIMEOUT_MS / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "[%s] Failed to take lock for requested_settings", __FUNCTION__);
        return ESP_FAIL;
    }
    requested_settings.fan = fan;
    xSemaphoreGive(xRequestedLock);
    ESP_LOGI(TAG, "[%s] matter called for new fan state: %d", __FUNCTION__, fan);
    return send_packet_updated_settings(handle);
}

/* It's unclear what the function codes mean or how to use them
uint8_t get_function_value(uint8_t code) {
    // the function_codes table is a list of key-value pairs encoded as a single byte
    // keys are 5-bit values in the range of 1 to 28
    // values are 2-bit values  in the range of 1 to 3
    if (code >= 101 && code <= 128) {
        for(int i = 0; i < sizeof(function_codes); i++) {
            uint8_t slot_key = ((function_codes[i] >> 2) & 0xff) + 100;
            if (code == slot_key) {
                return (function_codes[i] & 0x3);
            }
        }
    }
    return 0;
}

bool set_function_value(uint8_t code, uint8_t value) {
    if (code >= 101 && code <= 128 && value >= 1 && value <= 3) {
        for(int i = 0; i < sizeof(function_codes); i++) {
            uint8_t slot_key = ((function_codes[i] >> 2) & 0xff) + 100;
            if (code == slot_key) {
                function_codes[i] = ((code - 100) << 2) + value;
                return true;
            }
        }
    }
    return false;
}
*/

