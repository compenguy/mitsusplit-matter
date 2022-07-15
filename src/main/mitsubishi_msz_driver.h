#ifndef __mitsubishi_msz_driver_h__
#define __mitsubishi_msz_driver_h__

#include <esp_log.h>
#include <driver/uart.h>

typedef enum {
    msz_power_off = 0,
    msz_power_on = 1
} msz_setting_power_t;

typedef enum {
    msz_mode_heat = 1,
    msz_mode_dry = 2,
    msz_mode_cool = 3,
    msz_mode_fan = 7,
    msz_mode_auto = 8,
} msz_setting_mode_t;

typedef enum {
    msz_fan_auto = 0,
    msz_fan_quiet = 1,
    msz_fan_low = 2,
    msz_fan_med = 3,
    msz_fan_high = 5,
    msz_fan_full = 6,
} msz_setting_fan_t;

typedef enum {
    msz_vane_auto = 0,
    msz_vane_pos1 = 1,
    msz_vane_pos2 = 2,
    msz_vane_pos3 = 3,
    msz_vane_pos4 = 4,
    msz_vane_pos5 = 5,
    msz_vane_swing = 7,
} msz_setting_vane_t;

typedef enum {
    msz_widevane_farleft = 1,
    msz_widevane_left = 2,
    msz_widevane_center = 3,
    msz_widevane_right = 4,
    msz_widevane_farright= 5,
    msz_widevane_split = 8,
    msz_widevane_swing = 12,
} msz_setting_widevane_t;

typedef enum {
    msz_timermode_none = 0,
    msz_timermode_off = 1,
    msz_timermode_on = 1,
    msz_timermode_both = 1,
} msz_setting_timermode_t;

typedef struct msz_settings {
    msz_setting_power_t power;
    msz_setting_mode_t mode;
    msz_setting_fan_t fan;
    msz_setting_vane_t vane;
    msz_setting_widevane_t widevane;
    bool widevane_adj;
    msz_setting_timermode_t timermode;
    uint16_t timer_on_at_minutes;
    uint16_t timer_off_at_minutes;
    uint16_t timer_on_remaining_minutes;
    uint16_t timer_off_remaining_minutes;
    bool iSee;
    int16_t cool_temp_centi_C;
    int16_t heat_temp_centi_C;
    int16_t room_temp_centi_C;
    uint32_t compressor_frequency;
} msz_settings_t;

typedef void *mitsubishi_msz_handle_t;

mitsubishi_msz_handle_t mitsubishi_msz_driver_init(uart_driver_config_t *config);
esp_err_t mitsubishi_msz_driver_set_cooling_setpoint(mitsubishi_msz_handle_t handle, uint16_t temp_in_centi_C);
esp_err_t mitsubishi_msz_driver_set_heating_setpoint(mitsubishi_msz_handle_t handle, uint16_t temp_in_centi_C);
esp_err_t mitsubishi_msz_driver_set_power(mitsubishi_msz_handle_t handle, msz_setting_power_t);
esp_err_t mitsubishi_msz_driver_set_mode(mitsubishi_msz_handle_t handle, msz_setting_mode_t);
esp_err_t mitsubishi_msz_driver_set_fan(mitsubishi_msz_handle_t handle, msz_setting_fan_t);

#endif /* __mitsubishi_msz_driver_h__ */

