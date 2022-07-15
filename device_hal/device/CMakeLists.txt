include($ENV{ESP_MATTER_DEVICE_PATH}/esp_matter_device.cmake)

set(requires )

if(DEFINED uart_type)
    list(APPEND requires uart_driver)
endif()

if(DEFINED led_type)
    list(APPEND requires led_driver)
endif()

if(DEFINED button_type)
    if ("${button_type}" STREQUAL "iot")
        list(APPEND requires iot_button)
    else()
        list(APPEND requires button)
    endif()
endif()

idf_component_register(SRC_DIRS "${device_type}"
                        INCLUDE_DIRS include
                        REQUIRES "${requires}")
