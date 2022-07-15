include($ENV{ESP_MATTER_DEVICE_PATH}/esp_matter_device.cmake)

set(src_dirs )

if ("${uart_type}" STREQUAL "esp-idf")
    list(APPEND src_dirs esp-idf)
endif()

idf_component_register(SRC_DIRS "${src_dirs}"
                    INCLUDE_DIRS include)
