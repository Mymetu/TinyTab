cmake_minimum_required(VERSION 3.16)

# ESP-Hosted 1.4.x includes the legacy driver/sdmmc_host.h API. IDF 6 keeps
# that compatibility header under esp_driver_sdmmc/legacy, so expose it to
# all component targets without modifying the downloaded managed component.
if(EXISTS "$ENV{IDF_PATH}/components/esp_driver_sdmmc/legacy/include")
    include_directories("$ENV{IDF_PATH}/components/esp_driver_sdmmc/legacy/include")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32-P4-WIFI6-Touch-LCD-7B)

# Waveshare BSP 3.0.0 uses the native LVGL 9 display rotation type.
# No LVGL 8 compatibility alias is needed after the LVGL 9.5 migration.
