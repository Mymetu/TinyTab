cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# These components belong to the original MeshCore/audio application and are
# intentionally excluded from this standalone LVGL benchmark.
set(EXCLUDE_COMPONENTS
    bsp_extra
    meshcore_ble
    meshcore_core
    meshcore_radio
    chmorgan__esp-audio-player
    chmorgan__esp-libhelix-mp3
)
project(ESP32-P4-WIFI6-Touch-LCD-7B)

# Waveshare BSP 3.0.0 uses the native LVGL 9 display rotation type.
# No LVGL 8 compatibility alias is needed after the LVGL 9.5 migration.
