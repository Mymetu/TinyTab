set(RADIOLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/radiolib/src")

idf_component_register(
    SRCS
        "meshcore_radio.cpp"
        "${RADIOLIB_DIR}/Hal.cpp"
        "${RADIOLIB_DIR}/Module.cpp"
        "${RADIOLIB_DIR}/utils/CRC.cpp"
        "${RADIOLIB_DIR}/utils/ConvCode.cpp"
        "${RADIOLIB_DIR}/utils/Cryptography.cpp"
        "${RADIOLIB_DIR}/utils/Utils.cpp"
        "${RADIOLIB_DIR}/protocols/PhysicalLayer/PhysicalLayer.cpp"
        "${RADIOLIB_DIR}/modules/SX126x/SX126x.cpp"
        "${RADIOLIB_DIR}/modules/SX126x/SX126x_commands.cpp"
        "${RADIOLIB_DIR}/modules/SX126x/SX126x_config.cpp"
        "${RADIOLIB_DIR}/modules/SX126x/SX126x_LR_FHSS.cpp"
        "${RADIOLIB_DIR}/modules/SX126x/SX1262.cpp"
    INCLUDE_DIRS
        "include"
        "${RADIOLIB_DIR}"
    REQUIRES
        esp_driver_gpio
        esp_driver_spi
        esp_timer
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    RADIOLIB_STATIC_ONLY=1
    RADIOLIB_EXCLUDE_LORAWAN=1
    RADIOLIB_EXCLUDE_LR11X0=1
    RADIOLIB_EXCLUDE_SX127X=1
    RADIOLIB_EXCLUDE_SX128X=1
)
