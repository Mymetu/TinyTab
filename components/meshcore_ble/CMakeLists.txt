idf_component_register(
    SRCS "meshcore_ble.c"
    INCLUDE_DIRS "include"
    REQUIRES
        bt
        meshcore_core
        nvs_flash
        esp_hosted
)
