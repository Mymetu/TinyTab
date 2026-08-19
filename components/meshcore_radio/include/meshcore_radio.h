#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int nss_gpio;
    int mosi_gpio;
    int miso_gpio;
    int sck_gpio;
    int busy_gpio;
    int reset_gpio;
    int dio1_gpio;
    float frequency_mhz;
    float bandwidth_khz;
    uint8_t spreading_factor;
    uint8_t coding_rate;
    int8_t tx_power_dbm;
    float tcxo_voltage;
} meshcore_radio_config_t;

meshcore_radio_config_t meshcore_radio_default_config(void);
esp_err_t meshcore_radio_init(const meshcore_radio_config_t *config);
bool meshcore_radio_is_ready(void);
int16_t meshcore_radio_last_status(void);

#ifdef __cplusplus
}
#endif

