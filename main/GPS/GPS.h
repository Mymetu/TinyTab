#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    bool receiving;
    bool valid;
    double latitude;
    double longitude;
    float altitude_m;
    float speed_kmh;
    float hdop;
    uint8_t satellites;
    uint32_t last_sentence_ms;
    uint32_t last_fix_ms;
    bool time_valid;
    uint32_t utc_time;
    uint32_t last_time_ms;
} gps_data_t;

esp_err_t GPS_Init(void);
bool GPS_Get_Data(gps_data_t *data);
