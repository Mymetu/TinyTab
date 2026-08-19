#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t meshcore_ble_start(void);
esp_err_t meshcore_ble_set_enabled(bool enabled);
bool meshcore_ble_is_enabled(void);
bool meshcore_ble_is_started(void);
bool meshcore_ble_is_connected(void);

#ifdef __cplusplus
}
#endif
