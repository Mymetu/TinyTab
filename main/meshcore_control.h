#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Runtime gate for the MeshCore core/radio/BLE services. */
esp_err_t app_meshcore_set_enabled(bool enabled);
bool app_meshcore_is_enabled(void);
void app_meshcore_start_if_enabled(void);

#ifdef __cplusplus
}
#endif
