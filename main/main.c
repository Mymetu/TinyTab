#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"

#include "Display_Driver.h"
#include "GPS.h"
#include "SD_MMC.h"
#include "LVGL_App.h"
#include "meshcore_control.h"
#include "meshcore_core.h"
#include "meshcore_ble.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

static const char *TAG = "MAP_BENCH";
static volatile bool s_meshcore_enabled;
static volatile bool s_meshcore_starting;
static bool s_meshcore_config_loaded;

#define APP_CONFIG_NAMESPACE "app_config"
#define APP_CONFIG_MESHCORE "meshcore"

static esp_err_t meshcore_config_load(void)
{
    if (s_meshcore_config_loaded) return ESP_OK;
    s_meshcore_config_loaded = true;
    s_meshcore_enabled = false;

    nvs_handle_t handle;
    if (nvs_open(APP_CONFIG_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return ESP_OK;
    uint8_t value = 0;
    if (nvs_get_u8(handle, APP_CONFIG_MESHCORE, &value) == ESP_OK) {
        s_meshcore_enabled = value != 0;
    }
    nvs_close(handle);
    return ESP_OK;
}

static void meshcore_start_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    if (!s_meshcore_enabled) {
        s_meshcore_starting = false;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "starting MeshCore core and SX1262");
    esp_err_t err = meshcore_core_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MeshCore core start failed: %s", esp_err_to_name(err));
        s_meshcore_enabled = false;
        s_meshcore_starting = false;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "MeshCore core ready, starting BLE transport");
    /* Always start from the disabled state. This prevents an old BLE NVS
     * preference from silently re-enabling MeshCore at boot. */
    (void)meshcore_ble_set_enabled(false);
    err = meshcore_ble_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MeshCore BLE start failed: %s", esp_err_to_name(err));
    } else {
        (void)meshcore_ble_set_enabled(true);
        ESP_LOGI(TAG, "MeshCore BLE ready, node=%s PIN=%06lu",
                 meshcore_core_get_node_name(),
                 (unsigned long)meshcore_core_get_ble_pin());
    }
    s_meshcore_starting = false;
    vTaskDelete(NULL);
}

esp_err_t app_meshcore_set_enabled(bool enabled)
{
    meshcore_config_load();
    nvs_handle_t handle;
    esp_err_t err = nvs_open(APP_CONFIG_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(handle, APP_CONFIG_MESHCORE, enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err == ESP_OK) {
        s_meshcore_enabled = enabled;
        ESP_LOGI(TAG, "MeshCore preference saved: %s (restart required)",
                 enabled ? "enabled" : "disabled");
    }
    return err;
}

bool app_meshcore_is_enabled(void)
{
    meshcore_config_load();
    return s_meshcore_enabled;
}

void app_meshcore_start_if_enabled(void)
{
    if (!app_meshcore_is_enabled() || meshcore_core_is_running() || s_meshcore_starting) return;
    s_meshcore_starting = true;
    if (xTaskCreateWithCaps(meshcore_start_task, "meshcore_start", 6144, NULL, 5,
                            NULL, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        s_meshcore_starting = false;
        ESP_LOGE(TAG, "could not create MeshCore startup task");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "minimal LVGL map benchmark boot, reset_reason=%d",
             (int)esp_reset_reason());

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    /* The BSP owns the LVGL task, tick, display buffers and GT911 input. */
    if (!Display_Init()) {
        ESP_LOGE(TAG, "display initialization failed");
        return;
    }

    /* Keep GPS parsing independent from LVGL and map decoding. The driver
     * only consumes UART1 NMEA data; a missing receiver does not block UI. */
    if (GPS_Init() != ESP_OK) {
        ESP_LOGW(TAG, "GPS UART unavailable; map locate will report no fix");
    }

    /* Mount the card before creating the map so the initial tile scan is
     * deterministic and all later tile reads happen in the worker task. */
    SD_Init();

    if (bsp_display_lock(-1)) {
        Lvgl_App_Init();
        bsp_display_unlock();
        ESP_LOGI(TAG, "minimal map UI ready");
    } else {
        ESP_LOGE(TAG, "could not lock LVGL display");
    }

    /* The persisted preference is false on first boot. When enabled by the
     * user, start the three MeshCore services only after the UI is ready. */
    app_meshcore_start_if_enabled();
}
