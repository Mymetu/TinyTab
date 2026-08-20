#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"

#include "Display_Driver.h"
#include "GPS.h"
#include "SD_MMC.h"
#include "LVGL_App.h"
#include "meshcore_core.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

static const char *TAG = "MAP_BENCH";
static volatile bool s_meshcore_starting;

static void meshcore_start_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(TAG, "starting LoRa core and SX1262");
    esp_err_t err = meshcore_core_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LoRa core start failed: %s", esp_err_to_name(err));
        s_meshcore_starting = false;
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "LoRa core and SX1262 ready, node=%s",
             meshcore_core_get_node_name());
    s_meshcore_starting = false;
    vTaskDelete(NULL);
}

static void app_meshcore_start(void)
{
    if (meshcore_core_is_running() || s_meshcore_starting) return;
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

    /* MeshCore and its SX1262 radio are always started after the UI is ready,
     * keeping display initialization responsive while making radio services
     * available on every boot. */
    app_meshcore_start();
}
