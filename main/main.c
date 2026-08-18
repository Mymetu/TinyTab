#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"

#include "Display_Driver.h"
#include "GPS.h"
#include "SD_MMC.h"
#include "LVGL_App.h"

static const char *TAG = "MAP_BENCH";

void app_main(void)
{
    ESP_LOGI(TAG, "minimal LVGL map benchmark boot");

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
}
