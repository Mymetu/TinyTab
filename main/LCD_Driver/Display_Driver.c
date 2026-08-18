#include "Display_Driver.h"

#include "bsp/esp-bsp.h"
#include "esp_log.h"

static const char *TAG = "DISPLAY";
static lv_display_t *s_display;
uint8_t LCD_Backlight = 70;

bool Display_Init(void)
{
    if (s_display != NULL) {
        return true;
    }

    /* LVGL applies the display's 180-degree rotation to pointer coordinates.
     * Leave the GT911 raw axes unmirrored to avoid reversing them twice. */
    bsp_display_cfg_t cfg = {
        .lv_adapter_cfg = ESP_LV_ADAPTER_DEFAULT_CONFIG(),
        .rotation = ESP_LV_ADAPTER_ROTATE_180,
        /* Partial mode avoids forcing LVGL to render all 1024x600 pixels for
         * every map movement. The adapter still uses PPA for the 180-degree
         * flush rotation on ESP32-P4. */
        .tear_avoid_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL,
        .touch_flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    /* Keep LVGL/PPA submission on core 0. The map PNG worker is pinned to
     * core 1, so SD reads and inflate work cannot steal render time. */
    cfg.lv_adapter_cfg.task_core_id = 0;
    /* The adapter supports an external task stack. Moving this 8 KiB stack
     * out of internal RAM leaves more DMA-capable memory for the display
     * pipeline without moving LVGL objects or draw buffers. */
    cfg.lv_adapter_cfg.stack_in_psram = true;
    s_display = bsp_display_start_with_config(&cfg);
    if (s_display == NULL) {
        ESP_LOGE(TAG, "failed to initialize EK79007 display and GT911 touch");
        return false;
    }

    Set_Backlight(LCD_Backlight);
    ESP_LOGI(TAG, "EK79007 1024x600 ready (triple-partial, LVGL core 0)");
    return true;
}

bool Display_Is_Ready(void)
{
    return s_display != NULL;
}

void LCD_Init(void)
{
    (void)Display_Init();
}

void Backlight_Init(void)
{
    Set_Backlight(LCD_Backlight);
}

void Set_Backlight(uint8_t light)
{
    if (light > Backlight_MAX) {
        light = Backlight_MAX;
    }

    LCD_Backlight = light;
    if (s_display == NULL) {
        return;
    }

    if (light == 0) {
        bsp_display_backlight_off();
    } else {
        bsp_display_brightness_set(light);
    }
}
