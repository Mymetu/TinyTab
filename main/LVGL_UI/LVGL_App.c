#include "LVGL_App.h"
#include "Display_Driver.h"
#include "GPS.h"
#include "SD_MMC.h"
#include "LVGL_Chinese_Font.h"
#include "meshcore_core.h"
#include "lvgl.h"

#include "bsp/esp-bsp.h"
#include "esp_heap_caps.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "esp_app_desc.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "src/libs/lodepng/lodepng.h"

#include <dirent.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#define STATUS_H        36
#define SIDE_MENU_W     148
#define TILE_SIZE       256
#define MAP_W           (EXAMPLE_LCD_H_RES - SIDE_MENU_W)
#define MAP_H           (EXAMPLE_LCD_V_RES - STATUS_H)
#define GRID_COLS       6
#define GRID_ROWS       5
#define GRID_COUNT      (GRID_COLS * GRID_ROWS)
/* A fractional tile offset can expose 5x4 tiles on the 876x564 map viewport.
 * Keep one extra column and row so the moving layer always covers the right
 * and bottom edges. The decoded cache remains larger than two full grids. */
#define TILE_COUNT      80
#define PREFETCH_DEPTH  1
#define TILE_BYTES      (TILE_SIZE * TILE_SIZE * 2)
#define LOAD_QUEUE_LEN  32
#define RESULT_QUEUE_LEN (LOAD_QUEUE_LEN * 2)
#define MAP_RESULT_FRAME_MS 16
#define RAW_MAP_ROOT    "/sdcard"

static char s_map_root[96] = "/sdcard/map";

typedef struct {
    const char *name;
    int width;
    int height;
    int zoom;
} raw_map_info_t;

typedef struct {
    lv_image_dsc_t dsc;
    uint8_t *pixels;
    int z;
    int x;
    int y;
    uint32_t token;
    bool valid;
    bool loading;
    bool failed;
    uint32_t last_use;
} map_tile_t;

typedef struct {
    lv_obj_t *image;
    int z;
    int x;
    int y;
} map_cell_t;

typedef struct {
    lv_obj_t *obj;
    double tx;
    double ty;
} map_marker_t;

#define MAP_MARKER_MAX 8

typedef struct {
    uint8_t slot;
    uint32_t token;
    uint32_t generation;
    int z;
    int x;
    int y;
} tile_request_t;

typedef struct {
    uint8_t slot;
    uint32_t token;
    uint32_t generation;
    bool ok;
} tile_result_t;

static const char *TAG = "MAP";
static lv_obj_t *s_map_layer;
static lv_obj_t *s_status_time;
static lv_obj_t *s_status_info;
static lv_obj_t *s_map_message;
static lv_timer_t *s_map_timer;
static QueueHandle_t s_load_queue;
static QueueHandle_t s_result_queue;
static map_tile_t s_tiles[TILE_COUNT];
static map_cell_t s_cells[GRID_COUNT];
static uint16_t s_tile_capacity;
static uint32_t s_tile_use_clock;
static volatile uint32_t s_map_generation = 1;
static uint32_t s_result_drop_logs;
static uint32_t s_raw_tile_logs;
static uint32_t s_png_fallback_logs;
static lv_obj_t *s_raw_image;
static lv_image_dsc_t s_raw_dsc;
static uint8_t *s_raw_pixels;
static int s_raw_width;
static int s_raw_height;
static int s_raw_zoom;
static double s_raw_x;
static double s_raw_y;
static bool s_raw_mode;

static int s_levels[32];
static int s_level_count;
static int s_zoom = -1;
static bool s_raw_tile_tree;
static double s_world_x;
static double s_world_y;
static int s_grid_x0;
static int s_grid_y0;
static volatile bool s_refresh_pending;
static bool s_pan_active;
static lv_point_t s_pan_last;
static int s_pan_dir_x;
static int s_pan_dir_y;
static uint32_t s_frame_count;
static uint32_t s_frame_window_start;
static uint32_t s_last_render_us;
static uint32_t s_render_start_us;
static lv_obj_t *s_tabview;
static lv_obj_t *s_sys_ram_value;
static lv_obj_t *s_sys_ram_detail;
static lv_obj_t *s_sys_ram_bar;
static lv_obj_t *s_sys_psram_value;
static lv_obj_t *s_sys_psram_detail;
static lv_obj_t *s_sys_psram_bar;
static lv_obj_t *s_sys_sd_state;
static lv_obj_t *s_sys_sd_value;
static lv_obj_t *s_sys_sd_detail;
static lv_obj_t *s_sys_sd_bar;
static lv_obj_t *s_sys_uptime;
static lv_obj_t *s_scale_bar;
static lv_obj_t *s_scale_label;
static lv_obj_t *s_settings_home;
static lv_obj_t *s_developer_settings_page;
static lv_obj_t *s_fps_overlay_switch;
static lv_obj_t *s_repeat_forwarding_switch;
static lv_obj_t *s_repeat_forwarding_status;
static lv_obj_t *s_device_name_page;
static lv_obj_t *s_device_name_input;
static lv_obj_t *s_device_name_value;
static lv_obj_t *s_device_name_keyboard;
static lv_obj_t *s_lora_page;
static lv_obj_t *s_lora_frequency;
static lv_obj_t *s_lora_bandwidth;
static lv_obj_t *s_lora_sf;
static lv_obj_t *s_lora_cr;
static lv_obj_t *s_lora_power;
static lv_obj_t *s_lora_rx_gain;
static lv_obj_t *s_lora_status;
static lv_obj_t *s_lora_keyboard;
static lv_obj_t *s_brightness_page;
static lv_obj_t *s_brightness_value;
static lv_obj_t *s_brightness_detail_value;
static lv_obj_t *s_timeout_page;
static lv_obj_t *s_timeout_value;
static lv_obj_t *s_timeout_dropdown;
static lv_obj_t *s_storage_value;
static uint32_t s_screen_timeout_seconds = 60;
static bool s_screen_sleeping;
static uint8_t s_awake_brightness = 70;
static char s_device_name[33] = "TinyTab";
static map_marker_t s_markers[MAP_MARKER_MAX];
static int s_marker_count;
static int s_gps_marker_first = -1;
static lv_obj_t *s_map_device_bubble;
static lv_obj_t *s_map_device_bubble_tail;
static lv_obj_t *s_map_device_name_lbl;
static lv_obj_t *s_map_device_expand_lbl;
static lv_obj_t *s_map_device_info_lbl;
static lv_obj_t *s_map_locate_status_lbl;
static bool s_map_device_bubble_expanded;
static bool s_gps_map_position_valid;
static double s_gps_map_latitude;
static double s_gps_map_longitude;
static lv_obj_t *s_settings_gps_value;
static lv_obj_t *s_gps_settings_page;
static lv_obj_t *s_gps_detail_state;
static lv_obj_t *s_gps_detail_stream;
static lv_obj_t *s_gps_detail_satellites;
static lv_obj_t *s_gps_detail_coordinates;
static lv_obj_t *s_chat_msg_area;
static lv_obj_t *s_chat_input;
static lv_obj_t *s_chat_input_bar;
static lv_obj_t *s_chat_pending_status;
static char s_chat_pending_text[MESHCORE_CHAT_TEXT_MAX_LEN + 1];
static lv_obj_t *s_devices_list;
static meshcore_device_info_t *s_device_snapshot;
static uint32_t s_devices_generation = UINT32_MAX;
static uint32_t s_devices_last_render_ms;

static void map_zoom_to(int new_zoom);
static void scale_bar_update(void);
static void map_marker_update_position(map_marker_t *marker);
static void gps_ui_timer_cb(lv_timer_t *timer);
static void map_locate_btn_cb(lv_event_t *event);
static void map_device_bubble_cb(lv_event_t *event);
static void gps_settings_entry_cb(lv_event_t *event);
static void gps_settings_back_cb(lv_event_t *event);
static void developer_settings_entry_cb(lv_event_t *event);
static void developer_settings_back_cb(lv_event_t *event);
static void fps_overlay_switch_cb(lv_event_t *event);
static void repeat_forwarding_switch_cb(lv_event_t *event);
static void device_name_entry_cb(lv_event_t *event);
static void lora_settings_entry_cb(lv_event_t *event);
static void brightness_entry_cb(lv_event_t *event);
static void timeout_entry_cb(lv_event_t *event);
static void settings_back_cb(lv_event_t *event);
static void settings_status_timer_cb(lv_timer_t *timer);
static void chat_send_event_cb(lv_event_t *event);
static void chat_kb_show_cb(lv_event_t *event);
static void chat_kb_hide_cb(lv_event_t *event);
static void chat_meshcore_poll_cb(lv_timer_t *timer);
static void devices_update_cb(lv_timer_t *timer);

/* LVGL's sysmon overlay is intentionally always visible in this benchmark.
 * It reports FPS, CPU load and render/flush time independently of the map UI. */
static void display_event_cb(lv_event_t *event)
{
    switch (lv_event_get_code(event)) {
    case LV_EVENT_REFR_READY:
        s_frame_count++;
        break;
    case LV_EVENT_RENDER_START:
        s_render_start_us = (uint32_t)esp_timer_get_time();
        break;
    case LV_EVENT_RENDER_READY:
        if (s_render_start_us != 0) {
            s_last_render_us = (uint32_t)esp_timer_get_time() - s_render_start_us;
        }
        break;
    default:
        break;
    }
}

static void *lodepng_malloc_psram(size_t size)
{
    return heap_caps_malloc(size ? size : 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

static void lodepng_free_psram(void *ptr)
{
    heap_caps_free(ptr);
}

static void *lodepng_realloc_psram(void *ptr, size_t size)
{
    return heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

/* lodepng is built by the LVGL component without its default allocators. */
void *lodepng_malloc(size_t size) { return lodepng_malloc_psram(size); }
void lodepng_free(void *ptr) { lodepng_free_psram(ptr); }
void *lodepng_realloc(void *ptr, size_t size) { return lodepng_realloc_psram(ptr, size); }

static uint16_t rgb565(const uint8_t *p)
{
    return (uint16_t)(((p[0] >> 3) << 11) | ((p[1] >> 2) << 5) | (p[2] >> 3));
}

static bool read_tile_png(const char *path, uint8_t **png_out, size_t *size_out)
{
    *png_out = NULL;
    *size_out = 0;
    FILE *file = fopen(path, "rb");
    if (file == NULL) return false;
    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return false; }
    long length = ftell(file);
    if (length <= 0 || fseek(file, 0, SEEK_SET) != 0) { fclose(file); return false; }

    uint8_t *png = heap_caps_malloc((size_t)length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (png == NULL || fread(png, 1, (size_t)length, file) != (size_t)length) {
        if (png) heap_caps_free(png);
        fclose(file);
        return false;
    }
    fclose(file);
    *png_out = png;
    *size_out = (size_t)length;
    return true;
}

/* Raw tiles are little-endian RGB565, exactly the format consumed by LVGL.
 * Reading them directly avoids the several-hundred-millisecond PNG inflate
 * and RGB conversion path on the ESP32-P4. */
static bool read_tile_rgb565(const char *path, uint8_t *out)
{
    FILE *file = fopen(path, "rb");
    if (file == NULL) return false;
    size_t got = fread(out, 1, TILE_BYTES, file);
    int extra = fgetc(file);
    fclose(file);
    return got == TILE_BYTES && extra == EOF;
}

static bool decode_tile_png(uint8_t *png, size_t png_size, uint8_t *out)
{
    unsigned width = 0, height = 0;
    lv_draw_buf_t *decoded = NULL;
    /* Offline tiles are opaque 8-bit palette PNGs. Expanding straight to RGB
     * avoids producing and then discarding a fourth alpha byte per pixel. */
    unsigned error = lodepng_decode24((unsigned char **)&decoded, &width, &height,
                                      png, png_size);
    if (error != 0 || decoded == NULL || decoded->data == NULL ||
        width == 0 || height == 0 || width > TILE_SIZE || height > TILE_SIZE) {
        if (decoded != NULL) lv_draw_buf_destroy(decoded);
        return false;
    }

    const uint8_t *rgb = decoded->data;
    if (width == TILE_SIZE && height == TILE_SIZE) {
        const size_t pixels = (size_t)width * height;
        for (size_t i = 0; i < pixels; i++) {
            uint16_t value = rgb565(&rgb[i * 3]);
            size_t offset = i * 2;
            out[offset] = (uint8_t)value;
            out[offset + 1] = (uint8_t)(value >> 8);
        }
    } else {
        memset(out, 0, TILE_BYTES);
        for (unsigned y = 0; y < height; y++) {
            for (unsigned x = 0; x < width; x++) {
                uint16_t value = rgb565(&rgb[((size_t)y * width + x) * 3]);
                size_t offset = ((size_t)y * TILE_SIZE + x) * 2;
                out[offset] = (uint8_t)value;
                out[offset + 1] = (uint8_t)(value >> 8);
            }
        }
    }
    lv_draw_buf_destroy(decoded);
    return true;
}

static bool tile_key_equal(const map_tile_t *tile, int z, int x, int y)
{
    return tile->z == z && tile->x == x && tile->y == y;
}

static void map_tile_prepare_source(map_tile_t *tile)
{
    tile->dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    tile->dsc.header.w = TILE_SIZE;
    tile->dsc.header.h = TILE_SIZE;
    tile->dsc.header.stride = TILE_SIZE * 2;
    tile->dsc.data = tile->pixels;
    tile->dsc.data_size = TILE_BYTES;
}

static void map_loader_release_dropped(const tile_request_t *request)
{
    if (request == NULL || request->slot >= TILE_COUNT) return;
    map_tile_t *tile = &s_tiles[request->slot];
    if (tile->token != request->token) return;
    tile->loading = false;
    tile->valid = false;
    tile->failed = false;
    tile->z = INT32_MIN;
    s_refresh_pending = true;
}

static void map_loader_send_result(const tile_request_t *request, bool ok)
{
    tile_result_t result = {
        .slot = request->slot,
        .token = request->token,
        .generation = request->generation,
        .ok = ok,
    };
    if (xQueueSend(s_result_queue, &result, pdMS_TO_TICKS(20)) != pdTRUE) {
        map_loader_release_dropped(request);
        if (s_result_drop_logs++ < 4) {
            ESP_LOGW(TAG, "tile result queue full; keeping loader responsive");
        }
    }
}

static void map_loader_task(void *arg)
{
    (void)arg;
    tile_request_t request;
    uint32_t failure_logs = 0;
    uint32_t completed = 0;
    uint64_t read_total = 0;
    uint64_t decode_total = 0;
    for (;;) {
        if (xQueueReceive(s_load_queue, &request, portMAX_DELAY) != pdTRUE) continue;
        if (request.generation != s_map_generation) {
            map_loader_send_result(&request, false);
            continue;
        }
        char path[192];
        char raw_path[192];
        map_tile_t *tile = request.slot < TILE_COUNT ? &s_tiles[request.slot] : NULL;
        if (tile == NULL) {
            map_loader_send_result(&request, false);
            continue;
        }
        int64_t started = esp_timer_get_time();
        uint32_t read_us = 0;
        uint32_t decode_us = 0;
        bool raw_rgb565 = false;
        bool ok = false;
        /* The converter emits .rgb565; accept .bin as well for files made by
         * the earlier map tooling. Prefer raw data before opening PNG. */
        if (s_raw_tile_tree) {
            snprintf(raw_path, sizeof(raw_path), "%s/%d/%d/%d.rgb565", s_map_root,
                     request.z, request.x, request.y);
            if (read_tile_rgb565(raw_path, tile->pixels)) {
                raw_rgb565 = true;
                ok = true;
            } else {
                snprintf(raw_path, sizeof(raw_path), "%s/%d/%d/%d.bin", s_map_root,
                         request.z, request.x, request.y);
                if (read_tile_rgb565(raw_path, tile->pixels)) {
                    raw_rgb565 = true;
                    ok = true;
                }
            }
        }
        read_us = (uint32_t)(esp_timer_get_time() - started);
        snprintf(path, sizeof(path), "%s/%d/%d/%d.png", s_map_root,
                 request.z, request.x, request.y);
        if (!ok) {
            uint8_t *png = NULL;
            size_t png_size = 0;
            int64_t decode_started = esp_timer_get_time();
            ok = read_tile_png(path, &png, &png_size);
            read_us = (uint32_t)(decode_started - started);
            if (ok) {
                /* Keep the original MUI/backup topology: one worker reads and
                 * decodes a tile, so no second task or decode queue can delay
                 * the next visible tile. */
                ok = decode_tile_png(png, png_size, tile->pixels);
                decode_us = (uint32_t)(esp_timer_get_time() - decode_started);
            }
            if (png != NULL) heap_caps_free(png);
        }
        if (raw_rgb565) {
            if (s_raw_tile_logs++ < 3) {
                ESP_LOGI(TAG, "raw RGB565 tile ready: %d/%d/%d read=%lu us",
                         request.z, request.x, request.y,
                         (unsigned long)read_us);
            }
        } else if (s_png_fallback_logs++ < 1) {
            ESP_LOGI(TAG, "PNG tile path active (no raw RGB565 tree): %d/%d/%d",
                     request.z, request.x, request.y);
        }
        if (!ok && failure_logs < 12) {
            ESP_LOGW(TAG, "tile load failed: %s", path);
            failure_logs++;
        }
        map_loader_send_result(&request, ok);
        if (ok && tile->loading && tile->token == request.token &&
            request.generation == s_map_generation) {
            completed++;
            read_total += read_us;
            decode_total += decode_us;
            if ((completed & 15U) == 0) {
                ESP_LOGI(TAG, "tile pipeline avg: read=%lu us decode=%lu us (%lu tiles)",
                         (unsigned long)(read_total / completed),
                         (unsigned long)(decode_total / completed),
                         (unsigned long)completed);
            }
        }
        /* Give IDLE1 and the MeshCore task a scheduling point between tiles.
         * This does not affect LVGL, which remains on CPU0. */
        vTaskDelay(1);
    }
}

static void map_loader_start(void)
{
    s_load_queue = xQueueCreate(LOAD_QUEUE_LEN, sizeof(tile_request_t));
    s_result_queue = xQueueCreate(RESULT_QUEUE_LEN, sizeof(tile_result_t));
    if (s_load_queue == NULL || s_result_queue == NULL) {
        ESP_LOGE(TAG, "tile queues unavailable");
        return;
    }
    BaseType_t loader_ok = xTaskCreatePinnedToCoreWithCaps(
        map_loader_task, "map_tile", 12288, NULL, 4, NULL, 1,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (loader_ok != pdPASS) {
        ESP_LOGE(TAG, "tile pipeline tasks unavailable");
    } else {
        ESP_LOGI(TAG, "tile pipeline ready: one async SD/PNG worker on core 1");
    }
}

static void scan_levels(void)
{
    s_level_count = 0;
    DIR *root = opendir(s_map_root);
    if (root == NULL) return;
    struct dirent *entry;
    while ((entry = readdir(root)) != NULL && s_level_count < 32) {
        char *end = NULL;
        long value = strtol(entry->d_name, &end, 10);
        if (end != entry->d_name && *end == '\0' && value >= 0 && value <= 30) {
            s_levels[s_level_count++] = (int)value;
        }
    }
    closedir(root);
    for (int i = 0; i < s_level_count - 1; i++) {
        for (int j = i + 1; j < s_level_count; j++) {
            if (s_levels[j] < s_levels[i]) {
                int temp = s_levels[i]; s_levels[i] = s_levels[j]; s_levels[j] = temp;
            }
        }
    }
}

static bool find_level_center(int z, double *center_x, double *center_y)
{
    char path[160];
    snprintf(path, sizeof(path), "%s/%d", s_map_root, z);
    DIR *xdir = opendir(path);
    if (xdir == NULL) return false;
    int min_x = INT32_MAX, max_x = -1;
    struct dirent *entry;
    while ((entry = readdir(xdir)) != NULL) {
        char *end = NULL;
        long x = strtol(entry->d_name, &end, 10);
        if (end != entry->d_name && *end == '\0') {
            if (x < min_x) min_x = (int)x;
            if (x > max_x) max_x = (int)x;
        }
    }
    closedir(xdir);
    if (max_x < 0) return false;

    int middle_x = (min_x + max_x) / 2;
    snprintf(path, sizeof(path), "%s/%d/%d", s_map_root, z, middle_x);
    DIR *ydir = opendir(path);
    if (ydir == NULL) return false;
    int min_y = INT32_MAX, max_y = -1;
    while ((entry = readdir(ydir)) != NULL) {
        char *end = NULL;
        long y = strtol(entry->d_name, &end, 10);
        if (end != entry->d_name &&
            (*end == '\0' || strcmp(end, ".png") == 0 ||
             strcmp(end, ".rgb565") == 0 || strcmp(end, ".bin") == 0)) {
            if (strcmp(end, ".rgb565") == 0 || strcmp(end, ".bin") == 0) {
                s_raw_tile_tree = true;
            }
            if (y < min_y) min_y = (int)y;
            if (y > max_y) max_y = (int)y;
        }
    }
    closedir(ydir);
    if (max_y < 0) return false;
    *center_x = (min_x + max_x) * 0.5;
    *center_y = (min_y + max_y) * 0.5;
    return true;
}

static bool map_root_has_levels(const char *root)
{
    DIR *dir = opendir(root);
    if (dir == NULL) return false;
    struct dirent *entry;
    char path[256];
    bool found = false;
    while ((entry = readdir(dir)) != NULL) {
        char *end = NULL;
        long level = strtol(entry->d_name, &end, 10);
        if (end == entry->d_name || *end != '\0' || level < 1 || level > 30) continue;
        snprintf(path, sizeof(path), "%.*s/%.*s", 95, root, 95, entry->d_name);
        DIR *level_dir = opendir(path);
        if (level_dir != NULL) {
            closedir(level_dir);
            found = true;
            break;
        }
    }
    closedir(dir);
    return found;
}

static void select_map_root(void)
{
    if (map_root_has_levels(s_map_root)) return;

    DIR *sd = opendir(RAW_MAP_ROOT);
    if (sd == NULL) return;
    struct dirent *entry;
    char candidate[256];
    while ((entry = readdir(sd)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        snprintf(candidate, sizeof(candidate), "%s/%.*s", RAW_MAP_ROOT, 95, entry->d_name);
        if (map_root_has_levels(candidate)) {
            snprintf(s_map_root, sizeof(s_map_root), "%.*s", (int)sizeof(s_map_root) - 1, candidate);
            ESP_LOGI(TAG, "map root selected: %s", s_map_root);
            break;
        }
    }
    closedir(sd);
}

static void map_position_raw(void)
{
    lv_obj_set_pos(s_map_layer, -(lv_coord_t)lround(s_raw_x),
                   -(lv_coord_t)lround(s_raw_y));
}

/* The existing map generator writes little-endian RGB565 files at the SD
 * root. Prefer the largest available image, then fall back to /sdcard/map. */
static bool raw_map_load(void)
{
    static const raw_map_info_t maps[] = {
        {"map8.raw", 1920, 2560, 19},
        {"map4.raw", 960, 1280, 18},
        {"map2.raw", 480, 640, 17},
        {"map1.raw", 240, 320, 16},
    };
    static const char *raw_roots[] = {RAW_MAP_ROOT, "/sdcard/map", "/sdcard/map_out"};
    char path[160];
    FILE *file = NULL;
    raw_map_info_t selected = {0};

    for (size_t i = 0; i < sizeof(maps) / sizeof(maps[0]); i++) {
        for (size_t root = 0; root < sizeof(raw_roots) / sizeof(raw_roots[0]); root++) {
            snprintf(path, sizeof(path), "%s/%s", raw_roots[root], maps[i].name);
            file = fopen(path, "rb");
            if (file != NULL) break;
        }
        if (file != NULL) {
            selected = maps[i];
            break;
        }
    }
    if (file == NULL) {
        ESP_LOGW(TAG, "no RAW map found; expected /sdcard/map8.raw or PNG tiles in %s", s_map_root);
        return false;
    }

    size_t bytes = (size_t)selected.width * selected.height * 2;
    s_raw_pixels = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_raw_pixels == NULL || fread(s_raw_pixels, 1, bytes, file) != bytes) {
        ESP_LOGE(TAG, "failed to read RAW map %s (%u bytes)", path, (unsigned)bytes);
        if (s_raw_pixels != NULL) heap_caps_free(s_raw_pixels);
        s_raw_pixels = NULL;
        fclose(file);
        return false;
    }
    fclose(file);

    s_raw_width = selected.width;
    s_raw_height = selected.height;
    s_raw_zoom = selected.zoom;
    s_zoom = s_raw_zoom;
    s_raw_mode = true;
    s_raw_dsc.header.cf = LV_COLOR_FORMAT_RGB565;
    s_raw_dsc.header.w = s_raw_width;
    s_raw_dsc.header.h = s_raw_height;
    s_raw_dsc.header.stride = s_raw_width * 2;
    s_raw_dsc.data = s_raw_pixels;
    s_raw_dsc.data_size = bytes;
    lv_obj_set_size(s_map_layer, s_raw_width, s_raw_height);
    s_raw_image = lv_image_create(s_map_layer);
    lv_image_set_antialias(s_raw_image, false);
    lv_image_set_src(s_raw_image, &s_raw_dsc);
    s_raw_x = s_raw_width > MAP_W ? (s_raw_width - MAP_W) * 0.5 : 0;
    s_raw_y = s_raw_height > MAP_H ? (s_raw_height - MAP_H) * 0.5 : 0;
    map_position_raw();
    ESP_LOGI(TAG, "RAW map loaded: %s %dx%d (%d bytes)", path,
             s_raw_width, s_raw_height, (int)bytes);
    return true;
}

static void map_position_tiles(void)
{
    int base_x = (int)floor(s_world_x / TILE_SIZE);
    int base_y = (int)floor(s_world_y / TILE_SIZE);
    s_grid_x0 = base_x - 1;
    s_grid_y0 = base_y - 1;
    lv_obj_set_pos(s_map_layer,
                   -(lv_coord_t)lround(s_world_x - (double)s_grid_x0 * TILE_SIZE),
                   -(lv_coord_t)lround(s_world_y - (double)s_grid_y0 * TILE_SIZE));
}

/* Marker coordinates use the same global Web-Mercator tile space as the
 * PNG requests. They remain children of the moving layer, so ordinary finger
 * motion costs one parent-position update just like the tile images. */
static void map_marker_update_position(map_marker_t *marker)
{
    if (marker == NULL || marker->obj == NULL || s_zoom < 0) return;
    lv_obj_set_pos(marker->obj,
                   (lv_coord_t)lround((marker->tx - s_grid_x0) * TILE_SIZE),
                   (lv_coord_t)lround((marker->ty - s_grid_y0) * TILE_SIZE));
}

static double gps_longitude_to_tile_x(double longitude, double zoom)
{
    return (longitude + 180.0) / 360.0 * pow(2.0, zoom);
}

static double gps_latitude_to_tile_y(double latitude, double zoom)
{
    const double pi = 3.14159265358979323846;
    if (latitude > 85.05112878) latitude = 85.05112878;
    if (latitude < -85.05112878) latitude = -85.05112878;
    const double radians = latitude * pi / 180.0;
    return (1.0 - log(tan(radians) + 1.0 / cos(radians)) / pi) * 0.5 * pow(2.0, zoom);
}

/* ATGM33 reports WGS84. The offline tiles are the usual mainland GCJ-02
 * source, so apply the same conversion used by the reference application. */
static bool gps_outside_china(double latitude, double longitude)
{
    return longitude < 72.004 || longitude > 137.8347 ||
           latitude < 0.8293 || latitude > 55.8271;
}

static double gps_gcj_transform_lat(double x, double y)
{
    const double pi = 3.14159265358979323846;
    double value = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y +
                   0.1 * x * y + 0.2 * sqrt(fabs(x));
    value += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    value += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    value += (160.0 * sin(y / 12.0 * pi) + 320.0 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return value;
}

static double gps_gcj_transform_lon(double x, double y)
{
    const double pi = 3.14159265358979323846;
    double value = 300.0 + x + 2.0 * y + 0.1 * x * x +
                   0.1 * x * y + 0.1 * sqrt(fabs(x));
    value += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    value += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    value += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return value;
}

static void gps_wgs84_to_gcj02(double wgs_latitude, double wgs_longitude,
                               double *gcj_latitude, double *gcj_longitude)
{
    const double pi = 3.14159265358979323846;
    const double semi_major_axis = 6378245.0;
    const double eccentricity_sq = 0.00669342162296594323;
    if (gcj_latitude == NULL || gcj_longitude == NULL) return;
    if (gps_outside_china(wgs_latitude, wgs_longitude)) {
        *gcj_latitude = wgs_latitude;
        *gcj_longitude = wgs_longitude;
        return;
    }

    double lat_offset = gps_gcj_transform_lat(wgs_longitude - 105.0,
                                               wgs_latitude - 35.0);
    double lon_offset = gps_gcj_transform_lon(wgs_longitude - 105.0,
                                               wgs_latitude - 35.0);
    const double rad_latitude = wgs_latitude / 180.0 * pi;
    const double sin_latitude = sin(rad_latitude);
    const double magic = 1.0 - eccentricity_sq * sin_latitude * sin_latitude;
    const double sqrt_magic = sqrt(magic);
    lat_offset = lat_offset * 180.0 /
                 ((semi_major_axis * (1.0 - eccentricity_sq)) /
                  (magic * sqrt_magic) * pi);
    lon_offset = lon_offset * 180.0 /
                 (semi_major_axis / sqrt_magic * cos(rad_latitude) * pi);
    *gcj_latitude = wgs_latitude + lat_offset;
    *gcj_longitude = wgs_longitude + lon_offset;
}

static void queue_tile(int slot, int x, int y)
{
    map_tile_t *tile = &s_tiles[slot];
    if (s_load_queue == NULL || tile->pixels == NULL || tile->loading) return;
    tile->z = s_zoom;
    tile->x = x;
    tile->y = y;
    tile->valid = false;
    tile->failed = false;
    tile->loading = true;
    tile->last_use = ++s_tile_use_clock;
    tile->token++;
    tile_request_t request = {.slot = (uint8_t)slot, .token = tile->token,
                              .generation = s_map_generation,
                              .z = tile->z, .x = tile->x, .y = tile->y};
    if (xQueueSend(s_load_queue, &request, 0) != pdTRUE) {
        tile->loading = false;
        tile->z = INT32_MIN;
    }
}

static bool tile_in_grid(const map_tile_t *tile)
{
    return tile->z == s_zoom &&
           tile->x >= s_grid_x0 && tile->x < s_grid_x0 + GRID_COLS &&
           tile->y >= s_grid_y0 && tile->y < s_grid_y0 + GRID_ROWS;
}

static int tile_find(int z, int x, int y)
{
    for (int i = 0; i < TILE_COUNT; i++) {
        map_tile_t *tile = &s_tiles[i];
        if ((tile->valid || tile->loading || tile->failed) &&
            tile_key_equal(tile, z, x, y)) {
            tile->last_use = ++s_tile_use_clock;
            return i;
        }
    }
    return -1;
}

static int tile_evict_slot(void)
{
    int fallback = -1;
    uint32_t oldest = UINT32_MAX;
    for (int i = 0; i < TILE_COUNT; i++) {
        map_tile_t *tile = &s_tiles[i];
        if (tile->pixels == NULL) continue;
        if (!tile->loading && !tile->valid && !tile->failed) return i;
        if (!tile->loading && !tile_in_grid(tile) && tile->last_use <= oldest) {
            oldest = tile->last_use;
            fallback = i;
        }
    }
    return fallback;
}

static void request_tile_key(int z, int x, int y)
{
    if (s_load_queue == NULL) return;
    if (tile_find(z, x, y) >= 0) return;

    int slot = tile_evict_slot();
    if (slot < 0) return;
    map_tile_t *tile = &s_tiles[slot];
    tile->token++;
    tile->loading = false;
    tile->valid = false;
    tile->failed = false;
    queue_tile(slot, x, y);
}

static void map_prefetch_margin(void)
{
    if (s_load_queue == NULL || s_zoom < 0) return;

    /* Load the strip in the direction of travel first. The queue is FIFO, so
     * these tiles are ready before the less urgent side of the ring. */
    if (s_pan_dir_x != 0) {
        for (int depth = 1; depth <= PREFETCH_DEPTH; depth++) {
            int x = s_pan_dir_x > 0 ? s_grid_x0 + GRID_COLS + depth - 1
                                    : s_grid_x0 - depth;
            for (int row = 0; row < GRID_ROWS; row++) {
                request_tile_key(s_zoom, x, s_grid_y0 + row);
            }
        }
    } else if (s_pan_dir_y != 0) {
        for (int depth = 1; depth <= PREFETCH_DEPTH; depth++) {
            int y = s_pan_dir_y > 0 ? s_grid_y0 + GRID_ROWS + depth - 1
                                    : s_grid_y0 - depth;
            for (int col = 0; col < GRID_COLS; col++) {
                request_tile_key(s_zoom, s_grid_x0 + col, y);
            }
        }
    }

}

static void map_refresh_grid(void)
{
    if (s_zoom < 0) return;
    map_position_tiles();

    for (int i = 0; i < GRID_COUNT; i++) {
        if (s_cells[i].image != NULL) lv_obj_add_flag(s_cells[i].image, LV_OBJ_FLAG_HIDDEN);
    }

    /* Queue the edge in the drag direction first. The right edge is the most
     * noticeable one during a horizontal drag; putting it at the head of the
     * FIFO prevents the background ring from delaying those visible tiles. */
    if (s_pan_dir_x != 0) {
        const int first_col = s_pan_dir_x > 0 ? GRID_COLS - 1 : 0;
        const int last_col = s_pan_dir_x > 0 ? -1 : GRID_COLS;
        const int step = s_pan_dir_x > 0 ? -1 : 1;
        for (int col = first_col; col != last_col; col += step) {
            for (int row = 0; row < GRID_ROWS; row++) {
                const int x = s_grid_x0 + col;
                const int y = s_grid_y0 + row;
                int slot = tile_find(s_zoom, x, y);
                if (slot < 0) {
                    slot = tile_evict_slot();
                    if (slot >= 0) queue_tile(slot, x, y);
                }
            }
        }
    } else if (s_pan_dir_y != 0) {
        const int first_row = s_pan_dir_y > 0 ? GRID_ROWS - 1 : 0;
        const int last_row = s_pan_dir_y > 0 ? -1 : GRID_ROWS;
        const int step = s_pan_dir_y > 0 ? -1 : 1;
        for (int row = first_row; row != last_row; row += step) {
            for (int col = 0; col < GRID_COLS; col++) {
                const int x = s_grid_x0 + col;
                const int y = s_grid_y0 + row;
                int slot = tile_find(s_zoom, x, y);
                if (slot < 0) {
                    slot = tile_evict_slot();
                    if (slot >= 0) queue_tile(slot, x, y);
                }
            }
        }
    }

    /* Fill the remaining viewport from the centre outward. */
    const int centre_col = GRID_COLS / 2;
    const int centre_row = GRID_ROWS / 2;
    for (int distance = 0; distance <= GRID_COLS + GRID_ROWS; distance++) {
        for (int row = 0; row < GRID_ROWS; row++) {
            for (int col = 0; col < GRID_COLS; col++) {
                if (abs(col - centre_col) + abs(row - centre_row) != distance) continue;
                int x = s_grid_x0 + col;
                int y = s_grid_y0 + row;
                int cell_index = row * GRID_COLS + col;
                map_cell_t *cell = &s_cells[cell_index];
                cell->z = s_zoom;
                cell->x = x;
                cell->y = y;
                lv_obj_set_pos(cell->image, col * TILE_SIZE, row * TILE_SIZE);
                int slot = tile_find(s_zoom, x, y);
                if (slot < 0) {
                    slot = tile_evict_slot();
                    if (slot >= 0) queue_tile(slot, x, y);
                }
                if (slot < 0) continue;
                map_tile_t *tile = &s_tiles[slot];
                if (tile->valid && !tile->loading) {
                    lv_image_set_src(cell->image, &tile->dsc);
                    lv_obj_clear_flag(cell->image, LV_OBJ_FLAG_HIDDEN);
                }
            }
        }
    }
    for (int i = 0; i < s_marker_count; i++) {
        map_marker_update_position(&s_markers[i]);
    }
    s_refresh_pending = false;
}

static void map_results_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (s_raw_mode) return;
    tile_result_t result;
    unsigned current_budget = 2;
    bool released_stale = false;
    int stale_budget = 16;
    while (s_result_queue != NULL && stale_budget-- > 0 &&
           xQueuePeek(s_result_queue, &result, 0) == pdTRUE) {
        if (result.slot >= TILE_COUNT) {
            (void)xQueueReceive(s_result_queue, &result, 0);
            continue;
        }
        map_tile_t *tile = &s_tiles[result.slot];
        const bool stale = !tile->loading || tile->token != result.token ||
                           result.generation != s_map_generation ||
                           tile->z == INT32_MIN;
        /* Keep a small per-frame budget: two tiles improve initial map fill,
         * while avoiding the large invalidation burst from the old unbounded
         * result loop. */
        if (!stale && current_budget == 0) break;
        (void)xQueueReceive(s_result_queue, &result, 0);
        if (stale) {
            /* Only release the slot if this result still belongs to its
             * current request. Never alter a newer request's state. */
            if (tile->loading && tile->token == result.token) {
                tile->loading = false;
                tile->valid = false;
                tile->failed = false;
                tile->z = INT32_MIN;
            }
            released_stale = true;
            continue;
        }
        tile->loading = false;
        tile->valid = result.ok;
        tile->failed = !result.ok;
        current_budget--;
        if (result.ok) {
            map_tile_prepare_source(tile);
            if (tile_in_grid(tile)) {
                const int col = tile->x - s_grid_x0;
                const int row = tile->y - s_grid_y0;
                if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
                    map_cell_t *cell = &s_cells[row * GRID_COLS + col];
                    if (cell->z == tile->z && cell->x == tile->x && cell->y == tile->y) {
                        lv_image_set_src(cell->image, &tile->dsc);
                        lv_obj_clear_flag(cell->image, LV_OBJ_FLAG_HIDDEN);
                    }
                }
            }
            if (s_map_message != NULL) {
                lv_obj_add_flag(s_map_message, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    if (released_stale) {
        /* Refill slots released by cancelled zoom requests once per tick,
         * after the stale batch, instead of once per stale tile. */
        s_refresh_pending = true;
        map_refresh_grid();
    }
}

/* Web Mercator ground resolution at the equator. Keep the same 1/2/5 scale
 * progression as the original map UI so each zoom step remains predictable. */
static void scale_bar_update(void)
{
    if (s_scale_bar == NULL || s_scale_label == NULL || s_zoom < 0) return;

    const double metres_per_pixel = 40075016.686 / (pow(2.0, s_zoom) * TILE_SIZE);
    const double target_metres = metres_per_pixel * 80.0;
    const double magnitude = pow(10.0, floor(log10(target_metres)));
    const double normalized = target_metres / magnitude;
    double step;
    if (normalized < 1.5) {
        step = 1.0;
    } else if (normalized < 3.5) {
        step = 2.0;
    } else if (normalized < 7.5) {
        step = 5.0;
    } else {
        step = 10.0;
    }
    const double nice_metres = step * magnitude;
    int pixels = (int)lround(nice_metres / metres_per_pixel);
    if (pixels < 24) pixels = 24;
    if (pixels > 120) pixels = 120;
    lv_obj_set_width(s_scale_bar, pixels);

    char text[20];
    if (nice_metres >= 1000.0) {
        const double kilometres = nice_metres / 1000.0;
        if (kilometres >= 10.0 || fabs(kilometres - round(kilometres)) < 0.05) {
            snprintf(text, sizeof(text), "%.0f km", kilometres);
        } else {
            snprintf(text, sizeof(text), "%.1f km", kilometres);
        }
    } else {
        snprintf(text, sizeof(text), "%.0f m", nice_metres);
    }
    lv_label_set_text(s_scale_label, text);
    lv_obj_align_to(s_scale_label, s_scale_bar, LV_ALIGN_OUT_TOP_LEFT, 0, -2);
}

static void map_gps_marker_visibility(bool visible)
{
    if (s_gps_marker_first < 0) return;
    for (int i = 0; i < 3; i++) {
        lv_obj_t *obj = s_markers[s_gps_marker_first + i].obj;
        if (obj == NULL) continue;
        if (visible) lv_obj_remove_flag(obj, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    }
}

static void map_gps_marker_update(void)
{
    if (!s_gps_map_position_valid || s_gps_marker_first < 0 ||
        s_zoom < 0 || s_raw_mode) {
        map_gps_marker_visibility(false);
        return;
    }

    double map_latitude;
    double map_longitude;
    gps_wgs84_to_gcj02(s_gps_map_latitude, s_gps_map_longitude,
                       &map_latitude, &map_longitude);
    const double tile_x = gps_longitude_to_tile_x(map_longitude, s_zoom);
    const double tile_y = gps_latitude_to_tile_y(map_latitude, s_zoom);
    for (int i = 0; i < 3; i++) {
        s_markers[s_gps_marker_first + i].tx = tile_x;
        s_markers[s_gps_marker_first + i].ty = tile_y;
        map_marker_update_position(&s_markers[s_gps_marker_first + i]);
    }
    map_gps_marker_visibility(true);
}

static void map_device_bubble_apply_state(void)
{
    if (s_map_device_bubble == NULL) return;
    const lv_coord_t width = s_map_device_bubble_expanded ? 220 : 132;
    const lv_coord_t height = s_map_device_bubble_expanded ? 112 : 42;
    lv_obj_set_size(s_map_device_bubble, width, height);
    lv_obj_set_style_translate_x(s_map_device_bubble, -width / 2, 0);
    lv_obj_set_style_translate_y(s_map_device_bubble, -(height + 16), 0);
    lv_obj_set_width(s_map_device_name_lbl, width - 42);
    lv_obj_set_pos(s_map_device_expand_lbl, width - 22, 11);
    lv_obj_set_width(s_map_device_info_lbl, width - 28);
    if (s_map_device_bubble_expanded) {
        lv_obj_remove_flag(s_map_device_info_lbl, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_map_device_info_lbl, LV_OBJ_FLAG_HIDDEN);
    }
    lv_label_set_text(s_map_device_expand_lbl,
                      s_map_device_bubble_expanded ? LV_SYMBOL_UP : LV_SYMBOL_DOWN);
}

static void map_device_bubble_cb(lv_event_t *event)
{
    (void)event;
    s_map_device_bubble_expanded = !s_map_device_bubble_expanded;
    map_device_bubble_apply_state();
}

static double map_200m_zoom_level(void)
{
    if (s_level_count == 0) return s_zoom;
    const double target_mpp = 200.0 / 80.0;
    const double ideal_zoom = log(40075016.686 / (256.0 * target_mpp)) / log(2.0);
    int best = s_levels[0];
    double best_delta = fabs((double)best - ideal_zoom);
    for (int i = 1; i < s_level_count; i++) {
        const double delta = fabs((double)s_levels[i] - ideal_zoom);
        if (delta < best_delta) {
            best = s_levels[i];
            best_delta = delta;
        }
    }
    return best;
}

static void map_locate_btn_cb(lv_event_t *event)
{
    (void)event;
    gps_data_t gps = {0};
    if (!GPS_Get_Data(&gps) || !gps.receiving || !gps.valid ||
        s_level_count == 0 || s_raw_mode) {
        if (s_map_locate_status_lbl != NULL) {
            lv_label_set_text(s_map_locate_status_lbl,
                              s_raw_mode ? "Tile map required" : "GPS fix required");
            lv_obj_set_style_text_color(s_map_locate_status_lbl,
                                        lv_color_hex(0xB26A00), 0);
        }
        return;
    }

    s_gps_map_latitude = gps.latitude;
    s_gps_map_longitude = gps.longitude;
    s_gps_map_position_valid = true;
    const int target_zoom = (int)map_200m_zoom_level();
    map_zoom_to(target_zoom);

    double map_latitude;
    double map_longitude;
    gps_wgs84_to_gcj02(gps.latitude, gps.longitude,
                       &map_latitude, &map_longitude);
    s_world_x = gps_longitude_to_tile_x(map_longitude, s_zoom) * TILE_SIZE - MAP_W * 0.5;
    s_world_y = gps_latitude_to_tile_y(map_latitude, s_zoom) * TILE_SIZE - MAP_H * 0.5;
    s_refresh_pending = true;
    map_refresh_grid();
    map_gps_marker_update();
    scale_bar_update();
    ESP_LOGI(TAG, "GPS centered: WGS84 %.6f, %.6f", gps.latitude, gps.longitude);
}

static void gps_ui_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    gps_data_t gps = {0};
    const bool available = GPS_Get_Data(&gps);
    if (s_settings_gps_value != NULL) {
        if (!available) lv_label_set_text(s_settings_gps_value, "Unavailable");
        else if (gps.valid) lv_label_set_text_fmt(s_settings_gps_value,
                                                   "Fixed / %u sat", gps.satellites);
        else if (gps.receiving) lv_label_set_text_fmt(s_settings_gps_value,
                                                      "Searching / %u sat", gps.satellites);
        else lv_label_set_text(s_settings_gps_value, "Waiting");
    }
    if (s_gps_detail_state != NULL) {
        const char *state = !available ? "GPS unavailable" :
                            !gps.receiving ? "No NMEA stream" :
                            !gps.valid ? "Receiving NMEA / searching" : "Position fixed";
        const uint32_t color = !available || !gps.receiving ? 0xB3261E :
                               !gps.valid ? 0xB26A00 : 0x188038;
        lv_label_set_text(s_gps_detail_state, state);
        lv_obj_set_style_text_color(s_gps_detail_state, lv_color_hex(color), 0);
        lv_label_set_text(s_gps_detail_stream,
                          available && gps.receiving ?
                          "NMEA active  /  UART1  /  RX IO2  /  9600 baud" :
                          "UART1  /  RX IO2  /  TX IO3  /  9600 baud");
        lv_label_set_text_fmt(s_gps_detail_satellites, "%u satellites",
                              gps.satellites);
        if (gps.valid) {
            lv_label_set_text_fmt(s_gps_detail_coordinates,
                                  "WGS84  %.6f, %.6f", gps.latitude, gps.longitude);
        } else {
            lv_label_set_text(s_gps_detail_coordinates, "Waiting for a valid fix");
        }
    }

    if (s_map_device_info_lbl != NULL) {
        if (available && gps.valid) {
            const float speed = !isfinite(gps.speed_kmh) || gps.speed_kmh < 0.5f
                                    ? 0.0f : gps.speed_kmh;
            const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            char last_fix_text[24];
            char info_text[96];
            if (gps.last_fix_ms > 0) {
                const uint32_t age_s = (now_ms - gps.last_fix_ms) / 1000;
                snprintf(last_fix_text, sizeof(last_fix_text), "%lu s ago",
                         (unsigned long)age_s);
            } else {
                snprintf(last_fix_text, sizeof(last_fix_text), "--");
            }
            /* Format floats with libc first; LVGL's formatter may have float support disabled. */
            snprintf(info_text, sizeof(info_text),
                     "satellites: %u\nspeed: %.1f km/h\nlast fix: %s",
                     gps.satellites, speed, last_fix_text);
            lv_label_set_text(s_map_device_info_lbl, info_text);
        } else if (available && gps.receiving) {
            lv_label_set_text_fmt(s_map_device_info_lbl,
                                  "satellites: %u\nspeed: 0.0 km/h\nlast fix: searching",
                                  gps.satellites);
        } else {
            lv_label_set_text(s_map_device_info_lbl,
                              "satellites: --\nspeed: 0.0 km/h\nlast fix: --");
        }
    }

    if (available && gps.valid) {
        s_gps_map_latitude = gps.latitude;
        s_gps_map_longitude = gps.longitude;
        s_gps_map_position_valid = true;
        map_gps_marker_update();
    } else if (!s_gps_map_position_valid) {
        map_gps_marker_visibility(false);
    }
}

/* Change zoom while keeping the view center fixed. Tiles are always rendered
 * at native 256x256 size, so changing level only swaps the requested PNG set. */
static void map_zoom_to(int new_zoom)
{
    if (s_raw_mode || s_zoom < 0 || new_zoom == s_zoom ||
        s_level_count == 0) return;

    int min_zoom = s_levels[0];
    int max_zoom = s_levels[s_level_count - 1];
    if (new_zoom < min_zoom) new_zoom = min_zoom;
    if (new_zoom > max_zoom) new_zoom = max_zoom;
    if (new_zoom == s_zoom) return;

    const double scale = pow(2.0, (double)(new_zoom - s_zoom));
    const double focus_x = MAP_W * 0.5;
    const double focus_y = MAP_H * 0.5;
    s_world_x = (s_world_x + focus_x) * scale - focus_x;
    s_world_y = (s_world_y + focus_y) * scale - focus_y;
    s_zoom = new_zoom;

    /* Cancel queued work but retain completed tiles from every zoom level.
     * Returning to a recent zoom can then reuse PSRAM immediately. */
    s_map_generation++;
    for (int i = 0; i < GRID_COUNT; i++) {
        if (s_cells[i].image != NULL) lv_obj_add_flag(s_cells[i].image, LV_OBJ_FLAG_HIDDEN);
    }
    s_refresh_pending = true;
    ESP_LOGI(TAG, "zoom changed: z=%d center=(%.1f,%.1f)",
             s_zoom, s_world_x, s_world_y);
    map_refresh_grid();
    map_gps_marker_update();
    scale_bar_update();
}

static void map_zoom_btn_cb(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_CLICKED) return;
    const bool zoom_in = lv_event_get_user_data(event) != NULL;
    map_zoom_to(s_zoom + (zoom_in ? 1 : -1));
}

static void map_pan_event_cb(lv_event_t *event)
{
    lv_indev_t *indev = lv_event_get_indev(event);
    if (indev == NULL) return;
    lv_point_t point;
    lv_indev_get_point(indev, &point);
    switch (lv_event_get_code(event)) {
    case LV_EVENT_PRESSED:
        s_pan_active = true;
        s_pan_last = point;
        s_pan_dir_x = 0;
        s_pan_dir_y = 0;
        break;
    case LV_EVENT_PRESSING:
        if (!s_pan_active) break;
        if (s_raw_mode) {
            s_raw_x -= (point.x - s_pan_last.x);
            s_raw_y -= (point.y - s_pan_last.y);
            const double max_x = s_raw_width > MAP_W ? s_raw_width - MAP_W : 0;
            const double max_y = s_raw_height > MAP_H ? s_raw_height - MAP_H : 0;
            if (s_raw_x < 0) s_raw_x = 0;
            if (s_raw_y < 0) s_raw_y = 0;
            if (s_raw_x > max_x) s_raw_x = max_x;
            if (s_raw_y > max_y) s_raw_y = max_y;
            s_pan_last = point;
            map_position_raw();
            break;
        }
        s_world_x -= (point.x - s_pan_last.x);
        s_world_y -= (point.y - s_pan_last.y);
        const lv_coord_t dx = point.x - s_pan_last.x;
        const lv_coord_t dy = point.y - s_pan_last.y;
        if (abs((int)dx) >= abs((int)dy) && dx != 0) {
            s_pan_dir_x = dx > 0 ? -1 : 1;
            s_pan_dir_y = 0;
        } else if (dy != 0) {
            s_pan_dir_x = 0;
            s_pan_dir_y = dy > 0 ? -1 : 1;
        }
        const int old_grid_x0 = s_grid_x0;
        const int old_grid_y0 = s_grid_y0;
        s_pan_last = point;
        map_position_tiles();
        if (old_grid_x0 != s_grid_x0 || old_grid_y0 != s_grid_y0) {
            /* Requests queued for the previous viewport are now stale. The
             * worker drops them before touching the SD card. */
            s_map_generation++;
            s_refresh_pending = true;
            map_refresh_grid();
        }
        break;
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        s_pan_active = false;
        /* Keep touch-time work to one parent-position update. Prefetch only
         * after the gesture settles, matching the MUI/map_tiles approach. */
        map_prefetch_margin();
        break;
    default:
        break;
    }
}

static void status_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    uint32_t now = lv_tick_get();
    uint32_t elapsed = now - s_frame_window_start;
    uint32_t fps = elapsed ? (s_frame_count * 1000U / elapsed) : 0;
    uint64_t uptime = esp_timer_get_time() / 1000000ULL;
    lv_label_set_text_fmt(s_status_time, "%02u:%02u:%02u  %lu FPS",
                          (unsigned)((uptime / 3600) % 24),
                          (unsigned)((uptime / 60) % 60),
                          (unsigned)(uptime % 60), (unsigned long)fps);
    lv_label_set_text_fmt(s_status_info,
                          LV_SYMBOL_CHARGE " LoRa %s",
                          meshcore_core_is_running() ? "OK" : "Starting");
    s_frame_count = 0;
    s_frame_window_start = now;
    if (s_refresh_pending) map_refresh_grid();
}

static lv_obj_t *map_zoom_button_create(lv_obj_t *parent, const char *text,
                                        int y, bool zoom_in)
{
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_pos(button, MAP_W - 48, y);
    lv_obj_set_size(button, 38, 38);
    lv_obj_set_style_radius(button, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(0x37474F), 0);
    lv_obj_set_style_bg_opa(button, LV_OPA_80, 0);
    lv_obj_set_style_border_width(button, 1, 0);
    lv_obj_set_style_border_color(button, lv_color_white(), 0);
    lv_obj_set_style_border_opa(button, LV_OPA_40, 0);
    lv_obj_add_event_cb(button, map_zoom_btn_cb, LV_EVENT_CLICKED,
                        zoom_in ? (void *)(intptr_t)1 : NULL);

    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
    lv_obj_center(label);
    return button;
}

static lv_obj_t *system_card_create(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                                    lv_coord_t width, lv_coord_t height)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_size(card, width, height);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_bg_color(card, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0xDDE3EA), 0);
    lv_obj_set_style_radius(card, 7, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    return card;
}

static lv_obj_t *system_label_create(lv_obj_t *parent, const char *text,
                                     lv_coord_t x, lv_coord_t y,
                                     const lv_font_t *font, uint32_t color)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    return label;
}

static lv_obj_t *system_bar_create(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                                   lv_coord_t width, uint32_t color)
{
    lv_obj_t *bar = lv_bar_create(parent);
    lv_obj_set_pos(bar, x, y);
    lv_obj_set_size(bar, width, 8);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_radius(bar, 4, LV_PART_MAIN);
    lv_obj_set_style_radius(bar, 4, LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xE9EDF2), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(bar, lv_color_hex(color), LV_PART_INDICATOR);
    return bar;
}

static void system_format_bytes(char *buffer, size_t length, uint64_t bytes)
{
    const uint64_t kib = 1024ULL;
    const uint64_t mib = 1024ULL * 1024ULL;
    const uint64_t gib = 1024ULL * 1024ULL * 1024ULL;
    uint64_t unit;
    const char *suffix;
    if (bytes >= gib) {
        unit = gib;
        suffix = "GB";
    } else if (bytes >= mib) {
        unit = mib;
        suffix = "MB";
    } else {
        unit = kib;
        suffix = "KB";
    }
    uint64_t whole64 = bytes / unit;
    uint32_t whole = whole64 > UINT32_MAX ? UINT32_MAX : (uint32_t)whole64;
    uint32_t tenth = (uint32_t)(((bytes % unit) * 10ULL) / unit);
    if (tenth > 9) tenth = 9;
    snprintf(buffer, length, "%lu.%lu %s",
             (unsigned long)whole, (unsigned long)tenth, suffix);
}

static void system_update_memory(lv_obj_t *value, lv_obj_t *detail,
                                 lv_obj_t *bar, uint32_t caps)
{
    uint64_t total = heap_caps_get_total_size(caps);
    uint64_t free = heap_caps_get_free_size(caps);
    uint64_t largest = heap_caps_get_largest_free_block(caps);
    uint64_t used = total >= free ? total - free : 0;
    int percent = total > 0 ? (int)(used * 100ULL / total) : 0;
    char used_text[24], total_text[24], free_text[24], block_text[24];
    system_format_bytes(used_text, sizeof(used_text), used);
    system_format_bytes(total_text, sizeof(total_text), total);
    system_format_bytes(free_text, sizeof(free_text), free);
    system_format_bytes(block_text, sizeof(block_text), largest);
    lv_label_set_text_fmt(value, "%s / %s", used_text, total_text);
    lv_label_set_text_fmt(detail, "%d%% used  |  Free %s  |  Block %s",
                          percent, free_text, block_text);
    lv_bar_set_value(bar, percent, LV_ANIM_OFF);
}

static void system_update_cb(lv_timer_t *timer)
{
    if (s_sys_ram_value == NULL) return;
    if (timer != NULL && s_tabview != NULL &&
        lv_tabview_get_tab_active(s_tabview) != 4) return;

    system_update_memory(s_sys_ram_value, s_sys_ram_detail, s_sys_ram_bar,
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    system_update_memory(s_sys_psram_value, s_sys_psram_detail, s_sys_psram_bar,
                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    static uint8_t sd_countdown;
    if (sd_countdown == 0) {
        uint64_t total = 0;
        uint64_t used = 0;
        if (SD_Get_Usage(&total, &used)) {
            uint64_t free = total >= used ? total - used : 0;
            int percent = total > 0 ? (int)(used * 100ULL / total) : 0;
            char used_text[24], total_text[24], free_text[24];
            system_format_bytes(used_text, sizeof(used_text), used);
            system_format_bytes(total_text, sizeof(total_text), total);
            system_format_bytes(free_text, sizeof(free_text), free);
            lv_label_set_text(s_sys_sd_state, "SD card mounted");
            lv_obj_set_style_text_color(s_sys_sd_state, lv_color_hex(0x188038), 0);
            lv_label_set_text_fmt(s_sys_sd_value, "%s / %s", used_text, total_text);
            lv_label_set_text_fmt(s_sys_sd_detail, "%d%% used  |  Free %s  |  /sdcard",
                                  percent, free_text);
            lv_bar_set_value(s_sys_sd_bar, percent, LV_ANIM_OFF);
        } else {
            lv_label_set_text(s_sys_sd_state, "No SD card");
            lv_obj_set_style_text_color(s_sys_sd_state, lv_color_hex(0xB3261E), 0);
            lv_label_set_text(s_sys_sd_value, "Storage unavailable");
            lv_label_set_text(s_sys_sd_detail, "Insert or remount a FAT formatted card");
            lv_bar_set_value(s_sys_sd_bar, 0, LV_ANIM_OFF);
        }
        sd_countdown = 5;
    }
    sd_countdown--;

    uint64_t uptime = (uint64_t)esp_timer_get_time() / 1000000ULL;
    lv_label_set_text_fmt(s_sys_uptime, LV_SYMBOL_REFRESH "  Uptime %02llu:%02llu:%02llu",
                          (unsigned long long)(uptime / 3600ULL),
                          (unsigned long long)((uptime % 3600ULL) / 60ULL),
                          (unsigned long long)(uptime % 60ULL));
}

static void system_page_create(lv_obj_t *parent)
{
    lv_obj_set_style_pad_all(parent, 0, 0);
    lv_obj_set_style_bg_color(parent, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    system_label_create(parent, "Device Manager", 20, 14,
                        &lv_font_montserrat_24, 0x172033);
    system_label_create(parent, "Live hardware, memory and storage telemetry",
                        20, 43, &lv_font_montserrat_12, 0x697386);
    lv_obj_t *live_dot = lv_obj_create(parent);
    lv_obj_set_pos(live_dot, 781, 27);
    lv_obj_set_size(live_dot, 9, 9);
    lv_obj_remove_flag(live_dot, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(live_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(live_dot, 0, 0);
    lv_obj_set_style_bg_color(live_dot, lv_color_hex(0x21A366), 0);
    system_label_create(parent, "LIVE", 798, 23, &lv_font_montserrat_12, 0x526071);

    lv_obj_t *cpu = system_card_create(parent, 18, 72, 270, 154);
    system_label_create(cpu, LV_SYMBOL_CHARGE "  PROCESSOR", 16, 14,
                        &lv_font_montserrat_12, 0x1769AA);
    lv_obj_t *cpu_value = system_label_create(cpu, "", 16, 46,
                                               &lv_font_montserrat_24, 0x172033);
    lv_label_set_text_fmt(cpu_value, "%d MHz", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    lv_obj_t *model = system_label_create(cpu, "", 16, 82,
                                           &lv_font_montserrat_14, 0x475569);
    lv_label_set_text_fmt(model, "ESP32-P4  /  %u cores", chip.cores);
    lv_obj_t *revision = system_label_create(cpu, "", 16, 114,
                                              &lv_font_montserrat_12, 0x778292);
    lv_label_set_text_fmt(revision, "RISC-V  |  Silicon revision %u.%u",
                          chip.revision / 100, chip.revision % 100);

    lv_obj_t *ram = system_card_create(parent, 300, 72, 270, 154);
    system_label_create(ram, LV_SYMBOL_HOME "  INTERNAL RAM", 16, 14,
                        &lv_font_montserrat_12, 0x188038);
    s_sys_ram_value = system_label_create(ram, "-- / --", 16, 47,
                                           &lv_font_montserrat_20, 0x172033);
    s_sys_ram_detail = system_label_create(ram, "Waiting for heap data", 16, 84,
                                            &lv_font_montserrat_12, 0x778292);
    lv_obj_set_width(s_sys_ram_detail, 238);
    lv_label_set_long_mode(s_sys_ram_detail, LV_LABEL_LONG_MODE_DOTS);
    s_sys_ram_bar = system_bar_create(ram, 16, 121, 238, 0x21A366);

    lv_obj_t *psram = system_card_create(parent, 582, 72, 270, 154);
    system_label_create(psram, LV_SYMBOL_DRIVE "  PSRAM", 16, 14,
                        &lv_font_montserrat_12, 0x7B5CC7);
    s_sys_psram_value = system_label_create(psram, "-- / --", 16, 47,
                                             &lv_font_montserrat_20, 0x172033);
    s_sys_psram_detail = system_label_create(psram, "Waiting for heap data", 16, 84,
                                              &lv_font_montserrat_12, 0x778292);
    lv_obj_set_width(s_sys_psram_detail, 238);
    lv_label_set_long_mode(s_sys_psram_detail, LV_LABEL_LONG_MODE_DOTS);
    s_sys_psram_bar = system_bar_create(psram, 16, 121, 238, 0x7B5CC7);

    lv_obj_t *network = system_card_create(parent, 18, 238, 410, 226);
    system_label_create(network, LV_SYMBOL_CHARGE "  RADIO", 18, 16,
                        &lv_font_montserrat_14, 0x1769AA);
    system_label_create(network, "LoRa radio", 18, 50,
                        &lv_font_montserrat_20, 0x8A5A16);
    system_label_create(network, "Benchmark build", 18, 92,
                        &lv_font_montserrat_14, 0x344054);
    system_label_create(network, "LoRa + SX1262 start at boot",
                        18, 126, &lv_font_montserrat_12, 0x778292);

    lv_obj_t *storage = system_card_create(parent, 440, 238, 412, 226);
    system_label_create(storage, LV_SYMBOL_SD_CARD "  STORAGE", 18, 16,
                        &lv_font_montserrat_14, 0xB45F06);
    s_sys_sd_state = system_label_create(storage, "Checking SD card...", 18, 50,
                                          &lv_font_montserrat_20, 0x172033);
    s_sys_sd_value = system_label_create(storage, "-- / --", 18, 90,
                                          &lv_font_montserrat_18, 0x344054);
    s_sys_sd_detail = system_label_create(storage, "Mounted at /sdcard", 18, 124,
                                           &lv_font_montserrat_12, 0x778292);
    s_sys_sd_bar = system_bar_create(storage, 18, 155, 376, 0xE28A24);
    system_label_create(storage, "Offline map tiles and application media",
                        18, 190, &lv_font_montserrat_12, 0x778292);

    lv_obj_t *footer = system_card_create(parent, 18, 476, 834, 62);
    s_sys_uptime = system_label_create(footer, LV_SYMBOL_REFRESH "  Uptime --",
                                        16, 20, &lv_font_montserrat_12, 0x475569);
    uint32_t flash_bytes = 0;
    esp_flash_get_physical_size(NULL, &flash_bytes);
    lv_obj_t *flash = system_label_create(footer, "", 218, 20,
                                           &lv_font_montserrat_12, 0x475569);
    lv_label_set_text_fmt(flash, LV_SYMBOL_DRIVE "  Flash %lu MB",
                          (unsigned long)(flash_bytes / (1024UL * 1024UL)));
    lv_obj_t *software = system_label_create(footer, "", 408, 20,
                                              &lv_font_montserrat_12, 0x778292);
    lv_label_set_text_fmt(software, "IDF %s  /  LVGL %d.%d.%d",
                          esp_get_idf_version(), LVGL_VERSION_MAJOR,
                          LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);

    lv_timer_create(system_update_cb, 1000, NULL);
    system_update_cb(NULL);
}

static lv_obj_t *ui_card_create(lv_obj_t *parent, lv_coord_t x, lv_coord_t y,
                                lv_coord_t width, lv_coord_t height)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_size(card, width, height);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(card, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0xDDE3EA), 0);
    lv_obj_set_style_radius(card, 7, 0);
    lv_obj_set_style_pad_all(card, 0, 0);
    return card;
}

static void chat_page_create(lv_obj_t *parent)
{
    lv_obj_set_style_pad_all(parent, 0, 0);
    lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                       LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);

    /* Slim header bar */
    lv_obj_t *header = lv_obj_create(parent);
    lv_obj_set_size(header, LV_PCT(100), 46);
    lv_obj_set_style_bg_color(header, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
    lv_obj_set_style_border_side(header, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(header, 1, 0);
    lv_obj_set_style_border_color(header, lv_color_hex(0xE3E8EF), 0);
    lv_obj_remove_flag(header, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t *header_title = lv_label_create(header);
    lv_label_set_text(header_title, LV_SYMBOL_ENVELOPE "  Mesh Chat");
    lv_obj_align(header_title, LV_ALIGN_LEFT_MID, 14, 0);
    lv_obj_set_style_text_font(header_title, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(header_title, lv_color_hex(0x1F2937), 0);
    lv_obj_t *header_hint = lv_label_create(header);
    lv_label_set_text(header_hint, "LoRa mesh");
    lv_obj_align(header_hint, LV_ALIGN_RIGHT_MID, -14, 0);
    lv_obj_set_style_text_font(header_hint, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(header_hint, lv_color_hex(0x9AA3AF), 0);

    s_chat_msg_area = lv_obj_create(parent);
    lv_obj_set_pos(s_chat_msg_area, 0, 46);
    lv_obj_set_size(s_chat_msg_area, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_chat_msg_area, lv_color_hex(0xEEF2F7), 0);
    lv_obj_set_style_bg_opa(s_chat_msg_area, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_chat_msg_area, 0, 0);
    lv_obj_set_style_pad_all(s_chat_msg_area, 12, 0);
    lv_obj_set_style_pad_bottom(s_chat_msg_area, 66, 0);
    lv_obj_set_style_pad_row(s_chat_msg_area, 10, 0);
    lv_obj_set_style_text_font(s_chat_msg_area, &lv_font_utf8_16x16, 0);
    lv_obj_set_flex_flow(s_chat_msg_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_chat_msg_area, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_scroll_dir(s_chat_msg_area, LV_DIR_VER);
    lv_obj_add_flag(s_chat_msg_area, LV_OBJ_FLAG_SCROLL_MOMENTUM);

    s_chat_input_bar = lv_obj_create(parent);
    lv_obj_set_width(s_chat_input_bar, LV_PCT(100));
    lv_obj_set_height(s_chat_input_bar, 58);
    lv_obj_align(s_chat_input_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(s_chat_input_bar, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(s_chat_input_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_side(s_chat_input_bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_width(s_chat_input_bar, 1, 0);
    lv_obj_set_style_border_color(s_chat_input_bar, lv_color_hex(0xE3E8EF), 0);
    lv_obj_set_style_pad_all(s_chat_input_bar, 8, 0);
    lv_obj_set_style_shadow_width(s_chat_input_bar, 6, 0);
    lv_obj_set_style_shadow_opa(s_chat_input_bar, LV_OPA_20, 0);
    lv_obj_set_style_shadow_color(s_chat_input_bar, lv_color_hex(0x334155), 0);
    lv_obj_set_style_shadow_offset_y(s_chat_input_bar, -2, 0);
    lv_obj_set_flex_flow(s_chat_input_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(s_chat_input_bar, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    s_chat_input = lv_textarea_create(s_chat_input_bar);
    lv_textarea_set_one_line(s_chat_input, true);
    lv_textarea_set_placeholder_text(s_chat_input, "Type a message...");
    lv_textarea_set_max_length(s_chat_input, 128);
    lv_obj_set_flex_grow(s_chat_input, 1);
    lv_obj_set_height(s_chat_input, 42);
    lv_obj_set_style_radius(s_chat_input, 21, 0);
    lv_obj_set_style_border_width(s_chat_input, 0, 0);
    lv_obj_set_style_bg_color(s_chat_input, lv_color_hex(0xF2F4F7), 0);
    lv_obj_set_style_bg_opa(s_chat_input, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(s_chat_input, 14, 0);
    lv_obj_set_style_text_color(s_chat_input, lv_color_hex(0x1F2937), 0);

    lv_obj_t *send = lv_btn_create(s_chat_input_bar);
    lv_obj_set_size(send, 42, 42);
    lv_obj_set_style_radius(send, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(send, lv_color_hex(0x1E88E5), 0);
    lv_obj_set_style_shadow_width(send, 4, 0);
    lv_obj_set_style_shadow_opa(send, LV_OPA_30, 0);
    lv_obj_set_style_shadow_color(send, lv_color_hex(0x1E88E5), 0);
    lv_obj_add_event_cb(send, chat_send_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *send_label = lv_label_create(send);
    lv_label_set_text(send_label, "Send");
    lv_obj_set_style_text_color(send_label, lv_color_white(), 0);
    lv_obj_center(send_label);

    lv_obj_t *keyboard = lv_keyboard_create(lv_screen_active());
    lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(keyboard, s_chat_input);
    lv_obj_add_event_cb(keyboard, chat_kb_hide_cb, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(keyboard, chat_kb_hide_cb, LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(s_chat_input, chat_kb_show_cb, LV_EVENT_FOCUSED, keyboard);
    lv_timer_create(chat_meshcore_poll_cb, 100, NULL);
}

static void chat_format_time(char *out, size_t size, uint32_t timestamp)
{
    time_t value = (time_t)timestamp;
    struct tm local_time;
    if (localtime_r(&value, &local_time) != NULL) {
        strftime(out, size, "%Y-%m-%d %H:%M", &local_time);
    } else {
        snprintf(out, size, "1970-01-01 00:00");
    }
}

static void chat_add_meshcore_message(const meshcore_chat_message_t *message)
{
    if (message == NULL || s_chat_msg_area == NULL) return;

    char stamp[24];
    chat_format_time(stamp, sizeof(stamp), message->timestamp);

    /* The core reports the completed local broadcast asynchronously. Reuse
     * the optimistic sending bubble instead of adding a second copy. */
    if (message->is_local && s_chat_pending_status != NULL &&
        strcmp(message->text, s_chat_pending_text) == 0) {
        lv_label_set_text_fmt(s_chat_pending_status, "%s | %s", stamp,
                              message->status == MESHCORE_CHAT_BROADCAST ?
                              "broadcast" : "send failed");
        s_chat_pending_status = NULL;
        s_chat_pending_text[0] = '\0';
        return;
    }

    lv_obj_t *row = lv_obj_create(s_chat_msg_area);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, message->is_local ? LV_FLEX_ALIGN_END : LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t *message_column = lv_obj_create(row);
    lv_obj_set_width(message_column, LV_SIZE_CONTENT);
    lv_obj_set_height(message_column, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(message_column, 520, 0);
    lv_obj_set_style_bg_opa(message_column, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(message_column, 0, 0);
    lv_obj_set_style_pad_all(message_column, 0, 0);
    lv_obj_set_style_pad_row(message_column, 3, 0);
    lv_obj_set_flex_flow(message_column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(message_column,
                          message->is_local ? LV_FLEX_ALIGN_END : LV_FLEX_ALIGN_START,
                          message->is_local ? LV_FLEX_ALIGN_END : LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    lv_obj_t *bubble = lv_obj_create(message_column);
    lv_obj_set_width(bubble, LV_SIZE_CONTENT);
    lv_obj_set_height(bubble, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(bubble, 480, 0);
    lv_obj_set_style_radius(bubble, 16, 0);
    lv_obj_set_style_pad_all(bubble, 10, 0);
    lv_obj_set_style_pad_left(bubble, 13, 0);
    lv_obj_set_style_pad_right(bubble, 13, 0);
    lv_obj_set_style_border_width(bubble, 0, 0);
    lv_obj_set_style_shadow_width(bubble, 5, 0);
    lv_obj_set_style_shadow_opa(bubble, LV_OPA_20, 0);
    lv_obj_set_style_shadow_offset_y(bubble, 2, 0);
    if (message->is_local) {
        lv_obj_set_style_bg_color(bubble, lv_color_hex(0x1E88E5), 0);
        lv_obj_set_style_shadow_color(bubble, lv_color_hex(0x1565C0), 0);
    } else {
        lv_obj_set_style_bg_color(bubble, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_border_width(bubble, 1, 0);
        lv_obj_set_style_border_color(bubble, lv_color_hex(0xE3E8EF), 0);
        lv_obj_set_style_shadow_color(bubble, lv_color_hex(0x334155), 0);
    }

    lv_obj_t *text = lv_label_create(bubble);
    lv_obj_set_width(text, LV_SIZE_CONTENT);
    lv_obj_set_style_max_width(text, 440, 0);
    lv_obj_set_style_text_font(text, &lv_font_utf8_16x16, 0);
    lv_obj_set_style_text_color(text, message->is_local ? lv_color_white()
                                                       : lv_color_hex(0x1F2937), 0);
    lv_label_set_long_mode(text, LV_LABEL_LONG_WRAP);
    lv_label_set_text(text, message->text);

    char detail[96];
    if (message->is_local) {
        snprintf(detail, sizeof(detail), "%s | %s", stamp,
                 message->status == MESHCORE_CHAT_BROADCAST ? "broadcast" : "sending");
    } else if (message->metrics_valid) {
        snprintf(detail, sizeof(detail), "%s | RSSI %d dBm | SNR %.2f dB",
                 stamp, (int)message->rssi_dbm,
                 (double)message->snr_quarter_db / 4.0);
        if (message->router_count > 0) {
            size_t used = strlen(detail);
            snprintf(detail + used, sizeof(detail) - used, " | from %u router%s",
                     (unsigned)message->router_count,
                     message->router_count == 1 ? "" : "s");
        }
    } else {
        snprintf(detail, sizeof(detail), "%s | received", stamp);
    }
    lv_obj_t *time_label = lv_label_create(message_column);
    lv_label_set_text(time_label, detail);
    lv_obj_set_style_text_font(time_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(time_label, lv_color_hex(0x8A94A6), 0);
    lv_obj_set_style_text_opa(time_label, LV_OPA_90, 0);
    if (message->is_local && message->status == MESHCORE_CHAT_SENDING) {
        s_chat_pending_status = time_label;
        strlcpy(s_chat_pending_text, message->text, sizeof(s_chat_pending_text));
    }
    lv_obj_scroll_to_y(s_chat_msg_area, lv_obj_get_scroll_bottom(s_chat_msg_area), LV_ANIM_OFF);
}

static void chat_send_event_cb(lv_event_t *event)
{
    (void)event;
    if (s_chat_input == NULL) return;
    const char *text = lv_textarea_get_text(s_chat_input);
    if (text == NULL || text[0] == '\0') return;
    if (meshcore_core_send_public(text) == ESP_OK) {
        meshcore_chat_message_t pending = {};
        pending.is_local = true;
        pending.status = MESHCORE_CHAT_SENDING;
        pending.timestamp = (uint32_t)time(NULL);
        strlcpy(pending.text, text, sizeof(pending.text));
        chat_add_meshcore_message(&pending);
        lv_textarea_set_text(s_chat_input, "");
    }
}

static void chat_meshcore_poll_cb(lv_timer_t *timer)
{
    (void)timer;
    meshcore_chat_message_t message;
    while (meshcore_core_pop_message(&message)) {
        chat_add_meshcore_message(&message);
    }
}

static void chat_kb_show_cb(lv_event_t *event)
{
    lv_obj_t *keyboard = lv_event_get_user_data(event);
    if (keyboard == NULL) return;
    lv_obj_remove_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(keyboard, lv_event_get_target(event));
    if (s_chat_input_bar != NULL) {
        lv_obj_align(s_chat_input_bar, LV_ALIGN_BOTTOM_MID, 0,
                     -lv_obj_get_height(keyboard));
    }
}

static void chat_kb_hide_cb(lv_event_t *event)
{
    lv_obj_t *keyboard = lv_event_get_target(event);
    if (keyboard == NULL) return;
    lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(keyboard, NULL);
    if (s_chat_input_bar != NULL) lv_obj_align(s_chat_input_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
}

static void settings_row_style(lv_obj_t *row)
{
    lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                       LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);
    /* Let vertical drags that start on a label or row bubble to the settings list. */
    lv_obj_add_flag(row, LV_OBJ_FLAG_SCROLL_CHAIN_VER | LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_set_width(row, LV_PCT(100));
    lv_obj_set_height(row, 52);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_opa(row, LV_OPA_20, 0);
    lv_obj_set_style_border_color(row, lv_color_hex(0x9E9E9E), 0);
    lv_obj_set_style_pad_hor(row, 14, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
}

#define SETTINGS_NAMESPACE "app_settings"
#define SETTINGS_DEVICE_NAME "device_name"
#define SETTINGS_BRIGHTNESS "brightness"
#define SETTINGS_TIMEOUT "screen_timeout"

static void settings_load(void)
{
    nvs_handle_t handle;
    if (nvs_open(SETTINGS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return;
    size_t name_length = sizeof(s_device_name);
    if (nvs_get_str(handle, SETTINGS_DEVICE_NAME, s_device_name, &name_length) != ESP_OK ||
        s_device_name[0] == '\0') {
        strlcpy(s_device_name, "TinyTab", sizeof(s_device_name));
    }
    uint32_t brightness = LCD_Backlight;
    if (nvs_get_u32(handle, SETTINGS_BRIGHTNESS, &brightness) == ESP_OK &&
        brightness >= 5 && brightness <= 100) {
        Set_Backlight((uint8_t)brightness);
    }
    s_awake_brightness = LCD_Backlight;
    uint32_t timeout = 60;
    if (nvs_get_u32(handle, SETTINGS_TIMEOUT, &timeout) == ESP_OK &&
        (timeout == 0 || timeout == 30 || timeout == 60 ||
         timeout == 120 || timeout == 300)) {
        s_screen_timeout_seconds = timeout;
    }
    nvs_close(handle);
}

static esp_err_t settings_save_string(const char *key, const char *value)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_str(handle, key, value);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static esp_err_t settings_save_u32(const char *key, uint32_t value)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(SETTINGS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u32(handle, key, value);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static lv_obj_t *settings_detail_page_create(lv_obj_t *parent, const char *title)
{
    lv_obj_t *page = lv_obj_create(parent);
    lv_obj_set_size(page, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(page, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_bg_opa(page, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(page, 0, 0);
    lv_obj_set_style_pad_all(page, 0, 0);
    lv_obj_remove_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *back = lv_btn_create(page);
    lv_obj_set_pos(back, 16, 9);
    lv_obj_set_size(back, 42, 42);
    lv_obj_set_style_radius(back, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0xE5EAF0), 0);
    lv_obj_set_style_text_color(back, lv_color_hex(0x344054), 0);
    lv_obj_add_event_cb(back, settings_back_cb, LV_EVENT_CLICKED, page);
    lv_obj_t *icon = lv_label_create(back);
    lv_label_set_text(icon, LV_SYMBOL_LEFT);
    lv_obj_center(icon);

    lv_obj_t *heading = lv_label_create(page);
    lv_label_set_text(heading, title);
    lv_obj_set_pos(heading, 72, 12);
    lv_obj_set_style_text_font(heading, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(heading, lv_color_hex(0x172033), 0);
    return page;
}

static lv_obj_t *settings_card(lv_obj_t *parent, int x, int y, int w, int h)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_size(card, w, h);
    lv_obj_remove_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(card, lv_color_white(), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_border_color(card, lv_color_hex(0xDDE3EA), 0);
    lv_obj_set_style_radius(card, 7, 0);
    return card;
}

static void settings_show_page(lv_obj_t *page)
{
    if (s_settings_home == NULL || page == NULL) return;
    lv_obj_add_flag(s_settings_home, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(page, LV_OBJ_FLAG_HIDDEN);
}

static void settings_back_cb(lv_event_t *event)
{
    lv_obj_t *page = lv_event_get_user_data(event);
    if (page != NULL) lv_obj_add_flag(page, LV_OBJ_FLAG_HIDDEN);
    if (s_settings_home != NULL) lv_obj_remove_flag(s_settings_home, LV_OBJ_FLAG_HIDDEN);
    if (s_device_name_keyboard != NULL) lv_obj_add_flag(s_device_name_keyboard, LV_OBJ_FLAG_HIDDEN);
    if (s_lora_keyboard != NULL) lv_obj_add_flag(s_lora_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void settings_keyboard_show_cb(lv_event_t *event)
{
    lv_obj_t *keyboard = lv_event_get_user_data(event);
    if (keyboard == NULL) return;
    lv_keyboard_set_textarea(keyboard, lv_event_get_target(event));
    lv_obj_remove_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void settings_keyboard_hide_cb(lv_event_t *event)
{
    lv_obj_t *keyboard = lv_event_get_target(event);
    lv_keyboard_set_textarea(keyboard, NULL);
    lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
}

static void device_name_save_cb(lv_event_t *event)
{
    (void)event;
    const char *name = lv_textarea_get_text(s_device_name_input);
    if (name == NULL || name[0] == '\0') return;
    strlcpy(s_device_name, name, sizeof(s_device_name));
    if (settings_save_string(SETTINGS_DEVICE_NAME, s_device_name) == ESP_OK) {
        lv_label_set_text(s_device_name_value, s_device_name);
        if (s_map_device_name_lbl != NULL) {
            lv_label_set_text(s_map_device_name_lbl, s_device_name);
        }
    }
}

static void device_name_entry_cb(lv_event_t *event)
{
    (void)event;
    lv_textarea_set_text(s_device_name_input, s_device_name);
    settings_show_page(s_device_name_page);
}

static lv_obj_t *lora_text_field(lv_obj_t *parent, int x, int y,
                                 const char *label_text, lv_obj_t *keyboard)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, label_text);
    lv_obj_set_pos(label, x, y);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
    lv_obj_t *field = lv_textarea_create(parent);
    lv_obj_set_pos(field, x, y + 22);
    lv_obj_set_size(field, 238, 42);
    lv_textarea_set_one_line(field, true);
    lv_textarea_set_accepted_chars(field, "0123456789.-");
    lv_obj_add_event_cb(field, settings_keyboard_show_cb, LV_EVENT_FOCUSED, keyboard);
    return field;
}

static void lora_settings_entry_cb(lv_event_t *event)
{
    (void)event;
    meshcore_lora_config_t cfg = {0};
    if (meshcore_core_get_lora_config(&cfg) == ESP_OK) {
        char value[24];
        snprintf(value, sizeof(value), "%.3f", (double)cfg.frequency_mhz);
        lv_textarea_set_text(s_lora_frequency, value);
        snprintf(value, sizeof(value), "%.1f", (double)cfg.bandwidth_khz);
        lv_textarea_set_text(s_lora_bandwidth, value);
        snprintf(value, sizeof(value), "%d", cfg.tx_power_dbm);
        lv_textarea_set_text(s_lora_power, value);
        lv_dropdown_set_selected(s_lora_sf, cfg.spreading_factor - 5);
        lv_dropdown_set_selected(s_lora_cr, cfg.coding_rate - 5);
        if (cfg.rx_boosted_gain) lv_obj_add_state(s_lora_rx_gain, LV_STATE_CHECKED);
        else lv_obj_clear_state(s_lora_rx_gain, LV_STATE_CHECKED);
        lv_label_set_text(s_lora_status, "Radio settings loaded");
    } else {
        lv_label_set_text(s_lora_status, "Radio is starting");
    }
    settings_show_page(s_lora_page);
}

static void lora_save_cb(lv_event_t *event)
{
    (void)event;
    meshcore_lora_config_t cfg = {0};
    if (meshcore_core_get_lora_config(&cfg) != ESP_OK) {
        lv_label_set_text(s_lora_status, "Radio is not ready");
        return;
    }
    cfg.frequency_mhz = strtof(lv_textarea_get_text(s_lora_frequency), NULL);
    cfg.bandwidth_khz = strtof(lv_textarea_get_text(s_lora_bandwidth), NULL);
    cfg.tx_power_dbm = (int8_t)strtol(lv_textarea_get_text(s_lora_power), NULL, 10);
    cfg.spreading_factor = lv_dropdown_get_selected(s_lora_sf) + 5;
    cfg.coding_rate = lv_dropdown_get_selected(s_lora_cr) + 5;
    cfg.rx_boosted_gain = lv_obj_has_state(s_lora_rx_gain, LV_STATE_CHECKED);
    esp_err_t err = meshcore_core_set_lora_config(&cfg);
    lv_label_set_text_fmt(s_lora_status,
                          err == ESP_OK ? "Radio update queued" : "Save failed: %s",
                          esp_err_to_name(err));
}

static void lora_advert_cb(lv_event_t *event)
{
    bool flood = (bool)(uintptr_t)lv_event_get_user_data(event);
    esp_err_t err = meshcore_core_send_advert(flood);
    lv_label_set_text_fmt(s_lora_status,
                          err == ESP_OK ? "Advert queued" : "Advert failed: %s",
                          esp_err_to_name(err));
}

static void brightness_slider_cb(lv_event_t *event)
{
    uint8_t value = (uint8_t)lv_slider_get_value(lv_event_get_target(event));
    s_awake_brightness = value;
    Set_Backlight(value);
    lv_label_set_text_fmt(s_brightness_value, "%u%%", value);
    lv_label_set_text_fmt(s_brightness_detail_value, "%u%%", value);
    (void)settings_save_u32(SETTINGS_BRIGHTNESS, value);
}

static void brightness_entry_cb(lv_event_t *event)
{
    (void)event;
    settings_show_page(s_brightness_page);
}

static const char *timeout_text(uint32_t seconds)
{
    switch (seconds) {
    case 30: return "30 seconds";
    case 60: return "60 seconds";
    case 120: return "2 minutes";
    case 300: return "5 minutes";
    default: return "Never";
    }
}

static void timeout_changed_cb(lv_event_t *event)
{
    static const uint32_t values[] = {30, 60, 120, 300, 0};
    uint32_t selected = lv_dropdown_get_selected(lv_event_get_target(event));
    if (selected >= sizeof(values) / sizeof(values[0])) selected = 1;
    s_screen_timeout_seconds = values[selected];
    lv_label_set_text(s_timeout_value, timeout_text(s_screen_timeout_seconds));
    (void)settings_save_u32(SETTINGS_TIMEOUT, s_screen_timeout_seconds);
    lv_display_trigger_activity(NULL);
}

static void timeout_entry_cb(lv_event_t *event)
{
    (void)event;
    settings_show_page(s_timeout_page);
}

static void settings_status_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (s_storage_value != NULL) {
        uint64_t total = 0, used = 0;
        if (SD_Get_Usage(&total, &used)) {
            lv_label_set_text_fmt(s_storage_value, "%.1f / %.1f GB",
                                  (double)used / (1024.0 * 1024.0 * 1024.0),
                                  (double)total / (1024.0 * 1024.0 * 1024.0));
        } else {
            lv_label_set_text(s_storage_value, "Not mounted");
        }
    }

    uint32_t inactive = lv_display_get_inactive_time(NULL);
    if (s_screen_timeout_seconds != 0 &&
        inactive >= s_screen_timeout_seconds * 1000U && !s_screen_sleeping) {
        s_screen_sleeping = true;
        s_awake_brightness = LCD_Backlight;
        Set_Backlight(0);
    } else if (s_screen_sleeping && inactive < 1000U) {
        s_screen_sleeping = false;
        Set_Backlight(s_awake_brightness);
    }
}

static void settings_extra_pages_create(lv_obj_t *parent)
{
    s_device_name_page = settings_detail_page_create(parent, "Device Name");
    lv_obj_t *name_card = settings_card(s_device_name_page, 18, 76, 834, 170);
    lv_obj_t *name_label = lv_label_create(name_card);
    lv_label_set_text(name_label, "Name");
    lv_obj_set_pos(name_label, 14, 12);
    s_device_name_input = lv_textarea_create(name_card);
    lv_obj_set_pos(s_device_name_input, 14, 42);
    lv_obj_set_size(s_device_name_input, 610, 48);
    lv_textarea_set_one_line(s_device_name_input, true);
    lv_textarea_set_max_length(s_device_name_input, 32);
    lv_textarea_set_text(s_device_name_input, s_device_name);
    lv_obj_t *name_save = lv_btn_create(name_card);
    lv_obj_set_pos(name_save, 642, 42);
    lv_obj_set_size(name_save, 160, 48);
    lv_obj_add_event_cb(name_save, device_name_save_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *name_save_label = lv_label_create(name_save);
    lv_label_set_text(name_save_label, LV_SYMBOL_SAVE "  Save");
    lv_obj_center(name_save_label);
    s_device_name_keyboard = lv_keyboard_create(lv_screen_active());
    lv_obj_add_flag(s_device_name_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_device_name_keyboard, settings_keyboard_hide_cb,
                        LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(s_device_name_keyboard, settings_keyboard_hide_cb,
                        LV_EVENT_CANCEL, NULL);
    lv_obj_add_event_cb(s_device_name_input, settings_keyboard_show_cb,
                        LV_EVENT_FOCUSED, s_device_name_keyboard);

    s_lora_page = settings_detail_page_create(parent, "LoRa");
    lv_obj_t *lora_card = settings_card(s_lora_page, 18, 64, 834, 474);
    s_lora_keyboard = lv_keyboard_create(lv_screen_active());
    lv_obj_add_flag(s_lora_keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_lora_keyboard, settings_keyboard_hide_cb, LV_EVENT_READY, NULL);
    lv_obj_add_event_cb(s_lora_keyboard, settings_keyboard_hide_cb, LV_EVENT_CANCEL, NULL);
    s_lora_frequency = lora_text_field(lora_card, 14, 12, "Frequency (MHz)", s_lora_keyboard);
    s_lora_bandwidth = lora_text_field(lora_card, 286, 12, "Bandwidth (kHz)", s_lora_keyboard);
    s_lora_power = lora_text_field(lora_card, 558, 12, "TX power (dBm)", s_lora_keyboard);
    lv_obj_t *sf_label = lv_label_create(lora_card);
    lv_label_set_text(sf_label, "Spreading factor");
    lv_obj_set_pos(sf_label, 14, 100);
    s_lora_sf = lv_dropdown_create(lora_card);
    lv_dropdown_set_options(s_lora_sf, "SF5\nSF6\nSF7\nSF8\nSF9\nSF10\nSF11\nSF12");
    lv_obj_set_pos(s_lora_sf, 14, 124);
    lv_obj_set_size(s_lora_sf, 238, 42);
    lv_obj_t *cr_label = lv_label_create(lora_card);
    lv_label_set_text(cr_label, "Coding rate");
    lv_obj_set_pos(cr_label, 286, 100);
    s_lora_cr = lv_dropdown_create(lora_card);
    lv_dropdown_set_options(s_lora_cr, "CR 4/5\nCR 4/6\nCR 4/7\nCR 4/8");
    lv_obj_set_pos(s_lora_cr, 286, 124);
    lv_obj_set_size(s_lora_cr, 238, 42);
    lv_obj_t *rx_label = lv_label_create(lora_card);
    lv_label_set_text(rx_label, "RX boosted gain");
    lv_obj_set_pos(rx_label, 558, 106);
    s_lora_rx_gain = lv_switch_create(lora_card);
    lv_obj_set_pos(s_lora_rx_gain, 746, 100);
    lv_obj_t *save_lora = lv_btn_create(lora_card);
    lv_obj_set_pos(save_lora, 14, 230);
    lv_obj_set_size(save_lora, 238, 46);
    lv_obj_add_event_cb(save_lora, lora_save_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *save_lora_label = lv_label_create(save_lora);
    lv_label_set_text(save_lora_label, LV_SYMBOL_SAVE "  Save settings");
    lv_obj_center(save_lora_label);
    lv_obj_t *zero_advert = lv_btn_create(lora_card);
    lv_obj_set_pos(zero_advert, 286, 230);
    lv_obj_set_size(zero_advert, 238, 46);
    lv_obj_add_event_cb(zero_advert, lora_advert_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *zero_label = lv_label_create(zero_advert);
    lv_label_set_text(zero_label, "Zero-hop advert");
    lv_obj_center(zero_label);
    lv_obj_t *flood_advert = lv_btn_create(lora_card);
    lv_obj_set_pos(flood_advert, 558, 230);
    lv_obj_set_size(flood_advert, 238, 46);
    lv_obj_add_event_cb(flood_advert, lora_advert_cb, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)1);
    lv_obj_t *flood_label = lv_label_create(flood_advert);
    lv_label_set_text(flood_label, "Flood advert");
    lv_obj_center(flood_label);
    s_lora_status = lv_label_create(lora_card);
    lv_label_set_text(s_lora_status, "Radio is starting");
    lv_obj_set_pos(s_lora_status, 14, 304);

    s_brightness_page = settings_detail_page_create(parent, "Brightness");
    lv_obj_t *brightness_card = settings_card(s_brightness_page, 18, 76, 834, 170);
    s_brightness_detail_value = lv_label_create(brightness_card);
    lv_label_set_text_fmt(s_brightness_detail_value, "%u%%", LCD_Backlight);
    lv_obj_set_pos(s_brightness_detail_value, 14, 14);
    lv_obj_set_style_text_font(s_brightness_detail_value, &lv_font_montserrat_24, 0);
    lv_obj_t *brightness_slider = lv_slider_create(brightness_card);
    lv_obj_set_pos(brightness_slider, 14, 76);
    lv_obj_set_size(brightness_slider, 790, 28);
    lv_slider_set_range(brightness_slider, 5, 100);
    lv_slider_set_value(brightness_slider, LCD_Backlight, LV_ANIM_OFF);
    lv_obj_add_event_cb(brightness_slider, brightness_slider_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);

    s_timeout_page = settings_detail_page_create(parent, "Screen Timeout");
    lv_obj_t *timeout_card = settings_card(s_timeout_page, 18, 76, 834, 170);
    lv_obj_t *timeout_label = lv_label_create(timeout_card);
    lv_label_set_text(timeout_label, "Turn display off after");
    lv_obj_set_pos(timeout_label, 14, 16);
    s_timeout_dropdown = lv_dropdown_create(timeout_card);
    lv_dropdown_set_options(s_timeout_dropdown,
                            "30 seconds\n60 seconds\n2 minutes\n5 minutes\nNever");
    lv_obj_set_pos(s_timeout_dropdown, 14, 54);
    lv_obj_set_size(s_timeout_dropdown, 300, 46);
    uint32_t timeout_index = s_screen_timeout_seconds == 30 ? 0 :
                             s_screen_timeout_seconds == 60 ? 1 :
                             s_screen_timeout_seconds == 120 ? 2 :
                             s_screen_timeout_seconds == 300 ? 3 : 4;
    lv_dropdown_set_selected(s_timeout_dropdown, timeout_index);
    lv_obj_add_event_cb(s_timeout_dropdown, timeout_changed_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
}

static lv_obj_t *settings_value_row(lv_obj_t *list, const char *name,
                                     const char *value, bool arrow)
{
    lv_obj_t *row = lv_obj_create(list);
    settings_row_style(row);
    lv_obj_t *label = lv_label_create(row);
    lv_label_set_text(label, name);
    lv_obj_set_style_text_color(label, lv_color_hex(0x172033), 0);
    lv_obj_set_flex_grow(label, 1);
    lv_obj_t *value_label = lv_label_create(row);
    lv_label_set_text(value_label, value);
    lv_obj_set_style_text_color(value_label, lv_color_hex(0x6B7280), 0);
    lv_obj_set_style_text_font(value_label, &lv_font_montserrat_12, 0);
    if (arrow) {
        lv_obj_t *next = lv_label_create(row);
        lv_label_set_text(next, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(next, lv_color_hex(0x98A2B3), 0);
    }
    if (strcmp(name, "GPS") == 0) {
        lv_obj_add_event_cb(row, gps_settings_entry_cb, LV_EVENT_CLICKED, NULL);
    } else if (strcmp(name, "Device Name") == 0) {
        lv_obj_add_event_cb(row, device_name_entry_cb, LV_EVENT_CLICKED, NULL);
    } else if (strcmp(name, "LoRa") == 0) {
        lv_obj_add_event_cb(row, lora_settings_entry_cb, LV_EVENT_CLICKED, NULL);
    } else if (strcmp(name, "Brightness") == 0) {
        lv_obj_add_event_cb(row, brightness_entry_cb, LV_EVENT_CLICKED, NULL);
    } else if (strcmp(name, "Screen Timeout") == 0) {
        lv_obj_add_event_cb(row, timeout_entry_cb, LV_EVENT_CLICKED, NULL);
    }
    return value_label;
}

static void settings_page_create(lv_obj_t *parent)
{
    lv_obj_set_style_pad_all(parent, 0, 0);
    lv_obj_set_style_bg_color(parent, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
    lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE);

    s_settings_home = lv_obj_create(parent);
    lv_obj_set_size(s_settings_home, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_opa(s_settings_home, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(s_settings_home, 0, 0);
    lv_obj_set_style_pad_all(s_settings_home, 0, 0);
    /* Keep the page fixed and let only the list own vertical scrolling. */
    lv_obj_remove_flag(s_settings_home, LV_OBJ_FLAG_SCROLLABLE |
                       LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_ELASTIC |
                       LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(s_settings_home, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(s_settings_home, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *heading = lv_label_create(s_settings_home);
    lv_label_set_text(heading, "Settings");
    lv_obj_set_pos(heading, 20, 14);
    lv_obj_set_style_text_font(heading, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(heading, lv_color_hex(0x172033), 0);
    lv_obj_t *hint = lv_label_create(s_settings_home);
    lv_label_set_text(hint, "Application preferences");
    lv_obj_set_pos(hint, 22, 45);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(hint, lv_color_hex(0x697386), 0);

    lv_obj_t *list = lv_obj_create(s_settings_home);
    lv_obj_set_pos(list, 18, 76);
    lv_obj_set_size(list, 834, 462);
    lv_obj_set_style_bg_color(list, lv_color_white(), 0);
    lv_obj_set_style_border_width(list, 1, 0);
    lv_obj_set_style_border_color(list, lv_color_hex(0xDDE3EA), 0);
    lv_obj_set_style_radius(list, 7, 0);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_add_flag(list, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_remove_flag(list, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);

    s_device_name_value = settings_value_row(list, "Device Name", s_device_name, true);
    settings_value_row(list, "LoRa", "Radio", true);
    char brightness_text[12];
    snprintf(brightness_text, sizeof(brightness_text), "%u%%", LCD_Backlight);
    s_brightness_value = settings_value_row(list, "Brightness", brightness_text, true);
    s_timeout_value = settings_value_row(list, "Screen Timeout",
                                         timeout_text(s_screen_timeout_seconds), true);
    const esp_app_desc_t *app = esp_app_get_description();
    settings_value_row(list, "Firmware", app != NULL ? app->version : "1", false);
    s_settings_gps_value = settings_value_row(list, "GPS", "Waiting", true);
    s_storage_value = settings_value_row(list, "SD Card Storage", "Checking", false);

    lv_obj_t *developer = lv_obj_create(list);
    settings_row_style(developer);
    lv_obj_add_event_cb(developer, developer_settings_entry_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *developer_label = lv_label_create(developer);
    lv_label_set_text(developer_label, "Developer Options");
    lv_obj_set_style_text_color(developer_label, lv_color_hex(0x172033), 0);
    lv_obj_set_flex_grow(developer_label, 1);
    lv_obj_t *developer_value = lv_label_create(developer);
    lv_label_set_text(developer_value, "Open");
    lv_obj_set_style_text_color(developer_value, lv_color_hex(0x6B7280), 0);
    lv_obj_t *developer_next = lv_label_create(developer);
    lv_label_set_text(developer_next, LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_color(developer_next, lv_color_hex(0x98A2B3), 0);

    settings_extra_pages_create(parent);

    s_developer_settings_page = lv_obj_create(parent);
    lv_obj_set_size(s_developer_settings_page, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_developer_settings_page, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_bg_opa(s_developer_settings_page, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_developer_settings_page, 0, 0);
    lv_obj_set_style_pad_all(s_developer_settings_page, 0, 0);
    lv_obj_remove_flag(s_developer_settings_page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_developer_settings_page, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *back = lv_btn_create(s_developer_settings_page);
    lv_obj_set_pos(back, 16, 9);
    lv_obj_set_size(back, 42, 42);
    lv_obj_set_style_radius(back, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(back, lv_color_hex(0xE5EAF0), 0);
    lv_obj_set_style_text_color(back, lv_color_hex(0x344054), 0);
    lv_obj_add_event_cb(back, developer_settings_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *back_icon = lv_label_create(back);
    lv_label_set_text(back_icon, LV_SYMBOL_LEFT);
    lv_obj_center(back_icon);

    lv_obj_t *developer_title = lv_label_create(s_developer_settings_page);
    lv_label_set_text(developer_title, "Developer Options");
    lv_obj_set_pos(developer_title, 72, 10);
    lv_obj_set_style_text_font(developer_title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(developer_title, lv_color_hex(0x172033), 0);
    lv_obj_t *section = lv_label_create(s_developer_settings_page);
    lv_label_set_text(section, "DISPLAY DIAGNOSTICS");
    lv_obj_set_pos(section, 20, 65);
    lv_obj_set_style_text_font(section, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(section, lv_color_hex(0x778292), 0);

    lv_obj_t *fps_row = ui_card_create(s_developer_settings_page, 18, 88, 834, 68);
    lv_obj_t *fps_label = lv_label_create(fps_row);
    lv_label_set_text(fps_label, "FPS Overlay");
    lv_obj_align(fps_label, LV_ALIGN_LEFT_MID, 2, 0);
    lv_obj_set_style_text_font(fps_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(fps_label, lv_color_hex(0x172033), 0);
    s_fps_overlay_switch = lv_switch_create(fps_row);
    lv_obj_set_size(s_fps_overlay_switch, 56, 32);
    lv_obj_align(s_fps_overlay_switch, LV_ALIGN_RIGHT_MID, -2, 0);
    lv_obj_add_event_cb(s_fps_overlay_switch, fps_overlay_switch_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *fps_detail = lv_label_create(s_developer_settings_page);
    lv_label_set_text(fps_detail, "Show LVGL FPS, CPU, render and flush timing");
    lv_obj_set_pos(fps_detail, 20, 174);
    lv_obj_set_style_text_font(fps_detail, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(fps_detail, lv_color_hex(0x697386), 0);

    lv_obj_t *mesh_section = lv_label_create(s_developer_settings_page);
    lv_label_set_text(mesh_section, "MESH NETWORK");
    lv_obj_set_pos(mesh_section, 20, 220);
    lv_obj_set_style_text_font(mesh_section, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(mesh_section, lv_color_hex(0x778292), 0);

    lv_obj_t *repeat_row = ui_card_create(s_developer_settings_page, 18, 242, 834, 68);
    lv_obj_t *repeat_label = lv_label_create(repeat_row);
    lv_label_set_text(repeat_label, "Repeat Forwarding");
    lv_obj_align(repeat_label, LV_ALIGN_LEFT_MID, 2, 0);
    lv_obj_set_style_text_font(repeat_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(repeat_label, lv_color_hex(0x172033), 0);
    s_repeat_forwarding_switch = lv_switch_create(repeat_row);
    lv_obj_set_size(s_repeat_forwarding_switch, 56, 32);
    lv_obj_align(s_repeat_forwarding_switch, LV_ALIGN_RIGHT_MID, -2, 0);
    lv_obj_add_event_cb(s_repeat_forwarding_switch, repeat_forwarding_switch_cb,
                        LV_EVENT_VALUE_CHANGED, NULL);
    s_repeat_forwarding_status = lv_label_create(s_developer_settings_page);
    lv_label_set_text(s_repeat_forwarding_status,
                      "Allow this device to relay MeshCore packets for other nodes");
    lv_obj_set_pos(s_repeat_forwarding_status, 20, 326);
    lv_obj_set_style_text_font(s_repeat_forwarding_status, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_repeat_forwarding_status, lv_color_hex(0x697386), 0);

    s_gps_settings_page = lv_obj_create(parent);
    lv_obj_set_size(s_gps_settings_page, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_gps_settings_page, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_bg_opa(s_gps_settings_page, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_gps_settings_page, 0, 0);
    lv_obj_set_style_pad_all(s_gps_settings_page, 0, 0);
    lv_obj_remove_flag(s_gps_settings_page, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_gps_settings_page, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *gps_back = lv_btn_create(s_gps_settings_page);
    lv_obj_set_pos(gps_back, 16, 9);
    lv_obj_set_size(gps_back, 42, 42);
    lv_obj_set_style_radius(gps_back, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(gps_back, lv_color_hex(0xE5EAF0), 0);
    lv_obj_add_event_cb(gps_back, gps_settings_back_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *gps_back_icon = lv_label_create(gps_back);
    lv_label_set_text(gps_back_icon, LV_SYMBOL_LEFT);
    lv_obj_center(gps_back_icon);

    lv_obj_t *gps_title = lv_label_create(s_gps_settings_page);
    lv_label_set_text(gps_title, "GPS Status");
    lv_obj_set_pos(gps_title, 72, 10);
    lv_obj_set_style_text_font(gps_title, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(gps_title, lv_color_hex(0x172033), 0);

    lv_obj_t *gps_card = ui_card_create(s_gps_settings_page, 18, 72, 834, 132);
    s_gps_detail_state = lv_label_create(gps_card);
    lv_label_set_text(s_gps_detail_state, "Checking GPS...");
    lv_obj_set_pos(s_gps_detail_state, 16, 14);
    lv_obj_set_style_text_font(s_gps_detail_state, &lv_font_montserrat_20, 0);
    s_gps_detail_stream = lv_label_create(gps_card);
    lv_label_set_text(s_gps_detail_stream, "UART1  /  RX IO2  /  TX IO3  /  9600 baud");
    lv_obj_set_pos(s_gps_detail_stream, 16, 58);
    lv_obj_set_style_text_font(s_gps_detail_stream, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_gps_detail_stream, lv_color_hex(0x667085), 0);
    s_gps_detail_satellites = lv_label_create(gps_card);
    lv_label_set_text(s_gps_detail_satellites, "-- satellites");
    lv_obj_set_pos(s_gps_detail_satellites, 16, 88);
    lv_obj_set_style_text_font(s_gps_detail_satellites, &lv_font_montserrat_14, 0);
    s_gps_detail_coordinates = lv_label_create(s_gps_settings_page);
    lv_label_set_text(s_gps_detail_coordinates, "Waiting for a valid fix");
    lv_obj_set_pos(s_gps_detail_coordinates, 20, 236);
    lv_obj_set_style_text_font(s_gps_detail_coordinates, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(s_gps_detail_coordinates, lv_color_hex(0x172033), 0);

    lv_timer_create(settings_status_timer_cb, 250, NULL);
    settings_status_timer_cb(NULL);
}

static void developer_settings_entry_cb(lv_event_t *event)
{
    (void)event;
    if (s_settings_home == NULL || s_developer_settings_page == NULL) return;
    meshcore_lora_config_t cfg = {0};
    if (meshcore_core_get_lora_config(&cfg) == ESP_OK) {
        if (cfg.client_repeat) {
            lv_obj_add_state(s_repeat_forwarding_switch, LV_STATE_CHECKED);
            lv_label_set_text(s_repeat_forwarding_status,
                              "Enabled: this device relays packets for other nodes");
        } else {
            lv_obj_clear_state(s_repeat_forwarding_switch, LV_STATE_CHECKED);
            lv_label_set_text(s_repeat_forwarding_status,
                              "Disabled: this device does not relay other nodes' packets");
        }
    }
    lv_obj_add_flag(s_settings_home, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_developer_settings_page, LV_OBJ_FLAG_HIDDEN);
}

static void developer_settings_back_cb(lv_event_t *event)
{
    (void)event;
    if (s_settings_home == NULL || s_developer_settings_page == NULL) return;
    lv_obj_add_flag(s_developer_settings_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_settings_home, LV_OBJ_FLAG_HIDDEN);
}

static void gps_settings_entry_cb(lv_event_t *event)
{
    (void)event;
    if (s_settings_home == NULL || s_gps_settings_page == NULL) return;
    lv_obj_add_flag(s_settings_home, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_gps_settings_page, LV_OBJ_FLAG_HIDDEN);
    gps_ui_timer_cb(NULL);
}

static void gps_settings_back_cb(lv_event_t *event)
{
    (void)event;
    if (s_settings_home == NULL || s_gps_settings_page == NULL) return;
    lv_obj_add_flag(s_gps_settings_page, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(s_settings_home, LV_OBJ_FLAG_HIDDEN);
}

static void fps_overlay_switch_cb(lv_event_t *event)
{
    lv_obj_t *sw = lv_event_get_target(event);
    if (lv_obj_has_state(sw, LV_STATE_CHECKED)) {
        lv_sysmon_show_performance(NULL);
    } else {
        lv_sysmon_hide_performance(NULL);
    }
}

static void repeat_forwarding_switch_cb(lv_event_t *event)
{
    lv_obj_t *sw = lv_event_get_target(event);
    bool enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);
    meshcore_lora_config_t cfg = {0};
    esp_err_t err = meshcore_core_get_lora_config(&cfg);
    if (err == ESP_OK) {
        cfg.client_repeat = enabled ? 1 : 0;
        err = meshcore_core_set_lora_config(&cfg);
    }
    if (err != ESP_OK) {
        if (enabled) lv_obj_clear_state(sw, LV_STATE_CHECKED);
        else lv_obj_add_state(sw, LV_STATE_CHECKED);
        lv_label_set_text_fmt(s_repeat_forwarding_status, "Update failed: %s",
                              esp_err_to_name(err));
        return;
    }
    lv_label_set_text(s_repeat_forwarding_status,
                      enabled ? "Enabled: this device relays packets for other nodes"
                              : "Disabled: this device does not relay other nodes' packets");
}

static const char *device_type_name(uint8_t type)
{
    switch (type) {
    case 1: return "Companion";
    case 2: return "Repeater";
    case 3: return "Room server";
    case 4: return "Sensor";
    default: return "Mesh node";
    }
}

static uint32_t device_name_color(const char *name)
{
    static const uint32_t palette[] = {
        0x2563EB, 0x0F766E, 0x7C3AED, 0xC2410C, 0x047857, 0xBE123C
    };
    uint32_t hash = 2166136261u;
    while (name != NULL && *name != '\0') hash = (hash ^ (uint8_t)*name++) * 16777619u;
    return palette[hash % (sizeof(palette) / sizeof(palette[0]))];
}

static void device_snr_text(char *out, size_t size, int16_t quarter_db)
{
    const bool negative = quarter_db < 0;
    const uint16_t magnitude = (uint16_t)(negative ? -quarter_db : quarter_db);
    const unsigned fraction = (unsigned)((magnitude % 4U) * 25U);
    if (fraction == 0) snprintf(out, size, "SNR %s%u dB", negative ? "-" : "",
                                (unsigned)(magnitude / 4U));
    else snprintf(out, size, "SNR %s%u.%02u dB", negative ? "-" : "",
                  (unsigned)(magnitude / 4U), fraction);
}

static uint32_t device_now_epoch(void)
{
    const time_t wall_clock = time(NULL);
    return wall_clock > 0 ? (uint32_t)wall_clock : (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

static uint32_t device_message_epoch(const meshcore_device_info_t *device)
{
    if (device == NULL) return 0;
    if (device->last_message_epoch != 0) return device->last_message_epoch;
    return device->last_message_ms / 1000U;
}

static bool device_message_is_online(const meshcore_device_info_t *device, uint32_t now_epoch)
{
    const uint32_t message_epoch = device_message_epoch(device);
    return device != NULL && device->message_seen && now_epoch >= message_epoch &&
           now_epoch - message_epoch <= 3U * 60U * 60U;
}

static void device_format_last_message(char *out, size_t size,
                                       const meshcore_device_info_t *device,
                                       uint32_t now_epoch)
{
    if (device == NULL || !device->message_seen) {
        snprintf(out, size, "Offline | Last msg never");
        return;
    }
    const uint32_t message_epoch = device_message_epoch(device);
    const uint32_t age = now_epoch >= message_epoch ? now_epoch - message_epoch : 0;
    const char *state = device_message_is_online(device, now_epoch) ? "Online" : "Offline";
    char when[24] = "1970-01-01 00:00";
    time_t value = (time_t)message_epoch;
    struct tm local_time;
    if (localtime_r(&value, &local_time) != NULL) strftime(when, sizeof(when), "%Y-%m-%d %H:%M", &local_time);
    if (age < 60U) snprintf(out, size, "%s | %s", state, when);
    else if (age < 3600U) snprintf(out, size, "%s | %s (%lum ago)", state, when, (unsigned long)(age / 60U));
    else if (age < 86400U) snprintf(out, size, "%s | %s (%luh ago)", state, when, (unsigned long)(age / 3600U));
    else snprintf(out, size, "%s | %s (%lud ago)", state, when, (unsigned long)(age / 86400U));
}

static lv_obj_t *device_create_rabbit_icon(lv_obj_t *parent)
{
    const lv_color_t color = lv_color_hex(0x64748B);
    lv_obj_t *icon = lv_obj_create(parent);
    lv_obj_set_size(icon, 38, 38);
    lv_obj_set_style_bg_opa(icon, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(icon, 0, 0);
    lv_obj_set_style_pad_all(icon, 0, 0);
    lv_obj_remove_flag(icon, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t *left = lv_obj_create(icon);
    lv_obj_set_size(left, 7, 18); lv_obj_set_pos(left, 9, 1);
    lv_obj_set_style_radius(left, LV_RADIUS_CIRCLE, 0); lv_obj_set_style_bg_color(left, color, 0);
    lv_obj_set_style_border_width(left, 0, 0);
    lv_obj_t *right = lv_obj_create(icon);
    lv_obj_set_size(right, 7, 18); lv_obj_set_pos(right, 22, 1);
    lv_obj_set_style_radius(right, LV_RADIUS_CIRCLE, 0); lv_obj_set_style_bg_color(right, color, 0);
    lv_obj_set_style_border_width(right, 0, 0);
    lv_obj_t *head = lv_obj_create(icon);
    lv_obj_set_size(head, 28, 23); lv_obj_align(head, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_radius(head, LV_RADIUS_CIRCLE, 0); lv_obj_set_style_bg_color(head, color, 0);
    lv_obj_set_style_border_width(head, 0, 0);
    for (int i = 0; i < 2; ++i) {
        lv_obj_t *eye = lv_obj_create(head);
        lv_obj_set_size(eye, 3, 3); lv_obj_set_pos(eye, i ? 17 : 6, 7);
        lv_obj_set_style_radius(eye, LV_RADIUS_CIRCLE, 0); lv_obj_set_style_bg_color(eye, lv_color_white(), 0);
        lv_obj_set_style_border_width(eye, 0, 0);
    }
    return icon;
}

static void devices_render(const meshcore_device_info_t *devices, size_t count)
{
    if (s_devices_list == NULL) return;
    lv_obj_clean(s_devices_list);
    if (count == 0) {
        lv_obj_t *empty = lv_obj_create(s_devices_list);
        lv_obj_set_size(empty, LV_PCT(100), 150);
        lv_obj_set_style_bg_opa(empty, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(empty, 0, 0);
        lv_obj_remove_flag(empty, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *title = lv_label_create(empty);
        lv_label_set_text(title, "No MeshCore devices");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(title, lv_color_hex(0x334155), 0);
        lv_obj_align(title, LV_ALIGN_CENTER, 0, -12);
        lv_obj_t *detail = lv_label_create(empty);
        lv_label_set_text(detail, "Waiting for real node adverts");
        lv_obj_set_style_text_color(detail, lv_color_hex(0x94A3B8), 0);
        lv_obj_align_to(detail, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);
        return;
    }
    const uint32_t now_epoch = device_now_epoch();
    for (size_t i = 0; i < count; ++i) {
        const meshcore_device_info_t *device = &devices[i];
        lv_obj_t *row = lv_obj_create(s_devices_list);
        lv_obj_set_width(row, LV_PCT(100));
        lv_obj_set_height(row, 84);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                           LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);
        lv_obj_set_style_bg_color(row, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
        lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
        lv_obj_set_style_border_width(row, 1, 0);
        lv_obj_set_style_border_color(row, lv_color_hex(0xE2E8F0), 0);
        lv_obj_set_style_radius(row, 0, 0);
        lv_obj_set_style_pad_hor(row, 14, 0);
        lv_obj_set_style_pad_column(row, 12, 0);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);

        lv_obj_t *avatar = lv_obj_create(row);
        lv_obj_set_size(avatar, 48, 48);
        lv_obj_set_style_radius(avatar, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(avatar, 0, 0);
        lv_obj_set_style_bg_color(avatar, lv_color_hex(device_name_color(device->name)), 0);
        lv_obj_remove_flag(avatar, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *initial = lv_label_create(avatar);
        const char first = (device->name[0] >= 32 && device->name[0] <= 126) ?
                           device->name[0] : '#';
        char letter[2] = {first, '\0'};
        lv_label_set_text(initial, letter);
        lv_obj_set_style_text_color(initial, lv_color_white(), 0);
        lv_obj_set_style_text_font(initial, &lv_font_montserrat_18, 0);
        lv_obj_center(initial);

        lv_obj_t *info = lv_obj_create(row);
        lv_obj_set_height(info, LV_SIZE_CONTENT);
        lv_obj_set_flex_grow(info, 1);
        lv_obj_set_style_bg_opa(info, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(info, 0, 0);
        lv_obj_set_style_pad_all(info, 0, 0);
        lv_obj_set_style_pad_row(info, 4, 0);
        lv_obj_set_flex_flow(info, LV_FLEX_FLOW_COLUMN);
        lv_obj_remove_flag(info, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_t *name = lv_label_create(info);
        lv_label_set_text(name, device->name);
        lv_label_set_long_mode(name, LV_LABEL_LONG_DOT);
        lv_obj_set_width(name, LV_PCT(100));
        lv_obj_set_style_text_font(name, &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(name, lv_color_hex(0x0F172A), 0);
        lv_obj_t *type = lv_label_create(info);
        lv_label_set_text(type, device_type_name(device->type));
        lv_obj_set_style_text_font(type, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(type, lv_color_hex(0x64748B), 0);

        lv_obj_t *metric = lv_obj_create(row);
        lv_obj_set_size(metric, 230, LV_SIZE_CONTENT);
        lv_obj_set_style_bg_opa(metric, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(metric, 0, 0);
        lv_obj_set_style_pad_all(metric, 0, 0);
        lv_obj_remove_flag(metric, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_flex_flow(metric, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(metric, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_END);
        if (device->route_known && device->hop_count == 0 && device->metrics_valid) {
            lv_obj_t *rssi = lv_label_create(metric);
            lv_label_set_text_fmt(rssi, "RSSI %d dBm", (int)device->rssi_dbm);
            lv_obj_set_style_text_font(rssi, &lv_font_montserrat_14, 0);
            lv_obj_set_style_text_color(rssi, lv_color_hex(0x047857), 0);
            char snr_text[32]; device_snr_text(snr_text, sizeof(snr_text), device->snr_quarter_db);
            lv_obj_t *snr = lv_label_create(metric); lv_label_set_text(snr, snr_text);
            lv_obj_set_style_text_font(snr, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(snr, lv_color_hex(0x64748B), 0);
            char last[40]; device_format_last_message(last, sizeof(last), device, now_epoch);
            lv_obj_t *last_label = lv_label_create(metric); lv_label_set_text(last_label, last);
            lv_obj_set_style_text_font(last_label, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(last_label, device_message_is_online(device, now_epoch)
                                                  ? lv_color_hex(0x047857) : lv_color_hex(0x94A3B8), 0);
        } else if (device->route_known) {
            lv_obj_t *hops = lv_label_create(metric);
            lv_label_set_text_fmt(hops, "%u %s", (unsigned)device->hop_count,
                                  device->hop_count == 1 ? "hop" : "hops");
            lv_obj_set_style_text_font(hops, &lv_font_montserrat_16, 0);
            lv_obj_set_style_text_color(hops, lv_color_hex(0x2563EB), 0);
            lv_obj_t *relay = lv_label_create(metric); lv_label_set_text(relay, "Relayed");
            lv_obj_set_style_text_font(relay, &lv_font_montserrat_12, 0);
            lv_obj_set_style_text_color(relay, lv_color_hex(0x64748B), 0);
            char last[40]; device_format_last_message(last, sizeof(last), device, now_epoch);
            lv_obj_t *last_label = lv_label_create(metric); lv_label_set_text(last_label, last);
            lv_obj_set_style_text_font(last_label, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(last_label, device_message_is_online(device, now_epoch)
                                                  ? lv_color_hex(0x047857) : lv_color_hex(0x94A3B8), 0);
        } else {
            lv_obj_set_flex_flow(metric, LV_FLEX_FLOW_ROW);
            lv_obj_set_flex_align(metric, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
            device_create_rabbit_icon(metric);
            char last[40]; device_format_last_message(last, sizeof(last), device, now_epoch);
            lv_obj_t *last_label = lv_label_create(metric); lv_label_set_text(last_label, last);
            lv_obj_set_style_text_font(last_label, &lv_font_montserrat_10, 0);
            lv_obj_set_style_text_color(last_label, device_message_is_online(device, now_epoch)
                                                  ? lv_color_hex(0x047857) : lv_color_hex(0x94A3B8), 0);
        }
    }
}

static void devices_update_cb(lv_timer_t *timer)
{
    (void)timer;
    if (s_device_snapshot == NULL || s_devices_list == NULL) return;
    uint32_t generation = 0;
    const size_t count = meshcore_core_get_devices(s_device_snapshot,
                                                   MESHCORE_DEVICE_LIST_MAX,
                                                   &generation);
    const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (generation == s_devices_generation &&
        (uint32_t)(now_ms - s_devices_last_render_ms) < 10000U) return;
    const lv_coord_t scroll_y = s_devices_generation == UINT32_MAX
                                    ? 0 : lv_obj_get_scroll_y(s_devices_list);
    s_devices_generation = generation;
    devices_render(s_device_snapshot, count);
    s_devices_last_render_ms = now_ms;
    if (scroll_y > 0 && count > 0) lv_obj_scroll_to_y(s_devices_list, scroll_y, LV_ANIM_OFF);
}

static void devices_page_create(lv_obj_t *parent)
{
    lv_obj_set_style_pad_all(parent, 0, 0);
    lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC |
                       LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(parent, LV_DIR_NONE);
    lv_obj_set_scrollbar_mode(parent, LV_SCROLLBAR_MODE_OFF);

    s_devices_list = lv_obj_create(parent);
    lv_obj_set_size(s_devices_list, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(s_devices_list, lv_color_hex(0xF8FAFC), 0);
    lv_obj_set_style_bg_opa(s_devices_list, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_devices_list, 0, 0);
    lv_obj_set_style_pad_all(s_devices_list, 12, 0);
    lv_obj_set_style_pad_row(s_devices_list, 0, 0);
    lv_obj_add_flag(s_devices_list, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_remove_flag(s_devices_list, LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(s_devices_list, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_devices_list, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(s_devices_list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(s_devices_list, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_START);

    s_device_snapshot = heap_caps_calloc(MESHCORE_DEVICE_LIST_MAX,
                                         sizeof(meshcore_device_info_t),
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_device_snapshot == NULL) {
        s_device_snapshot = calloc(MESHCORE_DEVICE_LIST_MAX,
                                   sizeof(meshcore_device_info_t));
    }
    devices_update_cb(NULL);
    lv_timer_create(devices_update_cb, 1000, NULL);
}

static void map_gps_ui_create(lv_obj_t *parent)
{
    if (!s_raw_mode && s_marker_count + 3 <= MAP_MARKER_MAX) {
        s_gps_marker_first = s_marker_count;

        lv_obj_t *dot = lv_obj_create(s_map_layer);
        lv_obj_remove_flag(dot, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(dot, 16, 16);
        lv_obj_set_style_translate_x(dot, -8, 0);
        lv_obj_set_style_translate_y(dot, -8, 0);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(dot, 3, 0);
        lv_obj_set_style_border_color(dot, lv_color_white(), 0);
        lv_obj_set_style_bg_color(dot, lv_color_hex(0x087CF0), 0);
        lv_obj_set_style_shadow_width(dot, 7, 0);
        lv_obj_set_style_shadow_color(dot, lv_color_hex(0x064F9E), 0);
        lv_obj_set_style_shadow_opa(dot, LV_OPA_50, 0);
        lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
        s_markers[s_marker_count++] = (map_marker_t){.obj = dot, .tx = 0, .ty = 0};

        s_map_device_bubble_tail = lv_obj_create(s_map_layer);
        lv_obj_remove_flag(s_map_device_bubble_tail,
                           LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_size(s_map_device_bubble_tail, 12, 12);
        lv_obj_set_style_translate_x(s_map_device_bubble_tail, -6, 0);
        lv_obj_set_style_translate_y(s_map_device_bubble_tail, -22, 0);
        lv_obj_set_style_transform_rotation(s_map_device_bubble_tail, 450, 0);
        lv_obj_set_style_bg_color(s_map_device_bubble_tail, lv_color_white(), 0);
        lv_obj_set_style_border_width(s_map_device_bubble_tail, 0, 0);
        lv_obj_set_style_radius(s_map_device_bubble_tail, 2, 0);
        lv_obj_add_flag(s_map_device_bubble_tail, LV_OBJ_FLAG_HIDDEN);
        s_markers[s_marker_count++] = (map_marker_t){
            .obj = s_map_device_bubble_tail, .tx = 0, .ty = 0};

        s_map_device_bubble = lv_obj_create(s_map_layer);
        lv_obj_remove_flag(s_map_device_bubble, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(s_map_device_bubble, 0, 0);
        lv_obj_set_style_bg_color(s_map_device_bubble, lv_color_white(), 0);
        lv_obj_set_style_border_width(s_map_device_bubble, 1, 0);
        lv_obj_set_style_border_color(s_map_device_bubble, lv_color_hex(0x9BC8F5), 0);
        lv_obj_set_style_radius(s_map_device_bubble, 7, 0);
        lv_obj_set_style_shadow_width(s_map_device_bubble, 10, 0);
        lv_obj_set_style_shadow_color(s_map_device_bubble, lv_color_hex(0x344054), 0);
        lv_obj_set_style_shadow_opa(s_map_device_bubble, LV_OPA_20, 0);
        lv_obj_add_event_cb(s_map_device_bubble, map_device_bubble_cb,
                            LV_EVENT_CLICKED, NULL);

        s_map_device_name_lbl = lv_label_create(s_map_device_bubble);
        lv_label_set_text(s_map_device_name_lbl, s_device_name);
        lv_obj_set_pos(s_map_device_name_lbl, 12, 10);
        lv_obj_set_style_text_font(s_map_device_name_lbl, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(s_map_device_name_lbl, lv_color_hex(0x123B65), 0);
        lv_label_set_long_mode(s_map_device_name_lbl, LV_LABEL_LONG_MODE_DOTS);

        s_map_device_expand_lbl = lv_label_create(s_map_device_bubble);
        lv_obj_set_style_text_color(s_map_device_expand_lbl, lv_color_hex(0x1769AA), 0);
        s_map_device_info_lbl = lv_label_create(s_map_device_bubble);
        lv_label_set_text(s_map_device_info_lbl,
                          "satellites: --\nspeed: 0.0 km/h\nlast fix: --");
        lv_obj_set_pos(s_map_device_info_lbl, 14, 43);
        lv_obj_set_style_text_font(s_map_device_info_lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_line_space(s_map_device_info_lbl, 5, 0);
        lv_obj_set_style_text_color(s_map_device_info_lbl, lv_color_hex(0x465568), 0);
        s_map_device_bubble_expanded = false;
        map_device_bubble_apply_state();
        lv_obj_add_flag(s_map_device_bubble, LV_OBJ_FLAG_HIDDEN);
        s_markers[s_marker_count++] = (map_marker_t){
            .obj = s_map_device_bubble, .tx = 0, .ty = 0};
    }

    lv_obj_t *locate_btn = lv_btn_create(parent);
    lv_obj_set_pos(locate_btn, MAP_W - 48, 176);
    lv_obj_set_size(locate_btn, 38, 38);
    lv_obj_set_style_radius(locate_btn, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(locate_btn, lv_color_hex(0x1769AA), 0);
    lv_obj_set_style_bg_opa(locate_btn, LV_OPA_90, 0);
    lv_obj_set_style_border_width(locate_btn, 1, 0);
    lv_obj_set_style_border_color(locate_btn, lv_color_white(), 0);
    lv_obj_set_style_border_opa(locate_btn, LV_OPA_40, 0);
    lv_obj_add_event_cb(locate_btn, map_locate_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *locate_icon = lv_label_create(locate_btn);
    lv_label_set_text(locate_icon, LV_SYMBOL_GPS);
    lv_obj_set_style_text_color(locate_icon, lv_color_white(), 0);
    lv_obj_center(locate_icon);

}

static void map_create(lv_obj_t *parent)
{
    lv_obj_set_style_pad_all(parent, 0, 0);
    lv_obj_remove_flag(parent, LV_OBJ_FLAG_SCROLLABLE);
    s_map_layer = lv_obj_create(parent);
    lv_obj_set_size(s_map_layer, GRID_COLS * TILE_SIZE, GRID_ROWS * TILE_SIZE);
    lv_obj_set_style_pad_all(s_map_layer, 0, 0);
    lv_obj_set_style_border_width(s_map_layer, 0, 0);
    lv_obj_set_style_bg_opa(s_map_layer, LV_OPA_TRANSP, 0);
    lv_obj_remove_flag(s_map_layer, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    lv_obj_add_flag(parent, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(parent, map_pan_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(parent, map_pan_event_cb, LV_EVENT_PRESSING, NULL);
    lv_obj_add_event_cb(parent, map_pan_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(parent, map_pan_event_cb, LV_EVENT_PRESS_LOST, NULL);

    s_map_message = lv_label_create(parent);
    lv_obj_center(s_map_message);
    lv_obj_set_style_text_color(s_map_message, lv_color_hex(0x5B6470), 0);

    s_scale_bar = lv_obj_create(parent);
    lv_obj_set_pos(s_scale_bar, 8, MAP_H - 26);
    lv_obj_set_size(s_scale_bar, 80, 4);
    lv_obj_set_style_pad_all(s_scale_bar, 0, 0);
    lv_obj_set_style_bg_color(s_scale_bar, lv_color_hex(0x37474F), 0);
    lv_obj_set_style_bg_opa(s_scale_bar, LV_OPA_80, 0);
    lv_obj_set_style_radius(s_scale_bar, 2, 0);
    lv_obj_set_style_border_width(s_scale_bar, 0, 0);
    lv_obj_remove_flag(s_scale_bar, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    s_scale_label = lv_label_create(parent);
    lv_obj_set_style_text_color(s_scale_label, lv_color_hex(0x37474F), 0);
    lv_obj_set_style_text_font(s_scale_label, &lv_font_montserrat_10, 0);
    lv_obj_remove_flag(s_scale_label, LV_OBJ_FLAG_CLICKABLE);

    select_map_root();
    if (raw_map_load()) {
        lv_obj_add_flag(s_map_message, LV_OBJ_FLAG_HIDDEN);
        map_gps_ui_create(parent);
        s_map_timer = lv_timer_create(map_results_timer_cb, MAP_RESULT_FRAME_MS, NULL);
        lv_timer_create(gps_ui_timer_cb, 500, NULL);
        gps_ui_timer_cb(NULL);
        return;
    }

    s_tile_capacity = 0;
    for (int i = 0; i < TILE_COUNT; i++) {
        s_tiles[i].pixels = heap_caps_malloc(TILE_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_tiles[i].pixels == NULL) {
            ESP_LOGW(TAG, "PSRAM tile slot %d unavailable", i);
            continue;
        }
        s_tile_capacity++;
        s_tiles[i].z = INT32_MIN;
    }

    for (int i = 0; i < GRID_COUNT; i++) {
        s_cells[i].image = lv_image_create(s_map_layer);
        s_cells[i].z = INT32_MIN;
        lv_image_set_antialias(s_cells[i].image, false);
        lv_obj_add_flag(s_cells[i].image, LV_OBJ_FLAG_HIDDEN);
    }

    ESP_LOGI(TAG, "PSRAM tile cache: %u/%u slots, %u KiB",
             (unsigned)s_tile_capacity, (unsigned)TILE_COUNT,
             (unsigned)((s_tile_capacity * TILE_BYTES) / 1024U));

    scan_levels();
    double center_x = 0.0, center_y = 0.0;
    if (s_level_count > 0 && find_level_center(s_levels[s_level_count - 1], &center_x, &center_y)) {
        s_zoom = s_levels[s_level_count - 1];
        s_world_x = center_x * TILE_SIZE - MAP_W * 0.5;
        s_world_y = center_y * TILE_SIZE - MAP_H * 0.5;
        lv_label_set_text(s_map_message, "Loading SD map...");
        ESP_LOGI(TAG, "tile tree ready: root=%s levels=%d..%d z=%d center=(%.1f,%.1f)",
                 s_map_root, s_levels[0], s_levels[s_level_count - 1], s_zoom,
                 center_x, center_y);
        ESP_LOGI(TAG, "tile format: %s", s_raw_tile_tree ?
                 "PNG + optional RGB565" : "PNG (no raw probes)");
    } else {
        ESP_LOGW(TAG, "no PNG tiles found under %s (need z/x/y.png)", s_map_root);
        lv_label_set_text(s_map_message, "No /sdcard/map/z/x/y.png tiles");
    }

    map_loader_start();
    if (s_zoom >= 0) {
        map_refresh_grid();
        /* Start filling the surrounding ring immediately. Without this the
         * first stationary frame only has the viewport and its outer edge can
         * remain blank until the next gesture. */
        map_prefetch_margin();
        scale_bar_update();
    }
    map_zoom_button_create(parent, "+", 18, true);
    map_zoom_button_create(parent, "-", 64, false);
    map_gps_ui_create(parent);
    s_map_timer = lv_timer_create(map_results_timer_cb, MAP_RESULT_FRAME_MS, NULL);
    lv_timer_create(gps_ui_timer_cb, 500, NULL);
    gps_ui_timer_cb(NULL);
}

void Lvgl_App_Init(void)
{
    settings_load();

    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    lv_obj_t *status = lv_obj_create(screen);
    lv_obj_set_pos(status, 0, 0);
    lv_obj_set_size(status, LV_PCT(100), STATUS_H);
    lv_obj_remove_flag(status, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_pad_all(status, 0, 0);
    lv_obj_set_style_bg_color(status, lv_color_hex(0xF8F9FA), 0);
    lv_obj_set_style_bg_opa(status, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(status, 1, 0);
    lv_obj_set_style_border_color(status, lv_color_hex(0xD7DBE0), 0);

    lv_obj_t *title = lv_label_create(status);
    lv_label_set_text(title, LV_SYMBOL_GPS "  TinyTab");
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 12, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x374151), 0);

    s_status_time = lv_label_create(status);
    lv_obj_align(s_status_time, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_text_font(s_status_time, &lv_font_montserrat_14, 0);
    s_status_info = lv_label_create(status);
    lv_obj_align(s_status_info, LV_ALIGN_RIGHT_MID, -12, 0);
    lv_obj_set_style_text_font(s_status_info, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_status_info, lv_color_hex(0x374151), 0);

    /* Match the backup navigation shell: a fixed left tab bar and five pages.
     * MeshCore core and SX1262 are started by app_main after the UI is ready. */
    lv_obj_t *tv = lv_tabview_create(screen);
    s_tabview = tv;
    lv_tabview_set_tab_bar_position(tv, LV_DIR_LEFT);
    lv_tabview_set_tab_bar_size(tv, SIDE_MENU_W);
    lv_obj_set_pos(tv, 0, STATUS_H);
    lv_obj_set_size(tv, LV_PCT(100), MAP_H);
    lv_obj_set_style_bg_color(tv, lv_color_hex(0xF3F5F7), 0);
    lv_obj_set_style_bg_opa(tv, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(tv, 0, 0);
    lv_obj_remove_flag(lv_tabview_get_content(tv), LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *tab_btns = lv_tabview_get_tab_bar(tv);
    lv_obj_set_height(tab_btns, 390);
    lv_obj_set_style_bg_color(tab_btns, lv_color_hex(0xE9EDF2), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(tab_btns, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(tab_btns, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_top(tab_btns, 58, LV_PART_MAIN);
    lv_obj_set_style_pad_bottom(tab_btns, 12, LV_PART_MAIN);
    lv_obj_set_style_pad_left(tab_btns, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_right(tab_btns, 10, LV_PART_MAIN);
    lv_obj_set_style_pad_row(tab_btns, 8, LV_PART_MAIN);
    lv_obj_set_style_text_align(tab_btns, LV_TEXT_ALIGN_LEFT, LV_PART_ITEMS);
    lv_obj_set_style_text_color(tab_btns, lv_color_hex(0x5B6470), LV_PART_ITEMS);
    lv_obj_set_style_bg_opa(tab_btns, LV_OPA_TRANSP, LV_PART_ITEMS);
    lv_obj_set_style_radius(tab_btns, 9, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(tab_btns, 14, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(tab_btns, 10, LV_PART_ITEMS);
    lv_obj_set_style_bg_color(tab_btns, lv_color_hex(0xFFFFFF),
                              LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_text_color(tab_btns, lv_color_hex(0x1769AA),
                                LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_side(tab_btns, LV_BORDER_SIDE_LEFT,
                                 LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_width(tab_btns, 3,
                                  LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_border_color(tab_btns, lv_color_hex(0x3B82F6),
                                  LV_PART_ITEMS | LV_STATE_CHECKED);

    lv_obj_t *t1 = lv_tabview_add_tab(tv, LV_SYMBOL_ENVELOPE "  Chat");
    lv_obj_t *t2 = lv_tabview_add_tab(tv, LV_SYMBOL_LIST "  Devices");
    lv_obj_t *t3 = lv_tabview_add_tab(tv, LV_SYMBOL_GPS "  Map");
    lv_obj_t *t4 = lv_tabview_add_tab(tv, LV_SYMBOL_SETTINGS "  Settings");
    lv_obj_t *t5 = lv_tabview_add_tab(tv, LV_SYMBOL_DRIVE "  System");
    for (uint32_t i = 0; i < lv_tabview_get_tab_count(tv); i++) {
        lv_obj_t *tab_button = lv_tabview_get_tab_button(tv, i);
        lv_obj_t *tab_label = lv_obj_get_child(tab_button, 0);
        lv_obj_align(tab_label, LV_ALIGN_LEFT_MID, 14, 0);
        lv_obj_set_style_text_align(tab_label, LV_TEXT_ALIGN_LEFT, 0);
    }
    lv_obj_t *sidebar_title = lv_label_create(tab_btns);
    lv_label_set_text(sidebar_title, "NAVIGATION");
    lv_obj_set_style_text_color(sidebar_title, lv_color_hex(0x7B8490), 0);
    lv_obj_set_style_text_font(sidebar_title, &lv_font_montserrat_12, 0);
    lv_obj_align(sidebar_title, LV_ALIGN_TOP_LEFT, 14, 16);
    lv_obj_remove_flag(sidebar_title, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    chat_page_create(t1);
    devices_page_create(t2);
    map_create(t3);
    settings_page_create(t4);
    system_page_create(t5);
    lv_tabview_set_active(tv, 1, LV_ANIM_OFF);

    lv_display_t *display = lv_display_get_default();
    if (display != NULL) {
        lv_display_add_event_cb(display, display_event_cb, LV_EVENT_ALL, NULL);
    }
    /* Keep the diagnostic overlay off during normal map use. It can be enabled
     * from Settings -> Developer Options -> FPS Overlay. */
    lv_sysmon_hide_performance(NULL);
    s_frame_window_start = lv_tick_get();
    lv_timer_create(status_timer_cb, 1000, NULL);
    status_timer_cb(NULL);
}
