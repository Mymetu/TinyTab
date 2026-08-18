#include "GPS.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"

#define GPS_UART_NUM           UART_NUM_1
#define GPS_UART_BAUD          9600
#define GPS_UART_RX_GPIO       GPIO_NUM_2
#define GPS_UART_TX_GPIO       GPIO_NUM_3
#define GPS_RX_BUFFER_SIZE     2048
#define GPS_LINE_MAX           128
#define GPS_STALE_MS           3000

static const char *TAG = "GPS";
static portMUX_TYPE s_data_lock = portMUX_INITIALIZER_UNLOCKED;
static gps_data_t s_data;
static bool s_initialized;

static uint32_t gps_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static bool nmea_checksum_valid(const char *line)
{
    if (line == NULL || line[0] != '$') return false;

    const char *star = strchr(line, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0') return false;

    uint8_t checksum = 0;
    for (const char *p = line + 1; p < star; p++) {
        checksum ^= (uint8_t)*p;
    }

    int hi = hex_value(star[1]);
    int lo = hex_value(star[2]);
    return hi >= 0 && lo >= 0 && checksum == (uint8_t)((hi << 4) | lo);
}

static int nmea_split_fields(char *line, char **fields, int max_fields)
{
    int count = 0;
    if (line == NULL || max_fields <= 0) return 0;

    fields[count++] = line;
    for (char *p = line; *p != '\0'; p++) {
        if (*p == '*') {
            *p = '\0';
            break;
        }
        if (*p == ',' && count < max_fields) {
            *p = '\0';
            fields[count++] = p + 1;
        }
    }
    return count;
}

static bool nmea_coordinate(const char *value, const char *hemisphere,
                            bool latitude, double *coordinate)
{
    if (value == NULL || value[0] == '\0' || hemisphere == NULL ||
            hemisphere[0] == '\0' || coordinate == NULL) {
        return false;
    }

    char *end = NULL;
    double raw = strtod(value, &end);
    if (end == value || raw < 0.0) return false;

    double degrees = floor(raw / 100.0);
    double minutes = raw - degrees * 100.0;
    double limit = latitude ? 90.0 : 180.0;
    if (minutes >= 60.0 || degrees > limit) return false;

    double result = degrees + minutes / 60.0;
    char h = hemisphere[0];
    if (h == 'S' || h == 'W') {
        result = -result;
    } else if (h != 'N' && h != 'E') {
        return false;
    }
    *coordinate = result;
    return true;
}

static bool gps_is_leap_year(uint32_t year)
{
    return (year % 4U == 0U && year % 100U != 0U) || year % 400U == 0U;
}

static bool gps_parse_two_digits(const char *text, uint32_t *value)
{
    if (text == NULL || value == NULL || text[0] < '0' || text[0] > '9' ||
        text[1] < '0' || text[1] > '9') {
        return false;
    }
    *value = (uint32_t)(text[0] - '0') * 10U + (uint32_t)(text[1] - '0');
    return true;
}

static bool gps_rmc_utc_epoch(const char *time_text, const char *date_text,
                              uint32_t *epoch)
{
    if (time_text == NULL || date_text == NULL || epoch == NULL ||
        strlen(time_text) < 6 || strlen(date_text) < 6) {
        return false;
    }

    uint32_t hour, minute, second, day, month, year_short;
    if (!gps_parse_two_digits(time_text, &hour) ||
        !gps_parse_two_digits(time_text + 2, &minute) ||
        !gps_parse_two_digits(time_text + 4, &second) ||
        !gps_parse_two_digits(date_text, &day) ||
        !gps_parse_two_digits(date_text + 2, &month) ||
        !gps_parse_two_digits(date_text + 4, &year_short) ||
        hour > 23U || minute > 59U || second > 59U || month == 0U || month > 12U) {
        return false;
    }

    const uint32_t year = 2000U + year_short;
    static const uint8_t days_per_month[] = {31, 28, 31, 30, 31, 30,
                                             31, 31, 30, 31, 30, 31};
    uint32_t max_day = days_per_month[month - 1U];
    if (month == 2U && gps_is_leap_year(year)) max_day++;
    if (day == 0U || day > max_day) return false;

    uint64_t days = 0;
    for (uint32_t y = 1970U; y < year; y++) days += gps_is_leap_year(y) ? 366U : 365U;
    for (uint32_t m = 1U; m < month; m++) {
        days += days_per_month[m - 1U];
        if (m == 2U && gps_is_leap_year(year)) days++;
    }
    days += day - 1U;
    const uint64_t seconds = days * 86400ULL + hour * 3600ULL + minute * 60ULL + second;
    if (seconds > UINT32_MAX) return false;
    *epoch = (uint32_t)seconds;
    return true;
}

static void gps_mark_sentence(uint32_t now_ms)
{
    portENTER_CRITICAL(&s_data_lock);
    s_data.receiving = true;
    s_data.last_sentence_ms = now_ms;
    portEXIT_CRITICAL(&s_data_lock);
}

static void gps_parse_gga(char **fields, int count, uint32_t now_ms)
{
    if (count < 10) return;

    double latitude = 0.0;
    double longitude = 0.0;
    int quality = atoi(fields[6]);
    int satellites = atoi(fields[7]);
    bool position_valid = quality > 0 &&
                          nmea_coordinate(fields[2], fields[3], true, &latitude) &&
                          nmea_coordinate(fields[4], fields[5], false, &longitude);

    portENTER_CRITICAL(&s_data_lock);
    s_data.satellites = (uint8_t)(satellites < 0 ? 0 : (satellites > 255 ? 255 : satellites));
    if (fields[8][0] != '\0') s_data.hdop = strtof(fields[8], NULL);
    if (fields[9][0] != '\0') s_data.altitude_m = strtof(fields[9], NULL);
    s_data.valid = position_valid;
    if (position_valid) {
        s_data.latitude = latitude;
        s_data.longitude = longitude;
        s_data.last_fix_ms = now_ms;
    }
    portEXIT_CRITICAL(&s_data_lock);
}

static void gps_parse_rmc(char **fields, int count, uint32_t now_ms)
{
    if (count < 10) return;

    double latitude = 0.0;
    double longitude = 0.0;
    bool position_valid = fields[2][0] == 'A' &&
                          nmea_coordinate(fields[3], fields[4], true, &latitude) &&
                          nmea_coordinate(fields[5], fields[6], false, &longitude);
    uint32_t utc_time = 0;
    const bool time_valid = fields[2][0] == 'A' &&
                            gps_rmc_utc_epoch(fields[1], fields[9], &utc_time);

    portENTER_CRITICAL(&s_data_lock);
    if (fields[7][0] != '\0') s_data.speed_kmh = strtof(fields[7], NULL) * 1.852f;
    s_data.valid = position_valid;
    if (position_valid) {
        s_data.latitude = latitude;
        s_data.longitude = longitude;
        s_data.last_fix_ms = now_ms;
    }
    if (time_valid) {
        s_data.time_valid = true;
        s_data.utc_time = utc_time;
        s_data.last_time_ms = now_ms;
    }
    portEXIT_CRITICAL(&s_data_lock);
}

static void gps_parse_sentence(char *line)
{
    static bool stream_logged;
    static bool fix_logged;

    if (!nmea_checksum_valid(line)) return;

    uint32_t now_ms = gps_now_ms();
    gps_mark_sentence(now_ms);
    if (!stream_logged) {
        stream_logged = true;
        ESP_LOGI(TAG, "ATGM33 NMEA stream detected");
    }

    char *fields[20];
    int count = nmea_split_fields(line, fields, 20);
    size_t id_len = count > 0 ? strlen(fields[0]) : 0;
    if (id_len < 3) return;

    const char *type = fields[0] + id_len - 3;
    if (strcmp(type, "GGA") == 0) {
        gps_parse_gga(fields, count, now_ms);
    } else if (strcmp(type, "RMC") == 0) {
        gps_parse_rmc(fields, count, now_ms);
    } else {
        return;
    }

    gps_data_t data;
    if (!fix_logged && GPS_Get_Data(&data) && data.valid) {
        fix_logged = true;
        ESP_LOGI(TAG, "fix: %.6f, %.6f, satellites=%u, hdop=%.1f",
                 data.latitude, data.longitude, data.satellites, data.hdop);
    }
}

static void gps_uart_task(void *arg)
{
    (void)arg;
    uint8_t rx[128];
    char line[GPS_LINE_MAX];
    size_t line_len = 0;

    while (true) {
        int length = uart_read_bytes(GPS_UART_NUM, rx, sizeof(rx), pdMS_TO_TICKS(200));
        for (int i = 0; i < length; i++) {
            char c = (char)rx[i];
            if (c == '$') {
                line_len = 0;
                line[line_len++] = c;
            } else if (c == '\r' || c == '\n') {
                if (line_len > 0) {
                    line[line_len] = '\0';
                    gps_parse_sentence(line);
                    line_len = 0;
                }
            } else if (line_len > 0 && c >= 0x20 && c <= 0x7e) {
                if (line_len < sizeof(line) - 1) {
                    line[line_len++] = c;
                } else {
                    line_len = 0;
                }
            }
        }
    }
}

esp_err_t GPS_Init(void)
{
    if (s_initialized) return ESP_OK;

    const uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(GPS_UART_NUM, GPS_RX_BUFFER_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_param_config(GPS_UART_NUM, &uart_config);
    if (err == ESP_OK) {
        err = uart_set_pin(GPS_UART_NUM, GPS_UART_TX_GPIO, GPS_UART_RX_GPIO,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART configuration failed: %s", esp_err_to_name(err));
        uart_driver_delete(GPS_UART_NUM);
        return err;
    }

    if (xTaskCreatePinnedToCoreWithCaps(gps_uart_task, "atgm33", 4096, NULL, 3,
                                         NULL, 1,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(TAG, "failed to start ATGM33 task");
        uart_driver_delete(GPS_UART_NUM);
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "ATGM33 UART1 ready: RX GPIO2 <- GPS TX, TX GPIO3 -> GPS RX, 9600 baud");
    return ESP_OK;
}

bool GPS_Get_Data(gps_data_t *data)
{
    if (data == NULL || !s_initialized) return false;

    portENTER_CRITICAL(&s_data_lock);
    *data = s_data;
    portEXIT_CRITICAL(&s_data_lock);

    uint32_t now_ms = gps_now_ms();
    if ((uint32_t)(now_ms - data->last_sentence_ms) > GPS_STALE_MS) {
        data->receiving = false;
    }
    if ((uint32_t)(now_ms - data->last_fix_ms) > GPS_STALE_MS) {
        data->valid = false;
    }
    return true;
}
