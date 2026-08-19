#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MESHCORE_FRAME_MAX_LEN 176
#define MESHCORE_CHAT_TEXT_MAX_LEN 160
#define MESHCORE_DEVICE_NAME_MAX_LEN 32
#define MESHCORE_DEVICE_LIST_MAX 64

typedef struct {
    bool is_local;
    uint32_t timestamp;
    char text[MESHCORE_CHAT_TEXT_MAX_LEN + 1];
} meshcore_chat_message_t;

typedef struct {
    char name[MESHCORE_DEVICE_NAME_MAX_LEN];
    uint8_t type;
    bool route_known;
    bool metrics_valid;
    bool message_seen;
    uint8_t hop_count;
    int16_t rssi_dbm;
    int16_t snr_quarter_db;
    uint32_t last_heard_ms;
    uint32_t last_message_ms;
    uint32_t last_message_epoch;
} meshcore_device_info_t;

typedef struct {
    float frequency_mhz;
    float bandwidth_khz;
    uint8_t spreading_factor;
    uint8_t coding_rate;
    int8_t tx_power_dbm;
    uint8_t rx_boosted_gain;
    uint8_t client_repeat;
    uint8_t path_hash_mode;
    uint8_t autoadd_max_hops;
    uint8_t autoadd_config;
    uint8_t advert_loc_policy;
} meshcore_lora_config_t;

esp_err_t meshcore_core_start(void);
bool meshcore_core_is_running(void);
esp_err_t meshcore_core_send_public(const char *text);
bool meshcore_core_pop_message(meshcore_chat_message_t *message);
size_t meshcore_core_get_devices(meshcore_device_info_t *devices, size_t max_devices,
                                 uint32_t *generation);
esp_err_t meshcore_core_get_lora_config(meshcore_lora_config_t *config);
esp_err_t meshcore_core_set_lora_config(const meshcore_lora_config_t *config);
esp_err_t meshcore_core_send_advert(bool flood);

esp_err_t meshcore_core_transport_push_rx(const uint8_t *data, size_t len);
size_t meshcore_core_transport_pop_tx(uint8_t *data, size_t max_len);
void meshcore_core_transport_set_connected(bool connected);

const char *meshcore_core_get_node_name(void);
uint32_t meshcore_core_get_ble_pin(void);

#ifdef __cplusplus
}
#endif
