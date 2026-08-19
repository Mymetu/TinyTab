#include "meshcore_ble.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_sm.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "meshcore_core.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "nvs.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "meshcore_ble";

static const ble_uuid128_t s_service_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_rx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t s_tx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

static uint16_t s_tx_value_handle;
static uint16_t s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
static uint8_t s_own_address_type;
static bool s_notify_enabled;
static bool s_authenticated;
static bool s_started;
static bool s_enabled = true;
static bool s_enabled_loaded;

#define BLE_NVS_NAMESPACE "meshcore_ble"
#define BLE_NVS_ENABLED   "enabled"

void ble_store_config_init(void);

static void advertise(void);

static void load_enabled_setting(void)
{
    if (s_enabled_loaded) return;
    s_enabled_loaded = true;

    nvs_handle_t handle;
    if (nvs_open(BLE_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return;
    uint8_t enabled = 1;
    if (nvs_get_u8(handle, BLE_NVS_ENABLED, &enabled) == ESP_OK) {
        s_enabled = enabled != 0;
    }
    nvs_close(handle);
}

static esp_err_t save_enabled_setting(bool enabled)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(BLE_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(handle, BLE_NVS_ENABLED, enabled ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static void log_memory(const char *stage)
{
    const uint32_t internal_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    ESP_LOGI(TAG, "MEM %-12s internal=%u largest=%u dma=%u psram=%u",
             stage,
             (unsigned)heap_caps_get_free_size(internal_caps),
             (unsigned)heap_caps_get_largest_free_block(internal_caps),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

static int gatt_access(uint16_t connection_handle, uint16_t attribute_handle,
                       struct ble_gatt_access_ctxt *context, void *argument)
{
    (void)connection_handle;
    (void)attribute_handle;
    if (!s_enabled) return BLE_ATT_ERR_INSUFFICIENT_AUTHEN;
    if (argument == (void *)&s_rx_uuid && context->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint8_t frame[MESHCORE_FRAME_MAX_LEN];
        uint16_t length = 0;
        int rc = ble_hs_mbuf_to_flat(context->om, frame, sizeof(frame), &length);
        if (rc != 0 || length == 0 || length > sizeof(frame)) {
            return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
        }
        return meshcore_core_transport_push_rx(frame, length) == ESP_OK
                   ? 0
                   : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (argument == (void *)&s_tx_uuid && context->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_rx_uuid.u,
                .access_cb = gatt_access,
                .arg = (void *)&s_rx_uuid,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                         BLE_GATT_CHR_F_WRITE_ENC | BLE_GATT_CHR_F_WRITE_AUTHEN,
            },
            {
                .uuid = &s_tx_uuid.u,
                .access_cb = gatt_access,
                .arg = (void *)&s_tx_uuid,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY |
                         BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_READ_AUTHEN |
                         BLE_GATT_CHR_F_NOTIFY_INDICATE_ENC |
                         BLE_GATT_CHR_F_NOTIFY_INDICATE_AUTHEN,
                .val_handle = &s_tx_value_handle,
            },
            {0},
        },
    },
    {0},
};

static int gap_event(struct ble_gap_event *event, void *argument)
{
    (void)argument;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connection_handle = event->connect.conn_handle;
            s_notify_enabled = false;
            s_authenticated = false;
            meshcore_core_transport_set_connected(false);
            ESP_LOGI(TAG, "BLE connected, waiting for authentication");
            /* Start security immediately. The MeshCore mobile clients expect
             * the peripheral to initiate the bonded MITM pairing after the
             * link is established, before accessing the protected UART
             * characteristics. */
            int rc = ble_gap_security_initiate(event->connect.conn_handle);
            ESP_LOGI(TAG, "BLE security initiate rc=%d", rc);
        } else {
            advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_enabled = false;
        s_authenticated = false;
        meshcore_core_transport_set_connected(false);
        ESP_LOGI(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
        advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_tx_value_handle) {
            s_notify_enabled = event->subscribe.cur_notify != 0;
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE: {
        struct ble_gap_conn_desc description;
        const int find_rc = ble_gap_conn_find(event->enc_change.conn_handle, &description);
        if (find_rc == 0) {
            s_authenticated = event->enc_change.status == 0 &&
                              description.sec_state.encrypted &&
                              description.sec_state.authenticated;
            meshcore_core_transport_set_connected(s_authenticated);
            ESP_LOGI(TAG, "BLE security status=%d encrypted=%u authenticated=%u bonded=%u",
                     event->enc_change.status,
                     description.sec_state.encrypted,
                     description.sec_state.authenticated,
                     description.sec_state.bonded);
        } else {
            s_authenticated = false;
            meshcore_core_transport_set_connected(false);
            ESP_LOGW(TAG, "BLE security status=%d, conn lookup failed=%d",
                     event->enc_change.status, find_rc);
        }
        return 0;
    }

    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        struct ble_sm_io passkey = {0};
        passkey.action = event->passkey.params.action;
        ESP_LOGI(TAG, "BLE passkey action=%d", passkey.action);
        if (passkey.action == BLE_SM_IOACT_DISP) {
            passkey.passkey = meshcore_core_get_ble_pin();
        } else if (passkey.action == BLE_SM_IOACT_NUMCMP) {
            passkey.numcmp_accept = 1;
        } else {
            ESP_LOGW(TAG, "BLE passkey action %d is unsupported by DisplayOnly IO",
                     passkey.action);
            return 0;
        }
        int rc = ble_sm_inject_io(event->passkey.conn_handle, &passkey);
        ESP_LOGI(TAG, "Pairing PIN %06" PRIu32 ", inject rc=%d",
                 meshcore_core_get_ble_pin(), rc);
        return 0;
    }

    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc description;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &description) == 0) {
            ble_store_util_delete_peer(&description.peer_id_addr);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        return 0;

    default:
        return 0;
    }
}

static void advertise(void)
{
    if (!s_enabled) return;
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Advertising data failed: %d", rc);
        return;
    }

    /* The complete name and a 128-bit UUID do not fit in one 31-byte
     * advertising packet. Keep the UUID in the primary packet so MeshCore
     * scanners can filter by service, and put the full name in scan response. */
    struct ble_hs_adv_fields response = {0};
    response.name = (uint8_t *)ble_svc_gap_device_name();
    response.name_len = strlen((const char *)response.name);
    response.name_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&response);
    if (rc != 0) {
        ESP_LOGE(TAG, "Advertising scan response failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "Advertising payload configured: service UUID + name '%s' in scan response",
             ble_svc_gap_device_name());

    struct ble_gap_adv_params parameters = {0};
    parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;
    parameters.itvl_min = 0x20;
    parameters.itvl_max = 0x40;
    rc = ble_gap_adv_start(s_own_address_type, NULL, BLE_HS_FOREVER,
                           &parameters, gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Advertising start failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising active, rc=%d", rc);
    }
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &s_own_address_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE address setup failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "NimBLE host synchronized, address type=%u", s_own_address_type);
    if (s_enabled) advertise();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE reset: %d", reason);
}

static void host_task(void *parameter)
{
    (void)parameter;
    ESP_LOGI(TAG, "NimBLE host task started on CPU%d", xPortGetCoreID());
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void notify_task(void *parameter)
{
    (void)parameter;
    ESP_LOGI(TAG, "BLE transport task started on CPU%d", xPortGetCoreID());
    uint8_t frame[MESHCORE_FRAME_MAX_LEN];
    while (true) {
        if (s_enabled && s_connection_handle != BLE_HS_CONN_HANDLE_NONE &&
            s_authenticated && s_notify_enabled) {
            size_t length = meshcore_core_transport_pop_tx(frame, sizeof(frame));
            if (length > 0) {
                struct os_mbuf *packet = ble_hs_mbuf_from_flat(frame, length);
                if (packet != NULL) {
                    int rc = ble_gatts_notify_custom(s_connection_handle,
                                                     s_tx_value_handle, packet);
                    if (rc != 0) ESP_LOGW(TAG, "Notify failed: %d", rc);
                }
                vTaskDelay(pdMS_TO_TICKS(60));
                continue;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t meshcore_ble_start(void)
{
    if (s_started) return ESP_OK;
    if (!meshcore_core_is_running()) return ESP_ERR_INVALID_STATE;
    load_enabled_setting();

    ESP_LOGI(TAG, "BLE startup begin (ESP-Hosted VHCI), enabled=%u", s_enabled ? 1U : 0U);
    log_memory("before init");
    esp_err_t err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "NimBLE port initialized");
    log_memory("after init");

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_DISP_ONLY;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(s_services);
    if (rc == 0) rc = ble_gatts_add_svcs(s_services);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT service registration failed: %d", rc);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MeshCore GATT service registered");

    char device_name[48];
    snprintf(device_name, sizeof(device_name), "MeshCore-%s",
             meshcore_core_get_node_name());
    if (ble_svc_gap_device_name_set(device_name) != 0) return ESP_FAIL;
    ble_att_set_preferred_mtu(MESHCORE_FRAME_MAX_LEN + 3);
    ble_store_config_init();

    nimble_port_freertos_init(host_task);
    if (xTaskCreateWithCaps(notify_task, "meshcore_ble_tx", 4096, NULL, 6, NULL,
                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BLE transport task");
        log_memory("task failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "BLE transport task stack allocated in PSRAM");
    s_started = true;
    ESP_LOGI(TAG, "%s ready, PIN %06" PRIu32, device_name,
             meshcore_core_get_ble_pin());
    log_memory("ready");
    return ESP_OK;
}

esp_err_t meshcore_ble_set_enabled(bool enabled)
{
    load_enabled_setting();
    if (enabled == s_enabled) return ESP_OK;

    esp_err_t err = save_enabled_setting(enabled);
    if (err != ESP_OK) return err;
    s_enabled = enabled;

    if (!enabled) {
        meshcore_core_transport_set_connected(false);
        s_notify_enabled = false;
        s_authenticated = false;
        if (s_started && s_connection_handle != BLE_HS_CONN_HANDLE_NONE) {
            int rc = ble_gap_terminate(s_connection_handle, BLE_ERR_REM_USER_CONN_TERM);
            if (rc != 0) ESP_LOGW(TAG, "BLE disconnect while disabling failed: %d", rc);
        }
        s_connection_handle = BLE_HS_CONN_HANDLE_NONE;
        if (s_started) {
            int rc = ble_gap_adv_stop();
            if (rc != 0) ESP_LOGD(TAG, "BLE advertising was already stopped: %d", rc);
        }
        ESP_LOGI(TAG, "BLE disabled: advertising and transport stopped");
    } else {
        ESP_LOGI(TAG, "BLE enabled");
        if (s_started) advertise();
    }
    return ESP_OK;
}

bool meshcore_ble_is_enabled(void)
{
    load_enabled_setting();
    return s_enabled;
}

bool meshcore_ble_is_started(void)
{
    return s_started;
}

bool meshcore_ble_is_connected(void)
{
    return s_authenticated && s_connection_handle != BLE_HS_CONN_HANDLE_NONE;
}
