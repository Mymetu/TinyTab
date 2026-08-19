#include "meshcore_core.h"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <ctime>
#include <sys/time.h>

#include "Arduino.h"
#include "MyMesh.h"
#include "SPIFFS.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "esp_spiffs.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "helpers/ArduinoHelpers.h"
#include "meshcore_port_hooks.h"
#include "target.h"

namespace {

constexpr char kTag[] = "meshcore";
constexpr size_t kFrameQueueDepth = 8;
constexpr size_t kChatQueueDepth = 16;
constexpr size_t kCommandQueueDepth = 8;
constexpr uint32_t kMessageTimeStoreVersion = 1;
constexpr char kMessageTimeStoreNamespace[] = "mesh_msg_time";
constexpr char kMessageTimeStoreKey[] = "entries";
constexpr uint32_t kMeshQueueCaps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

struct HeardDeviceRecord {
    uint8_t public_key[PUB_KEY_SIZE];
    meshcore_device_info_t info;
};

struct TransportFrame {
    uint16_t length;
    uint8_t data[MESHCORE_FRAME_MAX_LEN];
};

struct PersistedMessageTime {
    uint8_t public_key[PUB_KEY_SIZE];
    uint32_t epoch;
    bool seen;
};

struct PersistedMessageTimeStore {
    uint32_t version;
    uint32_t count;
    PersistedMessageTime entries[MESHCORE_DEVICE_LIST_MAX];
};

struct CoreCommand {
    uint8_t kind;
    bool flood;
    meshcore_lora_config_t lora;
    char text[MESHCORE_CHAT_TEXT_MAX_LEN + 1];
};

enum : uint8_t {
    CORE_COMMAND_PUBLIC_MESSAGE = 0,
    CORE_COMMAND_APPLY_LORA = 1,
    CORE_COMMAND_SEND_ADVERT = 2,
};

QueueHandle_t s_transport_rx_queue;
QueueHandle_t s_transport_tx_queue;
QueueHandle_t s_chat_queue;
QueueHandle_t s_command_queue;
TaskHandle_t s_meshcore_task;
volatile bool s_running;
EXT_RAM_BSS_ATTR HeardDeviceRecord s_heard_devices[MESHCORE_DEVICE_LIST_MAX];
size_t s_heard_device_count;
uint32_t s_heard_device_generation;
portMUX_TYPE s_heard_device_lock = portMUX_INITIALIZER_UNLOCKED;
PersistedMessageTimeStore s_message_time_store = {};
bool s_message_time_store_loaded;

/* Keep the final byte sequence valid UTF-8 when a message exceeds the queue
 * buffer. MeshCore transports text as UTF-8 bytes and truncating in the middle
 * of a 2/3/4-byte character makes LVGL render a replacement glyph. */
size_t copy_utf8_truncated(char *destination, size_t destination_size, const char *source) {
    if(!destination || destination_size == 0) return 0;
    if(!source) {
        destination[0] = '\0';
        return 0;
    }

    const size_t limit = destination_size - 1;
    size_t copied = 0;
    while(copied < limit && source[copied] != '\0') {
        const uint8_t lead = static_cast<uint8_t>(source[copied]);
        size_t width = 1;
        if((lead & 0x80U) == 0) width = 1;
        else if((lead & 0xE0U) == 0xC0U) width = 2;
        else if((lead & 0xF0U) == 0xE0U) width = 3;
        else if((lead & 0xF8U) == 0xF0U) width = 4;

        if(width > 1) {
            if(copied + width > limit) break;
            bool valid = true;
            for(size_t i = 1; i < width; ++i) {
                const uint8_t continuation = static_cast<uint8_t>(source[copied + i]);
                if((continuation & 0xC0U) != 0x80U) {
                    valid = false;
                    break;
                }
            }
            if(!valid) width = 1;
        }
        if(copied + width > limit) break;
        std::memcpy(&destination[copied], &source[copied], width);
        copied += width;
    }
    destination[copied] = '\0';
    return copied;
}

void load_message_time_store() {
    nvs_handle_t handle;
    if(nvs_open(kMessageTimeStoreNamespace, NVS_READONLY, &handle) != ESP_OK) {
        s_message_time_store.version = kMessageTimeStoreVersion;
        s_message_time_store_loaded = true;
        return;
    }
    size_t size = sizeof(s_message_time_store);
    esp_err_t err = nvs_get_blob(handle, kMessageTimeStoreKey, &s_message_time_store, &size);
    nvs_close(handle);
    if(err != ESP_OK || size != sizeof(s_message_time_store) ||
       s_message_time_store.version != kMessageTimeStoreVersion ||
       s_message_time_store.count > MESHCORE_DEVICE_LIST_MAX) {
        s_message_time_store = {};
        s_message_time_store.version = kMessageTimeStoreVersion;
    }
    s_message_time_store_loaded = true;
    ESP_LOGI(kTag, "loaded %u persisted message timestamps",
             static_cast<unsigned>(s_message_time_store.count));
}

bool find_message_time(const uint8_t *public_key, uint32_t *epoch, bool *seen) {
    if(!s_message_time_store_loaded || !public_key) return false;
    for(uint32_t i = 0; i < s_message_time_store.count; ++i) {
        if(std::memcmp(s_message_time_store.entries[i].public_key, public_key, PUB_KEY_SIZE) == 0) {
            if(epoch) *epoch = s_message_time_store.entries[i].epoch;
            if(seen) *seen = s_message_time_store.entries[i].seen;
            return true;
        }
    }
    return false;
}

void save_message_time_store() {
    nvs_handle_t handle;
    if(nvs_open(kMessageTimeStoreNamespace, NVS_READWRITE, &handle) != ESP_OK) return;
    esp_err_t err = nvs_set_blob(handle, kMessageTimeStoreKey, &s_message_time_store,
                                 sizeof(s_message_time_store));
    if(err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if(err != ESP_OK) ESP_LOGW(kTag, "message timestamp save failed: %s", esp_err_to_name(err));
}

void update_message_time_store(const uint8_t *public_key, uint32_t epoch) {
    if(!s_message_time_store_loaded || !public_key) return;
    PersistedMessageTime *entry = nullptr;
    for(uint32_t i = 0; i < s_message_time_store.count; ++i) {
        if(std::memcmp(s_message_time_store.entries[i].public_key, public_key, PUB_KEY_SIZE) == 0) {
            entry = &s_message_time_store.entries[i];
            break;
        }
    }
    if(!entry && s_message_time_store.count < MESHCORE_DEVICE_LIST_MAX) {
        entry = &s_message_time_store.entries[s_message_time_store.count++];
        std::memset(entry, 0, sizeof(*entry));
        std::memcpy(entry->public_key, public_key, PUB_KEY_SIZE);
    }
    if(!entry) return;
    entry->epoch = epoch;
    entry->seen = true;
    save_message_time_store();
}

void upsert_device_record(const uint8_t *public_key, const char *name, uint8_t type,
                          bool route_known, uint8_t hop_count, bool metrics_valid,
                          int16_t rssi_dbm, int16_t snr_quarter_db,
                          uint32_t last_heard_ms) {
    if(!public_key || !name || name[0] == '\0') return;

    portENTER_CRITICAL(&s_heard_device_lock);
    size_t index = s_heard_device_count;
    bool existing = false;
    for(size_t i = 0; i < s_heard_device_count; ++i) {
        if(std::memcmp(s_heard_devices[i].public_key, public_key, PUB_KEY_SIZE) == 0) {
            index = i;
            existing = true;
            break;
        }
    }
    if(!existing) {
        if(s_heard_device_count < MESHCORE_DEVICE_LIST_MAX) {
            index = s_heard_device_count++;
        } else {
            index = 0;
            for(size_t i = 1; i < s_heard_device_count; ++i) {
                if(s_heard_devices[i].info.last_heard_ms <
                   s_heard_devices[index].info.last_heard_ms) {
                    index = i;
                }
            }
        }
        std::memcpy(s_heard_devices[index].public_key, public_key, PUB_KEY_SIZE);
        s_heard_devices[index].info = {};
        uint32_t persisted_epoch = 0;
        bool persisted_seen = false;
        if(find_message_time(public_key, &persisted_epoch, &persisted_seen) && persisted_seen) {
            s_heard_devices[index].info.message_seen = true;
            s_heard_devices[index].info.last_message_epoch = persisted_epoch;
        }
    }

    std::snprintf(s_heard_devices[index].info.name,
                  sizeof(s_heard_devices[index].info.name), "%s", name);
    s_heard_devices[index].info.type = type;
    s_heard_devices[index].info.route_known = route_known;
    s_heard_devices[index].info.hop_count = hop_count;
    if(metrics_valid || !existing) {
        s_heard_devices[index].info.metrics_valid = metrics_valid;
        s_heard_devices[index].info.rssi_dbm = metrics_valid ? rssi_dbm : 0;
        s_heard_devices[index].info.snr_quarter_db = metrics_valid ? snr_quarter_db : 0;
        s_heard_devices[index].info.last_heard_ms = metrics_valid ? last_heard_ms : 0;
    }
    ++s_heard_device_generation;
    portEXIT_CRITICAL(&s_heard_device_lock);
}

void remove_device_record(const uint8_t *public_key) {
    if(!public_key) return;
    portENTER_CRITICAL(&s_heard_device_lock);
    for(size_t i = 0; i < s_heard_device_count; ++i) {
        if(std::memcmp(s_heard_devices[i].public_key, public_key, PUB_KEY_SIZE) == 0) {
            for(size_t j = i + 1; j < s_heard_device_count; ++j) {
                s_heard_devices[j - 1] = s_heard_devices[j];
            }
            --s_heard_device_count;
            ++s_heard_device_generation;
            break;
        }
    }
    portEXIT_CRITICAL(&s_heard_device_lock);
}

void mark_device_message(const uint8_t *public_key) {
    if(!public_key) return;
    const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    const time_t wall_clock = time(nullptr);
    const uint32_t now_epoch = wall_clock > 0 ? static_cast<uint32_t>(wall_clock) : now_ms / 1000U;
    bool updated = false;
    portENTER_CRITICAL(&s_heard_device_lock);
    for(size_t i = 0; i < s_heard_device_count; ++i) {
        if(std::memcmp(s_heard_devices[i].public_key, public_key, PUB_KEY_SIZE) == 0) {
            s_heard_devices[i].info.message_seen = true;
            s_heard_devices[i].info.last_message_ms = now_ms;
            s_heard_devices[i].info.last_message_epoch = now_epoch;
            updated = true;
            ++s_heard_device_generation;
            break;
        }
    }
    portEXIT_CRITICAL(&s_heard_device_lock);
    if(updated) update_message_time_store(public_key, now_epoch);
}

class IdfBoard final : public mesh::MainBoard {
public:
    uint16_t getBattMilliVolts() override { return 0; }
    const char *getManufacturerName() const override { return "TinyTab P4"; }
    void reboot() override { esp_restart(); }
    uint32_t getIRQGpio() override { return 52; }
    uint8_t getStartupReason() const override { return BD_STARTUP_NORMAL; }
};

class IdfRtcClock final : public mesh::RTCClock {
public:
    uint32_t getCurrentTime() override {
        const time_t now = time(nullptr);
        if(now > 0) return static_cast<uint32_t>(now);
        /* Before GPS/NTP has supplied a wall clock, keep timestamps based
         * at the Unix epoch instead of inventing a modern date. */
        return millis() / 1000U;
    }

    void setCurrentTime(uint32_t timestamp) override {
        timeval value = {.tv_sec = static_cast<time_t>(timestamp), .tv_usec = 0};
        settimeofday(&value, nullptr);
    }
};

class IdfRng final : public mesh::RNG {
public:
    void random(uint8_t *destination, size_t size) override {
        esp_fill_random(destination, size);
    }
};

class QueueSerialInterface final : public BaseSerialInterface {
public:
    void enable() override { enabled_ = true; }
    void disable() override {
        enabled_ = false;
        connected_ = false;
    }
    bool isEnabled() const override { return enabled_; }
    bool isConnected() const override { return enabled_ && connected_; }
    bool isWriteBusy() const override {
        return s_transport_tx_queue && uxQueueSpacesAvailable(s_transport_tx_queue) == 0;
    }

    size_t writeFrame(const uint8_t source[], size_t length) override {
        if(!isConnected() || !source || length == 0 || length > MESHCORE_FRAME_MAX_LEN) return 0;
        TransportFrame frame = {};
        frame.length = static_cast<uint16_t>(length);
        std::memcpy(frame.data, source, length);
        return xQueueSend(s_transport_tx_queue, &frame, 0) == pdTRUE ? length : 0;
    }

    size_t checkRecvFrame(uint8_t destination[]) override {
        if(!enabled_ || !destination || !s_transport_rx_queue) return 0;
        TransportFrame frame = {};
        if(xQueueReceive(s_transport_rx_queue, &frame, 0) != pdTRUE) return 0;
        std::memcpy(destination, frame.data, frame.length);
        return frame.length;
    }

    void setConnected(bool connected) { connected_ = connected; }

private:
    volatile bool enabled_ = false;
    volatile bool connected_ = false;
};

IdfBoard s_board;
IdfRtcClock s_rtc_clock;
IdfRng s_identity_rng;
StdRNG s_fast_rng;
SimpleMeshTables s_tables;
DataStore s_store(SPIFFS, s_rtc_clock);
QueueSerialInterface s_serial_interface;

void delete_meshcore_queue(QueueHandle_t &queue) {
    if(queue != nullptr) {
        vQueueDeleteWithCaps(queue);
        queue = nullptr;
    }
}

void delete_meshcore_queues() {
    delete_meshcore_queue(s_transport_rx_queue);
    delete_meshcore_queue(s_transport_tx_queue);
    delete_meshcore_queue(s_chat_queue);
    delete_meshcore_queue(s_command_queue);
}

void log_memory(const char *stage) {
    constexpr uint32_t internal_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    ESP_LOGI(kTag, "MEM %-16s internal=%u largest=%u dma=%u psram=%u",
             stage,
             static_cast<unsigned>(heap_caps_get_free_size(internal_caps)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(internal_caps)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)));
}

bool mount_storage() {
    if(esp_spiffs_mounted("storage")) {
        ESP_LOGI(kTag, "SPIFFS storage already mounted");
        return true;
    }

    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "storage");
    if(!partition) {
        ESP_LOGE(kTag, "SPIFFS partition 'storage' was not found");
        return false;
    }
    ESP_LOGI(kTag, "SPIFFS mount begin: offset=0x%" PRIx32 " size=%" PRIu32
                   " bytes; a blank partition will be formatted once",
             partition->address, partition->size);
    const int64_t started_us = esp_timer_get_time();

    esp_vfs_spiffs_conf_t config = {};
    config.base_path = "/meshcore";
    config.partition_label = "storage";
    config.max_files = 10;
    config.format_if_mount_failed = true;
    const esp_err_t err = esp_vfs_spiffs_register(&config);
    if(err != ESP_OK) {
        ESP_LOGE(kTag, "SPIFFS mount/format failed after %lld ms: %s",
                 static_cast<long long>((esp_timer_get_time() - started_us) / 1000),
                 esp_err_to_name(err));
        return false;
    }

    size_t total = 0;
    size_t used = 0;
    const esp_err_t info_err = esp_spiffs_info("storage", &total, &used);
    ESP_LOGI(kTag, "SPIFFS ready in %lld ms: total=%u used=%u info=%s",
             static_cast<long long>((esp_timer_get_time() - started_us) / 1000),
             static_cast<unsigned>(total), static_cast<unsigned>(used),
             esp_err_to_name(info_err));
    return true;
}

void send_public_from_task(const char *text) {
    ChannelDetails channel;
    if(!the_mesh.getChannel(0, channel)) {
        ESP_LOGE(kTag, "Public channel is unavailable");
        return;
    }
    const uint32_t timestamp = s_rtc_clock.getCurrentTimeUnique();
    const int length = static_cast<int>(std::strlen(text));
    if(the_mesh.sendGroupMessage(timestamp, channel.channel, the_mesh.getNodeName(), text, length)) {
        meshcore_port_on_channel_message(timestamp, text, true);
    } else {
        ESP_LOGW(kTag, "Public message queue is full");
    }
}

void apply_lora_config_from_task(const meshcore_lora_config_t &config) {
    NodePrefs *prefs = the_mesh.getNodePrefs();
    prefs->freq = config.frequency_mhz;
    prefs->bw = config.bandwidth_khz;
    prefs->sf = config.spreading_factor;
    prefs->cr = config.coding_rate;
    prefs->tx_power_dbm = config.tx_power_dbm;
    prefs->rx_boosted_gain = config.rx_boosted_gain;
    prefs->client_repeat = config.client_repeat;
    prefs->path_hash_mode = config.path_hash_mode;
    prefs->autoadd_max_hops = config.autoadd_max_hops;
    prefs->autoadd_config = config.autoadd_config;
    prefs->advert_loc_policy = config.advert_loc_policy;
    the_mesh.savePrefs();
    radio_driver.setParams(prefs->freq, prefs->bw, prefs->sf, prefs->cr);
    radio_driver.setTxPower(prefs->tx_power_dbm);
    radio_driver.setRxBoostedGainMode(prefs->rx_boosted_gain != 0);
    ESP_LOGI(kTag, "LoRa config applied: freq=%.3f BW=%.1f SF%u CR4/%u TX=%d repeat=%u path=%u hops=%u",
             static_cast<double>(prefs->freq), static_cast<double>(prefs->bw),
             static_cast<unsigned>(prefs->sf), static_cast<unsigned>(prefs->cr),
             static_cast<int>(prefs->tx_power_dbm), static_cast<unsigned>(prefs->client_repeat),
             static_cast<unsigned>(prefs->path_hash_mode),
             static_cast<unsigned>(prefs->autoadd_max_hops));
}

void send_advert_from_task(bool flood) {
    NodePrefs *prefs = the_mesh.getNodePrefs();
    mesh::Packet *packet = prefs->advert_loc_policy == ADVERT_LOC_NONE
                               ? the_mesh.createSelfAdvert(prefs->node_name)
                               : the_mesh.createSelfAdvert(prefs->node_name,
                                                           sensors.node_lat, sensors.node_lon);
    if(!packet) {
        ESP_LOGW(kTag, "Unable to create self advert");
        return;
    }
    if(flood) {
        the_mesh.sendFloodAdvert(packet);
        ESP_LOGI(kTag, "Self advert sent using flood routing");
    } else {
        the_mesh.sendZeroHop(packet);
        ESP_LOGI(kTag, "Self advert sent using zero-hop routing");
    }
}

void meshcore_task(void *) {
    ESP_LOGI(kTag, "Core loop task started on CPU%d, stack free=%u",
             xPortGetCoreID(), static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)));
    CoreCommand command = {};
    int64_t next_health_log_us = esp_timer_get_time() + 60000000LL;
    while(true) {
        while(xQueueReceive(s_command_queue, &command, 0) == pdTRUE) {
            if(command.kind == CORE_COMMAND_APPLY_LORA) {
                apply_lora_config_from_task(command.lora);
            } else if(command.kind == CORE_COMMAND_SEND_ADVERT) {
                send_advert_from_task(command.flood);
            } else {
                send_public_from_task(command.text);
            }
        }
        the_mesh.loop();
        s_rtc_clock.tick();
        const int64_t now_us = esp_timer_get_time();
        if(now_us >= next_health_log_us) {
            ESP_LOGI(kTag, "Core healthy: stack free=%u RX=%u TX=%u RX errors=%u",
                     static_cast<unsigned>(uxTaskGetStackHighWaterMark(nullptr)),
                     static_cast<unsigned>(radio_driver.getPacketsRecv()),
                     static_cast<unsigned>(radio_driver.getPacketsSent()),
                     static_cast<unsigned>(radio_driver.getPacketsRecvErrors()));
            log_memory("core health");
            next_health_log_us = now_us + 60000000LL;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

}  // namespace

mesh::MainBoard &board = s_board;
SensorManager sensors;
IdfRadioAdapter radio_driver;
EXT_RAM_BSS_ATTR MyMesh the_mesh(radio_driver, s_fast_rng, s_rtc_clock,
                                 s_tables, s_store, nullptr);

IdfRadioAdapter *IdfRadioAdapter::instance_ = nullptr;

IdfRadioAdapter::IdfRadioAdapter()
    : radio_(nullptr), interrupt_ready_(false), state_(State::idle), rx_boosted_(true),
      spreading_factor_(11), packets_received_(0), packets_sent_(0), packet_receive_errors_(0) {
    instance_ = this;
}

void IdfRadioAdapter::bind(SX1262 *radio) { radio_ = radio; }

void IdfRadioAdapter::onRadioInterrupt() {
    if(instance_) instance_->interrupt_ready_ = true;
}

void IdfRadioAdapter::begin() {
    if(!radio_) return;
    radio_->setPacketReceivedAction(onRadioInterrupt);
    radio_->setPacketSentAction(onRadioInterrupt);
    radio_->setPreambleLength(spreading_factor_ <= 8 ? 32 : 16);
    state_ = State::idle;
    startReceive();
}

void IdfRadioAdapter::startReceive() {
    if(!radio_ || state_ == State::receive) return;
    interrupt_ready_ = false;
    if(radio_->startReceive() == RADIOLIB_ERR_NONE) state_ = State::receive;
}

int IdfRadioAdapter::recvRaw(uint8_t *bytes, int size) {
    int length = 0;
    if(radio_ && state_ == State::receive && interrupt_ready_) {
        interrupt_ready_ = false;
        length = std::min<int>(radio_->getPacketLength(), size);
        if(length <= 0 || radio_->readData(bytes, length) != RADIOLIB_ERR_NONE) {
            ++packet_receive_errors_;
            length = 0;
        } else {
            ++packets_received_;
        }
        state_ = State::idle;
    }
    startReceive();
    return length;
}

uint32_t IdfRadioAdapter::getEstAirtimeFor(int length) {
    return radio_ ? static_cast<uint32_t>(radio_->getTimeOnAir(length) / 1000U) : 0;
}

float IdfRadioAdapter::packetScore(float snr, int packet_length) {
    static constexpr float thresholds[] = {-7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f};
    if(spreading_factor_ < 7 || spreading_factor_ > 12) return 0.0f;
    const float threshold = thresholds[spreading_factor_ - 7];
    if(snr < threshold) return 0.0f;
    const float snr_score = (snr - threshold) / 10.0f;
    const float collision_penalty = 1.0f - packet_length / 256.0f;
    return std::clamp(snr_score * collision_penalty, 0.0f, 1.0f);
}

bool IdfRadioAdapter::startSendRaw(const uint8_t *bytes, int length) {
    if(!radio_ || !bytes || length <= 0) return false;
    radio_->standby();
    interrupt_ready_ = false;
    if(radio_->startTransmit(const_cast<uint8_t *>(bytes), length) != RADIOLIB_ERR_NONE) {
        state_ = State::idle;
        return false;
    }
    state_ = State::transmit;
    return true;
}

bool IdfRadioAdapter::isSendComplete() {
    if(state_ != State::transmit || !interrupt_ready_) return false;
    interrupt_ready_ = false;
    ++packets_sent_;
    return true;
}

void IdfRadioAdapter::onSendFinished() {
    if(radio_) radio_->finishTransmit();
    state_ = State::idle;
}

bool IdfRadioAdapter::isInRecvMode() const { return state_ == State::receive; }
bool IdfRadioAdapter::isReceiving() {
    if(!radio_ || state_ != State::receive) return false;
    const uint32_t irq = radio_->getIrqFlags();
    return (irq & (RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED |
                   RADIOLIB_SX126X_IRQ_HEADER_VALID)) != 0;
}
float IdfRadioAdapter::getLastRSSI() const { return radio_ ? radio_->getRSSI() : 0.0f; }
float IdfRadioAdapter::getLastSNR() const { return radio_ ? radio_->getSNR() : 0.0f; }
int IdfRadioAdapter::getNoiseFloor() const { return -120; }

void IdfRadioAdapter::setParams(float frequency, float bandwidth, uint8_t spreading_factor,
                                uint8_t coding_rate) {
    if(!radio_) return;
    radio_->standby();
    radio_->setFrequency(frequency);
    radio_->setBandwidth(bandwidth);
    radio_->setSpreadingFactor(spreading_factor);
    radio_->setCodingRate(coding_rate);
    spreading_factor_ = spreading_factor;
    radio_->setPreambleLength(spreading_factor_ <= 8 ? 32 : 16);
    state_ = State::idle;
}

void IdfRadioAdapter::setTxPower(int8_t dbm) {
    if(radio_) radio_->setOutputPower(dbm);
}

void IdfRadioAdapter::setRxBoostedGainMode(bool enabled) {
    if(radio_ && radio_->setRxBoostedGainMode(enabled) == RADIOLIB_ERR_NONE) rx_boosted_ = enabled;
}

bool IdfRadioAdapter::getRxBoostedGainMode() const { return rx_boosted_; }
uint32_t IdfRadioAdapter::getRngSeed() { return esp_random(); }

bool radio_init() {
    if(meshcore_radio_init(nullptr) != ESP_OK) return false;
    radio_driver.bind(meshcore_radio::radio());
    return true;
}

uint32_t radio_get_rng_seed() { return radio_driver.getRngSeed(); }

void radio_set_params(float frequency, float bandwidth, uint8_t spreading_factor,
                      uint8_t coding_rate) {
    radio_driver.setParams(frequency, bandwidth, spreading_factor, coding_rate);
}

void radio_set_tx_power(int8_t dbm) { radio_driver.setTxPower(dbm); }

mesh::LocalIdentity radio_new_identity() { return mesh::LocalIdentity(&s_identity_rng); }

extern "C" void meshcore_port_on_channel_message(uint32_t timestamp, const char *text,
                                                   bool is_local) {
    if(!s_chat_queue || !text) return;
    meshcore_chat_message_t message = {};
    message.is_local = is_local;
    message.timestamp = timestamp;
    copy_utf8_truncated(message.text, sizeof(message.text), text);
    if(xQueueSend(s_chat_queue, &message, 0) != pdTRUE) {
        meshcore_chat_message_t discarded;
        xQueueReceive(s_chat_queue, &discarded, 0);
        xQueueSend(s_chat_queue, &message, 0);
    }
}

extern "C" void meshcore_port_on_contact_advert(const uint8_t *public_key,
                                                  const char *name,
                                                  uint8_t type,
                                                  bool route_known,
                                                  uint8_t hop_count,
                                                  int16_t rssi_dbm,
                                                  int16_t snr_quarter_db) {
    const uint32_t last_heard_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    upsert_device_record(public_key, name, type, route_known, hop_count, true,
                         rssi_dbm, snr_quarter_db, last_heard_ms);

    if(route_known && hop_count == 0) {
        ESP_LOGI(kTag, "Device heard: %s direct RSSI=%d dBm SNR=%.2f dB", name,
                 static_cast<int>(rssi_dbm), static_cast<double>(snr_quarter_db) / 4.0);
    } else if(route_known) {
        ESP_LOGI(kTag, "Device heard: %s relayed hops=%u", name,
                 static_cast<unsigned>(hop_count));
    } else {
        ESP_LOGI(kTag, "Device heard: %s route unknown", name);
    }
}

extern "C" void meshcore_port_on_contact_loaded(const uint8_t *public_key,
                                                  const char *name,
                                                  uint8_t type,
                                                  bool route_known,
                                                  uint8_t hop_count) {
    upsert_device_record(public_key, name, type, route_known, hop_count,
                         false, 0, 0, 0);
    ESP_LOGI(kTag, "Contact loaded: %s route=%s hops=%u", name,
             route_known ? "known" : "unknown", static_cast<unsigned>(hop_count));
}

extern "C" void meshcore_port_on_contact_removed(const uint8_t *public_key) {
    remove_device_record(public_key);
}

extern "C" void meshcore_port_on_contact_message(const uint8_t *public_key) {
    mark_device_message(public_key);
}

esp_err_t meshcore_core_start(void) {
    if(s_running) {
        ESP_LOGI(kTag, "Core is already running");
        return ESP_OK;
    }

    ESP_LOGI(kTag, "Core startup begin");
    ESP_LOGI(kTag, "Mesh state: addr=%p size=%u external=%s",
             static_cast<void *>(&the_mesh), static_cast<unsigned>(sizeof(the_mesh)),
             esp_ptr_external_ram(&the_mesh) ? "yes" : "no");
    log_memory("startup begin");
    if(!mount_storage()) return ESP_FAIL;
    load_message_time_store();

    ESP_LOGI(kTag, "Initializing SX1262 radio");
    if(!radio_init()) {
        ESP_LOGE(kTag, "SX1262 initialization failed, RadioLib status=%d",
                 meshcore_radio_last_status());
        log_memory("radio failed");
        return ESP_FAIL;
    }

    ESP_LOGI(kTag, "Creating MeshCore queues");
    s_transport_rx_queue = xQueueCreateWithCaps(
        kFrameQueueDepth, sizeof(TransportFrame), kMeshQueueCaps);
    s_transport_tx_queue = xQueueCreateWithCaps(
        kFrameQueueDepth, sizeof(TransportFrame), kMeshQueueCaps);
    s_chat_queue = xQueueCreateWithCaps(
        kChatQueueDepth, sizeof(meshcore_chat_message_t), kMeshQueueCaps);
    s_command_queue = xQueueCreateWithCaps(
        kCommandQueueDepth, sizeof(CoreCommand), kMeshQueueCaps);
    if(!s_transport_rx_queue || !s_transport_tx_queue || !s_chat_queue || !s_command_queue) {
        ESP_LOGE(kTag, "Queue allocation failed: rx=%p tx=%p chat=%p command=%p",
                 s_transport_rx_queue, s_transport_tx_queue,
                 s_chat_queue, s_command_queue);
        log_memory("queue failed");
        delete_meshcore_queues();
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(kTag, "MeshCore queues allocated in PSRAM");

    ESP_LOGI(kTag, "Loading persistent MeshCore state");
    s_fast_rng.begin(radio_get_rng_seed());
    s_store.begin();
    ESP_LOGI(kTag, "Starting MeshCore protocol engine");
    the_mesh.begin(false);
    s_serial_interface.enable();
    the_mesh.startInterface(s_serial_interface);
    sensors.begin();

    /* MeshCore's protocol loop needs a large stack. The internal heap is
     * intentionally reserved for DMA/UI work, so allocate the task stack in
     * PSRAM while leaving the task control block in the internal heap. */
    if(xTaskCreateWithCaps(meshcore_task, "meshcore", 12288, nullptr, 7,
                           &s_meshcore_task,
                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(kTag, "Failed to create core loop task");
        log_memory("task failed");
        delete_meshcore_queues();
        return ESP_ERR_NO_MEM;
    }
    s_running = true;
    ESP_LOGI(kTag, "MeshCore started as %s on 480.375 MHz", the_mesh.getNodeName());
    log_memory("startup complete");
    return ESP_OK;
}

bool meshcore_core_is_running(void) { return s_running; }

esp_err_t meshcore_core_send_public(const char *text) {
    if(!s_running) return ESP_ERR_INVALID_STATE;
    if(!text || text[0] == '\0') return ESP_ERR_INVALID_ARG;
    CoreCommand command = {};
    command.kind = CORE_COMMAND_PUBLIC_MESSAGE;
    copy_utf8_truncated(command.text, sizeof(command.text), text);
    return xQueueSend(s_command_queue, &command, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t meshcore_core_get_lora_config(meshcore_lora_config_t *config) {
    if(!config || !meshcore_core_is_running()) return ESP_ERR_INVALID_STATE;
    const NodePrefs *prefs = the_mesh.getNodePrefs();
    config->frequency_mhz = prefs->freq;
    config->bandwidth_khz = prefs->bw;
    config->spreading_factor = prefs->sf;
    config->coding_rate = prefs->cr;
    config->tx_power_dbm = prefs->tx_power_dbm;
    config->rx_boosted_gain = prefs->rx_boosted_gain;
    config->client_repeat = prefs->client_repeat;
    config->path_hash_mode = prefs->path_hash_mode;
    config->autoadd_max_hops = prefs->autoadd_max_hops;
    config->autoadd_config = prefs->autoadd_config;
    config->advert_loc_policy = prefs->advert_loc_policy;
    return ESP_OK;
}

esp_err_t meshcore_core_set_lora_config(const meshcore_lora_config_t *config) {
    if(!config || !s_command_queue) return ESP_ERR_INVALID_STATE;
    if(config->frequency_mhz < 150.0f || config->frequency_mhz > 2500.0f ||
       config->bandwidth_khz < 7.8f || config->bandwidth_khz > 500.0f ||
       config->spreading_factor < 5 || config->spreading_factor > 12 ||
       config->coding_rate < 5 || config->coding_rate > 8 ||
       config->tx_power_dbm < -9 || config->tx_power_dbm > MAX_LORA_TX_POWER ||
       config->path_hash_mode > 2 || config->autoadd_max_hops > 64 ||
       config->advert_loc_policy > ADVERT_LOC_SHARE) {
        return ESP_ERR_INVALID_ARG;
    }
    CoreCommand command = {};
    command.kind = CORE_COMMAND_APPLY_LORA;
    command.lora = *config;
    return xQueueSend(s_command_queue, &command, pdMS_TO_TICKS(100)) == pdTRUE
               ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t meshcore_core_send_advert(bool flood) {
    if(!s_command_queue) return ESP_ERR_INVALID_STATE;
    CoreCommand command = {};
    command.kind = CORE_COMMAND_SEND_ADVERT;
    command.flood = flood;
    return xQueueSend(s_command_queue, &command, pdMS_TO_TICKS(100)) == pdTRUE
               ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool meshcore_core_pop_message(meshcore_chat_message_t *message) {
    return message && s_chat_queue && xQueueReceive(s_chat_queue, message, 0) == pdTRUE;
}

size_t meshcore_core_get_devices(meshcore_device_info_t *devices, size_t max_devices,
                                 uint32_t *generation) {
    portENTER_CRITICAL(&s_heard_device_lock);
    const size_t count = std::min(max_devices, s_heard_device_count);
    if(devices) {
        for(size_t i = 0; i < count; ++i) devices[i] = s_heard_devices[i].info;
    }
    if(generation) *generation = s_heard_device_generation;
    portEXIT_CRITICAL(&s_heard_device_lock);

    if(devices && count > 1) {
        std::sort(devices, devices + count,
                  [](const meshcore_device_info_t &left, const meshcore_device_info_t &right) {
                      return left.last_heard_ms > right.last_heard_ms;
                  });
    }
    return count;
}

esp_err_t meshcore_core_transport_push_rx(const uint8_t *data, size_t len) {
    if(!s_running || !data || len == 0 || len > MESHCORE_FRAME_MAX_LEN) return ESP_ERR_INVALID_ARG;
    TransportFrame frame = {};
    frame.length = static_cast<uint16_t>(len);
    std::memcpy(frame.data, data, len);
    return xQueueSend(s_transport_rx_queue, &frame, 0) == pdTRUE ? ESP_OK : ESP_ERR_NO_MEM;
}

size_t meshcore_core_transport_pop_tx(uint8_t *data, size_t max_len) {
    if(!data || !s_transport_tx_queue) return 0;
    TransportFrame frame = {};
    if(xQueuePeek(s_transport_tx_queue, &frame, 0) != pdTRUE || frame.length > max_len) return 0;
    xQueueReceive(s_transport_tx_queue, &frame, 0);
    std::memcpy(data, frame.data, frame.length);
    return frame.length;
}

void meshcore_core_transport_set_connected(bool connected) {
    s_serial_interface.setConnected(connected);
}

const char *meshcore_core_get_node_name(void) {
    return s_running ? the_mesh.getNodeName() : "TinyTab";
}

uint32_t meshcore_core_get_ble_pin(void) {
    return s_running ? the_mesh.getBLEPin() : 123456;
}
