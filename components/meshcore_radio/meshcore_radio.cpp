#include "meshcore_radio.hpp"

#include <array>
#include <cinttypes>
#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

namespace {

// The SD card already owns SPI2 with a different pin set.
constexpr spi_host_device_t kSpiHost = SPI3_HOST;
/* Start conservatively; this also makes first-bring-up wiring diagnostics
 * reliable. The clock can be raised after the module answers consistently. */
constexpr int kSpiClockHz = 2000000;
constexpr size_t kSpiMaxTransfer = 512;
const char *kTag = "meshcore_radio";

class EspIdfRadioHal final : public RadioLibHal {
public:
    EspIdfRadioHal()
        : RadioLibHal(GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, 0, 1,
                      GPIO_INTR_POSEDGE, GPIO_INTR_NEGEDGE) {}

    void configure(const meshcore_radio_config_t &config) {
        config_ = config;
        last_error_ = ESP_OK;
        trace_count_ = 0;
    }

    void init() override {
        ESP_LOGI(kTag, "RadioLib HAL init");
        spiBegin();
    }

    void term() override {
        ESP_LOGI(kTag, "RadioLib HAL term");
        spiEnd();
    }

    void pinMode(uint32_t pin, uint32_t mode) override {
        if(pin == RADIOLIB_NC || pin >= SOC_GPIO_PIN_COUNT) return;
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << pin;
        cfg.mode = mode == GpioModeOutput ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        record_error(gpio_config(&cfg), "gpio_config");
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
        if(pin == RADIOLIB_NC || pin >= SOC_GPIO_PIN_COUNT) return;
        record_error(gpio_set_level(static_cast<gpio_num_t>(pin), value ? 1 : 0),
                     "gpio_set_level");
    }

    uint32_t digitalRead(uint32_t pin) override {
        if(pin == RADIOLIB_NC || pin >= SOC_GPIO_PIN_COUNT) return 0;
        return static_cast<uint32_t>(gpio_get_level(static_cast<gpio_num_t>(pin)));
    }

    void attachInterrupt(uint32_t interrupt_num, void (*callback)(void), uint32_t mode) override {
        if(interrupt_num == RADIOLIB_NC || interrupt_num >= SOC_GPIO_PIN_COUNT) return;
        ensure_isr_service();
        callbacks_[interrupt_num] = callback;
        gpio_num_t gpio = static_cast<gpio_num_t>(interrupt_num);
        record_error(gpio_set_intr_type(gpio, static_cast<gpio_int_type_t>(mode)),
                     "gpio_set_intr_type");
        void *isr_arg = reinterpret_cast<void *>(static_cast<uintptr_t>(interrupt_num));
        esp_err_t err = gpio_isr_handler_add(gpio, gpio_isr, isr_arg);
        if(err == ESP_ERR_INVALID_STATE) {
            gpio_isr_handler_remove(gpio);
            err = gpio_isr_handler_add(gpio, gpio_isr, isr_arg);
        }
        record_error(err, "gpio_isr_handler_add");
    }

    void detachInterrupt(uint32_t interrupt_num) override {
        if(interrupt_num == RADIOLIB_NC || interrupt_num >= SOC_GPIO_PIN_COUNT) return;
        gpio_num_t gpio = static_cast<gpio_num_t>(interrupt_num);
        gpio_intr_disable(gpio);
        gpio_isr_handler_remove(gpio);
        callbacks_[interrupt_num] = nullptr;
    }

    void delay(RadioLibTime_t milliseconds) override {
        if(milliseconds == 0) return;
        if(milliseconds < 10) {
            esp_rom_delay_us(static_cast<uint32_t>(milliseconds) * 1000U);
        } else {
            vTaskDelay(pdMS_TO_TICKS(milliseconds));
        }
    }

    void delayMicroseconds(RadioLibTime_t microseconds) override {
        esp_rom_delay_us(static_cast<uint32_t>(microseconds));
    }

    RadioLibTime_t millis() override {
        return static_cast<RadioLibTime_t>(esp_timer_get_time() / 1000ULL);
    }

    RadioLibTime_t micros() override {
        return static_cast<RadioLibTime_t>(esp_timer_get_time());
    }

    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override {
        const int64_t deadline = esp_timer_get_time() + timeout;
        while(digitalRead(pin) == state) {
            if(esp_timer_get_time() >= deadline) return 0;
        }
        while(digitalRead(pin) != state) {
            if(esp_timer_get_time() >= deadline) return 0;
        }
        const int64_t start = esp_timer_get_time();
        while(digitalRead(pin) == state) {
            if(esp_timer_get_time() >= deadline) return 0;
        }
        return static_cast<long>(esp_timer_get_time() - start);
    }

    void spiBegin() override {
        if(device_ != nullptr) return;

        ESP_LOGI(kTag, "SPI3 init: SCK=%d MOSI=%d MISO=%d NSS=%d clock=%d Hz",
                 config_.sck_gpio, config_.mosi_gpio, config_.miso_gpio,
                 config_.nss_gpio, kSpiClockHz);

        spi_bus_config_t bus = {};
        bus.mosi_io_num = config_.mosi_gpio;
        bus.miso_io_num = config_.miso_gpio;
        bus.sclk_io_num = config_.sck_gpio;
        bus.quadwp_io_num = -1;
        bus.quadhd_io_num = -1;
        bus.max_transfer_sz = kSpiMaxTransfer;

        esp_err_t err = spi_bus_initialize(kSpiHost, &bus, SPI_DMA_CH_AUTO);
        if(err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            record_error(err, "spi_bus_initialize");
            return;
        }

        spi_device_interface_config_t dev = {};
        dev.clock_speed_hz = kSpiClockHz;
        dev.mode = 0;
        dev.spics_io_num = -1;
        dev.queue_size = 1;
        record_error(spi_bus_add_device(kSpiHost, &dev, &device_), "spi_bus_add_device");
        if(device_ != nullptr) ESP_LOGI(kTag, "SPI3 device attached successfully");
    }

    void spiBeginTransaction() override {}

    void spiTransfer(uint8_t *out, size_t len, uint8_t *in) override {
        if(device_ == nullptr || len == 0) return;
        spi_transaction_t transaction = {};
        transaction.length = len * 8U;
        transaction.tx_buffer = out;
        transaction.rx_buffer = in;
        record_error(spi_device_polling_transmit(device_, &transaction),
                     "spi_device_polling_transmit");
        if(trace_count_ < 12) {
            char tx_text[32] = {};
            char rx_text[32] = {};
            const size_t shown = std::min<size_t>(len, 8);
            size_t tx_pos = 0;
            size_t rx_pos = 0;
            for(size_t i = 0; i < shown; ++i) {
                tx_pos += std::snprintf(tx_text + tx_pos, sizeof(tx_text) - tx_pos,
                                        "%02X%s", out ? out[i] : 0,
                                        i + 1 < shown ? " " : "");
                rx_pos += std::snprintf(rx_text + rx_pos, sizeof(rx_text) - rx_pos,
                                        "%02X%s", in ? in[i] : 0,
                                        i + 1 < shown ? " " : "");
            }
            ESP_LOGI(kTag, "SPI trace %u: len=%u TX=[%s] RX=[%s]",
                     static_cast<unsigned>(trace_count_ + 1),
                     static_cast<unsigned>(len), tx_text, rx_text);
            ++trace_count_;
        }
    }

    void spiEndTransaction() override {}

    void spiEnd() override {
        if(device_ != nullptr) {
            spi_bus_remove_device(device_);
            device_ = nullptr;
        }
    }

    esp_err_t last_error() const { return last_error_; }

private:
    static void gpio_isr(void *arg) {
        const uintptr_t pin = reinterpret_cast<uintptr_t>(arg);
        if(pin < callbacks_.size() && callbacks_[pin] != nullptr) callbacks_[pin]();
    }

    static void ensure_isr_service() {
        if(isr_service_ready_) return;
        const esp_err_t err = gpio_install_isr_service(0);
        if(err == ESP_OK || err == ESP_ERR_INVALID_STATE) isr_service_ready_ = true;
    }

    void record_error(esp_err_t err, const char *operation) {
        if(err == ESP_OK) return;
        last_error_ = err;
        ESP_LOGE(kTag, "%s failed: %s", operation, esp_err_to_name(err));
    }

    meshcore_radio_config_t config_ = meshcore_radio_default_config();
    spi_device_handle_t device_ = nullptr;
    esp_err_t last_error_ = ESP_OK;
    uint32_t trace_count_ = 0;
    static inline std::array<void (*)(void), SOC_GPIO_PIN_COUNT> callbacks_ = {};
    static inline bool isr_service_ready_ = false;
};

EspIdfRadioHal s_hal;
Module s_module(&s_hal, GPIO_NUM_46, GPIO_NUM_52, GPIO_NUM_51, GPIO_NUM_50);
SX1262 s_radio(&s_module);
bool s_ready = false;
int16_t s_last_status = RADIOLIB_ERR_UNKNOWN;

bool check_radio_status(int16_t status, const char *operation) {
    s_last_status = status;
    if(status == RADIOLIB_ERR_NONE) return true;
    ESP_LOGE(kTag, "%s failed: %d", operation, status);
    return false;
}

}  // namespace

meshcore_radio_config_t meshcore_radio_default_config(void) {
    return {
        .nss_gpio = 46,
        .mosi_gpio = 47,
        .miso_gpio = 48,
        .sck_gpio = 49,
        .busy_gpio = 50,
        .reset_gpio = 51,
        .dio1_gpio = 52,
        .frequency_mhz = 480.375f,
        .bandwidth_khz = 125.0f,
        .spreading_factor = 11,
        .coding_rate = 5,
        .tx_power_dbm = 1,
        .tcxo_voltage = 1.8f,
    };
}

esp_err_t meshcore_radio_init(const meshcore_radio_config_t *config) {
    if(s_ready) return ESP_OK;
    const meshcore_radio_config_t selected = config != nullptr ? *config : meshcore_radio_default_config();

    if(selected.nss_gpio != GPIO_NUM_46 || selected.busy_gpio != GPIO_NUM_50 ||
       selected.reset_gpio != GPIO_NUM_51 || selected.dio1_gpio != GPIO_NUM_52) {
        ESP_LOGE(kTag, "runtime control-pin remapping is not supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    s_hal.configure(selected);
    ESP_LOGI(kTag,
             "SX1262 begin: NSS=%d MOSI=%d MISO=%d SCK=%d BUSY=%d RESET=%d DIO1=%d "
             "freq=%.3f MHz TCXO=%.1f V",
             selected.nss_gpio, selected.mosi_gpio, selected.miso_gpio,
             selected.sck_gpio, selected.busy_gpio, selected.reset_gpio,
             selected.dio1_gpio, selected.frequency_mhz, selected.tcxo_voltage);
    ESP_LOGI(kTag, "GPIO before begin: NSS=%d BUSY=%d RESET=%d DIO1=%d internal=%u largest=%u",
             gpio_get_level(static_cast<gpio_num_t>(selected.nss_gpio)),
             gpio_get_level(static_cast<gpio_num_t>(selected.busy_gpio)),
             gpio_get_level(static_cast<gpio_num_t>(selected.reset_gpio)),
             gpio_get_level(static_cast<gpio_num_t>(selected.dio1_gpio)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));

    for(int attempt = 1; attempt <= 3; ++attempt) {
        ESP_LOGI(kTag, "SX1262 probe attempt %d/3", attempt);
        s_last_status = s_radio.begin(selected.frequency_mhz, selected.bandwidth_khz,
                                      selected.spreading_factor, selected.coding_rate,
                                      RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                                      selected.tx_power_dbm, 16, selected.tcxo_voltage);
        if(s_last_status == RADIOLIB_ERR_NONE) break;
        ESP_LOGW(kTag,
                 "SX1262 probe failed: RadioLib=%d HAL=%s NSS=%d BUSY=%d RESET=%d DIO1=%d",
                 s_last_status, esp_err_to_name(s_hal.last_error()),
                 gpio_get_level(static_cast<gpio_num_t>(selected.nss_gpio)),
                 gpio_get_level(static_cast<gpio_num_t>(selected.busy_gpio)),
                 gpio_get_level(static_cast<gpio_num_t>(selected.reset_gpio)),
                 gpio_get_level(static_cast<gpio_num_t>(selected.dio1_gpio)));
        if(attempt < 3) vTaskDelay(pdMS_TO_TICKS(250));
    }
    if(!check_radio_status(s_last_status, "SX1262 begin")) return ESP_FAIL;
    if(!check_radio_status(s_radio.setCRC(1), "set CRC")) return ESP_FAIL;
    if(!check_radio_status(s_radio.setCurrentLimit(140.0f), "set current limit")) return ESP_FAIL;
    if(!check_radio_status(s_radio.setDio2AsRfSwitch(true), "enable DIO2 RF switch")) return ESP_FAIL;
    if(!check_radio_status(s_radio.setRxBoostedGainMode(true), "enable RX boosted gain")) return ESP_FAIL;

    s_ready = true;
    ESP_LOGI(kTag, "SX1262 ready: %.3f MHz BW %.1f SF%u CR4/%u TX %d dBm",
             selected.frequency_mhz, selected.bandwidth_khz,
             selected.spreading_factor, selected.coding_rate, selected.tx_power_dbm);
    return ESP_OK;
}

bool meshcore_radio_is_ready(void) { return s_ready; }

int16_t meshcore_radio_last_status(void) { return s_last_status; }

namespace meshcore_radio {

SX1262 *radio() { return &s_radio; }

RadioLibHal *hal() { return &s_hal; }

}  // namespace meshcore_radio
