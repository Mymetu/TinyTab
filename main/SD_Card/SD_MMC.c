#include "SD_MMC.h"
#include "bsp/esp-bsp.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "sdmmc_cmd.h"
#include <stdbool.h>
#include <stdint.h>

#define MOUNT_POINT "/sdcard"

static const char *SD_TAG = "SD";

uint32_t SDCard_Size = 0;
static sdmmc_card_t *s_sdcard;
static sd_pwr_ctrl_handle_t s_sd_power;


void SD_Init(void)
{
    /* ESP-Hosted uses SDMMC slot 1 for the onboard ESP32-C6.  IDF 6 does not
     * allow the BSP mount helper to create another SD host controller, so use
     * the TF socket's SD pins in SPI mode on SPI2 instead. */
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 64 * 1024,
    };

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &s_sd_power);
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "failed to enable SD LDO4: %s", esp_err_to_name(ret));
        return;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.pwr_ctrl_handle = s_sd_power;
    /* Keep the board's previously stable SDSPI clock.  At 40 MHz this card
     * reports CMD52 CRC errors and fails CSD initialization. */
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    const spi_bus_config_t bus_config = {
        .mosi_io_num = BSP_SD_CMD,
        .miso_io_num = BSP_SD_D0,
        .sclk_io_num = BSP_SD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 16 * 1024,
    };
    ret = spi_bus_initialize(host.slot, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "failed to initialize SD SPI bus: %s", esp_err_to_name(ret));
        sd_pwr_ctrl_del_on_chip_ldo(s_sd_power);
        s_sd_power = NULL;
        return;
    }

    sdspi_device_config_t device_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    device_config.host_id = host.slot;
    device_config.gpio_cs = BSP_SD_D3;

    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &device_config,
                                  &mount_config, &s_sdcard);
    if (ret != ESP_OK) {
        SDCard_Size = 0;
        ESP_LOGW(SD_TAG, "no SD card mounted over SPI: %s", esp_err_to_name(ret));
        spi_bus_free(host.slot);
        sd_pwr_ctrl_del_on_chip_ldo(s_sd_power);
        s_sd_power = NULL;
        return;
    }

    const uint64_t card_bytes = (uint64_t)s_sdcard->csd.capacity *
                                s_sdcard->csd.sector_size;
    SDCard_Size = (uint32_t)(card_bytes / (1024 * 1024));
    ESP_LOGI(SD_TAG, "SD filesystem mounted at %s (%lu MB, SPI default-speed mode)",
             MOUNT_POINT, (unsigned long)SDCard_Size);
    sdmmc_card_print_info(stdout, s_sdcard);
}

bool SD_Get_Usage(uint64_t *total_bytes, uint64_t *used_bytes)
{
    if (total_bytes == NULL || used_bytes == NULL || s_sdcard == NULL) {
        return false;
    }

    uint64_t total = 0;
    uint64_t free = 0;
    if (esp_vfs_fat_info(MOUNT_POINT, &total, &free) != ESP_OK || total == 0) {
        return false;
    }

    *total_bytes = total;
    *used_bytes = total >= free ? total - free : 0;
    return true;
}
