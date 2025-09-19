#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\sdspi\\sdspi_driver.c"
#include "sdspi_driver.h"
#include <string.h>
// #include "esp_vfs_fat.h"
// #include "sdmmc_cmd.h"
#include "esp_err.h"

#define TAG  "sdspi"
sdmmc_host_t host = SDSPI_HOST_DEFAULT();//定义一个SD卡的SPI主机对象
sdmmc_card_t *card;
void sdspi_init(void)
{
    esp_err_t ret;
    //文件挂载配置
    esp_vfs_fat_mount_config_t mount_cfg  ={
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 8*1024,
    };
    
    ESP_LOGI(TAG, "Initializing SD card");
    //初始化spi总线
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot,&buscfg,SDSPI_DEFAULT_DMA);
       if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }
    //配置sd卡 spi设备
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;
    //挂载sd卡
    ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT,&host,&slot_config,&mount_cfg,&card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    //打印挂载信息
    sdmmc_card_print_info(stdout, card);
}