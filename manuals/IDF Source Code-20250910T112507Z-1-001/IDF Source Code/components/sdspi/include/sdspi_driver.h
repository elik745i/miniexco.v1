#ifndef _SDSPI_DRIVER__H_
#define _SDSPI_DRIVER__H_
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#define EXAMPLE_MAX_CHAR_SIZE    64

#define MOUNT_POINT "/sdcard"//挂载点

#define PIN_NUM_MISO  GPIO_NUM_12
#define PIN_NUM_MOSI  GPIO_NUM_3
#define PIN_NUM_CLK   GPIO_NUM_11
#define PIN_NUM_CS    GPIO_NUM_2


void sdspi_init(void);
#endif