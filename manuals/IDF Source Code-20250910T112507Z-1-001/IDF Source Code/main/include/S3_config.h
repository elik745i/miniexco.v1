#ifndef _S3_CONFIG__H_
#define _S3_CONFIG__H_

// // SPI1
// #define S3_SPI1_CLK_IO                    GPIO_NUM_4
// #define S3_SPI1_MISO_IO                   GPIO_NUM_13
// #define S3_SPI1_MOSI_IO                   GPIO_NUM_21

// // SD Card 
// #define S3_SD_CLK_IO                      S3_SPI1_CLK_IO
// #define S3_SD_DAT0_IO                     S3_SPI1_MISO_IO
// #define S3_SD_CMD_IO                      S3_SPI1_MOSI_IO
// #define S3_SD_CD_IO                       GPIO_NUM_19

// Camera
#define S3_CAMERA_RESET_IO                -1
#define S3_CAMERA_XCLK_IO                 GPIO_NUM_33
#define S3_CAMERA_SIOD_IO                 GPIO_NUM_37
#define S3_CAMERA_SIOC_IO                 GPIO_NUM_36
#define S3_CAMERA_D7_IO                   GPIO_NUM_47
#define S3_CAMERA_D6_IO                   GPIO_NUM_48
#define S3_CAMERA_D5_IO                   GPIO_NUM_42
#define S3_CAMERA_D4_IO                   GPIO_NUM_8
#define S3_CAMERA_D3_IO                   GPIO_NUM_6
#define S3_CAMERA_D2_IO                   GPIO_NUM_4
#define S3_CAMERA_D1_IO                   GPIO_NUM_5
#define S3_CAMERA_D0_IO                   GPIO_NUM_7
#define S3_CAMERA_VSYNC_IO                GPIO_NUM_35
#define S3_CAMERA_HREF_IO                 GPIO_NUM_34
#define S3_CAMERA_PCLK_IO                 GPIO_NUM_41

#endif