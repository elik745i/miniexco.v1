#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\build_cli2\\sketch\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\main\\include\\camera_config.h"
#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\main\\include\\camera_config.h"
#ifndef _CAMERA_CONFIG__H_
#define _CAMERA_CONFIG__H_

#include "esp_camera.h"
#include "S3_config.h"
#define CAM_PIN_PWDN        -1  //power down is not used
#define CAM_PIN_RESET       S3_CAMERA_RESET_IO //software reset will be performed
#define CAM_PIN_XCLK        S3_CAMERA_XCLK_IO
#define CAM_PIN_SIOD        S3_CAMERA_SIOD_IO
#define CAM_PIN_SIOC        S3_CAMERA_SIOC_IO

#define CAM_PIN_D7          S3_CAMERA_D7_IO
#define CAM_PIN_D6          S3_CAMERA_D6_IO
#define CAM_PIN_D5          S3_CAMERA_D5_IO
#define CAM_PIN_D4          S3_CAMERA_D4_IO
#define CAM_PIN_D3          S3_CAMERA_D3_IO
#define CAM_PIN_D2          S3_CAMERA_D2_IO
#define CAM_PIN_D1          S3_CAMERA_D1_IO
#define CAM_PIN_D0          S3_CAMERA_D0_IO
#define CAM_PIN_VSYNC       S3_CAMERA_VSYNC_IO
#define CAM_PIN_HREF        S3_CAMERA_HREF_IO
#define CAM_PIN_PCLK        S3_CAMERA_PCLK_IO

// 摄像头配置
static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 16MHz

    .xclk_freq_hz = 20000000, //2640
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_VGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 2       //if more than one, i2s runs in continuous mode. Use only with JPEG
};
#endif