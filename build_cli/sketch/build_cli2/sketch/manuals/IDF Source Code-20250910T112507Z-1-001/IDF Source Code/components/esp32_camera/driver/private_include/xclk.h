#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\build_cli2\\sketch\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\esp32_camera\\driver\\private_include\\xclk.h"
#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\esp32_camera\\driver\\private_include\\xclk.h"
#pragma once

#include "esp_system.h"
#include "esp_camera.h"

esp_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz);

esp_err_t camera_enable_out_clock(const camera_config_t *config);

void camera_disable_out_clock(void);
