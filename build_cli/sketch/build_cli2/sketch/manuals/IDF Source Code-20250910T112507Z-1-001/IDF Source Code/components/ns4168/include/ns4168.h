#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\build_cli2\\sketch\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\ns4168\\include\\ns4168.h"
#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\ns4168\\include\\ns4168.h"
#ifndef _NS4168__H_
#define _NS4168__H_
#include "driver/i2s_std.h"
#include "driver/gpio.h"
extern i2s_chan_handle_t tx_handle;
#define NS4168_BCLK     GPIO_NUM_10
#define NS4168_WS       GPIO_NUM_45
#define NS4168_DOUT     GPIO_NUM_9
#define NS4168_LRCLK    GPIO_NUM_46

// #define NS4168_BCLK     GPIO_NUM_2
// #define NS4168_WS       GPIO_NUM_47
// #define NS4168_DOUT     GPIO_NUM_16
// #define NS4168_LRCLK    GPIO_NUM_46

#define I2S_FRE     44100
#define I2S_NUM     I2S_NUM_0

void i2s_init(void);
void i2s_play_music(void);
void apply_gain_to_pcm(uint8_t *pcm_data, size_t pcm_len, float gain, uint8_t *out_data);
#endif
