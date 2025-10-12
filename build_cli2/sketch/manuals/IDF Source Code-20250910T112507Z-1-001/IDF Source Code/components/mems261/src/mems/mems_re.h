#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\mems261\\src\\mems\\mems_re.h"
#ifndef _MEMS_RE__H_
#define _MEMS_RE__H_
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#define EXAMPLE_STD_BCLK_IO1        GPIO_NUM_39      // I2S bit clock io number
#define EXAMPLE_STD_WS_IO1          GPIO_NUM_40     // I2S word select io number
#define EXAMPLE_STD_DOUT_IO1        -1               // I2S data out io number
#define EXAMPLE_STD_DIN_IO1         GPIO_NUM_38      // I2S data in io number
#define EXAMPLE_BUFF_SIZE           128


extern i2s_chan_handle_t  rx_chan;        // I2S rx channel handler

void MEMS_Init(void);
#endif