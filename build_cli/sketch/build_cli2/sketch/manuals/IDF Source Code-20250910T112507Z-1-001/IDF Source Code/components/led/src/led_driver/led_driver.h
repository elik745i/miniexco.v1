#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\build_cli2\\sketch\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\led\\src\\led_driver\\led_driver.h"
#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\led\\src\\led_driver\\led_driver.h"
#ifndef _LED_DRIVER__H_
#define _LED_DRIVER__H_
#include "driver/gpio.h"

#define LED_PIN_NUM     GPIO_NUM_2
void led_init(void);
void led_open(void);
void led_close(void);
void led_task(void* param);
#endif