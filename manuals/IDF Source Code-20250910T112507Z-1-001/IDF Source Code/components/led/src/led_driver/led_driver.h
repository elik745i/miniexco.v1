#ifndef _LED_DRIVER__H_
#define _LED_DRIVER__H_
#include "driver/gpio.h"

#define LED_PIN_NUM     GPIO_NUM_2
void led_init(void);
void led_open(void);
void led_close(void);
void led_task(void* param);
#endif