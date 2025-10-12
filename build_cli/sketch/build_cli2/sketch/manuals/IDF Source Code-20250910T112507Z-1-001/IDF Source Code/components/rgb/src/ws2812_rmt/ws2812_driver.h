#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\build_cli2\\sketch\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\rgb\\src\\ws2812_rmt\\ws2812_driver.h"
#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\rgb\\src\\ws2812_rmt\\ws2812_driver.h"
#ifndef _WS2812_DRIVER__H_
#define _WS2812_DRIVER__H_

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      21

#define EXAMPLE_LED_NUMBERS         1
#define EXAMPLE_CHASE_SPEED_MS      1000


void ws2812_task(void * param);

#endif