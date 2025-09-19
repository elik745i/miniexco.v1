#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\build_cli2\\sketch\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\led\\src\\led_driver\\led_driver.c"
#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\led\\src\\led_driver\\led_driver.c"
#include "led_driver.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
void led_init(void)
{
    gpio_config_t led_cfg={
        .intr_type = 0,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask =(1ULL<<LED_PIN_NUM) ,
        .pull_down_en = 0,
        .pull_up_en =0 
    };
    gpio_config(&led_cfg);

}
void led_open(void)
{
    gpio_set_level(LED_PIN_NUM,0);
}
void led_close(void)
{
    gpio_set_level(LED_PIN_NUM,1);
}

void led_task(void* param)
{
    led_init();
    while(1)
    {
        led_open();
        vTaskDelay(1000/portTICK_PERIOD_MS);
        led_close();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}