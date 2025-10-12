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