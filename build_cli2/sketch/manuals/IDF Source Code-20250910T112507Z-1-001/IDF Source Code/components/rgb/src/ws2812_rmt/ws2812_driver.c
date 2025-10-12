#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\rgb\\src\\ws2812_rmt\\ws2812_driver.c"
#include "ws2812_driver.h"
#include "esp_check.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"

static const char *TAG = "ws2812";

static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];//一个灯有3颜色，所以用3个数组来存储一个灯的颜色
typedef struct {
    uint32_t resolution; /*!< Encoder resolution, in Hz */
} led_strip_encoder_config_t;

typedef struct {
    rmt_encoder_t base;//基础编码器
    rmt_encoder_t *bytes_encoder;//处理字节编码
    rmt_encoder_t *copy_encoder;//处理复制编码
    int state;//编码器的状态
    rmt_symbol_word_t reset_code;//复位码的RMT符号
} rmt_led_strip_encoder_t;

//发生数据或者复位
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);//指向结构体的起始地址
    rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    switch (led_encoder->state) {
    case 0: // send RGB data
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    // fall-through
    case 1: // send reset code
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                                sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);//删除编码器对象
    rmt_del_encoder(led_encoder->copy_encoder);//删除编码器对象
    free(led_encoder);
    return ESP_OK;
}

//复位编码器
static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t *led_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));//开辟空间
    ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for led strip encoder");
    led_encoder->base.encode = rmt_encode_led_strip;//发生数据或者复位
    led_encoder->base.del = rmt_del_led_strip_encoder;//删除编码器对象
    led_encoder->base.reset = rmt_led_strip_encoder_reset;//复位编码器
    // different led strip might have its own timing requirements, following parameter is for WS2812
    //发生0和1需要的时间，具体可以在这里修改，本次为sk6812mini，
    //0编码  TH0：0.2-0.4us  TL0:大于0.8      1编码  T1H: 0.6-1.0     T1L:大于0.2           
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        
        .bit0 = {
            .level0 = 1,
            .duration0 = 0.3 * config->resolution / 1000000, // T0H=0.3us
            .level1 = 0,
            .duration1 = 0.9 * config->resolution / 1000000, // T0L=0.9us
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 0.9 * config->resolution / 1000000, // T1H=0.9us
            .level1 = 0,
            .duration1 = 0.3 * config->resolution / 1000000, // T1L=0.3us
        },
        .flags.msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    uint32_t reset_ticks = config->resolution / 1000000 * 80 / 2; //  sk6812 reset code duration defaults to 80us
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = reset_ticks,
        .level1 = 0,
        .duration1 = reset_ticks,
    };
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
err:
    if (led_encoder) {
        if (led_encoder->bytes_encoder) {
            rmt_del_encoder(led_encoder->bytes_encoder);
        }
        if (led_encoder->copy_encoder) {
            rmt_del_encoder(led_encoder->copy_encoder);
        }
        free(led_encoder);
    }
    return ret;
}

void led_strip_hsv2rgb(uint32_t h, uint32_t *r, uint32_t *g, uint32_t *b)
{

    switch (h) {
    case 0:
        *r = 0xff;
        *g =0;
        *b = 0;
        break;
    case 1:
        *r = 0;
        *g = 0xff;
        *b = 0;
        break;
    case 2:
        *r = 0;
        *g = 0;
        *b = 0xff;
        break;
    case 3:
        *r = 0xff;
        *g = 0xff;
        *b = 0xff;
        break;
    }
}

void ws2812_task(void* param)
{
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;

    //创建RMT通道
    ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t led_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // 选择RMT外设时钟
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // 设置后台可挂起的事务数
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    //安装编码器
    ESP_LOGI(TAG, "Install led strip encoder");
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    //使能RMT通道
    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan));

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };
    while (1) {
        
        //实现红绿蓝，全部灯的电亮
        for(uint16_t hue = 0;hue<4;hue++)
        {
            for (int j = 0; j < EXAMPLE_LED_NUMBERS; j += 1) {
                // Build RGB pixels
                led_strip_hsv2rgb(hue,&red, &green, &blue);
                led_strip_pixels[j * 3 + 0] = green;
                led_strip_pixels[j * 3 + 1] = red;
                led_strip_pixels[j * 3 + 2] = blue;
            }
            // Flush RGB values to LEDs
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));//等待完成
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));

            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));//熄灭
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
        }
        //实现红绿蓝，流水灯的电亮
        for(uint16_t hue = 0;hue<4;hue++)
        {
            for(int j = 0; j < EXAMPLE_LED_NUMBERS; j += 1)
            {
                led_strip_hsv2rgb(hue,&red, &green, &blue);
                led_strip_pixels[j * 3 + 0] = green;
                led_strip_pixels[j * 3 + 1] = red;
                led_strip_pixels[j * 3 + 2] = blue;

            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));//等待完成
            vTaskDelay(pdMS_TO_TICKS(50));

            }
        }
        for(uint16_t hue = 3;hue>=0;hue--)
        {
            for(int j = EXAMPLE_LED_NUMBERS; j >=0; j--)
            {
                led_strip_hsv2rgb(hue,&red, &green, &blue);
                led_strip_pixels[j * 3 + 0] = green;
                led_strip_pixels[j * 3 + 1] = red;
                led_strip_pixels[j * 3 + 2] = blue;

            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));//等待完成
            vTaskDelay(pdMS_TO_TICKS(50));
            }
            memset(led_strip_pixels, 0, sizeof(led_strip_pixels));//熄灭
            ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
            vTaskDelay(pdMS_TO_TICKS(50));
            if(hue==0)
            {
                break;
            } 

        }

    // //使能RMT通道
    // ESP_LOGI(TAG, "Disable RMT TX channel");
    // ESP_ERROR_CHECK(rmt_disable(led_chan));

        //vTaskDelete(NULL);
    }
}



