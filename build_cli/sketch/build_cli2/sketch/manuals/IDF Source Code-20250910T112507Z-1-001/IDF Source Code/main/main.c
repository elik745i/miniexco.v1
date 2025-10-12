#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\build_cli2\\sketch\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\main\\main.c"
#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\main\\main.c"
#include <stdio.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "camera_http.h"
#include "camera_config.h"
#include "../components/led/src/led_driver/led_driver.h"
#include "../components/rgb/src/ws2812_rmt/ws2812_driver.h"

#include "../components/wifi/src/wifi_driver/wifi_driver.h"
#include "ns4168.h"
#include "esp_err.h"
#include "../components/sdspi/include/sdspi_driver.h"
#include "driver/i2s_std.h"
#include "format_wav.h"
#include "../components/mems261/src/mems/mems_re.h"

#define WIFI_MODE       1
#define WIFI_AP_SSID    "ESP32-S3"
#define WIFI_AP_PASS    "12345678"
static const char *TAG = "main";


#define REBOOT_WAIT 5000            // reboot after 5 seconds
#define AUDIO_BUFFER 2048           // buffer size for reading the wav file and sending to i2s
#define WAV_FILE "/sdcard/xtd.wav" // wav file to play

#define volume_factor 0.2 //调节音量


#define NUM_CHANNELS        (2) // For mono recording only!
#define BYTE_RATE           (44100 * (16 / 8))*NUM_CHANNELS
#define SAMPLE_SIZE         (16 * 1024)

static int16_t i2s_readraw_buff[SAMPLE_SIZE];
size_t bytes_read;
const int WAVE_HEADER_SIZE = 44;
#define gain    20  //增益


esp_err_t play_wav(char *fp)
{
  FILE *fh = fopen(fp, "rb");
  if (fh == NULL)
  {
    ESP_LOGE(TAG, "Failed to open file");
    return ESP_ERR_INVALID_ARG;
  }

  // skip the header...//跳过wav文件的开头
  fseek(fh, 44, SEEK_SET);

  // create a writer buffer
  int16_t *buf = calloc(AUDIO_BUFFER, sizeof(int16_t));
  size_t bytes_read = 0;
  size_t bytes_written = 0;

  //从文件中读取AUDIO_BUFFER个int16_t类型的数据，存储到buf缓冲区中,并返回实际成功读取的字节数
  bytes_read = fread(buf, sizeof(int16_t), AUDIO_BUFFER, fh);

  i2s_channel_enable(tx_handle);

  while (bytes_read > 0)
  {
    // adjust the volume of the buffer
    for (int i = 0; i < bytes_read; i++) {
      buf[i] *= volume_factor;
    }
    // write the buffer to the i2s
    i2s_channel_write(tx_handle, buf, bytes_read * sizeof(int16_t), &bytes_written, portMAX_DELAY);

    bytes_read = fread(buf, sizeof(int16_t), AUDIO_BUFFER, fh);
    ESP_LOGV(TAG, "Bytes read: %d", bytes_read);
  }

  i2s_channel_disable(tx_handle);
  free(buf);

  return ESP_OK;
}

void record_wav(uint32_t rec_time)
{
    // Use POSIX and C standard library functions to work with files.
    int flash_wr_size = 0;
    ESP_LOGI(TAG, "Opening file");

    uint32_t flash_rec_time = BYTE_RATE * rec_time;
    //文件头
    const wav_header_t wav_header =
    WAV_HEADER_PCM_DEFAULT(flash_rec_time, 16, 44100, 1);

    // First check if file exists before creating a new file.
    struct stat st;
    if (stat(MOUNT_POINT"/test11.wav", &st) == 0) {
        // Delete it if it exists
        unlink(MOUNT_POINT"/test11.wav");
    }

    // Create new WAV file
    FILE *f = fopen(MOUNT_POINT"/test11.wav", "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    // Write the header to the WAV file
    fwrite(&wav_header, sizeof(wav_header), 1, f);

    // Start recording
    while (flash_wr_size < flash_rec_time) {
        // Read the RAW samples from the microphone
        if (i2s_channel_read(rx_chan,i2s_readraw_buff, SAMPLE_SIZE, &bytes_read, 1000) == ESP_OK) {
        // Apply gain to the samples
        int sample_count = bytes_read / sizeof(int16_t);
        //printf("sample_count = %d\n",sample_count);
        for (int i = 0; i < sample_count; i++) {
            int16_t sample = ((int16_t *)i2s_readraw_buff)[i];
            // Apply gain adjustment
            sample = sample * gain;
            // Write the adjusted sample back to buffer
            ((int16_t *)i2s_readraw_buff)[i] = sample;
        }

        // Write the adjusted samples to the WAV file
        fwrite(i2s_readraw_buff, bytes_read, 1, f);
        flash_wr_size += bytes_read;
        } else {
            printf("Read Failed!\n");
        }
    }

    ESP_LOGI(TAG, "Recording done!");
    fclose(f);
    ESP_LOGI(TAG, "File written on SDCard");

}

void audio_play(void *param)
{
  i2s_init();
  // play the wav file
  ESP_LOGI(TAG, "Playing wav file");
  // ESP_ERROR_CHECK(play_wav(WAV_FILE));

  i2s_play_music();
  // that'll do pig... that'll do
  i2s_del_channel(tx_handle); // delete the channel



  vTaskDelete(NULL);
}

void audio_reconder(void *param)
{

    sdspi_init();
    MEMS_Init();

    record_wav(10);
    // Stop I2S driver and destroy
    ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
    ESP_ERROR_CHECK(i2s_del_channel(rx_chan));

  vTaskDelete(NULL);
}
void app_main(void)
{

    esp_err_t ret;
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    #if WIFI_MODE == 1
    WIFI_Init_Softap(WIFI_AP_SSID,WIFI_AP_PASS);
    #endif
    // 初始化摄像头配置
    ret = esp_camera_init(&camera_config);
    if (ESP_OK != ret){
        ESP_LOGE(TAG, "Camera Init Failed");
        for(;;){
            vTaskDelay(10);
        }
    }

    ESP_LOGI(TAG, "Camera Init OK.");

    // 初始化http服务器
    http_server_init();

    // xTaskCreate(led_task,"led_task",1024*4,NULL,6,NULL);
  xTaskCreate(ws2812_task,"ws2812_task",1024*4,NULL,6,NULL);
  xTaskCreatePinnedToCore(audio_play,"audio_play",1024*4,NULL,6,NULL,0);
  xTaskCreatePinnedToCore(audio_reconder,"audio_reconder",1024*4,NULL,6,NULL,1);

}
