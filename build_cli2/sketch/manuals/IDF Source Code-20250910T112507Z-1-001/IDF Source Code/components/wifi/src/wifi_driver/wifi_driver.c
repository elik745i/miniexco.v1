#line 1 "C:\\Users\\elik\\Documents\\Arduino\\MiniExco_v_2_00_78\\manuals\\IDF Source Code-20250910T112507Z-1-001\\IDF Source Code\\components\\wifi\\src\\wifi_driver\\wifi_driver.c"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_driver.h"
// #include "../../main/include/S3_config.h"

static char TAG[] ="WIFI message";

static esp_netif_t *netif_ap = NULL;

/*               AP        */
#define ESP_WIFI_CHANNEL_AP         1
#define MAX_STA_CONN_AP             5

#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1
#define WIFI_ESPTOUCH_DONE_BIT      BIT3
EventGroupHandle_t s_wifi_event_group;

typedef struct 
{
    char SSID[33];
    int8_t RSSI;
}wifi_scan_list_t;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if(event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID = %d",
                    MAC2STR(event->mac),event->aid);
        xEventGroupSetBits(s_wifi_event_group,WIFI_CONNECTED_BIT);
    }
    else if(event_id == WIFI_EVENT_AP_STADISCONNECTED){
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID = %d",
                    MAC2STR(event->mac),event->aid);
        xEventGroupClearBits(s_wifi_event_group,WIFI_CONNECTED_BIT);
    }
}
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    if(event_base == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* event = (ip_event_got_ip_t* ) event_data;
        ESP_LOGI(TAG,"got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group,WIFI_CONNECTED_BIT);
    }

}

void WIFI_Init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    netif_ap = esp_netif_create_default_wifi_ap();
    assert(netif_ap);
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID, &wifi_event_handler,NULL ));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,ESP_EVENT_ANY_ID, &ip_event_handler,NULL ));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));

}

void WIFI_Init_Softap(const char *APSSID,const char *APPASS)
{
    WIFI_Init();
    ESP_ERROR_CHECK(esp_wifi_start());
    wifi_config_t wifi_config = {
        .ap = {

            .ssid = "",
            .ssid_len = 0,
            .channel = ESP_WIFI_CHANNEL_AP,
            .password = "",
            .max_connection = MAX_STA_CONN_AP,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strlcpy((char*)wifi_config.ap.ssid,APSSID,sizeof(wifi_config.ap.ssid));
    if(strlen(APPASS)<8)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        ESP_LOGI(TAG,"password less than 8, The pattern is converted to auth open.");
    }
    else{
        strlcpy((char*)wifi_config.ap.password,APPASS,sizeof(wifi_config.ap.password));
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s PASSWORD:%s CHANNEL:%d",
            APSSID, APPASS, ESP_WIFI_CHANNEL_AP );

}
