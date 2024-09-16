#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/apps/sntp.h"


// Definiciones Modo STA (STAtion)
#define ESP_WIFI_SSID                          "Xx"
#define ESP_WIFI_PASS                          "Diego257b13"
#define ESP_MAXIMUM_RETRY_CONNECT               5
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SAE_MODE                       WPA3_SAE_PWE_BOTH
#define H2E_IDENTIFIER                          ""

// Definiciones Modo AP (Access Point)
#define ESP_WIFI_MY_SSID                        "AccessPoint_18"
#define ESP_WIFI_CHANNEL                        1
#define ESP_WIFI_MY_PASS                        "qwertyumm"
#define MAX_STA_CONN                            4


// Variables para el control del tiempo
TaskHandle_t  xHandle_task_time    = NULL;
float delay_in_seconds_task_time   = 2.5;

// Variables para el Control de Wifi
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static int s_retry_num     = 0;                


static const char *TAG = "wifi ";


static void NVS_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY_CONNECT) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    } else if (event_base == IP_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);

    } else if (event_base == IP_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init(void)
{
    ESP_LOGI(TAG, "ESP_WIFI_MODE_APSTA");
    s_wifi_event_group = xEventGroupCreate();

    // Inicialización del sistema de red e instancia de eventos
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    // Configuración de Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Manejadores de eventos
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));



    // Configuración de los modo STA
    wifi_config_t wifi_config_STA = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = H2E_IDENTIFIER,
        },
    };


    // Configuración de los modo AP
    wifi_config_t wifi_config_AP = {
        .ap = {
            .ssid = ESP_WIFI_MY_SSID,
            .ssid_len = strlen(ESP_WIFI_MY_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_MY_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                    .required = true,
            },
        },
    };
    if (strlen(ESP_WIFI_MY_PASS) == 0) {
        wifi_config_AP.ap.authmode = WIFI_AUTH_OPEN;
    }


    // Establecer el modo APSTA y configurar las interfaces Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_STA));
    ESP_ERROR_CHECK(esp_wifi_start());


    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             ESP_WIFI_MY_SSID, ESP_WIFI_MY_PASS, ESP_WIFI_CHANNEL);


    // Esperar si la conexión o fallo
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 ESP_WIFI_SSID, ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

}

void initialize_sntp(void) {
    ESP_LOGI(TAG, "Inicializando SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org"); // Servidor NTP
    sntp_init();
}

void obtain_time(void) {
    initialize_sntp();

    // Espera hasta obtener la hora
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;

    while(timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Esperando obtener la hora... (%d/%d)", retry, retry_count);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGE(TAG, "No se pudo obtener la hora actual.");
    } else {
        ESP_LOGI(TAG, "Hora actual obtenida correctamente.");

        setenv("TZ", "COT5", 1);                 // Cambia a zona horaria
        tzset();                                 // Aplica el ajuste de zona horaria
        localtime_r(&now, &timeinfo);            // Vuelve a obtener la hora local con la zona horaria ajustada

        ESP_LOGI(TAG, "La hora es: %s", asctime(&timeinfo));
    }
}


void task_time(void *arg)
{
    time_t now;
    struct tm timeinfo;
    /*
        Explicación Linea:
            arg:                     Es un puntero de tipo void * que recibe la tarea.
            (float *)arg:            Se castea a un puntero de tipo float. Le estamos diciendo al compilador que 
                                     trate el valor de arg como si fuera un puntero que apunta a una variable de 
                                     tipo float.
            *(float *)arg:           El operador * desreferencia el puntero convertido. Estamos accediendo al valor
                                     almacenado en la dirección de memoria apuntada por el puntero arg.
    */
    float delay_seconds = *((float *)arg);

    while(1){
        vTaskDelay((int)(delay_seconds * 1000) / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
        ESP_LOGI(TAG, "La fecha es: %s", asctime(&timeinfo));
    }

}



void app_main(void)
{
    NVS_init();

    esp_log_level_set("wifi",ESP_LOG_NONE); // Desahibita los log genericos de Wifi
    wifi_init();

    obtain_time();      // Inicializa y obtiene la hora actual

    xTaskCreate(task_time, "T_time", 1024, &delay_in_seconds_task_time, 0, &xHandle_task_time);
    configASSERT( xHandle_task_time );
}
