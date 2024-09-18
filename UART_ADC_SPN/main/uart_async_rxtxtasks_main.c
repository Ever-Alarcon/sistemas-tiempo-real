#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

#include "rgb.h"
#include "ESP32Utilidades.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>
#include <inttypes.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

///// Definiciones para el RGB 
const static char *TAG = "EXAMPLE";

#define GPIO_LED_1_R           (13)
#define GPIO_LED_1_G           (12)
#define GPIO_LED_1_B           (14)

led_rgb_t led_rgb_1;


////// Definiciones para el UART

#define UART_USED    (UART_NUM_0)
#define TXD_PIN      (GPIO_NUM_1)
#define RXD_PIN      (GPIO_NUM_3)

static const int RX_BUF_SIZE = 1024;

// Variables para los umbrales originales
int min_R = 22, max_R = 30;
int min_G = 15, max_G = 22;
int min_B = 0, max_B = 15;




/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
// ADC1 Channels
#if CONFIG_IDF_TARGET_ESP32
#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_6
#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_7
#define EXAMPLE_ADC_ATTEN ADC_ATTEN_DB_12
#else
#define EXAMPLE_ADC1_CHAN0 ADC_CHANNEL_2
#define EXAMPLE_ADC1_CHAN1 ADC_CHANNEL_3
#endif

// Parámetros de la ecuación de Steinhart-Hart
#define NTC_RESISTANCE 10000.0
#define REFERENCE_RESISTANCE 10000.0
#define ADC_REFERENCE_VOLTAGE 3300.0
#define BETA 3950
#define AMBIENT_TEMPERATURE 25

#define POT_MAX_RESISTANCE 10000.0

static int adc_raw[2][10];
static int voltage[2][10];

// Prototipos de funciones
static void init_adc(adc_oneshot_unit_handle_t *adc1_handle, adc_cali_handle_t *adc1_cali_chan0_handle, adc_cali_handle_t *adc1_cali_chan1_handle);
static void read_adc_temperature(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_chan0_handle, led_rgb_t *led_rgb_1);
static float adc_to_temperature(int adc_raw);
/*static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);*/


static float adc_to_temperature(int adc_raw) {
    // Convertir la lectura cruda del ADC a resistencia del NTC
    float voltage = adc_raw * (ADC_REFERENCE_VOLTAGE / 4095.0); // 4095 es el valor máximo de la lectura cruda del ADC
    float resistance_ntc = (REFERENCE_RESISTANCE * voltage) / (ADC_REFERENCE_VOLTAGE - voltage);

    // Aplicar la ecuación de Steinhart-Hart para calcular la temperatura en Kelvin
    float steinhart;
    steinhart = NTC_RESISTANCE / resistance_ntc; // Invertir (R0/R)
    steinhart = log(steinhart); // ln(R0/R)
    steinhart /= BETA; // 1/B * ln(R0/R)
    steinhart += 1.0 / (AMBIENT_TEMPERATURE + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; // Invertir nuevamente para obtener la temperatura
    steinhart -= 273.15; // Convertir de Kelvin a Celsius

    return steinhart;
}





// Tarea Encargada de la Transmición:
//
//  Envia unmensaje cada 4 segundos

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);

    while (1) {
        sendData(UART_USED, TX_TASK_TAG, "Env..\n");
        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
}


// Tarea encargada de Recepción

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);

    /*
        Explicación Linea:

            uint8_t     Entero sin signo de 8 bits (0 - 255).
            *           Indica que la variable es un puntero, almacena dirección de memoria.
            malloc      Asigna dinamicamente un bloque de memoria. 
                            Devuelve un puntero de la ubicación de ese bloque.
                            Si no lo logra devuleve NULL. 
            (uint8_t)   Conversión tipo casting.
                            Pasa el void* a uint8_t*
    
    */
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);

    /*static int valor_R = 255;
    static int valor_G = 255;
    static int valor_B = 255;*/

    while (1) {
        const int rxBytes = uart_read_bytes(UART_USED, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            
            if((data[0] == 'L') && (data[1] == 'E') && (data[2] == 'D')) {
                
                if (data[3] == 'R') {
                    if (strncmp((char*) (data + 4), "MIN", 3) == 0) {
                        min_R = atoi((char*) (data + 7));
                        RGB_CHANGE(led_rgb_1, 255, 250, 0);
                    } else if (strncmp((char*) (data + 4), "MAX", 3) == 0) {
                        max_R = atoi((char*) (data + 7));
                    }
                } else if (data[3] == 'G') {
                    if (strncmp((char*) (data + 4), "MIN", 3) == 0) {
                        min_G = atoi((char*) (data + 7));
                    } else if (strncmp((char*) (data + 4), "MAX", 3) == 0) {
                        max_G = atoi((char*) (data + 7));
                    }
                } else if (data[3] == 'B') {
                    if (strncmp((char*) (data + 4), "MIN", 3) == 0) {
                        min_B = atoi((char*) (data + 7));
                    } else if (strncmp((char*) (data + 4), "MAX", 3) == 0) {
                        max_B = atoi((char*) (data + 7));
                    }
                }

                ESP_LOGI(RX_TASK_TAG, "R MIN/MAX: %d/%d, G MIN/MAX: %d/%d, B MIN/MAX: %d/%d", 
                         min_R, max_R, min_G, max_G, min_B, max_B);
            }
        }
    }
    free(data);
}


void app_main(void)
{
    RGB_TIMER_INIT();
    led_rgb_1 = RGB_CHANNEL_INIT_1(GPIO_LED_1_R, GPIO_LED_1_G, GPIO_LED_1_B);
    RGB_CHANGE(led_rgb_1, 255, 150, 250);
        
    init_uart(UART_USED, RX_BUF_SIZE, TXD_PIN, RXD_PIN);
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);

    // Inicializar ADC y calibración
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    init_adc(&adc1_handle, &adc1_cali_chan0_handle, &adc1_cali_chan1_handle);
    // Bucle principal
    while (1) {
        read_adc_temperature(adc1_handle, adc1_cali_chan0_handle, &led_rgb_1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}







static void init_adc(adc_oneshot_unit_handle_t *adc1_handle, adc_cali_handle_t *adc1_cali_chan0_handle, adc_cali_handle_t *adc1_cali_chan1_handle) {
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc1_handle, EXAMPLE_ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc1_handle, EXAMPLE_ADC1_CHAN1, &config));

    *adc1_cali_chan0_handle = NULL;
    *adc1_cali_chan1_handle = NULL;
    example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, adc1_cali_chan0_handle);
    example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN1, EXAMPLE_ADC_ATTEN, adc1_cali_chan1_handle);
}

static void read_adc_temperature(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_chan0_handle, led_rgb_t *led_rgb_1) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
    
    if (adc1_cali_chan0_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
        ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, voltage[0][0]);
    }

    float temperature = adc_to_temperature(adc_raw[0][0]);
    ESP_LOGI(TAG, "Temperatura: %.2f °C", temperature);

    if (temperature > min_B && temperature < max_B) {
        RGB_CHANGE(*led_rgb_1, 225, 255, 0);
    } else if (temperature > min_G && temperature < max_G) {
        RGB_CHANGE(*led_rgb_1, 225, 0, 255);
    } else if (temperature > min_R && temperature < max_R){
        RGB_CHANGE(*led_rgb_1, 0, 225, 225);
    }
}
//azul --> temperatura más baja.  verde --> temperatura media .    rojo --> temperatura maxima