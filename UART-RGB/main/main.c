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

#define GPIO_LED_1_R           (13)
#define GPIO_LED_1_G           (12)
#define GPIO_LED_1_B           (14)

led_rgb_t led_rgb_1;


static const int RX_BUF_SIZE = 1024;

#define UART_USED (UART_NUM_0)
#define TXD_PIN   (GPIO_NUM_1)
#define RXD_PIN   (GPIO_NUM_3)



static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    while (1) {
        sendData(UART_USED, TX_TASK_TAG, "---\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_USED, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {


            ESP_LOGI(RX_TASK_TAG, "Recibido");
            data[rxBytes] = 0;
            
            if((data[0] == 'L') && (data[1] == 'E') && (data[2] == 'D')){
                
                if(data[3]=='R'){
                    RGB_CHANGE(led_rgb_1, atoi((char*)&data), 255, 255);
                }        
                if(data[3]=='G'){
                    RGB_CHANGE(led_rgb_1, 255, atoi((char*)&data), 255);
                }         
                if(data[3]=='B'){
                    RGB_CHANGE(led_rgb_1, 255, 255, atoi((char*)&data));
                }            

                //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);             
                 
            }
        }
    }
    free(data);
}


void app_main(void)
{
    RGB_TIMER_INIT();
    led_rgb_1 = RGB_CHANNEL_INIT_1(GPIO_LED_1_R, GPIO_LED_1_G, GPIO_LED_1_B);
        
    init_uart(UART_USED, RX_BUF_SIZE, TXD_PIN, RXD_PIN);
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    
    RGB_CHANGE(led_rgb_1, 255, 205, 200);
    while (1)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    
}
