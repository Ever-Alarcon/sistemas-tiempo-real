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


///// Definiciones para el RGB 

#define GPIO_LED_1_R           (13)
#define GPIO_LED_1_G           (12)
#define GPIO_LED_1_B           (14)

led_rgb_t led_rgb_1;


////// Definiciones para el UART

#define UART_USED    (UART_NUM_0)
#define TXD_PIN      (GPIO_NUM_1)
#define RXD_PIN      (GPIO_NUM_3)

static const int RX_BUF_SIZE = 1024;



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

    static int valor_R = 255;
    static int valor_G = 255;
    static int valor_B = 255;

    while (1) {

        // rxBytes es -1 si hay error, si no el número indica la cantidad de bytes leidos  
        const int rxBytes = uart_read_bytes(UART_USED, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {

            // Agrega \0 al final de la cadana para que las operaciones de strings funcionen
            data[rxBytes] = 0;
           
            if((data[0] == 'L') && (data[1] == 'E') && (data[2] == 'D')){
                
                if (data[3] == 'R') {

                    /*
                        Explicación:
                            Cuando se le resta '0' a un número que es un caracter queda el valor del número
                    */
                    valor_R = (data[4] - '0') * 100 + (data[5] - '0') * 10 + (data[6] - '0');

                } else if (data[3] == 'G') {

                    /*
                        Explicación:
                            data        Puntero de un arreglo de uint8_t (uint8_t*)
                            data + 4    Se desplaza el puntero data 4 posiciones hacia adelante en la memoria
                                            Saltando los 4 primeros carácteres
                            (char*)     Pasa el puntero de uint8_t* a char*
                                            Asegura que la función atoi trate bien el dato
                            atoi        Convierte una cadena de caracteres (apuntada por el puntero) en un int.
                    */
                    valor_G = atoi((char*) (data + 4));
                    
                } else if (data[3] == 'B') {
                    valor_B = atoi((char*) (data + 4));
                }

                RGB_CHANGE(led_rgb_1, valor_R, valor_G, valor_B);   
        
            }
            
        }
    }
    free(data);
}


void app_main(void)
{
    RGB_TIMER_INIT();
    led_rgb_1 = RGB_CHANNEL_INIT_1(GPIO_LED_1_R, GPIO_LED_1_G, GPIO_LED_1_B);
    RGB_CHANGE(led_rgb_1, 256, 256, 256);
    /*    
    init_uart(UART_USED, RX_BUF_SIZE, TXD_PIN, RXD_PIN);
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    */

}
