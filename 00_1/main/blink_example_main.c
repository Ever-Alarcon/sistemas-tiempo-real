#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

/* GPIO para el LED */
#define BLINK_GPIO 2

/* GPIO para el botón */
#define BUTTON_GPIO 18

static uint8_t s_led_state = 0;
#define ESP_INTR_FLAG_DEFAULT 0

static void blink_led(void)
{
    /* Encender el LED */
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Esperar 500 ms
    /* Apagar el LED */
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);  // Esperar 500 ms
}

static void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void configure_button(void)
{
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLUP_ONLY);
}

static void IRAM_ATTR button_isr_handler(void* arg)
{
    /* Cambiar el estado del LED cuando se presione el botón */
    s_led_state = !s_led_state;
    if (s_led_state) {
        blink_led();
    }
}

void app_main(void)
{
    /* Configurar el LED */
    configure_led();
    /* Configurar el botón */
    configure_button();
    
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    
    gpio_config(&io_conf);

    /* Instalar el servicio de ISR para el botón */
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    /* Asignar el manejador de ISR para el botón */
    gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL);
    
    while (1) {
        
        if(gpio_get_level(BUTTON_GPIO)==0){
            /* Titilar el LED */
            blink_led();
            s_led_state = !s_led_state;
            /* Retardo para el titilado del LED */
            /*vTaskDelay(500/ portTICK_PERIOD_MS);*/
        }
        else {
            gpio_set_level(BLINK_GPIO, 0);
            s_led_state= !s_led_state;
        }
    }
}