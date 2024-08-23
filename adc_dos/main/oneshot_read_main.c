/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <math.h>
#include <inttypes.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "rgb.h"

const static char *TAG = "EXAMPLE";

// Definiendo los pines de los Leds
#define GPIO_LED_1_R (13)
#define GPIO_LED_1_G (12)
#define GPIO_LED_1_B (14)
#define GPIO_LED_2_R (27)
#define GPIO_LED_2_G (26)
#define GPIO_LED_2_B (25)

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
static void init_leds(led_rgb_t *led_rgb_1, led_rgb_t *led_rgb_2);
static void init_adc(adc_oneshot_unit_handle_t *adc1_handle, adc_cali_handle_t *adc1_cali_chan0_handle, adc_cali_handle_t *adc1_cali_chan1_handle);
static void read_adc_temperature(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_chan0_handle, led_rgb_t *led_rgb_1);
static void read_adc_potentiometer(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_chan1_handle, led_rgb_t *led_rgb_2);
static float adc_to_temperature(int adc_raw);
static float pot_adc_to_voltage(int adc_raw);
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

void app_main(void) {
    // Inicializar LEDs
    led_rgb_t led_rgb_1, led_rgb_2;
    init_leds(&led_rgb_1, &led_rgb_2);

    // Inicializar ADC y calibración
    adc_oneshot_unit_handle_t adc1_handle;
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    adc_cali_handle_t adc1_cali_chan1_handle = NULL;
    init_adc(&adc1_handle, &adc1_cali_chan0_handle, &adc1_cali_chan1_handle);

    // Bucle principal
    while (1) {
        read_adc_temperature(adc1_handle, adc1_cali_chan0_handle, &led_rgb_1);
        read_adc_potentiometer(adc1_handle, adc1_cali_chan1_handle, &led_rgb_2);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Finalizar
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (adc1_cali_chan0_handle) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    if (adc1_cali_chan1_handle) {
        example_adc_calibration_deinit(adc1_cali_chan1_handle);
    }
}

// Implementación de la función
static float pot_adc_to_voltage(int adc_raw) {
    float pot_resistance = (adc_raw * POT_MAX_RESISTANCE) / 4095.0; // 4095 es el valor máximo de la lectura cruda del ADC
    float voltage = (pot_resistance / POT_MAX_RESISTANCE) * ADC_REFERENCE_VOLTAGE;
    return voltage;
}


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


static void init_leds(led_rgb_t *led_rgb_1, led_rgb_t *led_rgb_2) {
    RGB_TIMER_INIT();
    *led_rgb_1 = RGB_CHANNEL_INIT_1(GPIO_LED_1_R, GPIO_LED_1_G, GPIO_LED_1_B);
    *led_rgb_2 = RGB_CHANNEL_INIT_2(GPIO_LED_2_R, GPIO_LED_2_G, GPIO_LED_2_B);
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

    if (temperature > 0 && temperature < 20) {
        RGB_CHANGE(*led_rgb_1, 225, 0, 225);
    } else if (temperature > 20 && temperature < 30) {
        RGB_CHANGE(*led_rgb_1, 225, 225, 0);
    } else {
        RGB_CHANGE(*led_rgb_1, 0, 225, 225);
    }
}

static void read_adc_potentiometer(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_chan1_handle, led_rgb_t *led_rgb_2) {
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[0][1]));
    ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, adc_raw[0][1]);

    if (adc1_cali_chan1_handle) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
        ESP_LOGI(TAG, "Pot ADC Channel Cali Voltage: %d mV", voltage[0][1]);
    }

    float voltaje_pot = pot_adc_to_voltage(adc_raw[0][1]);
    ESP_LOGI(TAG, "Voltaje lineal: %.2f", voltaje_pot);

    if (voltaje_pot > 0 && voltaje_pot < 1100) {
        RGB_CHANGE(*led_rgb_2, 225, 0, 225);  // Color púrpura
    } else if (voltaje_pot >= 1100 && voltaje_pot < 2200) {
        RGB_CHANGE(*led_rgb_2, 225, 225, 0);  // Color amarillo
    } else {
        RGB_CHANGE(*led_rgb_2, 0, 225, 225);  // Color cian
    }
}



/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}