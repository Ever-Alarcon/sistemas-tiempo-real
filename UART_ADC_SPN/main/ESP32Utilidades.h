#ifndef ESP32UTILIDADES_H
#define ESP32UTILIDADES_H

#include "esp_log.h" 
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/uart.h"
#include "string.h"


static const char *TAG0 = "EXAMPLE";

bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void example_adc_calibration_deinit(adc_cali_handle_t handle);
void adc_oneshot_read_multisample(adc_oneshot_unit_handle_t handle, adc_channel_t chan, int *out_raw, int samples);

void init_uart(uart_port_t uart_num, int rx_buffer_size, int tx_io_num, int rx_io_num);
int sendData(uart_port_t uart_num, const char* logName, const char* data);

#endif /* ESP32UTILIDADES_H */