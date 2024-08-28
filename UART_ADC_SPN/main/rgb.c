#include "rgb.h"

void RGB_TIMER_INIT(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}


led_rgb_t RGB_CHANNEL_INIT_1(int GPIO_R, int GPIO_G, int GPIO_B){


    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .hpoint         = 0,

        .channel        = LEDC_CHANNEL_0,
        .gpio_num       = GPIO_R,
        .duty           = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .hpoint         = 0,

        .channel        = LEDC_CHANNEL_1,
        .gpio_num       = GPIO_G,
        .duty           = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    ledc_channel_config_t ledc_channel2 = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .hpoint         = 0,

        .channel        = LEDC_CHANNEL_2,
        .gpio_num       = GPIO_B,
        .duty           = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));

    led_rgb_t rgb_1;
    rgb_1.gpio_num_r = GPIO_R;
    rgb_1.channel_r = LEDC_CHANNEL_0;
    rgb_1.duty_r = 0;
    rgb_1.gpio_num_g = GPIO_G;
    rgb_1.channel_g = LEDC_CHANNEL_1;
    rgb_1.duty_g = 0;
    rgb_1.gpio_num_b = GPIO_B;
    rgb_1.channel_b = LEDC_CHANNEL_2;
    rgb_1.duty_b = 0;

    return rgb_1;

}


led_rgb_t RGB_CHANNEL_INIT_2(int GPIO_R, int GPIO_G, int GPIO_B){

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .hpoint         = 0,

        .channel        = LEDC_CHANNEL_3,
        .gpio_num       = GPIO_R,
        .duty           = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .hpoint         = 0,

        .channel        = LEDC_CHANNEL_4,
        .gpio_num       = GPIO_G,
        .duty           = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    ledc_channel_config_t ledc_channel2 = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .hpoint         = 0,

        .channel        = LEDC_CHANNEL_5,
        .gpio_num       = GPIO_B,
        .duty           = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));

    led_rgb_t rgb_2;
    rgb_2.gpio_num_r = GPIO_R;
    rgb_2.channel_r = LEDC_CHANNEL_3;
    rgb_2.duty_r = 0;
    rgb_2.gpio_num_g = GPIO_G;
    rgb_2.channel_g = LEDC_CHANNEL_4;
    rgb_2.duty_g = 0;
    rgb_2.gpio_num_b = GPIO_B;
    rgb_2.channel_b = LEDC_CHANNEL_5;
    rgb_2.duty_b = 0;

    return rgb_2;

}


void RGB_CHANGE(led_rgb_t rgb, int valor_r, int valor_g, int valor_b){

    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, rgb.channel_r, valor_r));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, rgb.channel_r));

    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, rgb.channel_g, valor_g));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, rgb.channel_g));

    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, rgb.channel_b, valor_b));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, rgb.channel_b));

}