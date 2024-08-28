#ifndef RGB_H
#define RGB_H

#include "driver/ledc.h"

#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT // Set duty resolution to 8 bits
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

typedef struct {
    int gpio_num_r;                   /*!< the LEDC output gpio_num, if you want to use gpio16, gpio_num = 16 */
    ledc_channel_t channel_r;         /*!< LEDC channel (0 - LEDC_CHANNEL_MAX-1) */
    uint32_t duty_r;                  /*!< LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */

    int gpio_num_g;                   
    ledc_channel_t channel_g;         
    uint32_t duty_g;                  

    int gpio_num_b;                   
    ledc_channel_t channel_b;         
    uint32_t duty_b;                  
} led_rgb_t;

void RGB_TIMER_INIT(void);
led_rgb_t RGB_CHANNEL_INIT_1(int GPIO_R, int GPIO_G, int GPIO_B);
led_rgb_t RGB_CHANNEL_INIT_2(int GPIO_R, int GPIO_G, int GPIO_B);
void RGB_CHANGE(led_rgb_t rgb, int valor_r, int valor_g, int valor_b);

#endif /* RGB_H */