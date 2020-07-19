#include <stdio.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


void config_led_pin(){
    uint64_t pin_mask = 0x00000000;
    // select pin 2
    uint64_t pin2 = (pin_mask) | (1 << 2);
    gpio_config_t conf;
    conf.pin_bit_mask = pin2;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);
}



void app_main(){

    config_led_pin();
    const TickType_t xDelay1000ms = pdMS_TO_TICKS(1000);

    //gpio_reset_pin(2);
    //gpio_set_direction(2, GPIO_MODE_OUTPUT);


    while(1){
        vTaskDelay(xDelay1000ms);
        gpio_set_level(2, 0);
        vTaskDelay(xDelay1000ms);
        gpio_set_level(2, 1);
        
        
        
    }
}