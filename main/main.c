#include <stdio.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "mpu6050.h"


void onboard_led_pin_init(){
    uint64_t pin_mask = 0x00000000;
    uint64_t pin2 = (pin_mask) | (1 << 2);
    gpio_config_t conf;
    conf.pin_bit_mask = pin2;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);
}

void blinky(){
    const TickType_t xDelay1000ms = pdMS_TO_TICKS(100);
    vTaskDelay(xDelay1000ms);
    gpio_set_level(2, 0);
    vTaskDelay(xDelay1000ms);
    gpio_set_level(2, 1);
}


void app_main(){

    onboard_led_pin_init();
    i2c_init();
    mpu6050_init();

    if(mpu6050_who_am_i() != 0x68){
        printf("invalid reading, check configuration");
    }

    while(1){
        //blinky();
        printf("%4.3f, %4.3f, %4.3f \n", mpu6050_read_x_accl()
                                       , mpu6050_read_y_accl()
                                       , mpu6050_read_z_accl());                
    }
}