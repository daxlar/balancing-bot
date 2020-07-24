#include <stdio.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "mpu6050.h"

#include "driver/mcpwm.h"

#define SERVO_MIN_PULSEWIDTH 1000 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2000 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 90 //Maximum angle in degree upto which servo can rotate


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



void mcpwm0_gpio_init(){
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 18);
}

void mcpwm1_gpio_init(){
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, 19);
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}


void app_main(){

    onboard_led_pin_init();
    i2c_init();
    mpu6050_init();

    if(mpu6050_who_am_i() != 0x68){
        printf("invalid reading, check configuration");
    }
    /*
    while(1){
        //blinky();
        printf("%4.3f, %4.3f, %4.3f \n", mpu6050_read_x_accl()
                                       , mpu6050_read_y_accl()
                                       , mpu6050_read_z_accl());                
    }
    */


    mcpwm0_gpio_init();
    uint32_t angle, count;
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    while (1) {
        for (count = 0; count < SERVO_MAX_DEGREE; count++) {
            printf("Angle of rotation: %d\n", count);
            angle = servo_per_degree_init(count);
            printf("pulse width: %dus\n", angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            vTaskDelay(10);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
    }
}