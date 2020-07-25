#include <stdio.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "mpu6050.h"

#include "driver/mcpwm.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "soc/ledc_reg.h"

#define SERVO_MIN_PULSEWIDTH 500 //Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 //Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 //Maximum angle in degree upto which servo can rotate
#define SERVO_PERIOD 20000.0
#define MAX_TIMER 32767


#define SERVO_PERIOD 20000.0
#define MAX_TIMER 32767

//onst static float SERVO_MAX_DUTY = (float)SERVO_MAX_PULSEWIDTH/SERVO_PERIOD;
const static float SERVO_MIN_DUTY = (float)SERVO_MIN_PULSEWIDTH/SERVO_PERIOD;


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

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

uint32_t calculate_duty(uint32_t angle)
{
    float duty = SERVO_MIN_DUTY + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (angle)) / (SERVO_MAX_DEGREE))/SERVO_PERIOD;
    
    duty = (float)MAX_TIMER*duty;
    uint32_t duty_int = (uint32_t)duty;
    
    return duty_int;
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
    const TickType_t xDelay1000ms = pdMS_TO_TICKS(10);
    int32_t angle, count;
    
    // not really sure why this line is needed
    periph_module_enable(PERIPH_LEDC_MODULE);

    ledc_timer_config_t ledc_timer;
    ledc_timer.duty_resolution = LEDC_TIMER_15_BIT;
    ledc_timer.freq_hz = 50;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.clk_cfg = LEDC_USE_APB_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel;
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.duty = 0;
    ledc_channel.gpio_num = 18;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);

    
    ledc_timer_config_t ledc_timer1;
    ledc_timer1.duty_resolution = LEDC_TIMER_15_BIT;
    ledc_timer1.freq_hz = 50;
    ledc_timer1.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer1.timer_num = LEDC_TIMER_1;
    ledc_timer1.clk_cfg = LEDC_USE_APB_CLK;
    ledc_timer_config(&ledc_timer1);
    

    ledc_channel_config_t ledc_channel1;
    ledc_channel1.channel = LEDC_CHANNEL_1;
    ledc_channel1.duty = 0;
    ledc_channel1.gpio_num = 19;
    ledc_channel1.intr_type = LEDC_INTR_DISABLE;
    ledc_channel1.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel1.timer_sel = LEDC_TIMER_1;
    ledc_channel_config(&ledc_channel1);

    while (1) {
        for (count = 0; count < SERVO_MAX_DEGREE; count++) {
            
            //angle = servo_per_degree_init(count);
            /*
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, angle);
            */
            angle = calculate_duty(count);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, angle);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, angle);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

            vTaskDelay(xDelay1000ms);     //Add delay, since it takes time for servo to rotate, generally 100ms/60degree rotation at 5V
        }
    }
}