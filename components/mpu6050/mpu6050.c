#include <stdio.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define DEFAULT_I2C_CLK_SPEED   400000
#define MPU6050_I2C_ADDRESS     0x68
#define ACK_CHECK_EN            1
#define I2C_WRITE_CMD           0
#define I2C_READ_CMD            1   

void print_hello(){
    printf("hello from mpu6050 \n");
}


void i2c_init(){
    // configure driver
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = DEFAULT_I2C_CLK_SPEED;
    i2c_param_config(I2C_NUM_0, &conf);
    
    // install driver
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

}
void check_who_am_i(uint8_t* data, uint8_t data_len){
    // run comms
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (MPU6050_I2C_ADDRESS << 1 | I2C_WRITE_CMD), ACK_CHECK_EN);
    i2c_master_write_byte(i2c_cmd_handle, 0x75, ACK_CHECK_EN);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd_handle);

    i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (MPU6050_I2C_ADDRESS << 1 | I2C_READ_CMD), ACK_CHECK_EN);
    i2c_master_read(i2c_cmd_handle, data, data_len, I2C_MASTER_NACK);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd_handle);
}