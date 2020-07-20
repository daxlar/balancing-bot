#include <stdio.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_WRITE_CMD           0
#define I2C_READ_CMD            1
#define ACK_CHECK_EN            1   

#define DEFAULT_I2C_CLK_SPEED   400000

#define SMPLRT_DIV              0x19
#define CONFIG                  0x1A
#define GYRO_CONFIG             0x1B
#define	ACCEL_CONFIG	        0x1C
#define	ACCEL_XOUT_H	        0x3B	
#define	ACCEL_XOUT_L	        0x3C
#define	ACCEL_YOUT_H	        0x3D
#define	ACCEL_YOUT_L	        0x3E
#define	ACCEL_ZOUT_H	        0x3F
#define	ACCEL_ZOUT_L	        0x40
#define	TEMP_OUT_H		        0x41
#define	TEMP_OUT_L		        0x42
#define	GYRO_XOUT_H		        0x43
#define	GYRO_XOUT_L		        0x44	
#define	GYRO_YOUT_H		        0x45
#define	GYRO_YOUT_L		        0x46
#define	GYRO_ZOUT_H		        0x47
#define	GYRO_ZOUT_L		        0x48
#define	PWR_MGMT_1		        0x6B
#define MPU6050_I2C_ADDRESS     0x68
#define WHO_AM_I                0x75

void print_hello(){
    printf("hello from mpu6050 \n");
}


void i2c_init(){
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_21;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = GPIO_NUM_22;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = DEFAULT_I2C_CLK_SPEED;
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void i2c_master_read_register(uint8_t register_address, uint8_t* data, uint8_t data_len){
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, (MPU6050_I2C_ADDRESS << 1 | I2C_WRITE_CMD), ACK_CHECK_EN);
    i2c_master_write_byte(i2c_cmd_handle, register_address, ACK_CHECK_EN);
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

void i2c_master_write_register(uint8_t register_address, uint8_t data){
    i2c_cmd_handle_t i2c_cmd_handle = i2c_cmd_link_create();
    i2c_master_start(i2c_cmd_handle);
    i2c_master_write_byte(i2c_cmd_handle, MPU6050_I2C_ADDRESS << 1 | I2C_WRITE_CMD, ACK_CHECK_EN);
    i2c_master_write_byte(i2c_cmd_handle, register_address, ACK_CHECK_EN);
    i2c_master_write_byte(i2c_cmd_handle, data, ACK_CHECK_EN);
    i2c_master_stop(i2c_cmd_handle);
    i2c_master_cmd_begin(I2C_NUM_0, i2c_cmd_handle, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(i2c_cmd_handle);
}

uint8_t mpu6050_who_am_i(){
    uint8_t data = 0;
    uint8_t data_len = 1;
    i2c_master_read_register(WHO_AM_I, &data, data_len);
    return data;
}

void mpu6050_init(){
    // setting clock source to internal 8MHz oscillator
    i2c_master_write_register(PWR_MGMT_1, 0x00);
    // Setting sample rate to 1kHz/(1+SMPLRT_DIV) = 125Hz
    i2c_master_write_register(SMPLRT_DIV, 0x07);
    // Setting DLPF to 5Hz at 19ms delay
    i2c_master_write_register(CONFIG, 0x06);
    // setting FS_SEL to +-2000 deg/s
    i2c_master_write_register(GYRO_CONFIG, 0x18);
    // setting AFS_SEL to +- 4g
    i2c_master_write_register(ACCEL_CONFIG, 0x01);
}

float mpu6050_read_x_accl(){
    uint8_t data[2];
    uint8_t data_len = 2;
    i2c_master_read_register(ACCEL_XOUT_H, data, data_len);
    int16_t ret_val = (data[0] << 8) | data[1];
    return((ret_val/8192.0));
}

float mpu6050_read_y_accl(){
    uint8_t data[2];
    uint8_t data_len = 2;
    i2c_master_read_register(ACCEL_YOUT_H, data, data_len);
    int16_t ret_val = (data[0] << 8) | data[1];
    return(ret_val/8192.0);
}

float mpu6050_read_z_accl(){
    uint8_t data[2];
    uint8_t data_len = 2;
    i2c_master_read_register(ACCEL_ZOUT_H, data, data_len);
    int16_t ret_val = (data[0] << 8) | data[1];
    return(ret_val/8192.0);
}