void print_hello();
void i2c_init();
void i2c_master_read_register(uint8_t register_address, uint8_t* data, uint8_t data_len);
uint8_t mpu6050_who_am_i();