#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define I2C_PORT_0              0
#define I2C_MASTER_SCL_PIN      11
#define I2C_MASTER_SDA_PIN      10

#define I2C_TIMEOUT_MS          10
#define READ_BUFFER_SIZE        12
#define I2C_HZ_400KHZ           400000
#define I2C_HZ_1MHZ             1000000

#define IMU_MAX_RETRY          5

#define REG_CTRL1_XL           0x10
#define REG_CTRL2_G            0x11
#define REG_OUTX_L_G           0x22

#define REG_CTRL1_XL_INIT      0x48
#define REG_CTRL2_G_INIT       0x44

#define SENSITIVITY_ACC_4G     0.122f
#define SENSITIVITY_GYRO_500DPS 17.5f

#define CAN_360_DLC            6
#define CAN_361_DLC            6

#define IMU_CS_PIN             GPIO_NUM_12

typedef struct {
    float gyro_x, gyro_y, gyro_z;
    float acc_x, acc_y, acc_z;
} imu_data_t;

typedef struct {
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t acc_x, acc_y, acc_z;
} imu_raw_t;

void IMU_Init( void );
void IMU_10ms( void );

#endif // IMU_H