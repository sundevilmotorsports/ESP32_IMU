#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "imu.h"
#include "can.h"
#include "freertos/FreeRTOS.h"


#define I2C_PORT_0                  0           // I2C port number
#define I2C_MASTER_SCL_PIN          11          // SCL GPIO pin number
#define I2C_MASTER_SDA_PIN          10          // SDA GPIO pin number
#define IMU_I2C_ADDR                0x6A        // IMU I2C address (datasheet)
#define I2C_TIMEOUT_MS              200         // I2C read/write timeout in milliseconds
#define READ_BUFFER_SIZE            12          // Number of bytes to read from the IMU
#define I2C_HZ_400KHZ               400000      // I2C 400kHz clock speed
#define I2C_HZ_1MHZ                 1000000     // I2C 1MHz clock speed

#define REG_CTRL1_XL                0x10        // Control register address for accelerometer
#define REG_CTRL2_G                 0x11        // Control register address for gyroscope
#define REG_OUTX_L_G                0x22        // Gyro X register address (start of all measurements)

#define REG_CTRL1_XL_INIT           0x48        // ODR = 104 Hz, FS = +/- 4g, 1st stage filter      
#define REG_CTRL2_G_INIT            0x44        // ODR = 104Hz, FS = 500 dps

#define SENSITIVITY_ACC_4G          0.122       // Accelerometer raw to physical sensitivity (in mg/LSB)
#define SENSITIVITY_GYRO_500DPS     17.5        // Gyroscope raw to physical sensitivity in (mdps/LSB)

#define CAN_360_DLC                 6           // Number of bytes to send in CAN message for ID 0x360 (gyro)
#define CAN_361_DLC                 6           // Number of bytes to send in CAN message for ID 0x361 (accelerometer)


typedef struct{ 
    float gyro_x;   // in degrees/s
    float gyro_y;   // in degrees/s
    float gyro_z;   // in degrees/s
    float acc_x;    // in m/s^2
    float acc_y;    // in m/s^2
    float acc_z;    // in m/s^2
} imu_data_t;

typedef struct
{
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z; 
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
} imu_raw_t;

 
static i2c_master_dev_handle_t dev_handle;
static imu_raw_t imu_raw;
static imu_data_t imu_data;
static const char *TAG = "IMU";


static void Read_Register( uint8_t register_addr, uint8_t* read_buf, size_t n_bytes );
static void Write_Register( uint8_t register_addr, uint8_t data );


static void Read_Register( uint8_t register_addr, uint8_t* read_buf, size_t n_bytes )
{
    /*
        Reads n_bytes starting from the address register_addr and stores them in read_buf.
    */
    i2c_master_transmit_receive( dev_handle, &register_addr,
                                 sizeof( register_addr ), read_buf, n_bytes, I2C_TIMEOUT_MS );

    // ESP_LOGI( TAG, "Reading Register 0x%02X: 0x%02X", register_addr, read_buf );
}

static void Write_Register( uint8_t register_addr, uint8_t data )
{
    /*
        Writes one byte to a register.
        The first byte is the register address, followed by the register value byte.
    */
    uint8_t write_buf[ 2 ] = { register_addr, data };
    i2c_master_transmit( dev_handle, write_buf, 2, I2C_TIMEOUT_MS );
}


void IMU_Init( void )
{
    /*
        Initialization function for the IMU (ISM330DHCX).
        It sets up the I2C bus and configures the IMU control registers.
    */
    ESP_LOGI( TAG, "Initializing IMU..." );

    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_0,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    if( ESP_OK == i2c_new_master_bus( &i2c_mst_config, &bus_handle ) )
    {
        ESP_LOGI( TAG, "I2C bus created." );
    }
    else
    {
        ESP_LOGE( TAG, "Failed to create I2C bus." );
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_I2C_ADDR,
        .scl_speed_hz = I2C_HZ_1MHZ,
    };

    if( ESP_OK == i2c_master_bus_add_device( bus_handle, &dev_cfg, &dev_handle ) )
    {
        ESP_LOGI( TAG, "IMU device added to I2C bus." );
    }
    else
    {
        ESP_LOGE( TAG, "Failed to add IMU device to I2C bus." );
    }

    Write_Register( REG_CTRL1_XL, REG_CTRL1_XL_INIT ); 
    Write_Register( REG_CTRL2_G, REG_CTRL2_G_INIT );

    ESP_LOGI( TAG, "Initializing IMU... DONE" );
}

void IMU_10ms( void )
{
    /*
        Reads the raw accelerometer and gyroscope values from the IMU, then converts them to physical float values.
        The values are stored in the imu_data struct.
    */

    // Read all registers at once
    Read_Register( REG_OUTX_L_G, ( uint8_t* ) &imu_raw, READ_BUFFER_SIZE );

    // Convert raw values to physical values
    imu_data.gyro_x = imu_raw.gyro_x * SENSITIVITY_GYRO_500DPS / 1000;
    imu_data.gyro_y = imu_raw.gyro_y * SENSITIVITY_GYRO_500DPS / 1000;
    imu_data.gyro_z = imu_raw.gyro_z * SENSITIVITY_GYRO_500DPS / 1000;
    imu_data.acc_x  = imu_raw.acc_x * SENSITIVITY_ACC_4G / 1000;
    imu_data.acc_y  = imu_raw.acc_y * SENSITIVITY_ACC_4G / 1000;
    imu_data.acc_z  = imu_raw.acc_z * SENSITIVITY_ACC_4G / 1000;

    // Transmit raw data over CAN
    // TODO: test if endianness is correct
    CAN_Transmit( 0x360, ( uint8_t* ) &imu_raw.gyro_x, CAN_360_DLC );   // gyro
    vTaskDelay(10);
    CAN_Transmit( 0x361, ( uint8_t* ) &imu_raw.acc_x, CAN_361_DLC );    // accelerometer

    // Only for debugging
    printf("\033[2J\033[H");
    ESP_LOGI( TAG, "Acceleration X: %f", imu_data.acc_x );
    ESP_LOGI( TAG, "Acceleration Y: %f", imu_data.acc_y );
    ESP_LOGI( TAG, "Acceleration Z: %f", imu_data.acc_z );
    ESP_LOGI( TAG, "Gyro X: %f", imu_data.gyro_x );
    ESP_LOGI( TAG, "Gyro Y: %f", imu_data.gyro_y );
    ESP_LOGI( TAG, "Gyro Z: %f", imu_data.gyro_z );
}