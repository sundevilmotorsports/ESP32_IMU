#include "imu.h"
#include "can.h"

static const uint8_t imu_addresses[] = {0x6A, 0x6B};

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

static imu_raw_t imu_raw;
static imu_data_t imu_data;

static const char *TAG = "IMU";

static uint8_t imu_addr = 0;
static uint8_t imu_fail_count = 0;
static bool imu_initialized = false;

static esp_err_t Read_Register(uint8_t reg, uint8_t *buf, size_t len){
    return i2c_master_transmit_receive(dev_handle, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

static esp_err_t Write_Register(uint8_t reg, uint8_t data){
    uint8_t buf[2] = {reg, data};
    return i2c_master_transmit(dev_handle, buf, 2, I2C_TIMEOUT_MS);
}

static void I2C_Bus_Recover(void){
    ESP_LOGW(TAG, "Resetting I2C bus...");
    if (bus_handle) {
        i2c_del_master_bus(bus_handle);
        bus_handle = NULL;
    }
    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_0,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus_handle));
    ESP_LOGI(TAG, "I2C bus reset complete");
}

static esp_err_t IMU_Select_Address(void){
    for (int i = 0; i < 2; i++) {
        uint8_t addr = imu_addresses[i];
        if (i2c_master_probe(bus_handle, addr, 10) == ESP_OK) {
            imu_addr = addr;
            ESP_LOGI(TAG, "IMU found at 0x%02X", addr);
            return ESP_OK;
        }
    }
    return ESP_FAIL;
}

static void IMU_Handle_Failure(void){
    imu_fail_count++;

    ESP_LOGW(TAG, "IMU fail count: %d", imu_fail_count);
    if (imu_fail_count >= IMU_MAX_RETRY) {
        ESP_LOGE(TAG, "IMU unresponsive → resetting bus");

        imu_fail_count = 0;
        imu_initialized = false;

        I2C_Bus_Recover();
        IMU_Init();
    }
}

void IMU_Init(void){
    ESP_LOGI(TAG, "Initializing IMU...");

    // CS pin is tied to a GPIO, setting it high to force i2c
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IMU_CS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(IMU_CS_PIN, 1);

    imu_fail_count = 0;

    i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT_0,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus_handle));

    if (IMU_Select_Address() != ESP_OK) {
        ESP_LOGE(TAG, "IMU not found");

        I2C_Bus_Recover();

        if (IMU_Select_Address() != ESP_OK) {
            ESP_LOGE(TAG, "IMU still not found after recovery");
            return;
        }
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = imu_addr,
        .scl_speed_hz = I2C_HZ_1MHZ,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    Write_Register(REG_CTRL1_XL, REG_CTRL1_XL_INIT);
    Write_Register(REG_CTRL2_G, REG_CTRL2_G_INIT);

    imu_initialized = true;

    ESP_LOGI(TAG, "IMU init done (addr=0x%02X)", imu_addr);
}

void IMU_10ms(void){
    if (!imu_initialized) return;

    esp_err_t ret = Read_Register(REG_OUTX_L_G, (uint8_t*)&imu_raw, READ_BUFFER_SIZE);

    if (ret != ESP_OK) {
        IMU_Handle_Failure();
        return;
    }

    imu_fail_count = 0;

    imu_data.gyro_x = imu_raw.gyro_x * SENSITIVITY_GYRO_500DPS / 1000;
    imu_data.gyro_y = imu_raw.gyro_y * SENSITIVITY_GYRO_500DPS / 1000;
    imu_data.gyro_z = imu_raw.gyro_z * SENSITIVITY_GYRO_500DPS / 1000;

    imu_data.acc_x = imu_raw.acc_x * SENSITIVITY_ACC_4G / 1000;
    imu_data.acc_y = imu_raw.acc_y * SENSITIVITY_ACC_4G / 1000;
    imu_data.acc_z = imu_raw.acc_z * SENSITIVITY_ACC_4G / 1000;

    // Transmit raw data over CAN
    // TODO: test if endianness is correct
    CAN_Transmit( 0x360, ( uint8_t* ) &imu_raw.gyro_x, CAN_360_DLC );   // gyro
    vTaskDelay(10);
    CAN_Transmit( 0x361, ( uint8_t* ) &imu_raw.acc_x, CAN_361_DLC );    // accelerometer

    // Only for debugging
    // ESP_LOGI( TAG, "Acceleration X: %f", imu_data.acc_x );
    // ESP_LOGI( TAG, "Acceleration Y: %f", imu_data.acc_y );
    // ESP_LOGI( TAG, "Acceleration Z: %f", imu_data.acc_z );
    // ESP_LOGI( TAG, "Gyro X: %f", imu_data.gyro_x );
    // ESP_LOGI( TAG, "Gyro Y: %f", imu_data.gyro_y );
    // ESP_LOGI( TAG, "Gyro Z: %f", imu_data.gyro_z );
}