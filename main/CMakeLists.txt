idf_component_register(SRCS "can.c" "tasks.c" "imu.c" "led.c" "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio
                       REQUIRES esp_driver_i2c
                       REQUIRES esp_driver_twai )