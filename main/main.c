#include "esp_log.h"

#include "led.h"
#include "imu.h"
#include "can.h"
#include "tasks.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG = "MAIN";


void app_main( void )
{
    LED_Init();
    IMU_Init();
    CAN_Init();

    TASKS_Init();
}
