#include "esp_log.h"

#include "led.h"
#include "imu.h"
#include "can.h"
#include "tasks.h"


static const char *TAG = "MAIN";


void app_main( void )
{
    LED_Init();
    IMU_Init();

    TASKS_Init();
}
