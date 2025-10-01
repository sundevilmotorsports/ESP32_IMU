#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "led.h"
#include "imu.h"
#include "tasks.h"


#define PERIOD_10MS    10
#define PERIOD_500MS   500


static const char *TAG = "TASKS";

static void TASKS_10ms( void *pvParameters );
static void TASKS_500ms( void *pvParameters );


static void TASKS_10ms( void *pvParameters )
{
    const TickType_t xFrequency = pdMS_TO_TICKS( PERIOD_10MS );
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while( 1 )
    {
        IMU_10ms();

        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}


static void TASKS_500ms( void *pvParameters )
{
    const TickType_t xFrequency = pdMS_TO_TICKS( PERIOD_500MS );
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while( 1 )
    {
        LED_500ms();

        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}


void TASKS_Init( void )
{
    /*
        Create all the periodic tasks from highest priority to lowest priority
    */
    ESP_LOGI( TAG, "Initializing tasks..." );

    xTaskCreate( TASKS_10ms, "Task10ms", 2048, NULL, 1, NULL );
    xTaskCreate( TASKS_500ms, "Task500ms", 2048, NULL, 0, NULL );

    ESP_LOGI( TAG, "Initializing tasks... DONE" );
}