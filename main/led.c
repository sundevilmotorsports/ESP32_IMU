#include "driver/gpio.h"
#include "esp_log.h" 
#include "led.h"


#define LED_GPIO 47         // LED GPIO pin number


static const char *TAG = "LED";
static uint8_t led_state = 0;


void LED_Init( void )
{
    /*
        Initialize the LED GPIO pin.
    */
    ESP_LOGI( TAG, "Initializing LED..." );

    gpio_reset_pin( LED_GPIO );
    gpio_set_direction( LED_GPIO, GPIO_MODE_OUTPUT );

    ESP_LOGI( TAG, "Initializing LED... DONE" );
}

void LED_500ms( void )
{   
    /*
        Periodic callback to toggle the LED state every 500ms.
    */
    led_state = !led_state;
    gpio_set_level( LED_GPIO, led_state );
}