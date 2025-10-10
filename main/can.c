#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/gpio.h"


#define CAN_TX_PIN          6           // CAN TX GPIO pin (output of ESP32)
#define CAN_RX_PIN          5           // CAN RX GPIO pin (input to ESP32)
#define CAN_BITRATE         1000000     // 1Mbps
#define CAN_QUEUE_SIZE      16          // Transmit queue size
#define TRANSMIT_NOW        0           // No wait time for transmit

#define TERMINATION_GPIO    18           // GPIO pin for CAN bus termination resistor control


typedef enum {
    CAN_STATE_OK,
    CAN_STATE_ERROR,
    CAN_STATE_RECOVERING
} can_state_t;


static const char *TAG = "CAN";
static twai_node_handle_t can_handle = NULL;
static twai_node_status_t node_status;
static twai_node_record_t node_record;
static uint8_t state = 0;
static can_state_t e_can_state = CAN_STATE_OK;


static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);


// static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
// {
//     uint8_t recv_buff[8] = {0};
//     twai_frame_t rx_frame = {
//         .buffer = recv_buff,
//         .buffer_len = sizeof(recv_buff),
//     };
//     if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
//         ESP_LOGI(TAG, "Received CAN message with ID 0x%03X", rx_frame.header.id);
//     }
//     return false;
// }


void CAN_Init( void )
{
    ESP_LOGI( TAG, "Initializing CAN..." );

    // Enable termination resistor
    gpio_reset_pin( TERMINATION_GPIO );
    gpio_set_direction( TERMINATION_GPIO, GPIO_MODE_OUTPUT );
    gpio_set_level( TERMINATION_GPIO, 1 );   // Enable termination resistor if used

    // Create a new CAN controller driver instance
    twai_onchip_node_config_t node_config = {
            .io_cfg = {
                .tx = CAN_TX_PIN,                // TX GPIO pin
                .rx = CAN_RX_PIN,               // RX GPIO pin
                .quanta_clk_out = -1,
                .bus_off_indicator = -1,             
            },
            .bit_timing = {
                .bitrate = CAN_BITRATE,
            },  
            .tx_queue_depth = CAN_QUEUE_SIZE,            // Transmit queue depth set to 16
            .flags.enable_self_test = 0,                 // Disable self test mode
            .flags.enable_loopback = 0,                  // Disable loopback mode
    };
    
    if( ESP_OK == twai_new_node_onchip( &node_config, &can_handle ) )
    {
        ESP_LOGI( TAG, "CAN node created." );
    }
    else
    {
        ESP_LOGE( TAG, "Failed to create CAN node." );
    }

    // Register callback function for received messages
    // twai_event_callbacks_t user_cbs = {
    //     .on_rx_done = twai_rx_cb,
    // };

    // if( ESP_OK == twai_node_register_event_callbacks( can_handle, &user_cbs, NULL ) )
    // {
    //     ESP_LOGI( TAG, "Registered CAN RX callback." );
    // }
    // else
    // {
    //     ESP_LOGE( TAG, "Failed to register CAN RX callback." );
    // }

    // Enable the CAN node
    if( ESP_OK == twai_node_enable( can_handle ) )
    {
        ESP_LOGI( TAG, "CAN node enabled." );
    }
    else
    {
        ESP_LOGE( TAG, "Failed to enable CAN node." );
    }

    ESP_LOGI( TAG, "Initializing CAN... DONE");
}


void CAN_Transmit( uint32_t id, uint8_t* data, uint8_t len )
{

    twai_node_get_info( can_handle, &node_status, &node_record );

    if( TWAI_ERROR_BUS_OFF == node_status.state )
    {
        e_can_state = CAN_STATE_ERROR;
    }

    switch( e_can_state )
    {
        case CAN_STATE_OK:
            twai_frame_t tx_msg = {
                .header = {
                    .id = id,
                    .dlc = len,
                    .ide = 0,       // Standard frame
                    .rtr = 0,       // Data frame
                    .fdf = 0,       // Not FD frame
                    .brs = 0,       // No bit rate switch
                    .esi = 0,       // Not an error frame
                },
                .buffer = data,
                .buffer_len = len,
            };

            if ( ESP_OK != twai_node_transmit( can_handle, &tx_msg, TRANSMIT_NOW ) ) 
            {
                ESP_LOGE( TAG, "Failed to transmit CAN message with ID 0x%03X", tx_msg.header.id );
            }
            
            break;

        case CAN_STATE_ERROR:
            twai_node_recover( can_handle );
            e_can_state = CAN_STATE_RECOVERING;
            ESP_LOGW( TAG, "Recovering CAN bus..." );
            break;

        case CAN_STATE_RECOVERING:
            if( TWAI_ERROR_ACTIVE == node_status.state )
            {
                e_can_state = CAN_STATE_OK;
                ESP_LOGI( TAG, "CAN bus recovered." );
            }
            break;
    }
    
}