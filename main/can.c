#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"


#define CAN_TX_PIN          5
#define CAN_RX_PIN          6
#define CAN_BITRATE         1000000  //  1Mbps
#define CAN_QUEUE_SIZE      5
#define TRANSMIT_NOW        0       // No wait time for transmit


static const char *TAG = "CAN";
static twai_node_handle_t can_handle = NULL;
static twai_node_status_t can_status;
static twai_node_record_t can_record;


void CAN_Init( void )
{
    ESP_LOGI( TAG, "Initializing CAN..." );

    twai_onchip_node_config_t node_config = {
            .io_cfg.tx = CAN_TX_PIN,                // TX GPIO pin
            .io_cfg.rx = CAN_RX_PIN,                // RX GPIO pin
            .bit_timing.bitrate = CAN_BITRATE,      // bitrate (kbps)
            .tx_queue_depth = 5,                    // Transmit queue depth set to 5
    };
    
    // Create a new TWAI controller driver instance
    ESP_ERROR_CHECK( twai_new_node_onchip( &node_config, &can_handle ) );

    // Start the TWAI controller
    ESP_ERROR_CHECK( twai_node_enable( can_handle ) );

    ESP_LOGI( TAG, "Initializing CAN... DONE");
}


void CAN_Transmit( uint32_t id, uint8_t* data, uint8_t len )
{

    twai_node_get_info( can_handle, &can_status, &can_record );

    if( can_status.state != TWAI_ERROR_ACTIVE )
    {
        ESP_LOGW( TAG, "CAN bus error. State: %d", can_status.state ); 
        // TODO: check if we need to try to re-init CAN here
    }
    else
    {
        twai_frame_t tx_msg = {
            .header.id = id,
            .header.ide = false,
            .buffer = data,
            .buffer_len = ( size_t )len
        };
    
        twai_node_transmit( can_handle, &tx_msg, TRANSMIT_NOW );
    }
    
}