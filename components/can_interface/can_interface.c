#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "control_loop.h"
#include "can_interface.h"


#define CAN_TX_GPIO                         (GPIO_NUM_20)
#define CAN_RX_GPIO                         (GPIO_NUM_21)

#define PROTOCOL_V_REF_PAYLOAD_BYTES        (2U)
#define PROTOCOL_LIVE_DATA_PAYLOAD_BYTES    (8U)
#define V_REF_MESSAGE_ID                    (0x01U)
#define LIVE_DATA_MESSAGE_ID                (0x10U)


static const char *tag = "CAN_INTERFACE";

/* ------------------ Private Functions ------------------ */
static void can_transmit_data(void * params);
static void can_receive_data(void * params);
static void build_live_data_message(uint8_t data[TWAI_FRAME_MAX_DLC]);
/* ------------------------------------------------------- */

/**
 * @brief Initialize can interface
 * 
 * @param None
 * 
 * @return None
 */
void can_interface_init(void)
{
    ESP_LOGI(tag, "--Initializing can interface--");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&g_config, &t_config, &f_config);

    twai_start();

    xTaskCreate(can_transmit_data, "can_transmit_data", 4056, NULL, 5, NULL);
    xTaskCreate(can_receive_data, "can_receive_data", 2048, NULL, 6, NULL);

    ESP_LOGI(tag, "--Can interface initialized--");
}

/**
 * @brief Transmit live data message every 10ms
 * 
 * @param params: Task params
 * 
 * @return None
 */
static void can_transmit_data(void * params)
{
    twai_message_t live_data_message = {
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = LIVE_DATA_MESSAGE_ID,
        .data_length_code = TWAI_FRAME_MAX_DLC,
        .data = {},
    };

    while (true) {
        build_live_data_message(live_data_message.data);

        twai_transmit(&live_data_message, pdMS_TO_TICKS(1));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Receive v_ref messages
 * 
 * @param params: Task params
 * 
 * @return None
 */
static void can_receive_data(void * params)
{
    twai_message_t msg_received = {};
    uint16_t new_freq_ref = 0;

    while (true) {
        
        if (twai_receive(&msg_received, portMAX_DELAY) == ESP_OK) {
            
            if (msg_received.identifier != V_REF_MESSAGE_ID) {
                continue;
            }

            if (msg_received.data_length_code != PROTOCOL_V_REF_PAYLOAD_BYTES) {
                ESP_LOGE(tag, "INVALID CAN MESSAGE LENGTH");
                continue;
            }

            new_freq_ref = msg_received.data[0] | ((uint16_t)msg_received.data[1] << 8U);

            set_freq_ref_rads(new_freq_ref);
        }
    }
}

/**
 * @brief Build live data message
 * 
 * @param data: Buffer to storage the live data message
 * 
 * @return None
 */
static void build_live_data_message(uint8_t data[TWAI_FRAME_MAX_DLC])
{
    uint16_t encoder_value = get_encoder_value_rads();
    uint16_t freq_ref = get_freq_ref_rads();

    data[0] = (uint8_t)(freq_ref & 0x00ff);
    data[1] = (uint8_t)((freq_ref & 0xff00) >> 8U);
    data[2] = (uint8_t)(encoder_value & 0x00ff);
    data[3] = (uint8_t)((encoder_value & 0xff00) >> 8U);
}