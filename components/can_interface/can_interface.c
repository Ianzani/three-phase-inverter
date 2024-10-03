#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "can_interface.h"


#define CAN_TX_GPIO         (GPIO_NUM_17)
#define CAN_RX_GPIO         (GPIO_NUM_18)


static const char *tag = "CAN_INTERFACE";

/* ------------------ Private Functions ------------------ */
static void can_transmite_data(void * params);
/* ------------------------------------------------------- */

void can_interface_init(void)
{
    ESP_LOGI(tag, "--Initializing can interface--");

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NO_ACK);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&g_config, &t_config, &f_config);

    twai_start();

    xTaskCreate(can_transmite_data, "can_transmite_data", 2048, NULL, 5, NULL);

    ESP_LOGI(tag, "--Can interface initialized--");
}

static void can_transmite_data(void * params)
{
    twai_message_t test_message = {
        .extd = 0,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = 0x555,
        .data_length_code = 4,
        .data = {0, 1, 2, 3},
    };

    while (true) {
        twai_transmit(&test_message, pdMS_TO_TICKS(1));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
