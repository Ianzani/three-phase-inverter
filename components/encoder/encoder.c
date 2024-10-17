#include <stdio.h>

#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "encoder.h"


#define PCNT_HIGH_LIMIT             (2048)
#define PCNT_LOW_LIMIT              (-2048)
#define GLITCH_FILTER_VALUE_NS      (500U)
#define CH_A_GPIO                   (17)        
#define CH_B_GPIO                   (18)


static const char* tag = "ENCODER";

static pcnt_unit_handle_t pcnt_unit = NULL;

/**
 * @brief Initialize the pulse counter in quadrature mode
 * 
 * @param None
 * 
 * @return None
 */
void encoder_init(void) {

    ESP_LOGI(tag, "--Initializing pulse counter--");
    pcnt_unit_config_t pcnt_unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    pcnt_new_unit(&pcnt_unit_config, &pcnt_unit);

    ESP_LOGI(tag, "--Configuring glitch filter--");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = GLITCH_FILTER_VALUE_NS,
    };
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    ESP_LOGI(tag, "--Configuring pulse counter gpios--");
    pcnt_chan_config_t ch_a_config = {
        .edge_gpio_num = CH_A_GPIO,
        .level_gpio_num = CH_B_GPIO,
    };
    pcnt_channel_handle_t pcnt_ch_a = NULL;
    pcnt_new_channel(pcnt_unit, &ch_a_config, &pcnt_ch_a);

    pcnt_chan_config_t ch_b_config = {
        .edge_gpio_num = CH_B_GPIO,
        .level_gpio_num = CH_A_GPIO,
    };
    pcnt_channel_handle_t pcnt_ch_b = NULL;
    pcnt_new_channel(pcnt_unit, &ch_b_config, &pcnt_ch_b);

    ESP_LOGI(tag, "--Configuring pulse counter actions--");
    pcnt_channel_set_edge_action(pcnt_ch_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_ch_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(pcnt_ch_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_ch_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    ESP_LOGI(tag, "--Starting pulse counter--");
    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);
}

/**
 * @brief Read the pulse counter value and flush it
 * 
 * @param None
 * 
 * @return Pulse counter value
 */
int32_t encoder_read_state(void)
{
    int32_t counter = 0;

    pcnt_unit_get_count(pcnt_unit, (int*)&counter);
    pcnt_unit_clear_count(pcnt_unit);

    return counter;
}
