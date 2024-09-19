#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "lookup_table.h"
#include "pwm_control.h"


gptimer_handle_t timer_handle = NULL;
volatile uint8_t freq = 60;

/* ------------------------------- Private Function ------------------------------- */
static bool IRAM_ATTR ISR_timer_on_alarm(gptimer_handle_t timer, 
                                         const gptimer_alarm_event_data_t *edata, 
                                         void *user_ctx);
/* -------------------------------------------------------------------------------- */

/**
 * @brief Initialize a preiodic timer with 100us
 * 
 * @param None
 * 
 * @retval None
*/
void sin_init_timer(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000
    };
    gptimer_new_timer(&timer_config, &timer_handle);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = ISR_timer_on_alarm
    };
    gptimer_register_event_callbacks(timer_handle, &cbs, NULL);

    gptimer_enable(timer_handle);

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = 100,
        .flags.auto_reload_on_alarm = true
    };

    gptimer_set_alarm_action(timer_handle, &alarm_config);
    gptimer_start(timer_handle);
}

/**
 * @brief Freq setter
 * 
 * @param freq_value: Value to set
 * 
 * @retval None
 */
void sin_set_freq(const uint8_t freq_value)
{   
    freq = freq_value;
}

/**
 * @brief ISR's 100us periodic timer
 * 
 * @param timer : Timer's handle
 * @param edata : A pointer to a struct with timer's information
 * @param user_ctx :
 * 
 * @retval 0 : A high task has not awoken
 * @retval 1 : A high task has awoken
*/
static bool ISR_timer_on_alarm(gptimer_handle_t timer, 
                               const gptimer_alarm_event_data_t *edata, 
                               void *user_ctx)
{
    static volatile int32_t master_index = 0;

    uint16_t phase_1_index = 0;
    uint32_t phase_1_comp = 0;

    //TODO Change the freq to a float number
    master_index += (int32_t)freq * 18; /* (100[scale] * 100[usec] * freq * 1800[points]) / (10^6)[sec] */

    phase_1_index = (uint16_t)(master_index/100.0);

    if(phase_1_index >= 1800) {
        phase_1_index -= 1800;
        master_index -= 180000;
    }

    phase_1_comp =(uint16_t)((sine_lookup_table[phase_1_index] * 493.0) / 65535) + 4;

    pwm_change_duty(phase_1_comp);

    return pdFALSE;
}
