#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "lookup_table.h"
#include "pwm_control.h"
#include "sin_calculator.h"

#define PERIODIC_TIME_RESOLUTION_HZ     (1000000U)
#define TIMER_PERIOD_S                  (1e-4f)

#define NOMINAL_VOLTAGE_V               (220.0f)
#define NOMINAL_FREQUENCY_HZ            (60.0f)
#define ANGULAR_COEF_V_F                (NOMINAL_VOLTAGE_V / NOMINAL_FREQUENCY_HZ) /* 220V / 60Hz */
#define MAX_AMPLITUDE_VALUE             (1.0f)
#define MIN_AMPLITUDE_VALUE             (0.1f)


static const char *tag = "SIN_CALCULATOR";

static TaskHandle_t sin_modulation_handle = NULL;
static gptimer_handle_t timer_handle = NULL;
static float freq_hz = 0;
static float amplitude = 0;


/* ------------------------------- Private Functions ------------------------------- */
static bool IRAM_ATTR ISR_timer_on_alarm(gptimer_handle_t timer, 
                                         const gptimer_alarm_event_data_t *edata, 
                                         void *user_ctx);
static void sin_modulation(void * params);
/* -------------------------------------------------------------------------------- */

/**
 * @brief Initialize a periodic timer with 100us
 * 
 * @param None
 * 
 * @retval None
*/
void sin_init_timer(void)
{
    ESP_LOGI(tag, "--Initializing sine modulation--");
    
    /* Initialize the modulation task, which runs every 100us */
    xTaskCreate(sin_modulation, "sin_modulation", 2048, NULL, 10, &sin_modulation_handle);
    
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = PERIODIC_TIME_RESOLUTION_HZ
    };
    gptimer_new_timer(&timer_config, &timer_handle);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = ISR_timer_on_alarm
    };
    gptimer_register_event_callbacks(timer_handle, &cbs, NULL);

    gptimer_enable(timer_handle);

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = TIMER_PERIOD_S * PERIODIC_TIME_RESOLUTION_HZ,
        .flags.auto_reload_on_alarm = true
    };

    gptimer_set_alarm_action(timer_handle, &alarm_config);
    gptimer_start(timer_handle);
}

/**
 * @brief Freq_hz setter
 * 
 * @param freq_value_rads: Value to set in rad/s
 * 
 * @retval None
 */
void sin_set_values(float freq_value_rads)
{   
    freq_hz = freq_value_rads / HZ_TO_RADS;

    amplitude = (freq_hz >= 0) ? ((freq_hz * ANGULAR_COEF_V_F) / NOMINAL_VOLTAGE_V) : ((-1.0 * freq_hz * ANGULAR_COEF_V_F) / NOMINAL_VOLTAGE_V);

    if (amplitude > MAX_AMPLITUDE_VALUE) {
        amplitude = MAX_AMPLITUDE_VALUE;
    }
    else if (amplitude < MIN_AMPLITUDE_VALUE) {
        amplitude = MIN_AMPLITUDE_VALUE;
    }
    else {
        //Do nothing
    }
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

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    vTaskNotifyGiveFromISR(sin_modulation_handle, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken;
}

/**
 * @brief Realize the sine modulation
 * 
 * @param params: A pointer to task params
 * 
 * @retval None
 */
static void sin_modulation(void * params)
{
    int32_t master_index = 0;

    while (true) {
        /* Wait the timer 100us ISR */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint16_t phase_index[NUM_OF_PHASES] = {};
        uint32_t phase_comp[NUM_OF_PHASES] = {};

        /* (1e4[scale] * 1e2[usec] * freq * 1800[points]) / (1e6)[sec] (+/- 0.5 to round))*/
        master_index += (freq_hz >= 0) ? (int32_t)(freq_hz * 1800.0 + 0.5) : (int32_t)(freq_hz * 1800.0 - 0.5); 

        /* This is used to generate an underflow at the lookup table */
        if (master_index < 0) {
            master_index += 18000000;
        }

        /* Calculate the phase indexs */
        phase_index[0] = (uint16_t)(master_index / 10000);
        phase_index[1] = phase_index[0] + 1200; /* +240° */
        phase_index[2] = phase_index[0] + 600; /* +120° */

        if(phase_index[0] >= LOOKUP_TABLE_LEN) {
            phase_index[0] -= LOOKUP_TABLE_LEN;
            master_index -= 18000000;
        }

        if(phase_index[1] >= LOOKUP_TABLE_LEN) {
            phase_index[1] -= LOOKUP_TABLE_LEN;
        }

        if(phase_index[2] >= LOOKUP_TABLE_LEN) {
            phase_index[2] -= LOOKUP_TABLE_LEN;
        }

        /* Here it's not used the full scale, due to deadtime */
        /* TODO: Think better about this scale (x*(493/65535) + 4)*/
        for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {
            uint16_t sine_with_gain = (uint16_t)((amplitude * sine_lookup_table[phase_index[i]]) + ((1 - amplitude) * 32767));
            phase_comp[i] =(uint16_t)((sine_with_gain * 493.0) / 65535) + 4;
        }

        pwm_change_duty(phase_comp);
    }
}
