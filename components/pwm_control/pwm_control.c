#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "pwm_control.h"


#define RESOLUTION_HZ               (10000000)                      // 10MHz - 100ns per tick
#define PERIOD_TICKS                (1000UL)                        // 1000 ticks = 100us = 1 period
#define MAX_INPUT_TICKS             (PERIOD_TICKS / 2)
#define GPIO_NUM_A1                 (42)
#define GPIO_NUM_B1                 (41)
#define GPIO_NUM_A2                 (40)
#define GPIO_NUM_B2                 (39)
#define GPIO_NUM_A3                 (38)
#define GPIO_NUM_B3                 (37) 
#define DEAD_TIME_IN_TICKS          (3)                             // 3 ticks = 300ns (deadtime = 600ns)


const static char *tag = "PWM_CONTROL";

static mcpwm_cmpr_handle_t comparator_A[NUM_OF_PHASES] = {};
static mcpwm_cmpr_handle_t comparator_B[NUM_OF_PHASES] = {};

/* -------------------- Private functions -------------------- */
static void sync_timers(mcpwm_timer_handle_t timers[NUM_OF_PHASES]);
/* ----------------------------------------------------------- */

void pwm_init(void)
{
    /* ------------------- Timers ------------------- */
    ESP_LOGI(tag, "--Creating timers--");

    mcpwm_timer_handle_t timer[NUM_OF_PHASES];
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = RESOLUTION_HZ,
        .period_ticks = PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
    };

    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {
        mcpwm_new_timer(&timer_config, &timer[i]);
    }
    /* ---------------------------------------------- */

    /* ------------------- Operators ------------------- */
    ESP_LOGI(tag, "--Creating operators--");

    mcpwm_oper_handle_t oper[NUM_OF_PHASES];
    mcpwm_operator_config_t oper_config = {
        .group_id = 0,
    };

    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {
        mcpwm_new_operator(&oper_config, &oper[i]);
        mcpwm_operator_connect_timer(oper[i], timer[i]);
    }
    /* ------------------------------------------------- */

    /* ------------------- Comparators ------------------- */
    ESP_LOGI(tag, "--Creating comparators--");

    mcpwm_comparator_config_t comparator_A_config = {
        .flags.update_cmp_on_tez = true,
    };

    mcpwm_comparator_config_t comparator_B_config = {
        .flags.update_cmp_on_tez = true,
    };

    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {
        mcpwm_new_comparator(oper[i], &comparator_A_config, &comparator_A[i]);
        mcpwm_new_comparator(oper[i], &comparator_B_config, &comparator_B[i]);
    }
    /* --------------------------------------------------- */

    /* ------------------- Generators------------------- */
    ESP_LOGI(tag, "--Creating generators--");

    mcpwm_gen_handle_t gen_A[NUM_OF_PHASES] = {};
    mcpwm_gen_handle_t gen_B[NUM_OF_PHASES] = {};
    mcpwm_generator_config_t gen_A_config = {};
    mcpwm_generator_config_t gen_B_config = {};
    const int gen_gpios_A[3] = {GPIO_NUM_A1, GPIO_NUM_A2, GPIO_NUM_A3};
    const int gen_gpios_B[3] = {GPIO_NUM_B1, GPIO_NUM_B2, GPIO_NUM_B3}; 

    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {
        gen_A_config.gen_gpio_num = gen_gpios_A[i];
        gen_B_config.gen_gpio_num = gen_gpios_B[i];

        mcpwm_new_generator(oper[i], &gen_A_config, &gen_A[i]);
        mcpwm_new_generator(oper[i], &gen_B_config, &gen_B[i]);
    }
    /* ------------------------------------------------ */

    /* ------------------- Comparator Values ------------------- */
    ESP_LOGI(tag, "--Defining comparator values--");
    turn_off_pwm_control_signal();
    /* --------------------------------------------------------- */

    /* ------------------- Comparator Actions ------------------- */
    ESP_LOGI(tag, "--Defining comparator actions--");

    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {                                             
        mcpwm_generator_set_action_on_compare_event(gen_A[i], 
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
                                                                                comparator_A[i], 
                                                                                MCPWM_GEN_ACTION_HIGH));
            
        mcpwm_generator_set_action_on_compare_event(gen_A[i], 
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, 
                                                                                comparator_A[i], 
                                                                                MCPWM_GEN_ACTION_LOW));

        mcpwm_generator_set_action_on_compare_event(gen_B[i], 
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
                                                                                comparator_B[i], 
                                                                                MCPWM_GEN_ACTION_LOW));
            
        mcpwm_generator_set_action_on_compare_event(gen_B[i], 
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, 
                                                                                comparator_B[i], 
                                                                                MCPWM_GEN_ACTION_HIGH));
    }
    /* ---------------------------------------------------------- */

    /* ------------------- Starting Timers ------------------- */
    ESP_LOGI(tag, "--Starting timers--");
    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) { 
        mcpwm_timer_enable(timer[i]);
        mcpwm_timer_start_stop(timer[i], MCPWM_TIMER_START_NO_STOP);
    }
    /* ------------------------------------------------------- */

    sync_timers(timer);
}

/**
 * @brief Change the pwm's compare value
 *
 * @param comp_value: Compare value (Range between 0 and PERIOD_TICKS/2)
 * 
 * @retval None 
 */
void pwm_change_duty(const uint32_t comp_value[NUM_OF_PHASES])
{
    int32_t comp_A[NUM_OF_PHASES] = {};
    int32_t comp_B[NUM_OF_PHASES] = {};
    int32_t max_ticks = PERIOD_TICKS / 2;

    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {
        comp_A[i]= (int32_t)comp_value[i] + DEAD_TIME_IN_TICKS;
        comp_B[i] = (int32_t)comp_value[i] - DEAD_TIME_IN_TICKS;

        if(comp_A[i] >= max_ticks) {
            comp_A[i] = max_ticks;
            comp_B[i] = max_ticks;
        }

        if(comp_B[i] <= 0) {
            comp_A[i] = 0;
            comp_B[i] = 0;    
        }
    }

    /* This is made here trying to avoid desynchronization */
    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {
        mcpwm_comparator_set_compare_value(comparator_A[i], (uint32_t)comp_A[i]);
        mcpwm_comparator_set_compare_value(comparator_B[i], (uint32_t)comp_B[i]);
    }
}

/**
 * @brief Turn off all six control signals (set zero the duty-cycle)
 * 
 * @param None
 * 
 * @return None
 */
void turn_off_pwm_control_signal(void)
{
    for (uint8_t i = 0; i < NUM_OF_PHASES; i++) {
        mcpwm_comparator_set_compare_value(comparator_A[i], 0ULL);
        mcpwm_comparator_set_compare_value(comparator_B[i], MAX_INPUT_TICKS);
    }
}

/**
 * @brief Synchronize the mcpwm timers
 * 
 * @param timers: A pointer to the timers to be synchronized
 * 
 * @retval None
 */
static void sync_timers(mcpwm_timer_handle_t timers[NUM_OF_PHASES])
{
    ESP_LOGI(tag, "--Synchronizing timers--");
    mcpwm_sync_handle_t soft_sync_source = NULL;
    mcpwm_soft_sync_config_t soft_sync_config = {};
    mcpwm_new_soft_sync_src(&soft_sync_config, &soft_sync_source);

    mcpwm_sync_handle_t timer_sync_source;
    mcpwm_timer_sync_src_config_t timer_sync_config = {
        .flags.propagate_input_sync = true,
    };
    mcpwm_new_timer_sync_src(timers[0], &timer_sync_config, &timer_sync_source);

    mcpwm_timer_sync_phase_config_t sync_phase_config = {
        .count_value = 0,
        .direction = MCPWM_TIMER_DIRECTION_UP,
        .sync_src = soft_sync_source,
    };
    mcpwm_timer_set_phase_on_sync(timers[0], &sync_phase_config);
    sync_phase_config.sync_src = timer_sync_source;
    for (uint8_t i = 1; i < NUM_OF_PHASES; i++) {
        mcpwm_timer_set_phase_on_sync(timers[i], &sync_phase_config);
    }

    mcpwm_soft_sync_activate(soft_sync_source);

    ESP_LOGI(tag, "--Timers synchronized--");
}