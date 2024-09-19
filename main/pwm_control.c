#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "pwm_control.h"


#define RESOLUTION_HZ               (10000000)                      // 10MHz - 100ns per tick
#define PERIOD_TICKS                (1000)                          // 1000 ticks = 100us = 1 period
#define GPIO_NUM_A                  (42)
#define GPIO_NUM_B                  (41)
#define DEAD_TIME_IN_TICKS          (3)                             // 3 ticks = 300ns (deadtime = 600ns)


mcpwm_cmpr_handle_t comparator_A = NULL;
mcpwm_cmpr_handle_t comparator_B = NULL;

void pwm_init(void)
{
    printf("Criando o timer\n");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = RESOLUTION_HZ,
        .period_ticks = PERIOD_TICKS,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
    };

    mcpwm_new_timer(&timer_config, &timer);
    printf("Timer criado com sucesso\n");

    printf("Criando operador\n");
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t oper_config = {
        .group_id = 0,
    };

    mcpwm_new_operator(&oper_config, &oper);
    printf("Operador criado com sucesso\n");

    printf("Conectando operador ao timer\n");
    mcpwm_operator_connect_timer(oper, timer);

    printf("Criando comparador e gerador\n");

    //COMPARADORES----------------     
    mcpwm_comparator_config_t comparator_A_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(oper, &comparator_A_config, &comparator_A);

    mcpwm_comparator_config_t comparator_B_config = {
        .flags.update_cmp_on_tez = true,
    };
    mcpwm_new_comparator(oper, &comparator_B_config, &comparator_B);
    //---------------------------

    // GERADORES ------------------
    mcpwm_gen_handle_t gen_A = NULL;
    mcpwm_generator_config_t gen_A_config = {
        .gen_gpio_num = GPIO_NUM_A,
    };
    mcpwm_new_generator(oper, &gen_A_config, &gen_A);

    mcpwm_gen_handle_t gen_B = NULL;
    mcpwm_generator_config_t gen_B_config = {
        .gen_gpio_num = GPIO_NUM_B,
    };
    mcpwm_new_generator(oper, &gen_B_config, &gen_B);
    //---------------------------

    //VALOR DE COMPARACAO -------
    printf("Definindo valores de comparação\n");
    mcpwm_comparator_set_compare_value(comparator_A, 0);
    mcpwm_comparator_set_compare_value(comparator_B, 0);
    //---------------------------

    //ACAO DE COMPARACAO --------
    printf("Configurando ações dos comparadores\n");                                                 
    mcpwm_generator_set_action_on_compare_event(gen_A, 
                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
                                                                               comparator_A, 
                                                                               MCPWM_GEN_ACTION_HIGH));
        
    mcpwm_generator_set_action_on_compare_event(gen_A, 
                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, 
                                                                               comparator_A, 
                                                                               MCPWM_GEN_ACTION_LOW));

    mcpwm_generator_set_action_on_compare_event(gen_B, 
                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, 
                                                                               comparator_B, 
                                                                               MCPWM_GEN_ACTION_LOW));
        
    mcpwm_generator_set_action_on_compare_event(gen_B, 
                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, 
                                                                               comparator_B, 
                                                                               MCPWM_GEN_ACTION_HIGH));
    //-----------------------------

    printf("Habilitando timer\n");
    mcpwm_timer_enable(timer);

    printf("Iniciando o timer\n");
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

}

/**
 * @brief Change the pwm's compare value
 *
 * @param comp_value: Compare value (Range between 0 and PERIOD_TICKS/2)
 * 
 * @retval None 
 */
void pwm_change_duty(const uint32_t comp_value)
{
    int32_t comp_A = 0;
    int32_t comp_B = 0;
    int32_t max_ticks = PERIOD_TICKS / 2;

    comp_A = (int32_t)comp_value + DEAD_TIME_IN_TICKS;
    comp_B = (int32_t)comp_value - DEAD_TIME_IN_TICKS;

    if(comp_A >= max_ticks) {
        comp_A = max_ticks;
        comp_B = max_ticks;
    }

    if(comp_B <= 0) {
        comp_A = 0;
        comp_B = 0;    
    }

    mcpwm_comparator_set_compare_value(comparator_A, (uint32_t)comp_A);
    mcpwm_comparator_set_compare_value(comparator_B, (uint32_t)comp_B);
}