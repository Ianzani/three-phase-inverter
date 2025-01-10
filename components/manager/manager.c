#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "can_interface.h"
#include "control_loop.h"
#include "sin_calculator.h"
#include "pwm_control.h"
#include "manager.h"


#define MANAGER_QUEUE_LEN                       (4)
#define DELAY_TO_TURN_OFF_CONTROL_SYSTEM_MS     (1000U)


typedef enum {
    TURN_OFF,
    TURN_ON
} manager_actions_e;


QueueHandle_t manager_queue = NULL;

static const char *tag = "MANAGER";

/* ---------------- Private Functions ---------------- */
static void run_manager(void *params);
static void first_step_turn_off_routine(void);
static void second_step_turn_off_routine(void);
static void turn_on_routine(void);
/* --------------------------------------------------- */

/**
 * @brief Initialize the manager
 * 
 * @param None
 * 
 * @return None
 */
void manager_init(void)
{
    ESP_LOGI(tag, "--Initializing manager--");
   
    manager_queue = xQueueCreate(MANAGER_QUEUE_LEN, sizeof(uint8_t));

    xTaskCreate(run_manager, "run_manager", 2048, NULL, 20, NULL);
}

/**
 * @brief Turn on the control system if it is turned off
 * 
 * @param None
 * 
 * @return None
 */
void turn_on_control_system(void)
{
    static uint8_t manager_action = (uint8_t)TURN_ON;

    if (xQueueSend(manager_queue, &manager_action, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(tag, "IT WAS NOT POSSIBLE ADDED A NEW ITEM INTO MANAGER QUEUE");
    }
}

/**
 * @brief Turn off the control system if it is turned on
 * 
 * @param None
 * 
 * @return None
 */
void turn_off_control_system(void)
{
    static uint8_t manager_action = (uint8_t)TURN_OFF;

    if (xQueueSend(manager_queue, &manager_action, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(tag, "IT WAS NOT POSSIBLE ADDED A NEW ITEM INTO MANAGER QUEUE");
    }
}

/**
 * @brief Manager task responsable for turn on or off the control system
 * 
 * @param params: Tasks parameters
 * 
 * @return None
 */
static void run_manager(void *params)
{
    uint8_t manager_action;

    while (true)
    {
        if (xQueueReceive(manager_queue, &manager_action, portMAX_DELAY) == pdPASS)
        {
            if (manager_action == TURN_OFF)
            {
                first_step_turn_off_routine();

                vTaskDelay(DELAY_TO_TURN_OFF_CONTROL_SYSTEM_MS);
                
                second_step_turn_off_routine();
            }
            else
            {
                turn_on_routine();
            }
        }
    }
}

/**
 * @brief First phase of the turn off control system routine
 * 
 * @param None
 * 
 * @return None
 */
static void first_step_turn_off_routine(void)
{
    stop_receive_reference_message();
    set_freq_ref_rads_turn_off();
}

/**
 * @brief Second phase of the turn off control system routine
 * 
 * @param None
 * 
 * @return None
 */
static void second_step_turn_off_routine(void)
{
    turn_off_and_reset_control_loop();
    turn_off_and_reset_sin_calulator();
    turn_off_pwm_control_signal();
    return_receive_reference_message();
}

/**
 * @brief Turn on control system routine
 * 
 * @param None
 * 
 * @return None
 */
static void turn_on_routine(void)
{
    start_control_loop();
    start_sin_calculator();
}