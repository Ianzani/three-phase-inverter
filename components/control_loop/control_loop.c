#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "encoder.h"
#include "sin_calculator.h"
#include "control_loop.h"


#define KP_GAIN_PI                      (0.015f)
#define KI_GAIN_PI                      (0.010f)
#define SATURATION_VALUE_PI             (15.5f)
#define SAMPLE_PERIOD_PI_S              (1e-3f)

#define PERIODIC_TIME_RESOLUTION_HZ     (1000000U)
#define MAX_FREQ_REF_RADS               (377U) /* ~60Hz*/


typedef struct {
    float kp;
    float ki;
    float saturation_value;
    float sample_period_s;
} pi_controller_t;


static const char *tag = "CONTROL_LOOP";

static gptimer_handle_t timer_handle = NULL;
static TaskHandle_t run_control_loop_handle = NULL;
static pi_controller_t pi_controller = {};

static float freq_ref_rads = 376.99f; /* Reference frequency in rad/s */
static float encoder_value_rads = 0;


/* ---------------------- Private functions ---------------------- */
static void create_pi_controller(const float kp, const float ki, 
                                 const float saturation_value, 
                                 const float sample_period_s,
                                 pi_controller_t * const pi_controller);

static bool ISR_timer_on_alarm(gptimer_handle_t timer, 
                               const gptimer_alarm_event_data_t *edata, 
                               void *user_ctx);

static void run_control_loop(void * params);

static float run_pi(const float in_value);

static float counter_to_filtered_rads(int32_t encoder_counter);
/* --------------------------------------------------------------- */

/**
 * @brief Initialize the control loop
 * 
 * @param None
 * 
 * @retval None
 */
void control_loop_init(void)
{
    ESP_LOGI(tag, "--Creating PI controller--");
    create_pi_controller(KP_GAIN_PI, KI_GAIN_PI, SATURATION_VALUE_PI, SAMPLE_PERIOD_PI_S, &pi_controller);

    ESP_LOGI(tag, "--Initializing control loop task--");
    xTaskCreate(run_control_loop, "run_control_loop", 4096, NULL, 9, &run_control_loop_handle);

    ESP_LOGI(tag, "--Initializing 1ms periodic timer--");
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
        .alarm_count = (uint64_t)(SAMPLE_PERIOD_PI_S * PERIODIC_TIME_RESOLUTION_HZ),
        .flags.auto_reload_on_alarm = true
    };

    gptimer_set_alarm_action(timer_handle, &alarm_config);
    gptimer_start(timer_handle);
}

/**
 * @brief freq_ref_rads getter
 * 
 * @param None
 * 
 * @return freq_ref_rads in 10^2*rad/s
 */
uint16_t get_freq_ref_rads(void)
{
    return (uint16_t)(freq_ref_rads * 100);
}

/**
 * @brief freq_ref_rads setter
 * 
 * @param value: Value to be set in 10^2*rad/s
 * 
 * @return None
 */
void set_freq_ref_rads(uint16_t value) 
{
    float tmp_freq = value / 100.0;

    if (tmp_freq >= MAX_FREQ_REF_RADS) {
        tmp_freq = MAX_FREQ_REF_RADS;
    }

    freq_ref_rads = tmp_freq;
}

/**
 * @brief encoder_value_rads getter
 * 
 * @param None
 * 
 * @return encoder_value_rads in 10^2*rad/s
 */
uint16_t get_encoder_value_rads(void)
{
    return (uint16_t)(encoder_value_rads * 100);
}

/**
 * @brief ISR's 1ms periodic timer
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
    
    vTaskNotifyGiveFromISR(run_control_loop_handle, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken;
}

/**
 * @brief Create a PI controller struct
 * 
 * @param kp: Proportional gain
 * @param ki: Integral gain
 * @param saturation_value: Output saturation value
 * @param sample_period_s: PI controller sample rate
 * @param pi_controller: A pointer to the new PI controller struct
 * 
 * @retval None
 */
static void create_pi_controller(const float kp, 
                                 const float ki, 
                                 const float saturation_value, 
                                 const float sample_period_s,
                                 pi_controller_t * const pi_controller)
{
    pi_controller->kp = kp;
    pi_controller->ki = ki;
    pi_controller->saturation_value = saturation_value;
    pi_controller->sample_period_s = sample_period_s;
}

/**
 * @brief Run the PI logic
 * 
 * @param in_value: Input value
 * 
 * @return The output value
 */
static float run_pi(const float in_value)
{
    static float last_out_value = 0;
    static float last_saturated_proporcional_value = 0;

    float proportional_value = pi_controller.kp * in_value;
    float integral_value = pi_controller.ki * pi_controller.sample_period_s * in_value;

    float out_value = proportional_value + integral_value - last_saturated_proporcional_value + last_out_value;

    /* Output value saturation */
    if (out_value >= pi_controller.saturation_value) {
        out_value = pi_controller.saturation_value;
    }
    else if (out_value <= -pi_controller.saturation_value) {
        out_value = -pi_controller.saturation_value;
    }
    else {
        //Do nothing
    }

    /* Last proportional value saturation */
    if (proportional_value >= pi_controller.saturation_value) {
        last_saturated_proporcional_value = pi_controller.saturation_value;
    }
    else if (proportional_value <= -pi_controller.saturation_value) {
        last_saturated_proporcional_value = -pi_controller.saturation_value;
    }
    else {
        last_saturated_proporcional_value = proportional_value;
    }

    last_out_value = out_value;

    return out_value;
}

/**
 * @brief Run the control loop
 * 
 * @param params: A pointer to task params
 * 
 * @retval None
 */
static void run_control_loop(void * params)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        encoder_value_rads = counter_to_filtered_rads(encoder_read_state());
        float slip_freq = run_pi(freq_ref_rads - encoder_value_rads);

        sin_set_values(slip_freq + encoder_value_rads);
    }
}

static float counter_to_filtered_rads(int32_t encoder_counter)
{ 
    static float last_encoder_read = 0;

    float encoder_read = (2 * PI_VALUE * (float)encoder_counter) / (pi_controller.sample_period_s * (float)EDGES_PER_ROTATION);

    float encoder_read_filtered = 0.3 * encoder_read + last_encoder_read * 0.7; 
    
    last_encoder_read = encoder_read;

    return encoder_read_filtered; 
}