#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "manager.h"
#include "encoder.h"
#include "sin_calculator.h"
#include "control_loop.h"


#define KP_GAIN_PI                      (0.015f)
#define KI_GAIN_PI                      (0.010f)
#define SATURATION_VALUE_PI             (15.5f)
#define SAMPLE_PERIOD_PI_S              (1e-3f)

#define PERIODIC_TIME_RESOLUTION_HZ     (1000000U)
#define MAX_FREQ_REF_RADS               (377.0f) /* ~60Hz*/
#define MECHANICAL_TO_SYNC_FREQ         (2)


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

static float freq_ref_rads = 0.0f; /* Reference frequency in rad/s */
static float encoder_value_rads = 0;

static bool control_loop_on = false;


/* ---------------------- Private functions ---------------------- */
static void create_pi_controller(const float kp, const float ki, 
                                 const float saturation_value, 
                                 const float sample_period_s,
                                 pi_controller_t * const pi_controller);

static bool ISR_timer_on_alarm(gptimer_handle_t timer, 
                               const gptimer_alarm_event_data_t *edata, 
                               void *user_ctx);

static void run_control_loop(void * params);

static float run_pi(const float in_value, const bool reset_pi);

static float counter_to_filtered_rads(int32_t encoder_counter);

static bool is_control_system_turned_off(void);

static bool is_control_system_turned_on(void);
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
 * @return freq_ref_rads in 10*rad/s
 */
int16_t get_freq_ref_rads(void)
{
    return (int16_t)(freq_ref_rads * 10 / MECHANICAL_TO_SYNC_FREQ);
}

/**
 * @brief freq_ref_rads setter
 * 
 * @param value: Value to be set in 10*rad/s
 * 
 * @return None
 */
void set_freq_ref_rads(int16_t value) 
{
    float tmp_freq = MECHANICAL_TO_SYNC_FREQ * value / 10.0;

    /* Turn off the control system */
    if ((tmp_freq >= -0.01) && (tmp_freq <= 0.01))
    {
        if (is_control_system_turned_on())
        {
            turn_off_control_system();
            freq_ref_rads = 0.0;
        }

        return;
    }

    /* Verify if the received reference frequency is valid */
    if ((tmp_freq <= (MIN_FREQ_REF_RADS - 0.1)) && (tmp_freq >= (-MIN_FREQ_REF_RADS + 0.1)))
    {
        ESP_LOGW(tag, "INVALID REFERENCE FREQUENCY");
        return;
    }

    /* Turn on the control system */
    if (is_control_system_turned_off())
    {
        turn_on_control_system();
    }

    /* Saturate the reference frequency */
    if (tmp_freq >= MAX_FREQ_REF_RADS) 
    {
        tmp_freq = MAX_FREQ_REF_RADS;
    }
    else if (tmp_freq <= -MAX_FREQ_REF_RADS)
    {
        tmp_freq = -MAX_FREQ_REF_RADS;
    }
    else
    {
        //Do nothing
    }

    freq_ref_rads = tmp_freq;
}

/**
 * @brief encoder_value_rads getter
 * 
 * @param None
 * 
 * @return encoder_value_rads in 10*rad/s
 */
int16_t get_encoder_value_rads(void)
{
    return (int16_t)(encoder_value_rads * 10 / MECHANICAL_TO_SYNC_FREQ);
}

/**
 * @brief Start running the control loop
 * 
 * @param None
 * 
 * @return None
 */
void start_control_loop(void)
{
    control_loop_on = true;
}

/**
 * @brief Stop running the control loop and reset the internal PI values
 * 
 * @param None
 * 
 * @return None
 */
void turn_off_and_reset_control_loop(void)
{
    control_loop_on = false;
    run_pi(0.0, true);
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
static float run_pi(const float in_value, const bool reset_pi)
{
    static float last_out_value = 0.0;
    static float last_saturated_proporcional_value = 0.0;

    if (reset_pi)
    {
        last_out_value = 0.0;
        last_saturated_proporcional_value = 0.0;

        return 0.0;
    }
    
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

        if (!control_loop_on)
        {
            continue;
        }

        encoder_value_rads = counter_to_filtered_rads(encoder_read_state()) * MECHANICAL_TO_SYNC_FREQ;

        float slip_freq = run_pi(freq_ref_rads - encoder_value_rads, false);

        sin_set_values(slip_freq + encoder_value_rads);
    }
}

/**
 * @brief Transform encoder counter into rad/s value and apply a filter
 * 
 * @param encoder_counter: Encoder counter value
 * 
 * @retval Encoder value in filtered rad/s
 */
static float counter_to_filtered_rads(int32_t encoder_counter)
{ 
    static float last_encoder_read = 0;

    float encoder_read = (2 * PI_VALUE * (float)encoder_counter) / (pi_controller.sample_period_s * (float)EDGES_PER_ROTATION);

    float encoder_read_filtered = 0.3 * encoder_read + last_encoder_read * 0.7; 
    
    last_encoder_read = encoder_read;

    return encoder_read_filtered; 
}

/**
 * @brief Verify if the system isn't running
 * 
 * @param None
 * 
 * @return A bool indicating the state
 */
static bool is_control_system_turned_off(void)
{
    return ((freq_ref_rads >= -0.1) && (freq_ref_rads <= 0.1));
}

/**
 * @brief Verify if the system is running
 * 
 * @param None
 * 
 * @return A bool indicating the state
 */
static bool is_control_system_turned_on(void)
{
    return ((freq_ref_rads >= MIN_FREQ_REF_RADS - 0.1) || (freq_ref_rads <= -MIN_FREQ_REF_RADS + 0.1));
}