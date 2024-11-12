#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_continuous.h"
#include "adc_interface.h"


#define NUM_OF_ADC_CH                       (3U)

#define NUM_OF_MSG_PER_CONV                 (NUM_OF_ADC_CH)
#define CONV_FRAME_SIZE_BYTES               (NUM_OF_MSG_PER_CONV * SOC_ADC_DIGI_DATA_BYTES_PER_CONV)
#define MAX_NUM_OF_CONV_IN_BUFFER           (2U)
#define MAX_ADC_BUF_LEN_BYTES               (MAX_NUM_OF_CONV_IN_BUFFER * CONV_FRAME_SIZE_BYTES)
#define ADC_SAMPLE_FREQ_HZ                  (3000U)

#define INVERSE_OF_SQUARE_ROOT_THREE        (0.57735f)
#define ADC_TO_STATOR_CURRENT_A_GAIN        (0.00888f)
#define ADC_TO_STATOR_CURRENT_A_OFFSET      (-16.8f)
#define ADC_TO_STATOR_CURRENT_B_GAIN        (0.00518f)
#define ADC_TO_STATOR_CURRENT_B_OFFSET      (-8.3f)
#define ADC_TO_BUS_VOLTAGE_GAIN             (0.255f)
#define ADC_TO_BUS_VOLTAGE_OFFSET           (-408.0f)

#define LEN_RING_BUFFER                     (50U)

typedef enum  {
    BUS_VOLTAGE_INDEX,
    STATOR_CURRENT_A_INDEX,
    STATOR_CURRENT_B_INDEX
} adc_buffer_indexs_e;


static const uint8_t adc_channels[NUM_OF_ADC_CH] = {ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6};

static adc_continuous_handle_t adc_handle = NULL;
static TaskHandle_t save_sensor_data_handle = NULL;

static uint16_t bus_voltage[LEN_RING_BUFFER] = {};
static uint16_t stator_current[LEN_RING_BUFFER] = {};
static uint8_t ring_buffer_index = 0;

static const char *tag = "ADC_INTERFACE";

/* ------------------------ Private Functions ------------------------ */
static bool IRAM_ATTR adc_conv_callback(adc_continuous_handle_t handle, 
                                        const adc_continuous_evt_data_t *edata, 
                                        void *user_data);
                                        
static void save_sensor_data(void * params);
static uint16_t calculate_peak_current(const uint16_t ia_adc, const uint16_t ib_adc);
static inline uint16_t calculate_bus_voltage(const uint16_t bus_voltage_adc);
/* ------------------------------------------------------------------- */

/**
 * @brief Initialize the ADCs
 * 
 * @param None
 * 
 * @return None
 */
void adc_interface_init(void)
{
    ESP_LOGI(tag, "--Initializing ADC task--");

    xTaskCreate(save_sensor_data, "save_sensor_data", 4096, NULL, 7, &save_sensor_data_handle);

    ESP_LOGI(tag, "--Initializing ADC interface--");

    adc_continuous_handle_cfg_t adc_handle_cfg = {
        .max_store_buf_size = MAX_ADC_BUF_LEN_BYTES,
        .conv_frame_size = CONV_FRAME_SIZE_BYTES
    };

    adc_continuous_new_handle(&adc_handle_cfg, &adc_handle);

    adc_continuous_config_t adc_config = {
        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
        .pattern_num = NUM_OF_ADC_CH
    };

    adc_digi_pattern_config_t adc_pattern_cfg[NUM_OF_ADC_CH] = {};

    for (uint8_t i = 0; i < NUM_OF_ADC_CH; i++)
    {
        adc_pattern_cfg[i].atten = ADC_ATTEN_DB_11;
        adc_pattern_cfg[i].channel = adc_channels[i];
        adc_pattern_cfg[i].unit = ADC_UNIT_1;
        adc_pattern_cfg[i].bit_width = ADC_BITWIDTH_12;
    }

    adc_config.adc_pattern = adc_pattern_cfg;

    adc_continuous_config(adc_handle, &adc_config);

    adc_continuous_evt_cbs_t ev_cbs = {
        .on_conv_done = adc_conv_callback
    };

    adc_continuous_register_event_callbacks(adc_handle, &ev_cbs, NULL);
    adc_continuous_start(adc_handle);

    ESP_LOGI(tag, "--ADC interface initialized--");

}

/**
 * @brief Bus_voltage getter
 * 
 * @param None
 * 
 * @return Bus_voltage value
 */
uint16_t get_bus_voltage(void) 
{
    uint32_t bus_voltage_sum = 0;
    
    for (uint8_t i = 0; i < LEN_RING_BUFFER; i++)
    {
        bus_voltage_sum += bus_voltage[i];
    }
    
    return bus_voltage_sum / LEN_RING_BUFFER;
}

/**
 * @brief Stator_current getter
 * 
 * @param None
 * 
 * @return Stator_current value
 */
uint16_t get_stator_current(void)
{
    uint32_t stator_current_sum = 0;
    
    for (uint8_t i = 0; i < LEN_RING_BUFFER; i++)
    {
        stator_current_sum += stator_current[i];
    }
    
    return stator_current_sum / LEN_RING_BUFFER;
}

/**
 * @brief ADC conversion done callback
 * 
 * @param handle: ADC handle
 * @param edata: A pointer to an event data struct
 * @param user_data: A pointer to some user data
 * 
 * @return A boll indicating if a higher priority task has woken
 */
static bool adc_conv_callback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(save_sensor_data_handle, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken;
}

/**
 * @brief Save the ADC values read into buffers
 * 
 * @param params: Task parameters
 * 
 * @return None
 */
static void save_sensor_data(void * params) 
{
    uint8_t read_adc_buffer[CONV_FRAME_SIZE_BYTES] = {};
    uint32_t num_of_bytes_read = 0;

    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        adc_continuous_read(adc_handle, read_adc_buffer, CONV_FRAME_SIZE_BYTES, &num_of_bytes_read, 0);

        adc_digi_output_data_t *data = (adc_digi_output_data_t *)read_adc_buffer;
       
        if (++ring_buffer_index >= LEN_RING_BUFFER)
        {
            ring_buffer_index = 0;
        }

        bus_voltage[ring_buffer_index] = calculate_bus_voltage(data[BUS_VOLTAGE_INDEX].type2.data);
        stator_current[ring_buffer_index] = calculate_peak_current(data[STATOR_CURRENT_A_INDEX].type2.data, data[STATOR_CURRENT_B_INDEX].type2.data);
    }
}

/**
 * @brief Calculate peak current using adc ia and ib outputs
 * 
 * @param ia_adc: Adc value of phase A current
 * @param ib_adc: Adc value of phase B current
 * 
 * @return The peak current in 10^2*A
 */
static uint16_t calculate_peak_current(const uint16_t ia_adc, const uint16_t ib_adc)
{
    float i_alpha = (ia_adc * ADC_TO_STATOR_CURRENT_A_GAIN) + ADC_TO_STATOR_CURRENT_A_OFFSET;
    float i_beta = (i_alpha * INVERSE_OF_SQUARE_ROOT_THREE) + 
                   (((ib_adc * ADC_TO_STATOR_CURRENT_B_GAIN) + ADC_TO_STATOR_CURRENT_B_OFFSET) * 2 * INVERSE_OF_SQUARE_ROOT_THREE);

    return (uint16_t) (sqrt((i_alpha * i_alpha) + (i_beta * i_beta)) * 100);
}

/**
 * @brief Calculate bus voltage using adc output
 * 
 * @param ia_adc: Adc value of bus voltage
 * 
 * @return The bus voltage in 10^2*V
 */
static inline uint16_t calculate_bus_voltage(const uint16_t bus_voltage_adc)
{
    return (uint16_t) (((bus_voltage_adc * ADC_TO_BUS_VOLTAGE_GAIN) + ADC_TO_BUS_VOLTAGE_OFFSET) * 100);
}
