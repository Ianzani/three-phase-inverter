#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm_control.h"

void app_main(void)
{
    pwm_init();

    // while(1) {

    //     uint16_t i = 0;

    //     if(++i >= 400) {
    //         pwm_change_duty(i);
    //     }
    //     else {
    //         i = 0;
    //     }

    //     vTaskDelay(1); 
    // }
}