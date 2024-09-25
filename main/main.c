#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm_control.h"
#include "sin_calculator.h"
#include "control_loop.h"


void app_main(void)
{
    pwm_init();

    sin_init_timer();

    control_loop_init();
}