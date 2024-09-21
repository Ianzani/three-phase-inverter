#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#define NUM_OF_PHASES               (3)

void pwm_init(void);
void pwm_change_duty(const uint32_t comp_value[NUM_OF_PHASES]);

#endif