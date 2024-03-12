#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

void pwm_init(void);
void pwm_change_duty(const uint16_t comp_value);

#endif