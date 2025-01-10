#ifndef SIN_CALCULATOR_H
#define SIN_CALCULATOR_H

#define PI_VALUE        (3.14159f)
#define HZ_TO_RADS      (2 * PI_VALUE)


void sin_init_timer(void);

void sin_set_values(const float freq_value_rads);

void start_sin_calculator(void);

void turn_off_and_reset_sin_calulator(void);

#endif