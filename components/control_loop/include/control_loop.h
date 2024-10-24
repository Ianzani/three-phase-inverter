#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H


void control_loop_init(void);

int16_t get_freq_ref_rads(void);

void set_freq_ref_rads(int16_t value);

int16_t get_encoder_value_rads(void);

#endif
