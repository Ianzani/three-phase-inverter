#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H


#define MIN_FREQ_REF_RADS           (20.95f)


void control_loop_init(void);

int16_t get_freq_ref_rads(void);

void set_freq_ref_rads(int16_t value);

int16_t get_encoder_value_rads(void);

void start_control_loop(void);

void turn_off_and_reset_control_loop(void);

#endif
