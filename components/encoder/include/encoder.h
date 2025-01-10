#ifndef ENCODER_H
#define ENCODER_H

#define EDGES_PER_ROTATION          (8000U)

void encoder_init(void);

int32_t encoder_read_state(void);

#endif