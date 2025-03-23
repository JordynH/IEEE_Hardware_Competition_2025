#ifndef ENCODER_H
#define ENCODER_H

#include "driver/pcnt.h"

void encoder_init(pcnt_unit_t unit, int encA, int encB);
void init_all_encoders();
int read_encoder(pcnt_unit_t unit);

#endif // ENCODER_H