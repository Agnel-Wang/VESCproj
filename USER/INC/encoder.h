#ifndef _ENCODER_H
#define _ENCODER_H

#include "stdbool.h"

void encoder_init(void);
float encoder_read_deg(void);
bool encoder_index_found(void);

#endif /* _ENCODER_H */
