#ifndef MC_INTERFACE_H_
#define MC_INTERFACE_H_

#include "datatypes.h"
#include "stm32f4xx.h"

// External variables
extern volatile uint16_t ADC_Value[];
extern volatile int ADC_curr_norm_value[];

// Functions
void mc_interface_init(mc_configuration *configuration);
float mc_interface_temp_motor_filtered(void);
void mc_interface_mc_timer_isr(void);

#define HW_DEAD_TIME_VALUE				60 // Dead time

#endif /* MC_INTERFACE_H_ */
