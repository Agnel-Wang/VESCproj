#ifndef MCPWM_FOC_H_
#define MCPWM_FOC_H_

#include "datatypes.h"

// Functions
void mcpwm_foc_init(volatile mc_configuration *configuration);

// Defines
#define MCPWM_FOC_INDUCTANCE_SAMPLE_CNT_OFFSET		10 // Offset for the inductance measurement sample time in timer ticks
#define MCPWM_FOC_INDUCTANCE_SAMPLE_RISE_COMP		50 // Current rise time compensation
#define MCPWM_FOC_CURRENT_SAMP_OFFSET				(2) // Offset from timer top for injected ADC samples

#endif /* MCPWM_FOC_H_ */
