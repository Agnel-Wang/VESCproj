#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

#include "datatypes.h"

// Default configuration parameters that can be overridden
#include "appconf_default.h"
#include "mcconf_default.h"

/*
 * Enable CAN-bus
 */
#define CAN_ENABLE					1

// Correction factor for computations that depend on the old resistor division factor
#define VDIV_CORR					((VIN_R2 / (VIN_R2 + VIN_R1)) / (2.2f / (2.2f + 33.0f)))

// Current ADC to amperes factor
#define FAC_CURRENT					((V_REG / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN))

#define V_REG						3.3f

/*
 * MCU
 */
#define SYSTEM_CORE_CLOCK			168000000


// Functions
void conf_general_init(void);
void conf_general_get_default_app_configuration(app_configuration *conf);
void conf_general_get_default_mc_configuration(mc_configuration *conf);

#endif /* CONF_GENERAL_H_ */
