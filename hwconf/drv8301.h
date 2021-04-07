#ifndef HWCONF_DRV8301_H_
#define HWCONF_DRV8301_H_

#include "datatypes.h"

// Functions
void drv8301_init(void);
void drv8301_set_oc_adj(int val);
void drv8301_set_oc_mode(drv8301_oc_mode mode);
int drv8301_read_faults(void);
void drv8301_reset_faults(void);
unsigned int drv8301_read_reg(int reg);
void drv8301_write_reg(int reg, int data);

// Defines
#define DRV8301_FAULT_FETLC_OC		(1 << 0)
#define DRV8301_FAULT_FETHC_OC		(1 << 1)
#define DRV8301_FAULT_FETLB_OC		(1 << 2)
#define DRV8301_FAULT_FETHB_OC		(1 << 3)
#define DRV8301_FAULT_FETLA_OC		(1 << 4)
#define DRV8301_FAULT_FETHA_OC		(1 << 5)
#define DRV8301_FAULT_OTW			(1 << 6)
#define DRV8301_FAULT_OTSD			(1 << 7)
#define DRV8301_FAULT_PVDD_UV		(1 << 8)
#define DRV8301_FAULT_GVDD_UV		(1 << 9)
#define DRV8301_FAULT_FAULT			(1 << 10)
#define DRV8301_FAULT_GVDD_OV		(1 << 11)

#endif /* HWCONF_DRV8301_H_ */
