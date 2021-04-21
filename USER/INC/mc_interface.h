#ifndef MC_INTERFACE_H_
#define MC_INTERFACE_H_

#include "datatypes.h"
#include "stm32f4xx.h"

// External variables
extern volatile uint16_t ADC_Value[];
extern volatile int ADC_curr_norm_value[];

// Functions
void mc_interface_init(mc_configuration *configuration);
void mc_interface_set_control_mode(mc_control_mode mode);
mc_control_mode mc_interface_get_control_mode(void);
void mc_interface_current_feedforward(float* iq_set);
float mc_interface_temp_motor_filtered(void);
mc_fault_code mc_interface_get_fault(void);
void mc_interface_release_motor(void);

void mc_interface_set_duty(float dutyCycle);
void mc_interface_set_current(float current);
void mc_interface_set_current_feedward(float current);
void mc_interface_set_pid_speed(float rpm);
void mc_interface_set_pid_pos(float pos);
void mc_interface_set_pos_set(float pos);
void mc_interface_set_pos_now(float pos);
void mc_interface_set_pos_max_rpm(float rpm);
void mc_interface_set_brake_current(float current);

float mc_interface_get_pid_pos_now(void);
float mc_interface_get_pos_now(void);
float mc_interface_get_tot_current(void);
float mc_interface_get_duty_cycle_set(void);
float mc_interface_get_duty_cycle_now(void);
float mc_interface_get_rpm_set(void);
float mc_interface_get_rpm_now(void);
float mc_interface_get_pos_set(void);
float mc_interface_get_pos_max_rpm(void);
void mc_interface_fault_stop(mc_fault_code fault);
void mc_interface_mc_timer_isr(void);

#define HW_DEAD_TIME_VALUE				60 // Dead time

#endif /* MC_INTERFACE_H_ */
