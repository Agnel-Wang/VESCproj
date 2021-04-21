#ifndef MCPWM_FOC_H_
#define MCPWM_FOC_H_

#include "datatypes.h"

// Functions
void mcpwm_foc_init(volatile mc_configuration *configuration);
bool mcpwm_foc_init_done(void);
void mcpwm_foc_set_configuration(volatile mc_configuration *configuration);
mc_state mcpwm_foc_get_state(void);
void mcpwm_foc_set_state(mc_state state);
void mcpwm_foc_stop_pwm(void);
void mcpwm_foc_set_ontrol_mode(mc_control_mode mode);
mc_control_mode mcpwm_foc_get_ontrol_mode(void);

void mcpwm_foc_set_duty(float dutyCycle);
void mcpwm_foc_set_current(float current);
void mcpwm_foc_set_brake_current(float current);
void mcpwm_foc_set_pid_speed(float rpm);
void mcpwm_foc_set_pid_pos(float pos);
void mcpwm_foc_set_pos_set(float pos);
void mcpwm_foc_set_pos_now(float pos);
void mcpwm_foc_set_pos_max_rpm(float rpm);
void mcpwm_foc_set_openloop(float current, float rpm);

float mcpwm_foc_get_duty_cycle_set(void);
float mcpwm_foc_get_duty_cycle_now(void);
float mcpwm_foc_get_rpm_set(void);
float mcpwm_foc_get_rpm_now(void);
float mcpwm_foc_get_pid_pos_set(void);
float mcpwm_foc_get_pid_pos_now(void);
float mcpwm_foc_get_pos_set(void);
float mcpwm_foc_get_pos_now(void);
float mcpwm_foc_get_pos_max_rpm(void);
float mcpwm_foc_get_tot_current(void);
float mcpwm_foc_get_tot_current_filtered(void);
float mcpwm_foc_get_abs_motor_current(void);
float mcpwm_foc_get_abs_motor_current_filtered(void);
float mcpwm_foc_get_tot_current_directional(void);
float mcpwm_foc_get_tot_current_directional_filtered(void);
float mcpwm_foc_get_id(void);
float mcpwm_foc_get_iq(void);
float mcpwm_foc_get_tot_current_in(void);
float mcpwm_foc_get_phase(void);
float mcpwm_foc_get_phase_observer(void);
float mcpwm_foc_get_phase_encoder(void);
float mcpwm_foc_get_vd(void);
float mcpwm_foc_get_vq(void);

// Defines
#define MCPWM_FOC_INDUCTANCE_SAMPLE_CNT_OFFSET		10 // Offset for the inductance measurement sample time in timer ticks
#define MCPWM_FOC_INDUCTANCE_SAMPLE_RISE_COMP		50 // Current rise time compensation
#define MCPWM_FOC_CURRENT_SAMP_OFFSET				(2) // Offset from timer top for injected ADC samples

#endif /* MCPWM_FOC_H_ */
