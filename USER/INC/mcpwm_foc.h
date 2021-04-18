#ifndef MCPWM_FOC_H_
#define MCPWM_FOC_H_

#include "datatypes.h"

// Functions
void mcpwm_foc_init(volatile mc_configuration *configuration);
bool mcpwm_foc_init_done(void);
void mcpwm_foc_set_configuration(volatile mc_configuration *configuration);
mc_state mcpwm_foc_get_state(void);
bool mcpwm_foc_is_dccal_done(void);    
void mcpwm_foc_stop_pwm(void);
void mcpwm_foc_set_duty(float dutyCycle);
void mcpwm_foc_set_pid_speed(float rpm);
void mcpwm_foc_set_pid_pos(float pos);
void mcpwm_foc_set_current(float current);
void mcpwm_foc_set_brake_current(float current);
void mcpwm_foc_set_openloop(float current, float rpm);
float mcpwm_foc_get_duty_cycle_set(void);
float mcpwm_foc_get_duty_cycle_now(void);
float mcpwm_foc_get_pid_pos_set(void);
float mcpwm_foc_get_pid_pos_now(void);
float mcpwm_foc_get_rpm(void);
float mcpwm_foc_get_tot_current(void);
float mcpwm_foc_get_tot_current_filtered(void);
float mcpwm_foc_get_abs_motor_current(void);
float mcpwm_foc_get_abs_motor_current_filtered(void);
float mcpwm_foc_get_tot_current_directional(void);
float mcpwm_foc_get_tot_current_directional_filtered(void);
float mcpwm_foc_get_id(void);
float mcpwm_foc_get_iq(void);
float mcpwm_foc_get_tot_current_in(void);
int mcpwm_foc_get_tachometer_value(bool reset);
int mcpwm_foc_get_tachometer_abs_value(bool reset);
float mcpwm_foc_get_phase(void);
float mcpwm_foc_get_phase_observer(void);
float mcpwm_foc_get_phase_encoder(void);
float mcpwm_foc_get_vd(void);
float mcpwm_foc_get_vq(void);
void mcpwm_foc_encoder_detect(float current, bool print, float *offset, float *ratio, bool *inverted);
float mcpwm_foc_get_last_inj_adc_isr_duration(void);

// Defines
#define MCPWM_FOC_INDUCTANCE_SAMPLE_CNT_OFFSET		10 // Offset for the inductance measurement sample time in timer ticks
#define MCPWM_FOC_INDUCTANCE_SAMPLE_RISE_COMP		50 // Current rise time compensation
#define MCPWM_FOC_CURRENT_SAMP_OFFSET				(2) // Offset from timer top for injected ADC samples

#endif /* MCPWM_FOC_H_ */
