#include "mc_interface.h"
#include "hw_60.h"
#include "drv8301.h"
#include "encoder.h"
#include "mcpwm_foc.h"
#include "utils.h"
#include "includes.h"
#include "hw_60.h"

// Macros
#define DIR_MULT		(m_conf.m_invert_direction ? -1.0f : 1.0f)
#define FAULT_VEC_LEN						25

// Global variables
volatile uint16_t ADC_Value[HW_ADC_CHANNELS];
volatile int ADC_curr_norm_value[3];

// Private variables
static volatile mc_configuration m_conf;
static mc_fault_code m_fault_now; // current motor fault
static fault_data fault_vec[FAULT_VEC_LEN]; // Logged fault data
static volatile int fault_vec_write;
static int m_ignore_iterations;
static volatile unsigned int m_cycles_running;
static volatile bool m_lock_override_once;
static volatile float m_motor_current_sum;
static volatile float m_input_current_sum;
static volatile float m_motor_current_iterations;
static volatile float m_input_current_iterations;
static volatile float m_motor_id_sum;
static volatile float m_motor_iq_sum;
static volatile float m_motor_id_iterations;
static volatile float m_motor_iq_iterations;
static volatile float m_amp_seconds;
static volatile float m_amp_seconds_charged;
static volatile float m_watt_seconds;
static volatile float m_watt_seconds_charged;
static volatile float m_position_set;
static volatile float m_temp_fet;
static volatile float m_temp_motor;

// Sampling variables
#define ADC_SAMPLE_MAX_LEN		2000
static volatile int16_t m_curr0_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_curr1_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph1_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph2_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_ph3_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_vzero_samples[ADC_SAMPLE_MAX_LEN];
static volatile uint8_t m_status_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_curr_fir_samples[ADC_SAMPLE_MAX_LEN];
static volatile int16_t m_f_sw_samples[ADC_SAMPLE_MAX_LEN];
static volatile int8_t m_phase_samples[ADC_SAMPLE_MAX_LEN];
static volatile int m_sample_len;
static volatile int m_sample_int;
static volatile int m_sample_now;
static volatile int m_sample_trigger;
static volatile float m_last_adc_duration_sample;


void mc_interface_init(mc_configuration *configuration) {
	m_conf = *configuration;
	m_fault_now = FAULT_CODE_NONE;
    fault_vec_write = 0;
	m_ignore_iterations = 0;
	m_cycles_running = 0;
	m_lock_override_once = false;
	m_motor_current_sum = 0.0f;
	m_input_current_sum = 0.0f;
	m_motor_current_iterations = 0.0f;
	m_input_current_iterations = 0.0f;
	m_motor_id_sum = 0.0f;
	m_motor_iq_sum = 0.0f;
	m_motor_id_iterations = 0.0f;
	m_motor_iq_iterations = 0.0f;
	m_amp_seconds = 0.0f;
	m_amp_seconds_charged = 0.0f;
	m_watt_seconds = 0.0f;
	m_watt_seconds_charged = 0.0f;
	m_position_set = 0.0f;
	m_last_adc_duration_sample = 0.0f;
	m_temp_fet = 0.0f;
	m_temp_motor = 0.0f;

	m_sample_len = 1000;
	m_sample_int = 1;
	m_sample_now = 0;
	m_sample_trigger = 0;
    
    // Start threads
    
    
    
	drv8301_set_oc_mode(configuration->m_drv8301_oc_mode);
	drv8301_set_oc_adj(configuration->m_drv8301_oc_adj);
    
    // Initialize encoder
    encoder_init();
    
    // Initialize selected implementation
    mcpwm_foc_init(&m_conf);
}

/**
 * Get filtered motor temperature. The temperature is pre-calculated, so this
 * functions is fast.
 *
 * @return
 * The filtered motor temperature.
 */
float mc_interface_temp_motor_filtered(void) {
	return m_temp_motor;
}

mc_fault_code mc_interface_get_fault(void) {
	return m_fault_now;
}

void mc_interface_set_duty(float dutyCycle) {
    mcpwm_foc_set_duty(DIR_MULT * dutyCycle);
}

void mc_interface_set_current(float current) {
    mcpwm_foc_set_current(DIR_MULT * current);
}

void mc_interface_set_brake_current(float current) {
    mcpwm_foc_set_brake_current(DIR_MULT * current);
}

void mc_interface_set_pid_speed(float rpm) {
    mcpwm_foc_set_pid_speed(DIR_MULT * rpm);
}

/**
 * Disconnect the motor and let it turn freely.
 */
void mc_interface_release_motor(void) {
	mc_interface_set_current(0.0f);
}

void mc_interface_set_pid_pos(float pos) {
    m_position_set = pos;
    
	pos *= DIR_MULT;
	utils_norm_angle(&pos);
    
    mcpwm_foc_set_pid_pos(pos);
}

float mc_interface_get_duty_cycle_set(void) {
    float ret = 0.0f;
    
    ret = mcpwm_foc_get_duty_cycle_set();
    
    return DIR_MULT * ret;
}

float mc_interface_get_duty_cycle_now(void) {
    float ret = 0.0f;
    
    ret = mcpwm_foc_get_duty_cycle_now();
    
    return DIR_MULT * ret;
}

float mc_interface_get_rpm(void) {
    float ret = 0.0f;
    
    ret = mcpwm_foc_get_rpm();
    
    return DIR_MULT * ret;
}

int mc_interface_get_tachometer_value(bool reset) {
    int ret = 0;
    
    ret = mcpwm_foc_get_tachometer_value(reset);
    
    return DIR_MULT * ret;
}

void mc_interface_fault_stop(mc_fault_code fault) {
 	if (m_fault_now == fault) {
		m_ignore_iterations = m_conf.m_fault_stop_time_ms;
		return;
	}   
    
    if(mcpwm_foc_is_dccal_done() && m_fault_now == FAULT_CODE_NONE) {
        OSSchedLock();
		volatile int val_samp = TIM8->CCR1;
		volatile int current_samp = TIM1->CCR4;
		volatile int tim_top = TIM1->ARR;
        OSSchedUnlock();
    
		fault_data fdata;
		fdata.fault = fault;
		fdata.current = mcpwm_foc_get_tot_current();
		fdata.current_filtered = mcpwm_foc_get_tot_current_filtered();
		fdata.voltage = GET_INPUT_VOLTAGE();
		fdata.duty = mc_interface_get_duty_cycle_now();
		fdata.rpm = mc_interface_get_rpm();
		fdata.tacho = mc_interface_get_tachometer_value(false);
		fdata.cycles_running = m_cycles_running;
		fdata.tim_val_samp = val_samp;
		fdata.tim_current_samp = current_samp;
		fdata.tim_top = tim_top;
		fdata.comm_step = 0;
		fdata.temperature = NTC_TEMP(ADC_IND_TEMP_MOS);
		if (fault == FAULT_CODE_DRV) {
			fdata.drv8301_faults = drv8301_read_faults();
		}
        fault_vec[fault_vec_write++] = fdata;
        if(fault_vec_write >= FAULT_VEC_LEN) {
            fault_vec_write = 0;
        }
    }
    
    m_ignore_iterations = m_conf.m_fault_stop_time_ms;
    
    mcpwm_foc_stop_pwm();
    
    m_fault_now = fault;
}

void mc_interface_mc_timer_isr(void) {
    //LED PWM Driver update
    
    const float input_voltage = GET_INPUT_VOLTAGE();
    
    // Check for faults that should stop the motor
    static int wrong_voltage_iterations = 0;
    if (input_voltage < m_conf.l_min_vin || input_voltage > m_conf.l_max_vin) {
        wrong_voltage_iterations++;
        
        if ((wrong_voltage_iterations >= 8)) {
            mc_interface_fault_stop(input_voltage < m_conf.l_min_vin ?
					FAULT_CODE_UNDER_VOLTAGE : FAULT_CODE_OVER_VOLTAGE);
        }
    } else {
        wrong_voltage_iterations = 0;
    }
    
	if (mcpwm_foc_get_state() == MC_STATE_RUNNING) {
		m_cycles_running++;
	} else {
		m_cycles_running = 0;
	}    
    
    const float current = mcpwm_foc_get_tot_current_filtered();
    const float current_in = mcpwm_foc_get_tot_current_in();
	m_motor_current_sum += current;
	m_input_current_sum += current_in;
	m_motor_current_iterations++;
	m_input_current_iterations++;  

	m_motor_id_sum += mcpwm_foc_get_id();
	m_motor_iq_sum += mcpwm_foc_get_iq();
	m_motor_id_iterations++;
	m_motor_iq_iterations++;

	float abs_current = mcpwm_foc_get_tot_current();
	float abs_current_filtered = current;  
    
    abs_current = mcpwm_foc_get_abs_motor_current();
    abs_current_filtered = mcpwm_foc_get_abs_motor_current_filtered();
    
    // Current fault code
	if (m_conf.l_slow_abs_current) {
		if (fabsf(abs_current_filtered) > m_conf.l_abs_current_max) {
			mc_interface_fault_stop(FAULT_CODE_ABS_OVER_CURRENT);
		}
	} else {
		if (fabsf(abs_current) > m_conf.l_abs_current_max) {
			mc_interface_fault_stop(FAULT_CODE_ABS_OVER_CURRENT);
		}
	}
    
	// DRV fault code
	if (IS_DRV_FAULT()) {
		mc_interface_fault_stop(FAULT_CODE_DRV);
	}
}
