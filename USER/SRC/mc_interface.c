#include "mc_interface.h"
#include "hw_60.h"

// Global variables
volatile uint16_t ADC_Value[HW_ADC_CHANNELS];
volatile int ADC_curr_norm_value[3];

// Private variables
static volatile mc_configuration m_conf;
static mc_fault_code m_fault_now;
static int m_ignore_iterations;
static volatile unsigned int m_cycles_running;
static volatile bool m_lock_enabled;
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

void mc_interface_init(mc_configuration *configuration) {
	m_conf = *configuration;
    
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

void mc_interface_mc_timer_isr(void) {

}
