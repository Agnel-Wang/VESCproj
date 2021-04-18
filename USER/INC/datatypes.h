/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

// Data types
typedef enum {
   MC_STATE_OFF = 0,
   MC_STATE_DETECTING,
   MC_STATE_RUNNING,
   MC_STATE_FULL_BRAKE,
} mc_state;

typedef enum {
	COMM_MODE_INTEGRATE = 0,
	COMM_MODE_DELAY
} mc_comm_mode;

// Auxiliary output mode
typedef enum {
	OUT_AUX_MODE_OFF = 0,
	OUT_AUX_MODE_ON_AFTER_2S,
	OUT_AUX_MODE_ON_AFTER_5S,
	OUT_AUX_MODE_ON_AFTER_10S
} out_aux_mode;

typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR
} mc_fault_code;

typedef enum {
	CONTROL_MODE_DUTY = 0,
	CONTROL_MODE_SPEED,
	CONTROL_MODE_CURRENT,
	CONTROL_MODE_CURRENT_BRAKE,
	CONTROL_MODE_POS,
	CONTROL_MODE_HANDBRAKE,
	CONTROL_MODE_OPENLOOP,
	CONTROL_MODE_NONE
} mc_control_mode;

typedef enum {
	DISP_POS_MODE_NONE = 0,
	DISP_POS_MODE_INDUCTANCE,
	DISP_POS_MODE_OBSERVER,
	DISP_POS_MODE_ENCODER,
	DISP_POS_MODE_PID_POS,
	DISP_POS_MODE_PID_POS_ERROR,
	DISP_POS_MODE_ENCODER_OBSERVER_ERROR
} disp_pos_mode;

typedef struct {
	float cycle_int_limit;
	float cycle_int_limit_running;
	float cycle_int_limit_max;
	float comm_time_sum;
	float comm_time_sum_min_rpm;
	int32_t comms;
	uint32_t time_at_comm;
} mc_rpm_dep_struct;

typedef enum {
	DRV8301_OC_LIMIT = 0,
	DRV8301_OC_LATCH_SHUTDOWN,
	DRV8301_OC_REPORT_ONLY,
	DRV8301_OC_DISABLED
} drv8301_oc_mode;

typedef enum {
	DEBUG_SAMPLING_OFF = 0,
	DEBUG_SAMPLING_NOW,
	DEBUG_SAMPLING_START,
	DEBUG_SAMPLING_TRIGGER_START,
	DEBUG_SAMPLING_TRIGGER_FAULT,
	DEBUG_SAMPLING_TRIGGER_START_NOSEND,
	DEBUG_SAMPLING_TRIGGER_FAULT_NOSEND,
	DEBUG_SAMPLING_SEND_LAST_SAMPLES
} debug_sampling_mode;

typedef struct {
	// Limits
	float l_current_max;
	float l_current_min;
	float l_in_current_max;
	float l_in_current_min;
	float l_abs_current_max;
	float l_min_erpm;
	float l_max_erpm;
	float l_erpm_start;
	float l_max_erpm_fbrake;
	float l_max_erpm_fbrake_cc;
	float l_min_vin;
	float l_max_vin;
	float l_battery_cut_start;
	float l_battery_cut_end;
	bool l_slow_abs_current;
	float l_temp_fet_start;
	float l_temp_fet_end;
	float l_temp_motor_start;
	float l_temp_motor_end;
	float l_temp_accel_dec;
	float l_min_duty;
	float l_max_duty;
	float l_watt_max;
	float l_watt_min;
	// Overridden limits (Computed during runtime)
	float lo_current_max;
	float lo_current_min;
	float lo_in_current_max;
	float lo_in_current_min;
	float lo_current_motor_max_now;
	float lo_current_motor_min_now;
	// Sensorless (bldc)
	float sl_min_erpm;
	float sl_min_erpm_cycle_int_limit;
	float sl_max_fullbreak_current_dir_change;
	float sl_cycle_int_limit;
	float sl_phase_advance_at_br;
	float sl_cycle_int_rpm_br;
	float sl_bemf_coupling_k;
	// FOC
	float foc_current_kp;
	float foc_current_ki;
	float foc_f_sw;
	float foc_dt_us;
	float foc_encoder_offset;
	bool  foc_encoder_inverted;
	float foc_encoder_ratio;
	float foc_motor_l;
	float foc_motor_r;
	float foc_motor_flux_linkage;
	float foc_observer_gain;
	float foc_observer_gain_slow;
	float foc_pll_kp;
	float foc_pll_ki;
	float foc_duty_dowmramp_kp;
	float foc_duty_dowmramp_ki;
	float foc_openloop_rpm;
	float foc_sl_openloop_hyst;
	float foc_sl_openloop_time;
	float foc_sl_d_current_duty;
	float foc_sl_d_current_factor;
	float foc_sl_erpm;
	bool  foc_sample_v0_v7;
	bool  foc_sample_high_current;
	float foc_sat_comp;
	bool  foc_temp_comp;
	float foc_temp_comp_base_temp;
	float foc_current_filter_const;
	// Speed PID
	float s_pid_kp;
	float s_pid_ki;
	float s_pid_kd;
	float s_pid_kd_filter;
	float s_pid_min_erpm;
	bool  s_pid_allow_braking;
	// Pos PID
	float p_pid_kp;
	float p_pid_ki;
	float p_pid_kd;
	float p_pid_kd_filter;
	float p_pid_ang_div;
	// Current controller
	float cc_startup_boost_duty;
	float cc_min_current; //Absolute values less than cc_min_current will release the motor.
	float cc_gain;
	float cc_ramp_step_max;
	// Misc
	int32_t m_fault_stop_time_ms;
	float m_duty_ramp_step;
	float m_current_backoff_gain;
	bool m_invert_direction;
	drv8301_oc_mode m_drv8301_oc_mode;
	int m_drv8301_oc_adj;
	float m_bldc_f_sw_min;
	float m_bldc_f_sw_max;
	float m_dc_f_sw;
	float m_ntc_motor_beta;
	out_aux_mode m_out_aux_mode;
} mc_configuration;

// Throttle curve mode
typedef enum {
	THR_EXP_EXPO = 0,
	THR_EXP_NATURAL,
	THR_EXP_POLY
} thr_exp_mode;

typedef struct {
	// Settings
	uint8_t controller_id;
	uint32_t timeout_msec;
	float timeout_brake_current;
	bool send_can_status;
	uint32_t send_can_status_rate_hz;
} app_configuration;

// Communication commands
typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING
} COMM_PACKET_ID;

typedef struct {
	int id;
	float rpm;
	float current;
	float pos;
} can_status_msg;

// Logged fault data
typedef struct {
	mc_fault_code fault;
	float current;
	float current_filtered;
	float voltage;
	float duty;
	float rpm;
	int tacho;
	int cycles_running;
	int tim_val_samp;
	int tim_current_samp;
	int tim_top;
	int comm_step;
	float temperature;
	int drv8301_faults;
} fault_data;

// External LED state
typedef enum {
	LED_EXT_OFF = 0,
	LED_EXT_NORMAL,
	LED_EXT_BRAKE,
	LED_EXT_TURN_LEFT,
	LED_EXT_TURN_RIGHT,
	LED_EXT_BRAKE_TURN_LEFT,
	LED_EXT_BRAKE_TURN_RIGHT,
	LED_EXT_BATT
} LED_EXT_STATE;

#endif /* DATATYPES_H_ */
