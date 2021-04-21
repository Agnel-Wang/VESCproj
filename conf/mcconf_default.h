#ifndef MCCONF_DEFAULT_H_
#define MCCONF_DEFAULT_H_

// Limits
#ifndef MCCONF_L_CURRENT_MAX
#define MCCONF_L_CURRENT_MAX			5.0f	// Current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_CURRENT_MIN
#define MCCONF_L_CURRENT_MIN			-5.0f	// Current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX			10.0f	// Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN			-10.0f	// Input current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		130.0f	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE			22.5f		// Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE			25.5f	// Maximum input voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_START
#define MCCONF_L_BATTERY_CUT_START		23.0f	// Start limiting the positive current at this voltage
#endif
#ifndef MCCONF_L_BATTERY_CUT_END
#define MCCONF_L_BATTERY_CUT_END		22.5f		// Limit the positive current completely at this voltage
#endif
#ifndef MCCONF_L_RPM_MAX
#define MCCONF_L_RPM_MAX				100000.0f	// The motor speed limit (Upper)
#endif
#ifndef MCCONF_L_RPM_MIN
#define MCCONF_L_RPM_MIN				-100000.0f	// The motor speed limit (Lower)
#endif
#ifndef MCCONF_L_RPM_START
#define MCCONF_L_RPM_START				0.8f		// Fraction of full speed where RPM current limiting starts
#endif
#ifndef MCCONF_L_SLOW_ABS_OVERCURRENT
#define MCCONF_L_SLOW_ABS_OVERCURRENT	true	// Use the filtered (and hence slower) current for the overcurrent fault detection
#endif
#ifndef MCCONF_L_MIN_DUTY
#define MCCONF_L_MIN_DUTY				0.005f	// Minimum duty cycle
#endif
#ifndef MCCONF_L_MAX_DUTY
#define MCCONF_L_MAX_DUTY				0.95f	// Maximum duty cycle
#endif
#ifndef MCCONF_L_CURR_MAX_RPM_FBRAKE
#define MCCONF_L_CURR_MAX_RPM_FBRAKE	300		// Maximum electrical RPM to use full brake at
#endif
#ifndef MCCONF_L_CURR_MAX_RPM_FBRAKE_CC
#define MCCONF_L_CURR_MAX_RPM_FBRAKE_CC	1500	// Maximum electrical RPM to use full brake at with current control
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_START
#define MCCONF_L_LIM_TEMP_FET_START		85.0f	// MOSFET temperature where current limiting should begin
#endif
#ifndef MCCONF_L_LIM_TEMP_FET_END
#define MCCONF_L_LIM_TEMP_FET_END		100.0f	// MOSFET temperature where everything should be shut off
#endif
#ifndef MCCONF_L_LIM_TEMP_MOTOR_START
#define MCCONF_L_LIM_TEMP_MOTOR_START	85.0f	// MOTOR temperature where current limiting should begin
#endif
#ifndef MCCONF_L_LIM_TEMP_MOTOR_END
#define MCCONF_L_LIM_TEMP_MOTOR_END		100.0f	// MOTOR temperature where everything should be shut off
#endif
#ifndef MCCONF_L_LIM_TEMP_ACCEL_DEC
#define MCCONF_L_LIM_TEMP_ACCEL_DEC		0.15f	// Decrease temperature limits this much during acceleration
#endif
#ifndef MCCONF_L_WATT_MAX
#define MCCONF_L_WATT_MAX				15000.0f	// Maximum wattage output
#endif
#ifndef MCCONF_L_WATT_MIN
#define MCCONF_L_WATT_MIN				-15000.0f	// Minimum wattage output (braking)
#endif

// Speed PID parameters
#ifndef MCCONF_S_PID_KP
#define MCCONF_S_PID_KP					0.004f	// Proportional gain
#endif
#ifndef MCCONF_S_PID_KI
#define MCCONF_S_PID_KI					0.004f	// Integral gain
#endif
#ifndef MCCONF_S_PID_KD
#define MCCONF_S_PID_KD					0.0001f	// Derivative gain
#endif
#ifndef MCCONF_S_PID_KD_FILTER
#define MCCONF_S_PID_KD_FILTER			0.2f	// Derivative filter
#endif
#ifndef MCCONF_S_PID_MIN_RPM
#define MCCONF_S_PID_MIN_RPM			0.0f	// Minimum allowed RPM
#endif

// Position PID parameters
#ifndef MCCONF_P_PID_KP
#define MCCONF_P_PID_KP					0.03f	// Proportional gain
#endif
#ifndef MCCONF_P_PID_KI
#define MCCONF_P_PID_KI					0.0f		// Integral gain
#endif
#ifndef MCCONF_P_PID_KD
#define MCCONF_P_PID_KD					0.0004f	// Derivative gain
#endif
#ifndef MCCONF_P_PID_KD_FILTER
#define MCCONF_P_PID_KD_FILTER			0.2f		// Derivative filter
#endif
#ifndef MCCONF_P_PID_ANG_DIV
#define MCCONF_P_PID_ANG_DIV			1.0f		// Divide angle by this value
#endif
#ifndef MCCONF_P_MAX_SPEED //**
#define MCCONF_P_MAX_SPEED              2100.f  // maximun speed in control pos mode
#endif

// Current control parameters
#ifndef MCCONF_CC_GAIN
#define MCCONF_CC_GAIN					0.0046f	// Current controller error gain
#endif
#ifndef MCCONF_CC_MIN_CURRENT
#define MCCONF_CC_MIN_CURRENT			0.1f		// Minimum allowed current
#endif
#ifndef MCCONF_CC_STARTUP_BOOST_DUTY
#define MCCONF_CC_STARTUP_BOOST_DUTY	0.01f	// The lowest duty cycle to use in current control mode (has to be > MCPWM_MIN_DUTY_CYCLE)
#endif
#ifndef MCCONF_CC_RAMP_STEP
#define MCCONF_CC_RAMP_STEP				0.04f	// Maximum duty cycle ramping step in CC mode
#endif

// FOC
#ifndef MCCONF_FOC_CURRENT_KP //**
#define MCCONF_FOC_CURRENT_KP			0.0024f
#endif
#ifndef MCCONF_FOC_CURRENT_KI //**
#define MCCONF_FOC_CURRENT_KI			10.40f
#endif
#ifndef MCCONF_FOC_F_SW
#define MCCONF_FOC_F_SW					20000.0f
#endif
#ifndef MCCONF_FOC_DT_US
#define MCCONF_FOC_DT_US				0.08f // Microseconds for dead time compensation
#endif
#ifndef MCCONF_FOC_ENCODER_INVERTED //**
#define MCCONF_FOC_ENCODER_INVERTED		true
#endif
#ifndef MCCONF_FOC_ENCODER_OFFSET //**
#define MCCONF_FOC_ENCODER_OFFSET		202.7f
#endif
#ifndef MCCONF_FOC_ENCODER_RATIO //**
#define MCCONF_FOC_ENCODER_RATIO		7.0f
#endif
#ifndef MCCONF_FOC_PLL_KP
#define MCCONF_FOC_PLL_KP				2000.0f
#endif
#ifndef MCCONF_FOC_PLL_KI
#define MCCONF_FOC_PLL_KI				40000.0f
#endif
#ifndef MCCONF_FOC_MOTOR_L //**
#define MCCONF_FOC_MOTOR_L				0.00000236f
#endif
#ifndef MCCONF_FOC_MOTOR_R //**
#define MCCONF_FOC_MOTOR_R				0.0104f
#endif
#ifndef MCCONF_FOC_MOTOR_FLUX_LINKAGE //**
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE	0.002139f
#endif
#ifndef MCCONF_FOC_OBSERVER_GAIN //**
#define MCCONF_FOC_OBSERVER_GAIN		2.1836e8f		// Can be something like 600 / L
#endif
#ifndef MCCONF_FOC_OBSERVER_GAIN_SLOW
#define MCCONF_FOC_OBSERVER_GAIN_SLOW	0.3f		// Observer gain scale at minimum duty cycle
#endif
#ifndef MCCONF_FOC_DUTY_DOWNRAMP_KP
#define MCCONF_FOC_DUTY_DOWNRAMP_KP		10.0f	// PI controller for duty control when decreasing the duty
#endif
#ifndef MCCONF_FOC_DUTY_DOWNRAMP_KI
#define MCCONF_FOC_DUTY_DOWNRAMP_KI		200.0f	// PI controller for duty control when decreasing the duty
#endif
#ifndef MCCONF_FOC_SL_D_CURRENT_DUTY
#define MCCONF_FOC_SL_D_CURRENT_DUTY	0.0f		// Inject d-axis current below this duty cycle in sensorless more
#endif
#ifndef MCCONF_FOC_SL_D_CURRENT_FACTOR
#define MCCONF_FOC_SL_D_CURRENT_FACTOR	0.0f		// Maximum q-axis current factor
#endif
#ifndef MCCONF_FOC_SL_ERPM
#define MCCONF_FOC_SL_ERPM				2500.0f	// ERPM above which only the observer is used
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)
#endif
#ifndef MCCONF_FOC_SAMPLE_HIGH_CURRENT
#define MCCONF_FOC_SAMPLE_HIGH_CURRENT	false	// High current sampling mode (requires three shunts)
#endif
#ifndef MCCONF_FOC_SAT_COMP
#define MCCONF_FOC_SAT_COMP				0.0f		// Stator saturation compensation
#endif
#ifndef MCCONF_FOC_TEMP_COMP
#define MCCONF_FOC_TEMP_COMP			false	// Motor temperature compensation
#endif
#ifndef MCCONF_FOC_TEMP_COMP_BASE_TEMP
#define MCCONF_FOC_TEMP_COMP_BASE_TEMP	25.0f	// Motor temperature compensation base temperature
#endif
#ifndef MCCONF_FOC_CURRENT_FILTER_CONST
#define MCCONF_FOC_CURRENT_FILTER_CONST	0.1f		// Filter constant for the filtered currents
#endif

// Misc
#ifndef MCCONF_M_FAULT_STOP_TIME
#define MCCONF_M_FAULT_STOP_TIME		500	// Ignore commands for this duration in msec when faults occur
#endif
#ifndef MCCONF_M_RAMP_STEP
#define MCCONF_M_RAMP_STEP				0.02f	// Duty cycle ramping step (1000 times/sec) at maximum duty cycle
#endif
#ifndef MCCONF_M_CURRENT_BACKOFF_GAIN
#define MCCONF_M_CURRENT_BACKOFF_GAIN	0.5f		// The error gain of the current limiting algorithm
#endif
#ifndef MCCONF_M_INVERT_DIRECTION
#define MCCONF_M_INVERT_DIRECTION		false // Invert the motor direction
#endif
#ifndef MCCONF_M_DRV8301_OC_MODE
#define MCCONF_M_DRV8301_OC_MODE		DRV8301_OC_LIMIT // DRV8301 over current protection mode
#endif
#ifndef MCCONF_M_DRV8301_OC_ADJ
#define MCCONF_M_DRV8301_OC_ADJ			16 // DRV8301 over current protection threshold
#endif

#endif /* MCCONF_DEFAULT_H_ */
