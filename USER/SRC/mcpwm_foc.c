#include "mcpwm_foc.h"
#include "mc_interface.h"
#include "includes.h"
#include "stm32f4xx_conf.h"
#include "hw_60.h"
#include "sys.h"
#include "delay.h"
#include "conf_general.h"
#include "utils.h"
#include "encoder.h"

// Macros
#define TIMER_UPDATE_DUTY(duty1, duty2, duty3) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM1->CR1 &= ~TIM_CR1_UDIS;
        
#define TIMER_UPDATE_SAMP(samp) \
		TIM8->CCR1 = samp;
        
#define TIMER_UPDATE_SAMP_TOP(samp, top) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM1->ARR = top; \
		TIM8->CCR1 = samp; \
		TIM1->CR1 &= ~TIM_CR1_UDIS; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;

#define TIMER_UPDATE_DUTY_SAMP(duty1, duty2, duty3, samp) \
		TIM1->CR1 |= TIM_CR1_UDIS; \
		TIM8->CR1 |= TIM_CR1_UDIS; \
		TIM1->CCR1 = duty1; \
		TIM1->CCR2 = duty2; \
		TIM1->CCR3 = duty3; \
		TIM8->CCR1 = samp; \
		TIM1->CR1 &= ~TIM_CR1_UDIS; \
		TIM8->CR1 &= ~TIM_CR1_UDIS;
         
// Structs
typedef struct {
	volatile bool updated;
	volatile unsigned int top;
	volatile unsigned int duty;
	volatile unsigned int val_sample;
	volatile unsigned int curr1_sample;
	volatile unsigned int curr2_sample;
	volatile unsigned int curr3_sample;
} mc_timer_struct;

typedef struct {
	int sample_num;
	float avg_current_tot;
	float avg_voltage_tot;
	bool measure_inductance_now;
	float measure_inductance_duty;
} mc_sample_t;

// Private types
typedef struct {
	float id_target;
	float iq_target;
	float max_duty;
	float duty_now;
	float phase;
	float i_alpha;
	float i_beta;
	float i_abs;
	float i_abs_filter;
	float i_bus;
	float v_bus;
	float v_alpha;
	float v_beta;
	float mod_d;
	float mod_q;
	float id;
	float iq;
	float id_filter;
	float iq_filter;
	float vd;
	float vq;
	float vd_int;
	float vq_int;
	uint32_t svm_sector;
} motor_state_t;


// Private variables
static volatile mc_configuration *m_conf;
static volatile mc_state m_state;
static volatile mc_control_mode m_control_mode;
static volatile motor_state_t m_motor_state;
static volatile mc_timer_struct timer_struct;
static volatile mc_sample_t m_samples;
static volatile int direction;
static volatile bool m_init_done;
static volatile bool dccal_done;
static volatile int curr_samp_volt; // Use the voltage-synchronized samples for this current sample

static volatile int comm_step; // Range [1 6]
static volatile int detect_step; // Range [0 5]
static volatile int curr0_sum;
static volatile int curr1_sum;
static volatile int curr2_sum;
static volatile int m_curr_samples;
static volatile int curr0_offset;
static volatile int curr1_offset;
static volatile int curr2_offset;
static volatile bool m_phase_override;
static volatile float m_phase_now_override;
static volatile float m_duty_cycle_set;
static volatile float m_id_set;
static volatile float m_iq_set;
static volatile float m_openloop_speed;
static volatile bool m_output_on;
static volatile float m_pos_pid_set;
static volatile float m_speed_pid_set_rpm;
static volatile float m_phase_now_observer;
static volatile float m_phase_now_observer_override;
static volatile bool m_phase_observer_override;
static volatile float m_phase_now_encoder;
static volatile float m_phase_now_encoder_no_index;
static volatile float m_observer_x1;
static volatile float m_observer_x2;
static volatile float m_pll_phase;
static volatile float m_pll_speed;
static volatile int m_tachometer;
static volatile int m_tachometer_abs;
static volatile float last_inj_adc_isr_duration;
static volatile float m_pos_pid_now;
static volatile float last_current_sample;
static volatile float last_current_sample_filtered;
static volatile float mcpwm_detect_currents_avg[6];
static volatile float mcpwm_detect_avg_samples[6];
static volatile float switching_frequency_now;
static volatile int has_commutated;
static volatile float m_gamma_now;

// Private functions
static void do_dc_cal(void);
static void start_pwm_hw(void);
static void stop_pwm_hw(void);
void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
		float dt, volatile float *x1, volatile float *x2, volatile float *phase);
static float correct_encoder(float obs_angle, float enc_angle, float speed);
static void pll_run(float phase, float dt, volatile float *phase_var, volatile float *speed_var);
static void control_current(volatile motor_state_t *state_m, float dt);
static void run_pid_control_speed(float dt);
static void run_pid_control_pos(float angle_now, float angle_set, float dt);
    
void mcpwm_foc_init(volatile mc_configuration *configuration) {
    OSSchedLock();
    
    m_init_done = false;
    
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;    
    
    // Initialize variables
    m_conf = configuration;
    m_state = MC_STATE_OFF;
    m_control_mode = CONTROL_MODE_NONE;
	curr0_sum = 0;
	curr1_sum = 0;
	curr2_sum = 0;
	m_curr_samples = 0;
	dccal_done = false;
	m_phase_override = false;
	m_phase_now_override = 0.0f;
	m_duty_cycle_set = 0.0f;
	m_id_set = 0.0f;
	m_iq_set = 0.0f;
	m_openloop_speed = 0.0f;
	m_output_on = false;
	m_pos_pid_set = 0.0f;
	m_speed_pid_set_rpm = 0.0f;
	m_phase_now_observer = 0.0f;
	m_phase_now_observer_override = 0.0f;
	m_phase_observer_override = false;
	m_phase_now_encoder = 0.0f;
	m_phase_now_encoder_no_index = 0.0f;
	m_observer_x1 = 0.0f;
	m_observer_x2 = 0.0f;
	m_pll_phase = 0.0f;
	m_pll_speed = 0.0f;
	m_tachometer = 0;
	m_tachometer_abs = 0;
	last_inj_adc_isr_duration = 0;
	m_pos_pid_now = 0.0f;
	m_gamma_now = 0.0f;
	memset((void*)&m_motor_state, 0, sizeof(motor_state_t));
	memset((void*)&m_samples, 0, sizeof(mc_sample_t));
    
	TIM_DeInit(TIM1);
	TIM_DeInit(TIM8);
	TIM1->CNT = 0;
	TIM8->CNT = 0;
    
    // TIM1 clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)m_conf->foc_f_sw;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// Channel 1, 2 and 3 Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM1->ARR / 2;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
	TIM_BDTRInitStructure.TIM_DeadTime = HW_DEAD_TIME_VALUE;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	TIM_CCPreloadControl(TIM1, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
    
    /*
	 * ADC!
	 */
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	// Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);
    
    nvicEnableVector(DMA2_Stream4_IRQn, 3);

	// DMA for the ADC
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC_Value;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC->CDR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);

	// DMA2_Stream0 enable
	DMA_Cmd(DMA2_Stream4, ENABLE);

	// Enable transfer complete interrupt
	DMA_ITConfig(DMA2_Stream4, DMA_IT_TC, ENABLE);

	// ADC Common Init
	// Note that the ADC is running at 42MHz, which is higher than the
	// specified 36MHz in the data sheet, but it works.
	ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV;

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);

	// Enable Vrefint channel
	ADC_TempSensorVrefintCmd(ENABLE);

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	hw_setup_adc_channels();

	// Enable ADC1, ADC2, ADC3
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);

	// ------------- Timer8 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 500;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM8, ENABLE);
	TIM_CCPreloadControl(TIM8, ENABLE);

	// PWM outputs have to be enabled in order to trigger ADC on CCx
	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	// TIM1 Master and TIM8 slave
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Reset);

	// Enable TIM1 and TIM8
	TIM_Cmd(TIM1, ENABLE);
	TIM_Cmd(TIM8, ENABLE);

	// Main Output Enable
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // ADC sampling locations
    stop_pwm_hw();
    
	// Sample intervals. For now they are fixed with voltage samples in the center of V7
	// and current samples in the center of V0
	TIM8->CCR1 = MCPWM_FOC_CURRENT_SAMP_OFFSET;

	// Enable CC1 interrupt, which will be fired in V0 and V7
	TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);
	nvicEnableVector(TIM8_CC_IRQn, 6);

    OSSchedUnlock();

	// Calibrate current offset
	ENABLE_GATE();
    do_dc_cal();
    
	// Various time measurements
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(((168000000 / 2) / 10000000) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM12, ENABLE);

	// WWDG configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);
	WWDG_SetPrescaler(WWDG_Prescaler_1);
	WWDG_SetWindowValue(255);
	WWDG_Enable(100);
    
    m_init_done = true;
}

static void do_dc_cal(void) {
	// Wait max 5 seconds
	int cnt = 0;
	while(IS_DRV_FAULT()){
		delay_ms(1);
		cnt++;
		if (cnt > 5000) {
			break;
		}
	};

	delay_ms(1000);
	curr0_sum = 0;
	curr1_sum = 0;
	curr2_sum = 0;

	m_curr_samples = 0;
	while(m_curr_samples < 4000) {};
	curr0_offset = curr0_sum / m_curr_samples;
	curr1_offset = curr1_sum / m_curr_samples;
	curr2_offset = curr2_sum / m_curr_samples;
    
    dccal_done = true;
}

//dma ISR function
void mcpwm_foc_tim_sample_int_handler(void) {
    TIM12->CNT = 0;
    
    bool is_v7 = !(TIM1->CR1 & TIM_CR1_DIR);
    
    if (!m_samples.measure_inductance_now) {
        if (!m_conf->foc_sample_v0_v7 && is_v7) {
            return;
        }
    }
    
    // Reset the watchdog
	WWDG_SetCounter(100);
    
	int curr0 = ADC_Value[ADC_IND_CURR1];
	int curr1 = ADC_Value[ADC_IND_CURR2];
	int curr2 = ADC_Value[ADC_IND_CURR3];
    
	curr0_sum += curr0;
	curr1_sum += curr1;
	curr2_sum += curr2;
   
	curr0 -= curr0_offset;
	curr1 -= curr1_offset;
	curr2 -= curr2_offset; 
    
	m_curr_samples++;
    
	ADC_curr_norm_value[0] = curr0;
	ADC_curr_norm_value[1] = curr1;
	ADC_curr_norm_value[2] = curr2;
    
    // Use the best current samples depending on the modulation state.
    if (m_conf->foc_sample_high_current) {
        // High current sampling mode. Choose the lower currents to derive the highest one in order to be able to measure higher currents.
		const float i0_abs = fabsf(ADC_curr_norm_value[0]);
		const float i1_abs = fabsf(ADC_curr_norm_value[1]);
		const float i2_abs = fabsf(ADC_curr_norm_value[2]);
        
		if (i0_abs > i1_abs && i0_abs > i2_abs) {
			ADC_curr_norm_value[0] = -(ADC_curr_norm_value[1] + ADC_curr_norm_value[2]);
		} else if (i1_abs > i0_abs && i1_abs > i2_abs) {
			ADC_curr_norm_value[1] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[2]);
		} else if (i2_abs > i0_abs && i2_abs > i1_abs) {
			ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
		}
    } else {
		if (m_conf->foc_sample_v0_v7 && is_v7) {
			if (TIM1->CCR1 < TIM1->CCR2 && TIM1->CCR1 < TIM1->CCR3) {
				ADC_curr_norm_value[0] = -(ADC_curr_norm_value[1] + ADC_curr_norm_value[2]);
			} else if (TIM1->CCR2 < TIM1->CCR1 && TIM1->CCR2 < TIM1->CCR3) {
				ADC_curr_norm_value[1] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[2]);
			} else if (TIM1->CCR3 < TIM1->CCR1 && TIM1->CCR3 < TIM1->CCR2) {
				ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
			}
		} else {
			if (TIM1->CCR1 > TIM1->CCR2 && TIM1->CCR1 > TIM1->CCR3) {
				ADC_curr_norm_value[0] = -(ADC_curr_norm_value[1] + ADC_curr_norm_value[2]);
			} else if (TIM1->CCR2 > TIM1->CCR1 && TIM1->CCR2 > TIM1->CCR3) {
				ADC_curr_norm_value[1] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[2]);
			} else if (TIM1->CCR3 > TIM1->CCR1 && TIM1->CCR3 > TIM1->CCR2) {
				ADC_curr_norm_value[2] = -(ADC_curr_norm_value[0] + ADC_curr_norm_value[1]);
			}
		}
    }
    
	float ia = ADC_curr_norm_value[0] * FAC_CURRENT;
	float ib = ADC_curr_norm_value[1] * FAC_CURRENT;
    
	if (m_samples.measure_inductance_now) {
		if (!is_v7) {
			return;
		}
        
        static int inductance_state = 0;
        const uint32_t duty_cnt = (uint32_t)((float)TIM1->ARR * m_samples.measure_inductance_duty);
        const uint32_t samp_time = duty_cnt - MCPWM_FOC_INDUCTANCE_SAMPLE_CNT_OFFSET;
        
		if (inductance_state == 0) {
			TIMER_UPDATE_DUTY_SAMP(0, 0, 0, samp_time);
			start_pwm_hw();
		} else if (inductance_state == 2) {
			TIMER_UPDATE_DUTY(duty_cnt,	0, duty_cnt);
		} else if (inductance_state == 3) {
			m_samples.avg_current_tot += -((float)curr1 * FAC_CURRENT);
			m_samples.avg_voltage_tot += GET_INPUT_VOLTAGE();
			m_samples.sample_num++;
			TIMER_UPDATE_DUTY(0, 0, 0);
		} else if (inductance_state == 5) {
			TIMER_UPDATE_DUTY(0, duty_cnt, duty_cnt);
		} else if (inductance_state == 6) {
			m_samples.avg_current_tot += -((float)curr0 * FAC_CURRENT);
			m_samples.avg_voltage_tot += GET_INPUT_VOLTAGE();
			m_samples.sample_num++;
			TIMER_UPDATE_DUTY(0, 0, 0);
		} else if (inductance_state == 8) {
			TIMER_UPDATE_DUTY(duty_cnt, duty_cnt, 0);
        } else if (inductance_state == 9) {
            m_samples.avg_current_tot += -((float)curr2 * FAC_CURRENT);
			m_samples.avg_voltage_tot += GET_INPUT_VOLTAGE();
			m_samples.sample_num++;
			stop_pwm_hw();
			TIMER_UPDATE_SAMP(MCPWM_FOC_CURRENT_SAMP_OFFSET);
        } else if (inductance_state == 10) {
			inductance_state = 0;
			m_samples.measure_inductance_now = false;
			return;
		}
         
        inductance_state++;
        return;
    }
    
    float dt;
	if (m_conf->foc_sample_v0_v7) {
		dt = 1.0f / m_conf->foc_f_sw;
	} else {
		dt = 1.0f / (m_conf->foc_f_sw / 2.0f);
	}
    
    UTILS_LP_FAST(m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1f);
    
	float enc_ang = 0;
    //encoder is configured
    enc_ang = encoder_read_deg();
    float phase_tmp = enc_ang;
    if (m_conf->foc_encoder_inverted) {
        phase_tmp = 360.0f - phase_tmp;
    }
    phase_tmp *= m_conf->foc_encoder_ratio;
    phase_tmp -= m_conf->foc_encoder_offset;
    utils_norm_angle((float*)&phase_tmp);
    m_phase_now_encoder = phase_tmp * (M_PI / 180.0f);
    
	static float phase_before = 0.0f;
	const float phase_diff = utils_angle_difference_rad(m_motor_state.phase, phase_before);
	phase_before = m_motor_state.phase;
    
    if (m_state == MC_STATE_RUNNING) {
		// Clarke transform assuming balanced currents
		m_motor_state.i_alpha = ia;
		m_motor_state.i_beta = ONE_BY_SQRT3 * ia + TWO_BY_SQRT3 * ib;
    
		const float duty_abs = fabsf(m_motor_state.duty_now);
		float id_set_tmp = m_id_set;
		float iq_set_tmp = m_iq_set;
		m_motor_state.max_duty = m_conf->l_max_duty;
        
		static float duty_filtered = 0.0f;
		UTILS_LP_FAST(duty_filtered, m_motor_state.duty_now, 0.1f);
		utils_truncate_number(&duty_filtered, -1.0f, 1.0f);

		float duty_set = m_duty_cycle_set;
		bool control_duty = m_control_mode == CONTROL_MODE_DUTY;
        
		// When the filtered duty cycle in sensorless mode becomes low in brake mode, the
		// observer has lost tracking. Use duty cycle control with the lowest duty cycle
		// to get as smooth braking as possible.
		if (m_control_mode == CONTROL_MODE_CURRENT_BRAKE
				//				&& (m_conf->foc_sensor_mode != FOC_SENSOR_MODE_ENCODER) // Don't use this with encoders
				&& fabsf(duty_filtered) < 0.03f) {
			control_duty = true;
			duty_set = 0.0f;
		}

		// Brake when set ERPM is below min ERPM
		if (m_control_mode == CONTROL_MODE_SPEED &&
				fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
			control_duty = true;
			duty_set = 0.0f;
		}
        
		if (control_duty) {
			// Duty cycle control
			static float duty_i_term = 0.0f;
			if (fabsf(duty_set) < (duty_abs - 0.05f) ||
					(SIG(m_motor_state.vq) * m_motor_state.iq) < m_conf->lo_current_min) {
				// Truncating the duty cycle here would be dangerous, so run a PID controller.

				// Compensation for supply voltage variations
				float scale = 1.0f / GET_INPUT_VOLTAGE();

				// Compute error
				float error = duty_set - m_motor_state.duty_now;

				// Compute parameters
				float p_term = error * m_conf->foc_duty_dowmramp_kp * scale;
				duty_i_term += error * (m_conf->foc_duty_dowmramp_ki * dt) * scale;

				// I-term wind-up protection
				utils_truncate_number(&duty_i_term, -1.0f, 1.0f);

				// Calculate output
				float output = p_term + duty_i_term;
				utils_truncate_number(&output, -1.0f, 1.0f);
				iq_set_tmp = output * m_conf->lo_current_max;
			} else {
				// If the duty cycle is less than or equal to the set duty cycle just limit
				// the modulation and use the maximum allowed current.
				duty_i_term = 0.0f;
				m_motor_state.max_duty = duty_set;
				if (duty_set > 0.0f) {
					iq_set_tmp = m_conf->lo_current_max;
				} else {
					iq_set_tmp = -m_conf->lo_current_max;
				}
			}
		} else if (m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			// Braking
			iq_set_tmp = fabsf(iq_set_tmp);

			if (phase_diff > 0.0f) {
				iq_set_tmp = -iq_set_tmp;
			} else if (phase_diff == 0.0f) {
				iq_set_tmp = 0.0f;
			}
		}
        
        // Run observer
		if (!m_phase_override) {
			observer_update(m_motor_state.v_alpha, m_motor_state.v_beta,
					m_motor_state.i_alpha, m_motor_state.i_beta, dt,
					&m_observer_x1, &m_observer_x2, &m_phase_now_observer);
		}
        
        if (encoder_index_found()) {
            m_motor_state.phase = correct_encoder(m_phase_now_observer, m_phase_now_encoder, m_pll_speed);
        } else {
            // Rotate the motor in open loop if the index isn't found.
            m_motor_state.phase = m_phase_now_encoder_no_index;
        }

        if (!m_phase_override) {
            id_set_tmp = 0.0f;
        }
        
		// Force the phase to 0 in handbrake mode so that the current simply locks the rotor.
		if (m_control_mode == CONTROL_MODE_HANDBRAKE) {
			m_motor_state.phase = 0.0f;
		} else if (m_control_mode == CONTROL_MODE_OPENLOOP) {
			static float openloop_angle = 0.0f;
			openloop_angle += dt * m_openloop_speed;
			utils_norm_angle_rad(&openloop_angle);
			m_motor_state.phase = openloop_angle;
		}
        
		if (m_phase_override) {
			m_motor_state.phase = m_phase_now_override;
		}
        
		// Apply current limits
		// TODO: Consider D axis current for the input current as well.
		const float mod_q = m_motor_state.mod_q;
		if (mod_q > 0.001f) {
			utils_truncate_number(&iq_set_tmp, m_conf->lo_in_current_min / mod_q, m_conf->lo_in_current_max / mod_q);
		} else if (mod_q < -0.001f) {
			utils_truncate_number(&iq_set_tmp, m_conf->lo_in_current_max / mod_q, m_conf->lo_in_current_min / mod_q);
		}

		if (mod_q > 0.0f) {
			utils_truncate_number(&iq_set_tmp, m_conf->lo_current_min, m_conf->lo_current_max);
		} else {
			utils_truncate_number(&iq_set_tmp, -m_conf->lo_current_max, -m_conf->lo_current_min);
		}
        
		utils_saturate_vector_2d(&id_set_tmp, &iq_set_tmp,
				utils_max_abs(m_conf->lo_current_max, m_conf->lo_current_min));

		m_motor_state.id_target = id_set_tmp;
		m_motor_state.iq_target = iq_set_tmp;

		control_current(&m_motor_state, dt);
    } else {
		// Track back emf
		float Va = ADC_VOLTS(ADC_IND_SENS1) * ((VIN_R1 + VIN_R2) / VIN_R2);
		float Vb = ADC_VOLTS(ADC_IND_SENS2) * ((VIN_R1 + VIN_R2) / VIN_R2);
		float Vc = ADC_VOLTS(ADC_IND_SENS3) * ((VIN_R1 + VIN_R2) / VIN_R2);
    
		// Full Clarke transform (no balanced voltages)
		m_motor_state.v_alpha = (2.0f / 3.0f) * Va - (1.0f / 3.0f) * Vb - (1.0f / 3.0f) * Vc;
		m_motor_state.v_beta = ONE_BY_SQRT3 * Vb - ONE_BY_SQRT3 * Vc;

		float c, s;
		utils_fast_sincos_better(m_motor_state.phase, &s, &c);

		// Park transform
		float vd_tmp = c * m_motor_state.v_alpha + s * m_motor_state.v_beta;
		float vq_tmp = c * m_motor_state.v_beta  - s * m_motor_state.v_alpha;

		UTILS_NAN_ZERO(m_motor_state.vd);
		UTILS_NAN_ZERO(m_motor_state.vq);

		UTILS_LP_FAST(m_motor_state.vd, vd_tmp, 0.2f);
		UTILS_LP_FAST(m_motor_state.vq, vq_tmp, 0.2f);

		m_motor_state.vd_int = m_motor_state.vd;
		m_motor_state.vq_int = m_motor_state.vq;

		// Update corresponding modulation
		m_motor_state.mod_d = m_motor_state.vd / ((2.0f / 3.0f) * m_motor_state.v_bus);
		m_motor_state.mod_q = m_motor_state.vq / ((2.0f / 3.0f) * m_motor_state.v_bus);

		// The current is 0 when the motor is undriven
		m_motor_state.i_alpha = 0.0f;
		m_motor_state.i_beta = 0.0f;
		m_motor_state.id = 0.0f;
		m_motor_state.iq = 0.0f;
		m_motor_state.id_filter = 0.0f;
		m_motor_state.iq_filter = 0.0f;
		m_motor_state.i_bus = 0.0f;
		m_motor_state.i_abs = 0.0f;
		m_motor_state.i_abs_filter = 0.0f;

		// Run observer
		observer_update(m_motor_state.v_alpha, m_motor_state.v_beta,
				m_motor_state.i_alpha, m_motor_state.i_beta, dt, &m_observer_x1,
				&m_observer_x2, &m_phase_now_observer);

        m_motor_state.phase = correct_encoder(m_phase_now_observer, m_phase_now_encoder, m_pll_speed); 
    }
    
	// Calculate duty cycle
	m_motor_state.duty_now = SIG(m_motor_state.vq) *
			sqrtf(m_motor_state.mod_d * m_motor_state.mod_d +
					m_motor_state.mod_q * m_motor_state.mod_q) / SQRT3_BY_2;

	// Run PLL for speed estimation
	pll_run(m_motor_state.phase, dt, &m_pll_phase, &m_pll_speed);

	// Update tachometer (resolution = 60 deg as for BLDC)
	float ph_tmp = m_motor_state.phase;
	utils_norm_angle_rad(&ph_tmp);
	int step = (int)floorf((ph_tmp + M_PI) / (2.0f * M_PI) * 6.0f);
	utils_truncate_number_int(&step, 0, 5);
	static int step_last = 0;
	int diff = step - step_last;
	step_last = step;

	if (diff > 3) {
		diff -= 6;
	} else if (diff < -2) {
		diff += 6;
	}

	m_tachometer += diff;
	m_tachometer_abs += ABS(diff);

	// Track position control angle
	// TODO: Have another look at this.
	float angle_now = 0.0f;
    
    //encoder is configured
    angle_now = enc_ang;
    
    
	if (m_conf->p_pid_ang_div > 0.98f && m_conf->p_pid_ang_div < 1.02f) {
		m_pos_pid_now = angle_now;
	} else {
		static float angle_last = 0.0f;
		float diff_f = utils_angle_difference(angle_now, angle_last);
		angle_last = angle_now;
		m_pos_pid_now += diff_f / m_conf->p_pid_ang_div;
		utils_norm_angle((float*)&m_pos_pid_now);
	}

	// Run position control
	if (m_state == MC_STATE_RUNNING) {
		run_pid_control_pos(m_pos_pid_now, m_pos_pid_set, dt);
	}

	// MCIF handler
	mc_interface_mc_timer_isr();

	last_inj_adc_isr_duration = (float) TIM12->CNT / 10000000.0f;
}

void DMA2_Stream4_IRQHandler(void) {
    if (DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4) == SET) {
        mcpwm_foc_tim_sample_int_handler();
        
        DMA_ClearFlag(DMA2_Stream4, DMA_IT_TCIF4); //清除中断标志
        DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
    }
}

static void pll_run(float phase, float dt, volatile float *phase_var, volatile float *speed_var) {
	UTILS_NAN_ZERO(*phase_var);
	float delta_theta = phase - *phase_var;
	utils_norm_angle_rad(&delta_theta);
	UTILS_NAN_ZERO(*speed_var);
	*phase_var += (*speed_var + m_conf->foc_pll_kp * delta_theta) * dt;
	utils_norm_angle_rad((float*)phase_var);
	*speed_var += m_conf->foc_pll_ki * delta_theta * dt;
}
        
/**
 * Run the current control loop.
 *
 * @param state_m
 * The motor state.
 *
 * Parameters that shall be set before calling this function:
 * id_target
 * iq_target
 * max_duty
 * phase
 * i_alpha
 * i_beta
 * v_bus
 *
 * Parameters that will be updated in this function:
 * i_bus
 * i_abs
 * i_abs_filter
 * v_alpha
 * v_beta
 * mod_d
 * mod_q
 * id
 * iq
 * id_filter
 * iq_filter
 * vd
 * vq
 * vd_int
 * vq_int
 * svm_sector
 *
 * @param dt
 * The time step in seconds.
 */
static void control_current(volatile motor_state_t *state_m, float dt) {

}

static void run_pid_control_speed(float dt) {

}

static void run_pid_control_pos(float angle_now, float angle_set, float dt) {

}

static float correct_encoder(float obs_angle, float enc_angle, float speed) {
	float rpm_abs = fabsf(speed / ((2.0f * M_PI) / 60.0f));
	static bool using_encoder = true;

	// Hysteresis 5 % of total speed
	float hyst = m_conf->foc_sl_erpm * 0.05f;
	if (using_encoder) {
		if (rpm_abs > (m_conf->foc_sl_erpm + hyst)) {
			using_encoder = false;
		}
	} else {
		if (rpm_abs < (m_conf->foc_sl_erpm - hyst)) {
			using_encoder = true;
		}
	}

	return using_encoder ? enc_angle : obs_angle;
}

// See http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
		float dt, volatile float *x1, volatile float *x2, volatile float *phase) {

	const float L = (3.0f / 2.0f) * m_conf->foc_motor_l;
	const float lambda = m_conf->foc_motor_flux_linkage;
	float R = (3.0f / 2.0f) * m_conf->foc_motor_r;

	// Saturation compensation
	const float sign = (m_motor_state.iq * m_motor_state.vq) >= 0.0f ? 1.0f : -1.0f;
	R -= R * sign * m_conf->foc_sat_comp * (m_motor_state.i_abs_filter / m_conf->l_current_max);

	// Temperature compensation
	const float t = mc_interface_temp_motor_filtered();
	if (m_conf->foc_temp_comp && t > -5.0f) {
		R += R * 0.00386f * (t - m_conf->foc_temp_comp_base_temp);
	}

	const float L_ia = L * i_alpha;
	const float L_ib = L * i_beta;
	const float R_ia = R * i_alpha;
	const float R_ib = R * i_beta;
	const float lambda_2 = SQ(lambda);
	const float gamma_half = m_gamma_now * 0.5f;

	// Iterative with some trial and error
	const int iterations = 6;
	const float dt_iteration = dt / (float)iterations;
	for (int i = 0;i < iterations;i++) {
		float err = lambda_2 - (SQ(*x1 - L_ia) + SQ(*x2 - L_ib));
		float gamma_tmp = gamma_half;
		if (utils_truncate_number_abs(&err, lambda_2 * 0.2f)) {
			gamma_tmp *= 10.0f;
		}
		float x1_dot = -R_ia + v_alpha + gamma_tmp * (*x1 - L_ia) * err;
		float x2_dot = -R_ib + v_beta + gamma_tmp * (*x2 - L_ib) * err;

		*x1 += x1_dot * dt_iteration;
		*x2 += x2_dot * dt_iteration;
	}

	UTILS_NAN_ZERO(*x1);
	UTILS_NAN_ZERO(*x2);

	*phase = utils_fast_atan2(*x2 - L_ib, *x1 - L_ia);
}


static void stop_pwm_hw(void) {
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_ForcedAction_InActive);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    
    //set switching frequency
//    switching_frequency_now = m_conf->m_bldc_f_sw_max;
//    mc_timer_struct timer_tmp;
//    
//    OSSchedLock();
//    timer_tmp = timer_struct;
//    OSSchedUnlock();
//    
//    timer_tmp.top = SYSTEM_CORE_CLOCK / (int)switching_frequency_now;
//	update_adc_sample_pos(&timer_tmp);
//	set_next_timer_settings(&timer_tmp);
    
	m_output_on = false;
}

static void start_pwm_hw(void) {
	TIM_SelectOCxM(TIM1, TIM_Channel_1, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_2, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable);

	TIM_SelectOCxM(TIM1, TIM_Channel_3, TIM_OCMode_PWM1);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable);
    
	m_output_on = true;
}

//ADC interrupt service routine
void ADC1_2_3_IRQHandler(void) {
	ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
}

//TIM8_CC interrupt service routine
void TIM8_CC_IRQHandler(void) {
    if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) {
        if(m_init_done) {
            // Generate COM event here for synchronization
            TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
        }

		// Clear the IT pending bit
		TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
    }
}

