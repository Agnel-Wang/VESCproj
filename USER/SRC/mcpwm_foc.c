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
#include "math.h"

static void sincosf(float value, float *s, float *c);
static void sincosf(float value, float *s, float *c) {
    *s = utils_sin_f32(value);
    *c = utils_cos_f32(value);
}

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
	int sample_num;
	float avg_current_tot;
	float avg_voltage_tot;
	bool measure_inductance_now; //当前是否正在测量电感，用于mcpwm_foc_measure_inductance()函数中
	float measure_inductance_duty; //测量电感时的占空比
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
	float i_abs; //The magnitude of the motor current.
	float i_abs_filter;
	float i_bus;
	float v_bus; //DC link voltage
	float v_alpha;
	float v_beta;
	float mod_d;
	float mod_q;
	float id;
	float iq;
	float id_filter; //the filtered motor current
	float iq_filter;
	float vd;
	float vq;
	float vd_int;
	float vq_int;
	uint32_t svm_sector;
} motor_state_t;

// Private variables
static volatile mc_configuration *m_conf;
static volatile mc_state m_state; // 运行状态
static volatile bool m_begin;
static volatile mc_control_mode m_control_mode; // 控制模式
static volatile motor_state_t m_motor_state; // 电机状态参数

static volatile bool m_init_done; // foc初始化完成
static volatile int detect_step; // 转速计单步计数值 Range [0 5]
static volatile int curr0_sum; // 1路电流采样累计值
static volatile int curr1_sum; // 2路电流采样累计值
static volatile int curr2_sum; // 3路电流采样累计值
static volatile int m_curr_samples; // 计算电流零偏时的计数值
static volatile int curr0_offset; // 1路电流采样零偏补偿值
static volatile int curr1_offset; // 2路电流采样零偏补偿值
static volatile int curr2_offset; // 3路电流采样零偏补偿值
static volatile bool m_phase_override;
static volatile float m_phase_now_override;
static volatile float m_duty_cycle_set; // 占空比设定
static volatile float m_id_set; // d轴电流设定
static volatile float m_iq_set; // q轴电流设定
static volatile float m_openloop_speed;
static volatile bool m_output_on; // 是否是否输出pwm 只判断一次
static volatile float m_pos_pid_set; //位置设定 [0, 360]
static volatile float m_pos_set; // 双环嵌套的位置环设定
static volatile float m_speed_pid_set_rpm; // 速度设定 ERPM
static volatile float m_phase_now_observer; // 通过观测器获得的当前相位角
static volatile float m_phase_now_observer_override;
static volatile bool m_phase_observer_override;
static volatile float m_phase_now_encoder; // 通过编码器获得的当前相位角
static volatile float m_phase_now_encoder_no_index;
static volatile float m_observer_x1;
static volatile float m_observer_x2;
static volatile float m_pll_phase;
static volatile float m_pll_speed; //speed now. (rad/s) 
static volatile float last_inj_adc_isr_duration; // 上一次adc采样中断持续时间
static volatile float m_pos_pid_now; // 当前位置,经过ang_div后的
static volatile float m_gamma_now;
static volatile float m_pos_now; // 累计位置
static volatile float m_pos_max_limit;

// Private functions
static void do_dc_cal(void);
static void start_pwm_hw(void);
static void stop_pwm_hw(void);
void observer_update(float v_alpha, float v_beta, float i_alpha, float i_beta,
		float dt, volatile float *x1, volatile float *x2, volatile float *phase);
static float correct_encoder(float obs_angle, float enc_angle, float speed);
static void svm(float alpha, float beta, uint32_t PWMHalfPeriod,
		uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector);
static void pll_run(float phase, float dt, volatile float *phase_var, volatile float *speed_var);
static void control_current(volatile motor_state_t *state_m, float dt);
static void run_pid_control_speed(float dt);
static void run_pid_control_angle(float angle_now, float angle_set, float dt);
static void run_pid_control_pos(float pos_now, float pos_set, float dt);
    
void mcpwm_foc_init(volatile mc_configuration *configuration) {
    OSSchedLock();
    
    m_init_done = false;
    
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;    
    
    // Initialize variables
    m_conf = configuration;
    m_state = MC_STATE_OFF;
    m_begin = false;
    m_control_mode = CONTROL_MODE_NONE;
	curr0_sum = 0;
	curr1_sum = 0;
	curr2_sum = 0;
	m_curr_samples = 0;
	m_phase_override = false;
	m_phase_now_override = 0.0f;
	m_duty_cycle_set = 0.0f;
	m_id_set = 0.0f;
	m_iq_set = 0.0f;
	m_openloop_speed = 0.0f;
	m_output_on = false;
	m_pos_pid_set = 0.0f;
    m_pos_set = 0.0;
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
	last_inj_adc_isr_duration = 0;
	m_pos_pid_now = 0.0f;
    m_pos_now = 0.0f;
    m_pos_max_limit = m_conf->p_max_speed;
	m_gamma_now = 0.0f;
	memset((void*)&m_motor_state, 0, sizeof(motor_state_t));
    
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
	DMA_InitStructure.DMA_BufferSize = HW_ADC_CHANNELS; // 通道数15
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
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2; //时钟分频2，即ADC时钟42MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // 采样间隔5cycles
	ADC_CommonInit(&ADC_CommonInitStructure);

	// Channel-specific settings
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 打开扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 关闭连续转换，需要DMA触发信号，即TIM8_IT_CC1
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Falling; // 在下降沿时检测
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1; // 触发方式TIM8_CC1中断
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 数据右对齐
	ADC_InitStructure.ADC_NbrOfConversion = HW_ADC_NBR_CONV; // 顺序进行规则转换的ADC通道的数目

	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);

	// Enable Vrefint channel
	ADC_TempSensorVrefintCmd(ENABLE);

	// Enable DMA request after last transfer (Multi-ADC mode)
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

	hw_setup_adc_channels(); // 采样间隔15cycles, 总转换时间TCONV = 15 + 12 = 27cycles
    // 采样时间 = (采样间隔 + 总转换时间) * 采样通道数 / ADC时钟
    // 采样时间 = (5 + 27) * 15 / 42,000,000 = 11.4us
    
	// Enable ADC1, ADC2, ADC3
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);

	// ------------- Timer8 for ADC sampling ------------- //
	// Time Base configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SYSTEM_CORE_CLOCK / (int)m_conf->foc_f_sw;
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
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(((SYSTEM_CORE_CLOCK / 2) / 10000000) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM12, ENABLE);
    
    // ------------- Timer5 for speed cycle ------------- //
    // TIM5 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    // Time base configuration
	TIM_TimeBaseStructure.TIM_Period = 12000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 7 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

 	TIM5->DIER |= 1<<0;//允许更新中断
	nvicEnableVector(TIM5_IRQn, 7);
	TIM_Cmd(TIM5, ENABLE);
    
    m_init_done = true;
}

bool mcpwm_foc_init_done(void) {
    return m_init_done;
}

void mcpwm_foc_set_configuration(volatile mc_configuration *configuration) {
    m_conf = configuration;
    
    m_control_mode = CONTROL_MODE_NONE;
    m_state = MC_STATE_OFF;
    stop_pwm_hw();
    uint32_t top = SYSTEM_CORE_CLOCK / (int)m_conf->foc_f_sw;
	TIMER_UPDATE_SAMP_TOP(MCPWM_FOC_CURRENT_SAMP_OFFSET, top);
}

mc_state mcpwm_foc_get_state(void) {
    return m_state;
}

void mcpwm_foc_set_state(mc_state state) {
    // 电机运行前需提前设定模式
    if(m_control_mode != CONTROL_MODE_NONE) {
        m_state = state;
    }
}

/**
 * Switch off all FETs.TIM12->CNT = 0;
 */
void mcpwm_foc_stop_pwm(void) {
    m_state = MC_STATE_RUNNING;
    m_control_mode = CONTROL_MODE_CURRENT;
	mcpwm_foc_set_current(0.0f);
}

void mcpwm_foc_set_ontrol_mode(mc_control_mode mode) {
    m_control_mode = mode;
}

mc_control_mode mcpwm_foc_get_ontrol_mode(void) {
	return m_control_mode;
}

/**
 * Use duty cycle control. Absolute values less than MCPWM_MIN_DUTY_CYCLE will stop the motor.
 * 
 * @param dutyCycle
 * The duty cycle to use.
 */
void mcpwm_foc_set_duty(float dutyCycle) {
		m_duty_cycle_set = dutyCycle;
}

/**
 * Use PID rpm control. Note that this value has to be multiplied by half of
 * the number of motor poles.
 *
 * @param rpm
 * The electrical RPM goal value to use.
 * 实为ERPM
 */
void mcpwm_foc_set_pid_speed(float rpm) {
	m_speed_pid_set_rpm = rpm;
}

/**
 * Use PID position control. Note that this only works when encoder support
 * is enabled.
 *
 * @param pos
 * The desired position of the motor in degrees.
 */
void mcpwm_foc_set_pid_pos(float pos) {
	m_pos_pid_set = pos;
}

void mcpwm_foc_set_pos_set(float pos) {
    m_pos_set = pos;
}

void mcpwm_foc_set_pos_now(float pos) {
    m_pos_now = pos;
}

void mcpwm_foc_set_pos_max_rpm(float rpm) {
    m_pos_max_limit = rpm;
}

/**
 * Use current control and specify a goal current to use. The sign determines
 * the direction of the torque. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use.
 */
void mcpwm_foc_set_current(float current) {
    if( m_control_mode == CONTROL_MODE_CURRENT ) {
        if (fabsf(current) < m_conf->cc_min_current) {
            m_control_mode = CONTROL_MODE_NONE;
            m_state = MC_STATE_OFF;
            stop_pwm_hw();
            return;
        }
        
        utils_truncate_number(&current, -m_conf->l_current_max, m_conf->l_current_max);
        m_iq_set = current;
    }
}

/**
 * Brake the motor with a desired current. Absolute values less than
 * conf->cc_min_current will release the motor.
 *
 * @param current
 * The current to use. Positive and negative values give the same effect.
 */
void mcpwm_foc_set_brake_current(float current) {
    if( m_control_mode == CONTROL_MODE_CURRENT_BRAKE ) {
        if (fabsf(current) < m_conf->cc_min_current) {
            m_control_mode = CONTROL_MODE_NONE;
            m_state = MC_STATE_OFF;
            stop_pwm_hw();
            return;
        }

        utils_truncate_number(&current, -m_conf->l_current_max, m_conf->l_current_max);
        m_iq_set = current;
    }
}

float mcpwm_foc_get_duty_cycle_set(void) {
	return m_duty_cycle_set;
}

float mcpwm_foc_get_duty_cycle_now(void) {
	return m_motor_state.duty_now;
}

float mcpwm_foc_get_pid_pos_set(void) {
	return m_pos_pid_set;
}

float mcpwm_foc_get_pid_pos_now(void) {
	return m_pos_pid_now;
}
float mcpwm_foc_get_pos_set(void) {
    return m_pos_set;
}

float mcpwm_foc_get_pos_now(void) {
    return m_pos_now;
}

float mcpwm_foc_get_pos_max_rpm(void) {
    return m_pos_max_limit;
}

/**
 * Calculate the current RPM of the motor. This is a signed value and the sign
 * depends on the direction the motor is rotating in. Note that this value has
 * to be divided by half the number of motor poles.
 *
 * @return
 * The RPM value.
 */
float mcpwm_foc_get_rpm_now(void) {
	return m_pll_speed / ((2.0f * M_PI) / 60.0f);
}

float mcpwm_foc_get_rpm_set(void) {
    return m_speed_pid_set_rpm;
}

/**
 * Get the motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current. This is the q-axis current which produces torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_foc_get_tot_current(void) {
	return SIG(m_motor_state.vq) * m_motor_state.iq;
}

/**
 * Get the filtered motor current. The sign of this value will
 * represent whether the motor is drawing (positive) or generating
 * (negative) current. This is the q-axis current which produces torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_foc_get_tot_current_filtered(void) {
	return SIG(m_motor_state.vq) * m_motor_state.iq_filter;
}

/**
 * Get the magnitude of the motor current, which includes both the
 * D and Q axis.
 *
 * @return
 * The magnitude of the motor current.
 */
float mcpwm_foc_get_abs_motor_current(void) {
	return m_motor_state.i_abs;
}

/**
 * Get the filtered magnitude of the motor current, which includes both the
 * D and Q axis.
 *
 * @return
 * The magnitude of the motor current.
 */
float mcpwm_foc_get_abs_motor_current_filtered(void) {
	return m_motor_state.i_abs_filter;
}

/**
 * Get the motor current. The sign of this value represents the direction
 * in which the motor generates torque.
 *
 * @return
 * The motor current.
 */
float mcpwm_foc_get_tot_current_directional(void) {
	return m_motor_state.iq;
}

/**
 * Get the filtered motor current. The sign of this value represents the
 * direction in which the motor generates torque.
 *
 * @return
 * The filtered motor current.
 */
float mcpwm_foc_get_tot_current_directional_filtered(void) {
	return m_motor_state.iq_filter;
}

/**
 * Get the direct axis motor current.
 *
 * @return
 * The D axis current.
 */
float mcpwm_foc_get_id(void) {
	return m_motor_state.id;
}

/**
 * Get the quadrature axis motor current.
 *
 * @return
 * The Q axis current.
 */
float mcpwm_foc_get_iq(void) {
	return m_motor_state.iq;
}

/**
 * Get the input current to the motor controller.
 *
 * @return
 * The input current.
 */
float mcpwm_foc_get_tot_current_in(void) {
	return m_motor_state.i_bus;
}


/**
 * Read the motor phase.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase(void) {
	float angle = m_motor_state.phase * (180.0f / M_PI);
	utils_norm_angle(&angle);
	return angle;
}

/**
 * Read the phase that the observer has calculated.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase_observer(void) {
	float angle = m_phase_now_observer * (180.0f / M_PI);
	utils_norm_angle(&angle);
	return angle;
}

/**
 * Read the phase from based on the encoder.
 *
 * @return
 * The phase angle in degrees.
 */
float mcpwm_foc_get_phase_encoder(void) {
	float angle = m_phase_now_encoder * (180.0f / M_PI);
	utils_norm_angle(&angle);
	return angle;
}

float mcpwm_foc_get_vd(void) {
	return m_motor_state.vd;
}

float mcpwm_foc_get_vq(void) {
	return m_motor_state.vq;
}


//Private functions

void TIM5_IRQHandler(void) {
    if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
        const float dt = 0.001f;//速度环频率1000hz

		// Update and the observer gain.
		m_gamma_now = utils_map(fabsf(m_motor_state.duty_now), 0.0f, 1.0f,
				m_conf->foc_observer_gain * m_conf->foc_observer_gain_slow, m_conf->foc_observer_gain);

        switch(m_control_mode) {
            case CONTROL_MODE_SPEED:
                run_pid_control_speed(dt);
                break;
            case CONTROL_MODE_POS:
                run_pid_control_pos(m_pos_set, m_pos_now, dt);
                break;
            default:
                break;
        }
        
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}

//dma ISR function
void mcpwm_foc_tim_sample_int_handler(void) {
    TIM12->CNT = 0;
    
    bool is_v7 = !(TIM1->CR1 & TIM_CR1_DIR);
    
    if (!m_conf->foc_sample_v0_v7 && is_v7) {
        return;
    }
    
    // Reset the watchdog
	//	WWDG_SetCounter(100);
    
    //1. 电流采样
    //1.1 获取原始采样电流的AD值
	int curr0 = ADC_Value[ADC_IND_CURR1];
	int curr1 = ADC_Value[ADC_IND_CURR2];
	int curr2 = ADC_Value[ADC_IND_CURR3];
    
    //1.2 获得电流在时间上的累计值，目前不知道这个值用来做什么
	curr0_sum += curr0;
	curr1_sum += curr1;
	curr2_sum += curr2;
   
    //1.3 对电流值进行修正，减去零偏
	curr0 -= curr0_offset;
	curr1 -= curr1_offset;
	curr2 -= curr2_offset; 
    
	m_curr_samples++;
    
    //1.4 实际电流AD值
	ADC_curr_norm_value[0] = curr0;
	ADC_curr_norm_value[1] = curr1;
	ADC_curr_norm_value[2] = curr2;
    
    //1.5 去除最大值，根据基尔霍夫定律对采样电流值进行修正 ia = - ( ib + ic )
    // Use the best current samples depending on the modulation state.
    if (m_conf->foc_sample_high_current) { //FOC - Advanced 里面， 默认关闭
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
    
    //1.6 将电流从AD值转换为安培
	float ia = ADC_curr_norm_value[0] * FAC_CURRENT;
	float ib = ADC_curr_norm_value[1] * FAC_CURRENT;
    
    float dt;
	if (m_conf->foc_sample_v0_v7) {
		dt = 1.0f / m_conf->foc_f_sw;
	} else {
		dt = 1.0f / (m_conf->foc_f_sw / 2.0f);
	}
    
    UTILS_LP_FAST(m_motor_state.v_bus, GET_INPUT_VOLTAGE(), 0.1f); //电源电压滤波
    
    //2. 通过编码器获得当前相位
	float enc_ang = 0;
    //encoder is configured 本程序只运行在配置绝对值编码器的情况下
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
    
    // 3. 参数计算
    // 3.1 运行状态
    if (m_state == MC_STATE_RUNNING) {
        // 3.1.1 占空比计算
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
        
        // 3.1.2 不同模式下的参数校正
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
        
        // 占空比模式
		if (control_duty) {
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
        // 电流制动模式
		} else if (m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
			// Braking
			iq_set_tmp = fabsf(iq_set_tmp);

			if (phase_diff > 0.0f) {
				iq_set_tmp = -iq_set_tmp;
			} else if (phase_diff == 0.0f) {
				iq_set_tmp = 0.0f;
			}
		}
        
        // 3.1.3 运行观测器
		if (!m_phase_override) {
			observer_update(m_motor_state.v_alpha, m_motor_state.v_beta,
					m_motor_state.i_alpha, m_motor_state.i_beta, dt,
					&m_observer_x1, &m_observer_x2, &m_phase_now_observer);
		}
        
        // 3.1.5 根据观测器对相位进行校正
        m_motor_state.phase = correct_encoder(m_phase_now_observer, m_phase_now_encoder, m_pll_speed);
        

        if (!m_phase_override) {
            id_set_tmp = 0.0f;
        }
        
		if (m_phase_override) {
			m_motor_state.phase = m_phase_now_override;
		}
        
		// 3.1.6 电流限制
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

        // 3.1.7 运行电流环
		control_current(&m_motor_state, dt);
    } else {
        // 3.2 非运行状态
		// 3.2.1 Track back emf
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

		// 3.2.2 Update corresponding modulation
		m_motor_state.mod_d = m_motor_state.vd / ((2.0f / 3.0f) * m_motor_state.v_bus);
		m_motor_state.mod_q = m_motor_state.vq / ((2.0f / 3.0f) * m_motor_state.v_bus);

		// 3.2.3 The current is 0 when the motor is undriven
		m_motor_state.i_alpha = 0.0f;
		m_motor_state.i_beta = 0.0f;
		m_motor_state.id = 0.0f;
		m_motor_state.iq = 0.0f;
		m_motor_state.id_filter = 0.0f;
		m_motor_state.iq_filter = 0.0f;
		m_motor_state.i_bus = 0.0f;
		m_motor_state.i_abs = 0.0f;
		m_motor_state.i_abs_filter = 0.0f;

		// 3.2.4 Run observer
		observer_update(m_motor_state.v_alpha, m_motor_state.v_beta,
				m_motor_state.i_alpha, m_motor_state.i_beta, dt, &m_observer_x1,
				&m_observer_x2, &m_phase_now_observer);

        m_motor_state.phase = correct_encoder(m_phase_now_observer, m_phase_now_encoder, m_pll_speed); 
    }
    
	// 4. Calculate duty cycle
	m_motor_state.duty_now = SIG(m_motor_state.vq) *
			sqrtf(m_motor_state.mod_d * m_motor_state.mod_d +
					m_motor_state.mod_q * m_motor_state.mod_q) / SQRT3_BY_2;

	// 5. Run PLL for speed estimation
	pll_run(m_motor_state.phase, dt, &m_pll_phase, &m_pll_speed);

    // 7. 位置更新
    //encoder is configured
	float angle_now = 0.0f;
    angle_now = enc_ang;
    
    static float angle_last = 0.0f;
    float diff_f = utils_angle_difference(angle_now, angle_last);
    angle_last = angle_now;
    m_pos_pid_now += diff_f / m_conf->p_pid_ang_div;
    m_pos_now += diff_f / m_conf->p_pid_ang_div;
    utils_norm_angle((float*)&m_pos_pid_now);

	// 8. Run position control
	if (m_state == MC_STATE_RUNNING && m_control_mode == CONTROL_MODE_ANGLE) {
		run_pid_control_angle(m_pos_pid_now, m_pos_pid_set, dt);
	}

	// 9. MCIF handler
	mc_interface_mc_timer_isr();

	last_inj_adc_isr_duration = (float) TIM12->CNT / 10000000.0f;
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
}


void DMA2_Stream4_IRQHandler(void) {
    OSIntEnter();
    
    if(DMA_GetITStatus(DMA2_Stream4, DMA_IT_TCIF4)) {
        mcpwm_foc_tim_sample_int_handler();
        DMA_ClearITPendingBit(DMA2_Stream4, DMA_IT_TCIF4);
    }
        
    OSIntExit();
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
    // 1. 计算相位角的三角函数值
	float c,s;
	utils_fast_sincos_better(state_m->phase, &s, &c);

    // 2. 最大占空比限制
	float max_duty = fabsf(state_m->max_duty);
	utils_truncate_number(&max_duty, 0.0f, m_conf->l_max_duty);

    // 3. Park Tr, 计算 Id & Iq
	state_m->id = c * state_m->i_alpha + s * state_m->i_beta;
	state_m->iq = c * state_m->i_beta  - s * state_m->i_alpha;
	UTILS_LP_FAST(state_m->id_filter, state_m->id, m_conf->foc_current_filter_const);
	UTILS_LP_FAST(state_m->iq_filter, state_m->iq, m_conf->foc_current_filter_const);

    // 4. 计算误差值
	float Ierr_d = state_m->id_target - state_m->id;
	float Ierr_q = state_m->iq_target - state_m->iq;

    // 5. 通过PI控制器计算 Uq & Ud
	state_m->vd = state_m->vd_int + Ierr_d * m_conf->foc_current_kp;
	state_m->vq = state_m->vq_int + Ierr_q * m_conf->foc_current_kp;

	// 6. Temperature compensation
	const float t = mc_interface_temp_motor_filtered();
	float ki = m_conf->foc_current_ki;
	if (m_conf->foc_temp_comp && t > -5.0f) {
		ki += ki * 0.00386f * (t - m_conf->foc_temp_comp_base_temp);
	}

	state_m->vd_int += Ierr_d * (ki * dt);
	state_m->vq_int += Ierr_q * (ki * dt);

	// 7. 电压饱和限制
	utils_saturate_vector_2d((float*)&state_m->vd, (float*)&state_m->vq,
			(2.0f / 3.0f) * max_duty * SQRT3_BY_2 * state_m->v_bus);

	state_m->mod_d = state_m->vd / ((2.0f / 3.0f) * state_m->v_bus);
	state_m->mod_q = state_m->vq / ((2.0f / 3.0f) * state_m->v_bus);

	// 8. Windup protection
	utils_truncate_number_abs((float*)&state_m->vd_int, (2.0f / 3.0f) * max_duty * SQRT3_BY_2 * state_m->v_bus);
	utils_truncate_number_abs((float*)&state_m->vq_int, (2.0f / 3.0f) * max_duty * SQRT3_BY_2 * state_m->v_bus);

	state_m->i_bus = state_m->mod_d * state_m->id + state_m->mod_q * state_m->iq;
	state_m->i_abs = sqrtf(SQ(state_m->id) + SQ(state_m->iq));
	state_m->i_abs_filter = sqrtf(SQ(state_m->id_filter) + SQ(state_m->iq_filter));

    // 9. Inv. Park Tr
	float mod_alpha = c * state_m->mod_d - s * state_m->mod_q;
	float mod_beta  = c * state_m->mod_q + s * state_m->mod_d;

	// 10. Deadtime compensation
	const float i_alpha_filter = c * state_m->id_target - s * state_m->iq_target;
	const float i_beta_filter = c * state_m->iq_target + s * state_m->id_target;
	const float ia_filter = i_alpha_filter;
	const float ib_filter = -0.5f * i_alpha_filter + SQRT3_BY_2 * i_beta_filter;
	const float ic_filter = -0.5f * i_alpha_filter - SQRT3_BY_2 * i_beta_filter;
	const float mod_alpha_filter_sgn = (2.0f / 3.0f) * SIG(ia_filter) - (1.0f / 3.0f) * SIG(ib_filter) - (1.0f / 3.0f) * SIG(ic_filter);
	const float mod_beta_filter_sgn = ONE_BY_SQRT3 * SIG(ib_filter) - ONE_BY_SQRT3 * SIG(ic_filter);
	const float mod_comp_fact = m_conf->foc_dt_us * 1e-6f * m_conf->foc_f_sw;
	const float mod_alpha_comp = mod_alpha_filter_sgn * mod_comp_fact;
	const float mod_beta_comp = mod_beta_filter_sgn * mod_comp_fact;

	// Apply compensation here so that 0 duty cycle has no glitches.
	state_m->v_alpha = (mod_alpha - mod_alpha_comp) * (2.0f / 3.0f) * state_m->v_bus;
	state_m->v_beta = (mod_beta - mod_beta_comp) * (2.0f / 3.0f) * state_m->v_bus;

	// 11. 输出占空比
	uint32_t duty1, duty2, duty3, top;
	top = TIM1->ARR;
	svm(-mod_alpha, -mod_beta, top, &duty1, &duty2, &duty3, (uint32_t*)&state_m->svm_sector);
	TIMER_UPDATE_DUTY(duty1, duty2, duty3);

	if (!m_output_on) {
		start_pwm_hw();
	}
}

// Magnitude must not be larger than sqrt(3)/2, or 0.866f
static void svm(float alpha, float beta, uint32_t PWMHalfPeriod,
	uint32_t* tAout, uint32_t* tBout, uint32_t* tCout, uint32_t *svm_sector) {
	uint32_t sector;

    // 1. 根据电压空间矢量图判断扇区
	if (beta >= 0.0f) {
		if (alpha >= 0.0f) {
			//quadrant I
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 2;
			} else {
				sector = 1;
			}
		} else {
			//quadrant II
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 3;
			} else {
				sector = 2;
			}
		}
	} else {
		if (alpha >= 0.0f) {
			//quadrant IV
			if (-ONE_BY_SQRT3 * beta > alpha) {
				sector = 5;
			} else {
				sector = 6;
			}
		} else {
			//quadrant III
			if (ONE_BY_SQRT3 * beta > alpha) {
				sector = 4;
			} else {
				sector = 5;
			}
		}
	}

    // 2. 由电压推导占空比
	// PWM timings
	uint32_t tA, tB, tC;

	switch (sector) {

	// sector 1-2
	case 1: {
		// Vector on-times
		uint32_t t1 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t2 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tA = (PWMHalfPeriod - t1 - t2) / 2;
		tB = tA + t1;
		tC = tB + t2;

		break;
	}

	// sector 2-3
	case 2: {
		// Vector on-times
		uint32_t t2 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t3 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tB = (PWMHalfPeriod - t2 - t3) / 2;
		tA = tB + t3;
		tC = tA + t2;

		break;
	}

	// sector 3-4
	case 3: {
		// Vector on-times
		uint32_t t3 = (TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t4 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tB = (PWMHalfPeriod - t3 - t4) / 2;
		tC = tB + t3;
		tA = tC + t4;

		break;
	}

	// sector 4-5
	case 4: {
		// Vector on-times
		uint32_t t4 = (-alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t5 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tC = (PWMHalfPeriod - t4 - t5) / 2;
		tB = tC + t5;
		tA = tB + t4;

		break;
	}

	// sector 5-6
	case 5: {
		// Vector on-times
		uint32_t t5 = (-alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t6 = (alpha - ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tC = (PWMHalfPeriod - t5 - t6) / 2;
		tA = tC + t5;
		tB = tA + t6;

		break;
	}

	// sector 6-1
	case 6: {
		// Vector on-times
		uint32_t t6 = (-TWO_BY_SQRT3 * beta) * PWMHalfPeriod;
		uint32_t t1 = (alpha + ONE_BY_SQRT3 * beta) * PWMHalfPeriod;

		// PWM timings
		tA = (PWMHalfPeriod - t6 - t1) / 2;
		tC = tA + t1;
		tB = tC + t6;

		break;
	}
	}

	*tAout = tA;
	*tBout = tB;
	*tCout = tC;
	*svm_sector = sector;
}
        
static void run_pid_control_speed(float dt) {
	static float i_term = 0.0f;
	static float prev_error = 0.0f;
	float p_term;
	float d_term;

	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_SPEED) {
		i_term = 0.0f;
		prev_error = 0.0f;
		return;
	}

	const float rpm = mcpwm_foc_get_rpm_now();
	float error = m_speed_pid_set_rpm - rpm;

	// Too low RPM set. Reset state and return.
	if (fabsf(m_speed_pid_set_rpm) < m_conf->s_pid_min_erpm) {
		i_term = 0.0f;
		prev_error = error;
		return;
	}

	// Compute parameters
	p_term = error * m_conf->s_pid_kp * (1.0f / 20.0f);
	i_term += error * (m_conf->s_pid_ki * dt) * (1.0f / 20.0f);
	d_term = (error - prev_error) * (m_conf->s_pid_kd / dt) * (1.0f / 20.0f);

	// Filter D
	static float d_filter = 0.0f;
	UTILS_LP_FAST(d_filter, d_term, m_conf->s_pid_kd_filter);
	d_term = d_filter;

	// I-term wind-up protection
	utils_truncate_number(&i_term, -1.0f, 1.0f);

	// Store previous error
	prev_error = error;

	// Calculate output
	float output = p_term + i_term + d_term;
	utils_truncate_number(&output, -1.0f, 1.0f);

	m_iq_set = output * m_conf->lo_current_max;
}

static void run_pid_control_angle(float angle_now, float angle_set, float dt) {
	static float i_term = 0;
	static float prev_error = 0;
	float p_term;
	float d_term;

	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_ANGLE) {
		i_term = 0;
		prev_error = 0;
		return;
	}

	// Compute parameters
	float error = utils_angle_difference(angle_set, angle_now);

    if (m_conf->foc_encoder_inverted) {
        error = -error;
    }

	p_term = error * m_conf->p_pid_kp;
	i_term += error * (m_conf->p_pid_ki * dt);

	static float dt_int = 0.0f;
	dt_int += dt;
	if (error == prev_error) {
		d_term = 0.0f;
	} else {
		d_term = (error - prev_error) * (m_conf->p_pid_kd / dt_int);
		dt_int = 0.0f;
	}

	// Filter D
	static float d_filter = 0.0f;
	UTILS_LP_FAST(d_filter, d_term, m_conf->p_pid_kd_filter);
	d_term = d_filter;


	// I-term wind-up protection
	utils_truncate_number_abs(&p_term, 1.0f);
	utils_truncate_number_abs(&i_term, 1.0f - fabsf(p_term));

	// Store previous error
	prev_error = error;

	// Calculate output
	float output = p_term + i_term + d_term;
	utils_truncate_number(&output, -1.0f, 1.0f);

    if (encoder_index_found()) {
        m_iq_set = output * m_conf->lo_current_max;
    } else {
        // Rotate the motor with 40 % power until the encoder index is found.
        m_iq_set = 0.4f * m_conf->lo_current_max;
    }
    
    mc_interface_current_feedforward(&m_iq_set);
}

static void run_pid_control_pos(float pos_now, float pos_set, float dt) {
	static float i_term = 0;
	static float prev_error = 0;
	float p_term;
	float d_term;
    
	// PID is off. Return.
	if (m_control_mode != CONTROL_MODE_ANGLE) {
		i_term = 0;
		prev_error = 0;
		return;
	}
    
    // Compute parameters
    float error = pos_set - pos_now;
    //反向？
   
	p_term = error * m_conf->p_pid_kp;
	i_term += error * (m_conf->p_pid_ki * dt);

	static float dt_int = 0.0f;
	dt_int += dt;
	if (error == prev_error) {
		d_term = 0.0f;
	} else {
		d_term = (error - prev_error) * (m_conf->p_pid_kd / dt_int);
		dt_int = 0.0f;
	}

	// Filter D
	static float d_filter = 0.0f;
	UTILS_LP_FAST(d_filter, d_term, m_conf->p_pid_kd_filter);
	d_term = d_filter;


	// I-term wind-up protection
	utils_truncate_number_abs(&p_term, 1.0f);
	utils_truncate_number_abs(&i_term, 1.0f - fabsf(p_term));

	// Store previous error
	prev_error = error;

	// Calculate output
	float output = p_term + i_term + d_term;
	utils_truncate_number(&output, -1.0f, 1.0f);
    
    m_speed_pid_set_rpm = output * m_pos_max_limit;
    
    run_pid_control_speed(dt);
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
    OSIntEnter();
    
	ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    
    OSIntExit();
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
