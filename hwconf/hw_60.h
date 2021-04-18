#ifndef HW_60_H_
#define HW_60_H_

#include "sys.h"

// HW properties
#define HW_HAS_DRV8301
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS

#define HW_HAS_LED_BOOT1

// Macros
#define ENABLE_GATE()			GPIO_SetBits(GPIOB, PIN5)
#define DISABLE_GATE()			GPIO_ResetBits(GPIOB, PIN5)

#define DCCAL_ON()
#define DCCAL_OFF()
#define IS_DRV_FAULT()			(!GPIO_ReadInputDataBit(GPIOB, PIN7))

/*
 * ADC Vector
 *
 * 0:	IN0		SENS1
 * 1:	IN1		SENS2
 * 2:	IN2		SENS3
 * 3:	IN10	CURR1
 * 4:	IN11	CURR2
 * 5:	IN12	CURR3
 * 6:	IN5		ADC_EXT1
 * 7:	IN6		ADC_EXT2
 * 8:	IN3		TEMP_PCB
 * 9:	IN14	TEMP_MOTOR
 * 10:	IN15	ADC_EXT3
 * 11:	IN13	AN_IN
 * 12:	Vrefint
 * 13:	IN0		SENS1
 * 14:	IN1		SENS2
 */

#define HW_ADC_CHANNELS			15
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			5

// ADC Indexes
#define ADC_IND_SENS1			0
#define ADC_IND_SENS2			1
#define ADC_IND_SENS3			2
#define ADC_IND_CURR1			3
#define ADC_IND_CURR2			4
#define ADC_IND_CURR3			5
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VIN_SENS		11
#define ADC_IND_VREFINT			12

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3f
#endif
#ifndef VIN_R1
#define VIN_R1					39000.0f
#endif
#ifndef VIN_R2
#define VIN_R2					2200.0f
#endif
#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN		20.0f
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		0.0005f
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0f) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val)		((4095.0f * 10000.0f) / adc_val - 10000.0f)
#define NTC_TEMP(adc_ind)		(1.0f / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0f) / 3380.0f) + (1.0f / 298.15f)) - 273.15f)

#define NTC_RES_MOTOR(adc_val)	(10000.0f / ((4095.0f / (float)adc_val) - 1.0f)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0f / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0f) / beta) + (1.0f / 298.15f)) - 273.15f)

// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0f * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// Number of servo outputs
#define HW_SERVO_NUM			2

// UART Peripheral
#define HW_UART_DEV				UARTD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			PIN10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			PIN11

// ICU Peripheral for servo decoding
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				PIN6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			PIN10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			PIN11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		PIN6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		PIN7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		PIN8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// NRF pins
#define NRF_PORT_CSN			GPIOB
#define NRF_PIN_CSN				PIN12
#define NRF_PORT_SCK			GPIOB
#define NRF_PIN_SCK				PIN4
#define NRF_PORT_MOSI			GPIOB
#define NRF_PIN_MOSI			PIN3
#define NRF_PORT_MISO			GPIOD
#define NRF_PIN_MISO			PIN2

// SPI pins
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			PIN4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			PIN5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			PIN7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			PIN6

// SPI for DRV8301
#define DRV8301_MOSI_GPIO		GPIOC
#define DRV8301_MOSI_PIN		PIN12
#define DRV8301_MISO_GPIO		GPIOC
#define DRV8301_MISO_PIN		PIN11
#define DRV8301_SCK_GPIO		GPIOC
#define DRV8301_SCK_PIN			PIN10
#define DRV8301_CS_GPIO			GPIOC
#define DRV8301_CS_PIN			PIN9

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default setting overrides
#ifndef MCCONF_L_MAX_ABS_CURRENT
#define MCCONF_L_MAX_ABS_CURRENT		150.0f	// The maximum absolute current above which a fault is generated
#endif
#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7			true	// Run control loop in both v0 and v7 (requires phase shunts)
#endif

// Setting limits
#define HW_LIM_CURRENT			-120.0f, 120.0f
#define HW_LIM_CURRENT_IN		-120.0f, 120.0f
#define HW_LIM_CURRENT_ABS		0.0f, 160.0f
#define HW_LIM_VIN				6.0f, 57.0f
#define HW_LIM_ERPM				-200e3f,	200e3f
#define HW_LIM_DUTY_MIN			0.0f, 0.1f
#define HW_LIM_DUTY_MAX			0.0f, 0.99f
#define HW_LIM_TEMP_FET			-40.0f, 110.0f

void hw_init_gpio(void);
void hw_setup_adc_channels(void);

#endif /* HW_60_H_ */
