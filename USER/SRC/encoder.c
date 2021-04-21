#include "encoder.h"
#include "stm32f4xx_tim.h"
#include "hw_60.h"
#include "sys.h"
#include "utils.h"
#include "conf_general.h"

#define AS5047_SAMPLE_RATE_HZ		20000

#define SPI_SW_MISO_GPIO			GPIOC
#define SPI_SW_MISO_PIN				PIN7
#define SPI_SW_SCK_GPIO				GPIOC
#define SPI_SW_SCK_PIN				PIN6
#define SPI_SW_CS_GPIO				GPIOC
#define SPI_SW_CS_PIN				PIN8

// Private variables
static bool index_found = false;
static float last_enc_angle = 0.0f;

// Private functions
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);

void encoder_init(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
    GPIO_Set(SPI_SW_MISO_GPIO,SPI_SW_MISO_PIN,GPIO_MODE_IN,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_NONE);
    GPIO_Set(SPI_SW_SCK_GPIO,SPI_SW_SCK_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
    GPIO_Set(SPI_SW_CS_GPIO,SPI_SW_CS_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
    
    // Enable timer clock
    HW_ENC_TIM_CLK_EN();
    
    // Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = ((SYSTEM_CORE_CLOCK / 2 / AS5047_SAMPLE_RATE_HZ) - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(HW_ENC_TIM, &TIM_TimeBaseStructure);
    
    // Enable overflow interrupt
	TIM_ITConfig(HW_ENC_TIM, TIM_IT_Update, ENABLE);
    
    // Enable timer
	TIM_Cmd(HW_ENC_TIM, ENABLE);
    
    nvicEnableVector(HW_ENC_TIM_ISR_CH, 6);
    
    index_found = true;
}

/**
 * Timer interrupt
 */
void encoder_tim_isr(void) {
	uint16_t pos;

	spi_begin();
	spi_transfer(&pos, 0, 1);
	spi_end();

	pos &= 0x3FFF;
	last_enc_angle = ((float)pos * 360.0f) / 16384.0f;
}

// Software SPI
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t recieve = 0;

		for (int bit = 0;bit < 16;bit++) {
			send <<= 1;

			spi_delay();
            GPIO_SetBits(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();

			int r1, r2, r3;
			r1 = GPIO_ReadInputDataBit(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r2 = GPIO_ReadInputDataBit(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r3 = GPIO_ReadInputDataBit(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);

			recieve <<= 1;
			if (utils_middle_of_3_int(r1, r2, r3)) {
				recieve |= 1;
			}

			GPIO_ResetBits(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
            
			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

static void spi_begin(void) {
    GPIO_ResetBits(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_end(void) {
    GPIO_SetBits(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

static void spi_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}

float encoder_read_deg(void) {
    return last_enc_angle;
}

/**
 * Check if the index pulse is found.
 *
 * @return
 * True if the index is found, false otherwise.
 */
bool encoder_index_found(void) {
	return index_found;
}

//encoder interrupt service routine
void HW_ENC_TIM_ISR_VEC() {
    if (TIM_GetITStatus(HW_ENC_TIM, TIM_IT_Update) != RESET) {
    encoder_tim_isr();
        
    // Clear the IT pending bit
    TIM_ClearITPendingBit(HW_ENC_TIM, TIM_IT_Update);
    }
}