#include "drv8301.h"

#include "stm32f4xx_conf.h"
#include "hw_60.h"
#include "sys.h"
#include "delay.h"
#include "utils.h"

//Private variables
static unsigned int reg;

// Private functions
static uint16_t spi_exchange(uint16_t x);
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);

void drv8301_init(void) {
    //DRV8301 SPI
    GPIO_Set(DRV8301_MISO_GPIO, DRV8301_MISO_PIN, GPIO_MODE_IN, GPIO_OTYPE_PP, GPIO_SPEED_100M,GPIO_PUPD_NONE);
    GPIO_Set(DRV8301_SCK_GPIO, DRV8301_SCK_PIN, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU);
    GPIO_Set(DRV8301_CS_GPIO, DRV8301_CS_PIN, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU);
    GPIO_Set(DRV8301_MOSI_GPIO, DRV8301_MOSI_PIN, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU);
    
    delay_ms(100);
    
	// Disable OC
	drv8301_write_reg(2, 0x0430);
	drv8301_write_reg(2, 0x0430);
    
    drv8301_set_oc_adj(1); // 1 -- 14.468A
    drv8301_reset_faults();
}

/**
 * Set the threshold of the over current protection of the DRV8301. It works by measuring
 * the voltage drop across drain-source of the MOSFETs and activates when it is higher than
 * a set value. Notice that this current limit is not very accurate.
 *
 * @param val
 * The value to use. Range [0 31]. A lower value corresponds to a lower current limit. See
 * the drv8301 datasheet for how to convert these values to currents.
 *
 * Overcurrent Trip = OC_ADJ_SET / MOSFET R_DS(on)
 * where MOSFET R_DS(on)  = 4.7mOhm
 */
void drv8301_set_oc_adj(int val) {
	int reg = drv8301_read_reg(2);
	reg &= 0x003F;
	reg |= (val & 0x1F) << 6;
	drv8301_write_reg(2, reg);
}

/**
 * Set the over current protection mode of the DRV8301.
 *
 * @param mode
 * The over current protection mode.
 */
void drv8301_set_oc_mode(drv8301_oc_mode mode) {
	int reg = drv8301_read_reg(2);
	reg &= 0xFFCF;
	reg |= (mode & 0x03) << 4;
	drv8301_write_reg(2, reg);
}

/**
 * Read the fault codes of the DRV8301.
 *
 * @return
 * The fault codes, where the bits represent the following:
 * b0: FETLC_OC
 * b1: FETHC_OC
 * b2: FETLB_OC
 * b3: FETHB_OC
 * b4: FETLA_OC
 * b5: FETHA_OC
 * b6: OTW
 * b7: OTSD
 * b8: PVDD_UV
 * b9: GVDD_UV
 * b10: FAULT
 * b11: GVDD_OV
 *
 */
int drv8301_read_faults(void) {
	int r0 = drv8301_read_reg(0);
	int r1 = drv8301_read_reg(1);
	return r0 | (((r1 >> 7) & 0x01) << 4);
}

/**
 * Reset all latched faults.
 */
void drv8301_reset_faults(void) {
	int reg = drv8301_read_reg(2);
	reg |= 1 << 2;
	drv8301_write_reg(2, reg);
}

unsigned int drv8301_read_reg(int reg) {
	uint16_t out = 0;
	out |= (1 << 15);
	out |= (reg & 0x0F) << 11;
	out |= 0x807F;

	if (reg != 0) {
		spi_begin();
		spi_exchange(out);
		spi_end();
	}

	spi_begin();
	uint16_t res = spi_exchange(0xFFFF);
	spi_end();

	return res;
}

void drv8301_write_reg(int reg, int data) {
	uint16_t out = 0;
	out |= (reg & 0x0F) << 11;
	out |= data & 0x7FF;

	spi_begin();
	spi_exchange(out);
	spi_end();
}

// Software SPI
static uint16_t spi_exchange(uint16_t x) {
	uint16_t rx;
	spi_transfer(&rx, &x, 1);
	return rx;
}

static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0;i < length;i++) {
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;
		uint16_t recieve = 0;

		for (int bit = 0;bit < 16;bit++) {
            GPIO_WriteBit(DRV8301_MOSI_GPIO, DRV8301_MOSI_PIN, send >> 15);
			send <<= 1;

            GPIO_SetBits(DRV8301_SCK_GPIO, DRV8301_SCK_PIN);
			spi_delay();

			GPIO_ResetBits(DRV8301_SCK_GPIO, DRV8301_SCK_PIN);

			int r1, r2, r3;
			r1 = GPIO_ReadInputDataBit(DRV8301_MISO_GPIO, DRV8301_MISO_PIN);
			__NOP();
			r2 = GPIO_ReadInputDataBit(DRV8301_MISO_GPIO, DRV8301_MISO_PIN);
			__NOP();
			r3 = GPIO_ReadInputDataBit(DRV8301_MISO_GPIO, DRV8301_MISO_PIN);

			recieve <<= 1;
			if (utils_middle_of_3_int(r1, r2, r3)) {
				recieve |= 1;
			}

			spi_delay();
		}

		if (in_buf) {
			in_buf[i] = recieve;
		}
	}
}

static void spi_begin(void) {
	GPIO_ResetBits(DRV8301_CS_GPIO, DRV8301_CS_PIN);
}

static void spi_end(void) {
	GPIO_SetBits(DRV8301_CS_GPIO, DRV8301_CS_PIN);
}

static void spi_delay(void) {
	for (volatile int i = 0;i < 10;i++) {
		__NOP();
	}
}

