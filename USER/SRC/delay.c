#include "delay.h"

void wait_ms(uint16_t t) {
	for(int i=0;i<t;i++) {
		int a=42000;
		while(a--);
	}
}

void wait_us(uint16_t t) {
	for(int i=0;i<t;i++) {
		int a=42;
		while(a--);
	}
}

void delay_ms(uint16_t ms) {
    uint32_t ticks = ms * (OS_TICKS_PER_SEC / 1000.0f);
    OSTimeDly(ticks);
}


void delay(uint16_t sec) {
    uint32_t ticks = sec * OS_TICKS_PER_SEC;
    OSTimeDly(ticks);
}