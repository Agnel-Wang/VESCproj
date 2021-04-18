#include "led.h"

void Led_Show(void) {
	LED_GREEN_ON(); delay_ms(500);
	LED_RED_ON();   delay_ms(500);
	LED_BLUE_ON();  delay_ms(500);
	LED_GREEN_OFF();delay_ms(500);
	LED_RED_OFF();  delay_ms(500);
	LED_BLUE_OFF(); delay_ms(500);
}

void Task_LED(void *pdata) {
  for(;;){
    Led_Show();
    delay_ms(200);
  }
}

