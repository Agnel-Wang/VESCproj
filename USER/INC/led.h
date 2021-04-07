#ifndef _LED_H
#define _LED_H

#include "datatypes.h"
#include "sys.h"
#include "delay.h"

#define LED_GPIO                GPIOB
#define LED_GREEN_PIN           PIN0
#define LED_RED_PIN             PIN1
#define LED_BLUE_PIN            PIN2

#define LED_BLUE_OFF()  	    LED_GPIO->BSRRL = LED_BLUE_PIN
#define LED_RED_OFF()  			LED_GPIO->BSRRL = LED_RED_PIN
#define LED_GREEN_OFF()  		LED_GPIO->BSRRL = LED_GREEN_PIN

#define LED_RED_ON()   			LED_GPIO->BSRRH = LED_RED_PIN
#define LED_BLUE_ON()   		LED_GPIO->BSRRH = LED_BLUE_PIN
#define LED_GREEN_ON()   		LED_GPIO->BSRRH = LED_GREEN_PIN

#define LED_RED_TOGGLE()		LED_GPIO->ODR ^= LED_RED_PIN
#define LED_BLUE_TOGGLE()		LED_GPIO->ODR ^= LED_BLUE_PIN
#define LED_GREEN_TOGGLE()	    LED_GPIO->ODR ^= LED_GREEN_PIN

void LEDInit(void);
void Led_Show(void);
void Task_LED(void *pdata);

#endif
