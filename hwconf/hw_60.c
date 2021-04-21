#include "hw_60.h"
#include "stm32f4xx_conf.h"
#include "led.h"
#include "drv8301.h"
#include "sys.h"

void hw_init_gpio(void) {
    // GPIO clock enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    
    // LEDs
    GPIO_Set(GPIOB,LED_RED_PIN|LED_GREEN_PIN|LED_BLUE_PIN,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_2M,GPIO_PUPD_NONE);
    
    //ENABLE_GATE
    GPIO_Set(GPIOB, PIN5, GPIO_MODE_OUT, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_PU);
    ENABLE_GATE();
    
    // GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
    GPIO_Set(GPIOA, PIN8|PIN9|PIN10, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_NONE);
    GPIO_AF_Set(GPIOA, 8, 1); //TIM1
    GPIO_AF_Set(GPIOA, 9, 1);
    GPIO_AF_Set(GPIOA, 10, 1);
    
    GPIO_Set(GPIOB, PIN13|PIN14|PIN15, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_NONE);
    GPIO_AF_Set(GPIOB, 13, 1); //TIM1
    GPIO_AF_Set(GPIOB, 14, 1);
    GPIO_AF_Set(GPIOB, 15, 1);
    
    //Fault Pin
    GPIO_Set(GPIOB, PIN7, GPIO_MODE_IN, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_PU);
    
    //ADC Pins
    GPIO_Set(GPIOA, PIN0|PIN1|PIN2|PIN3|PIN5|PIN6, GPIO_MODE_AIN, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_NONE);
    GPIO_Set(GPIOC, PIN0|PIN1|PIN2|PIN3|PIN4|PIN5, GPIO_MODE_AIN, GPIO_OTYPE_PP, GPIO_SPEED_100M, GPIO_PUPD_NONE);
    
    //CAN1
    GPIO_Set(GPIOB, PIN8|PIN9, GPIO_MODE_AF, GPIO_OTYPE_PP, GPIO_SPEED_25M, GPIO_PUPD_PU);
    GPIO_AF_Set(GPIOB, 8, 9);
    GPIO_AF_Set(GPIOB, 9, 9);
    
    drv8301_init();
}

void hw_setup_adc_channels(void) {
    // ADC1 regular channels
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);

    // ADC2 regular channels
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);

    // ADC3 regular channels
    ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);

    // Injected channels
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
}
