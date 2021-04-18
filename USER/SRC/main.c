#include "stm32f4xx_conf.h"
#include "includes.h"
#include "conf_general.h"
#include "hw_60.h"
#include "led.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"

/*
 * Timers used:
 * TIM1: mcpwm_out
 * TIM12: mcpwm
 * TIM8: mcpwm_sample
 * TIM3: servo_dec/Encoder (HW_R2)/servo_simple
 * TIM4: WS2811/WS2812 LEDs/Encoder (other HW)
 * TIM5: speed cycle
 *
 * DMA/stream   Device      Function
 * 1, 2         I2C1        Nunchuk, temp on rev 4.5f
 * 1, 7         I2C1        Nunchuk, temp on rev 4.5f
 * 1, 1         UART3       HW_R2
 * 1, 3         UART3       HW_R2
 * 2, 2         UART6       Other HW
 * 2, 7         UART6       Other HW
 * 2, 4         ADC         mcpwm
 * 1, 0         TIM4        WS2811/WS2812 LEDs CH1 (Ch 1)
 * 1, 3         TIM4        WS2811/WS2812 LEDs CH2 (Ch 2)
 *
 */

/****UCOSII Tasks Set****/
//START Task
#define START_TASK_PRIO         60
#define START_STK_SIZE      	256
__align(8) OS_STK START_TASK_STK[START_STK_SIZE];
static void Task_Start(void *pdata);

//Main Perioic
#define MAIN_TASK_PRIO          20
#define MAIN_STK_SIZE           1024
__align(8) OS_STK MAIN_TASK_STK[MAIN_STK_SIZE];
static void Task_Main(void* pdata);

//LCD Task
#define LED_TASK_PRIO       	55
#define LED_STK_SIZE     		512
__align(8) OS_STK LED_TASK_STK[LED_STK_SIZE];

OS_EVENT *RUN;

int main(void) {
    // 本杰明的MCO2PRE为4分频，RTCPRE为8分频，此处暂未更改
    // 修改 PLL_M = 8
	SystemInit();
    
    hw_init_gpio();
    
    mc_configuration mcconf;
    conf_general_get_default_mc_configuration(&mcconf);
    mc_interface_init(&mcconf); 
    
	OSInit();
	OSTaskCreate(Task_Start, NULL, &START_TASK_STK[START_STK_SIZE - 1], START_TASK_PRIO);
	OSStart();
}

static void Task_Start(void *pdata) {
	OS_CPU_SR cpu_sr = 0;
	OS_CPU_SysTickInit();
	OS_ENTER_CRITICAL();
  
    OSTaskCreate(Task_Main, NULL, (OS_STK *)&MAIN_TASK_STK[MAIN_STK_SIZE - 1],MAIN_TASK_PRIO);
    OSTaskCreate(Task_LED, NULL, (OS_STK *)&LED_TASK_STK[LED_STK_SIZE - 1],LED_TASK_PRIO);
  
    OSTaskSuspend(START_TASK_PRIO);
    OS_EXIT_CRITICAL();
}

static void Task_Main(void *pdata) {
    for(;;) {
        
        delay_ms(10);
    }
}
