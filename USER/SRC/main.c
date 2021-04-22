#include "stm32f4xx_conf.h"
#include "includes.h"
#include "conf_general.h"
#include "hw_60.h"
#include "led.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "common_can.h"

/*
 * Timers used:
 * TIM1: mcpwm_out
 * TIM3: Encoder
 * TIM4: CAN1
 * TIM5: speed cycle
 * TIM8: mcpwm_sample
 * TIM12: 计算单次电流环持续时间
 *
 * DMA/stream   Device      Function
 * 2, 4         ADC         mcpwm
 *
 * Peripherals              NIVC
 * DMA2_Stream4_IRQn        3
 * TIM8_CC_IRQn             6
 * TIM3_IRQn                6
 * TIM5_IRQn                7
 * TIM4_IRQn                8
 * CAN1_RX0_IRQn            11
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
    // 本杰明的MCO2PRE为4分频，RTCPRE为8分频，暂未更改
    // 修改 PLL_M = 8
    // 但目前因此UCOS的时钟慢了25/8,待改
	SystemInit();
    
    hw_init_gpio();
    
    mc_configuration mcconf;
    conf_general_get_default_mc_configuration(&mcconf);
    mc_interface_init(&mcconf); 
    
#if CAN_ENABLE
    comm_can_init();
#endif

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
