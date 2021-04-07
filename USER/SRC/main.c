#include "stm32f4xx_conf.h"
#include "includes.h"
#include "conf_general.h"
#include "hw_60.h"
#include "led.h"
#include "mc_interface.h"

/****UCOSII Tasks Set****/
//START Task
#define START_TASK_PRIO         60
#define START_STK_SIZE      	256
__align(8) OS_STK START_TASK_STK[START_STK_SIZE];
static void Task_Start(void *pdata);

//LCD Task
#define LED_TASK_PRIO       	25
#define LED_STK_SIZE     		512
__align(8) OS_STK LED_TASK_STK[LED_STK_SIZE];

OS_EVENT *RUN;

int main(void) {
	SystemInit();
        
    hw_init_gpio();
    
    mc_configuration mcconf;
    conf_general_get_default_mc_configuration(&mcconf);
    mc_interface_init(&mcconf); 
    
    GPIO_ResetBits(GPIOA, PIN8);
    GPIO_ResetBits(GPIOA, PIN9);
    GPIO_ResetBits(GPIOA, PIN10);
    GPIO_ResetBits(GPIOB, PIN13);
    GPIO_ResetBits(GPIOB, PIN14);
    GPIO_ResetBits(GPIOB, PIN15);
    
	OSInit();
	OSTaskCreate(Task_Start, (void *)0, &START_TASK_STK[START_STK_SIZE - 1], START_TASK_PRIO);
	OSStart();
}

static void Task_Start(void *pdata) {
	OS_CPU_SR cpu_sr = 0;
	OS_CPU_SysTickInit();
	OS_ENTER_CRITICAL(); //进入临界区（无法被中断打断）
  
    OSTaskCreate(Task_LED, NULL, (OS_STK *)&LED_TASK_STK[LED_STK_SIZE - 1],LED_TASK_PRIO);
  
    OSTaskSuspend(START_TASK_PRIO); //挂起起始任务
    OS_EXIT_CRITICAL();             //退出临界区
}
