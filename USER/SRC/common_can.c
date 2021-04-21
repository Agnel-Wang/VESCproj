#include "common_can.h"
#include "sys.h"
#include "datatypes.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_tim.h"
#include "conf_general.h"
#include "mc_interface.h"
#include "mcpwm_foc.h"
#include "buffer.h"
#include "stdarg.h"

// Settings
#define RX_FRAMES_SIZE	50
#define CAN_SEN_RATE_PSC 2

// Variables
static CanRxMsg rx_frames[RX_FRAMES_SIZE];
static volatile uint8_t rx_frames_front;
static volatile uint8_t rx_frames_rear;

// Private functions
void canReceive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage);
void canProcess(const CanRxMsg RxMessage);
void canAnswer(uint8_t *data, const CanRxMsg RxMessage);
void comm_can_transmit(uint8_t *data, uint8_t len);

void comm_can_init(void) {
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    
    rx_frames_front = 0;
    rx_frames_rear = 0;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    
    /* CAN cell init */
    CAN_InitStructure.CAN_TTCM = DISABLE; //非时间触发通道模式
    CAN_InitStructure.CAN_ABOM = DISABLE; //软件对CAN_MCR寄存器的INRQ位置1，随后清0，一旦监测到128次连续11位的隐性位，就退出离线状态
    CAN_InitStructure.CAN_AWUM = DISABLE; //睡眠模式由软件唤醒
    CAN_InitStructure.CAN_NART = DISABLE; //禁止报文自动发送，即只发送一次，无论结果如何
    CAN_InitStructure.CAN_RFLM = DISABLE; //报文不锁定，新的覆盖旧的
    CAN_InitStructure.CAN_TXFP = DISABLE; //发送FIFO的优先级由标识符决定
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // CAN硬件工作在正常模式

    /* Seting BaudRate */
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq; //重新同步跳跃宽度为一个时间单位
    CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq; //时间段1占用8个时间单位
    CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq; //时间段2占用7个时间单位
    CAN_InitStructure.CAN_Prescaler = 3;     //分频系数（Fdiv）
    CAN_Init(CAN1, &CAN_InitStructure);      //初始化CAN1
    
    /* CAN filter init */
    for(int i=0; i<9; i++) {
        CAN_FilterInitStructure.CAN_FilterNumber = i;
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
        CAN_FilterInitStructure.CAN_FilterIdHigh =(((APPCONF_CONTROLLER_NODE + i)<< 3) & 0xFFFF0000) >> 16;
        CAN_FilterInitStructure.CAN_FilterIdLow = ((APPCONF_CONTROLLER_NODE + i)<< 3) & 0xFFFF;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (0xFFFF00 << 3) >> 16;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = (0xFFFF << 3) & 0xFFFF;
        CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0;
        CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
        CAN_FilterInit(&CAN_FilterInitStructure);
    }
    
	TIM_DeInit(TIM7);
	TIM7->CNT = 0;
    
    // Time Base configuration
	TIM_TimeBaseStructure.TIM_Prescaler = (CAN_SEN_RATE_PSC - 1);
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = (SYSTEM_CORE_CLOCK / 2 / CAN_SEN_RATE_PSC / APPCONF_SEND_CAN_STATUS_RATE_HZ - 1);
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    
    // Enable overflow interrupt
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    nvicEnableVector(TIM7_IRQn, 10);
    nvicEnableVector(CAN1_RX0_IRQn, 11);
    
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/**
 * CAN报文接收函数
 * 将收到的报文存入队列内
 */
void canReceive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage) {
    static CanRxMsg rx_msg; 
    if(((rx_frames_front + 1) % RX_FRAMES_SIZE) == rx_frames_rear) { // 队满
        return;
    } else {
        CAN_Receive(CANx, FIFONumber, &rx_msg);
        *RxMessage = rx_msg;
        rx_frames_front = (rx_frames_front + 1) % RX_FRAMES_SIZE;
    }
}

/**
 * CAN报文处理函数
 * 依次处理队列内的报文
 */
void canProcess(const CanRxMsg RxMessage) {
    uint8_t buffer[4];
    CanRxMsg rx_msg; 
    rx_msg = RxMessage;
    if(rx_frames_front == rx_frames_rear) { // 队空
        return;
    } else {
        if((rx_msg.IDE = CAN_Id_Standard) && (rx_msg.RTR == CAN_RTR_DATA)) {
            uint16_t m_id = rx_msg.StdId - APPCONF_CONTROLLER_NODE;
            if(m_id == 0 || m_id == APPCONF_CONTROLLER_ID) {
                if(rx_msg.Data[0] == 'U' && rx_msg.Data[1] == 'M') {
                    if(rx_msg.DLC == 4) {
                        int32_t index = 0;
                        uint32_t mode = mc_interface_get_control_mode();
                        buffer_append_uint32(buffer, mode, &index);
                        canAnswer(buffer, rx_msg);
                    } else if(rx_msg.DLC == 8) {
                        int32_t index = 4;
                        int32_t mode = buffer_get_int32(rx_msg.Data, &index);
                        mc_interface_set_control_mode(mode);
                    }
                } else if(rx_msg.Data[0] == 'B' && rx_msg.Data[1] == 'G') {
                    //暂未实现
                } else if(rx_msg.Data[0] == 'J' && rx_msg.Data[1] == 'V') {
                    if(rx_msg.DLC == 4) {
                        int32_t index = 0;
                        float pos = mc_interface_get_pos_set();
                        buffer_append_float32(buffer, pos, &index);
                        canAnswer(buffer, rx_msg);
                    } else if(rx_msg.DLC == 8) {
                        int32_t index = 4;
                        float rpm = buffer_get_float32(rx_msg.Data, &index);
                        mc_interface_set_pos_set(rpm);
                    }
                } else if(rx_msg.Data[0] == 'M' && rx_msg.Data[1] == 'O') {
                    if(rx_msg.DLC == 4) {
                        int32_t index = 0;
                        uint32_t state = mcpwm_foc_get_state();
                        buffer_append_uint32(buffer, state, &index);
                        canAnswer(buffer, rx_msg);
                    } else if(rx_msg.DLC == 8) {
                        int32_t index = 4;
                        mc_state state = buffer_get_int32(rx_msg.Data, &index);
                        mcpwm_foc_set_state(state);
                    }
                } else if(rx_msg.Data[0] == 'P' && rx_msg.Data[1] == 'A') {
                    if(rx_msg.DLC == 4) {
                        int32_t index = 0;
                        float pos = mc_interface_get_pos_set();
                        buffer_append_float32(buffer, pos, &index);
                        canAnswer(buffer, rx_msg);
                    } else if(rx_msg.DLC == 8) {
                        int32_t index = 4;
                        float pos = buffer_get_float32(rx_msg.Data, &index);
                        mc_interface_set_pos_set(pos);
                    }
                } else if(rx_msg.Data[0] == 'P' && rx_msg.Data[1] == 'X') {
                    if(rx_msg.DLC == 4) {
                        int32_t index = 0;
                        float pos = mc_interface_get_pos_now();
                        buffer_append_float32(buffer, pos, &index);
                        canAnswer(buffer, rx_msg);
                    } else if(rx_msg.DLC == 8) {
                        int32_t index = 4;
                        float pos = buffer_get_float32(rx_msg.Data, &index);
                        mc_interface_set_pos_now(pos);
                    }
                } else if(rx_msg.Data[0] == 'S' && rx_msg.Data[1] == 'P') {
                    if(rx_msg.DLC == 4) {
                        int32_t index = 0;
                        float rpm = mc_interface_get_rpm_set();
                        buffer_append_float32(buffer, rpm, &index);
                        canAnswer(buffer, rx_msg);
                    } else if(rx_msg.DLC == 8) {
                        int32_t index = 4;
                        float rpm = buffer_get_float32(rx_msg.Data, &index);
                        mc_interface_set_pid_speed(rpm);
                    }
                } else if(rx_msg.Data[0] == 'V' && rx_msg.Data[1] == 'X') {
                        int32_t index = 0;
                        float rpm = mc_interface_get_rpm_now();
                        buffer_append_float32(buffer, rpm, &index);
                        canAnswer(buffer, rx_msg);
                } else if(rx_msg.Data[0] == 'P' && rx_msg.Data[2] == 'F') { // 前馈模式
                        int32_t index = 4;
                        float angle = (float)buffer_get_int16(rx_msg.Data, &index) / 10.0f;
                        float cur_feedward = (float)buffer_get_int16(rx_msg.Data, &index) / 100.0f;
                        mc_interface_set_current_feedward(cur_feedward);
                        mc_interface_set_pid_pos(angle);
                }
            }
        }
        rx_frames_rear = (rx_frames_rear + 1) % RX_FRAMES_SIZE;
    }
}

/**
 * CAN反馈函数
 * 将查询的数据返回
 */
void canAnswer(uint8_t *data, const CanRxMsg RxMessage) {
    static CanTxMsg tx_msg;
    tx_msg.IDE = RxMessage.IDE;
    tx_msg.StdId = APPCONF_CONTROLLER_ANSWER_NODE + APPCONF_CONTROLLER_ID;
    tx_msg.DLC = 8;
    tx_msg.RTR = RxMessage.RTR;
    if(RxMessage.DLC == 4) {
        memcpy(tx_msg.Data,RxMessage.Data,4);
        memcpy(&tx_msg.Data[4], data, 4);
    } else if(RxMessage.DLC == 8) {
        //TODO: 此处应改为返回设定后的数据
        memcpy(tx_msg.Data,RxMessage.Data,8);
    }
}

void canSendstatus(void) {
    // Send status message
    uint8_t buffer[8];
    int32_t send_index = 0;
    buffer_append_int16(buffer, (int16_t)APPCONF_CONTROLLER_ID, &send_index);
    buffer_append_int16(buffer, (int16_t)mc_interface_get_rpm_now(), &send_index);
	buffer_append_int16(buffer, (int16_t)(mc_interface_get_tot_current() * 10.0f), &send_index);		
	buffer_append_int16(buffer, (int16_t)(mc_interface_get_pid_pos_now() * 10.0f), &send_index);		
    comm_can_transmit(buffer, send_index);
}

void comm_can_transmit(uint8_t *data, uint8_t len) {
	if (len > 8) {
		len = 8;
	}

#if CAN_ENABLE
	CanTxMsg txmsg;
	txmsg.IDE = CAN_Id_Standard;
	txmsg.StdId = APPCONF_CONTROLLER_ANSWER_NODE + APPCONF_CONTROLLER_ID;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.Data, data, len);
    CAN_Transmit(CAN1, &txmsg);
#endif
}

void CAN1_RX0_IRQHandler(void) {
    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
        canReceive(CAN1, CAN_FIFO0, &rx_frames[rx_frames_front]);
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

// CAN process
void TIM7_IRQHandler(void) {
    if(TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
        canProcess(rx_frames[rx_frames_rear]);
#if APPCONF_SEND_CAN_STATUS
        canSendstatus();
#endif
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    }
}
