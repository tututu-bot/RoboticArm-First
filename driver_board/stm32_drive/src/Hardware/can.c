/**
 * can通信流程分析：
 * 1.can硬件接收到报文
 * 2.触发中断
 * 3.执行中断入口函数USB_LP_CAN1_RX0_IRQHandler
 * 4.重写入口函数逻辑USB_LP_CAN1_RX0_IRQHandler->执行Can_ReceiveMsg
 * 5.Can_ReceiveMsg解析报文信息(cmd_num、data、len)并打包成结构体,将结构体存入任务队列中
 * 6.队列一旦就数据就会唤醒ctrl_task：取出帧->调handleCommand函数->解析编号->执行对应动作
 * 常见问题：
 * 一、任务执行问题：长任务和短任务全部放在任务队列中，执行任务遵循先进先出原则。
 * 解决方案：创建2个队列一个处理高优先级任务（短任务），一个处理低优先级（长任务）
 * 二、失联保护问题：主控板和驱动板联系不上了，驱动板收不到主控板发来的报文了。
 * 考虑情况：① 主控板死机/重启/拔线 -> 不再发任何报文
 *         ② CAN线断、接触不良、收发器掉电、bus-off -> 报文到不了
 * 解决方案：超时即停。
 * 心跳监测，主控板每秒发送PING指令到驱动板，如果驱动板3秒都没接收到PING信号则启动失联保护策略
 * 保护策略：1)停止当前动作 2)任务队列删除所有动作帧 
 *         3)缓慢恢复homepose，串口返回失联信息 4)一旦收到报文，自动恢复正常可操作状态
 */
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "can.h"
#include "cmd_num.h"
#include "servo.h"

CAN_HandleTypeDef can;
QueueHandle_t can_rx_queue   = NULL;
QueueHandle_t can_resp_queue = NULL;
volatile TickType_t last_rx_tick = 0;

//CAN的GPIO配置
static void can_gpio_init(void){
    //启动GPIOA和CAN1时钟树
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    //绑定CAN1的RX和TX引脚为PA11和PA12
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_11;  g.Mode = GPIO_MODE_INPUT;  g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_12;  g.Mode = GPIO_MODE_AF_PP;  g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
}
//CAN的基本配置
static void can_config_init(void){
    //配置对象为CAN1
    can.Instance = CAN1;
    //波特率设置
    can.Init.Prescaler = 4;
    can.Init.SyncJumpWidth = CAN_SJW_1TQ;
    can.Init.TimeSeg1 = CAN_BS1_15TQ;
    can.Init.TimeSeg2 = CAN_BS2_2TQ;
    //选择模式：正常收发模式
    can.Init.Mode = CAN_MODE_NORMAL;
    //自动机制配置
    can.Init.TimeTriggeredMode = DISABLE;
    can.Init.AutoBusOff = ENABLE;
    can.Init.AutoWakeUp = DISABLE;
    can.Init.AutoRetransmission = ENABLE;
    can.Init.ReceiveFifoLocked = DISABLE;
    can.Init.TransmitFifoPriority = DISABLE;
    //配置写入寄存器
    HAL_CAN_Init(&can);
}
//CAN的滤波器配置
static void can_filter_init(void){
    CAN_FilterTypeDef f = {0};
    //0号滤波器
    f.FilterBank = 0;
    //模式配置
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0; f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0; f.FilterMaskIdLow = 0;
    //接收的报文放FIFO0
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&can, &f);
}
void can_init() {
    printf("stm-can-init");
    //CAN启动前（GPIO、配置、滤波器）
    can_gpio_init();
    can_config_init();
    can_filter_init();
    //启动CAN
    HAL_CAN_Start(&can);
    HAL_CAN_ActivateNotification(&can, CAN_IT_RX_FIFO0_MSG_PENDING);

    // ★ 队列必须放在最后建！xQueueCreate 会进 FreeRTOS 临界区，而调度器启动前
    //   uxCriticalNesting 还是哨兵值 0xaaaaaaaa（见 port.c:133），vPortExitCritical()
    //   判 != 0 就不恢复 BASEPRI（port.c:382）→ 中断被永久屏蔽 → SysTick 停 →
    //   HAL_GetTick() 冻结 → 上面 HAL_CAN_Start/Init 的 10ms 超时永不成立 → 死循环。
    //   规则：vTaskStartScheduler() 之前，先做硬件，FreeRTOS 对象放最后。
    can_rx_queue   = xQueueCreate(2, sizeof(can_frame_t));
    can_resp_queue = xQueueCreate(4, sizeof(can_frame_t));
}
void can_enable_rx(void) {
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}
//接收报文
void Can_ReceiveMsg(CAN_HandleTypeDef *can) {
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &rx, data) != HAL_OK) return;
    last_rx_tick = xTaskGetTickCountFromISR();//更新最后联系时间
    //解析报文信息(cmd_num、data、len)并打包成结构体
    can_frame_t f;
    f.cmd_num = rx.StdId;
    f.len     = rx.DLC;
    for (int i = 0; i < rx.DLC && i < 8; i++) f.data[i] = data[i];
    QueueHandle_t q = (rx.StdId == CAN_PING || rx.StdId == CAN_READ_ANGLES) ? can_resp_queue : can_rx_queue;
    BaseType_t hpw = pdFALSE;
    //结构体入队
    xQueueSendFromISR(q, &f, &hpw);
    portYIELD_FROM_ISR(hpw);
}
//发送报文
bool Can_SendMsg(uint16_t cmd_num, const uint8_t *data, uint8_t len){
    printf("stm32-应答主控板");
    CAN_TxHeaderTypeDef tx = {0};
    tx.StdId = cmd_num; 
    tx.IDE = CAN_ID_STD; 
    tx.RTR = CAN_RTR_DATA; 
    tx.DLC = len;
    uint32_t mailbox;
    return HAL_CAN_AddTxMessage(&can, &tx, (uint8_t *)data, &mailbox) == HAL_OK;
}
//根据指令编号找到对应方法执行
void handleCommand(uint16_t cmd_num, const uint8_t *data, uint8_t len){
    printf("处理指令");
    uint8_t ack[2] = { (uint8_t)cmd_num, 0 };
    //先应答，再执行任务。
    Can_SendMsg(CAN_ACK, ack,2);
    //执行动作
    if (cmd_num == CAN_MOVETO && len == 8){
        //转动花费时间
        uint16_t spend_time = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
        int target[6];
        for (int i = 0; i < 6; i++) target[i] = data[i];
        servo_moveTo(target, spend_time);
        uint8_t done[6];
        for (int i = 0; i < 6; i++) done[i] = (uint8_t)current_angles[i];
        Can_SendMsg(CAN_MOVETO_DONE, done, 6);
    }
    //请求角度
    else if (cmd_num == CAN_READ_ANGLES){
        uint8_t actual_angles[6];
        for (int i = 0; i < 6; i++) actual_angles[i] = (uint8_t)read_actual_angle(i);
        //回应角度
        Can_SendMsg(CAN_RESPONSE_ANGLES, actual_angles, 6);
    }
    //心跳监测
    else if (cmd_num == CAN_PING){
        //回应心跳
        Can_SendMsg(CAN_PONG, 0, 0);
    }
}
void USB_LP_CAN1_RX0_IRQHandler(void) { 
    Can_ReceiveMsg(&can); 
}

