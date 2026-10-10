#include "driver/twai.h"            // ★ ESP-IDF 的 CAN(TWAI) 驱动
#include "freertos/FreeRTOS.h"      // pdMS_TO_TICKS
#include "freertos/task.h"          // vTaskDelay（心跳任务用）
#include "esp_log.h"                // 可选：日志输出
#include "cmd_num.h"                  // 自研：协议合同（ID 定义）
#include "mcu.h"                    // 自研：本模块对外接口

#define TWAI_TX_PIN 21
#define TWAI_RX_PIN 22
volatile int mcu_link_ok = 0;

//初始化can通信
void mcu_init(void){    
    //gpio配置21->tx，22->rx
    twai_general_config_t gpio = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    //报文队列，最多可存10条报文
    gpio.rx_queue_len = 10;
    //波特率500kbit/s
    twai_timing_config_t baud_rate = TWAI_TIMING_CONFIG_500KBITS(); // 两侧同速率
    //过滤器，筛选接收的报文
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    //写入配置，启动
    esp_err_t err = twai_driver_install(&gpio, &baud_rate, &filter);
    if (err != ESP_OK)
        printf("mcu_init: twai_driver_install 失败 0x%x\n", err);
    err = twai_start();
    if (err != ESP_OK)
        printf("mcu_init: twai_start 失败 0x%x\n", err);
}
//发送信息（mcu_transact调用）
//参数：消息ID、消息数据、数据长度
static void send_msg(uint16_t cmd_num, const uint8_t *data, uint8_t len){
    twai_message_t msg = {.identifier = cmd_num, .data_length_code = len};
    for (int i = 0; i < len; i++) msg.data[i] = data[i];

    esp_err_t e = twai_transmit(&msg, pdMS_TO_TICKS(50));
    if (e != ESP_OK){
        twai_status_info_t st;
        twai_get_status_info(&st);
        printf("发送失败: 0x%x | state=%d tx_err=%lu rx_err=%lu\n",
               e, st.state, (unsigned long)st.tx_error_counter, (unsigned long)st.rx_error_counter);
        if (st.state == TWAI_STATE_BUS_OFF){
            twai_initiate_recovery();
            for (int i = 0; i < 20 && st.state != TWAI_STATE_STOPPED; i++){
                vTaskDelay(pdMS_TO_TICKS(50));
                twai_get_status_info(&st);
            }
            twai_start();
        }
    }
}
//can通信事务 = 发送 + 应答(内部调用)
//参数：指令编号、报文数据、发送长度、应答指令编号、应答数据、应答长度
static int mcu_transact(uint16_t cmd_num, const uint8_t *data, uint8_t len,uint16_t r_cmd_num, uint8_t *r_data, uint8_t *r_len){
    int ok = 0;
    for (int attempt = 0; attempt < 3 && !ok; attempt++){//发送3次消息
        //1.发送消息
        send_msg(cmd_num, data, len);
        for (int wait = 0; wait < 5; wait++){
            twai_message_t m;
            //2.处理应答信息
            if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK){
                //收到应答，无应答ID
                if (!r_cmd_num && m.identifier == CAN_ACK && m.data[0] == (uint8_t)cmd_num){
                    ok = 1;
                    break;
                }
                //收到应答，有应答ID
                if (r_cmd_num && m.identifier == r_cmd_num){
                    if (r_len)
                        *r_len = m.data_length_code;
                    if (r_data)
                        for (int i = 0; i < m.data_length_code && i < 8; i++)
                            r_data[i] = m.data[i];
                    ok = 1;
                    break;
                }
            }
        }
    }
    //更新链路状态：应答到了就是通，没到就是断
    mcu_link_ok = ok;
    return ok;
}
//发送转动舵机指令给驱动层
int mcu_moveto(const int angle[6], int spend_time){
    uint8_t msg[8];
    for (int i = 0; i < 6; i++) msg[i] = (uint8_t)angle[i];
    msg[6] = spend_time & 0xFF;
    msg[7] = (spend_time >> 8) & 0xFF;

    send_msg(CAN_MOVETO, msg, 8);

    int tries = (spend_time + 1000) / 10;
    for (int i = 0; i < tries; i++){
        twai_message_t m;
        if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK && m.identifier == CAN_MOVETO_DONE){
            mcu_link_ok = 1;
            return 1;
        }
    }
    mcu_link_ok = 0;
    return 0;
}
//发一次心跳给驱动板，收不到 CAN_PONG 就是链路没通
int mcu_ping(void){
    uint8_t msg[8], len = 0;
    return mcu_transact(CAN_PING, NULL, 0, CAN_PONG, msg, &len);
}
//获取舵机角度
int mcu_get_angles(int actual_angle[6]){
    uint8_t msg[8], len = 0;
    if (mcu_transact(CAN_READ_ANGLES, NULL, 0, CAN_RESPONSE_ANGLES, msg, &len) && len >= 6){
        for (int i = 0; i < 6; i++)
            actual_angle[i] = msg[i];
        return 1;
    }
    return 0;
}
//指定某个舵机转动
int mcu_servo_write(int idx, int angle){
    uint8_t msg[8];
    uint8_t resp[2],len = 0;
    msg[0] = (uint8_t)idx;
    msg[1] = (uint8_t)angle;
    return mcu_transact(CAN_POSE_MOVE, msg, 2, CAN_POSE_MOVE_DONE, resp, &len);
}