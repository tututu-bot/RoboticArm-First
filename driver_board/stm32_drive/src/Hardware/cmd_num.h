// CAN发送的指令编号
#pragma once
//执行动作
#define CAN_MOVETO          0x101
//执行完毕回应
#define CAN_MOVETO_DONE     0x187
//请求角度
#define CAN_READ_ANGLES     0x103
//回应角度
#define CAN_RESPONSE_ANGLES 0x183
//收到报文回应
#define CAN_ACK             0x181
//心跳监测
#define CAN_PING            0x105
//心跳监测回应
#define CAN_PONG            0x185