#pragma once
#define TWAI_TX_PIN 21
#define TWAI_RX_PIN 22
extern volatile int mcu_link_ok;

void mcu_init(void);
int mcu_moveto(const int angle[6], int spend_time);
int mcu_get_angles(int out[6]);