#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "main.h"

extern char tx_buffer[20];
extern float speed_LF;
extern float speed_RF;
extern float speed_LB;
extern float speed_RB;

void Task_1ms(void);

#endif