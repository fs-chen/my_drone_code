#ifndef __USART5TOULTRA_H
#define __USART5TOULTRA_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include <string.h>
#include <os.h>
#include "Task.h"
void Uart5toOpenmv_Init(u32 Bound);
void Uart5toUltra_Init(u32 Bound);
void ReceiveUltraData(void);
void SendStopflag(void);
extern _Data_Rx Vision_Horizon; 
extern volatile uint16_t ReceiveHeight;
#define ULTRA_SPEED 0.34f  //unit mm/us
extern uint8_t RequireFlag;
void SendRequire(uint8_t type);

#endif
