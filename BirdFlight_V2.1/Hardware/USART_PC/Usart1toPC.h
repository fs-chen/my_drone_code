#ifndef _USARTTOPC_H
#define _USARTTOPC_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include <string.h>
#include <os.h>
#include "Task.h"
#include "stdio.h"
void Uart1_tx(uint8_t *data,uint16_t size);
void Usart1toPC_Init(u32 Bound);

extern _Data_Rx PC_rx;
///////////////////////////////////////////////////////////////////////////////////////////////////////
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);


///////////////////////////////////////////////////////////////////////////////////////////////////////
#endif 
