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
#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);


///////////////////////////////////////////////////////////////////////////////////////////////////////
#endif 
