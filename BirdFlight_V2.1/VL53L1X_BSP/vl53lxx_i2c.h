#ifndef __VL53LXX_I2C_H
#define __VL53LXX_I2C_H
#include "explore_system.h" 
#include "stdbool.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly
 * VL53 IIC��������	
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.3
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
 *
 * �޸�˵��:
 * �汾V1.3 ���Ӷ�vl53l1x��IIC������
********************************************************************************/

/*IO��������*/
#define SDA_IN()  {GPIOA->MODER&=~(3<<(11*2));GPIOA->MODER|=0<<11*2;}	//PA11����ģʽ
#define SDA_OUT() {GPIOA->MODER&=~(3<<(11*2));GPIOA->MODER|=1<<11*2;}   //PA11���ģʽ
/*IO��������*/	 
#define VL53_SCL    PAout(12) 	//SCL
#define VL53_SDA    PAout(11) 	//SDA	 
#define READ_SDA	PAin(11)  	//����SDA 

//VL53���в�������
void vl53IICInit(void);			/*��ʼ��VL53��IO��*/				 
u8 vl53IICReadByte(u8 devaddr,u8 addr, u8* data);		/*��һ�ֽ�*/
void vl53IICWriteByte(u8 devaddr,u8 addr,u8 data);		/*дһ�ֽ�*/
void vl53IICRead(u8 devaddr,u8 addr,u8 len,u8 *rbuf);	/*������ȡ����ֽ�*/
void vl53IICWrite(u8 devaddr,u8 addr,u8 len,u8 *wbuf);	/*����д�����ֽ�*/
bool vl53IICWriteBit(u8 devaddr,u8 addr, u8 bitNum, u8 data);	/*iic д��ĳ��λ*/

void vl53l1Read(u8 devaddr,u16 addr,u8 len,u8 *rbuf);	/*������ȡ����ֽ�*/
void vl53l1Write(u8 devaddr,u16 addr,u8 len,u8 *wbuf);	/*����д�����ֽ�*/
	
#endif 


