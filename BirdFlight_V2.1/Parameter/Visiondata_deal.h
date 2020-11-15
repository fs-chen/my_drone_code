#ifndef __VISIONDATA_DEAL_H
#define __VISIONDATA_DEAL_H

#include "stm32f4xx.h"
#include "DronePara.h"
#include "Task.h"
#include "MedianFiler.h"
#include "arm_math.h"
#include "Usart6toVision.h"
#include "LineInspection.h"


typedef enum
{
	BarCode=0,
	QRCode=1,
	Tower=2,
	Line=3
}VisionRequireType;

//////////////
extern u8 BlackspotsFlag;
extern u8 OpticalflowFlag;
//点位数据
extern volatile float Pix_Xinfo;
extern volatile float Pix_Yinfo;
//光流数据
extern volatile float OpticalFlow_x;
extern volatile float OpticalFlow_y;
extern volatile float OpticalFlow_integralx;
extern volatile float OpticalFlow_integraly;;
void VisionDataDealButton(_Data_Rx rx);
void VisionDataDealHorizon(_Data_Rx rx);
void SendHeightToVision(void);
float CushionSet(float Object,uint8_t step,float target);
#endif


