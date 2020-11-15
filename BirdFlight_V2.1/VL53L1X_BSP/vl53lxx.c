#include "vl53lxx_i2c.h"
#include "vl53lxx.h"
#include "vl53l1_api.h"

#include "task.h"

/********************************************************************************	 
 * ������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
 * ALIENTEK MiniFly	
 * vl53lxxӦ�ô���, ����vl53l0x��vl53l1x
 * ����ԭ��@ALIENTEK
 * ������̳:www.openedv.com
 * ��������:2018/5/2
 * �汾��V1.0
 * ��Ȩ���У�����ؾ���
 * Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
 * All rights reserved
********************************************************************************/

//extern OS_TCB IICtoLaser_TaskTCB;

u16 vl53lxxId = 0;	/*vl53оƬID*/
bool isEnableVl53lxx = true;		/*�Ƿ�ʹ�ܼ���*/
bool isInitvl53l1x = false;	/*��ʼ��vl53l1x*/
bool reInitvl53l1x = false;	/*�ٴγ�ʼ��vl53l1x*/

//static u8 count = 0;
static u8 validCnt = 0;
static u8 inValidCnt = 0;

static u16 range_last = 0;
float quality = 1.0f;

zRange_t vl53lxx;

void vl53l1xTask(void* arg);
	
void IICtoLaser_Init(void)
{
//	OS_ERR err;
	vl53IICInit();	
	//delay_ms(10);
	VL53L1_RdWord(&dev, 0x010F, &vl53lxxId);	// vl53l1xx��ʼ��
	if(vl53lxxId == VL53L1X_ID)	{
		if (isInitvl53l1x)
		{
			reInitvl53l1x = true;
//			OS_TaskResume(&IICtoLaser_TaskTCB, &err); // �ָ�����������
		}
		else	/*�״ν���vl53l1x����ģ��*/
		{		
			isInitvl53l1x = true;			
			//xTaskCreate(vl53l1xTask, "VL53L1X", 300, NULL, 5, &vl53l1xTaskHandle);	/*����������ģ������*/
		}
		return;
	} else {		
		while(1) {	// ��������
			BEEP_ON;
			delay_ms(5);
			BEEP_OFF;
			delay_ms(5);
		}
	}
}

void vl53l1xMeasure(void)
{
	int status;
	u8 isDataReady = 0;
	static VL53L1_RangingMeasurementData_t rangingData;
	
	status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);
				
	if(isDataReady)
	{
		status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
		if(status==0)
		{
			range_last = rangingData.RangeMilliMeter * 0.1f;	//��λcm	
			RT_Info.LASER_Alt = range_last;				
		}
		status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
	}	
	
	if(range_last < VL53L1X_MAX_RANGE)			
		validCnt++;			
	else 			
		inValidCnt++;			
	
	if(inValidCnt + validCnt == 10)
	{
		quality += (validCnt/10.f - quality) * 0.1f;	//��ͨ
		validCnt = 0;
		inValidCnt = 0;
	}	
}

bool vl53lxxReadRange(zRange_t* zrange)
{
	if(vl53lxxId == VL53L1X_ID) 
	{
		zrange->quality = quality;		//���Ŷ�
		vl53lxx.quality = quality;
		
		if (range_last != 0 && range_last < VL53L1X_MAX_RANGE) 
		{			
			zrange->distance = (float)range_last;	//��λ[cm]	
			vl53lxx.distance = 	zrange->distance;
			return true;
		}
	}
	
	return false;
}

/*ʹ�ܼ���*/
void setVl53lxxState(u8 enable)
{
	isEnableVl53lxx = enable;
}

/*
void vl53l1xTask(void* arg)
{
	OS_ERR	err;
	int status;
	u8 isDataReady = 0;
	uint32_t xLastWakeTime = OS_TS_GET();
	static VL53L1_RangingMeasurementData_t rangingData;

	IICtoLaser_Init();
	if(vl53lxxId == 0) {
		RT_Info.batteryVoltage = 10.0;
		OSTimeDlyHMSM(0,0,4,0,OS_OPT_TIME_HMSM_STRICT,&err);
	}
	vl53l1xSetParam();	//����vl53l1x ����
	
	while(1) 
	{
		if(reInitvl53l1x == true)
		{
			count = 0;
			reInitvl53l1x = false;			
			vl53l1xSetParam();	//����vl53l1x ����
			xLastWakeTime = OS_TS_GET();
		}else
		{	
			RT_Info.batteryVoltage = 5.0;	
			status = VL53L1_GetMeasurementDataReady(&dev, &isDataReady);
						
			if(isDataReady)
			{
				
				status = VL53L1_GetRangingMeasurementData(&dev, &rangingData);
				if(status==0)
				{
					range_last = rangingData.RangeMilliMeter * 0.1f;	//��λcm
					RT_Info.batteryVoltage = 10.0;
					RT_Info.LASER_Alt = range_last;				
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(&dev);
			}	
			
			if(range_last < VL53L1X_MAX_RANGE)			
				validCnt++;			
			else 			
				inValidCnt++;			
			
			if(inValidCnt + validCnt == 10)
			{
				quality += (validCnt/10.f - quality) * 0.1f;	//��ͨ
				validCnt = 0;
				inValidCnt = 0;
			}
						
//			if(getModuleID() != OPTICAL_FLOW)
//			{
//				if(++count > 10)
//				{
//					count = 0;
//					VL53L1_StopMeasurement(&dev);
//					vTaskSuspend(vl53l1xTaskHandle);	//���𼤹�������			
//				}				
//			}else count = 0;
//			
		}		
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
*/


