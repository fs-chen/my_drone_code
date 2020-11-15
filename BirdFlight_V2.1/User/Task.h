#ifndef __TASK_H
#define __TASK_H
#include "os_app_hooks.h"
#include "explore_system.h"
#include "explore_systick.h"
#include "stm32f4xx.h"
#include "includes.h"
#include "cpu.h"
#include "os.h"
#include "IMU_AHRS.h"
#include "DronePara.h"
#include "Data_PC.h"
#include "Inner_control.h"
#include "Outer_control.h"
#include "Usart1toPC.h"
#include "Usart6toVision.h"
#include "Data_deal.h"
#include "Visiondata_deal.h"
#include "Adc_Battery.h"
#include "Usart3toBluetooth.h"
#include "General_Gpio.h"
#include "Position.h"
#include "Neurons.h"
#include "Usart5toUltra.h"
#include <stdbool.h>
#include "UltraCombine.h"
#include "vl53lxx.h"
#include "optical_flow.h"
#include "LineInspection.h"
//#include "my_platform.h"
//�ź���
extern OS_SEM IMU_proc;
extern OS_SEM DataDeal_proc;
extern OS_SEM Vision_proc;
extern OS_SEM Vision_proc_Horizon;
extern OS_SEM AttitudeInner_proc;
extern OS_SEM Platform_Semp;
//ȫ���ⲿ����
extern DroneFlightControl FlightControl;     
extern DronePIDPara Para_Info;								 
extern DroneRTInfo RT_Info;
extern DroneErrangle Errangle_Info;
extern DroneTargetInfo Target_Info;
extern Controller Control_Info;
extern Throttle Throttle_Info;

extern u8 StartFly;
extern bool USE_BLUETOOTH; 
extern volatile float US100_Altinfo;

#endif 

