#ifndef __DRONWPARA_H
#define __DRONWPARA_H

#include "stm32f4xx.h"

#define TX_LEN  512
#define RX_LEN  512

typedef enum
{ 
	Drone_Mode_None=0,
  Drone_Mode_Pitch= 1,//外环实验	
  Drone_Mode_Roll= 2, 
	Drone_Mode_4Axis= 3,	
	Drone_Mode_RatePitch= 4, //内环实验
  Drone_Mode_RateRoll= 5, 	
}DroneFlightMode_TypeDef;

typedef enum
{  
	Drone_Off  = 0x00,//起飞或者调试打开电机
  Drone_On   = 0x01,//关闭电机
  Drone_Land = 0x02,//降落	
}DroneFlightOnOff_TypeDef;

typedef enum
{  
	Report_SET      = 0x01,
  Report_RESET    = 0x00, 		 	
}DroneReportSW_TypeDef;


typedef struct
{
	DroneFlightOnOff_TypeDef OnOff;
	DroneFlightMode_TypeDef droneMode;
	DroneReportSW_TypeDef ReportSW;
	int landFlag;
}DroneFlightControl;


typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
	float RateRoll;
	float RatePitch;
	float RateYaw;
	float Height;
	float AccHeight;
}DroneTargetInfo;


typedef struct
{
	float feedforwardOut;//前馈控制输出
	float pOut;
  float iOut;
	float dOut;
	float value;//PID控制输出
	
}PIDOut;

typedef struct
{
	float Kp;
	float Ki;
	float	Kd;
}PID;

typedef struct
{
	PID Pitch;
	PID Roll;
	PID Yaw;
 	PID Height;
	
	PID ratePitch;
	PID rateRoll;
	PID rateYaw;
	PID accHeight;
	
}DronePIDPara;

/* 平地误差数据 */
typedef struct
{
	float fixedErroPitch;
	float fixedErroRoll;
}DroneErrangle;

/*遥控信息*/
typedef struct
{
	float pV;//pitch速度
  float rV;//roll速度
	float hV;//上升速度
	float yV;//航向速度
}Controller;


/*无人机实时信息*/
typedef struct
{
	float Pitch; 
	float Roll;
  float Yaw;
	float batteryVoltage;
  float	ratePitch;			//Pitch轴的角速度
	float rateRoll;				//Roll轴的角速度
	float rateYaw;				//Yaw轴的角速度
	float US100_Alt;			//US100超声波高度
	float LASER_Alt;		//激光测距高度
	float US100_Alt_V;		//US100超声波速度
	float FlowX;					//光流位置X
	float FlowY;          //光流位置Y
	float FlowX_V;				//光流速度X
	float FlowY_V;				//光流速度Y
	float PointX;					//点数据X位移
	float PointY;					//点数据Y位移
	float PointX_V;				//点数据X的速度
	float PointY_V;				//点数据Y的速度
	int lowPowerFlag;			//低电压标志位
}DroneRTInfo;

typedef struct
{
	unsigned int M1;
	unsigned int M2;
	unsigned int M3;
	unsigned int M4;
}Throttle;

typedef struct
{
	unsigned int len;
	unsigned char buf[20];
}_Data_Rx;

#endif

