#ifndef __DRONWPARA_H
#define __DRONWPARA_H

#include "stm32f4xx.h"

#define TX_LEN  512
#define RX_LEN  512

typedef enum
{ 
	Drone_Mode_None=0,
  Drone_Mode_Pitch= 1,//�⻷ʵ��	
  Drone_Mode_Roll= 2, 
	Drone_Mode_4Axis= 3,	
	Drone_Mode_RatePitch= 4, //�ڻ�ʵ��
  Drone_Mode_RateRoll= 5, 	
}DroneFlightMode_TypeDef;

typedef enum
{  
	Drone_Off  = 0x00,//��ɻ��ߵ��Դ򿪵��
  Drone_On   = 0x01,//�رյ��
  Drone_Land = 0x02,//����	
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
	float feedforwardOut;//ǰ���������
	float pOut;
  float iOut;
	float dOut;
	float value;//PID�������
	
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

/* ƽ��������� */
typedef struct
{
	float fixedErroPitch;
	float fixedErroRoll;
}DroneErrangle;

/*ң����Ϣ*/
typedef struct
{
	float pV;//pitch�ٶ�
  float rV;//roll�ٶ�
	float hV;//�����ٶ�
	float yV;//�����ٶ�
}Controller;


/*���˻�ʵʱ��Ϣ*/
typedef struct
{
	float Pitch; 
	float Roll;
  float Yaw;
	float batteryVoltage;
  float	ratePitch;			//Pitch��Ľ��ٶ�
	float rateRoll;				//Roll��Ľ��ٶ�
	float rateYaw;				//Yaw��Ľ��ٶ�
	float US100_Alt;			//US100�������߶�
	float LASER_Alt;		//������߶�
	float US100_Alt_V;		//US100�������ٶ�
	float FlowX;					//����λ��X
	float FlowY;          //����λ��Y
	float FlowX_V;				//�����ٶ�X
	float FlowY_V;				//�����ٶ�Y
	float PointX;					//������Xλ��
	float PointY;					//������Yλ��
	float PointX_V;				//������X���ٶ�
	float PointY_V;				//������Y���ٶ�
	int lowPowerFlag;			//�͵�ѹ��־λ
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

