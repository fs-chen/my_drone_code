#ifndef __LINEINSPECTION_H
#define __LINEINSPECTION_H


#include "Task.h"


void LineInspectInit(void);


typedef enum
{
	Takeoff=0,									//起飞完成标准：四个期望起伏在一个范围内并且保持一段时间。
	
	
	AdjustPosA=1,    //校准标准：像素面的参数（距离）以及中心位置（位置）的反馈稳定在一个范围内并且保持一段时间														//										跟地面摄像头角度标定成90左右并且保持一段时间
														//找到线：线的坐标在摄像头中心处范围起伏（高度）并且稳定
														//线的面参数（距离）反馈稳定，并且稳定。
	Inspect1=2,									//巡线：等待openmv给出识别到条码的信号进行声光显示。

	
	AdjustPosB=3,								//调整和B的位置关系，调整完等待openmv发送信号。等待转弯信号
	
	
	TurnCorner=4,		
	
	
	Inspect2=5,	
	
	
	Land=6
}StageType;


typedef struct
{
	struct
	{		
		float Target_X_V;
		float Target_Y_V;
		float Target_H;
		float Target_Yaw_Acc;
	}Target;
	StageType Stage;

}LineInspectObject;
//寻线过程对象


//巡线过程
void LineInspectInit(void);
void TakeOffPro(void);
void AdjustAPosPro(void);
void Inspect1Pro(void);
void AdjustBPosPro(void);
void TurnCornerPro(void);
void Inspect2Pro(void);
void LandPro(void);

#endif

