#include "LineInspection.h"


LineInspectObject LineInspect;
//巡线结构体
uint8_t FindTowerBFlag=0;
//发现塔B的标志Bit
uint8_t FindBarCodeFlag=0;
//发现条码的标志Bit
uint8_t FindQRCodeFlag=0;
//发现二维码的标志Bit
uint8_t TurnCornerFinishFlag=0;
//转弯完成的标志Bit
uint8_t FindTowerAFlag=0;
//返程时发现A的标志Bit
void LineInspectInit(void)
{
	LineInspect.Stage = AdjustPosA;
	LineInspect.Target.Target_X_V = 0.f;
	LineInspect.Target.Target_Y_V = 0.f;
	LineInspect.Target.Target_H = 1.f;
	LineInspect.Target.Target_Yaw_Acc = 0.f;
}
extern uint8_t NoRegTowerFlagH;
//水平摄像头未识别塔
extern uint8_t NoRegLineFlagH;
//水平摄像头未识别到线标志位 1为未识别到 0则相反
extern int16_t TowerPosX;
//塔的X坐标
extern uint16_t TowerDeltaX;
//塔的X宽度
extern int16_t LinePosY;
//线的Y坐标
extern uint16_t LineDeltaY;
//线的Y宽度
int16_t TowerTarX;
//塔的期望X坐标
uint16_t TowerTarDeltaX;
//塔的期望宽度
int16_t LineTarY;
//线的期望Y坐标
uint16_t LineTarDeltaY;
//线的期望高度

uint8_t LineDeltaYDeathZoon=4;
//对线的高度的死区宽度
uint8_t TowerXDeathZoon=4;
//对塔的位置死区宽度
uint8_t LinePosYDeathZoon=4;
//对线的位置高度Y的死区
uint8_t TowerDeltaXDeathZoon=4;
//对塔的宽度的死区宽度
float Inspect1Speed=0.01f;
//巡线1速度
float AdjustXSpeed=0.005f;
//光流X方向调整速度
float AdjustYSpeed=0.005f;
//光流Y方向调整速度
uint8_t AdjustStep=10;
//调整步长
float AdjustHeightWeight=0.005f;
//高度调整步长

extern DroneRTInfo RT_Info;     
//飞行器实时数据
extern DroneTargetInfo Target_Info;     
//飞行器目标的全局变量
extern DroneFlightControl FlightControl;   
//飞行器状态变量


void TakeOffPro(void)
{
//	if()//起飞信号
	{
		float HeightBias=RT_Info.US100_Alt - Target_Info.Height;
		static uint8_t TakeOffCnt=0;
		if(ABS(HeightBias)<0.05f)
		{
			TakeOffCnt++;
			if(TakeOffCnt>=200)
			{
				TakeOffCnt=0;
				LineInspect.Stage = AdjustPosA;  //起飞完成后,进行位置调整A
			}
		}
		else
		{
			TakeOffCnt=0;		
		}
	}
}


//调接至塔A
void AdjustAPosPro(void)
{	
	TowerTarX = -50;
//	TowerTarDeltaX = 20;
	//对塔的期望
	//对塔的控制
//	int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
//	int16_t TowerErroPosX = TowerTarX - TowerPosX;
	int16_t LineErroPosY = LineTarY - LinePosY;
//	int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
	//计算塔误差
	//此处的X是摄像头X
//	static uint8_t AdjustCnt=0;
	if(NoRegTowerFlagH!=1)
	{
		//接收正常
//		if((TowerErroPosX>-TowerXDeathZoon) && (TowerErroPosX<TowerXDeathZoon))
//		{
//			TowerErroPosX=0;			
//		}//死区
//		if(TowerErroPosX>0)//作用在光流Y轴上。
//		{
//			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_Y_V>=AdjustYSpeed)
//			{
//				LineInspect.Target.Target_Y_V = AdjustYSpeed;
//			}
//		}
//		else if(TowerErroPosX<0)
//		{
//			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_Y_V<=-AdjustYSpeed)
//			{
//				LineInspect.Target.Target_Y_V = -AdjustYSpeed;
//			}			
//		}
//		else
//		{
//			if(LineInspect.Target.Target_Y_V > 0)
//			{
//				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
//				if(LineInspect.Target.Target_Y_V<=0)
//				{
//					LineInspect.Target.Target_Y_V = 0;
//				}					
//			}
//			else if(LineInspect.Target.Target_Y_V < 0)
//			{
//				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
//				if(LineInspect.Target.Target_Y_V>=0)
//				{
//					LineInspect.Target.Target_Y_V = 0;
//				}
//			}
//			else
//			{
//				LineInspect.Target.Target_Y_V = 0.0f;
//			}

//		}
		

//		TowerTarDeltaX = 10;
//		//对线的期望

//		
//		//计算线误差
//		//此处的Y是摄像头Y
//		if((TowerErroDeltaX>-TowerDeltaXDeathZoon) && (TowerErroDeltaX<TowerDeltaXDeathZoon))
//		{
//			TowerErroDeltaX=0;			
//		}//死区

//		
//		if(TowerErroDeltaX>0) //作用在光流X轴上。
//		{
//			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
//			{
//				LineInspect.Target.Target_X_V=-AdjustXSpeed;
//			}
//		}
//		else if(TowerErroDeltaX<0)
//		{
//			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
//			{
//				LineInspect.Target.Target_X_V=AdjustXSpeed;
//			}
//		}
//		else
//		{
//			if(LineInspect.Target.Target_X_V>0)
//			{
//				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
//				if(LineInspect.Target.Target_X_V<=0)
//				{
//					LineInspect.Target.Target_X_V=0;
//				}
//			}
//			else if(LineInspect.Target.Target_X_V<0)
//			{
//				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
//				if(LineInspect.Target.Target_X_V>=0)
//				{
//					LineInspect.Target.Target_X_V=0;
//				}
//			}
//			else
//			{
//				LineInspect.Target.Target_X_V = 0.0f;
//			}
//		}
//		if((ABS(TowerErroPosX)<(2*TowerXDeathZoon)) && (ABS(LineErroPosY)<(2*LinePosYDeathZoon)) && (ABS(TowerErroDeltaX)<(2*TowerDeltaXDeathZoon)))
//		{
//			AdjustCnt++;
//			if(AdjustCnt>=200)
//			{
//				AdjustCnt=0;		
//				LineInspect.Stage = Inspect1;
//			}//调节稳定后 开始巡线
//		}
//		else
//		{
//			AdjustCnt=0;
//		}				
		LineInspect.Target.Target_X_V=0;
	}
	else
	{
		//未接收到正确的数据包
//		if(LineInspect.Target.Target_Y_V > 0)
//		{
//			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_Y_V<=0)
//			{
//				LineInspect.Target.Target_Y_V = 0;
//			}					
//		}
//		else if(LineInspect.Target.Target_Y_V < 0)
//		{
//			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_Y_V>=0)
//			{
//				LineInspect.Target.Target_Y_V = 0;
//			}
//		}
//		else
//		{
//			LineInspect.Target.Target_Y_V = 0.0f;
//		}
		
		
//		if(LineInspect.Target.Target_X_V>0)
//		{
//			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V<=0)
//			{
//				LineInspect.Target.Target_X_V=0;
//			}
//		}
//		else if(LineInspect.Target.Target_X_V<0)
//		{
//			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V>=0)
//			{
//				LineInspect.Target.Target_X_V=0;
//			}
//		}
//		else
//		{
//			LineInspect.Target.Target_X_V = 0.0f;
//		}		
	}
	
	if(NoRegLineFlagH!=1)
	{
		//对线的控制
		//识别到正确的数据
		LineTarY = 0;
//		ALLLED_ON;
		if((LineErroPosY>-LinePosYDeathZoon) && (LineErroPosY<LinePosYDeathZoon))
		{
			LineErroPosY=0;			
		}//死区		
		if(LineErroPosY>0)//作用在高度上
		{
			//说明无人机偏下 需要提高高度
//			LineInspect.Target.Target_H+=AdjustHeightWeight;
//			Target_Info.Height+=AdjustHeightWeight;
		}
		else if(LineErroPosY<0)
		{
			//说明无人机偏上 需要降低高度
//			LineInspect.Target.Target_H-=AdjustHeightWeight;
//			Target_Info.Height-=AdjustHeightWeight;
		}
		else
		{
//			LineInspect.Target.Target_H+=0.f;
			Target_Info.Height+=0;
		}				
	}
	else
	{
		ALLLED_OFF;
//		//未识别到正确的数据
//		if(LineInspect.Target.Target_X_V>0)
//		{
//			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V<=0)
//			{
//				LineInspect.Target.Target_X_V=0;
//			}
//		}
//		else if(LineInspect.Target.Target_X_V<0)
//		{
//			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
//			if(LineInspect.Target.Target_X_V>=0)
//			{
//				LineInspect.Target.Target_X_V=0;
//			}
//		}
//		else
//		{
//			LineInspect.Target.Target_X_V = 0.0f;
//		}
//		
		LineInspect.Target.Target_H+=0.f;
	}

}


void Inspect1Pro(void)
{
	if(NoRegLineFlagH!=1)
	{	
		LineTarY=0;
		LineTarDeltaY = 10;
		//对线的期望
		int16_t LineErroPosY = LineTarY - LinePosY;
		int16_t LineErroDeltaY = LineTarDeltaY - LineDeltaY;
		//计算线宽度误差。
		
		if((LineErroDeltaY>-LineDeltaYDeathZoon) && (LineErroDeltaY<LineDeltaYDeathZoon))
		{
			LineErroDeltaY=0;
			
		}//死区	
		if((LineErroPosY>-LinePosYDeathZoon) && (LineErroPosY<LinePosYDeathZoon))
		{
			LineErroPosY=0;
			
		}//死区
		
		if(LineErroDeltaY>0)//作用在光流X轴上
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V=-AdjustXSpeed;
			}
		}
		else if(LineErroDeltaY<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V=AdjustXSpeed;
			}
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}			
			}
			else
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}						
		}
		
		if(LineErroPosY>0)//作用在高度上
		{
			//说明无人机偏下 需要提高高度
			LineInspect.Target.Target_H+=AdjustHeightWeight;
		}
		else if(LineErroPosY<0)
		{
			//说明无人机偏上 需要降低高度
			LineInspect.Target.Target_H-=AdjustHeightWeight;
		}
		else
		{
			LineInspect.Target.Target_H+=0.f;
		}	
		
		if(FindTowerBFlag || FindBarCodeFlag)
		{
			//发现了塔B 或者 条码
			if(FindBarCodeFlag)//几乎不可能同时出现
			{
				//发现了条码
				static uint16_t BeepCnt=0;
				BEEP_ON; //发现条码 蜂鸣器响起提示 
				LineInspect.Target.Target_Y_V -= (Inspect1Speed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=0)
				{				
					LineInspect.Target.Target_Y_V=0;						
					BeepCnt++;
					if(BeepCnt>=500)
					{
						BEEP_OFF; //响起一秒关闭
						BeepCnt=0;
						FindBarCodeFlag=0;//将标志位置0;
					}
				}
			}
			else if(FindTowerBFlag)
			{	
				//发现了塔B
				LineInspect.Target.Target_Y_V -= (Inspect1Speed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=0)
				{
					LineInspect.Target.Target_Y_V=0;			
				}
				static uint8_t FlowYAdjustCnt=0;
				if(ABS(RT_Info.FlowY)<0.03f)
				{
				
					//飞行器稳定后一段时间后
					FlowYAdjustCnt++;
					if(FlowYAdjustCnt>=100)
					{	
						FindTowerBFlag=0;
						FlowYAdjustCnt=0;
						LineInspect.Stage = AdjustPosB;
					}
				}
				else
				{
					FlowYAdjustCnt=0;
				
				}
			}
		}
		else
		{
			LineInspect.Target.Target_Y_V += (Inspect1Speed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=Inspect1Speed)
			{
				LineInspect.Target.Target_Y_V=Inspect1Speed;
			}
		}	
	}
	else
	{
		LineInspect.Target.Target_H+=0;
		
		if(LineInspect.Target.Target_X_V>0)
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=0)
			{
				LineInspect.Target.Target_X_V=0;
			}
		}
		else if(LineInspect.Target.Target_X_V<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=0)
			{
				LineInspect.Target.Target_X_V=0;
			}			
		}
		else
		{
			LineInspect.Target.Target_X_V = 0.0f;
		}	
		
		
		if(LineInspect.Target.Target_Y_V>0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else if(LineInspect.Target.Target_Y_V<0)
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}			
		}
		else
		{
			LineInspect.Target.Target_Y_V = 0.0f;
		}			
	}
}


//如何从巡线过程触发到这一过程
void AdjustBPosPro(void)
{
	if(NoRegTowerFlagH!=1)
	{
		//数据正确
		TowerTarX = 0;
		TowerTarDeltaX = 30;
		//对塔的期望
		int16_t TowerErroX = TowerTarX - TowerPosX;
		int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
		//计算对塔的误差值。
		if(ABS(TowerErroX)<TowerXDeathZoon)
		{
			TowerErroX=0;
		}//死区
		if(ABS(TowerErroDeltaX)<TowerDeltaXDeathZoon)
		{
			TowerErroDeltaX=0;
		}	//死区
		
		
		if(TowerErroX>0) //作用在Y轴上
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=AdjustYSpeed)
			{
				LineInspect.Target.Target_Y_V=AdjustYSpeed;
			}
		}
		else if(TowerErroX<0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=-AdjustYSpeed)
			{
				LineInspect.Target.Target_Y_V=-AdjustYSpeed;
			}
		}
		else
		{
			if(LineInspect.Target.Target_Y_V>0)
			{
				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=0)
				{
					LineInspect.Target.Target_Y_V=0;
				}
			}
			else if(LineInspect.Target.Target_Y_V<0)
			{
				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V>=0)
				{
					LineInspect.Target.Target_Y_V=0;
				}
			}
			else
			{
				LineInspect.Target.Target_Y_V = 0.0f;
			}		
		}

	
		if(TowerErroDeltaX>0)//作用在X轴上
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V = -AdjustXSpeed;		
			}
		}
		else if(TowerErroDeltaX<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V = AdjustXSpeed;		
			}		
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V = 0;		
				}			
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V = 0;		
				}	
			}
			else
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}
		}
		//对与塔B位置的控制
		static uint8_t AdjustBCnt=0;
		if((ABS(RT_Info.FlowX_V)<0.03f)&&(ABS(RT_Info.FlowY_V)<0.03f)&&(ABS(TowerErroDeltaX)<(2*TowerDeltaXDeathZoon))&&(ABS(TowerErroX)<(2*TowerXDeathZoon)))
		{
			//位置稳定之后 等待Openmv拍摄二维码
			
			AdjustBCnt++;
			//调整稳定后计数

		}
		else
		{
			AdjustBCnt=0;
		}
		if(AdjustBCnt>=100)
		{		
			if(FindQRCodeFlag)
			{
				AdjustBCnt=0;
				FindQRCodeFlag=0;
				LineInspect.Stage=TurnCorner;
				//开始转角
			}
		}		
	}
	else
	{
		//未识别到正确的位置
		if(LineInspect.Target.Target_Y_V>0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else if(LineInspect.Target.Target_Y_V<0)
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else
		{
			LineInspect.Target.Target_Y_V = 0.0f;
		}	
		
		if(LineInspect.Target.Target_X_V>0)
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=0)
			{
				LineInspect.Target.Target_X_V = 0;		
			}			
		}
		else if(LineInspect.Target.Target_X_V<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=0)
			{
				LineInspect.Target.Target_X_V = 0;		
			}	
		}
		else
		{
			LineInspect.Target.Target_X_V = 0.0f;
		}		
	}	
	
	if(NoRegLineFlagH!=1)
	{
		//识别到正确数据
		LineTarY=0;
		int16_t LineErroPosY = LineTarY - LinePosY;	
		//线A的高度期望
		if((LineErroPosY>-LinePosYDeathZoon) && (LineErroPosY<LinePosYDeathZoon))
		{
			LineErroPosY=0;
			
		}//死区	
		if(LineErroPosY>0)//作用在高度上
		{
			//说明无人机偏下 需要提高高度
			LineInspect.Target.Target_H+=AdjustHeightWeight;
		}
		else if(LineErroPosY<0)
		{
			//说明无人机偏上 需要降低高度
			LineInspect.Target.Target_H-=AdjustHeightWeight;
		}
		else
		{
			LineInspect.Target.Target_H+=0.f;
		}

	}
	else
	{
		LineInspect.Target.Target_H+=0.f;	
	}
}



void TurnCornerPro(void)
{
	if(TurnCornerFinishFlag!=1)
	{
		if(NoRegTowerFlagH!=1)
		{
			//识别到正确数据
			//转弯过程
			TowerTarX = 0;
			TowerTarDeltaX = 30;
			//对塔的期望
			int16_t TowerErroX = TowerTarX - TowerPosX;
			int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
			//计算对塔的误差值。
			if(ABS(TowerErroX)<TowerXDeathZoon)
			{
				TowerErroX=0;
			}//死区
			if(ABS(TowerErroDeltaX)<TowerDeltaXDeathZoon)
			{
				TowerErroDeltaX=0;
			}//死区
			
			if(TowerErroX>0)//作用在Y轴上
			{
				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V>=AdjustYSpeed)
				{
					LineInspect.Target.Target_Y_V=AdjustYSpeed;
				}
			}
			else if(TowerErroX<0)
			{
				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=-AdjustYSpeed)
				{
					LineInspect.Target.Target_Y_V=-AdjustYSpeed;
				}
			}
			else
			{
				if(LineInspect.Target.Target_Y_V>0)
				{
					LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_Y_V<0)
					{
						LineInspect.Target.Target_Y_V = 0.0f;
					}
				}
				else if(LineInspect.Target.Target_Y_V<0)
				{
					LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_Y_V>0)
					{
						LineInspect.Target.Target_Y_V = 0.0f;
					}
				}	
				else
				{
					LineInspect.Target.Target_Y_V = 0.0f;
				}
				
			}
			
			if(TowerErroDeltaX>0)//作用在X轴上
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
				{
					LineInspect.Target.Target_X_V = -AdjustXSpeed;			
				}
			}
			else if(TowerErroDeltaX<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
				{
					LineInspect.Target.Target_X_V = AdjustXSpeed;	
				}
			}
			else
			{
				if(LineInspect.Target.Target_X_V>0)
				{
					LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);;
					if(LineInspect.Target.Target_X_V<=0)
					{
						LineInspect.Target.Target_X_V=0;
					}
				}
				else if(LineInspect.Target.Target_X_V<0)
				{
					LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);;
					if(LineInspect.Target.Target_X_V>=0)
					{
						LineInspect.Target.Target_X_V=0;
					}		
				}
				else
				{
					LineInspect.Target.Target_X_V = 0.0f;
				}
			}
		}
		else
		{
			//识别不到正确信号
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);;
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);;
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}		
			}
			else
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}
			
			if(LineInspect.Target.Target_Y_V>0)
			{
				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<0)
				{
					LineInspect.Target.Target_Y_V = 0.0f;
				}
			}
			else if(LineInspect.Target.Target_Y_V<0)
			{
				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V>0)
				{
					LineInspect.Target.Target_Y_V = 0.0f;
				}
			}	
			else
			{
				LineInspect.Target.Target_Y_V = 0.0f;
			}			
		}
	}
	else 
	{
		//转弯完成 调整位置
		if(NoRegLineFlagH!=1)
		{
			//识别到正确数据
			LineTarY=0;
			LineTarDeltaY=20;
			int16_t LineErroPosY= LineTarY-LinePosY;
			int16_t LineErroDeltaY = LineTarDeltaY-LineDeltaY;
			if(ABS(LineErroPosY)<LinePosYDeathZoon)
			{
				LineErroPosY=0;	
			}//死区
			if(ABS(LineErroDeltaY)<LineDeltaYDeathZoon)
			{
				LineErroDeltaY=0;
			}//死区
			
			if(LineErroPosY>0)//作用在高度上
			{
				//说明无人机偏下 需要提高高度
				LineInspect.Target.Target_H+=AdjustHeightWeight;
			}
			else if(LineErroPosY<0)
			{
				//说明无人机偏上 需要降低高度
				LineInspect.Target.Target_H-=AdjustHeightWeight;
			}
			else
			{
				LineInspect.Target.Target_H+=0.f;
			}
			
			if(LineErroDeltaY>0)	//作用在光流X上
			{	
				//说明远离了线,需要往光流负方向飞行
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
				{
					LineInspect.Target.Target_X_V=-AdjustXSpeed;
				}
			}
			else if(LineErroDeltaY<0)
			{
				//说明离太近了,往光流正方向飞
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
				{
					LineInspect.Target.Target_X_V=AdjustXSpeed;
				}
			}
			else
			{
				if(LineInspect.Target.Target_X_V>0)
				{
					LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_X_V<=0)
					{
						LineInspect.Target.Target_X_V=0;
					}				
				}
				else if(LineInspect.Target.Target_X_V<0)
				{
					LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_X_V>=0)
					{
						LineInspect.Target.Target_X_V=0;
					}
				}
				else
				{
					LineInspect.Target.Target_X_V=0;
				}
			
			}
			static uint8_t StableCnt=0;
			if((LineErroPosY<(2*LinePosYDeathZoon)) && (LineErroDeltaY<(2*LineDeltaYDeathZoon)))
			{	
				//稳定计时
				
				StableCnt++;
				if(StableCnt>=40)
				{
					StableCnt=0;
					LineInspect.Stage = Inspect2;
				}				
			}
			else
			{
				StableCnt=0;
			}
		}
		else
		{
			//识别不到正确数据
			LineInspect.Target.Target_H+=0.f;
			
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}				
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else
			{
				LineInspect.Target.Target_X_V=0;
			}			
		}
	}
}



void Inspect2Pro(void)
{
	if(NoRegLineFlagH!=1)
	{
		LineTarY=0;
		LineTarDeltaY = 10;
		//对线的期望
		int16_t LineErroDeltaY = LineTarDeltaY - LineDeltaY;
		//计算线宽度误差。
		int16_t LineErroPosY = LineTarY - LinePosY;
		if(ABS(LineErroDeltaY)<LineDeltaYDeathZoon)
		{
			LineErroDeltaY=0;
		}//死区
		if(ABS(LineErroPosY)<LinePosYDeathZoon)
		{
			LineErroPosY=0;	
		}//死区
		
		if(LineErroDeltaY>0)//作用在光流X轴上
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=-AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V=-AdjustXSpeed;
			}
		}
		else if(LineErroDeltaY<0)
		{	
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V=AdjustXSpeed;
			}
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}
			}
			else 
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}		
		}	
		
		if(LineErroPosY>0)//作用在高度上
		{
			//说明无人机偏下 需要提高高度
			LineInspect.Target.Target_H+=AdjustHeightWeight;
		}
		else if(LineErroPosY<0)
		{
			//说明无人机偏上 需要降低高度
			LineInspect.Target.Target_H-=AdjustHeightWeight;
		}
		else
		{
			LineInspect.Target.Target_H+=0.f;
		}	
		
		if(FindTowerAFlag)
		{
			LineInspect.Target.Target_Y_V -= (Inspect1Speed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=0)
			{
				LineInspect.Target.Target_Y_V=0;			
			}
			static uint8_t FlowYAdjustCnt=0;
			if(ABS(RT_Info.FlowY)<0.03f)
			{
				
				//飞行器稳定后一段时间后
				FlowYAdjustCnt++;
				if(FlowYAdjustCnt>=100)
				{	
					FindTowerAFlag=0;
					FlowYAdjustCnt=0;
					LineInspect.Stage = Land;
				}
			}
			else
			{
				FlowYAdjustCnt=0;
			}
			
		}
		else
		{
			LineInspect.Target.Target_Y_V += (Inspect1Speed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=Inspect1Speed)
			{
				LineInspect.Target.Target_Y_V=Inspect1Speed;
			}
		
		}
	}
	else
	{
		//识别不到正确数据
		LineInspect.Target.Target_H+=0.f;
		
		if(LineInspect.Target.Target_X_V>0)
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V<=0)
			{
				LineInspect.Target.Target_X_V=0;
			}
		}
		else if(LineInspect.Target.Target_X_V<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V>=0)
			{
				LineInspect.Target.Target_X_V=0;
			}
		}
		else 
		{
			LineInspect.Target.Target_X_V = 0.0f;
		}			

		
		if(LineInspect.Target.Target_Y_V>0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V<=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else if(LineInspect.Target.Target_Y_V<0)
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V>=0)
			{
				LineInspect.Target.Target_Y_V=0;
			}
		}
		else 
		{
			LineInspect.Target.Target_Y_V = 0.0f;
		}			
	}
}


//如何从返程巡线触发进入到这一过程
void LandPro(void)
{
	if(NoRegTowerFlagH!=1)
	{
		//识别到正确数据
		TowerTarX = 0;
		TowerTarDeltaX = 20;
		//对塔的期望

		//对塔的控制
		int16_t TowerErroPosX = TowerTarX - TowerPosX;
		int16_t TowerErroDeltaX = TowerTarDeltaX - TowerDeltaX;
		//计算塔误差
		//此处的X是摄像头X
		if(TowerErroPosX>0)//作用在光流Y轴上。
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V >= AdjustYSpeed)
			{
				LineInspect.Target.Target_Y_V = AdjustYSpeed;
			}
		}
		else if(TowerErroPosX<0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V <= -AdjustYSpeed)
			{
				LineInspect.Target.Target_Y_V = -AdjustYSpeed;
			}			
		}
		else
		{
			if(LineInspect.Target.Target_Y_V>0)
			{
				LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V <= 0)
				{
					LineInspect.Target.Target_Y_V = 0;
				}					
			}
			else if(LineInspect.Target.Target_Y_V<0)
			{
				LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V >=0)
				{
					LineInspect.Target.Target_Y_V = 0;
				}			
			}
			else
			{
				LineInspect.Target.Target_Y_V = 0.0f;
			}
		}
		
		if(TowerErroDeltaX>0) //作用在光流X轴上.
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V <= -AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V = -AdjustXSpeed;
			}
		}
		else if(TowerErroDeltaX<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V >= AdjustXSpeed)
			{
				LineInspect.Target.Target_X_V = AdjustXSpeed;
			}
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V <= 0)
				{
					LineInspect.Target.Target_X_V = 0;
				}					
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V >=0)
				{
					LineInspect.Target.Target_X_V = 0;
				}			
			}
			else
			{
				LineInspect.Target.Target_X_V = 0.0f;
			}		
		}
		static uint8_t StableCnt=0;
		if((ABS(RT_Info.FlowX_V)<=0.3f)&&(ABS(RT_Info.FlowY_V)<=0.3f))
		{
			
			StableCnt++;
			if(StableCnt>=200)
			{
				StableCnt=0;
				OS_ERR err;
				FlightControl.landFlag=1;		
				OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err);			
				while(1)
				{
					BEEP_ON;
					delay_ms(50);
					BEEP_OFF;
					delay_ms(20);
					BEEP_ON;
					LED1_ON;
					LED2_OFF;
					LED3_OFF;
					LED4_OFF;
					delay_ms(100);
					BEEP_OFF;
					LED2_ON;
					LED1_OFF;
					LED3_OFF;
					LED4_OFF;
					delay_ms(130);
					BEEP_ON;
					LED3_ON;
					LED1_OFF;
					LED2_OFF;
					LED4_OFF;
					delay_ms(240);
					BEEP_OFF;
					LED4_ON;
					LED2_OFF;
					LED3_OFF;
					LED1_OFF;
					delay_ms(175);
				}					
			}
		
		}
		else
		{
			StableCnt=0;
		}
	}
	else
	{
		//识别不到正确数据
		if(LineInspect.Target.Target_X_V>0)
		{
			LineInspect.Target.Target_X_V -= (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V <= 0)
			{
				LineInspect.Target.Target_X_V = 0;
			}					
		}
		else if(LineInspect.Target.Target_X_V<0)
		{
			LineInspect.Target.Target_X_V += (AdjustXSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_X_V >=0)
			{
				LineInspect.Target.Target_X_V = 0;
			}			
		}
		else
		{
			LineInspect.Target.Target_X_V = 0.0f;
		}
		
		
		if(LineInspect.Target.Target_Y_V>0)
		{
			LineInspect.Target.Target_Y_V -= (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V <= 0)
			{
				LineInspect.Target.Target_Y_V = 0;
			}					
		}
		else if(LineInspect.Target.Target_Y_V<0)
		{
			LineInspect.Target.Target_Y_V += (AdjustYSpeed/(float)AdjustStep);
			if(LineInspect.Target.Target_Y_V >=0)
			{
				LineInspect.Target.Target_Y_V = 0;
			}			
		}
		else
		{
			LineInspect.Target.Target_Y_V = 0.0f;
		}		
		static uint8_t StableCnt2=0;
		if((ABS(RT_Info.FlowX_V)<=0.3f)&&(ABS(RT_Info.FlowY_V)<=0.3f))
		{
			
			StableCnt2++;
			if(StableCnt2>=200)
			{
				StableCnt2=0;
				OS_ERR err;
				FlightControl.landFlag=1;		
				OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,&err);			
				while(1)
				{
					BEEP_ON;
					delay_ms(50);
					BEEP_OFF;
					delay_ms(20);
					BEEP_ON;
					LED1_ON;
					LED2_OFF;
					LED3_OFF;
					LED4_OFF;
					delay_ms(100);
					BEEP_OFF;
					LED2_ON;
					LED1_OFF;
					LED3_OFF;
					LED4_OFF;
					delay_ms(130);
					BEEP_ON;
					LED3_ON;
					LED1_OFF;
					LED2_OFF;
					LED4_OFF;
					delay_ms(240);
					BEEP_OFF;
					LED4_ON;
					LED2_OFF;
					LED3_OFF;
					LED1_OFF;
					delay_ms(175);
				}					
			}	
		}
		else
		{
			StableCnt2=0;
		}
	}
}


