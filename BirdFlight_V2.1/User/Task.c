/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Task.c
 * 描述   :任务函数
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Task.h"

/*全局变量*/
DroneFlightControl FlightControl;      //飞行器状态变量
DronePIDPara Para_Info;								 //飞行器PID参数集合全局变量
DroneRTInfo RT_Info;			             //飞行器实时数据
DroneErrangle Errangle_Info;           //飞行器平地校准数据
DroneTargetInfo Target_Info;           //飞行器目标的全局变量
Controller Control_Info;               //手柄控制全局变量
Throttle Throttle_Info;                //油门全局变量
float XAdjustSpeed=0.2f,YAdjustSpeed=0.2f;
extern LineInspectObject LineInspect;
extern int16_t Xpos,Ypos;
extern uint8_t ButtonRequireFlag,NoRegFlagB;
void Cycle_task(void *p_arg)
{
	uint8_t AdjustStep=100;
	static uint8_t flag=0;
	OS_ERR err;
	p_arg = p_arg;
	LineInspect.Target.Target_Yaw_Acc=0.f;
	LineInspect.Target.Target_Y_V -=0.00f;
	LineInspect.Target.Target_X_V -=0.00f;
	while(1)
	{
		if(ButtonRequireFlag==0)
		{
			SendRequireCycle();
			ButtonRequireFlag=1;
		}
		NoRegFlagB=1;
		if(NoRegFlagB==0)
		{
			if(Xpos>0)
			{
				//说明在右边 光流要X向右走  正
				LineInspect.Target.Target_X_V += (XAdjustSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=XAdjustSpeed)
				{
					LineInspect.Target.Target_X_V=XAdjustSpeed;
				}				
			}
			else if(Xpos<0)
			{
				//说明在左边 光流要X向左走 负
				LineInspect.Target.Target_X_V -= (XAdjustSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=-XAdjustSpeed)
				{
					LineInspect.Target.Target_X_V=-XAdjustSpeed;
				}				
				
			}
			else
			{
				if(LineInspect.Target.Target_X_V>0)
				{
					LineInspect.Target.Target_X_V -= (XAdjustSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_X_V<=0)
					{
						LineInspect.Target.Target_X_V=0;
					}		
				}
				else if(LineInspect.Target.Target_X_V<0)
				{
					LineInspect.Target.Target_X_V += (XAdjustSpeed/(float)AdjustStep);
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
			
			if(Ypos>0)
			{
				//说明在下 光流要X向下走 负的
				LineInspect.Target.Target_Y_V -= (YAdjustSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=-YAdjustSpeed)
				{
					LineInspect.Target.Target_Y_V=-YAdjustSpeed;
				}			
				
			}
			else  if(Ypos<0)
			{
				//说明在上 光流要X向上走 正的
				LineInspect.Target.Target_Y_V += (YAdjustSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V>=YAdjustSpeed)
				{
					LineInspect.Target.Target_Y_V=YAdjustSpeed;
				}									
			}
			else
			{
				if(LineInspect.Target.Target_Y_V>0)
				{
					LineInspect.Target.Target_Y_V -= (YAdjustSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_Y_V<=0)
					{
						LineInspect.Target.Target_Y_V=0;
					}		
				}
				else if(LineInspect.Target.Target_Y_V<0)
				{
					LineInspect.Target.Target_Y_V += (YAdjustSpeed/(float)AdjustStep);
					if(LineInspect.Target.Target_Y_V>=0)
					{
						LineInspect.Target.Target_Y_V=0;
					}						
				
				}
				else
				{
					LineInspect.Target.Target_Y_V=0;
				}			
			}			
		}
		else
		{
			if(LineInspect.Target.Target_X_V>0)
			{
				LineInspect.Target.Target_X_V -= (XAdjustSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V<=0)
				{
					LineInspect.Target.Target_X_V=0;
				}		
			}
			else if(LineInspect.Target.Target_X_V<0)
			{
				LineInspect.Target.Target_X_V += (XAdjustSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_X_V>=0)
				{
					LineInspect.Target.Target_X_V=0;
				}						
			
			}
			else
			{
				LineInspect.Target.Target_X_V=0;
			}		

				
			if(LineInspect.Target.Target_Y_V>0)
			{
				LineInspect.Target.Target_Y_V -= (YAdjustSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V<=0)
				{
					LineInspect.Target.Target_Y_V=0;
				}		
			}
			else if(LineInspect.Target.Target_Y_V<0)
			{
				LineInspect.Target.Target_Y_V += (YAdjustSpeed/(float)AdjustStep);
				if(LineInspect.Target.Target_Y_V>=0)
				{
					LineInspect.Target.Target_Y_V=0;
				}						
			
			}
			else
			{
				LineInspect.Target.Target_Y_V=0;
			}			
		
		}
		
		
		if(Take_Off)
		{
			if(RT_Info.US100_Alt<0.1f)
			{
				ALLLED_ON;
				delay_ms(3000);
				FlightControl.OnOff = Drone_On; //起飞信号						
				FlightControl.droneMode = Drone_Mode_4Axis;
				Inner_pidinit();			//每次开启，都要清零PID以及控制数据
				Outer_pidinit();
				Neurons_pidinit();	
				flag=1;			
			}							
		
		}
		if(flag)
		{
			static uint16_t cnt=0;
			cnt++;
			if(cnt>=3000)
			{	
				cnt=3001;
				FlightControl.landFlag=1;
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
			
		
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);			
	}
}


extern uint8_t NoRegTowerFlagH;
extern int16_t TowerPosX;
extern uint8_t FindTowerBFlag;
//发现塔B的标志Bit
extern uint8_t FindBarCodeFlag;
//发现条码的标志Bit
extern uint8_t FindQRCodeFlag;
//发现二维码的标志Bit

uint8_t ButtonLineAngleTg=90;
extern uint8_t HorizonVisionErroFlag;
extern uint8_t ButtonVisionErroFlag;
//视觉模块失联错误标志位
//标志位放在Timer3_Ahrs_Inner.c定时器3中断中 

extern uint8_t ButtonLineAngleFb;
extern uint8_t NoRegFlagB;//底部未识别标志位
extern uint8_t TurnCornerFinishFlag;//转弯完成的标志Bit


extern int16_t QRCodeX,QRCodeY;
extern int16_t BarCodeX,BarCodeY;


extern uint8_t TurnFlag;

uint8_t step=1;
uint8_t landflag=0;
void LineInspect_task(void *p_arg)
{
	
	OS_ERR err;
	p_arg = p_arg;
	
	while(1)
	{
//		if(HorizonVisionErroFlag==0)
		{
			if(step == 1)//起飞 
			{
				if(Take_Off)
				{
					if(RT_Info.US100_Alt<0.1f)
					{
						ALLLED_ON;
						delay_ms(3000);
						FlightControl.OnOff = Drone_On; //起飞信号						
						FlightControl.droneMode = Drone_Mode_4Axis;
						Inner_pidinit();			//每次开启，都要清零PID以及控制数据
						Outer_pidinit();
						Neurons_pidinit();					
					}				
				}
				static uint8_t HeightCnt=0;
				LineInspect.Target.Target_Yaw_Acc=0;
				if(RT_Info.US100_Alt>0.80f)
				{

					HeightCnt++;
					if(HeightCnt>=100)
						step=2;
				}
				else 
				{
					HeightCnt=0;	
					LED1_ON;
					LED2_OFF;
					LED3_OFF;
					LED4_OFF;					
				}
			}
			else if(step == 2) //巡线
			{
				LineInspect.Target.Target_Yaw_Acc=0;
				LineInspect.Target.Target_Y_V=0.20f;
				LED1_OFF;
				LED2_ON;
				LED3_OFF;
				LED4_OFF;	
				static uint32_t BCnt=0,RstBarCnt=800;
				BCnt++;
				//等触发1 条码

				if(FindBarCodeFlag)
				{					
					LineInspect.Target.Target_X_V=0.0f;
					LineInspect.Target.Target_Y_V=0.0f;
					RstBarCnt=0;
				}	
				else 
				{
					LineInspect.Target.Target_Y_V=0.20f;
					LineInspect.Target.Target_X_V = 0.00f;
					static uint8_t i=0;
					i++;
					if(i>15)
					{
						if(!RequireFlag)
						{	
							
							if(RstBarCnt>800) //识别到条码后要4s才会开始重新识别
							{			
								RstBarCnt=1001;
								i=0;
								SendRequire(BarCode);
							}
							else
							{
								i=16;
								RstBarCnt++;
							}
				
						}			
						else 
						{
							i=16;
						}
					}				
				}
				

				if(BCnt>=1000) //开始巡线后要15s才会可以识别二维码
				{
					//等触发2 二维码
					if(FindQRCodeFlag)
					{		
						LineInspect.Target.Target_X_V=0.0f;
						LineInspect.Target.Target_Y_V=0.0f;									
					}
					else 
					{
						static uint8_t j=0;
						j++;
						if(j>40)
						{
							
							if(!RequireFlag)
							{	
								j=0;
								SendRequire(QRCode);
							}		
							else
							{
								j=41;
							}							
						}												
						LineInspect.Target.Target_X_V=0.f;
//						LineInspect.Target.Target_Y_V=0.10f;//巡线速度
						if(TurnFlag)
							step=3;
					}
				}
				if(BCnt>=16000)//巡线后80s还未转弯即降落
				{
					step=5;
				}
			}
			else if(step == 3)//转头
			{
				LED1_OFF;
				LED2_OFF;
				LED3_ON;
				LED4_OFF;	
				LineInspect.Target.Target_Y_V=0.2f;
				LineInspect.Target.Target_X_V = 0.00f;
				static float YawS=0.f,YawE=0.f;
				static float YawSum=0.f;
				static uint8_t startflag=1;
				static uint32_t turnCnt=0;
				turnCnt++;
				if(turnCnt>=4000) //转弯20s还未成功即降落
				{
					step=5;
				}
				if(startflag)
				{
					YawE=0;
					YawS=0;
					startflag=0;
				}
				else 
					YawE=RT_Info.Yaw;							
				if(YawSum>182)
				{
					YawSum += (YawS - YawE);
					LineInspect.Target.Target_Yaw_Acc=-2000;
				
					YawS = RT_Info.Yaw;
				}
				else if(YawSum<178)
				{
					YawSum += (YawS - YawE);
					LineInspect.Target.Target_Yaw_Acc=2000;
					
					YawS = RT_Info.Yaw;
				}
				else
				{				
					LineInspect.Target.Target_Yaw_Acc=0;
					step=4;
					TowerPosX=0;
				}		
			}
			else if(step ==4)//回家
			{
				LED1_OFF;
				LED2_OFF;
				LED3_OFF;
				LED4_ON;	
				LineInspect.Target.Target_Yaw_Acc=0;				
				LineInspect.Target.Target_X_V = 0.00f;
				static uint16_t backCnt=0;
				static uint8_t z=0;
				backCnt++;
				z++;
				if(backCnt>=1000) //回家开始5s后才可是识别杆 并降落
				{
					LineInspect.Target.Target_Y_V=0.20f;
					if(z>6)
					{
						if(!RequireFlag)
						{
							z=0;
							SendRequire(Tower);
						}
						else
						{
							z=7;					
						}
					}					
					if(landflag)
					{
						step=5;							
					}
				}
				else
				{
					LineInspect.Target.Target_Y_V=0.25f; //一开始速度较大
				}
				if(backCnt>=4000) //回家最多20s
				{
					step=5;
				}
			}
			else if(step==5)
			{
				LineInspect.Target.Target_Yaw_Acc=0.f;
				LineInspect.Target.Target_Y_V -=0.0010f;
				if(LineInspect.Target.Target_Y_V<=0)
				{
					LineInspect.Target.Target_Y_V=0;
					FlightControl.landFlag=1;
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
			}
		}
//		else
		{
		
		
		}
//		if(ButtonVisionErroFlag)//底部摄像头失联
//		{
//			FlightControl.landFlag=1;
//			LineInspect.Target.Target_H=0.f;
//			LineInspect.Target.Target_X_V=0.f;
//			LineInspect.Target.Target_Y_V==0.f;	
//			LineInspect.Target.Target_Yaw_Acc = 0;	
////			BEEP_ON;
//		}
//		else
//		{
////			BEEP_OFF;
//			if(NoRegFlagB)//数据不可信的处理
//			{	
//				LineInspect.Target.Target_Yaw_Acc = 0;					
//			}
//			else	//数据可信的处理
//			{
//				if(LineInspect.Stage == TurnCorner)
//				{
//					static uint8_t TurnStep=1;
//					if(TurnStep==1)
//					{
//						//第一步--转弯至150°左右
//						if(ButtonLineAngleFb>60)
//						{
//							LineInspect.Target.Target_Yaw_Acc = 1000;		
//						}
//						else
//						{
//							TurnStep=2;
//						}
//						
//					}
//					else if(TurnStep==2)
//					{
//						ButtonLineAngleTg=90;
//						if(ABS(ButtonLineAngleFb-ButtonLineAngleTg)<2)
//						{
//							TurnCornerFinishFlag=1;
//							ButtonLineAngleFb=ButtonLineAngleTg;
//						}//死区
//						if(ButtonLineAngleFb>ButtonLineAngleTg)
//						{
//							LineInspect.Target.Target_Yaw_Acc = -1000;		
//						}
//						else if(ButtonLineAngleFb<ButtonLineAngleTg)
//						{
//							LineInspect.Target.Target_Yaw_Acc = 1000;		
//						}
//						else
//						{						
//							LineInspect.Target.Target_Yaw_Acc = 0;	
//						}		
//					}
//				}
//				else if((LineInspect.Stage == AdjustPosA) || (LineInspect.Stage == Inspect1) || (LineInspect.Stage == AdjustPosB) || (LineInspect.Stage == Inspect2))
//				{
//					//巡线模式下的处理					
//					ButtonLineAngleTg=90;
//					if(ABS(ButtonLineAngleFb-ButtonLineAngleTg)<2)
//					{
//						ButtonLineAngleFb=ButtonLineAngleTg;
//					}//死区
//					if(ButtonLineAngleFb>ButtonLineAngleTg)
//					{
//						LineInspect.Target.Target_Yaw_Acc = -1000;		
//					}
//					else if(ButtonLineAngleFb<ButtonLineAngleTg)
//					{
//						LineInspect.Target.Target_Yaw_Acc = 1000;		
//					}
//					else
//					{
//						LineInspect.Target.Target_Yaw_Acc = 0;	
//					}					
//				}
//				else if((LineInspect.Stage == Takeoff) || (LineInspect.Stage == Land))
//				{
//					LineInspect.Target.Target_Yaw_Acc = 0;	
//				
//				}
//				else
//				{
//					LineInspect.Target.Target_Yaw_Acc = 0;	
//				}
//			}
//		}



//		if(HorizonVisionErroFlag)//水平摄像头失联
//		{
//			FlightControl.landFlag=1;
//			LineInspect.Target.Target_H=0.f;
//			LineInspect.Target.Target_X_V=0.f;
//			LineInspect.Target.Target_Y_V==0.f;	
////			LineInspect.Target.Target_Yaw_Acc = 0;	
//			
//		}
//		else
//		{			
//			switch(LineInspect.Stage) //
//			{
//				case Takeoff:			//起飞
//				{
//					TakeOffPro();
//					break;
//				}				
//				case AdjustPosA://校准塔A
//				{
//					AdjustAPosPro();
//					break;
//				}
//				case Inspect1:			//寻找异常巡线
//				{
//					Inspect1Pro();
//					break;
//				}
//				case AdjustPosB://校准塔B
//				{
//					AdjustBPosPro();
//					break;
//				
//				}
//				case TurnCorner://转弯
//				{
//					TurnCornerPro();
//					break;
//				}
//				case Inspect2:		//返程巡线
//				{
//					Inspect2Pro();
//					break;
//				}
//				case Land:		//着陆降落
//				{
//					LandPro();
//					break;
//				}
//				default:
//				{
//					break;
//				}
//			}	
//		}
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);			
	}
}


/**
 * @Description IMU任务 500HZ
 */
OS_SEM IMU_proc;
void IMU_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	OSSemCreate ((OS_SEM*)&IMU_proc,
								(CPU_CHAR*)"IMU_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1)
	{
		OSSemPend (&IMU_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		/*获取IMU解算后角度*/
		IMU_getInfo();

	}
}

/*
 * @Description 姿态内环控制 400HZ
 */
extern void imu_set_D1_pulse_width(uint16_t period);
extern void imu_set_D2_pulse_width(uint16_t period);
OS_SEM AttitudeInner_proc;
uint8_t StartFly = 0;
void AttitudeInner_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	OSSemCreate ((OS_SEM*)&AttitudeInner_proc,
								(CPU_CHAR*)"AttitudeInner_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	static unsigned int fly_Pretime = 0;
	while(1)
	{
		OSSemPend (&AttitudeInner_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		if(FlightControl.OnOff==Drone_On)
		{
			/*预飞程序*/
			if(fly_Pretime<800)
			{
				fly_Pretime=fly_Pretime + 1;
				PID_OUT(300,300,300,300);
			}
			else
			{
				StartFly = 1;//外环高度开始计算 内环位置控制计算开始
				AttitudeInner_control();
				/*保护程序*/
				if(RT_Info.Pitch >= 25 ||  RT_Info.Pitch <= -25 || 
							RT_Info.Roll >= 25 ||  RT_Info.Roll <= -25)
				{

							PID_OUT(0,0,0,0);
							while(1);
				}
			}		
		}
		else
		{
			/* 参数归零 */
			fly_Pretime = 0;
			StartFly = 0;
			PID_OUT(0,0,0,0);
			Inner_pidinit();
			Outer_pidinit();

			Neurons_pidinit();
		}
	}
}

/**
 * @Description 位置内环控制 250HZ
 */
void PostionInner_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		if(StartFly == 1) 
		{
			//只要预起飞完成后，才回开始进行内环计算。
			/*内环位置控制计算*/
			PostionInner_control();
		}
		OSTimeDlyHMSM(0,0,0,4,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description 平台任务函数
 */
//OS_SEM Platform_Semp;
//extern void imu_task(void);
//extern void Servo_init(void);
//extern void platform_control(int16_t target_angle_roll,int16_t target_angle_pitch);
//extern void imu_keep_setting(void);
//extern uint16_t plat_cnt;
//extern uint8_t plat_break_flag;
//uint8_t plat_update_flag=0;
//void Platform_task(void *p_arg)
//{
//	OS_ERR err;
//	p_arg = p_arg;
////	OSSemCreate ((OS_SEM*)&Platform_Semp,
////							(CPU_CHAR*)"platform semphore",
////							(OS_SEM_CTR)1,
////							(OS_ERR*)&err);
////	Servo_init();
//	while(1)
//	{
////		OSSemPend(&Platform_Semp,0,OS_OPT_PEND_BLOCKING,0,&err);
//		if(plat_update_flag){
//			imu_task();		
//			platform_control(0,0);
//			plat_update_flag=0;
//			plat_cnt=0;
//			plat_break_flag=0;
//		}
//		if(plat_break_flag==1){
//			imu_set_D2_pulse_width(1500);
//			imu_set_D1_pulse_width(1500);
////			printf("platform disconnect\r\n");
//			DMA_ClearFlag(DMA1_Stream6,DMA_IT_TCIF6);  
//			DMA_Cmd(DMA1_Stream6,DISABLE);   	
//		}
//		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_HMSM_STRICT,&err);
//	}		
//}



/**
 * @Description 光流模组数据接收 100Hz
 */
void SpitoOptical_task(void *p_arg)
{
	opticalFlowTask(p_arg);
	
}

/**
 * @Description 融合任务 200HZ
 */
volatile float US100_Altinfo;
void Combine_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		//位置融合重写
		/*黑点位置融合*/
//		Altitude_Ultrasonic(RT_Info.LASER_Alt/100.f,Accel_Src);
//		OpticalFlow_Estimation(RT_Info.LASER_Alt/100.f,opFlow.deltaVel[X]/100.f,opFlow.deltaVel[Y]/100.f,Accel_Src,Acc_Flow_x,Acc_Flow_y);
//		SendHeightToVision();
		h_Estimation(US100_Altinfo,Accel_Src);
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description 姿态外环任务 200HZ
 */
void AttitudeOuter_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
//	static uint16_t i=0,j=0;
	while(1)
	{
//		if(j==0){
//			i++;
//			
//		}else i--;
//		if(i==200)j=1;
//		if(i==0)j=0;
//		if(i%3==0)imu_set_D1_pulse_width(1400+i);
		AttitudeOuter_control();
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
/**
 * @Description 位置外环任务 125HZ
 */
void PostionOuter_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		/*外环高度开始计算*/
		if(StartFly == 1)
		{
			PostionOuter_control();
		}
		OSTimeDlyHMSM(0,0,0,8,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 视觉数据接收  button
 */
OS_SEM Vision_proc;
void Vision_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	static _Data_Rx Vision_data;
	//创建二进制文件
	OSSemCreate ((OS_SEM*)&Vision_proc,
								(CPU_CHAR*)"Vision_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1)
	{
		//视觉接收重写
		OSSemPend (&Vision_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		memcpy(Vision_data.buf,Vision.buf,sizeof(Vision.buf));
		Vision_data.len = Vision.len;
		VisionDataDealButton(Vision_data);


	}
}

/**
 * @Description 视觉数据接收 horizon
 */
OS_SEM Vision_proc_Horizon;
void Uart5toOpenmvHorizon_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	static _Data_Rx Vision_data;
	//创建二进制文件
	OSSemCreate ((OS_SEM*)&Vision_proc_Horizon,
								(CPU_CHAR*)"Vision_proc_Horizon",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);	
	while(1)
	{
		OSSemPend (&Vision_proc_Horizon,0,OS_OPT_PEND_BLOCKING,0,&err);
		memcpy(Vision_data.buf,Vision_Horizon.buf,sizeof(Vision_Horizon.buf));
		Vision_data.len = Vision_Horizon.len;
		VisionDataDealHorizon(Vision_data);
	}

}



/**
 * @Description PC数据接收
 */
bool USE_BLUETOOTH = true; 
OS_SEM DataDeal_proc;//信号量
void DataDeal_task(void *p_arg)
{
	static _Data_Rx PC_data;
	OS_ERR err;
	p_arg = p_arg;
	//创建信号量
	OSSemCreate ((OS_SEM*)&DataDeal_proc,
								(CPU_CHAR*)"DataDeal_proc",
									(OS_SEM_CTR)1,
										(OS_ERR*)&err);
	while(1)
	{
		
		//蓝牙数据协议重写
		OSSemPend (&DataDeal_proc,0,OS_OPT_PEND_BLOCKING,0,&err);
		if(USE_BLUETOOTH)
		{
			memcpy(PC_data.buf,Bluetooth_rx.buf,sizeof(Bluetooth_rx.buf));
			PC_data.len = Bluetooth_rx.len;
			dataStitching(PC_data);//蓝牙数据拼接
		}
		else
		{
			memcpy(PC_data.buf,PC_rx.buf,sizeof(PC_rx.buf));
			PC_data.len = PC_rx.len;
			dataDeal(PC_data);//数据处理
		}
	}
}

/**
 * @Description Usart1toPC任务 40HZ
 */
void Usart1toPC_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		//不需要用到
		/*上传PID数据*/
		if(FlightControl.ReportSW==Report_SET)
		{
			sendParaInfo();
			FlightControl.ReportSW=Report_RESET;
		}
		/*上传实时数据*/
		sendRTInfo();
		OSTimeDlyHMSM(0,0,0,25,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description IICtoLaser任务 20HZ
 */
void IICtoLaser_task(void *p_arg)
{
	OS_ERR	err;
	p_arg = p_arg;	
	uint32_t xLastWakeTime = OS_TS_GET();
	
	if(vl53lxxId == 0) {	// 低音长鸣
		while(1) {
			BEEP_ON;
			OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
			BEEP_OFF;
			OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err);
		}
	}
	vl53l1xSetParam();	// 设置vl53l1x 参数
	
	while(1) 
	{
		if(reInitvl53l1x == true) {
			reInitvl53l1x = false;			
			vl53l1xSetParam();	// 设置vl53l1x 参数
			xLastWakeTime = OS_TS_GET();
		} else {	
			vl53l1xMeasure();
		}
//		printf("heigth laser %f:\r\n",RT_Info.LASER_Alt);
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}


/**
 * @Description Uart5toUltra任务 20HZ
 */
void Uart5toUltra_task(void *p_arg)
{
	//可能要改新的超声波
	OS_ERR err;
	p_arg = p_arg;
	static u8 Height_num = 7;
//	static u8 stopflag = 0;
	static int HeightData[10];
	while(1)
	{
		/* 串口接收超声波数据 */
		ReceiveUltraData();
		US100_Altinfo = Median_filter(ReceiveHeight,Height_num,HeightData)/1000.f;
//		printf("heigth ultra %f:\r\n",US100_Altinfo);
		/* 15s自动降落程序 */
//		if(US100_Altinfo > 0.35f)
//		{
//			if(stopflag == 0)
//			{
//				SendStopflag();
//				stopflag = 1;
//			}
//		}
//		else if(US100_Altinfo <= 0.35f)
//		{
//			stopflag = 0;
//		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 电池电压任务 10HZ
 */
void Battery_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		/*获取滤波后的电压*/
		RT_Info.batteryVoltage = Average_battery(Get_battery());
		/* 起飞电压必须高于11.45V 才可以起飞*/
		if(RT_Info.batteryVoltage<10.8f && (FlightControl.OnOff != Drone_On))
		{
			RT_Info.lowPowerFlag = 1;
//			ALLLED_ON;
			OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
//			ALLLED_OFF;
			OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		}
		else 
		{
			/*飞行中如果电压低于10.60V则自动降落*/
//			if(RT_Info.batteryVoltage < 10.60f)
//			{
//				FlightControl.landFlag=1;
//			}
			RT_Info.lowPowerFlag=0;
//			ALLLED_OFF;
		}
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description Usart mode 切换任务函数 5HZ
 */
void Usartmode_task(void *p_arg)
{
	//不需要，默认蓝牙串口
	OS_ERR err;
	p_arg = p_arg;
//	u8 Switchflag = 0;
	while(1)
	{
		/*板子K3按键*/
//		if(Usart_mode)
//		{
//			USE_BLUETOOTH = !USE_BLUETOOTH;
//		}
//		/*开启蓝牙串口关闭板载串口*/
//		if((USE_BLUETOOTH) && (Switchflag == 0))
//		{
//			BluetoothLED_ON;
//			USART_Cmd(USART3,ENABLE);
//			USART_Cmd(USART1,DISABLE);
//			Switchflag = 1;
//		}
//		else if((!USE_BLUETOOTH) && (Switchflag == 1))
//		{
//			BluetoothLED_OFF;
//			USART_Cmd(USART1,ENABLE);
//			USART_Cmd(USART3,DISABLE);
//			Switchflag = 0;
//		}
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/**
 * @Description 一键起飞切换任务函数 2.5HZ   默认不适用，需要使用打开线程
*/
void TakeOff_task(void *p_arg)
{
	//改
	OS_ERR err;
	p_arg = p_arg;
	static int TakeOff_flag = 0;
	static int TakeOff_cnt = 0;
	while(1)
	{
		if(Take_Off)
		{
			TakeOff_flag = 1;
		}
		if(TakeOff_flag == 1 && RT_Info.lowPowerFlag == 0 && US100_Altinfo<0.1f)
		{
			TakeOff_cnt++;
			if(TakeOff_cnt % 2 == 0)
			{
//				ALLLED_ON;
			}
			else
			{
//				ALLLED_OFF;
			}
			if(TakeOff_cnt == 10)
			{
				FlightControl.OnOff = Drone_On;
				FlightControl.droneMode = Drone_Mode_4Axis;
				throttleBasic = 520;	//起飞基础量550
				Inner_pidinit();			//每次开启，都要清零PID以及控制数据
				Outer_pidinit();
//				printf("takeoff init\r\n");
				Neurons_pidinit();
				TakeOff_flag = 0;
				TakeOff_cnt = 0;
			}
		}
		OSTimeDlyHMSM(0,0,0,400,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

