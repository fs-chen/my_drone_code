/******************** (C) COPYRIGHT 2015-2017 Xiluna Tech ************************
 * 作者 	:Xiluna Tech
 * 文件名 :Data_deal.c
 * 描述   :处理PC发来的数据
 * 官网   :http://xiluna.com/
 * 公众号 :XilunaTech
**********************************************************************************/
#include "Data_deal.h"
/* 油门设置拉力 */
volatile float throttleBasic;
int sumCheck(_Data_Rx rx)
{
	int i=0;
	unsigned char sum;
	for(i=0;i<rx.len-1;i++)
	{
		sum^=rx.buf[i];
	}
	if(sum==rx.buf[rx.len-1])
		return 1;
	else 
		return 0;
}

static u8 firstdata[512];
static u8 secdata[512];
_Data_Rx Stitchingdata;
//蓝牙数据拼接
void dataStitching(_Data_Rx rx)
{
	static u8 i,firstlen;
	static u8 BluetoothStitch[20];
	static u8 stitchingflag = 0;
	/*第二帧数据*/
	if(rx.len!=20 && stitchingflag == 1 && rx.buf[0]!=0x55 && rx.buf[1]!=0xAA)
	{
		memcpy(secdata,rx.buf,sizeof(rx.buf));
		for(i = firstlen;i<(rx.len + firstlen);i++)
		{
			BluetoothStitch[i] = secdata[i-firstlen];
		}
		Stitchingdata.len = sizeof(BluetoothStitch);
		stitchingflag = 0;
		memcpy(Stitchingdata.buf,BluetoothStitch,sizeof(BluetoothStitch));
		dataDeal(Stitchingdata);
	}
	/*第一帧数据*/
	else if(rx.len<=20 && rx.buf[0]==0x55)
	{
		memcpy(firstdata,rx.buf,sizeof(rx.buf));
		for(i = 0;i<rx.len;i++)
		{
			BluetoothStitch[i] = firstdata[i];
		}
		firstlen = 	rx.len;	
		stitchingflag = 1;
	}
	/*使用笔记本自带蓝牙不需要数据的拼接*/
	else if(rx.len==20 && rx.buf[0]==0x55 && rx.buf[1]==0xAA)
	{
		dataDeal(rx);
	}
}
uint8_t TurnFlag=0;
extern uint8_t takeoff_suce_flag;
void dataDeal(_Data_Rx rx)
{
//	float tmp=0.0f;
	u8 HexToFloat[4];
	float pidParaTemp[3];
	u8 i,j;
	/*飞行控制指令 */
	if( rx.len==20 && rx.buf[0]==0x55 && rx.buf[1]==0xAA )
	{
//		printf("recieve rc data\r\n");
//		if(rx.buf[2]==0xff && RT_Info.lowPowerFlag==0) //无人机开关 低电压模式下不接受启动信号
		if(rx.buf[2]==0xff) //无人机开关 低电压模式下不接受启动信号
		{
			if(rx.buf[3]==0){
				FlightControl.OnOff = Drone_Off;
//				printf("get off the plane\r\n");
			}
			else if(rx.buf[3]==1)
			{	
				Inner_pidinit();		//每次开启，都要清零PID以及控制数据		
				Outer_pidinit();	
				Neurons_pidinit();					
				FlightControl.OnOff = Drone_On;
				FlightControl.droneMode = Drone_Mode_4Axis;
//				printf("\r\n");
//				printf("get on the plane\r\n");
//				printf("\r\n");
			}
			else if(rx.buf[3]==2)	//降落命令		
			{
				FlightControl.landFlag=1;
//				printf("land the plane\r\n");
			}						
		}
		/*设置目标角度----在四轴起飞的时候不起作用*/
		else if(rx.buf[2]==0x01 && FlightControl.droneMode!=Drone_Mode_4Axis ) 
		{
//			printf("set the target angle");
			/*  Target_Pitch */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[3+i];
			}
			Target_Info.Pitch = Hex_To_Decimal(HexToFloat,4);
			/*  Target_Roll */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[7+i];
			}
			Target_Info.Roll = Hex_To_Decimal(HexToFloat,4);								
			/*  Target_RateYaw */					
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[11+i];
			}
			Target_Info.RateYaw = Hex_To_Decimal(HexToFloat,4); 

			/*  Target_Height ****************************/
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[15+i];
			}
		}	
		else if(rx.buf[2]==0x02) //选择实验飞行模式同时配置基础量
		{
//			printf("select the fly mode\r\n");
			switch(rx.buf[3])
			{
				case 0:
						FlightControl.droneMode=Drone_Mode_None;
					break;
				case 1:
						FlightControl.droneMode=Drone_Mode_Pitch;
						throttleBasic = 250;
					break;
				case 2:
						FlightControl.droneMode=Drone_Mode_Roll;
						throttleBasic = 250;					
					break;
				case 3:
						FlightControl.droneMode=Drone_Mode_4Axis; 
						throttleBasic = 520;
					break;
				case 4:
						FlightControl.droneMode=Drone_Mode_RatePitch; 
						throttleBasic = 250;
					break;
				case 5:
						FlightControl.droneMode=Drone_Mode_RateRoll;
						throttleBasic = 250;
				default:
					break;
			}
		}
		else if(rx.buf[2]==0x03) //读取PID
		{
//			printf("read the pid prameter\r\n");
			FlightControl.ReportSW=Report_SET;					
		}					
		else if(rx.buf[2]==0x04) //校准磁力计
		{
//						if(rx.buf[3]==1)
//						{
//							LSM303_Start_Calib();
//						}
//						else if (rx.buf[3]==2)
//						{
//							LSM303_Save_Calib();
//						}
		}					
		/*设置Pitch的PID*/	
		else if(rx.buf[2] == 0x05)
		{
//			printf("set the pitch round pid parameter\r\n");
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
			Para_Info.Pitch.Kp=pidParaTemp[0];
			Para_Info.Pitch.Ki=pidParaTemp[1];
			Para_Info.Pitch.Kd=pidParaTemp[2];
			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}				
		/*设置Roll的PID*/					
		else if(rx.buf[2] == 0x06)
		{
//			printf("set the roll round pid parameter\r\n");
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}     
			Para_Info.Roll.Kp=pidParaTemp[0];
			Para_Info.Roll.Ki=pidParaTemp[1];
			Para_Info.Roll.Kd=pidParaTemp[2];
			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/*设置Yaw的PID*/					
		else if(rx.buf[2] == 0x07)
		{
//			printf("set the yaw round pid parameter\r\n");
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
			Para_Info.Yaw.Kp=pidParaTemp[0];
			Para_Info.Yaw.Ki=pidParaTemp[1];
			Para_Info.Yaw.Kd=pidParaTemp[2];
			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/*设置Height的PID*/					
		else if(rx.buf[2] == 0x08)
		{
//			printf("set the height round pid parameter\r\n");
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
			Para_Info.Height.Kp=pidParaTemp[0];
			Para_Info.Height.Ki=pidParaTemp[1];
			Para_Info.Height.Kd=pidParaTemp[2];
			Write_config();		
			FlightControl.ReportSW=Report_SET;
		}
		/*偏置角度*/
		else if(rx.buf[2] == 0x0a)
		{
//			printf("set the angle offset\r\n");
			Errangle_Info.fixedErroPitch = RT_Info.Pitch;
			Errangle_Info.fixedErroRoll = RT_Info.Roll;
			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}	 
	 /*设置ratePitch的PID*/					
		else if(rx.buf[2] == 11)
		{
//			printf("set the ratepitch round pid parameter\r\n");
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
			Para_Info.ratePitch.Kp=pidParaTemp[0];
			Para_Info.ratePitch.Ki=pidParaTemp[1];
			Para_Info.ratePitch.Kd=pidParaTemp[2];
			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
	 /*设置rateRoll的PID*/					
		else if(rx.buf[2] == 12)
		{
//			printf("set the rateroll round pid parameter\r\n");
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
			Para_Info.rateRoll.Kp=pidParaTemp[0];
			Para_Info.rateRoll.Ki=pidParaTemp[1];
			Para_Info.rateRoll.Kd=pidParaTemp[2];
			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/*设置rateYaw的PID*/					
		else if(rx.buf[2] == 13)
		{
//			printf("set the rateyaw round pid parameter\r\n");
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
			Para_Info.rateYaw.Kp=pidParaTemp[0];
			Para_Info.rateYaw.Ki=pidParaTemp[1];
			Para_Info.rateYaw.Kd=pidParaTemp[2];
			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}
		/*设置accHeight的PID*/					
		else if(rx.buf[2] == 14)
		{
//			printf("set the accheight round pid parameter\r\n");
			for(i=0;i<3;i++)
			{
				for(j=0;j<4;j++)
				{
					HexToFloat[j]=rx.buf[3+j+i*4];
				}
				pidParaTemp[i]=Hex_To_Decimal(HexToFloat,4);
			}  
			Para_Info.accHeight.Kp=pidParaTemp[0];
			Para_Info.accHeight.Ki=pidParaTemp[1];
			Para_Info.accHeight.Kd=pidParaTemp[2];
			Write_config();	
			FlightControl.ReportSW=Report_SET;
		}	
		/*设置目标Rate*/
		else if(rx.buf[2]==15) 
		{
//			printf("set the target acce rate \r\n");
			/* Target_RatePitch */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[3+i];
			}
			Target_Info.RatePitch = Hex_To_Decimal(HexToFloat,4); 
			/* Target_RateRoll */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[7+i];
			}
			Target_Info.RateRoll = Hex_To_Decimal(HexToFloat,4);										
		}
		/*遥控信息*/
//		else if(rx.buf[2]==0x11){
////			printf("receive the control data\r\n");
//			/* contrlInfo.pV */
//			for(i=0;i<4;i++)
//			{
//				HexToFloat[i]=rx.buf[3+i];
//			}
//			Target_Info.Pitch = (Hex_To_Decimal(HexToFloat,4)/50.f); 
//			/* contrlInfo.rV */
////			printf("target pitch: %f\r\n",Target_Info.Pitch);
//			for(i=0;i<4;i++)
//			{
//				HexToFloat[i]=rx.buf[7+i];
//			}
//			Target_Info.Roll = (Hex_To_Decimal(HexToFloat,4)/50.f); 		
////			printf("target roll: %f\r\n",Target_Info.Roll);
//			/* contrlInfo.yV */
//			for(i=0;i<4;i++)
//			{
//				HexToFloat[i]=rx.buf[11+i];
//			}
//			Control_Info.yV = Hex_To_Decimal(HexToFloat,4); 
////			printf("target yawrate: %f\r\n",Control_Info.yV);
//			//试用20hz到50hz的蓝牙接收频率
//			/*contrlInfo.hV */
//			for(i=0;i<4;i++)
//			{
//				HexToFloat[i]=rx.buf[15+i];
//			}

//			Control_Info.hV = Hex_To_Decimal(HexToFloat,4); 	
////			printf("target height velocity: %f\t",Control_Info.hV);
//			if(Control_Info.hV<0.3f && Control_Info.hV>-0.3f)
//			{
//				float tmpH=Target_Info.Height+Control_Info.hV;			
//				if(tmpH<1.3f)//最高目标高度1米3
//					Target_Info.Height=tmpH;
//				else
//					Target_Info.Height=1.3;
//			}
////			printf("target height: %f\t",Target_Info.Height);	
//		}
		
		
		
		/*遥控信息*/
		else if(rx.buf[2]==0x11)
		{
			/* contrlInfo.pV */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[3+i];
			}
			Control_Info.pV = Hex_To_Decimal(HexToFloat,4); 
			/* contrlInfo.rV */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[7+i];
			}
			
			Control_Info.rV = Hex_To_Decimal(HexToFloat,4); 	
			
			
			if(Control_Info.rV>=240)
			{
				TurnFlag=1;
			}
			
			
			
			/* contrlInfo.yV */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[11+i];
			}
			Control_Info.yV = Hex_To_Decimal(HexToFloat,4); 	
			/*contrlInfo.hV */
			for(i=0;i<4;i++)
			{
				HexToFloat[i]=rx.buf[15+i];
			}
			Control_Info.hV = Hex_To_Decimal(HexToFloat,4); 	
			if(Control_Info.hV<0.3f && Control_Info.hV>-0.3f)
			{
				float tmpH=Target_Info.Height+Control_Info.hV;			
				if(tmpH<1.3f)//最高目标高度1米3
					Target_Info.Height=tmpH;
				else
					Target_Info.Height=1.3;
			}
		}
	}
}

